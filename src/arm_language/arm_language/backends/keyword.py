"""
Offline keyword backend — no network, no GPU, no credentials.

This exists for two concrete jobs, neither of which is "be a good parser":

* **Remote contributors.** Architecture D7 says a member with no lab access
  should be able to exercise the whole pipeline. That promise dies at the first
  stage if PARSE needs an API key, so there has to be *a* backend that runs
  anywhere.
* **Hermetic CI.** The orchestrator's dry-run (task 4.1) traverses every stage
  against mocks on every PR. A stage that needs the network is a stage that
  makes CI flaky.

What it is not
--------------
It is **not** a fallback for the robot to silently drop to, and it must not be
used to claim task 3.1's acceptance bar. It pattern-matches a fixed set of
English imperatives; it has no idea what a sentence means. "grab me a coffee
while you're up" parses as a pick of "coffee". The corpus marks the entries
that need real language understanding as ``llm_only`` precisely so this
backend's score is never mistaken for the parser's.

Its saving grace is that it is held to the *same* validator and guardrails as
the cloud backend — so when it is wrong, it is wrong in ways the layers above
can still catch, and the pipeline's safety properties do not depend on which
backend is loaded.
"""
import json
import re

from .. import schema

# Leading politeness and framing we can drop without changing the request.
_PREAMBLE = r'(?:please\s+)?(?:can|could|would|will)?\s*(?:you\s+)?(?:please\s+)?'

# Imperatives that mean "move an object". `put`/`place`/`move` usually carry a
# destination too, which the place-clause patterns below pick up.
_VERB = (r'(?:pick\s+up|pick|grab|get|fetch|retrieve|take|lift|'
         r'bring(?:\s+me)?|hand(?:\s+me)?|give\s+me|put|place|move)')

_COMMAND_RE = re.compile(rf'^{_PREAMBLE}{_VERB}\s+(?P<rest>.+)$')

# "... and put it in the bin", "... and place them on the shelf". Matched
# first because it also strips the dangling "and put it" from the target.
_PLACE_CLAUSE_RE = re.compile(
    r'\s*(?:,\s*)?(?:and\s+|then\s+)*'
    r'(?:put|place|drop|set|leave|stick)\s+(?:it|them|that|those)?\s*'
    r'(?:in|into|inside|on|onto|under|next\s+to|beside)\s+(?P<dest>.+)$'
)

# Bare "... in the bin" with no second verb.
_PLACE_PREP_RE = re.compile(
    r'\s+(?:in|into|inside|onto|on)\s+(?P<dest>.+)$'
)

# Questions and chat. Checked before the verb patterns so "can you tell me
# where the hammer is" does not match on a stray verb.
_QUESTION_RE = re.compile(
    r'^\s*(?:what|who|when|where|why|how|which|is|are|do|does|did|tell)\b'
)

# Words that name no object. Their presence is the main signal this backend
# has for "the speaker was vague", and it maps onto a low confidence.
_VAGUE = frozenset({'it', 'that', 'this', 'them', 'those', 'these', 'thing',
                    'things', 'something', 'anything', 'stuff', 'one'})

# Modifier vocabulary. Small and literal — widening it is not the way to make
# this backend better; using the cloud backend is.
_MODIFIER_WORDS = {
    'color': frozenset({'red', 'green', 'blue', 'yellow', 'orange', 'purple',
                        'black', 'white', 'grey', 'gray', 'brown', 'pink'}),
    'size': frozenset({'small', 'little', 'tiny', 'big', 'large', 'long',
                       'short', 'wide', 'narrow'}),
    'material': frozenset({'wooden', 'wood', 'metal', 'metallic', 'plastic',
                           'rubber', 'steel', 'aluminium', 'aluminum'}),
    'position': frozenset({'left', 'right', 'near', 'far', 'front', 'back',
                           'nearest', 'closest', 'furthest'}),
}

_CONFIDENCE_NAMED = 0.85
"""A verb and a concrete noun. Below the 0.75 accept threshold? No — above it,
deliberately: these are the cases this backend genuinely handles."""

_CONFIDENCE_VAGUE = 0.20
"""Pronoun with no antecedent. Low enough to land in the refuse band."""


def _reject(reason: str) -> str:
    """Return a schema-conforming rejection."""
    return _payload(schema.ACTION_REJECT, '', None, {}, 0.9, reason)


def _payload(action, target, place, modifiers, confidence, reason):
    """Serialise one response, filling every key the strict schema requires."""
    return json.dumps({
        'action': action,
        'target_query': target,
        'place_target': place,
        'modifiers': {key: modifiers.get(key) for key in schema.MODIFIER_KEYS},
        'confidence': confidence,
        'reason': reason,
    })


def _extract_modifiers(phrase: str) -> dict:
    """
    Pull known adjectives out of a phrase.

    The adjectives stay in ``target_query`` as well — the detector benefits
    from "red screwdriver" over "screwdriver", while the structured copy is
    what later phases re-rank on.
    """
    found = {}
    for word in phrase.split():
        for key, vocabulary in _MODIFIER_WORDS.items():
            if word in vocabulary and key not in found:
                found[key] = word
    return found


def _trim_target(phrase: str) -> str:
    """Tidy a target phrase without changing which object it names."""
    phrase = re.sub(r'\s*\b(?:and|then)\b\s*$', '', phrase.strip())
    phrase = phrase.strip(' \t,.!?')
    words = phrase.split()
    while words and words[0] in schema.ARTICLES:
        words = words[1:]
    # An over-long phrase is left as-is on purpose: the validator's
    # noun-phrase check should see it and refuse, rather than this backend
    # quietly truncating a bad parse into a plausible-looking one.
    return ' '.join(words)


class KeywordBackend:
    """Rule-based English imperative matcher. Deterministic, offline."""

    name = 'keyword'

    def complete(self, utterance: str) -> str:
        """Return schema-conforming JSON for ``utterance``."""
        text = re.sub(r'\s+', ' ', utterance).strip().lower()

        if not text:
            return _reject('Empty request.')

        if text.endswith('?') or _QUESTION_RE.match(text):
            return _reject('That is a question, not a request to move an '
                           'object.')

        match = _COMMAND_RE.match(text.rstrip('.!'))
        if not match:
            return _reject('No recognised instruction to pick something up.')

        rest = match.group('rest').strip()

        place = None
        place_match = _PLACE_CLAUSE_RE.search(rest) or _PLACE_PREP_RE.search(rest)
        if place_match:
            place = _trim_target(place_match.group('dest'))
            rest = rest[:place_match.start()]

        target = _trim_target(rest)
        if not target:
            return _reject('No object named in the request.')

        # A pronoun target is the one ambiguity this backend reliably detects,
        # so it reports it honestly rather than guessing an object.
        vague = all(word in _VAGUE for word in target.split())
        confidence = _CONFIDENCE_VAGUE if vague else _CONFIDENCE_NAMED

        action = (schema.ACTION_PICK_AND_PLACE if place
                  else schema.ACTION_PICK)
        return _payload(action, target, place, _extract_modifiers(target),
                        confidence, None)

    def describe(self) -> str:
        """Return a one-line description for startup logs."""
        return f'{self.name} (offline rule-based; not for acceptance runs)'
