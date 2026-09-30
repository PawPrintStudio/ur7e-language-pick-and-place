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

# --- jog grammar (task 3.2) --------------------------------------------------
# "go up a bit", "could you go down by 2?", "move left 3 cm", "raise it a
# little", "spin slowly", "rotate at speed -1", "go home", "go to the start
# pose but 3 cm up". Each pattern anchors on a direction/verb word right after
# the preamble, so "move the hammer" still falls through to the pick grammar.
_DIRECTION_WORDS = {
    'up': 'up', 'upward': 'up', 'upwards': 'up', 'higher': 'up',
    'down': 'down', 'downward': 'down', 'downwards': 'down', 'lower': 'down',
    'left': 'left', 'right': 'right',
    'forward': 'forward', 'forwards': 'forward', 'ahead': 'forward',
    'back': 'backward', 'backward': 'backward', 'backwards': 'backward',
}
_DIRECTION_RE = '|'.join(sorted(_DIRECTION_WORDS, key=len, reverse=True))

# Verbs that imply a direction on their own.
_VERB_DIRECTION = {'raise': 'up', 'lift': 'up', 'lower': 'down', 'drop': 'down'}

_A_BIT = r'(?:a\s+(?:bit|little|tad|touch|smidge)(?:\s+bit)?|slightly|a\s+little\s+bit)'
_AMOUNT = (r'(?:(?:by\s+)?(?P<n>\d+(?:\.\d+)?)\s*'
           r'(?:cm|centimet(?:er|re)s?|centimeters?)?)')
_UNITS_TAIL = r'(?:\s*(?:cm|centimet(?:er|re)s?))?'

_MOVE_RE = re.compile(
    rf'^{_PREAMBLE}(?:(?:go|move|come|shift|step|travel|head|nudge)\s+'
    rf'(?:it\s+|the\s+(?:arm|tool|robot|gripper|hand)\s+)?)?'
    rf'(?P<dir>{_DIRECTION_RE})'
    rf'(?:\s+(?:(?P<bit>{_A_BIT})|{_AMOUNT}))?\s*$'
)
_VERB_MOVE_RE = re.compile(
    rf'^{_PREAMBLE}(?P<verb>raise|lift|lower|drop)'
    rf'(?:\s+(?:it|the\s+(?:arm|tool|robot|gripper|hand)))?'
    rf'(?:\s+(?:(?P<bit>{_A_BIT})|{_AMOUNT}))?\s*$'
)

_SPEED_WORDS = {
    'slowly': 1, 'slow': 1, 'gently': 1, 'a bit': 1, 'a little': 1,
    'normally': 2, 'medium': 2,
    'fast': 3, 'quickly': 3, 'quick': 3, 'rapidly': 3,
}
_ROTATE_RE = re.compile(
    rf'^{_PREAMBLE}(?:spin|rotate|turn|twist)'
    r'(?:\s+(?:it|the\s+(?:wrist|tool|gripper|hand|arm)))?(?:\s+around)?'
    r'(?P<rest>.*)$'
)
_ROTATE_SPEED_RE = re.compile(r'(?:at\s+)?speed\s+(?P<lvl>[+-]?\s*\d)')
# Longest alternative first: "deg" would otherwise match the front of
# "degrees" and leave "rees" behind as unexplained text.
_ROTATE_ANGLE_RE = re.compile(r'(?:by\s+)?(?P<deg>\d+(?:\.\d+)?)\s*(?:degrees?|deg|°)')
_ROTATE_CW_RE = re.compile(r'\b(?:clockwise|cw|to\s+the\s+right)\b')
_ROTATE_CCW_RE = re.compile(
    r'\b(?:counter-?\s?clockwise|anti-?\s?clockwise|ccw|to\s+the\s+left)\b')

# A pose request needs "to" after the verb ("go to the start pose", "return
# to home") or the bare word "home" ("go home"). Without that anchor, "move
# the blue block onto the tray" would parse as a pose called "the blue block".
_GO_TO_RE = re.compile(
    rf'^{_PREAMBLE}(?:go|move|come|return|head|drive)(?:\s+back)?'
    r'(?:\s+to\s+(?:the\s+)?|\s+(?=home\b))(?P<pose>[a-z][a-z0-9 \-]*?)'
    r'(?:\s+(?:pose|position|posture|spot))?'
    r'(?:\s*(?:,|but|and(?:\s+then)?|then|plus)?\s+'
    rf'(?:(?:go|move)\s+)?(?:(?P<pre_n>\d+(?:\.\d+)?){_UNITS_TAIL}\s+)?'
    rf'(?P<dir>{_DIRECTION_RE})(?:\s+(?:(?P<bit>{_A_BIT})|{_AMOUNT}))?)?\s*$'
)

_CONFIDENCE_JOG = 0.85


def _jog_payload(action, motion, confidence=_CONFIDENCE_JOG):
    """Serialise a jog response with a full `motion` object."""
    full = {key: motion.get(key) for key in schema.MOTION_KEYS}
    return json.dumps({
        'action': action,
        'target_query': '',
        'place_target': None,
        'modifiers': {key: None for key in schema.MODIFIER_KEYS},
        'motion': full,
        'confidence': confidence,
        'reason': None,
    })


def _amount(match) -> object:
    """Distance from a move match: explicit number, 'a bit', or None."""
    if match.group('bit'):
        return schema.SMALL_MOVE_CM
    if match.groupdict().get('n'):
        return float(match.group('n'))
    if match.groupdict().get('pre_n'):
        return float(match.group('pre_n'))
    return None


def _try_jog(text: str):
    """Return jog JSON for ``text``, or None if it is not a jog request."""
    match = _MOVE_RE.match(text)
    if match:
        return _jog_payload(schema.ACTION_MOVE, {
            'direction': _DIRECTION_WORDS[match.group('dir')],
            'distance_cm': _amount(match),
        })

    match = _VERB_MOVE_RE.match(text)
    if match:
        return _jog_payload(schema.ACTION_MOVE, {
            'direction': _VERB_DIRECTION[match.group('verb')],
            'distance_cm': _amount(match),
        })

    match = _ROTATE_RE.match(text)
    if match:
        rest = match.group('rest')
        level = None
        speed = _ROTATE_SPEED_RE.search(rest)
        if speed:
            level = int(speed.group('lvl').replace(' ', ''))
            rest = rest.replace(speed.group(0), ' ')
        else:
            for word, value in _SPEED_WORDS.items():
                if re.search(rf'\b{word}\b', rest):
                    level = value
                    rest = re.sub(rf'\b{word}\b', ' ', rest, count=1)
                    break
        clockwise = _ROTATE_CW_RE.search(rest)
        counter = _ROTATE_CCW_RE.search(rest)
        if counter:
            rest = rest.replace(counter.group(0), ' ')
        elif clockwise:
            rest = rest.replace(clockwise.group(0), ' ')
            level = -(level if level is not None else schema.DEFAULT_SPEED_LEVEL)
        angle = _ROTATE_ANGLE_RE.search(rest)
        if angle:
            rest = rest.replace(angle.group(0), ' ')
        # Whatever is left must be filler. "turn off the robot" leaves "off
        # the robot", which is not a rotation — fall through so the pick
        # grammar (and then the rejection path) gets it.
        leftover = re.sub(r'\b(?:a|bit|little|please|for|me|around|by|at|'
                          r'the|way|now|and|then)\b', ' ', rest)
        if leftover.strip(' ,.'):
            match = None
        else:
            return _jog_payload(schema.ACTION_ROTATE, {
                'speed_level': level,
                'angle_deg': float(angle.group('deg')) if angle else None,
            })

    match = _GO_TO_RE.match(text)
    if match:
        pose = _trim_target(match.group('pose'))
        # "go up" already matched above; a pose that is itself a direction
        # word, or empty, is not a pose.
        if pose and pose not in _DIRECTION_WORDS:
            motion = {'pose_name': pose}
            if match.group('dir'):
                motion['direction'] = _DIRECTION_WORDS[match.group('dir')]
                motion['distance_cm'] = _amount(match)
            return _jog_payload(schema.ACTION_GO_TO, motion)

    return None


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
        'motion': {key: None for key in schema.MOTION_KEYS},
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

        # Jog requests are checked before the question filter: "could you go
        # up a bit?" is a polite request with a question mark, not a question.
        jog = _try_jog(text.rstrip('.!?'))
        if jog is not None:
            return jog

        if text.endswith('?') or _QUESTION_RE.match(text):
            return _reject('That is a question, not a request to move an '
                           'object.')

        match = _COMMAND_RE.match(text.rstrip('.!'))
        if not match:
            return _reject('No recognised instruction: say what to pick up, or a '
                           'jog such as "go up 2", "spin slowly", "go home".')

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
