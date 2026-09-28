"""
Turn whatever the backend said into a structure we are willing to trust.

This module is the trust boundary. Everything upstream of it — a cloud LLM, a
local model, a regex fallback — is treated as an *untrusted text generator*,
because that is what it is. Nothing here assumes the backend cooperated.

Why the boundary is here and not in the backend
-----------------------------------------------
Each backend could validate its own output, and the cloud one even gets
constrained decoding for free. But then the safety properties would be a
property of *each backend* rather than of the node, and the weakest backend
would set the real floor. Validating once, downstream of all of them, means
adding a local Llama backend later cannot widen what the arm will act on.

What "valid" means here is deliberately narrow: correct shape, known
vocabulary, and a ``target_query`` that is genuinely a noun phrase. Whether we
are *confident enough* to move is a separate question, answered by
:mod:`arm_language.guardrails`.
"""
import json
import re
from dataclasses import dataclass, field
from typing import Dict, Optional

from . import schema
from .result import ReasonCode

_WHITESPACE = re.compile(r'\s+')

# Leading/trailing junk an LLM sometimes wraps a phrase in.
_EDGE_PUNCTUATION = ' \t\'"`.,;:!?()[]{}'

_TARGET_CHARS = re.compile(rf'^[{schema.TARGET_CHAR_CLASS}]+$')


class ValidationError(Exception):
    """
    Raised when backend output cannot be trusted.

    Internal to the parse pipeline: :mod:`arm_language.parser` catches this and
    converts it into a refusal. It never escapes the public API — callers of
    ``IntentParser.parse`` get a :class:`~arm_language.result.ParseResult`.
    """

    def __init__(self, reason_code: ReasonCode, message: str) -> None:
        """Store the stable reason code alongside the human-readable message."""
        super().__init__(message)
        self.reason_code = reason_code
        self.message = message


@dataclass(frozen=True)
class ValidatedIntent:
    """
    Structurally valid backend output, before any policy is applied.

    Distinct from :class:`~arm_language.result.Command` because it can still
    carry ``action == "reject"``, and because no confidence policy has run yet.
    A ``Command`` means "we will act on this"; a ``ValidatedIntent`` only means
    "we understood the shape of this".
    """

    action: str
    target_query: str
    place_target: Optional[str] = None
    modifiers: Dict[str, str] = field(default_factory=dict)
    confidence: float = 0.0
    reason: Optional[str] = None


def normalize_phrase(raw: str) -> str:
    """
    Reduce a noun phrase to the canonical form NanoOWL will see.

    Lowercases, collapses whitespace, strips edge punctuation and quoting, and
    drops a leading article. The goal is that "the Hammer", "a hammer" and
    "hammer" all produce the *same* detector query — otherwise two runs of the
    same spoken command are not comparable, and neither are their logs.

    Note this only *normalises*; it never repairs. Deciding whether the result
    is acceptable is :func:`check_noun_phrase`'s job.
    """
    text = _WHITESPACE.sub(' ', raw).strip(_EDGE_PUNCTUATION).strip().lower()
    words = text.split()
    if len(words) > 1 and words[0] in schema.ARTICLES:
        words = words[1:]
    return ' '.join(words)


def check_noun_phrase(phrase: str, field_name: str) -> None:
    """
    Raise unless ``phrase`` is a bare noun phrase.

    This is the check the whole node is built around. NanoOWL takes noun
    phrases, not sentences (architecture D3) — feeding it "pick up the hammer"
    degrades detection quietly, producing a *worse grasp* rather than an error.
    Silent degradation is the failure mode we cannot debug from a run log, so
    we turn it into a loud refusal here.

    Rejects rather than repairs: if the backend handed us an instruction where
    a noun belonged, its understanding of the request is wrong, and trimming
    the verb off would hide that while leaving the rest of the parse suspect.
    """
    if not phrase:
        raise ValidationError(
            ReasonCode.TARGET_EMPTY,
            f'The backend did not name an object for `{field_name}`.',
        )

    if len(phrase) > schema.MAX_TARGET_CHARS:
        raise ValidationError(
            ReasonCode.TARGET_NOT_NOUN_PHRASE,
            f'`{field_name}` is {len(phrase)} characters; a noun phrase should '
            f'be under {schema.MAX_TARGET_CHARS}.',
        )

    if any(char in phrase for char in schema.SENTENCE_PUNCTUATION):
        raise ValidationError(
            ReasonCode.TARGET_NOT_NOUN_PHRASE,
            f'`{field_name}` reads like a sentence, not an object name: '
            f'"{phrase}".',
        )

    if not _TARGET_CHARS.match(phrase):
        raise ValidationError(
            ReasonCode.TARGET_NOT_NOUN_PHRASE,
            f'`{field_name}` contains characters that do not belong in an '
            f'object name: "{phrase}".',
        )

    words = phrase.split()
    if len(words) > schema.MAX_TARGET_WORDS:
        raise ValidationError(
            ReasonCode.TARGET_NOT_NOUN_PHRASE,
            f'`{field_name}` is {len(words)} words; expected at most '
            f'{schema.MAX_TARGET_WORDS} ("{phrase}").',
        )

    # Only the FIRST word is checked against the verb list, because these words
    # are perfectly good nouns elsewhere in a phrase — a makerspace really does
    # own guitar picks. "pick" leading the phrase means an instruction leaked
    # through; "guitar pick" is an object.
    #
    # Known cost of this rule: an object whose name STARTS with one of these
    # words is refused. "drop cloth" and "file" are the realistic casualties.
    # We take that trade because the two failures are not symmetric — a
    # refused drop cloth is a visible message the member can rephrase past,
    # while an accepted "drop the hammer" silently sends an instruction to the
    # detector and degrades the grasp with no error anywhere.
    if words[0] in schema.IMPERATIVE_VERBS:
        raise ValidationError(
            ReasonCode.TARGET_NOT_NOUN_PHRASE,
            f'`{field_name}` starts with the verb "{words[0]}" — that is an '
            f'instruction, not an object name ("{phrase}").',
        )


def _require(condition: bool, reason_code: ReasonCode, message: str) -> None:
    """Raise :class:`ValidationError` unless ``condition`` holds."""
    if not condition:
        raise ValidationError(reason_code, message)


def _validate_modifiers(raw: object) -> Dict[str, str]:
    """
    Check the modifier map and drop the keys the speaker did not use.

    The wire format requires every known key to be present (strict structured
    outputs cannot express "optional"), so most of them arrive as ``null``.
    Those are noise; what survives is only what the speaker actually said.
    """
    _require(isinstance(raw, dict), ReasonCode.SCHEMA_VIOLATION,
             '`modifiers` must be an object.')

    unknown = sorted(set(raw) - set(schema.MODIFIER_KEYS))
    _require(
        not unknown,
        ReasonCode.MODIFIER_KEY_UNKNOWN,
        f'Unknown modifier key(s): {", ".join(unknown)}. Known keys are '
        f'{", ".join(schema.MODIFIER_KEYS)}.',
    )

    cleaned = {}
    for key, value in raw.items():
        if value is None:
            continue
        _require(isinstance(value, str), ReasonCode.SCHEMA_VIOLATION,
                 f'Modifier `{key}` must be a string or null.')
        normalized = normalize_phrase(value)
        if normalized:
            cleaned[key] = normalized
    return cleaned


def validate(raw_text: str) -> ValidatedIntent:
    """
    Parse and validate one backend response.

    Raises :class:`ValidationError` with a stable reason code on any problem.
    Returns a :class:`ValidatedIntent` that is structurally sound — which is
    not yet a promise that we will act on it.
    """
    try:
        payload = json.loads(raw_text)
    except (ValueError, TypeError) as exc:
        raise ValidationError(
            ReasonCode.NOT_JSON,
            f'The backend did not return JSON ({exc}).',
        ) from exc

    _require(isinstance(payload, dict), ReasonCode.SCHEMA_VIOLATION,
             'The backend returned JSON, but not a JSON object.')

    missing = sorted(set(schema.COMMAND_SCHEMA['required']) - set(payload))
    _require(not missing, ReasonCode.SCHEMA_VIOLATION,
             f'Missing required field(s): {", ".join(missing)}.')

    # An unexpected top-level key means the model improvised. We refuse instead
    # of ignoring it: an extra field is evidence it was answering a different
    # question than the one we asked.
    extra = sorted(set(payload) - set(schema.COMMAND_SCHEMA['required']))
    _require(not extra, ReasonCode.SCHEMA_VIOLATION,
             f'Unexpected field(s): {", ".join(extra)}.')

    action = payload['action']
    _require(isinstance(action, str), ReasonCode.SCHEMA_VIOLATION,
             '`action` must be a string.')
    action = action.strip().lower()
    _require(action in schema.BACKEND_ACTIONS, ReasonCode.SCHEMA_VIOLATION,
             f'`action` must be one of {", ".join(schema.BACKEND_ACTIONS)}; '
             f'got "{action}".')

    _require(isinstance(payload['target_query'], str),
             ReasonCode.SCHEMA_VIOLATION, '`target_query` must be a string.')
    target_query = normalize_phrase(payload['target_query'])

    confidence = payload['confidence']
    # bool is a subclass of int in Python, and `True` is not a confidence.
    _require(isinstance(confidence, (int, float))
             and not isinstance(confidence, bool),
             ReasonCode.SCHEMA_VIOLATION, '`confidence` must be a number.')
    confidence = float(confidence)
    # Out of range is not a rounding slip; it means the backend is not working
    # to the contract, so clamping would be inventing trust we do not have.
    _require(0.0 <= confidence <= 1.0, ReasonCode.SCHEMA_VIOLATION,
             f'`confidence` must be between 0.0 and 1.0; got {confidence}.')

    reason = payload['reason']
    _require(reason is None or isinstance(reason, str),
             ReasonCode.SCHEMA_VIOLATION, '`reason` must be a string or null.')

    modifiers = _validate_modifiers(payload['modifiers'])

    place_raw = payload['place_target']
    _require(place_raw is None or isinstance(place_raw, str),
             ReasonCode.SCHEMA_VIOLATION,
             '`place_target` must be a string or null.')
    place_target = normalize_phrase(place_raw) if place_raw else None

    # A rejection carries no object, so the noun-phrase rules do not apply.
    if action != schema.ACTION_REJECT:
        check_noun_phrase(target_query, 'target_query')

        # Cross-field rule: the destination exists if and only if the action
        # needs one. JSON Schema can only express this with `oneOf` branches
        # that strict structured outputs reject — so it lives here, where it
        # also reads more clearly than it would as schema.
        if action == schema.ACTION_PICK_AND_PLACE:
            _require(place_target is not None,
                     ReasonCode.PLACE_TARGET_MISSING,
                     'A pick_and_place command needs a destination, but '
                     '`place_target` was empty.')
            check_noun_phrase(place_target, 'place_target')
        else:
            _require(place_target is None,
                     ReasonCode.PLACE_TARGET_UNEXPECTED,
                     f'`place_target` was set to "{place_target}" on a plain '
                     f'pick, which has no destination.')

    return ValidatedIntent(
        action=action,
        target_query=target_query,
        place_target=place_target,
        modifiers=modifiers,
        confidence=confidence,
        reason=reason,
    )
