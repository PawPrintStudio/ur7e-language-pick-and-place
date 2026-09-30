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

    # Jog parameters, defaults already applied (see _validate_motion).
    direction: Optional[str] = None
    distance_cm: Optional[float] = None
    speed_level: Optional[int] = None
    angle_deg: Optional[float] = None
    pose_name: Optional[str] = None


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


def _number(raw: object, name: str) -> Optional[float]:
    """Return ``raw`` as a float, or None; refuse bools and non-numbers."""
    if raw is None:
        return None
    _require(isinstance(raw, (int, float)) and not isinstance(raw, bool),
             ReasonCode.SCHEMA_VIOLATION, f'`motion.{name}` must be a number or null.')
    return float(raw)


def _validate_motion(action: str, raw: object) -> Dict[str, object]:
    """
    Check the ``motion`` object against the action, and apply defaults.

    The cross-field rules live here for the same reason ``place_target``'s do:
    "a move needs a direction, a rotate must not have one" is one line of
    Python and a thicket of ``oneOf`` in JSON Schema.

    The bounds checks are the load-bearing part. A language model can be
    talked into writing ``distance_cm: 100``; nothing can talk this function
    into passing it through. Refusing (rather than clamping to the maximum)
    is deliberate: a clamped value moves the arm somewhere the speaker did not
    ask for, while a refusal tells them what the limit is.
    """
    # Absent entirely is tolerated (older backends and fixtures never send
    # it) and means "no motion parameters" — identical to all-null.
    if raw is None:
        raw = {}
    _require(isinstance(raw, dict), ReasonCode.SCHEMA_VIOLATION,
             '`motion` must be an object.')
    unknown = sorted(set(raw) - set(schema.MOTION_KEYS))
    _require(not unknown, ReasonCode.SCHEMA_VIOLATION,
             f'Unknown motion key(s): {", ".join(unknown)}.')

    direction = raw.get('direction')
    _require(direction is None or isinstance(direction, str),
             ReasonCode.SCHEMA_VIOLATION, '`motion.direction` must be a string or null.')
    if direction is not None:
        direction = direction.strip().lower()
        _require(direction in schema.DIRECTIONS, ReasonCode.SCHEMA_VIOLATION,
                 f'`motion.direction` must be one of {", ".join(schema.DIRECTIONS)}; '
                 f'got "{direction}".')
    distance = _number(raw.get('distance_cm'), 'distance_cm')
    angle = _number(raw.get('angle_deg'), 'angle_deg')
    speed = raw.get('speed_level')
    _require(speed is None or (isinstance(speed, int) and not isinstance(speed, bool)),
             ReasonCode.SCHEMA_VIOLATION, '`motion.speed_level` must be an integer or null.')
    pose_raw = raw.get('pose_name')
    _require(pose_raw is None or isinstance(pose_raw, str),
             ReasonCode.SCHEMA_VIOLATION, '`motion.pose_name` must be a string or null.')
    pose_name = normalize_phrase(pose_raw) if pose_raw else None

    given = {key: value for key, value in [
        ('direction', direction), ('distance_cm', distance),
        ('speed_level', speed), ('angle_deg', angle), ('pose_name', pose_name),
    ] if value is not None}

    def only(*allowed: str) -> None:
        stray = sorted(set(given) - set(allowed))
        _require(not stray, ReasonCode.MOTION_FIELD_UNEXPECTED,
                 f'`{action}` does not use motion.{", motion.".join(stray)}.')

    def check_distance(value: float) -> None:
        _require(0.0 < value <= schema.MAX_MOVE_CM, ReasonCode.MOTION_OUT_OF_BOUNDS,
                 f'A move must be between 0 and {schema.MAX_MOVE_CM:g} cm; '
                 f'got {value:g} cm.')

    if action == schema.ACTION_MOVE:
        only('direction', 'distance_cm')
        _require(direction is not None, ReasonCode.MOTION_FIELD_MISSING,
                 'A move needs a direction (up, down, left, right, forward, backward).')
        if distance is None:
            distance = schema.DEFAULT_MOVE_CM
        check_distance(distance)
        return {'direction': direction, 'distance_cm': distance}

    if action == schema.ACTION_ROTATE:
        only('speed_level', 'angle_deg')
        if speed is None:
            speed = schema.DEFAULT_SPEED_LEVEL
        _require(speed != 0 and abs(speed) <= schema.MAX_SPEED_LEVEL,
                 ReasonCode.MOTION_OUT_OF_BOUNDS,
                 f'Speed level must be between -{schema.MAX_SPEED_LEVEL} and '
                 f'{schema.MAX_SPEED_LEVEL}, not zero; got {speed}.')
        if angle is None:
            angle = schema.DEFAULT_ROTATE_DEG
        _require(0.0 < angle <= schema.MAX_ROTATE_DEG, ReasonCode.MOTION_OUT_OF_BOUNDS,
                 f'A rotation must be between 0 and {schema.MAX_ROTATE_DEG:g} degrees; '
                 f'got {angle:g}.')
        return {'speed_level': speed, 'angle_deg': angle}

    if action == schema.ACTION_GO_TO:
        only('pose_name', 'direction', 'distance_cm')
        _require(pose_name is not None, ReasonCode.MOTION_FIELD_MISSING,
                 'A go_to needs the name of a pose ("home").')
        check_noun_phrase(pose_name, 'motion.pose_name')
        # An offset is optional, but it is all-or-nothing: a distance with no
        # direction is not a move anyone asked for.
        if direction is not None or distance is not None:
            _require(direction is not None, ReasonCode.MOTION_FIELD_MISSING,
                     'A go_to offset needs a direction as well as a distance.')
            if distance is None:
                distance = schema.DEFAULT_MOVE_CM
            check_distance(distance)
        return {'pose_name': pose_name, 'direction': direction,
                'distance_cm': distance}

    # pick, pick_and_place, reject: motion parameters are noise at best and
    # evidence of a confused parse at worst. Refuse rather than ignore.
    only()
    return {}


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

    # `motion` is required on the wire (strict structured outputs) but
    # tolerated when absent here, so a backend that predates the jog actions
    # keeps working. Absent means "no motion parameters".
    missing = sorted(set(schema.COMMAND_SCHEMA['required']) - set(payload)
                     - {'motion'})
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

    motion = _validate_motion(action, payload.get('motion'))

    if action in schema.JOG_ACTIONS:
        # A jog moves the tool, not an object: a named target here means the
        # model blended two requests ("move up and grab the hammer").
        _require(not target_query, ReasonCode.SCHEMA_VIOLATION,
                 f'`{action}` moves the tool itself; `target_query` must be '
                 f'empty, got "{target_query}".')
        _require(place_target is None, ReasonCode.PLACE_TARGET_UNEXPECTED,
                 f'`{action}` has no destination object; `place_target` must be null.')
        _require(not modifiers, ReasonCode.SCHEMA_VIOLATION,
                 f'`{action}` takes no object modifiers.')

    # A rejection or jog carries no object, so the noun-phrase rules do not apply.
    if action not in (schema.ACTION_REJECT,) + schema.JOG_ACTIONS:
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
        **motion,
    )
