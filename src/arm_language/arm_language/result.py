"""
What a parse produces: an outcome, a reason code, and maybe a command.

Two design choices here are worth understanding before you use this module.

**A refusal is a value, not an exception.** ``parse()`` returns a
:class:`ParseResult` for every input, including garbage. Nothing raises. This
matters because the caller is a state machine driving a robot arm: an exception
path that someone forgets to catch becomes an unhandled crash mid-run, whereas
an ``outcome`` field cannot be forgotten — you have to read it to get at the
command. The type system does the remembering.

**Every outcome carries a stable reason code.** ``message`` is for humans and
will be reworded; :class:`ReasonCode` is for logs, dashboards, and the
per-stage run records in architecture §1.2. Counting "how often did we refuse,
and why" needs a string that does not change when someone improves the wording.
"""
from dataclasses import dataclass, field
from enum import Enum
import json
from typing import Dict, Optional

from . import schema


class Outcome(str, Enum):
    """
    The three things a parse can conclude.

    Subclasses ``str`` so it serialises straight into JSON reports and ROS
    string fields without a conversion step at every boundary.
    """

    ACCEPTED = 'accepted'
    """Valid and confident. The caller may proceed to motion."""

    NEEDS_CONFIRMATION = 'needs_confirmation'
    """Valid but uncertain. Echo ``message`` to a human first (issue #23)."""

    REFUSED = 'refused'
    """Not actionable. ``command`` is ``None``; do not move."""


class ReasonCode(str, Enum):
    """Why a parse ended the way it did. Stable across wording changes."""

    OK = 'ok'
    """Accepted with no reservations."""

    # --- input-side refusals (we never reached the backend) ---
    EMPTY_UTTERANCE = 'empty_utterance'
    UTTERANCE_TOO_LONG = 'utterance_too_long'

    # --- backend-side failures ---
    BACKEND_ERROR = 'backend_error'
    """The backend raised: no network, no credentials, timeout, 5xx."""

    NOT_JSON = 'not_json'
    """The backend returned something that is not parseable JSON."""

    # --- schema violations ---
    SCHEMA_VIOLATION = 'schema_violation'
    """Missing field, wrong type, or an unexpected top-level key."""

    MODIFIER_KEY_UNKNOWN = 'modifier_key_unknown'
    ACTION_NOT_ALLOWED = 'action_not_allowed'
    """The action is not on the motion whitelist (issue #23)."""

    PLACE_TARGET_MISSING = 'place_target_missing'
    PLACE_TARGET_UNEXPECTED = 'place_target_unexpected'

    # --- jog actions (move / rotate / go_to) ---
    MOTION_FIELD_MISSING = 'motion_field_missing'
    """A jog action without the parameter it needs (a move with no direction)."""
    MOTION_FIELD_UNEXPECTED = 'motion_field_unexpected'
    """A motion parameter on an action that has no use for it."""
    MOTION_OUT_OF_BOUNDS = 'motion_out_of_bounds'
    """Distance, angle or speed outside the bounds in :mod:`arm_language.schema`.
    This is the refusal that makes "go up one metre" impossible by construction."""

    # --- semantic checks ---
    TARGET_EMPTY = 'target_empty'
    TARGET_NOT_NOUN_PHRASE = 'target_not_noun_phrase'
    """A sentence, an instruction, or too long to be a noun phrase."""

    # --- policy ---
    LLM_REJECTED = 'llm_rejected'
    """The backend itself judged this not to be a manipulation request."""

    LOW_CONFIDENCE = 'low_confidence'
    CONFIRMATION_REQUIRED = 'confirmation_required'
    """Valid and confident, but the policy asks a human before every move."""


@dataclass(frozen=True)
class Command:
    """
    A validated manipulation command.

    Frozen because a command that passed validation must not be edited
    afterwards — a mutable command means the thing that was checked and the
    thing that reaches the arm can differ.

    Mirrors ``arm_interfaces/msg/Command`` field for field, but deliberately
    does not import it: the whole parser core runs, and is tested, without a
    ROS installation. Only :mod:`arm_language.intent_parser_node` touches ROS
    types.
    """

    action: str
    target_query: str
    place_target: Optional[str] = None
    modifiers: Dict[str, str] = field(default_factory=dict)
    confidence: float = 0.0

    # Jog parameters (schema.JOG_ACTIONS). After validation these are never
    # None on the action that uses them: defaults have already been applied,
    # so the executor reads numbers, not "maybe a number".
    direction: Optional[str] = None
    distance_cm: Optional[float] = None
    speed_level: Optional[int] = None
    angle_deg: Optional[float] = None
    pose_name: Optional[str] = None

    def modifiers_json(self) -> str:
        """
        Return ``modifiers`` as the JSON string the ROS message carries.

        Keys are sorted so the same command always serialises identically —
        which makes run logs diffable and lets tests compare strings.
        """
        return json.dumps(self.modifiers, sort_keys=True)

    def echo(self) -> str:
        """
        Render the command back as a plain-English sentence.

        This is the "did you mean ...?" string of issue #23. It is built from
        the *parsed structure*, never from the user's original words — so what
        the human confirms is exactly what the arm will act on. Echoing the raw
        utterance back would confirm nothing: the misparse is precisely the
        difference between the two.
        """
        if self.action in schema.JOG_ACTIONS:
            return self._echo_jog()

        # Modifiers the speaker used to identify the object normally survive
        # in `target_query` too ("red screwdriver"), so only the ones missing
        # from it are worth prepending — otherwise the echo reads "the red red
        # screwdriver", which is exactly the kind of wrongness that trains
        # people to stop reading confirmations.
        extra = [self.modifiers[key] for key in schema.MODIFIER_KEYS
                 if self.modifiers.get(key)
                 and self.modifiers[key] not in self.target_query]
        described = ' '.join(extra + [self.target_query])
        if self.action == schema.ACTION_PICK_AND_PLACE and self.place_target:
            return f'pick up the {described} and place it in the {self.place_target}'
        return f'pick up the {described}'

    def _echo_jog(self) -> str:
        """Echo for move / rotate / go_to / teleop, with every number the arm will use."""
        if self.action == schema.ACTION_TELEOP:
            return 'hand you the controls: keyboard teleop until you press q'
        if self.action == schema.ACTION_MOVE:
            return f'move the tool {self.direction} by {self.distance_cm:g} cm'
        if self.action == schema.ACTION_ROTATE:
            level = self.speed_level or 0
            spin = 'counter-clockwise' if level > 0 else 'clockwise'
            pace = {1: 'slowly', 2: 'at medium speed', 3: 'fast'}.get(abs(level), '')
            return (f'rotate the wrist {self.angle_deg:g} degrees {spin} '
                    f'{pace} (speed {level:+d})')
        offset = ''
        if self.direction and self.distance_cm:
            offset = f', then {self.direction} by {self.distance_cm:g} cm'
        return f'go to the "{self.pose_name}" pose{offset}'


@dataclass(frozen=True)
class ParseResult:
    """The complete outcome of one parse attempt."""

    outcome: Outcome
    reason_code: ReasonCode
    message: str
    command: Optional[Command] = None

    # Free-form diagnostics: backend name, latency, the raw text on a failure.
    # Never read by control flow — it exists so a failed run can be explained
    # after the fact without re-running it.
    detail: Dict[str, object] = field(default_factory=dict)

    @property
    def may_move(self) -> bool:
        """
        Whether the caller may command motion from this result alone.

        The single predicate every motion-side caller should branch on. It is
        deliberately conservative: ``NEEDS_CONFIRMATION`` is ``False`` here,
        because the confirmation has not happened yet at the moment this is
        read.
        """
        return self.outcome is Outcome.ACCEPTED and self.command is not None

    @classmethod
    def refuse(cls, reason_code: ReasonCode, message: str,
               **detail: object) -> 'ParseResult':
        """Build a refusal. Convenience for the many refusal paths."""
        return cls(
            outcome=Outcome.REFUSED,
            reason_code=reason_code,
            message=message,
            command=None,
            detail=dict(detail),
        )
