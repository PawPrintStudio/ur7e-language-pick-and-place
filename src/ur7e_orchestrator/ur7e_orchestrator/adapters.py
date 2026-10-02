"""
The adapter interface: everything the state machine asks of the outside world.

The workflow never talks to ROS, the camera, the language model, or the
robot directly. It calls the methods of one ``Adapters`` object. Swapping
that object is how the same state machine runs against mocks in CI
(``mock.MockAdapters``) and against the real arm in the lab.

Threading contract for whoever implements the real adapter: every method
is called on one persistent worker thread, one call at a time, so objects
the adapter creates on that thread stay on it. The single exception is
``abort()``: when a stage times out the worker is, by definition, stuck
inside the call that timed out, so ``abort()`` is then called from a
different thread and must be safe to run concurrently with that call.
"""
from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class Intent:
    """Result of PARSE: what the operator asked for."""

    action: str             # 'pick' or 'pick_and_place'
    target_query: str       # noun phrase for the detector, e.g. 'red block'
    place_query: str = ''   # '' when the action has no place target
    echo: str = ''          # human-readable restatement


@dataclass(frozen=True)
class Detection:
    """Result of DETECT: a handle to one object found in one capture."""

    capture_id: str
    confidence: float


@dataclass(frozen=True)
class Target:
    """Result of LOCATE: where the object is, in the robot's frame."""

    xyz: tuple              # metres, base_link
    yaw: float              # radians
    detail: dict = field(default_factory=dict)


@dataclass(frozen=True)
class GraspResult:
    """Result of GRASP: whether the jaws closed on something."""

    holding: bool           # False = jaws closed on nothing (grasp miss)
    width_mm: float
    detail: dict = field(default_factory=dict)


class StageError(Exception):
    """
    A stage failed for a known cause.

    ``reason`` is a short, stable snake_case code (``unreachable``,
    ``not_found``): it is what logs are grouped and compared by, so it must
    not contain run-specific text. ``message`` is the free-form explanation
    for the human reading one particular run.
    """

    def __init__(self, reason, message=''):
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason


class Refused(StageError):
    """The parser refused or the operator declined: no motion may follow."""


class NotFound(StageError):
    """The detector found nothing, or could not choose between candidates."""


class SafetyAbort(StageError):
    """Protective stop, e-stop, or program stopped: the run is never retried."""


class Adapters:
    """
    Base class for adapters; every method raises NotImplementedError.

    Subclass it and override all methods. A method reports a failure by
    raising ``StageError`` (or one of its subclasses) with a stable reason
    code; any other exception is recorded as ``unexpected_error``.
    """

    def parse(self, utterance: str) -> Intent:
        """Turn the operator's utterance into an Intent, or raise Refused."""
        raise NotImplementedError

    def observe(self) -> None:
        """Move the arm clear of the camera view."""
        raise NotImplementedError

    def detect(self, query: str) -> Detection:
        """Find the object named by ``query``, or raise NotFound."""
        raise NotImplementedError

    def locate(self, detection: Detection) -> Target:
        """Turn a detection into a metric pose in base_link."""
        raise NotImplementedError

    def plan(self, intent: Intent, target: Target, place: Optional[Target]) -> dict:
        """Return a JSON-able plan summary; raise StageError if unreachable."""
        raise NotImplementedError

    def approach(self) -> None:
        """Open the jaws, hover above the target, and descend."""
        raise NotImplementedError

    def grasp(self) -> GraspResult:
        """Close the jaws and report whether they hold something."""
        raise NotImplementedError

    def lift(self) -> None:
        """Move straight up, clear of the table."""
        raise NotImplementedError

    def retreat(self) -> None:
        """Carry to the place/drop location, release, and back away."""
        raise NotImplementedError

    def home(self) -> None:
        """Return the arm to its home pose."""
        raise NotImplementedError

    def release(self) -> None:
        """Open the jaws where the arm is (used when a grasp missed)."""
        raise NotImplementedError

    def safety_ok(self) -> tuple:
        """
        Return ``(ok, reason)``; checked before every stage that moves hardware.

        When ``ok`` is False the run aborts with ``reason`` as its reason
        code, so return a stable snake_case code such as ``protective_stop``.
        """
        raise NotImplementedError

    def abort(self) -> None:
        """
        Stop any motion in flight, best effort.

        Called after a stage timeout or a safety abort. After a timeout it
        runs on a different thread than the stuck call (see the module
        docstring), and it should return quickly.
        """
        raise NotImplementedError
