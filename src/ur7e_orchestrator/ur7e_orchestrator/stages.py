"""
The fixed stage sequence of the standardized pick workflow.

Every run walks the same stages in the same order. That is the whole point:
when two runs can only differ in *which stage* they stopped at and *why*,
their logs can be compared line by line. Nothing in this module does any
work; it only names the stages and holds their default time budgets.
"""
from enum import Enum


class Stage(Enum):
    """One step of the workflow; the value is the name written to logs."""

    IDLE = 'IDLE'            # between runs; a run record starts here
    PARSE = 'PARSE'          # utterance -> Intent
    OBSERVE = 'OBSERVE'      # move the arm clear of the camera view
    DETECT = 'DETECT'        # find the named object(s) in the image
    LOCATE = 'LOCATE'        # image detection -> metric pose in base_link
    PLAN = 'PLAN'            # check reachability, build the motion plan
    APPROACH = 'APPROACH'    # open jaws, hover above the target, descend
    GRASP = 'GRASP'          # close the jaws
    LIFT = 'LIFT'            # straight up, clear of the table
    RETREAT = 'RETREAT'      # carry to the place/drop location and release
    HOME = 'HOME'            # back to the home pose


# Enum members iterate in definition order, so this is the README's sequence.
STAGE_ORDER = tuple(Stage)

# Seconds a whole stage may take before the run is aborted. IDLE has no
# entry: it makes no adapter call, so there is nothing to wait for.
DEFAULT_TIMEOUTS_S = {
    Stage.PARSE: 30.0,
    Stage.OBSERVE: 120.0,
    Stage.DETECT: 60.0,
    Stage.LOCATE: 15.0,
    Stage.PLAN: 60.0,
    Stage.APPROACH: 180.0,
    Stage.GRASP: 30.0,
    Stage.LIFT: 120.0,
    Stage.RETREAT: 240.0,
    Stage.HOME: 180.0,
}

# How long the best-effort ``abort()`` call itself may take.
ABORT_TIMEOUT_S = 10.0

# Stages that move hardware: ``safety_ok()`` is checked on entry to each.
MOTION_STAGES = frozenset({
    Stage.OBSERVE,
    Stage.APPROACH,
    Stage.GRASP,
    Stage.LIFT,
    Stage.RETREAT,
    Stage.HOME,
})

# A failure in one of these stages is followed by a courtesy HOME: the arm
# has (or may have) left home, and its jaws are known to be empty. PARSE is
# left out because nothing has moved yet; GRASP and later are left out
# because the arm may be holding something.
HOME_AFTER_FAILURE = frozenset({
    Stage.OBSERVE,
    Stage.DETECT,
    Stage.LOCATE,
    Stage.PLAN,
    Stage.APPROACH,
})


def as_stage(value):
    """Return the Stage for a Stage or its name in any case ('detect')."""
    if isinstance(value, Stage):
        return value
    try:
        return Stage(str(value).strip().upper())
    except ValueError:
        names = ', '.join(stage.value for stage in STAGE_ORDER)
        raise ValueError(f'unknown stage {value!r}; expected one of: {names}') from None


def next_stage(stage):
    """Return the stage that follows ``stage`` in the fixed order."""
    return STAGE_ORDER[STAGE_ORDER.index(stage) + 1]


def resolve_timeouts(overrides=None):
    """Return the default timeouts with per-stage ``overrides`` applied."""
    timeouts = dict(DEFAULT_TIMEOUTS_S)
    for key, seconds in (overrides or {}).items():
        stage = as_stage(key)
        if stage not in DEFAULT_TIMEOUTS_S:
            raise ValueError(f'{stage.value} makes no adapter call and has no timeout')
        if not seconds > 0:
            raise ValueError(f'timeout for {stage.value} must be positive, got {seconds!r}')
        timeouts[stage] = float(seconds)
    return timeouts
