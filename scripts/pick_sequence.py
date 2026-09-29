"""One pick-and-place plan, shared by every way of running it.

The same ordered list of steps drives both:

* ``pick_place_demo.py`` — simulation (tiers 1-3), where the gripper is a
  ``GripperCommand`` action and every step runs straight through; and
* ``lab_urcap_handoff.py --pick-place`` — the real UR7e + RG2, where the
  gripper belongs to the OnRobot URCap on the pendant, so each gripper step
  becomes a ROS -> pendant -> ROS handoff (docs/URCAP_HANDOFF.md).

Keeping the plan as plain data (no ROS imports) means the sequence you watch
in simulation is, step for step, the sequence the real arm executes; only the
executor differs. It also lets this file be unit-tested without ROS.
"""
from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

# Primitive names match ur7e_interfaces/action/ExecutePrimitive.action.
GOTO_NAMED = "goto_named"
APPROACH_ABOVE = "approach_above"
DESCEND = "cartesian_descend"
LIFT = "cartesian_lift"
RETREAT = "retreat"

OPEN = "open"
CLOSE = "close"

# Tall enough that a joint-space plan from the hover pose to any named pose
# does not need to route around the table collision box
# (ur7e_pick_place_bringup/planning_scene.py).
HOVER_HEIGHT = 0.25  # m

XYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class Motion:
    """One ``ExecutePrimitive`` goal. ``xyz`` is a base_link target (m)."""

    primitive: str
    named_pose: str = ""
    xyz: Optional[XYZ] = None
    z_offset: float = 0.0


@dataclass(frozen=True)
class Gripper:
    """Open or close. Widths/force belong to the executor, not the plan."""

    action: str


Step = object  # Motion | Gripper


def pick_place_steps(pick_xyz: XYZ, place_xyz: XYZ,
                     hover: float = HOVER_HEIGHT) -> List[Step]:
    """Home -> open -> pick -> lift -> carry -> place -> retreat -> home."""
    for xyz in (pick_xyz, place_xyz):
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            raise ValueError(f"target must be three finite metres, got {xyz}")
    if not 0 < hover <= 0.4:
        raise ValueError("hover height must be in (0, 0.4] m")
    return [
        Motion(GOTO_NAMED, named_pose="home"),
        Gripper(OPEN),
        Motion(APPROACH_ABOVE, xyz=tuple(pick_xyz), z_offset=hover),
        Motion(DESCEND, z_offset=hover),
        Gripper(CLOSE),
        Motion(LIFT, z_offset=hover),
        Motion(APPROACH_ABOVE, xyz=tuple(place_xyz), z_offset=hover),
        Motion(DESCEND, z_offset=hover),
        Gripper(OPEN),
        Motion(RETREAT, z_offset=hover),
        # Via `observe`: a direct plan from above the place target to `home`
        # was observed routing through the table collision box and failing.
        Motion(GOTO_NAMED, named_pose="observe"),
        Motion(GOTO_NAMED, named_pose="home"),
    ]


def split_for_handoff(steps: List[Step]):
    """Cut the plan into the three arm segments of the pendant program.

    The pendant program (docs/URCAP_HANDOFF.md) has exactly one close phase
    followed by one open phase, and starts with the jaws already open (its
    hello message reports the initial width, which the coordinator checks).
    So a leading ``open`` is satisfied by that check and dropped, and what
    remains must be exactly: motions, close, motions, open, motions.

    Returns ``(before_close, before_open, after_open)`` lists of Motion.
    """
    segments: List[List[Motion]] = [[]]
    grips: List[str] = []
    for step in steps:
        if isinstance(step, Motion):
            segments[-1].append(step)
        elif isinstance(step, Gripper):
            if step.action == OPEN and not grips:
                continue
            grips.append(step.action)
            segments.append([])
        else:
            raise TypeError(f"unknown step {step!r}")
    if grips != [CLOSE, OPEN]:
        raise ValueError(f"handoff program supports close then open, got {grips}")
    return segments[0], segments[1], segments[2]
