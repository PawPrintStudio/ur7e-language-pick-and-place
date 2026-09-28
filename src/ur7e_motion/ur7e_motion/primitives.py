"""
The five motion primitives behind ``ExecutePrimitive`` (task 1.5).

Each is a small function over a live ``pymoveit2.MoveIt2`` instance — read
any one in isolation to understand what it does, per the project's
learning-platform principle (docs/IMPLEMENTATION_PLAN.md).

Named poses (``home``, ``observe``) are resolved from the *running*
move_group's SRDF via ``RobotDescription.joint_positions()``, not duplicated
as constants here — the SRDF (``ur7e_pick_place_bringup/srdf``) is the single
source of truth, so editing a named pose there never requires a code change
here.
"""
from copy import deepcopy
import math
import time


class PrimitiveError(Exception):
    """Raised for a malformed goal or a motion that failed to plan/execute."""


def _verify_endpoint(moveit2, target, timeout_sec):
    """Require feedback at the requested TCP pose after a successful action."""
    actual = moveit2.compute_fk(timeout_sec=timeout_sec)
    if actual is None or actual.header.frame_id != target.header.frame_id:
        raise PrimitiveError("could not verify TCP endpoint in target frame")
    a, b = actual.pose.position, target.pose.position
    error = math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
    q, r = actual.pose.orientation, target.pose.orientation
    dot = abs(q.x*r.x + q.y*r.y + q.z*r.z + q.w*r.w)
    if not math.isfinite(error) or not math.isfinite(dot) or error > 0.01 or dot < math.cos(0.05):
        raise PrimitiveError(f"TCP endpoint mismatch ({error:.4f} m)")


def goto_named(moveit2, description, group_name, named_pose, timeout_sec):
    """Move to an SRDF ``group_state`` by name (e.g. ``home``, ``observe``)."""
    if not named_pose:
        raise PrimitiveError("goto_named requires 'named_pose'")
    try:
        joint_positions = description.joint_positions(named_pose, group_name)
    except ValueError as error:
        raise PrimitiveError(f"unknown named pose '{named_pose}': {error}") from error
    if not moveit2.move_to_configuration(joint_positions, timeout_sec=timeout_sec):
        raise PrimitiveError(f"failed to submit motion to '{named_pose}'")
    if not moveit2.wait_until_executed(timeout_sec=timeout_sec):
        raise PrimitiveError(f"motion to '{named_pose}' did not complete")


def approach_above(moveit2, target_pose, z_offset, timeout_sec, configure_ik=None):
    """
    Move to a hover pose ``z_offset`` metres above ``target_pose``.

    ``target_pose`` carries the grasp orientation (top-down in v1, per
    ARCHITECTURE.md D3) — only its ``position.z`` is shifted.
    """
    if target_pose is None:
        raise PrimitiveError("approach_above requires 'target_pose'")
    hover = deepcopy(target_pose)
    hover.pose.position.z += z_offset
    if configure_ik is not None:
        deadline = time.monotonic() + timeout_sec

        def remaining():
            budget = deadline - time.monotonic()
            if budget <= 0:
                raise PrimitiveError('approach planning/execution deadline exceeded')
            return budget

        # Find a branch that can reach the grasp AND its complete vertical
        # approach before moving. A reachable hover alone is insufficient:
        # the elbow can hit the table halfway down on the wrong IK branch.
        for _ in range(4):
            remaining()
            configure_ik('global')
            grasp_state = moveit2.compute_ik(
                target_pose.pose.position, target_pose.pose.orientation,
                timeout_sec=remaining(),
            )
            if grasp_state is None:
                continue
            configure_ik('local')
            hover_state = moveit2.compute_ik(
                hover.pose.position, hover.pose.orientation,
                start_joint_state=grasp_state, timeout_sec=remaining(),
            )
            if hover_state is None:
                continue
            descent = moveit2.plan(
                pose=target_pose, start_joint_state=hover_state, cartesian=True,
                cartesian_fraction_threshold=1.0, timeout_sec=remaining(),
            )
            if descent is None:
                continue
            names = list(moveit2.joint_names)
            values = dict(zip(hover_state.name, hover_state.position))
            if not moveit2.move_to_configuration(
                [values[name] for name in names], timeout_sec=remaining(),
            ):
                continue
            if not moveit2.wait_until_executed(timeout_sec=remaining()):
                raise PrimitiveError('approach-above motion did not complete')
            _verify_endpoint(moveit2, hover, remaining())
            return
        raise PrimitiveError('no collision-free hover and complete descent found')
    if not moveit2.move_to_pose(pose=hover, cartesian=False, timeout_sec=timeout_sec):
        raise PrimitiveError("failed to submit approach-above motion")
    if not moveit2.wait_until_executed(timeout_sec=timeout_sec):
        raise PrimitiveError("approach-above motion did not complete")
    _verify_endpoint(moveit2, hover, timeout_sec)


def _cartesian_z_move(moveit2, delta_z, timeout_sec, what):
    current = moveit2.compute_fk(timeout_sec=timeout_sec)
    if current is None:
        raise PrimitiveError(f"{what}: could not read the current TCP pose (FK failed)")
    target = deepcopy(current)
    target.pose.position.z += delta_z
    if not moveit2.move_to_pose(
        pose=target, cartesian=True, cartesian_fraction_threshold=1.0,
        timeout_sec=timeout_sec,
    ):
        raise PrimitiveError(f"failed to submit {what} motion")
    if not moveit2.wait_until_executed(timeout_sec=timeout_sec):
        raise PrimitiveError(f"{what} motion did not complete")
    _verify_endpoint(moveit2, target, timeout_sec)


def cartesian_descend(moveit2, z_offset, timeout_sec):
    """Move straight down ``z_offset`` metres in the base frame."""
    _cartesian_z_move(moveit2, -abs(z_offset), timeout_sec, "cartesian_descend")


def cartesian_lift(moveit2, z_offset, timeout_sec):
    """Move straight up ``z_offset`` metres in the base frame."""
    _cartesian_z_move(moveit2, abs(z_offset), timeout_sec, "cartesian_lift")


def retreat(moveit2, z_offset, timeout_sec):
    """
    Move straight up ``z_offset`` metres.

    Same motion as ``cartesian_lift``, kept as a separate primitive/name
    because it means something different in the demo sequence (leaving the
    grasp site vs. transporting an object) — see
    ur7e_interfaces/action/ExecutePrimitive.action.
    """
    _cartesian_z_move(moveit2, abs(z_offset), timeout_sec, "retreat")
