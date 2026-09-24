"""The five motion primitives behind ``ExecutePrimitive`` (task 1.5), each a
small function over a live ``pymoveit2.MoveIt2`` instance — read any one in
isolation to understand what it does, per the project's learning-platform
principle (docs/IMPLEMENTATION_PLAN.md).

Named poses (``home``, ``observe``) are resolved from the *running*
move_group's SRDF via ``RobotDescription.joint_positions()``, not duplicated
as constants here — the SRDF (``ur7e_pick_place_bringup/srdf``) is the single
source of truth, so editing a named pose there never requires a code change
here.
"""
from geometry_msgs.msg import PoseStamped
from pymoveit2.robot_description import RobotDescription


class PrimitiveError(Exception):
    """Raised for a malformed goal or a motion that failed to plan/execute."""


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


def approach_above(moveit2, target_pose, z_offset, timeout_sec):
    """Move to a hover pose ``z_offset`` metres above ``target_pose``.

    ``target_pose`` carries the grasp orientation (top-down in v1, per
    ARCHITECTURE.md D3) — only its ``position.z`` is shifted.
    """
    if target_pose is None:
        raise PrimitiveError("approach_above requires 'target_pose'")
    hover = PoseStamped()
    hover.header = target_pose.header
    hover.pose = target_pose.pose
    hover.pose.position.z += z_offset
    if not moveit2.move_to_pose(pose=hover, cartesian=True, timeout_sec=timeout_sec):
        raise PrimitiveError("failed to submit approach-above motion")
    if not moveit2.wait_until_executed(timeout_sec=timeout_sec):
        raise PrimitiveError("approach-above motion did not complete")


def _cartesian_z_move(moveit2, delta_z, timeout_sec, what):
    current = moveit2.compute_fk(timeout_sec=timeout_sec)
    if current is None:
        raise PrimitiveError(f"{what}: could not read the current TCP pose (FK failed)")
    target = PoseStamped()
    target.header = current.header
    target.pose = current.pose
    target.pose.position.z += delta_z
    if not moveit2.move_to_pose(pose=target, cartesian=True, timeout_sec=timeout_sec):
        raise PrimitiveError(f"failed to submit {what} motion")
    if not moveit2.wait_until_executed(timeout_sec=timeout_sec):
        raise PrimitiveError(f"{what} motion did not complete")


def cartesian_descend(moveit2, z_offset, timeout_sec):
    """Move straight down ``z_offset`` metres in the base frame."""
    _cartesian_z_move(moveit2, -abs(z_offset), timeout_sec, "cartesian_descend")


def cartesian_lift(moveit2, z_offset, timeout_sec):
    """Move straight up ``z_offset`` metres in the base frame."""
    _cartesian_z_move(moveit2, abs(z_offset), timeout_sec, "cartesian_lift")


def retreat(moveit2, z_offset, timeout_sec):
    """Move straight up ``z_offset`` metres — same motion as cartesian_lift,
    kept as a separate primitive/name because it means something different in
    the demo sequence (leaving the grasp site vs. transporting an object) —
    see ur7e_interfaces/action/ExecutePrimitive.action.
    """
    _cartesian_z_move(moveit2, abs(z_offset), timeout_sec, "retreat")
