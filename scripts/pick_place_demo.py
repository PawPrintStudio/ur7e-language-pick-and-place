#!/usr/bin/env python3
"""Task 1.7's scripted pick: home -> approach above the pick pose -> descend
-> close gripper -> lift -> transport to the place pose -> open gripper ->
retreat -> home.

Chains ``ur7e_motion``'s ``execute_primitive`` action
(``ur7e_interfaces/action/ExecutePrimitive``) with the gripper's stock
``GripperCommand`` action (``ur7e_pick_place_bringup``'s
``gripper_action_controller``) — no new capability here, this script only
sequences primitives already validated independently (see the package
READMEs). That's deliberate: task 1.7 is the *integration* proof, not a place
to introduce new motion logic.

Pick/place poses come from ``ur7e_pick_place_bringup/planning_scene.py`` —
keep the two in sync; a mismatch means the arm reaches for empty space or
into the object it should be avoiding. Both carry the same
"PLACEHOLDER, re-measure in a lab session" caveat.

Run against any tier (1 mock, 2 URSim, 3 Gazebo) — bring up
``ur7e_pick_place_bringup`` + MoveIt + ``ur7e_motion`` first, per each
package's own README.
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from ur7e_interfaces.action import ExecutePrimitive

# HOVER_HEIGHT is re-exported: perception_pick_demo imports it from here.
from pick_sequence import HOVER_HEIGHT, OPEN, Gripper, Motion, pick_place_steps  # noqa: F401

# Must match ur7e_pick_place_bringup/planning_scene.py's `pick_object` /
# `place_target` collision boxes — see that file for the "why these numbers"
# comment (PLACEHOLDER, task 1.7's own acceptance criterion: taped pose).
PICK_POSE = (0.45, -0.15, 0.08)   # table_top_z (0.0) + object height (0.08)
PLACE_POSE = (0.45, 0.20, 0.08)

# Top-down grasp only in v1 (ARCHITECTURE.md D3): gripper Z axis pointing
# straight down is a 180 deg rotation about X from the tool's rest
# orientation — the conventional "wrist down" quaternion for a UR tool frame.
GRASP_ORIENTATION = (1.0, 0.0, 0.0, 0.0)  # x, y, z, w

GRIPPER_OPEN = 0.10       # m — matches the SRDF `open` group_state (task 1.4)
GRIPPER_CLOSED = 0.0      # m — SRDF `closed` group_state; real width depends
                          # on the object once 1.1's bench setup exists
GRIPPER_MAX_EFFORT = 20.0
ACTION_TIMEOUT_SEC = 30.0


def _pose(xyz):
    p = PoseStamped()
    p.header.frame_id = "base_link"
    p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
    (p.pose.orientation.x, p.pose.orientation.y,
     p.pose.orientation.z, p.pose.orientation.w) = GRASP_ORIENTATION
    return p


def goal_fields(motion: Motion) -> dict:
    """Turn a plan step into ``ExecutePrimitive.Goal`` keyword fields."""
    fields = {"primitive": motion.primitive, "named_pose": motion.named_pose,
              "z_offset": motion.z_offset}
    if motion.xyz is not None:
        fields["target_pose"] = _pose(motion.xyz)
    return fields


class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__("pick_place_demo")
        self._motion = ActionClient(self, ExecutePrimitive, "execute_primitive")
        self._gripper = ActionClient(self, GripperCommand, "gripper_action_controller/gripper_cmd")

    def _wait(self, client, name):
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"action server '{name}' not available")

    def _run_primitive(self, **goal_fields):
        self._wait(self._motion, "execute_primitive")
        goal = ExecutePrimitive.Goal(**goal_fields)
        self.get_logger().info(f"-> {goal_fields.get('primitive')}")
        future = self._motion.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=ACTION_TIMEOUT_SEC)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"goal rejected: {goal_fields}")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=ACTION_TIMEOUT_SEC)
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f"primitive failed: {result.message}")

    def _run_gripper(self, position):
        self._wait(self._gripper, "gripper_action_controller/gripper_cmd")
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        self.get_logger().info(f"-> gripper to {position:.3f} m")
        future = self._gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=ACTION_TIMEOUT_SEC)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("gripper goal rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=ACTION_TIMEOUT_SEC)
        result = result_future.result().result
        if not result.reached_goal and not result.stalled:
            raise RuntimeError("gripper did not reach goal or stall (unexpected either way)")

    def run(self):
        # The step list lives in pick_sequence.py so the real-hardware
        # coordinator (lab_urcap_handoff.py --pick-place) runs the same plan.
        for step in pick_place_steps(PICK_POSE, PLACE_POSE):
            if isinstance(step, Gripper):
                self._run_gripper(GRIPPER_OPEN if step.action == OPEN else GRIPPER_CLOSED)
            else:
                self._run_primitive(**goal_fields(step))
        self.get_logger().info("pick-place sequence complete")


def main():
    rclpy.init()
    node = PickPlaceDemo()
    status = 0
    try:
        node.run()
    except Exception as error:  # noqa: BLE001 — top-level script, report and exit non-zero
        node.get_logger().error(f"sequence failed: {error}")
        status = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
