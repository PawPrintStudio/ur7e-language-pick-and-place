"""The RG2 in Gazebo, with the same interface as ``lab_pick.Rg2``.

Lets ``lab_pick.py --sim gazebo`` run the real-arm pipeline against the Gazebo
world (ros2 launch ur7e_perception gazebo_demo.launch.py):

* the jaws are moved through Gazebo's ``gripper_action_controller``;
* holding is the world's grasp latch (a DetachableJoint on ``pick_object``,
  ``/pick_object/attach`` and ``/detach``), because two-finger contact physics
  in Gazebo is not stable enough to carry a block.

The latch is engaged only when the jaws close with the fingertips actually at
the block. A grasp that misses reports a closed, empty gripper, exactly like the
real RG2, so the orchestrator's miss handling is exercised honestly. Grasp
quality itself is never proven here: that stays a real-arm result.
"""
import math
import re
import subprocess
import time

from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty

BLOCK_WIDTH_MM = 30.0           # pick_object is 30 x 30 x 80 mm (worlds/pick_place_table.sdf)
GRASP_RADIUS_M = 0.06           # fingertips this close to the block centre = it is between them
POSE_TOPIC = '/world/pick_place_table/dynamic_pose/info'


def block_position(timeout=5.0):
    """(x, y, z) of pick_object from Gazebo's own pose stream, or None."""
    try:
        text = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', POSE_TOPIC],
                              capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = re.search(r'name: "pick_object".*?position \{\s*x: ([-\d.e]+)\s*'
                      r'y: ([-\d.e]+)\s*z: ([-\d.e]+)', text, re.S)
    return tuple(float(v) for v in match.groups()) if match else None


class GazeboRg2:
    """Gripper action + grasp latch; widths in mm, like the RG2's URCap."""

    def __init__(self, executor, timeout=8.0):
        self.executor, self.timeout = executor, timeout
        self.node = Node('gazebo_rg2')
        self.action = ActionClient(self.node, GripperCommand,
                                   '/gripper_action_controller/gripper_cmd')
        self.attach = self.node.create_publisher(Empty, '/pick_object/attach', 10)
        self.detach = self.node.create_publisher(Empty, '/pick_object/detach', 10)
        if not self.action.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('Gazebo gripper controller not available '
                               '(is gazebo_demo.launch.py running?)')
        self.width, self.holding = 80.0, False

    def state(self):
        status = 2 if self.holding else 0
        return {'width': self.width, 'busy': False, 'status': status,
                'grip_detected': self.holding}

    def _command(self, width_mm, force_n):
        goal = GripperCommand.Goal()
        goal.command.position = max(0.0, min(0.110, width_mm / 1000.0))
        goal.command.max_effort = float(force_n)
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('Gazebo gripper refused the goal')
        result = handle.get_result_async()
        # The simulated fingers may stall short of the target (known, see
        # src/ur7e_gazebo/README.md); the latch, not finger contact, holds.
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=self.timeout)

    def _publish(self, publisher):
        for _ in range(3):  # a fresh publisher may not be matched yet
            publisher.publish(Empty())
            rclpy.spin_once(self.node, timeout_sec=0.1)
        time.sleep(0.3)

    def move(self, width_mm, force_n, timeout=8.0):
        """Command a width; latch or unlatch the block. Return the state."""
        closing = width_mm < self.width
        if not closing and self.holding:
            self._publish(self.detach)
            self.holding = False
        self._command(width_mm, force_n)
        if closing:
            tips = self.executor.fk(self.executor.fresh(settle_s=0.3)).position
            block = block_position()
            near = (block is not None
                    and math.dist((tips.x, tips.y, tips.z), block) < GRASP_RADIUS_M)
            if near:
                self._publish(self.attach)
                self.holding = True
                self.width = BLOCK_WIDTH_MM
            else:
                self.width = width_mm   # closed on nothing: a miss
        else:
            self.width = width_mm
        state = self.state()
        state['grip_detected'] = self.holding
        return state

    def destroy(self):
        self.node.destroy_node()
