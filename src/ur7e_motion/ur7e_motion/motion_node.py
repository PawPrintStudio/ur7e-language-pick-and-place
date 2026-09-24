#!/usr/bin/env python3
"""motion_node (task 1.5): an ``ExecutePrimitive`` action server over
``pymoveit2``. Each goal runs exactly one primitive from
``primitives.py`` — the demo script (``scripts/pick_place_demo.py``) chains
several goals to build the scripted pick sequence (task 1.7).

Robot/group configuration (joint names, base link, end effector, group name)
is discovered from the *running* move_group node's SRDF at startup, not
hardcoded here — see ``primitives.py``'s docstring for why.
"""
from threading import Thread

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robot_description import RobotDescription

from ur7e_interfaces.action import ExecutePrimitive
from ur7e_motion import primitives
from ur7e_motion.primitives import PrimitiveError

DEFAULT_TIMEOUT_SEC = 30.0


class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_node")
        self.declare_parameter("group_name", "ur_onrobot_manipulator")
        self.declare_parameter("timeout_sec", DEFAULT_TIMEOUT_SEC)
        self._group_name_param = self.get_parameter("group_name").value
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._callback_group = ReentrantCallbackGroup()

    def setup(self):
        """Discover the robot config and stand up MoveIt2 + the action server.

        Split from ``__init__`` because discovery calls
        ``move_group/get_parameters`` and blocks on the response — it needs
        this node already being spun by an executor (started in ``main()``),
        or the request's own response callback never runs. Same reason
        pymoveit2's own examples spin in a background thread before building
        ``RobotConfiguration``/``MoveIt2``.
        """
        self.get_logger().info("discovering robot configuration from move_group...")
        self._description = RobotDescription.from_node(self, remote_node_name="move_group")
        kwargs = self._description.moveit2_kwargs(self._group_name_param)
        self._group_name = kwargs["group_name"]

        self._moveit2 = MoveIt2(node=self, callback_group=self._callback_group, **kwargs)

        self._server = ActionServer(
            self,
            ExecutePrimitive,
            "execute_primitive",
            execute_callback=self._execute,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"motion_node ready (group '{self._group_name}', "
            f"joints {kwargs['joint_names']})"
        )

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = ExecutePrimitive.Result()
        feedback = ExecutePrimitive.Feedback()
        feedback.stage = f"running {goal.primitive}"
        goal_handle.publish_feedback(feedback)

        try:
            self._dispatch(goal)
        except PrimitiveError as error:
            self.get_logger().error(f"{goal.primitive} failed: {error}")
            goal_handle.abort()
            result.success = False
            result.message = str(error)
            return result
        except Exception as error:  # noqa: BLE001 — report, don't crash the server
            self.get_logger().error(f"{goal.primitive} raised {type(error).__name__}: {error}")
            goal_handle.abort()
            result.success = False
            result.message = f"unexpected error: {error}"
            return result

        goal_handle.succeed()
        result.success = True
        result.message = f"{goal.primitive} completed"
        return result

    def _dispatch(self, goal):
        timeout = self._timeout_sec
        p = goal.primitive
        if p == ExecutePrimitive.Goal.PRIMITIVE_GOTO_NAMED:
            primitives.goto_named(
                self._moveit2, self._description, self._group_name, goal.named_pose, timeout
            )
        elif p == ExecutePrimitive.Goal.PRIMITIVE_APPROACH_ABOVE:
            primitives.approach_above(self._moveit2, goal.target_pose, goal.z_offset, timeout)
        elif p == ExecutePrimitive.Goal.PRIMITIVE_CARTESIAN_DESCEND:
            primitives.cartesian_descend(self._moveit2, goal.z_offset, timeout)
        elif p == ExecutePrimitive.Goal.PRIMITIVE_CARTESIAN_LIFT:
            primitives.cartesian_lift(self._moveit2, goal.z_offset, timeout)
        elif p == ExecutePrimitive.Goal.PRIMITIVE_RETREAT:
            primitives.retreat(self._moveit2, goal.z_offset, timeout)
        else:
            raise PrimitiveError(f"unknown primitive '{p}'")


def main():
    rclpy.init()
    node = MotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        node.setup()
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
