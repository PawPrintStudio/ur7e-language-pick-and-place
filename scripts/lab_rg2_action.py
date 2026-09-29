#!/usr/bin/env python3
"""Supervised native RG2 ROS action bench adapter. Arm program must be stopped.

Same action name/units as the project's gripper controller: metres and newtons.
This adapter intentionally limits targets to 40..80 mm, changes to <=20 mm,
and force to 0<force<=10 N. Do not run alongside another gripper controller.
Measured effort is unavailable and is reported as NaN, never as force feedback.
"""
import math
import threading
import xmlrpc.client

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from lab_rg2_native import (
    MotionCanceled, TimeoutTransport, dashboard_gate, identify, move, validate_state,
)


class GripperBench(Node):
    def __init__(self):
        super().__init__("lab_rg2_native_action")
        self.host = self.declare_parameter("robot_ip", "192.168.56.101").value
        self.tool = self.declare_parameter("tool_index", 2).value
        self.serial = self.declare_parameter("serial", "1000042561").value
        self.lock = threading.Lock()
        self.reserved = False
        self.server = ActionServer(
            self, GripperCommand, "/gripper_action_controller/gripper_cmd",
            execute_callback=self.execute, goal_callback=self.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Bench adapter ready; arm must remain stopped; effort feedback unavailable")

    def rpc(self):
        return xmlrpc.client.ServerProxy(
            f"http://{self.host}:41414/", transport=TimeoutTransport())

    def goal(self, request):
        width, force = request.command.position, request.command.max_effort
        if not (math.isfinite(width) and math.isfinite(force)
                and 0.040 <= width <= 0.080 and 0 < force <= 10):
            return GoalResponse.REJECT
        with self.lock:
            if self.reserved:
                return GoalResponse.REJECT
            self.reserved = True
        return GoalResponse.ACCEPT

    def execute(self, handle):
        result = GripperCommand.Result()
        result.position = float("nan")
        result.effort = float("nan")
        rpc = self.rpc()

        def feedback(state):
            result.position = float(state["width"] / 1000)
            message = GripperCommand.Feedback()
            message.position = result.position
            message.effort = float("nan")
            handle.publish_feedback(message)

        try:
            dashboard_gate(self.host)
            identify(rpc, self.tool, self.serial)
            feedback(validate_state(rpc.rg_get_all_variables(self.tool)))
            state = move(rpc, self.host, self.tool,
                         handle.request.command.position * 1000,
                         handle.request.command.max_effort,
                         feedback=feedback, canceled=lambda: handle.is_cancel_requested)
            result.position = float(state["width"] / 1000)
            result.reached_goal = True
            handle.succeed()
        except MotionCanceled:
            try:
                result.position = float(validate_state(rpc.rg_get_all_variables(self.tool))["width"] / 1000)
                handle.canceled()
            except Exception as error:
                self.get_logger().error(f"Post-cancel feedback invalid: {error}")
                handle.abort()
        except Exception as error:
            self.get_logger().error(str(error))
            handle.abort()
        finally:
            with self.lock:
                self.reserved = False
        return result


def main():
    rclpy.init()
    node = GripperBench()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
