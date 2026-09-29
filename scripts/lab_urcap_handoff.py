#!/usr/bin/env python3
"""Supervised ROS -> pendant RG Grip -> ROS acceptance coordinator.

No motion by default. Requires the exact pendant tree in docs/URCAP_HANDOFF.md.
``--pick-place`` runs the same step plan as the simulation demo
(pick_sequence.py), handing the gripper steps to the pendant.
Never uploads URScript, bypasses OnRobot, unlocks a stop, or starts a program.
"""
import argparse
import json
import math
import select
import socket
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import SafetyMode
from ur7e_interfaces.action import ExecutePrimitive

from pick_place_demo import goal_fields
from pick_sequence import GOTO_NAMED, Motion, pick_place_steps, split_for_handoff
from urcap_handoff_protocol import Settings, run_session

ARM_JOINTS = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)


class Coordinator(Node):
    def __init__(self, args):
        super().__init__("lab_urcap_handoff")
        self.args = args
        self.running = None
        self.safety = None
        self.joints = None
        self.joint_time = 0.0
        self.speed = None
        self.speed_time = 0.0
        self.expected_running = None
        self.stationary = True
        self.anchor = None
        self.active_goals = set()
        self.goal = None
        # (before_close, before_open, after_open) motions for --pick-place.
        self.segments = getattr(args, "segments", None)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/io_and_status_controller/robot_program_running",
                                 lambda m: setattr(self, "running", m.data), qos)
        self.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                                 lambda m: setattr(self, "safety", m.mode), qos)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Float64, "/speed_scaling_state_broadcaster/speed_scaling",
                                 self.on_speed, 10)
        self.create_subscription(GoalStatusArray,
                                 "/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/status",
                                 self.on_status, qos)
        self.handback = self.create_client(Trigger, "/io_and_status_controller/hand_back_control")
        self.stop = self.create_client(Trigger, "/dashboard_client/stop")
        self.controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.motion = ActionClient(self, ExecutePrimitive, "/execute_primitive")

    def on_joints(self, msg):
        self.joints = msg
        self.joint_time = time.monotonic()

    def on_speed(self, msg):
        self.speed = msg.data
        self.speed_time = time.monotonic()

    def on_status(self, msg):
        self.active_goals = {bytes(x.goal_info.goal_id.uuid) for x in msg.status_list
                             if x.status in (GoalStatus.STATUS_ACCEPTED,
                                             GoalStatus.STATUS_EXECUTING,
                                             GoalStatus.STATUS_CANCELING)}

    def report(self, event):
        print(json.dumps({"time": time.time(), **event}), flush=True)

    def check(self):
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown")
        rclpy.spin_once(self, timeout_sec=0.005)
        now = time.monotonic()
        if (self.safety != SafetyMode.NORMAL or self.joints is None
                or now - self.joint_time > 0.5
                or self.speed is None or not math.isfinite(self.speed)
                or not 0 <= self.speed <= self.args.max_speed_percent
                or now - self.speed_time > 0.5):
            raise RuntimeError("Safety/speed/telemetry gate failed; no automatic continuation")
        if self.expected_running is not None and self.running != self.expected_running:
            raise RuntimeError("Unexpected External Control transition")
        msg = self.joints
        if len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
            raise RuntimeError("Incomplete joint telemetry")
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if not all(j in positions and math.isfinite(positions[j])
                   and math.isfinite(velocities[j]) for j in ARM_JOINTS):
            raise RuntimeError("Missing/non-finite arm joint feedback")
        if self.stationary:
            if max(abs(velocities[j]) for j in ARM_JOINTS) > 0.005 or self.active_goals:
                raise RuntimeError("Arm moving or trajectory active during gripper handoff")
            current = [positions[j] for j in ARM_JOINTS]
            if self.anchor is None:
                self.anchor = current
            if max(abs(a - b) for a, b in zip(current, self.anchor)) > 0.003:
                raise RuntimeError("Arm position drift during gripper handoff")

    def wait_future(self, future, seconds=5):
        deadline = time.monotonic() + seconds
        while not future.done():
            self.check()
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS response timed out; will not retry")
        self.check()
        result = future.result()
        if result is None:
            raise RuntimeError("Empty ROS response")
        return result

    def wait_running(self, expected):
        self.expected_running = None
        deadline = time.monotonic() + 10
        while True:
            self.check()
            if self.running == expected:
                self.expected_running = expected
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f"External Control did not become {expected}")

    def wait_ready(self):
        self.wait_running(True)
        deadline = time.monotonic() + 10
        while True:
            self.check()
            if not self.controllers.service_is_ready():
                raise RuntimeError("Controller manager unavailable")
            result = self.wait_future(self.controllers.call_async(ListControllers.Request()))
            if any(c.name == "scaled_joint_trajectory_controller" and c.state == "active"
                   for c in result.controller):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Arm controller did not reactivate after handoff")
        # Hold a steady arm state before handing off or reporting readiness.
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            self.check()

    def hand_back(self):
        self.check()
        if not self.handback.service_is_ready():
            raise RuntimeError("Documented hand_back_control service unavailable")
        self.report({"event": "hand_back_requested"})
        self.expected_running = None
        response = self.wait_future(self.handback.call_async(Trigger.Request()))
        if not response.success:
            raise RuntimeError(f"Handback failed: {response.message}")

    def wait_left(self):
        self.wait_running(False)

    def phase_motions(self, phase):
        if self.segments is not None:
            return self.segments[phase - 1]
        name = self.args.before_close_pose if phase == 1 else self.args.before_open_pose
        return [Motion(GOTO_NAMED, named_pose=name)] if name else []

    def move(self, phase):
        self.run_motions(self.phase_motions(phase))

    def run_motions(self, motions):
        if not motions:
            return
        self.check()
        if self.speed <= 0 or not self.motion.server_is_ready():
            raise RuntimeError("Motion server unavailable or speed zero")
        self.stationary = False
        self.anchor = None
        for motion in motions:
            self.run_one(motion)
        # Wait for the trajectory's terminal status/settled joint sample to
        # arrive before establishing the handoff's stationary reference.
        deadline = time.monotonic() + 3
        while True:
            self.check()
            speeds = dict(zip(self.joints.name, self.joints.velocity))
            if not self.active_goals and max(abs(speeds[j]) for j in ARM_JOINTS) <= 0.005:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Arm failed to settle after motion")
        self.stationary = True
        self.check()

    def run_one(self, motion):
        label = motion.named_pose or motion.primitive
        self.report({"event": "motion_requested", "primitive": motion.primitive,
                     "pose": motion.named_pose or None,
                     "xyz": list(motion.xyz) if motion.xyz else None})
        self.goal = self.wait_future(
            self.motion.send_goal_async(ExecutePrimitive.Goal(**goal_fields(motion))))
        if not self.goal.accepted:
            raise RuntimeError(f"Motion {label} rejected")
        result = self.wait_future(self.goal.get_result_async(), 60)
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            raise RuntimeError(f"Motion {label} failed: {result.result.message}")
        self.goal = None

    def stop_after_error(self):
        # Closing the peer socket also makes the pendant gate halt. Stop is
        # best effort, explicitly logged; no unlock/play/resend is performed.
        try:
            if self.goal is not None and self.goal.accepted:
                self.goal.cancel_goal_async()
            if not self.stop.wait_for_service(timeout_sec=1):
                raise RuntimeError("Dashboard stop service unavailable")
            future = self.stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3)
            if not future.done() or not future.result().success:
                raise RuntimeError("Dashboard stop not acknowledged")
            self.report({"event": "program_stop_acknowledged"})
        except Exception as error:
            self.report({"event": "program_stop_unconfirmed", "error": str(error),
                         "operator_action": "Stop at pendant; inspect before restarting"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-gripper", action="store_true")
    parser.add_argument("--supervised", action="store_true")
    parser.add_argument("--bind", default="192.168.56.1")
    parser.add_argument("--robot-ip", default="192.168.56.101")
    parser.add_argument("--port", type=int, default=50010)
    parser.add_argument("--close-mm", type=float, default=59.0)
    parser.add_argument("--open-mm", type=float, default=70.0)
    parser.add_argument("--force-n", type=float, default=5.0)
    parser.add_argument("--expect-object", action="store_true")
    parser.add_argument("--max-speed-percent", type=float, choices=[10.0, 50.0], default=10.0)
    parser.add_argument("--before-close-pose")
    parser.add_argument("--before-open-pose")
    parser.add_argument("--validated-poses", action="store_true")
    parser.add_argument("--pick-place", action="store_true",
                        help="run the full pick_sequence.py plan (needs --pick-xyz/--place-xyz)")
    parser.add_argument("--pick-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--place-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--hover-m", type=float, default=0.25)
    args = parser.parse_args()
    if not args.execute_gripper or not args.supervised:
        parser.error("requires --execute-gripper --supervised; see docs/URCAP_HANDOFF.md")
    if (bool(args.before_close_pose) != bool(args.before_open_pose)
            or (args.before_close_pose and not args.validated_poses)):
        parser.error("both named poses require --validated-poses after physical TCP/pose acceptance")
    args.segments = None
    if args.pick_place:
        if args.before_close_pose:
            parser.error("--pick-place replaces --before-close-pose/--before-open-pose")
        if not (args.pick_xyz and args.place_xyz and args.validated_poses):
            parser.error("--pick-place requires measured --pick-xyz and --place-xyz "
                         "plus --validated-poses; no default physical poses exist")
        try:
            args.segments = split_for_handoff(
                pick_place_steps(tuple(args.pick_xyz), tuple(args.place_xyz), args.hover_m))
        except ValueError as error:
            parser.error(str(error))
    elif args.pick_xyz or args.place_xyz:
        parser.error("--pick-xyz/--place-xyz are only used with --pick-place")
    settings = Settings(close_mm=args.close_mm, open_mm=args.open_mm,
                        force_n=args.force_n, expect_object=args.expect_object)
    rclpy.init(args=[])
    node = Coordinator(args)
    connected = False
    try:
        deadline = time.monotonic() + 10
        while node.joints is None or node.safety is None or node.speed is None or node.running is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= deadline:
                raise TimeoutError("Driver telemetry unavailable")
        node.check()
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.bind, args.port))
            server.listen(1)
            node.report({"event": "awaiting_pendant_play", "bind": args.bind, "port": args.port})
            deadline = time.monotonic() + 120
            while not select.select([server], [], [], 0.02)[0]:
                node.check()
                if time.monotonic() >= deadline:
                    raise TimeoutError("No pendant connection within 120 seconds")
            connection, peer = server.accept()
            with connection:
                if peer[0] != args.robot_ip:
                    raise RuntimeError("Connection peer does not match configured robot")
                connected = True
                result = run_session(connection, settings, node)
                # The pendant's final External Control node keeps ROS in
                # control, so the retreat/return-home segment runs here.
                if node.segments is not None:
                    node.run_motions(node.segments[2])
                node.report({"event": "handoff_sequence_complete", "results": result})
        return 0
    except (Exception, KeyboardInterrupt) as error:
        node.report({"event": "handoff_failed", "error": str(error)})
        if connected:
            node.stop_after_error()
        return 1
    finally:
        node.motion.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
