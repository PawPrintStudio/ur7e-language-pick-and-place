"""Real ROS services/topics with a fake pendant; no robot or motion hardware.

Run in the development image with --network none and ROS_DOMAIN_ID=86.
"""
import os
import socket
import threading
import time
from types import SimpleNamespace
import unittest

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import SafetyMode
from ur7e_interfaces.action import ExecutePrimitive

from lab_urcap_handoff import ARM_JOINTS, Coordinator
from urcap_handoff_protocol import Settings, Wire, run_session


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_handoff_robot")
        self.running = True
        self.reject = False
        self.fault_on_grip = False
        self.reject_motion = False
        self.mode = SafetyMode.NORMAL
        self.handbacks = 0
        self.stops = 0
        self.poses = []
        self.ready_at = 0.0
        self.inactive_checks = 0
        self.publish_joints = True
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.program_pub = self.create_publisher(Bool, "/io_and_status_controller/robot_program_running", qos)
        self.safety_pub = self.create_publisher(SafetyMode, "/io_and_status_controller/safety_mode", qos)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.speed_pub = self.create_publisher(Float64, "/speed_scaling_state_broadcaster/speed_scaling", 10)
        self.create_service(Trigger, "/io_and_status_controller/hand_back_control", self.handback)
        self.create_service(Trigger, "/dashboard_client/stop", self.stop)
        self.create_service(ListControllers, "/controller_manager/list_controllers", self.controllers)
        self.action = ActionServer(self, ExecutePrimitive, "/execute_primitive", self.motion)
        self.create_timer(0.02, self.publish)

    def publish(self):
        self.program_pub.publish(Bool(data=self.running))
        self.safety_pub.publish(SafetyMode(mode=self.mode))
        self.speed_pub.publish(Float64(data=10.0 if self.running else 0.0))
        if self.publish_joints:
            self.joint_pub.publish(JointState(name=list(ARM_JOINTS), position=[0.0]*6, velocity=[0.0]*6))

    def handback(self, request, response):
        self.handbacks += 1
        response.success = not self.reject
        response.message = "mock rejection" if self.reject else "ok"
        if response.success:
            self.running = False
        return response

    def stop(self, request, response):
        self.stops += 1
        self.running = False
        response.success = True
        return response

    def controllers(self, request, response):
        active = self.running and time.monotonic() >= self.ready_at
        if not active:
            self.inactive_checks += 1
        response.controller = [ControllerState(name="scaled_joint_trajectory_controller",
                                                state="active" if active else "inactive")]
        return response

    def motion(self, handle):
        self.poses.append(handle.request.named_pose)
        result = ExecutePrimitive.Result()
        result.success = not self.reject_motion
        result.message = "mock motion"
        if self.reject_motion:
            handle.abort()
        else:
            handle.succeed()
        return result


class RosHandoffTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get("ROS_DOMAIN_ID") != "86":
            raise RuntimeError("Set ROS_DOMAIN_ID=86 for these isolated tests")
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.fake = FakeRobot()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.fake)
        self.worker = threading.Thread(target=self.executor.spin)
        self.worker.start()
        self.node = Coordinator(SimpleNamespace(max_speed_percent=10.0,
                                               before_close_pose=None, before_open_pose=None))
        deadline = time.monotonic() + 5
        while (self.node.joints is None or self.node.running is None
               or self.node.safety is None or self.node.speed is None
               or not self.node.handback.service_is_ready()
               or not self.node.controllers.service_is_ready()
               or not self.node.motion.server_is_ready()):
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.assertLess(time.monotonic(), deadline)
        self.grants = []

    def tearDown(self):
        self.node.motion.destroy()
        self.node.destroy_node()
        self.executor.shutdown()
        self.worker.join(3)
        self.fake.action.destroy()
        self.fake.destroy_node()

    def cycle(self):
        local, peer = socket.socketpair()
        settings = Settings()
        failures = []

        def pendant():
            try:
                with peer:
                    wire = Wire(peer, lambda: None, timeout=2)
                    wire.send(*settings.hello, 700)
                    nonce, = wire.receive(1)
                    for phase, target in ((1, 590), (2, 700)):
                        deadline = time.monotonic() + 3
                        while self.fake.running:
                            if time.monotonic() > deadline:
                                return
                            time.sleep(0.005)
                        wire.send(nonce, -phase)
                        self.grants.append(wire.receive(2))
                        if self.fake.fault_on_grip:
                            self.fake.mode = SafetyMode.PROTECTIVE_STOP
                            time.sleep(0.1)
                        wire.send(nonce, phase, target, 0, 0)
                        wire.receive(2)
                        self.fake.ready_at = time.monotonic() + 0.2
                        self.fake.running = True
            except (ConnectionError, OSError, TimeoutError):
                pass
            except BaseException as error:
                failures.append(error)

        worker = threading.Thread(target=pendant)
        worker.start()
        try:
            with local:
                return run_session(local, settings, self.node, timeout=2)
        finally:
            worker.join(4)
            self.assertFalse(worker.is_alive())
            if failures:
                raise failures[0]

    def test_two_handoffs_wait_for_controller_reactivation(self):
        result = self.cycle()
        self.assertEqual(len(result), 2)
        self.assertEqual(self.fake.handbacks, 2)
        self.assertGreater(self.fake.inactive_checks, 0)
        self.assertTrue(self.node.running)

    def test_denied_handback_never_grants_grip(self):
        self.fake.reject = True
        with self.assertRaisesRegex(RuntimeError, "Handback failed"):
            self.cycle()
        self.assertEqual(self.grants, [])
        self.assertEqual(self.fake.handbacks, 1)

    def test_safety_stop_prevents_next_phase(self):
        self.fake.fault_on_grip = True
        with self.assertRaisesRegex(RuntimeError, "Safety"):
            self.cycle()
        self.assertEqual(len(self.grants), 1)
        self.assertEqual(self.fake.handbacks, 1)
        self.node.stop_after_error()
        self.assertEqual(self.fake.stops, 1)

    def test_optional_named_moves_complete_before_handoffs(self):
        self.node.args.before_close_pose = "accepted_grasp_pose"
        self.node.args.before_open_pose = "accepted_release_pose"
        self.assertEqual(len(self.cycle()), 2)
        self.assertEqual(self.fake.poses, ["accepted_grasp_pose", "accepted_release_pose"])

    def test_failed_named_move_never_hands_back(self):
        self.fake.reject_motion = True
        self.node.args.before_close_pose = "rejected_pose"
        with self.assertRaisesRegex(RuntimeError, "Named motion failed"):
            self.cycle()
        self.assertEqual(self.fake.handbacks, 0)
        self.assertEqual(self.grants, [])

    def test_unexpected_external_control_exit_fails_closed(self):
        self.node.wait_ready()
        self.fake.running = False
        deadline = time.monotonic() + 2
        with self.assertRaisesRegex(RuntimeError, "Unexpected External Control"):
            while time.monotonic() < deadline:
                self.node.check()
        self.assertEqual(self.fake.handbacks, 0)

    def test_stale_joint_telemetry_is_rejected(self):
        self.node.wait_ready()
        self.fake.publish_joints = False
        deadline = time.monotonic() + 2
        with self.assertRaisesRegex(RuntimeError, "telemetry gate"):
            while time.monotonic() < deadline:
                self.node.check()
        self.assertEqual(self.fake.handbacks, 0)


if __name__ == "__main__":
    unittest.main()
