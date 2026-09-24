#!/usr/bin/env python3
"""safety_monitor (task 1.6): watches the driver's safety/program-running
status and reacts to a protective stop — the one failure mode every other
Phase 1 node assumes someone else handles.

On a transition INTO ``PROTECTIVE_STOP`` (or any of the other non-NORMAL
`SafetyMode`\\ s that mean the robot stopped moving on its own): cancel any
goal ``motion_node`` has in flight and publish a status message other nodes
(the demo script, eventually the orchestrator in Phase 4) can watch instead
of discovering the stop the hard way — a hung action.

Recovery is a service, not automatic: ``unlock_protective_stop`` only clears
the safety system's lock. It does **not** restore control — the trajectory
controller still holds its stale pre-stop command (RUNBOOK "URSim rehearsal
session", 2026-09-17: this is exactly the wedge that made a rehearsal
session's naive recovery loop forever). The service reports the two manual
steps a person still has to do: press Play (or headless-mode equivalent),
then restart the driver process. A ROS node cannot restart its own driver's
process from inside — see docs/SIMULATION.md's tier-2 recovery drill, which
this service is the sim/lab-tested reference for.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import SafetyMode

from control_msgs.action import FollowJointTrajectory, GripperCommand
from ur7e_interfaces.action import ExecutePrimitive

# SafetyMode values that mean "the robot is not going to keep executing the
# current trajectory" — everything except NORMAL and the informational
# REDUCED (a valid, still-moving operating mode, e.g. speed-limited).
STOPPED_SAFETY_MODES = {
    SafetyMode.PROTECTIVE_STOP,
    SafetyMode.SAFEGUARD_STOP,
    SafetyMode.SYSTEM_EMERGENCY_STOP,
    SafetyMode.ROBOT_EMERGENCY_STOP,
    SafetyMode.VIOLATION,
    SafetyMode.FAULT,
    SafetyMode.AUTOMATIC_MODE_SAFEGUARD_STOP,
    SafetyMode.SYSTEM_THREE_POSITION_ENABLING_STOP,
}

# Action servers this node may need to cancel goals on when a stop happens.
# Cancelling is best-effort: a server that isn't up (e.g. motion_node not
# running yet) is simply skipped.
CANCELLABLE_ACTIONS = {
    "motion_node": (ExecutePrimitive, "execute_primitive"),
    "gripper": (GripperCommand, "gripper_action_controller/gripper_cmd"),
    "arm_trajectory": (FollowJointTrajectory, "scaled_joint_trajectory_controller/follow_joint_trajectory"),
}


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")
        self._last_mode = None
        self._status_pub = self.create_publisher(String, "/ur7e/safety_event", 10)

        # io_and_status_controller publishes safety_mode as transient-local
        # (latched) so a late subscriber still gets the current state.
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                                  self._on_safety_mode, qos)
        self.create_subscription(Bool, "/io_and_status_controller/robot_program_running",
                                  self._on_program_running, 10)

        cb = ReentrantCallbackGroup()
        self._action_clients = {
            name: ActionClient(self, action_type, action_name, callback_group=cb)
            for name, (action_type, action_name) in CANCELLABLE_ACTIONS.items()
        }
        self._unlock_client = self.create_client(
            Trigger, "/dashboard_client/unlock_protective_stop", callback_group=cb
        )

        self.create_service(Trigger, "~/recover", self._recover, callback_group=cb)
        self.get_logger().info("safety_monitor ready — watching safety_mode")

    def _on_safety_mode(self, msg):
        mode = msg.mode
        if mode == self._last_mode:
            return
        stopped_now = mode in STOPPED_SAFETY_MODES
        was_stopped = self._last_mode in STOPPED_SAFETY_MODES
        self._last_mode = mode
        mode_name = _mode_name(mode)

        if stopped_now and not was_stopped:
            self.get_logger().warning(f"safety_mode -> {mode_name}: cancelling in-flight goals")
            self._cancel_all()
            self._publish_event(f"STOPPED: {mode_name}")
        elif not stopped_now and was_stopped:
            self._publish_event(f"CLEARED: {mode_name} (recovery service still required)")
        else:
            self._publish_event(f"safety_mode: {mode_name}")

    def _on_program_running(self, msg):
        if not msg.data:
            self._publish_event("robot program not running")

    def _cancel_all(self):
        for name, client in self._action_clients.items():
            if not client.server_is_ready():
                continue
            try:
                client.cancel_all_goals()
                self.get_logger().info(f"cancelled goals on '{name}'")
            except Exception as error:  # noqa: BLE001 — best-effort, log and continue
                self.get_logger().warning(f"could not cancel '{name}': {error}")

    def _publish_event(self, text):
        self._status_pub.publish(String(data=text))

    def _recover(self, request, response):
        if not self._unlock_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "dashboard_client not available (mock hardware has no dashboard)"
            return response
        future = self._unlock_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or not future.result() or not future.result().success:
            response.success = False
            response.message = "unlock_protective_stop failed or timed out"
            return response
        response.success = True
        response.message = (
            "Unlocked. Two manual steps remain, in order (RUNBOOK, "
            "docs/SIMULATION.md tier-2 recovery drill): (1) press Play on the "
            "pendant / headless-mode equivalent, (2) RESTART the driver "
            "process — reconnecting without a restart wedges the arm, the "
            "trajectory controller holds its stale pre-stop command."
        )
        return response


def _mode_name(mode):
    for name in dir(SafetyMode):
        if name.isupper() and getattr(SafetyMode, name) == mode:
            return name
    return f"UNKNOWN({mode})"


def main():
    rclpy.init()
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
