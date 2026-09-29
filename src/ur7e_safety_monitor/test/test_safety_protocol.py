"""Exercise cancellation and recovery over ROS without connecting to hardware."""

import threading
import time

import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import SafetyMode

from ur7e_safety_monitor.safety_monitor_node import SafetyMonitor


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("ROS operation timed out")


@pytest.fixture
def graph():
    # Never let these test servers/cancel requests join the lab's domain 42.
    rclpy.init(domain_id=143)
    monitor = SafetyMonitor()
    peer = rclpy.create_node("safety_protocol_test")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(monitor)
    executor.add_node(peer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield monitor, peer
    executor.shutdown(timeout_sec=5)
    thread.join(timeout=5)
    peer.destroy_node()
    monitor.destroy_node()
    rclpy.shutdown()


def test_protective_stop_cancels_real_action_goal(graph):
    """The monitor must cancel goals submitted by another action client."""
    monitor, peer = graph
    released = threading.Event()

    def execute(goal):
        deadline = time.monotonic() + 8
        while not goal.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.01)
        if goal.is_cancel_requested:
            goal.canceled()
            released.set()
        else:
            goal.abort()
        return FollowJointTrajectory.Result()

    action = "scaled_joint_trajectory_controller/follow_joint_trajectory"
    server = ActionServer(
        peer, FollowJointTrajectory, action, execute,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup(),
    )
    client = ActionClient(peer, FollowJointTrajectory, action)
    try:
        assert client.wait_for_server(timeout_sec=5)
        sent = client.send_goal_async(FollowJointTrajectory.Goal())
        wait_for(sent.done)
        goal = sent.result()
        assert goal.accepted
        wait_for(monitor._action_clients["arm_trajectory"].service_is_ready)
        monitor._on_safety_mode(SafetyMode(mode=SafetyMode.NORMAL))
        monitor._on_safety_mode(SafetyMode(mode=SafetyMode.PROTECTIVE_STOP))
        assert released.wait(timeout=5), "The active goal was not cancelled"
        result = goal.get_result_async()
        wait_for(result.done)
        assert result.result().status == 5  # STATUS_CANCELED
    finally:
        client.destroy()
        server.destroy()


def test_recovery_response_preserves_manual_restart_order(graph):
    """A successful unlock returns without recursively spinning the node."""
    monitor, peer = graph

    def unlock(request, response):
        response.success = True
        return response

    service = peer.create_service(
        Trigger, "/dashboard_client/unlock_protective_stop", unlock
    )
    client = peer.create_client(Trigger, "/safety_monitor/recover")
    try:
        assert client.wait_for_service(timeout_sec=5)
        assert monitor._unlock_client.wait_for_service(timeout_sec=5)
        future = client.call_async(Trigger.Request())
        wait_for(future.done)
        response = future.result()
        assert response.success
        assert response.message.index("RESTART") < response.message.index("press Play")
        assert "manual steps remain" in response.message
    finally:
        peer.destroy_client(client)
        peer.destroy_service(service)
