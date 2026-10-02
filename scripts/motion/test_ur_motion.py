"""A cancelled ROS action must never be reported as a successful demo."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import ur_motion


@pytest.mark.parametrize('status, code, expected',
                         [(4, 0, True), (5, 0, False), (6, 0, False), (4, -4, False)])
def test_terminal_status_controls_success(monkeypatch, status, code, expected):
    monkeypatch.setattr(ur_motion.rclpy, 'spin_until_future_complete',
                        lambda *args, **kwargs: None)
    result = SimpleNamespace(status=status, result=SimpleNamespace(error_code=code))
    goal = SimpleNamespace(accepted=True, get_result_async=lambda: Mock(result=lambda: result))
    client = Mock()
    client.wait_for_server.return_value = True
    client.send_goal_async.return_value = Mock(result=lambda: goal)
    anchor = dict.fromkeys(ur_motion.JOINTS, 0.0)
    node = SimpleNamespace(_start=anchor, _ac=client, _check=Mock(), get_logger=lambda: Mock())
    assert ur_motion.MotionClient.run(node, [(anchor, 1.0)]) is expected
    sent = client.send_goal_async.call_args.args[0].trajectory
    assert len(sent.points) == 2
    assert sent.points[0].time_from_start.sec == 0


def test_a_goal_that_never_finishes_is_cancelled_and_reported_failed(monkeypatch):
    """Seen in the lab 2026-10-02: a lift stalled at a joint limit for two minutes."""
    monkeypatch.setattr(ur_motion.rclpy, 'spin_until_future_complete',
                        lambda *args, **kwargs: None)
    pending = Mock()
    pending.done.return_value = False
    goal = Mock(accepted=True)
    goal.get_result_async.return_value = pending
    client = Mock()
    client.wait_for_server.return_value = True
    client.send_goal_async.return_value = Mock(result=lambda: goal)
    anchor = dict.fromkeys(ur_motion.JOINTS, 0.0)
    node = SimpleNamespace(_start=anchor, _ac=client, _check=Mock(), get_logger=lambda: Mock())
    assert ur_motion.MotionClient.run(node, [(anchor, 1.0)], timeout=0.1) is False
    goal.cancel_goal_async.assert_called_once()
