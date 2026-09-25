"""Prevent partial Cartesian plans and false endpoint success."""
from copy import deepcopy
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from ur7e_motion.primitives import PrimitiveError, approach_above, cartesian_lift


def pose(z):
    """Build a metric pose in the robot base frame."""
    result = PoseStamped()
    result.header.frame_id = 'base_link'
    result.pose.position.z = z
    result.pose.orientation.w = 1.0
    return result


def test_partial_plan_rejected():
    """Require all waypoints before a Cartesian trajectory is executed."""
    moveit = Mock()
    moveit.compute_fk.return_value = pose(.1)
    moveit.move_to_pose.return_value = False
    with pytest.raises(PrimitiveError):
        cartesian_lift(moveit, .1, 5)
    assert moveit.move_to_pose.call_args.kwargs['cartesian_fraction_threshold'] == 1.0
    moveit.wait_until_executed.assert_not_called()


def test_success_status_with_wrong_endpoint_rejected():
    """A controller success alone must not establish arrival."""
    moveit = Mock()
    moveit.compute_fk.return_value = pose(.1)
    with pytest.raises(PrimitiveError, match='endpoint mismatch'):
        cartesian_lift(moveit, .1, 5)


def test_approach_preserves_input_and_routes_in_joint_space():
    """Changing the hover height must not mutate the detected grasp pose."""
    moveit = Mock()
    target = pose(.1)
    original = deepcopy(target)
    moveit.compute_fk.return_value = pose(.35)
    approach_above(moveit, target, .25, 5)
    assert target == original
    assert not moveit.move_to_pose.call_args.kwargs['cartesian']


def test_approach_checks_descent_before_moving():
    """Select a grasp-compatible hover and validate the entire descent first."""
    moveit = Mock()
    moveit.joint_names = ['joint']
    state = JointState(name=['joint'], position=[1.0])
    moveit.compute_ik.return_value = state
    moveit.compute_fk.return_value = pose(.35)
    configure = Mock()
    approach_above(moveit, pose(.1), .25, 5, configure)
    assert [call.args[0] for call in configure.call_args_list] == ['global', 'local']
    assert moveit.plan.call_args.kwargs['cartesian_fraction_threshold'] == 1.0
    calls = [call[0] for call in moveit.method_calls]
    assert calls.index('plan') < calls.index('move_to_configuration')


def test_unreachable_descent_never_moves():
    """A reachable hover must not authorize a blocked descent."""
    moveit = Mock()
    moveit.plan.return_value = None
    with pytest.raises(PrimitiveError, match='complete descent'):
        approach_above(moveit, pose(.1), .25, 5, Mock())
    moveit.move_to_configuration.assert_not_called()
    moveit.move_to_pose.assert_not_called()
