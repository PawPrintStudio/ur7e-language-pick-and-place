"""Offline checks for the lab pick adapter: geometry, gripper logic, planning choices.

No robot, no ROS graph: the executor is built without its node and the
gripper talks to a fake XML-RPC object. Run with::

    python3 -m pytest -q scripts/test_lab_pick.py
"""
import math
import os
import sys

from geometry_msgs.msg import Pose
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lab_pick  # noqa: E402
from lab_jog import JogError  # noqa: E402
from ur7e_orchestrator.adapters import Intent, StageError, Target  # noqa: E402


def pose_with(yaw_x, xyz=(0.0, 0.0, 0.5)):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    q = pose.orientation
    q.x, q.y, q.z, q.w = lab_pick.top_down_quaternion(yaw_x)
    return pose


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.4, math.pi / 2])
def test_top_down_quaternion_round_trip(yaw):
    measured, tilt = lab_pick.tool_yaw(pose_with(yaw))
    assert abs(math.remainder(measured - yaw, 2 * math.pi)) < 1e-9
    assert tilt < 1e-6


def test_tilted_tool_is_reported():
    pose = Pose()
    half = math.radians(170) / 2  # 10 degrees short of pointing straight down
    pose.orientation.x, pose.orientation.w = math.sin(half), math.cos(half)
    assert abs(lab_pick.tool_yaw(pose)[1] - 10.0) < 1e-6


def test_nearest_equivalent_prefers_the_smaller_wrist_turn():
    assert abs(lab_pick.nearest_equivalent(math.radians(170), math.radians(-5))
               - math.radians(-10)) < 1e-9
    assert abs(lab_pick.nearest_equivalent(0.2, 0.0) - 0.2) < 1e-9


class FakeRpc:
    """Jaws that move 10 mm per poll and stall at ``stall`` (an object) if set."""

    def __init__(self, width=80.0, stall=None, accept=0, never_stops=False):
        self.width, self.stall, self.accept = width, stall, accept
        self.target, self.never_stops, self.stopped = width, False, False
        self.runaway = never_stops

    def rg_grip(self, tool, width, force):
        self.target = width
        self.never_stops = self.runaway
        return self.accept

    def rg_stop(self, tool):
        self.stopped = True
        return 0

    def rg_get_all_variables(self, tool):
        limit = self.target if self.stall is None else max(self.target, self.stall)
        moving = abs(self.width - limit) > 1e-6
        if moving:
            self.width += max(-10.0, min(10.0, limit - self.width))
        if self.never_stops:
            self.width += 1.0
            moving = True
        holding = self.stall is not None and not moving and self.width > self.target
        return {'width': self.width, 'busy': moving, 'status': (1 if moving else 0)
                | (2 if holding else 0), 'safety_failed': False, 's1_pushed': False,
                's1_triggered': False, 's2_pushed': False, 's2_triggered': False}


def gripper(rpc):
    rg2 = lab_pick.Rg2.__new__(lab_pick.Rg2)
    rg2.rpc, rg2.tool, rg2.max_force = rpc, 2, 25.0
    return rg2


def test_gripper_reaches_an_open_width():
    state = gripper(FakeRpc(width=40.0)).move(80.0, 10.0)
    assert abs(state['width'] - 80.0) < 0.5 and not state['grip_detected']


def test_gripper_stalling_on_an_object_is_a_grasp_not_an_error():
    state = gripper(FakeRpc(width=80.0, stall=38.0)).move(0.0, 20.0)
    assert abs(state['width'] - 38.0) < 0.5 and state['grip_detected']


def test_gripper_refuses_out_of_bounds_and_reports_rejection_and_timeout():
    with pytest.raises(StageError, match='bounds'):
        gripper(FakeRpc()).move(120.0, 10.0)
    with pytest.raises(StageError, match='bounds'):
        gripper(FakeRpc()).move(50.0, 40.0)
    with pytest.raises(StageError, match='rg_grip returned'):
        gripper(FakeRpc(accept=3)).move(50.0, 10.0)
    rpc = FakeRpc(never_stops=True)
    with pytest.raises(StageError, match='still moving'):
        gripper(rpc).move(50.0, 10.0, timeout=0.6)
    assert rpc.stopped


class StubExecutor(lab_pick.PickExecutor):
    """PickExecutor geometry with planning replaced by a recorder."""

    def __init__(self, anchor, tool_yaw_x=0.3, unreachable=None):
        self.anchor, self.tool_length, self.hover = np.asarray(anchor, float), 0.2, 0.12
        self.plan_only, self.poses = True, {'ready': {}}
        self.tool_yaw_x, self.unreachable, self.planned = tool_yaw_x, unreachable, []
        self.tool_link, self.tooling = lab_pick.TOOL_LINK, dict(lab_pick.lab_tooling.DEFAULTS)

    def fresh(self, settle_s=1.0):
        return 'state'

    def fk(self, state):
        return pose_with(self.tool_yaw_x)

    def plan_pose(self, state, pose, avoid_collisions=True):
        self.planned.append((pose, avoid_collisions))
        if self.unreachable is not None and len(self.planned) == self.unreachable:
            raise JogError('only 40% of the straight line is reachable')
        return [{}]

    def predicted(self, state, points):
        return state

    def excursion(self, state, points):
        return self.swings.pop(0) if getattr(self, 'swings', None) else 0.0

    def plan_joint_line(self, state, goal, step=0.03):
        return []


def adapters(executor, **overrides):
    config = dict(lab_pick.DEFAULT_CONFIG, observe_pose='ready', home_pose='ready')
    config.update(overrides)
    return lab_pick.LabAdapters(executor, lab_pick.FakeRg2(), None, '127.0.0.1', config)


def tilted_table():
    """A table 30 mm below the base plane, sloping 1 cm per metre along x."""
    anchor = np.eye(4)
    slope = math.atan(0.01)
    anchor[:3, :3] = [[math.cos(slope), 0, -math.sin(slope)], [0, 1, 0],
                      [math.sin(slope), 0, math.cos(slope)]]
    anchor[:3, 3] = [-0.5, 0.0, -0.03]
    return anchor


def test_table_height_follows_the_measured_plane():
    executor = StubExecutor(tilted_table())
    assert abs(executor.table_z(-0.5, 0.2) - (-0.03)) < 1e-9
    assert abs(executor.table_z(-0.4, 0.0) - (-0.029)) < 1e-4
    assert abs(executor.tool_z(-0.5, 0.0, 0.008) - (-0.03 + 0.2 + 0.008)) < 1e-9
    with pytest.raises(JogError, match='floor'):
        executor.tool_z(-0.5, 0.0, 0.002)


def test_plan_heights_for_a_cube_keep_the_tool_yaw():
    executor = StubExecutor(np.eye(4))
    lab = adapters(executor)
    target = Target((-0.4, 0.2, 0.04), 0.0, {'height_m': 0.04, 'axis_ratio': 1.1})
    summary = lab.plan(Intent('pick', 'red block'), target, None)
    assert summary['tip_above_table_mm'] == 12.0     # 40 mm - 28 mm finger reach
    assert summary['hover_mm'] == 160                # 120 mm above the object top
    assert abs(lab.grasp_yaw - 0.3) < 1e-9           # near-square: do not turn the wrist
    assert summary['release_xy'] == summary['pick_xy']
    # approach and carry are collision-checked; the vertical segments are not.
    assert [avoid for _, avoid in executor.planned] == [True, False, False, True, False, False]
    lowest = min(pose.position.z for pose, _ in executor.planned)
    assert abs(lowest - (0.2 + 0.012)) < 1e-9


def test_plan_turns_the_jaws_across_a_long_object_and_respects_min_clearance():
    executor = StubExecutor(np.eye(4), tool_yaw_x=0.0)
    lab = adapters(executor)
    flat = Target((-0.4, 0.2, 0.012), math.radians(30),
                  {'height_m': 0.012, 'axis_ratio': 9.0})
    summary = lab.plan(Intent('pick', 'screwdriver'), flat, None)
    assert summary['tip_above_table_mm'] == 8.0
    # Jaws close along tool X, so X is perpendicular to the 30-degree long axis;
    # of the two equivalent angles (120, -60) the one nearer the current 0 wins.
    assert abs(math.degrees(lab.grasp_yaw) - (-60.0)) < 1e-6


def test_plan_places_on_top_of_a_located_place_target():
    executor = StubExecutor(np.eye(4))
    lab = adapters(executor)
    target = Target((-0.4, 0.2, 0.04), 0.0, {'height_m': 0.04, 'axis_ratio': 1.0})
    place = Target((-0.5, -0.1, 0.06), 0.0, {'height_m': 0.06, 'axis_ratio': 1.0})
    summary = lab.plan(Intent('pick_and_place', 'red block', 'box'), target, place)
    assert summary['release_xy'] == [-0.5, -0.1]
    assert summary['release_tip_mm'] == 76.0         # 60 + 12 + 4 mm
    assert summary['hover_mm'] == 180                # clears the taller of the two


def test_unreachable_target_is_refused_at_plan_time_and_tilt_is_checked():
    lab = adapters(StubExecutor(np.eye(4), unreachable=4))
    target = Target((-0.4, 0.2, 0.04), 0.0, {'height_m': 0.04, 'axis_ratio': 1.0})
    with pytest.raises(StageError) as caught:
        lab.plan(Intent('pick', 'red block'), target, None)
    assert caught.value.reason == 'unreachable'
    tilted = StubExecutor(np.eye(4))
    half = math.radians(170) / 2
    tilted.fk = lambda state: Pose(orientation=type(Pose().orientation)(
        x=math.sin(half), w=math.cos(half)))
    with pytest.raises(StageError) as caught:
        adapters(tilted).plan(Intent('pick', 'red block'), target, None)
    assert caught.value.reason == 'tool_not_vertical'


# --- the Dual Quick Changer tool stack (lab_tooling.py, 2026-10-08) --------------------

def test_rg2_tcp_planning_puts_the_fingertips_above_the_real_surface():
    executor = StubExecutor(tilted_table())
    executor.tool_link = lab_pick.PICK_LINK
    reach = executor.tooling['tcp_to_tips_m']
    # The TCP points straight down, so the tips are tcp_to_tips below it; the
    # calibration's tool0 length (0.2 here) no longer enters.
    assert abs(executor.tool_z(-0.5, 0.0, 0.008) - (-0.03 + reach + 0.008)) < 1e-9


def test_rg2_tcp_start_may_be_tilted_but_tool0_start_may_not():
    target = Target((-0.4, 0.2, 0.04), 0.0, {'height_m': 0.04, 'axis_ratio': 1.0})
    half = math.radians(60) / 2   # the RG2 at `ready`: 60 deg off vertical
    tilted_fk = lambda state: Pose(orientation=type(Pose().orientation)(  # noqa: E731
        x=math.sin(half), w=math.cos(half)))
    rg2 = StubExecutor(np.eye(4))
    rg2.tool_link, rg2.fk = lab_pick.PICK_LINK, tilted_fk
    summary = adapters(rg2).plan(Intent('pick', 'red block'), target, None)
    assert summary['waypoints'] == 6
    flange = StubExecutor(np.eye(4))
    flange.fk = tilted_fk
    with pytest.raises(StageError, match='tilted'):
        adapters(flange).plan(Intent('pick', 'red block'), target, None)


class Args:
    def __init__(self, execute, allow=False):
        self.execute, self.allow_unverified_tooling = execute, allow


def test_unverified_tooling_blocks_real_motion_only():
    unverified = dict(lab_pick.lab_tooling.DEFAULTS, verified=False)
    lab_pick.check_tooling(Args(execute=False), unverified)          # plan-only: fine
    lab_pick.check_tooling(Args(execute=True, allow=True), unverified)  # simulation
    with pytest.raises(RuntimeError, match='not verified'):
        lab_pick.check_tooling(Args(execute=True), unverified)
    lab_pick.check_tooling(Args(execute=True), dict(unverified, verified=True))


def test_rg2_tcp_takes_the_jaw_angle_with_the_least_joint_swing():
    executor = StubExecutor(np.eye(4), tool_yaw_x=0.0)
    executor.tool_link = lab_pick.PICK_LINK
    executor.swings = [6.4, 1.5, 3.0, 2.9]      # measured in simulation, 2026-10-08
    target = Target((-0.4, 0.2, 0.04), 0.0, {'height_m': 0.04, 'axis_ratio': 1.1})
    lab = adapters(executor)
    lab.plan(Intent('pick', 'red block'), target, None)
    assert abs(math.degrees(lab.grasp_yaw) - 90.0) < 1e-6   # the 1.5 rad candidate
    # A long object only accepts the two jaw angles across its axis.
    executor = StubExecutor(np.eye(4), tool_yaw_x=0.0)
    executor.tool_link, executor.swings = lab_pick.PICK_LINK, [2.0, 0.5]
    long = Target((-0.4, 0.2, 0.012), 0.0, {'height_m': 0.012, 'axis_ratio': 9.0})
    lab = adapters(executor)
    lab.plan(Intent('pick', 'screwdriver'), long, None)
    assert abs(abs(math.degrees(lab.grasp_yaw)) - 90.0) < 1e-6
    assert math.degrees(lab.grasp_yaw) < 0                   # the second (-90) one
