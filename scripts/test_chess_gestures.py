"""Gestures: planned from home, checked with FK, skipped when unsafe. No ROS."""
import json
import math
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import chess_gestures as g
    import lab_chess
except ImportError:
    g = lab_chess = None

pytestmark = pytest.mark.skipif(g is None, reason='python-chess is not installed')

HOME = {j: 0.0 for j in g.JOINTS}
BOARD = [[0, 0, 0], [0.4, 0, 0], [0.4, 0.4, 0], [0, 0.4, 0]]     # a1 at the origin, 50 mm squares


class FakeExecutor:
    """Toy arm: the fingertips sit 0.15 m beside the board edge, 0.12 m up at home.

    wrist_1 tilts the tool toward the board (x shrinks, tips drop);
    wrist_2 moves them sideways along y; wrist_3 only twists.
    """
    plan_only = True
    tooling = {'tcp_to_tips_m': 0.045}

    def __init__(self, home_xyz=(-0.15, 0.2, 0.165)):
        self.poses = {'chess_home': dict(HOME)}
        self.home_xyz = home_xyz
        self.executed = []

    def fresh(self, settle_s=0.3):
        return types.SimpleNamespace(joints=dict(HOME))

    @staticmethod
    def joints_of(state):
        return dict(state.joints)

    def plan_joint_line(self, state, goal, step=0.03):
        start = state.joints
        span = max(abs(goal[j] - start[j]) for j in g.JOINTS)
        count = max(2, int(math.ceil(span / step))) if span > 1e-3 else 0
        return [{j: start[j] + (goal[j] - start[j]) * i / count for j in g.JOINTS}
                for i in range(1, count + 1)]

    def predicted(self, state, points):
        return types.SimpleNamespace(joints=dict(points[-1])) if points else state

    def fk(self, state):
        w1, w2 = state.joints[g.W1], state.joints[g.W2]
        x0, y0, z0 = self.home_xyz
        # tcp pose: tool pointing down (quaternion 180 deg about x), tips 45 mm below
        reach = 0.25
        position = types.SimpleNamespace(x=x0 + reach * math.sin(w1), y=y0 + reach * math.sin(w2),
                                         z=z0 - reach * (1 - math.cos(w1)) - reach * (1 - math.cos(w2)))
        return types.SimpleNamespace(position=position,
                                     orientation=types.SimpleNamespace(x=1.0, y=0.0, z=0.0, w=0.0))

    def execute(self, state, points, rate):
        self.executed.append((len(points), rate))
        return {'executed': True, 'waypoints': len(points)}


class FakeLab:
    def __init__(self):
        self.gripper = types.SimpleNamespace(moves=[], move=lambda w, f: self.gripper.moves.append(w))

    def _move(self, action):
        return action()


def geometry():
    return lab_chess.BoardGeometry(BOARD)


def test_fingertips_follow_the_tool_axis():
    ex = FakeExecutor()
    x, y, z = g.fingertips(ex, ex.fresh())
    assert (x, y) == pytest.approx((-0.15, 0.2)) and z == pytest.approx(0.165 - 0.045)


def test_every_gesture_plans_from_home_and_ends_there():
    ex = FakeExecutor()
    for name in g.GESTURES:
        points, report = g.plan(ex, geometry(), name)
        assert points, (name, report)
        assert all(abs(points[-1][j]) < 1e-9 for j in g.JOINTS)       # back home
        assert report['lowest_fingertip_over_board_mm'] >= g.MIN_CLEAR_M * 1000


def test_a_gesture_that_would_swing_over_the_board_is_refused_whole():
    ex = FakeExecutor(home_xyz=(-0.04, 0.2, 0.165))       # home only 40 mm off the edge
    points, reason = g.plan(ex, geometry(), 'nod')         # wrist_1 tilts 50 mm toward the board
    assert points == [] and 'over the board' in reason


def test_a_gesture_that_would_dip_too_low_is_refused_whole():
    ex = FakeExecutor(home_xyz=(-0.15, 0.2, 0.11))        # tips 65 mm up at home
    points, reason = g.plan(ex, geometry(), 'droop')
    assert points == [] and 'mm over the board plane' in reason


def test_not_at_home_means_no_gesture():
    ex = FakeExecutor()
    away = types.SimpleNamespace(joints=dict(HOME, shoulder_pan_joint=0.3))
    points, reason = g.plan(ex, geometry(), 'wave', state=away)
    assert points == [] and 'from home' in reason


def test_perform_claps_only_for_real_and_never_raises(monkeypatch):
    said = []
    ex, lab = FakeExecutor(), FakeLab()
    result = g.perform('captured', ex, lab, geometry(), 0.4, lambda tag, p: said.append((tag, p)))
    assert result['executed'] and ex.executed == [(result['waypoints'], 0.4)]
    assert lab.gripper.moves == []                          # plan-only: no claps
    ex.plan_only = False
    g.perform('won', ex, lab, geometry(), 0.4, lambda tag, p: said.append((tag, p)))
    assert lab.gripper.moves == [50.0, 12.0] * 3
    assert g.perform('nothing', ex, lab, geometry(), 0.4, lambda *a: None) is None
    monkeypatch.setattr(ex, 'fresh', lambda settle_s=0.3: (_ for _ in ()).throw(RuntimeError('driver gone')))
    assert g.perform('check', ex, lab, geometry(), 0.4, lambda tag, p: said.append((tag, p))) is None
    assert 'driver gone' in said[-1][1]


def test_events_are_appended_as_json_lines(tmp_path):
    path = tmp_path / 'events.jsonl'
    g.log_event(str(path), 'captured', san='Nxd4', fen='x')
    g.log_event(str(path), 'check')
    rows = [json.loads(l) for l in path.read_text().splitlines()]
    assert [r['event'] for r in rows] == ['captured', 'check'] and rows[0]['san'] == 'Nxd4'
