"""The arm's reactions: wrist-only gestures from chess_home, and the event log.

Why wrist-only offsets from the taught home and no teaching: the three wrist
joints turn the gripper about itself, so the fingertips stay within a hand's
breadth of where home put them, beside the board. Every gesture is still
checked before it runs: the path is planned through MoveIt's collision check
(``plan_joint_line``), and the fingertips are followed with forward
kinematics so they never come within ``MIN_CLEAR_M`` of the board plane or
``MARGIN_M`` of the board's footprint. A gesture that fails either check is
skipped with a reason, never trimmed.

Events (``log/chess/events.jsonl``, one JSON object per line) are the same
feed the host-side voice reads (spec S4). No ROS imported here: ``perform``
takes the executor it is given.
"""
import json
import math
import os
import time

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
W1, W2, W3 = 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'

MIN_CLEAR_M = 0.06      # fingertips above the board plane, at every point
MARGIN_M = 0.03         # fingertips outside the board's footprint by this much
HOME_TOL_RAD = 0.05     # a gesture starts only from home

# Keyframes: joint offsets from home, radians; the last frame returns home.
GESTURES = {
    'nod': [{W1: 0.20}, {W1: -0.10}, {W1: 0.20}, {}],
    'shake': [{W2: 0.30}, {W2: -0.30}, {W2: 0.30}, {W2: -0.30}, {}],
    'celebrate': [{W3: 0.70}, {W3: -0.70}, {W3: 0.70}, {}],
    'droop': [{W1: -0.35}, {W1: -0.35}, {}],
    'dance': [{W2: 0.25, W3: 0.50}, {W2: -0.25, W3: -0.50}, {W2: 0.25, W3: 0.50},
              {W2: -0.25, W3: -0.50}, {}],
    'wave': [{W2: 0.30}, {W2: -0.30}, {W2: 0.30}, {W2: -0.30}, {}],
}
# Which gesture an event gets; claps are three quick jaw open/close cycles.
EVENT_GESTURE = {
    'captured': ('celebrate', True), 'lost': ('shake', False), 'check': ('nod', False),
    'won': ('dance', True), 'lost_game': ('droop', False), 'draw': ('nod', False),
    'greeting': ('wave', False),
}
CLAP_OPEN_MM, CLAP_CLOSE_MM, CLAP_FORCE_N = 50.0, 12.0, 10.0


def log_event(path, event, **detail):
    """Append one event line; the voice process tails this file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    row = dict(event=event, stamp=time.strftime('%Y-%m-%dT%H:%M:%S'), t=time.time(), **detail)
    with open(path, 'a') as stream:
        stream.write(json.dumps(row, default=str) + '\n')
    return row


def _rotation(q):
    """3x3 rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def fingertips(executor, state):
    """Fingertip position (x, y, z) for a RobotState: the TCP pose plus the tool axis."""
    pose = executor.fk(state)
    p, q = pose.position, pose.orientation
    r = _rotation((q.x, q.y, q.z, q.w))
    reach = float(executor.tooling['tcp_to_tips_m'])
    return (p.x + r[0][2] * reach, p.y + r[1][2] * reach, p.z + r[2][2] * reach)


def outside_board(geometry, x, y, margin_m=MARGIN_M):
    """Is (x, y) outside the board's footprint by ``margin_m``?"""
    import chess_vision
    u, v = chess_vision.board_uv(geometry, [[x, y]])[0]
    edge = margin_m / (8 * geometry.square_m)
    return u < -edge or u > 1 + edge or v < -edge or v > 1 + edge


def plan(executor, geometry, name, state=None, step=0.03, check_every=3):
    """Plan a gesture from the arm's present (home) pose.

    Returns (points, report): the joint path through every keyframe, or
    ([], reason) when the arm is not at home or any checked point would put
    the fingertips too low or over the board.
    """
    frames = GESTURES[name]
    state = executor.fresh(settle_s=0.3) if state is None else state
    home = executor.poses.get('chess_home')
    if home is None:
        return [], 'no chess_home taught'
    now = executor.joints_of(state)
    off = max(abs(now[j] - home[j]) for j in JOINTS)
    if off > HOME_TOL_RAD:
        return [], f'the arm is {off:.2f} rad from home'
    path, lowest = [], None
    for frame in frames:
        goal = {j: home[j] + frame.get(j, 0.0) for j in JOINTS}
        points = executor.plan_joint_line(state, goal, step=step)
        for index, point in enumerate(points):
            if index % check_every and index != len(points) - 1:
                continue
            probe = executor.predicted(state, [point])
            x, y, z = fingertips(executor, probe)
            clear = z - geometry.z_at(x, y)
            lowest = clear if lowest is None else min(lowest, clear)
            if clear < MIN_CLEAR_M:
                return [], f'{name}: fingertips would be {clear * 1000:.0f} mm over the board plane'
            if not outside_board(geometry, x, y):
                return [], f'{name}: fingertips would be over the board'
        path.extend(points)
        state = executor.predicted(state, points)
    return path, {'gesture': name, 'waypoints': len(path),
                  'lowest_fingertip_over_board_mm': round((lowest or 0.0) * 1000)}


def perform(event, executor, lab, geometry, rate, say, claps=True):
    """React to a game event from home: plan, check, run, clap. Never raises."""
    gesture, clap = EVENT_GESTURE.get(event, (None, False))
    if gesture is None:
        return None
    try:
        state = executor.fresh(settle_s=0.3)
        points, report = plan(executor, geometry, gesture, state)
        if not points:
            say('GESTURE', f'skipped: {report}')
            return None
        if clap and claps and not executor.plan_only:
            for _ in range(3):
                lab.gripper.move(CLAP_OPEN_MM, CLAP_FORCE_N)
                lab.gripper.move(CLAP_CLOSE_MM, CLAP_FORCE_N)
        result = lab._move(lambda: dict(executor.execute(state, points, rate), stage=gesture))
        say('GESTURE', dict(report, executed=result.get('executed'), event=event))
        return result
    except Exception as error:      # noqa: BLE001 - a gesture must never stop the game
        say('GESTURE', f'skipped: {error}')
        return None
