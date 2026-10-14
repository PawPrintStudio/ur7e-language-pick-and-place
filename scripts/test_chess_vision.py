"""Offline checks for chess_vision.py: the scan, reading a move, following a moved board.

Stereo matching itself is tested in ur7e_perception (rendered pairs); these
start from 3-D points, as BoardCamera.points returns them. Needs python-chess.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import chess
    import chess_vision as cv_
    import lab_chess
except ImportError:
    chess = cv_ = lab_chess = None

pytestmark = pytest.mark.skipif(cv_ is None, reason='python-chess or the stereo module missing')

CAMERA_XY = (0.2, 1.2)          # beyond the 8th rank, looking back over the board


class FakeCamera:
    """What scan() reads of BoardCamera: the board, the camera pose, piece image size."""

    def __init__(self, geometry):
        self.geometry = geometry
        self.base_from_camera = np.eye(4)
        self.base_from_camera[:2, 3] = CAMERA_XY

    def expected_pixels(self, xy):
        return 1000.0


def board():
    # 400 mm board, a1 at the origin, files along +x, ranks along +y, 50 mm squares.
    return lab_chess.BoardGeometry([[0, 0, 0], [0.4, 0, 0], [0.4, 0.4, 0], [0, 0.4, 0]])


def cloud(g, pieces, rng, noise=0.0015):
    """Board surface points plus, per piece, the camera-facing half of a cylinder."""
    uv = rng.uniform(0, 1, (20000, 2))
    surface = np.array([g.point(u, v) for u, v in uv])
    xyz, height = [surface], [rng.normal(0, noise, len(surface))]
    for (cx, cy), top, radius in pieces:
        toward = np.subtract(CAMERA_XY, (cx, cy))
        facing = math.atan2(toward[1], toward[0])
        # From above the camera sees the near half of the sides and the head all
        # round (measured: centres come out right without a front-side correction).
        z = rng.uniform(0, top, 1500)
        half = rng.uniform(-math.pi / 2, math.pi / 2, 1500)
        whole = rng.uniform(-math.pi, math.pi, 1500)
        angle = facing + np.where(z > 0.6 * top, whole, half)
        xyz.append(np.c_[cx + radius * np.cos(angle), cy + radius * np.sin(angle), z])
        height.append(z + rng.normal(0, noise, 1500))
    xyz, height = np.vstack(xyz), np.concatenate(height)
    return {'xyz': xyz, 'height': height, 'uv': cv_.board_uv(g, xyz[:, :2]),
            'rgb': np.full((len(xyz), 3), 150.0)}


def test_scan_finds_the_piece_its_height_and_its_real_centre():
    g, rng = board(), np.random.default_rng(1)
    centre = np.array(g.square_xy('e4')) + [0.006, -0.004]       # 7 mm off centre
    points = cloud(g, [(centre, 0.040, 0.009)], rng)
    # SGBM's floating mismatches: a dense blob at 65 mm with nothing below it.
    blob = np.array(g.square_xy('c6')) + rng.normal(0, 0.004, (300, 2))
    points['xyz'] = np.vstack([points['xyz'], np.c_[blob, np.full(300, 0.065)]])
    points['height'] = np.concatenate([points['height'], np.full(300, 0.065)])
    points['uv'] = cv_.board_uv(g, points['xyz'][:, :2])
    points['rgb'] = np.vstack([points['rgb'], np.full((300, 3), 150.0)])
    squares = cv_.scan(points, FakeCamera(g))
    occupied = [n for n, e in squares.items() if e['occupied']]
    assert occupied == ['e4']                                       # the blob is not a piece
    e4 = squares['e4']
    assert e4['top_m'] == pytest.approx(0.040, abs=0.003)
    assert np.hypot(*(np.array(e4['centre']) - centre)) < 0.002     # the axis, not the front
    assert all(e['seen'] for e in squares.values())


def test_the_gripper_is_not_a_piece():
    g, rng = board(), np.random.default_rng(2)
    tips = np.r_[g.square_xy('d5'), 0.06]
    points = cloud(g, [(tips[:2], 0.20, 0.03)], rng)                # the RG2, down to the tips
    points['height'] = points['height'] + (points['height'] > 0.01) * 0.045
    points['xyz'][:, 2] = points['height']
    kept = cv_.without_robot(points, tips)
    squares = cv_.scan(kept, FakeCamera(g))
    assert not any(e['occupied'] for e in squares.values())


def scan_of(position, hidden=()):
    return {chess.square_name(s): {'occupied': position.piece_at(s) is not None,
                                   'seen': chess.square_name(s) not in hidden}
            for s in chess.SQUARES}


def test_the_student_move_is_read_from_the_board():
    start = chess.Board()
    after = start.copy()
    after.push_uci('e2e4')
    move, margin, _ = cv_.infer_move(start, scan_of(after))
    assert move.uci() == 'e2e4' and margin >= 2
    # e2e3 and e2e4 differ only on e3/e4: hide e4 and the two tie.
    _, margin, _ = cv_.infer_move(start, scan_of(after, hidden=('e3', 'e4')))
    assert margin == 0


def test_a_capture_needs_the_colour():
    position = chess.Board('4k3/8/8/3p4/4P3/8/8/4K3 w - - 0 1')
    after = position.copy()
    after.push_uci('e4d5')                       # exd5: d5 stays occupied, by White now
    squares = scan_of(after)
    colours = {'d5': chess.WHITE, 'e1': chess.WHITE, 'e8': chess.BLACK}
    move, margin, _ = cv_.infer_move(position, squares, colours)
    assert move.uci() == 'e4d5' and margin >= 1


def test_a_move_is_read_from_what_changed_even_with_the_back_rank_missing():
    start = chess.Board()
    after = start.copy()
    after.push_uci('e2e4')
    hidden = ('e8', 'f8', 'g8', 'h8', 'g1')                 # what this camera misses every scan
    before_scan, after_scan = scan_of(start, hidden=hidden), scan_of(after, hidden=hidden)
    move, margin, _ = cv_.infer_move_diff(start, before_scan, after_scan)
    assert move.uci() == 'e2e4' and margin >= 2
    # The absolute reading of the same scans is hurt by the misses; the diff is not.
    # A square that reads wrong in BOTH scans (g1 "empty") does not count either.
    wrong_both = {n: dict(e, occupied=False) if n == 'g1' else e for n, e in scan_of(start).items()}
    wrong_after = {n: dict(e, occupied=False) if n == 'g1' else e for n, e in scan_of(after).items()}
    move, margin, _ = cv_.infer_move_diff(start, wrong_both, wrong_after)
    assert move.uci() == 'e2e4' and margin >= 2


def test_a_capture_needs_the_colour_flip_on_its_square():
    position = chess.Board('4k3/8/8/3p4/4P3/8/8/4K3 w - - 0 1')
    after = position.copy()
    after.push_uci('e4d5')                                  # exd5: only e4 changes in occupancy
    before_scan, after_scan = scan_of(position), scan_of(after)
    # The camera sees a white piece on d5 now: the capture is read, ahead of e4-e5.
    move, margin, _ = cv_.infer_move_diff(position, before_scan, after_scan, {'d5': True, 'e1': True, 'e8': False})
    assert move.uci() == 'e4d5' and margin >= 1
    # No colour information: a lifted pawn looks the same, so the capture is not confident.
    move, margin, _ = cv_.infer_move_diff(position, before_scan, after_scan)
    assert not (move.uci() == 'e4d5' and margin >= 1)
    # d5 still reads black: the pawn is in the hand, not on d5; e4-e5 (its own square hidden) wins.
    move, margin, _ = cv_.infer_move_diff(position, before_scan, after_scan, {'d5': False})
    assert move.uci() != 'e4d5'


def test_a_bishop_in_the_hand_is_not_a_capture():
    # 2026-10-14: after 1.e4 a6, the student lifted the f1 bishop; the diff read Bxa6.
    position = chess.Board('rnbqkbnr/1ppppppp/p7/8/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2')
    before_scan = scan_of(position)
    lifted = {n: dict(e, occupied=False) if n == 'f1' else e for n, e in before_scan.items()}
    colours = {'a6': False, 'e1': True, 'e8': False}       # a6 still shows the black pawn
    move, margin, ranking = cv_.infer_move_diff(position, before_scan, lifted, colours)
    assert not any(m.uci() == 'f1a6' and s >= ranking[0][0] for s, m in ranking[:1])
    assert margin == 0 or move.uci() != 'f1a6'              # nothing confident: the operator waits


def test_a_moved_board_is_followed_from_the_grid():
    pytest.importorskip('cv2')
    import test_lab_chess as t
    g = lab_chess.BoardGeometry([[0.198, 0.841, -0.102], [-0.093, 0.839, -0.106],
                                 [-0.093, 0.551, -0.105], [0.198, 0.551, -0.101]])
    matrix = np.array([[900.0, 0, 640], [0, 900, 360], [0, 0, 1]])
    camera = t.look_at(np.array([0.05, 1.25, 0.35]), [0.05, 0.70, -0.10])
    # The board is bumped: 10 mm along x, 7 mm along y, turned 2 deg about its centre.
    centre = g.point(0.5, 0.5)
    turn = np.eye(4)
    a = math.radians(2)
    turn[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    to_centre, back = np.eye(4), np.eye(4)
    to_centre[:3, 3], back[:3, 3] = -centre, centre + [0.010, 0.007, 0]
    bump = back @ turn @ to_centre
    moved = lab_chess.BoardGeometry(cv_.shifted_corners(g.corners, bump))
    gray = t.render_board(moved, camera, matrix)
    _, image, _, _, world = lab_chess.locate_camera(
        g, gray, matrix, np.zeros(5), chess.Board(None), previous=camera[:3, 3], with_world=True)
    shift, rms = cv_.board_motion(world, image.reshape(-1, 2), camera, matrix, np.zeros(5),
                                  centre)
    assert rms < 1.0
    found = cv_.shifted_corners(g.corners, shift)
    assert np.abs(found - moved.corners).max() < 0.002              # every corner within 2 mm
    mm, deg = cv_.shift_size(shift, centre)
    assert mm == pytest.approx(12.2, abs=1.0) and deg == pytest.approx(2.0, abs=0.3)


def test_an_unmoved_board_with_pieces_reads_as_unmoved():
    pytest.importorskip('cv2')
    import test_lab_chess as t
    g = lab_chess.BoardGeometry([[0.198, 0.841, -0.102], [-0.093, 0.839, -0.106],
                                 [-0.093, 0.551, -0.105], [0.198, 0.551, -0.101]])
    matrix = np.array([[900.0, 0, 640], [0, 900, 360], [0, 0, 1]])
    camera = t.look_at(np.array([0.05, 1.25, 0.35]), [0.05, 0.70, -0.10])
    start = chess.Board()
    gray = t.render_board(g, camera, matrix, position=start)       # a partial grid only
    _, image, _, used, world = lab_chess.locate_camera(
        g, gray, matrix, np.zeros(5), start, previous=camera[:3, 3], with_world=True)
    shift, _ = cv_.board_motion(world, image.reshape(-1, 2), camera, matrix, np.zeros(5),
                                g.point(0.5, 0.5))
    mm, deg = cv_.shift_size(shift, g.point(0.5, 0.5))
    assert used < 49 and mm < 1.0 and abs(deg) < 0.6      # blobs nudge the corners


def test_tracking_a_nudged_empty_board_never_reads_a_half_turn(monkeypatch):
    pytest.importorskip('cv2')
    import test_lab_chess as t
    g = lab_chess.BoardGeometry([[0.198, 0.841, -0.102], [-0.093, 0.839, -0.106],
                                 [-0.093, 0.551, -0.105], [0.198, 0.551, -0.101]])
    matrix = np.array([[900.0, 0, 640], [0, 900, 360], [0, 0, 1]])
    camera = t.look_at(np.array([0.05, 1.25, 0.35]), [0.05, 0.70, -0.10])
    bump = cv_.planar_motion(0.012, -0.008, math.radians(1.5), g.point(0.5, 0.5))
    moved = lab_chess.BoardGeometry(cv_.shifted_corners(g.corners, bump))
    gray = t.render_board(moved, camera, matrix)
    rgb = np.dstack([gray] * 3)
    monkeypatch.setattr(lab_chess, 'grab_pair', lambda *a, **k: (rgb, rgb))
    import json
    saved = {'camera': {'base_from_camera': camera.tolist(), 'mode': 'TEST'}}
    import ur7e_perception.camera as camera_module
    monkeypatch.setattr(camera_module, 'zed_factory_calibration',
                        lambda conf, mode: (matrix, np.zeros(5), (1280, 720)))
    path = '/tmp/test_track_board.json'
    json.dump(saved, open(path, 'w'))
    for _ in range(3):                            # the detector is random; the answer is not
        shift, mm, deg, rms, used = lab_chess.track_board(g, path=path)
        assert mm == pytest.approx(14.4, abs=1.5) and deg == pytest.approx(1.5, abs=0.4)


def test_the_live_view_marks_pieces_differences_and_the_move():
    pytest.importorskip('cv2')
    import cv2
    import test_lab_chess as t
    g = lab_chess.BoardGeometry([[0.198, 0.841, -0.102], [-0.093, 0.839, -0.106],
                                 [-0.093, 0.551, -0.105], [0.198, 0.551, -0.101]])
    matrix = np.array([[900.0, 0, 640], [0, 900, 360], [0, 0, 1]])
    camera = t.look_at(np.array([0.05, 1.25, 0.35]), [0.05, 0.70, -0.10])
    image = np.dstack([t.render_board(g, camera, matrix)] * 3).astype(np.uint8)
    expected = chess.Board('8/8/8/P7/8/8/8/8 w - - 0 1')            # the game: pawn on a5
    squares = {chess.square_name(s): {'occupied': False, 'seen': True} for s in chess.SQUARES}
    squares['a6'] = {'occupied': True, 'seen': True, 'top_m': 0.041,
                     'centre': list(g.square_xy('a6'))}                # the camera: pawn on a6
    squares['h8']['seen'] = False
    view = {'squares': squares, 'colours': {'a6': True}, 'expected': expected,
            'planned': {'from': 'a5', 'to': 'a6', 'capture': False}}
    out = lab_chess.draw_watch(image, g, camera, matrix, np.zeros(5), None, {}, view)
    camera_from_base = np.linalg.inv(camera)
    rvec = cv2.Rodrigues(camera_from_base[:3, :3])[0]

    def pixel(square):
        x, y = g.square_xy(square)
        return cv2.projectPoints(np.array([[x, y, g.z_at(x, y)]]), rvec, camera_from_base[:3, 3],
                                 matrix, None)[0].ravel().astype(int)

    x, y = pixel('a5')
    patch = out[y - 30:y + 30, x - 40:x + 40].astype(int)
    red = (patch[:, :, 2] > 150) & (patch[:, :, 1] < 100) & (patch[:, :, 0] < 100)
    assert red.sum() > 5                    # small warning corners: expected pawn is missing
    pixels = out.astype(int)
    blue = (pixels[:, :, 0] > 200) & (pixels[:, :, 1] < 180) & (pixels[:, :, 2] < 80)
    assert blue.sum() > 40                  # planned-move arrow remains visible
