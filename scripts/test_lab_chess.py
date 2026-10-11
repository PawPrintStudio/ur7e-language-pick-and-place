"""Offline checks for the chess layer: geometry, move steps, targets, the width check.

No robot, no ROS graph. Needs python-chess. Run with::

    python3 -m pytest -q scripts/test_lab_chess.py
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import chess
    import lab_chess
except ImportError:       # python-chess missing: skip, do not break collection
    chess = lab_chess = None

pytestmark = pytest.mark.skipif(lab_chess is None, reason='python-chess is not installed')

Z = 0.012   # board top above the table in the fake lab


def square_board(z=Z):
    """A 400 mm board with a1 at the origin, files along +x, ranks along +y."""
    return lab_chess.BoardGeometry([[0, 0, z], [0.4, 0, z], [0.4, 0.4, z], [0, 0.4, z]],
                                   box=[0.5, -0.1, 0.05])


def test_square_centres_axis_aligned():
    g = square_board()
    assert g.square_m == pytest.approx(0.05)
    assert g.square_xy('a1') == pytest.approx((0.025, 0.025))
    assert g.square_xy('h1') == pytest.approx((0.375, 0.025))
    assert g.square_xy('a8') == pytest.approx((0.025, 0.375))
    assert g.square_xy('e4') == pytest.approx((0.225, 0.175))
    assert g.file_yaw() == pytest.approx(0.0)
    assert g.complaints() == []


def test_square_centres_follow_a_rotated_board():
    # The same board turned a quarter turn: files run along +y, ranks along -x.
    g = lab_chess.BoardGeometry([[0, 0, Z], [0, 0.4, Z], [-0.4, 0.4, Z], [-0.4, 0, Z]])
    assert g.square_xy('h1') == pytest.approx((-0.025, 0.375))
    assert g.square_xy('a8') == pytest.approx((-0.375, 0.025))
    assert g.file_yaw() == pytest.approx(math.pi / 2)


def test_complaints_name_a_bad_corner_and_an_uneven_board():
    g = lab_chess.BoardGeometry([[0, 0, Z], [0.4, 0, Z], [0.42, 0.4, Z], [0, 0.4, Z + 0.01]])
    text = ' '.join(g.complaints())
    assert 'diagonals' in text and 'off the plane' in text


def test_save_and_load_round_trip(tmp_path):
    path = str(tmp_path / 'board.json')
    square_board().save(path)
    g = lab_chess.BoardGeometry.load(path)
    assert g.square_xy('d5') == pytest.approx((0.175, 0.225))
    assert g.box.tolist() == [0.5, -0.1, 0.05]


def test_bad_square_names_are_refused():
    with pytest.raises(ValueError):
        square_board().square_xy('i9')
    with pytest.raises(ValueError):
        lab_chess.parse_step('e2 e9')
    assert lab_chess.parse_step('e2 e4') == ('e2', 'e4')
    assert lab_chess.parse_step('e4->box') == ('e4', 'box')


def steps(fen, uci):
    board = chess.Board(fen) if fen else chess.Board()
    return lab_chess.steps_for(board, chess.Move.from_uci(uci))


def test_steps_for_the_kinds_of_move():
    assert steps(None, 'e2e4') == ([('e2', 'e4')], '')
    # exd5: the taken pawn goes to the box first.
    assert steps('rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2', 'e4d5') == \
        ([('d5', 'box'), ('e4', 'd5')], '')
    # Castling: king, then rook; both sides.
    castle = 'r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1'
    assert steps(castle, 'e1g1') == ([('e1', 'g1'), ('h1', 'f1')], '')
    assert steps(castle, 'e1c1') == ([('e1', 'c1'), ('a1', 'd1')], '')
    # En passant: the pawn passed over sits beside the mover, not on the destination.
    assert steps('rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3', 'e5f6') == \
        ([('f5', 'box'), ('e5', 'f6')], '')
    # Promotion: a note for the operator.
    assert steps('8/P7/8/8/8/8/8/k6K w - - 0 1', 'a7a8q')[1].startswith('promotion')


def test_parse_move_takes_san_and_uci_and_refuses_illegal():
    board = chess.Board()
    assert lab_chess.parse_move(board, 'Nf3') == chess.Move.from_uci('g1f3')
    assert lab_chess.parse_move(board, 'e2e4') == chess.Move.from_uci('e2e4')
    with pytest.raises(ValueError):
        lab_chess.parse_move(board, 'e2e5')


class FakeExecutor:
    plan_only = False        # as with --execute: the width check runs

    def table_z(self, x, y):
        return 0.0


class FakeLab:
    """Just what ChessAdapters touches of lab_pick.LabAdapters."""

    def __init__(self, closed_width=20.0):
        self.executor = FakeExecutor()
        self.config = {'finger_reach': 0.028, 'open_mm': 80.0}
        self.closed_width = closed_width
        self.held_width = None

    def grasp(self):
        self.held_width = self.closed_width
        return lab_chess.GraspResult(True, self.closed_width, {})


def adapters(measured=False, closed_width=20.0):
    pieces = lab_chess.load_pieces()
    pieces['measured'] = measured
    # Fixed sizes: the tests must not depend on what was last measured in the lab.
    pieces['pieces'] = {s: {'height_m': h, 'width_mm': 20} for s, h in
                        zip('kqbnrp', (0.095, 0.085, 0.070, 0.060, 0.055, 0.048))}
    return lab_chess.ChessAdapters(FakeLab(closed_width), square_board(), pieces)


def test_targets_carry_the_board_thickness_once():
    a = adapters()
    intent = a.parse('e2 e4')
    assert intent.action == 'pick_and_place' and intent.place_query == 'e4'
    pick = a.locate(a.detect('e2'))
    place = a.locate(a.detect('e4'))
    pawn = a.pieces['pieces']['p']['height_m']
    assert pick.xyz[:2] == pytest.approx((0.225, 0.075))
    assert pick.detail['height_m'] == pytest.approx(Z + pawn)     # minus finger_reach = tip height
    assert pick.detail['piece'] == 'pawn'
    assert place.detail['height_m'] == 0.0            # the thickness is already in the tip height
    # Jaws close across the reported axis: yaw + 90 deg = the board's diagonal.
    assert (pick.yaw + math.pi / 2) == pytest.approx(math.pi / 4)


def test_box_is_a_surface_at_its_rim():
    a = adapters()
    a.parse('e4 box')
    target = a.locate(a.detect('box'))
    assert target.xyz[:2] == pytest.approx((0.5, -0.1))
    assert target.detail['height_m'] == pytest.approx(0.05)


def test_an_empty_square_is_refused_as_a_pick():
    a = adapters()
    a.parse('e4 e5')
    with pytest.raises(lab_chess.StageError):
        a.detect('e4')


def test_width_check_only_when_measured():
    a = adapters(measured=False, closed_width=40.0)
    a.parse('e2 e4')
    a.locate(a.detect('e2'))
    assert a.grasp().holding                                   # placeholders: trust the jaws
    a = adapters(measured=True, closed_width=40.0)
    a.parse('e2 e4')
    a.locate(a.detect('e2'))
    result = a.grasp()
    assert not result.holding and result.detail['reason'] == 'width_off'
    assert a.lab.held_width is None
    a = adapters(measured=True, closed_width=22.0)
    a.parse('e2 e4')
    a.locate(a.detect('e2'))
    assert a.grasp().holding                                   # within 6 mm of 20


def test_random_mover_is_legal_and_prefers_captures():
    engine = lab_chess.Engine(path='none', seed=1)
    board = chess.Board('rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2')
    move = engine.choose(board)
    assert move in board.legal_moves and board.is_capture(move)
    assert 'random' in engine.name


def test_board_thickness_replaces_the_project1_table():
    class Executor:
        table_z = anchor = calibration = table_surface_z = None
    ex = Executor()
    surface = lab_chess.use_board_surface(ex, square_board(z=0.05), 0.020)
    assert surface == pytest.approx(0.03)
    assert ex.table_z(0.2, 0.2) == pytest.approx(0.03)        # the board is exactly 20 mm thick
    assert ex.table_surface_z == pytest.approx(0.03) and ex.anchor is None
    assert ex.lab_obstacles(0.03) == []


def test_a_tilted_board_is_one_plane_not_a_bad_touch():
    # The 2026-10-10 board: the h-side 4 mm lower than the a-side.
    g = lab_chess.BoardGeometry([[0, 0, 0.0], [0.4, 0, -0.004], [0.4, 0.4, -0.004],
                                 [0, 0.4, 0.0]])
    assert g.complaints() == []
    assert g.z_at(0.0, 0.2) == pytest.approx(0.0)
    assert g.z_at(0.4, 0.2) == pytest.approx(-0.004)
    assert g.tilt_deg() == pytest.approx(0.573, abs=0.01)
    class Executor:
        table_z = anchor = calibration = table_surface_z = None
    ex = Executor()
    lab_chess.use_board_surface(ex, g, 0.020)
    assert ex.table_z(0.4, 0.1) == pytest.approx(-0.024)       # follows the tilt
    assert ex.table_surface_z == pytest.approx(-0.024)         # level slab under the lowest corner


def test_a_touch_off_the_plane_is_flagged():
    g = lab_chess.BoardGeometry([[0, 0, 0.0], [0.4, 0, 0.0], [0.4, 0.4, -0.006],
                                 [0, 0.4, 0.0]])
    assert any('off the plane' in text for text in g.complaints())


def render_board(geometry, base_from_camera, matrix, size=(1280, 720), position=None):
    """A synthetic camera view of the empty board (a1 dark), for the overlay tests."""
    import cv2
    np = lab_chess.np
    side, border = 800, 100
    texture = np.full((side + 2 * border,) * 2, 255, np.uint8)
    for f in range(8):
        for r in range(8):
            if (f + r) % 2 == 0:            # a1 (0, 0) is dark
                x, y = border + f * 100, border + r * 100
                texture[y:y + 100, x:x + 100] = 40
    camera_from_base = np.linalg.inv(base_from_camera)
    rvec = cv2.Rodrigues(camera_from_base[:3, :3])[0]
    corners = np.array([geometry.point(u, v) for u, v in ((0, 0), (1, 0), (1, 1), (0, 1))])
    image_pts = cv2.projectPoints(corners, rvec, camera_from_base[:3, 3], matrix, None)[0]
    tex_pts = np.float32([[border, border], [border + side, border],
                          [border + side, border + side], [border, border + side]])
    h = cv2.getPerspectiveTransform(tex_pts, image_pts.reshape(4, 2).astype(np.float32))
    image = cv2.warpPerspective(texture, h, size, borderValue=200)
    if position is not None:            # pieces: blobs that hide the corners around them
        for square, piece in position.piece_map().items():
            f, r = chess.square_file(square), chess.square_rank(square)
            centre = geometry.point((f + 0.5) / 8, (r + 0.5) / 8) + [0, 0, 0.02]
            (x, y), = cv2.projectPoints(centre.reshape(1, 3), rvec, camera_from_base[:3, 3],
                                        matrix, None)[0].reshape(1, 2)
            radius = int(0.6 * np.linalg.norm(image_pts[1, 0] - image_pts[0, 0]) / 8)
            cv2.circle(image, (int(x), int(y)), radius,
                       230 if piece.color == chess.WHITE else 70, -1)
    return image


def look_at(eye, target):
    np = lab_chess.np
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    pose = np.eye(4)
    pose[:3, :3] = np.c_[x, np.cross(z, x), z]
    pose[:3, 3] = eye
    return pose


def test_overlay_recovers_the_camera_and_the_a1_corner():
    pytest.importorskip('cv2')
    np = lab_chess.np
    # The 2026-10-10 board: 290 mm, tilted 0.8 deg, files along -x.
    g = lab_chess.BoardGeometry([[0.198, 0.841, -0.102], [-0.093, 0.839, -0.106],
                                 [-0.093, 0.551, -0.105], [0.198, 0.551, -0.101]])
    matrix = np.array([[900.0, 0, 640], [0, 900, 360], [0, 0, 1]])
    truth = look_at(np.array([0.05, 1.25, 0.35]), [0.05, 0.70, -0.10])
    gray = render_board(g, truth, matrix)
    # An empty board looks the same turned 180 deg: one of the two answers is right.
    empty = chess.Board(None)
    poses = [lab_chess.locate_camera(g, gray, matrix, np.zeros(5), empty, flip)
             for flip in (False, True)]
    assert all(rms < 1.0 and used == 49 for _, _, rms, used in poses)
    assert any(np.allclose(pose[:3, 3], truth[:3, 3], atol=0.01) for pose, *_ in poses)


def test_overlay_works_with_the_pieces_set_up():
    pytest.importorskip('cv2')
    np = lab_chess.np
    g = lab_chess.BoardGeometry([[0.198, 0.841, -0.102], [-0.093, 0.839, -0.106],
                                 [-0.093, 0.551, -0.105], [0.198, 0.551, -0.101]])
    matrix = np.array([[900.0, 0, 640], [0, 900, 360], [0, 0, 1]])
    truth = look_at(np.array([0.05, 1.25, 0.35]), [0.05, 0.70, -0.10])
    start = chess.Board()
    gray = render_board(g, truth, matrix, position=start)
    pose, grid, rms, used = lab_chess.locate_camera(g, gray, matrix, np.zeros(5), start)
    assert used < 49                                   # the pieces hide part of the grid
    # A partial grid leaves the camera's distance loose (1-2 cm) but puts the
    # squares in the right place in the image, which is what the overlay needs.
    import cv2

    def pixels(base_from_camera):
        camera_from_base = np.linalg.inv(base_from_camera)
        centres = np.array([g.point((f + 0.5) / 8, (r + 0.5) / 8)
                            for f in range(8) for r in range(8)])
        return cv2.projectPoints(centres, cv2.Rodrigues(camera_from_base[:3, :3])[0],
                                 camera_from_base[:3, 3], matrix, None)[0].reshape(-1, 2)
    truth_px = pixels(truth)
    square_px = np.linalg.norm(truth_px[8] - truth_px[0])            # a1 -> b1
    assert np.linalg.norm(pixels(pose) - truth_px, axis=1).max() < 0.2 * square_px


def test_swapped_corners_are_caught():
    mirrored = lab_chess.BoardGeometry([[0, 0, Z], [0, 0.4, Z], [0.4, 0.4, Z], [0.4, 0, Z]])
    assert any('clockwise' in text for text in mirrored.complaints())
    assert square_board().complaints() == []


def test_best_square_snaps_noisy_touches_and_keeps_the_tilt():
    np = lab_chess.np
    touches = [[0.0, 0.0, 0.0], [0.29, 0.0, -0.004], [0.29, 0.29, -0.004], [0.0, 0.29, 0.0]]
    touches[0][0] += 0.002                       # a1 touched 2 mm toward h1
    touches[3][0] += 0.0025                      # a8 too: the 2026-10-10 touches
    corners, misses = lab_chess.best_square(touches)
    g = lab_chess.BoardGeometry(corners)
    sides = [np.linalg.norm(corners[i, :2] - corners[(i + 1) % 4, :2]) for i in range(4)]
    assert max(sides) - min(sides) < 1e-9 and misses.max() < 0.002
    assert g.complaints() == []
    assert g.z_at(0.29, 0.1) == pytest.approx(-0.004, abs=1e-4)


def test_an_empty_board_keeps_the_camera_where_it_was():
    pytest.importorskip('cv2')
    np = lab_chess.np
    g = lab_chess.BoardGeometry([[0.198, 0.841, -0.102], [-0.093, 0.839, -0.106],
                                 [-0.093, 0.551, -0.105], [0.198, 0.551, -0.101]])
    matrix = np.array([[900.0, 0, 640], [0, 900, 360], [0, 0, 1]])
    truth = look_at(np.array([0.05, 1.25, 0.35]), [0.05, 0.70, -0.10])
    gray = render_board(g, truth, matrix)
    for _ in range(3):              # the detector is random: the answer must not be
        pose, *_ = lab_chess.locate_camera(g, gray, matrix, np.zeros(5), chess.Board(None),
                                           previous=truth[:3, 3] + 0.03)
        assert np.allclose(pose[:3, 3], truth[:3, 3], atol=0.01)


def test_a_refit_keeps_the_camera(tmp_path):
    import json
    path = str(tmp_path / 'board.json')
    square_board().save(path)
    data = json.load(open(path))
    data['camera'] = {'base_from_camera': [[1, 0, 0, 0]] * 4}
    json.dump(data, open(path, 'w'))
    square_board(z=0.02).save(path)
    assert json.load(open(path))['camera'] == data['camera']


def test_watch_says_on_target_only_within_5_mm():
    g = square_board()                                # 50 mm squares, e4 centre (0.225, 0.175)
    on = lab_chess.tip_status(g, (0.227, 0.176, Z + 0.010), 'e4')
    assert on['ok'] and on['under'] == 'e4' and on['above_board_mm'] == 10
    near = lab_chess.tip_status(g, (0.233, 0.175, Z + 0.010), 'e4')
    assert not near['ok'] and near['under'] == 'e4' and near['error_mm'] == 8.0
    wrong = lab_chess.tip_status(g, (0.275, 0.175, Z + 0.010), 'e4')
    assert not wrong['ok'] and wrong['under'] == 'f4'
    idle = lab_chess.tip_status(g, (0.9, 0.9, Z + 0.2), None)
    assert idle['ok'] is None and idle['under'] is None


def test_a_measured_grip_height_sets_the_fingertips():
    pawn = {'height_m': 0.040, 'grip_m': 0.037, 'width_mm': 15}
    assert lab_chess.tip_above_base(pawn) == pytest.approx(0.032)
    assert lab_chess.tip_above_base({'height_m': 0.095}) == pytest.approx(0.067)
