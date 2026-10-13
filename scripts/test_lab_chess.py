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
    assert 'diagonals' in text and 'heights spread' in text


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
    plan_only = True

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
