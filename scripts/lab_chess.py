#!/usr/bin/env python3
"""Chess on the UR7e: the arm plays a student (phase 2, docs/CHESS.md).

A move needs no camera. The board geometry says where every square is and the
piece table says how tall each piece is, so the orchestrator's DETECT and
LOCATE become a lookup, and every proven stage after them (PLAN, APPROACH,
GRASP, LIFT, RETREAT, HOME) runs unchanged from ``lab_pick.py``.

    python3 scripts/lab_chess.py touch --corner 1      # fingertip on a board corner (1..4) or box
    python3 scripts/lab_chess.py fit                    # the four touches -> lab_chess.json
    python3 scripts/lab_chess.py squares                # print the square centres (no robot)
    python3 scripts/lab_chess.py goto e4 [--touch]      # hover over a square, point at it
    python3 scripts/lab_chess.py move e2e4              # one move through the orchestrator
    python3 scripts/lab_chess.py play                   # a match: the student's move is typed in

Nothing moves without ``--execute``. First real moves: pendant slider low,
``--max-speed-percent 25``, every step confirmed (docs/CHESS.md).

Corners 1..4 are the outer corners of the playing area: 1 = a1, 2 = h1,
3 = h8, 4 = a8 (a1 is the dark square at White's left). Files run from
corner 1 toward 2, ranks from corner 1 toward 4. ``box`` is a point on the
rim of the captured-pieces box; taken pieces are let go just above it.
"""
import argparse
import json
import math
import os
import random
import re
import shutil
import sys
import time

import chess
import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'ur7e_orchestrator'))
from ur7e_orchestrator.adapters import (  # noqa: E402
    Adapters, Detection, GraspResult, Intent, Refused, StageError, Target)

GEOMETRY_FILE = os.path.join(_HERE, 'lab_chess.json')
PIECES_FILE = os.path.join(_HERE, 'lab_chess.yaml')
TOUCH_FILE = os.path.join(_HERE, '.lab_chess_touches.json')
CORNERS = ('1', '2', '3', '4')          # a1, h1, h8, a8
BOX = 'box'
SQUARE = re.compile(r'^[a-h][1-8]$')
STEP = re.compile(r'^\s*([a-h][1-8])\s*(?:->|to|,)?\s*([a-h][1-8]|box)\s*$')


def say(tag, payload):
    """One line per event, machine-readable (same format as lab_pick)."""
    text = payload if isinstance(payload, str) else json.dumps(payload, default=str)
    print(f'{tag:8} {text}', flush=True)


def ask(prompt, auto_yes=False):
    if auto_yes:
        return True
    try:
        return input(f'{prompt} [y/N] ').strip().lower() in ('y', 'yes')
    except EOFError:
        return False


# --- the board ---------------------------------------------------------------------------

class BoardGeometry:
    """Square centres in base_link, from the four touched outer corners.

    A square's centre is read off the quadrilateral by bilinear interpolation,
    so a rotated board, or touches a few millimetres off square, still put
    every square where the fingertip found its corners.
    """

    def __init__(self, corners, box=None, notes=None):
        self.corners = np.asarray(corners, dtype=float).reshape(4, 3)
        self.box = None if box is None else np.asarray(box, dtype=float).reshape(3)
        self.notes = dict(notes or {})
        sides = [np.linalg.norm(self.corners[i, :2] - self.corners[(i + 1) % 4, :2])
                 for i in range(4)]
        self.square_m = float(np.mean(sides)) / 8
        self.top_z = float(np.mean(self.corners[:, 2]))    # the board's top surface

    @classmethod
    def load(cls, path=GEOMETRY_FILE):
        if not os.path.exists(path):
            raise FileNotFoundError(f'{path} is missing: touch the four corners, then fit')
        with open(path) as stream:
            data = json.load(stream)
        return cls(data['corners'], data.get('box'), data.get('notes'))

    def save(self, path=GEOMETRY_FILE):
        data = {'corners': self.corners.tolist(),
                'box': None if self.box is None else self.box.tolist(),
                'square_m': round(self.square_m, 4), 'top_z': round(self.top_z, 4),
                'notes': self.notes}
        with open(path, 'w') as stream:
            json.dump(data, stream, indent=2)
            stream.write('\n')

    def square_xy(self, name):
        if not SQUARE.match(name):
            raise ValueError(f'{name!r} is not a square (a1..h8)')
        u = (ord(name[0]) - ord('a') + 0.5) / 8      # along the files, corner 1 -> 2
        v = (int(name[1]) - 1 + 0.5) / 8              # along the ranks, corner 1 -> 4
        c1, c2, c3, c4 = self.corners[:, :2]
        p = (1 - u) * (1 - v) * c1 + u * (1 - v) * c2 + u * v * c3 + (1 - u) * v * c4
        return float(p[0]), float(p[1])

    def file_yaw(self):
        """Direction of the files (a toward h) in base_link, radians."""
        d = self.corners[1, :2] - self.corners[0, :2]
        return math.atan2(d[1], d[0])

    def complaints(self, side_tol_m=0.006, flat_tol_m=0.004):
        """What is wrong with the four touches, as strings; empty when nothing is."""
        c, out = self.corners, []
        sides = [np.linalg.norm(c[i, :2] - c[(i + 1) % 4, :2]) for i in range(4)]
        pairs = ((0, 2, 'the a1-h1 and h8-a8 edges'), (1, 3, 'the h1-h8 and a8-a1 edges'))
        for a, b, what in pairs:
            if abs(sides[a] - sides[b]) > side_tol_m:
                out.append(f'{what} differ by {abs(sides[a] - sides[b]) * 1000:.1f} mm')
        diagonals = [np.linalg.norm(c[0, :2] - c[2, :2]), np.linalg.norm(c[1, :2] - c[3, :2])]
        if abs(diagonals[0] - diagonals[1]) > side_tol_m:
            out.append(f'the diagonals differ by {abs(diagonals[0] - diagonals[1]) * 1000:.1f} mm: '
                       'one corner is off')
        spread = c[:, 2].max() - c[:, 2].min()
        if spread > flat_tol_m:
            out.append(f'corner heights spread {spread * 1000:.1f} mm: the board is not flat '
                       'or one touch pressed harder')
        return out


def load_pieces(path=PIECES_FILE):
    """The piece table: heights, jaw-closed widths, the jaw opening (lab_chess.yaml)."""
    with open(path) as stream:
        data = yaml.safe_load(stream)
    missing = [s for s in 'kqrbnp' if s not in data.get('pieces', {})]
    if missing:
        raise ValueError(f'{path}: no size for {", ".join(missing)}')
    return data


# --- moves -------------------------------------------------------------------------------

def steps_for(board, move):
    """Return (steps, note): the physical steps for ``move``, (from, to), ``box`` as a destination.

    A capture clears the destination first. Castling is the king, then the
    rook. En passant removes the pawn that was passed over, which is beside
    the mover, not on the destination. A promotion is a note: the piece is
    swapped by hand.
    """
    frm, to = chess.square_name(move.from_square), chess.square_name(move.to_square)
    if board.is_castling(move):
        rank = frm[1]
        rook_from, rook_to = ('h', 'f') if to[0] == 'g' else ('a', 'd')
        steps = [(frm, to), (rook_from + rank, rook_to + rank)]
    elif board.is_en_passant(move):
        steps = [(to[0] + frm[1], BOX), (frm, to)]
    elif board.is_capture(move):
        steps = [(to, BOX), (frm, to)]
    else:
        steps = [(frm, to)]
    note = ''
    if move.promotion:
        note = f'promotion: swap the pawn on {to} for a {chess.piece_name(move.promotion)} by hand'
    return steps, note


def parse_step(text):
    found = STEP.match(text)
    if not found:
        raise ValueError(f'{text!r}: say "e2 e4" or "e4 box"')
    return found.group(1), found.group(2)


def parse_move(board, text):
    """SAN ("Nf3", "O-O") or UCI ("g1f3"); it must be legal on ``board``."""
    text = text.strip()
    try:
        return board.parse_san(text)
    except ValueError:
        pass
    try:
        return board.parse_uci(text)
    except ValueError:
        raise ValueError(f'{text!r} is not a legal move here')


# --- the orchestrator's adapter ----------------------------------------------------------

class ChessAdapters(Adapters):
    """``lab_pick.LabAdapters`` with the language and camera stages replaced by a lookup.

    ``lab`` owns the executor and the gripper; every motion stage is handed to
    it untouched. The jaws close along the board's diagonal: a piece's nearest
    neighbours are one square away along the file and the rank, 1.4 squares
    away diagonally, so the open jaws have the most room there.
    """

    def __init__(self, lab, geometry, pieces, board=None):
        self.lab, self.geometry, self.pieces = lab, geometry, pieces
        self.board = board if board is not None else chess.Board()
        self.intent = None
        self.expected_width_mm = None

    # The lookup.

    def parse(self, utterance):
        try:
            frm, to = parse_step(utterance)
        except ValueError as error:
            raise Refused('not_a_step', str(error))
        self.intent = Intent('pick_and_place', frm, to, f'{frm} -> {to}')
        return self.intent

    def detect(self, query):
        role = 'pick' if query == self.intent.target_query else 'place'
        if role == 'pick' and self.board.piece_at(chess.parse_square(query)) is None:
            raise StageError('empty_square', f'{query} is empty on the board as I know it')
        return Detection(f'{role}:{query}', 1.0)

    def locate(self, detection):
        role, name = detection.capture_id.split(':', 1)
        table_z = self.lab.executor.table_z
        # Reported as the object's "long axis": LabAdapters closes the jaws across it.
        yaw = self.geometry.file_yaw() - math.pi / 4
        if name == BOX:
            if self.geometry.box is None:
                raise StageError('no_box', 'no captured-pieces box: touch --corner box, then fit')
            x, y, z = (float(v) for v in self.geometry.box)
            rim = max(0.0, z - table_z(x, y))
            return Target((x, y, z), yaw, {'height_m': round(rim, 4), 'axis_ratio': 100.0})
        x, y = self.geometry.square_xy(name)
        top = self.geometry.top_z
        if role == 'place':
            # 0, not the board thickness: the release height already carries
            # it, through the pick's tip height.
            return Target((x, y, top), yaw, {'height_m': 0.0, 'axis_ratio': 100.0})
        piece = self.board.piece_at(chess.parse_square(name))
        size = self.pieces['pieces'][piece.symbol().lower()]
        measured = bool(self.pieces.get('measured'))
        self.expected_width_mm = float(size['width_mm']) if measured else None
        thickness = max(0.0, top - table_z(x, y))
        return Target((x, y, top + float(size['height_m'])), yaw,
                      {'height_m': round(thickness + float(size['height_m']), 4),
                       'axis_ratio': 100.0, 'piece': chess.piece_name(piece.piece_type)})

    def grasp(self):
        """A width far from the piece's known width is a miss, whatever the jaws felt."""
        result = self.lab.grasp()
        expected = self.expected_width_mm
        if expected is None or not result.holding:
            return result
        tolerance = float(self.pieces.get('width_tolerance_mm', 6.0))
        if abs(result.width_mm - expected) > tolerance:
            self.lab.held_width = None
            return GraspResult(False, result.width_mm,
                               dict(result.detail, expected_mm=expected, reason='width_off'))
        return result

    # The proven stages, untouched.

    def observe(self):
        self.lab.observe()

    def plan(self, intent, target, place):
        return self.lab.plan(intent, target, place)

    def approach(self):
        self.lab.approach()

    def lift(self):
        self.lab.lift()

    def retreat(self):
        self.lab.retreat()

    def release(self):
        self.lab.release()

    def home(self):
        self.lab.home()

    def safety_ok(self):
        return self.lab.safety_ok()

    def abort(self):
        self.lab.abort()


class Engine:
    """The arm's brain: Stockfish when it is installed, otherwise a random mover."""

    def __init__(self, path='stockfish', skill=1, seconds=0.5, seed=None):
        self.seconds, self.random, self.engine = seconds, random.Random(seed), None
        found = shutil.which(path) if path and path != 'none' else None
        if found:
            import chess.engine
            self.engine = chess.engine.SimpleEngine.popen_uci(found)
            try:
                self.engine.configure({'Skill Level': int(skill)})
            except chess.engine.EngineError:
                pass    # not every UCI engine has the option
        self.name = os.path.basename(found) if found else 'random mover (captures first)'

    def choose(self, board):
        if self.engine is not None:
            import chess.engine
            return self.engine.play(board, chess.engine.Limit(time=self.seconds)).move
        moves = list(board.legal_moves)
        captures = [m for m in moves if board.is_capture(m)]
        return self.random.choice(captures or moves)

    def close(self):
        if self.engine is not None:
            self.engine.quit()


# --- sessions with the arm ---------------------------------------------------------------

def open_session(args, board):
    """Executor, lab adapters, chess adapters and orchestrator (needs the ROS graph)."""
    import rclpy
    import lab_pick
    from ur7e_orchestrator.workflow import Orchestrator
    geometry = BoardGeometry.load(args.geometry)
    pieces = load_pieces(args.pieces)
    rclpy.init()
    executor, lab, _ = lab_pick.build(args)
    lab.config['open_mm'] = float(pieces.get('open_mm', lab.config['open_mm']))
    tallest = max(float(p['height_m']) for p in pieces['pieces'].values())
    # A carried piece hangs (its height - finger_reach) below the fingertips,
    # which hover args.hover above the carried piece's top.
    clearance = args.hover + lab.config['finger_reach'] - tallest
    if clearance < 0.03:
        say('WARNING', f'a carried piece clears the tallest piece by {clearance * 1000:.0f} mm; '
                       'raise --hover')
    adapters = ChessAdapters(lab, geometry, pieces, board)
    orchestrator = Orchestrator(adapters, log_dir=args.log_dir, on_event=lab_pick.on_event)
    return executor, lab, adapters, orchestrator


def close_session(session):
    import rclpy
    executor, _, _, orchestrator = session
    orchestrator.close()
    executor.destroy_node()
    executor.motion.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def run_steps(orchestrator, steps, auto_yes):
    """Run each (from, to) through the orchestrator; return how many finished."""
    for done, (frm, to) in enumerate(steps):
        if not ask(f'{frm} -> {to}: go?', auto_yes):
            say('RESULT', {'outcome': 'declined', 'step': f'{frm} {to}'})
            return done
        record = orchestrator.run(f'{frm} {to}')
        say('RESULT', {'run_id': record.run_id, 'outcome': record.outcome,
                       'reason': record.reason, 'message': record.message,
                       'duration_s': round(record.duration_s, 1), 'retries': record.retries})
        if record.outcome != 'succeeded':
            return done
    return len(steps)


# --- commands ----------------------------------------------------------------------------

def read_touches():
    if not os.path.exists(TOUCH_FILE):
        return {}
    with open(TOUCH_FILE) as stream:
        return json.load(stream)


def cmd_touch(args):
    from lab_table_calibration import measure_fingertip
    tip, _, tilt, width_mm, depth_mm = measure_fingertip(args.robot_ip, args.tool_index)
    touches = read_touches()
    touches[args.corner] = {'tip': [round(float(v), 4) for v in tip], 'tilt_deg': round(tilt, 2),
                            'jaw_width_mm': width_mm, 'jaw_depth_mm': depth_mm,
                            'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
    with open(TOUCH_FILE, 'w') as stream:
        json.dump(touches, stream, indent=2)
    say('TOUCH', dict(corner=args.corner, recorded=sorted(touches), **touches[args.corner]))
    if tilt > 5:
        say('WARNING', f'the RG2 is tilted {tilt:.1f} deg from straight down; '
                       'touch with a vertical RG2')
    return 0


def cmd_fit(args):
    touches = read_touches()
    missing = [c for c in CORNERS if c not in touches]
    if missing:
        say('BLOCKED', f'corners not touched yet: {", ".join(missing)}')
        return 2
    box = touches.get(BOX, {}).get('tip')
    geometry = BoardGeometry([touches[c]['tip'] for c in CORNERS], box,
                             notes={'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
                                    'touches': touches})
    complaints = geometry.complaints()
    for text in complaints:
        say('WARNING', text)
    if complaints and not args.force:
        say('BLOCKED', 're-touch the corner that disagrees (or --force to keep these)')
        return 1
    geometry.save(args.geometry)
    say('FIT', {'square_mm': round(geometry.square_m * 1000, 1),
                'top_z': round(geometry.top_z, 4),
                'file_yaw_deg': round(math.degrees(geometry.file_yaw()), 1),
                'box': None if box is None else [round(v, 4) for v in box],
                'file': os.path.relpath(args.geometry)})
    return 0


def cmd_squares(args):
    geometry = BoardGeometry.load(args.geometry)
    say('BOARD', {'square_mm': round(geometry.square_m * 1000, 1),
                  'top_z': round(geometry.top_z, 4),
                  'file_yaw_deg': round(math.degrees(geometry.file_yaw()), 1),
                  'box': None if geometry.box is None else geometry.box.round(4).tolist()})
    print('centres in base_link, mm:')
    for rank in '87654321':
        cells = []
        for file in 'abcdefgh':
            x, y = geometry.square_xy(file + rank)
            cells.append(f'{file}{rank} ({x * 1000:4.0f},{y * 1000:4.0f})')
        print('  '.join(cells))
    return 0


def cmd_goto(args):
    from lab_pick import tool_yaw
    session = open_session(args, chess.Board())
    executor, lab, adapters, _ = session
    try:
        geometry = adapters.geometry
        x, y = geometry.square_xy(args.square)
        thickness = max(0.0, geometry.top_z - executor.table_z(x, y))
        yaw, _ = tool_yaw(executor.fk(executor.fresh()))
        say('TARGET', {'square': args.square, 'xy': [round(x, 4), round(y, 4)],
                       'board_top_z': round(geometry.top_z, 4),
                       'thickness_mm': round(thickness * 1000, 1)})
        hover = thickness + args.hover
        lab._move(lambda: executor.move_pose('hover', executor.pose_at(x, y, hover, yaw)))
        if args.touch:
            if not executor.plan_only:
                # 10 mm, not 0: empty jaws driven shut stall on each other and
                # the RG2 flags a grip (lab_pick.verify_marker).
                lab.gripper.move(10.0, 10.0)
            lab._move(lambda: executor.move_pose(
                'point', executor.pose_at(x, y, thickness + 0.010, yaw), avoid_collisions=False))
            if not executor.plan_only and not args.yes:
                input(f'Fingertips 10 mm over {args.square}. Measure the offset from the '
                      'square centre, then press Enter: ')
            lab._move(lambda: executor.move_pose(
                'back_up', executor.pose_at(x, y, hover, yaw), avoid_collisions=False))
        if not args.stay:
            lab.home()
        return 0
    finally:
        close_session(session)


def cmd_move(args):
    board = chess.Board(args.fen) if args.fen else chess.Board()
    try:
        move = parse_move(board, args.move)
    except ValueError as error:
        say('BLOCKED', str(error))
        return 2
    steps, note = steps_for(board, move)
    say('MOVE', {'san': board.san(move), 'steps': [f'{a} {b}' for a, b in steps], 'note': note})
    if args.steps_only:
        return 0
    session = open_session(args, board)
    try:
        done = run_steps(session[3], steps, args.yes)
        if done == len(steps) and note:
            say('NOTE', note)
        return 0 if done == len(steps) else 1
    finally:
        close_session(session)


def cmd_play(args):
    board = chess.Board(args.fen) if args.fen else chess.Board()
    arm = chess.WHITE if args.arm_white else chess.BLACK
    engine = Engine(args.engine, args.skill, args.think, args.seed)
    say('ENGINE', engine.name)
    session = None if args.steps_only else open_session(args, board)
    try:
        while not board.is_game_over(claim_draw=True):
            print(f'\n{board}\n')
            side = 'White' if board.turn == chess.WHITE else 'Black'
            if board.turn != arm:
                text = input(f'Student ({side}): move, e.g. e5 or e7e5 (quit to stop): ').strip()
                if text.lower() in ('quit', 'resign', 'q'):
                    say('GAME', 'stopped by the operator')
                    return 0
                try:
                    move = parse_move(board, text)
                except ValueError as error:
                    say('REFUSED', str(error))
                    continue
                say('STUDENT', board.san(move))
                board.push(move)
                continue
            move = engine.choose(board)
            steps, note = steps_for(board, move)
            say('ARM', {'san': board.san(move), 'steps': [f'{a} {b}' for a, b in steps]})
            if session is not None:
                if not args.yes:
                    input('Everyone clear of the board? Press Enter to let the arm move: ')
                remaining = list(steps)
                while remaining:
                    done = run_steps(session[3], remaining, args.yes)
                    remaining = remaining[done:]
                    if not remaining:
                        break
                    answer = input('The arm did not finish. r = retry from this step, '
                                   'h = I finished it by hand, q = quit: ').strip().lower()
                    if answer == 'h':
                        break
                    if answer != 'r':
                        say('GAME', 'stopped after a failed move')
                        return 1
            if note:
                say('NOTE', note)
                if session is not None and not args.yes:
                    input('Swapped? Press Enter: ')
            board.push(move)
        say('GAME', {'result': board.result(claim_draw=True), 'moves': len(board.move_stack)})
        return 0
    finally:
        engine.close()
        if session is not None:
            close_session(session)


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--geometry', default=GEOMETRY_FILE, help='board geometry (lab_chess.json)')
    cli.add_argument('--pieces', default=PIECES_FILE, help='piece table (lab_chess.yaml)')
    sub = cli.add_subparsers(dest='command', required=True)

    p = sub.add_parser('touch', help='record the fingertip on a board corner or the box rim')
    p.add_argument('--corner', required=True, choices=CORNERS + (BOX,))
    p.add_argument('--robot-ip', default='192.168.56.101')
    p.add_argument('--tool-index', type=int, default=2,
                   help='OnRobot device index of the RG2 (as lab_table_calibration.py)')
    p.set_defaults(run=cmd_touch)
    p = sub.add_parser('fit', help='the four touches -> lab_chess.json')
    p.add_argument('--force', action='store_true', help='keep touches that disagree')
    p.set_defaults(run=cmd_fit)
    p = sub.add_parser('squares', help='print every square centre (no robot)')
    p.set_defaults(run=cmd_squares)

    try:
        from lab_pick import add_session_arguments      # needs ROS; the three commands below do too
    except ImportError as error:
        add_session_arguments = None
        reason = f'(unavailable here: {error.name or error})'
    arm = []
    p = sub.add_parser('goto', help='hover over a square; --touch points at it 10 mm up')
    p.add_argument('square')
    p.add_argument('--touch', action='store_true')
    p.add_argument('--stay', action='store_true', help='do not return home afterwards')
    p.set_defaults(run=cmd_goto)
    arm.append(p)
    p = sub.add_parser('move', help='one move through the orchestrator (plan-only by default)')
    p.add_argument('move', help='SAN or UCI, e.g. Nf3 or g1f3')
    p.add_argument('--fen', help='position to move from (default: the start)')
    p.add_argument('--steps-only', action='store_true', help='print the steps, no robot')
    p.set_defaults(run=cmd_move)
    arm.append(p)
    p = sub.add_parser('play', help='a match; the student types their move, the arm plays')
    p.add_argument('--fen')
    p.add_argument('--arm-white', action='store_true',
                   help='the arm plays White (default: Black)')
    p.add_argument('--engine', default='stockfish', help='UCI engine binary, or none')
    p.add_argument('--skill', type=int, default=1, help='Stockfish Skill Level 0..20')
    p.add_argument('--think', type=float, default=0.5, help='seconds per engine move')
    p.add_argument('--seed', type=int, help='random mover seed')
    p.add_argument('--steps-only', action='store_true', help='play in the terminal, no robot')
    p.set_defaults(run=cmd_play)
    arm.append(p)
    for p in arm:
        if add_session_arguments is not None:
            add_session_arguments(p)
        else:
            p.add_argument('--yes', action='store_true')
            p.description = f'{p.prog}: robot options {reason}'
    args = cli.parse_args()
    if getattr(args, 'command', '') in ('goto', 'move', 'play') and add_session_arguments is None \
            and not getattr(args, 'steps_only', False):
        say('BLOCKED', f'{args.command} needs the ROS graph {reason}')
        return 2
    try:
        return args.run(args)
    except (FileNotFoundError, ValueError) as error:
        say('BLOCKED', str(error))
        return 2
    except StageError as error:
        say('BLOCKED', f'{error.reason}: {error.message}')
        return 1
    except KeyboardInterrupt:
        say('ABORTED', 'interrupted')
        return 130


if __name__ == '__main__':
    sys.exit(main())
