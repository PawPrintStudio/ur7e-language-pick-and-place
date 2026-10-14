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
from contextlib import contextmanager
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
import chess_gestures  # noqa: E402

GEOMETRY_FILE = os.path.join(_HERE, 'lab_chess.json')
ZED_CONF = os.path.join(_HERE, '..', 'docs', 'calibration', 'zed2i_SN35717973.conf')
OVERLAY_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'overlay.jpg')
TARGET_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'target.json')
EVENTS_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'events.jsonl')
SAID_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'said.json')     # chess_voice.py's current line
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


# --- the operator's answers: from the dashboard or the terminal, whichever is first ----

ANSWER_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'answer.json')
GO, STOP, NEW_GAME, QUIT = 'y', 'n', 'new', 'quit'
_stdin_closed = False


class NewGame(Exception):
    """The operator asked for a fresh game (a new group at the board)."""


class QuitGame(Exception):
    """The operator stopped the match."""


def option(key, label, style=''):
    return {'key': key, 'label': label, 'style': style}


def _answer_from_file(prompt_id):
    """The dashboard's answer to this prompt, or None. A stale answer is dropped."""
    try:
        with open(ANSWER_FILE) as stream:
            data = json.load(stream)
    except (OSError, ValueError):
        return None
    answer = str(data.get('answer', '')).strip()
    wildcard = data.get('id') == '*' and answer.lower() in ('new', 'new game', 'quit', 'q')
    if data.get('id') != prompt_id and not wildcard:
        try:
            os.remove(ANSWER_FILE)          # stale: a click for an earlier question
        except FileNotFoundError:
            pass
        return None
    try:
        os.remove(ANSWER_FILE)
    except FileNotFoundError:
        pass
    return answer


def _line_from_terminal(timeout_s):
    """A line typed in the terminal within ``timeout_s``, or None (also at EOF)."""
    global _stdin_closed
    if _stdin_closed:
        time.sleep(timeout_s)
        return None
    import select
    try:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    except (OSError, ValueError):            # no usable stdin (a service, a test)
        _stdin_closed = True
        return None
    if not ready:
        return None
    line = sys.stdin.readline()
    if line == '':
        _stdin_closed = True
        return None
    return line.strip()


def ask_operator(kind, text, options, allow_text=False, auto=None, poll=None,
                 timeout_s=None, default=None):
    """Show a prompt on the operator page and in the terminal; return the answer.

    The answer is an option key (``GO``, ``STOP``, ...) or, with ``allow_text``,
    whatever was typed. Every prompt carries an id, so a click left over from an
    earlier question never answers this one; the page's standing New game and
    Quit buttons answer with id ``*`` and are taken by any prompt. ``poll`` is
    called while waiting and may answer itself (the camera reading the
    student's move); ``timeout_s`` answers ``default`` when nobody did (a
    confirmation that stands unless the operator says no).

    Why a file and not a socket: ``watch`` already serves the page and owns the
    only HTTP port; ``play`` is a separate process (often another terminal).
    The same state.json hand-off the dashboard reads is enough for one operator.
    """
    if auto is not None:
        return auto
    prompt_id = f'{time.time():.3f}-{random.randrange(1 << 20):x}'
    keys = {o['key']: o['label'] for o in options}
    started = time.time()
    publish_state(prompt={'id': prompt_id, 'kind': kind, 'text': text, 'options': options,
                          'allow_text': allow_text, 'timeout_s': timeout_s, 'default': default},
                  beat=started)
    hint = ', '.join(f'{k or "Enter"} = {v}' for k, v in keys.items())
    if allow_text:
        hint += ', or type it'
    if timeout_s:
        hint += f', {keys.get(default, default)} in {timeout_s:.0f} s'
    print(f'{text} [{hint}] ', end='', flush=True)
    try:
        last_beat = time.time()
        while True:
            answer = _answer_from_file(prompt_id)
            if answer is None:
                answer = _line_from_terminal(0.2)
            if answer is None and poll is not None:
                answer = poll()
            if answer is None and timeout_s and time.time() - started > timeout_s:
                answer = default
            if answer is None:
                if time.time() - last_beat > 1.0:
                    publish_state(beat=time.time())
                    last_beat = time.time()
                continue
            print()
            low = answer.lower()
            # New game and Quit are standing buttons on the operator page:
            # accepted on every prompt, so a group can leave mid-move. The
            # arm finishes the step it is on; nothing is interrupted.
            if low in ('new', 'new game'):
                raise NewGame
            if low in ('quit', 'q', 'resign'):
                raise QuitGame
            if low in ('y', 'yes', 'go'):
                low = GO
            if low in keys:
                return low
            if allow_text:
                return answer
            print(f'{answer!r} is not one of: {hint}')
    finally:
        publish_state(prompt=None, beat=time.time())


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
        self.top_z = float(np.mean(self.corners[:, 2]))    # the board's top surface, on average
        # The top as a plane, z = a + b x + c y: a board on a table that is not
        # level with the robot base is tilted (2026-10-10: 0.8 deg, 4 mm across).
        design = np.c_[np.ones(4), self.corners[:, :2]]
        self.plane = np.linalg.lstsq(design, self.corners[:, 2], rcond=None)[0]

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
        if os.path.exists(path):
            # The camera's pose (overlay) is relative to the robot, not the board:
            # a re-fit keeps it.
            with open(path) as stream:
                camera = json.load(stream).get('camera')
            if camera is not None:
                data['camera'] = camera
        with open(path, 'w') as stream:
            json.dump(data, stream, indent=2)
            stream.write('\n')

    def z_at(self, x, y):
        """The board's top surface under (x, y), base_link metres."""
        return float(self.plane[0] + self.plane[1] * x + self.plane[2] * y)

    def tilt_deg(self):
        return math.degrees(math.atan(math.hypot(self.plane[1], self.plane[2])))

    def square_xy(self, name):
        if not SQUARE.match(name):
            raise ValueError(f'{name!r} is not a square (a1..h8)')
        u = (ord(name[0]) - ord('a') + 0.5) / 8      # along the files, corner 1 -> 2
        v = (int(name[1]) - 1 + 0.5) / 8              # along the ranks, corner 1 -> 4
        x, y, _ = self.point(u, v)
        return float(x), float(y)

    def point(self, u, v):
        """Board point at fraction u along the files (a->h), v along the ranks (1->8)."""
        c1, c2, c3, c4 = self.corners[:, :2]
        x, y = (1 - u) * (1 - v) * c1 + u * (1 - v) * c2 + u * v * c3 + (1 - u) * v * c4
        return np.array([x, y, self.z_at(x, y)])

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
        # Seen from above, a1 -> h1 -> h8 always turns counter-clockwise; clockwise
        # means two corners were swapped and every square would be mirrored.
        e1, e2 = c[1, :2] - c[0, :2], c[2, :2] - c[1, :2]
        if e1[0] * e2[1] - e1[1] * e2[0] < 0:
            out.append('a1 -> h1 -> h8 turns clockwise seen from above: corners 2 and 4 '
                       'are swapped (h1 must be on the right of a1, from white\'s side)')
        # A tilted board is fine (heights come from the plane); four touches
        # that are not on one plane mean one of them slipped or pressed in.
        # With four corners, one touch d off shows as d/4 at every corner.
        off = 4 * max(abs(z - self.z_at(x, y)) for x, y, z in c)
        if off > flat_tol_m:
            out.append(f'a corner is about {off * 1000:.1f} mm off the plane of the others: '
                       'one touch slipped or pressed harder')
        return out


# --- the camera's view of the board ---------------------------------------------------

def inner_corners(geometry):
    """The 49 inner grid corners in base_link, [file boundary i][rank boundary j]."""
    return np.array([[geometry.point(i / 8, j / 8) for j in range(1, 8)] for i in range(1, 8)])


def visible_grids(gray):
    """Every distinct patch of the board's inner-corner grid found, largest first, (rows, cols, 2) px.

    Pieces and the gripper hide part of the grid; any rectangle of at least 3 x 3
    corners will do. A wooden board has little contrast (walnut on maple), so
    the search also runs on a locally contrast-enhanced copy. OpenCV's detector
    answers differently from call to call on the same image (2026-10-10: 6x3,
    3x5, 7x3 on one frame), so it gets several tries.
    """
    import cv2
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_LARGER
    enhanced = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
    found_grids = []
    for source in (enhanced, gray, enhanced, gray):
        for scale in (1, 2):           # a small board in a low-resolution frame: upscaled
            image = source if scale == 1 else cv2.resize(source, None, fx=scale, fy=scale,
                                                         interpolation=cv2.INTER_CUBIC)
            for pattern in ((7, 7), (7, 5), (7, 3), (5, 3), (3, 3)):
                ok, found, meta = cv2.findChessboardCornersSBWithMeta(image, pattern, flags)
                if not ok:
                    continue
                # meta marks each corner that is the top-left of a square; the
                # grid ends one row and column past the last marked one. With
                # CALIB_CB_LARGER anything beyond that is padding, often off the board.
                grid = found.reshape(meta.shape[0], meta.shape[1], 2) / scale
                rows, cols = np.where((meta != 0).any(1))[0], np.where((meta != 0).any(0))[0]
                grid = grid[rows[0]:rows[-1] + 2, cols[0]:cols[-1] + 2]
                if min(grid.shape[:2]) >= 2 and not any(
                        g.shape == grid.shape and np.abs(g - grid).max() < 2 for g in found_grids):
                    found_grids.append(grid)
                break
    return sorted(found_grids, key=lambda g: -g.size)


def visible_grid(gray):
    """The largest patch of the board's grid the camera sees (see visible_grids), or None."""
    grids = visible_grids(gray)
    return grids[0] if grids else None


def look_score(gray, geometry, pose, matrix, dist, position):
    """How well the image matches the position for a candidate camera pose (grey levels).

    Empty squares must show the light/dark pattern; squares with white pieces
    must look lighter than squares with black ones.
    """
    import cv2
    camera_from_base = np.linalg.inv(pose)
    rvec, tvec = cv2.Rodrigues(camera_from_base[:3, :3])[0], camera_from_base[:3, 3]
    h, w = gray.shape

    def sample(point):
        (x, y), = cv2.projectPoints(point.reshape(1, 3), rvec, tvec, matrix,
                                    np.asarray(dist, float))[0].reshape(1, 2)
        x, y = int(round(x)), int(round(y))
        if not (2 <= x < w - 2 and 2 <= y < h - 2):
            return None
        return float(np.median(gray[y - 2:y + 3, x - 2:x + 3]))

    groups = {'light': [], 'dark': [], 'white': [], 'black': []}
    for square in chess.SQUARES:
        f, r = chess.square_file(square), chess.square_rank(square)
        centre = geometry.point((f + 0.5) / 8, (r + 0.5) / 8)
        piece = position.piece_at(square)
        if piece is None:
            value, key = sample(centre), 'dark' if (f + r) % 2 == 0 else 'light'
        else:
            value = sample(centre + np.array([0, 0, 0.02]))   # the piece's body
            key = 'white' if piece.color == chess.WHITE else 'black'
        if value is not None:
            groups[key].append(value)
    mean = {k: (np.mean(v) if v else 0.0) for k, v in groups.items()}
    return (mean['light'] - mean['dark']) + 0.5 * (mean['white'] - mean['black'])


def locate_camera(geometry, gray, matrix, dist, position=None, flip=False, previous=None,
                  with_world=False, every=False, grids=None):
    """Camera pose from whatever part of the board's grid is visible.

    Returns (base_from_camera 4x4, the corners used (px), rms px, corners used).
    A patch of grid fits anywhere on the board, facing any way; the candidate
    whose empty squares and pieces look like ``position`` (default: the start)
    wins. An empty board scores the same turned 180 deg; then the candidate
    nearer ``previous`` (the last camera position, xyz) wins, since cameras do
    not turn around between runs. ``flip`` takes the other one.
    """
    import cv2
    position = position or chess.Board()
    grids = visible_grids(gray) if grids is None else grids
    if not grids:
        raise RuntimeError('no part of the board grid found: check the board is in view '
                           'and lit evenly')
    inner = inner_corners(geometry)
    dist = np.asarray(dist, float)
    # The detector sometimes invents a patch; the next one found may be real.
    for grid in grids:
        candidates, best_rms, max_rms = _placements(grid, inner, geometry, gray, matrix, dist,
                                                    position)
        if candidates:
            break
    if not candidates:
        raise RuntimeError(f'the corners found do not form the board ({best_rms:.0f} px off, '
                           f'{max_rms:.1f} allowed): check nothing covers the board, try again')
    candidates.sort(key=lambda c: -c[0])
    if every:                     # track_board decides among them itself
        return [(pose, image, rms, image.shape[0] * image.shape[1], world)
                for _, rms, pose, image, world in candidates]
    if (previous is not None and len(candidates) > 1
            and candidates[1][0] > 0.9 * candidates[0][0]):        # a tie: the 180 deg twin
        near = lambda c: np.linalg.norm(c[2][:3, 3] - np.asarray(previous))  # noqa: E731
        candidates[:2] = sorted(candidates[:2], key=near)
    score, rms, pose, image, world = candidates[min(int(flip), len(candidates) - 1)]
    if with_world:
        return pose, image, rms, image.shape[0] * image.shape[1], world
    return pose, image, rms, image.shape[0] * image.shape[1]


def _placements(grid, inner, geometry, gray, matrix, dist, position):
    """Every way a detected patch can sit on the board that fits: (candidates, best, allowed)."""
    import cv2
    steps = np.r_[np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(),
                  np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel()]
    # A low-resolution or cluttered frame makes the detector invent corners off
    # the board (2026-10-10, 672x376 with pieces on); those fit no placement.
    max_rms = 0.1 * float(np.median(steps))
    candidates, best_rms = [], np.inf
    for k in range(4):
        for image in (np.rot90(grid, k), np.rot90(grid, k).transpose(1, 0, 2)):
            cols, rows = image.shape[:2]            # along the files, along the ranks
            pts = np.ascontiguousarray(image.reshape(-1, 2), dtype=np.float64)
            for i0 in range(8 - cols):
                for j0 in range(8 - rows):
                    world = np.ascontiguousarray(
                        inner[i0:i0 + cols, j0:j0 + rows].reshape(-1, 3), dtype=np.float64)
                    good, rvec, tvec = cv2.solvePnP(world, pts, matrix, dist)
                    if not good:
                        continue
                    rotation = cv2.Rodrigues(rvec)[0]
                    pose = np.eye(4)
                    pose[:3, :3] = rotation.T
                    pose[:3, 3] = (-rotation.T @ tvec).ravel()
                    if pose[2, 3] <= geometry.z_at(pose[0, 3], pose[1, 3]):
                        continue                    # a mirror image: camera under the board
                    projected = cv2.projectPoints(world, rvec, tvec, matrix, dist)[0]
                    rms = float(np.sqrt(np.mean(np.sum(
                        (projected.reshape(-1, 2) - pts) ** 2, axis=1))))
                    best_rms = min(best_rms, rms)
                    if rms > max_rms:
                        continue
                    score = look_score(gray, geometry, pose, matrix, dist, position)
                    candidates.append((score, rms, pose, image, world))
    return candidates, best_rms, max_rms


def draw_overlay(bgr, geometry, base_from_camera, matrix, dist, detected=None, labels='all'):
    """The calibrated squares drawn over the camera image."""
    import cv2
    camera_from_base = np.linalg.inv(base_from_camera)
    rvec = cv2.Rodrigues(camera_from_base[:3, :3])[0]
    tvec = camera_from_base[:3, 3]

    def px(points):
        flat = np.asarray(points, float).reshape(-1, 3)
        out = cv2.projectPoints(flat, rvec, tvec, matrix, np.asarray(dist, float))[0]
        return np.round(out.reshape(-1, 2)).astype(int)

    out = bgr.copy()
    lattice = px([[geometry.point(i / 8, j / 8) for j in range(9)] for i in range(9)]).reshape(9, 9, 2)
    for k in range(9):
        cv2.polylines(out, [lattice[k, :]], False, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.polylines(out, [lattice[:, k]], False, (0, 255, 255), 1, cv2.LINE_AA)
    scale = max(0.5, np.linalg.norm(lattice[8, 0] - lattice[0, 0]) / 350)   # ~ the board's width
    for f in range(8):
        for r in range(8):
            if labels == 'edges':           # live video: a1..h1 and a..8 along two edges only
                if r != 0 and f != 0:
                    continue
            (x, y), = px([geometry.point((f + 0.5) / 8, (r + 0.5) / 8)])
            cv2.putText(out, f'{"abcdefgh"[f]}{r + 1}', (x - int(10 * scale), y + int(5 * scale)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4 * scale, (0, 0, 255), max(1, int(scale)),
                        cv2.LINE_AA)
    if detected is not None:
        for x, y in np.round(detected.reshape(-1, 2)).astype(int):
            cv2.circle(out, (x, y), 2, (0, 255, 0), -1)
    return out


# --- the camera in the loop (chess_vision.py): pieces, heights, centres, board shift ----

PAIR_SHM = '/dev/shm/lab_chess_pair.npy'   # `watch` shares its newest raw stereo pair here
EMPTY_CLOSE_MM = 9.3     # the RG2 reads this closed on nothing (2026-10-10)
BOX_RELEASE_MM = 70.0    # jaw opening when letting go over the box (nothing beside it)
SQUARE_RELEASE_MAX_MM = 42.0   # ... and the most on a square: neighbours sit 1.4 squares away


def zed_device(device=None):
    """The ZED's /dev/video number: as given, else found by name (numbers change on replug)."""
    if device is not None:
        return device
    import glob
    number = lambda path: int(os.path.basename(os.path.dirname(path))[len('video'):])  # noqa: E731
    for path in sorted(glob.glob('/sys/class/video4linux/video*/name'), key=number):
        with open(path) as stream:
            if 'ZED' in stream.read():
                return number(path)
    raise RuntimeError('no ZED among the video devices: is it plugged in?')


def grab_pair(device=None, max_age_s=1.5, newer_than=0.0):
    """A raw stereo pair (left, right RGB): from a running `watch`, else the camera.

    ``newer_than`` (a time.time()) waits for a frame `watch` shared after it,
    so a retry gets a different picture without fighting `watch` for the camera.
    """
    if os.path.exists(PAIR_SHM) and time.time() - os.path.getmtime(PAIR_SHM) < max_age_s:
        deadline = time.time() + 3.0
        while os.path.getmtime(PAIR_SHM) <= newer_than and time.time() < deadline:
            time.sleep(0.1)
        pair = np.load(PAIR_SHM)
        return pair[0], pair[1]
    import types
    from lab_table_calibration import camera_settings
    from ur7e_perception.camera import Camera
    camera = Camera(**camera_settings(types.SimpleNamespace(device=zed_device(device))))
    try:
        for _ in range(10):       # auto-exposure
            camera.read_pair()
        pair = camera.read_pair()
    finally:
        camera.release()
    if pair is None or pair[1] is None:
        raise RuntimeError('no stereo pair from the ZED (is another program using it?)')
    return pair


class Vision:
    """One stereo look at the board: per-square occupancy, height, centre (chess_vision)."""

    def __init__(self, geometry, device=None, conf=ZED_CONF, path=GEOMETRY_FILE):
        import chess_vision
        with open(path) as stream:
            saved = json.load(stream).get('camera')
        if saved is None:
            raise ValueError('no camera pose yet: run overlay once (camera fixed from then on)')
        self.cv = chess_vision
        self.camera = chess_vision.BoardCamera(geometry, saved['base_from_camera'], conf,
                                               saved['mode'])
        self.device = device
        # The scan the student's move is compared against (spec S6): taken
        # after the arm's move, or at the start of the game.
        self.baseline = None

    def look(self, tips=None):
        """Scan the board; ``tips`` (fingertips xyz) removes the gripper from the points."""
        left, right = grab_pair(self.device)
        points = self.camera.points(left, right)
        if tips is not None:
            points = self.cv.without_robot(points, tips)
        return self.cv.scan(points, self.camera), points, left


def executor_tips(executor):
    """Fingertips (base xyz) from the arm's present pose; the tool points down."""
    pose = executor.fk(executor.fresh()).position
    return (pose.x, pose.y, pose.z - executor.tooling['tcp_to_tips_m'])


def board_text(squares):
    """The scan as 8 lines: height in mm where a piece stands, . empty, ? hidden."""
    rows = []
    for r in range(7, -1, -1):
        cells = []
        for f in range(8):
            e = squares[f'{"abcdefgh"[f]}{r + 1}']
            cells.append(f"{1000 * e['top_m']:3.0f}" if e['occupied']
                         else (' . ' if e['seen'] else ' ? '))
        rows.append(f'{r + 1} ' + ' '.join(cells))
    return '\n'.join(rows + ['   a   b   c   d   e   f   g   h'])


def cmd_scan(args):
    """Look at the board once: what stands where, how tall, how far off centre."""
    geometry = BoardGeometry.load(args.geometry)
    vision = Vision(geometry, args.device, args.conf, args.geometry)
    tips = None
    if not args.no_robot:
        import lab_tooling
        from lab_table_calibration import tool_pose
        tips = (tool_pose() @ lab_tooling.tool0_to_tips())[:3, 3]
    squares, points, _ = vision.look(tips)
    print(board_text(squares))
    found = {}
    for name, e in squares.items():
        if e['occupied']:
            cx, cy = geometry.square_xy(name)
            found[name] = {'top_mm': round(1000 * e['top_m'], 1),
                           'off_centre_mm': [round(1000 * (e['centre'][0] - cx), 1),
                                             round(1000 * (e['centre'][1] - cy), 1)]}
    say('SCAN', {'pieces': found, 'hidden': [n for n, e in squares.items() if not e['seen']],
                 'disparity_offset_px': round(points['offset_px'], 2),
                 'board_noise_mm': round(points['board_noise_mm'] or 0, 1)})
    return 0


def track_board(geometry, device=None, conf=ZED_CONF, position=None, path=GEOMETRY_FILE):
    """Where the board is now, from the grid seen by the fixed camera: (shift 4x4, mm, deg, px)."""
    import cv2
    import chess_vision
    from ur7e_perception.camera import zed_factory_calibration
    with open(path) as stream:
        saved = json.load(stream).get('camera')
    if saved is None:
        raise ValueError('no camera pose yet: run overlay once')
    base_from_camera = np.array(saved['base_from_camera'])
    matrix, dist, _ = zed_factory_calibration(conf, saved['mode'])
    # The grid detector is random: a frame can yield only bad patches (2026-10-10:
    # 11 px off, then fine on the next frame). Try a few frames before giving up.
    asked = 0.0
    for attempt in range(4):
        left, _ = grab_pair(device, newer_than=asked)
        asked = time.time()
        try:
            return _track_frame(geometry, cv2.cvtColor(left, cv2.COLOR_RGB2GRAY), matrix, dist,
                                base_from_camera, position)
        except RuntimeError as error:
            last = error
    raise last


def board_region_grids(geometry, gray, matrix, dist, base_from_camera, margin_px=20):
    """Find conventional grids near the saved board, excluding background patterns.

    Coordinates are returned in the original image; the camera intrinsics and
    the downstream pose/board-motion acceptance thresholds remain unchanged.
    ``margin_px`` widens the crop when the board may have moved since the pose
    was saved (a re-touch after the stand moved: overlay uses 80).
    """
    import cv2
    camera_from_base = np.linalg.inv(base_from_camera)
    world = np.array([geometry.point(u, v)
                      for u, v in ((0, 0), (1, 0), (1, 1), (0, 1))])
    projected = cv2.projectPoints(world, cv2.Rodrigues(camera_from_base[:3, :3])[0],
                                 camera_from_base[:3, 3], matrix,
                                 np.asarray(dist, float))[0].reshape(-1, 2)
    if not np.isfinite(projected).all():
        return []
    low = np.maximum(np.floor(projected.min(0) - margin_px).astype(int), 0)
    high = np.minimum(np.ceil(projected.max(0) + margin_px).astype(int),
                      [gray.shape[1], gray.shape[0]])
    if np.any(high - low < 16):
        return []
    crop = gray[low[1]:high[1], low[0]:high[0]]
    grids = []
    for cols, rows in ((7, 7), (3, 7), (7, 3), (3, 5), (5, 3)):
        ok, points = cv2.findChessboardCorners(
            crop, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            continue
        points = cv2.cornerSubPix(crop, points, (5, 5), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
        grids.append(points.reshape(rows, cols, 2) + low)
        if rows == cols == 7:
            break
    return grids


def _track_frame(geometry, gray, matrix, dist, base_from_camera, position):
    """track_board on one grey frame."""
    import chess_vision
    # Which detected corner is which is ambiguous on a repeating, nearly empty
    # board (turned 180 deg it looks the same: 2026-10-10 read a nudge as 150 mm
    # and a half turn). The camera is fixed, so every match that fits is turned
    # into a board motion (x, y, turn) and the smallest one is the answer.
    centre = geometry.point(0.5, 0.5)
    best = None
    position = position if position is not None else chess.Board(None)
    regional = board_region_grids(geometry, gray, matrix, dist, base_from_camera)
    try:
        candidates = locate_camera(geometry, gray, matrix, dist, position,
                                   every=True, grids=regional)
    except RuntimeError:
        candidates = locate_camera(geometry, gray, matrix, dist, position, every=True)
    for _, image, _, used, world in candidates:
        shift, rms = chess_vision.board_motion(world, image.reshape(-1, 2), base_from_camera,
                                               matrix, dist, centre)
        mm, deg = chess_vision.shift_size(shift, centre)
        size = mm + 2.5 * abs(deg)          # 1 deg turns the board's edge about 2.5 mm
        if rms < 2.0 and (best is None or size < best[0]):
            best = (size, shift, mm, deg, rms, used)
    if best is None:
        raise RuntimeError('the grid matched, but no board position fits it well')
    return best[1:]


def cmd_track(args):
    """Has the board moved since it was touched? --apply moves the calibration with it."""
    import chess_vision
    geometry = BoardGeometry.load(args.geometry)
    position = chess.Board(args.fen) if args.fen else chess.Board(None)
    shift, mm, deg, rms, used = track_board(geometry, args.device, args.conf, position,
                                            args.geometry)
    say('TRACK', {'board_moved_mm': round(mm, 1), 'turned_deg': round(deg, 2),
                  'grid_fit_px': round(rms, 2), 'grid_corners_used': used})
    if mm < 1.5 and abs(deg) < 0.3:
        say('OK', 'the board is where it was touched')
        return 0
    if mm > 30 or abs(deg) > 5:
        say('BLOCKED', 'moved too far to follow safely: put it back or touch the corners again')
        return 1
    if not args.apply:
        say('HINT', 'run again with --apply to move the calibration with the board')
        return 0
    follow_board(geometry, shift, mm, deg, args.geometry)
    return 0


def follow_board(geometry, shift, mm, deg, path=GEOMETRY_FILE):
    """Move the calibration (in memory and on disk) by a board shift the camera measured.

    The box does not move: it is not on the board.
    """
    import chess_vision
    moved = BoardGeometry(chess_vision.shifted_corners(geometry.corners, shift), geometry.box,
                          dict(geometry.notes, tracked={'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
                                                        'moved_mm': round(mm, 1),
                                                        'turned_deg': round(deg, 2)}))
    moved.save(path)
    geometry.__dict__.update(moved.__dict__)        # every holder of this object sees it
    say('UPDATED', {'file': os.path.relpath(path), 'moved_mm': round(mm, 1),
                    'turned_deg': round(deg, 2)})


def check_board(geometry, position, args):
    """Session start with the camera: has the board moved since it was calibrated?"""
    try:
        shift, mm, deg, rms, used = track_board(geometry, getattr(args, 'device', None),
                                                position=position)
    except RuntimeError as error:
        say('WARNING', f'board position not checked: {error}')
        return
    say('TRACK', {'board_moved_mm': round(mm, 1), 'turned_deg': round(deg, 2),
                  'grid_fit_px': round(rms, 2), 'grid_corners_used': used})
    if mm < 1.5 and abs(deg) < 0.3:
        return
    if mm > 30 or abs(deg) > 5:
        raise ValueError('the board moved too far to follow safely: put it back or touch '
                         'the corners again')
    answer = 'y' if args.yes else input(
        f'The board sits {mm:.1f} mm / {deg:.1f} deg from its calibration (camera assumed '
        'fixed). Follow it? [y/N] ').strip().lower()
    if answer.startswith('y'):
        follow_board(geometry, shift, mm, deg, args.geometry)


# --- the live view: is the arm over the square it was sent to? -----------------------

STATE_FILE = os.path.join(_HERE, '..', 'log', 'chess', 'state.json')
DASHBOARD_HTML = os.path.join(_HERE, 'chess_dashboard.html')


def publish_state(**fields):
    """Merge fields into the shared game state (position, planned move) the views read.

    A move ending (planned=None) also clears the arm's target: a leftover target
    read as 'OFF TARGET' on an idle arm (2026-10-11).
    """
    if 'planned' in fields and fields['planned'] is None and os.path.exists(TARGET_FILE):
        os.remove(TARGET_FILE)
    state = read_state()
    state.update(fields, stamp=time.strftime('%Y-%m-%dT%H:%M:%S'))
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    with open(STATE_FILE + '.tmp', 'w') as stream:
        json.dump(state, stream)
    os.replace(STATE_FILE + '.tmp', STATE_FILE)


def read_state():
    try:
        with open(STATE_FILE) as stream:
            return json.load(stream)
    except (OSError, ValueError):
        return {}


@contextmanager
def game_state_lock():
    """Keep reset from replacing the record while a chess command owns it."""
    import fcntl
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    with open(STATE_FILE + '.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('another chess command is active; quit T3 before resetting '
                               'or starting a new game') from None
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def new_game_state(board, arm_white=False):
    """Replace the record completely; never retain a previous failure or grip."""
    state = dict(fen=board.fen(), planned=None, moves=move_list(board),
                 arm='white' if arm_white else 'black',
                 stamp=time.strftime('%Y-%m-%dT%H:%M:%S'))
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    with open(STATE_FILE + '.tmp', 'w') as stream:
        json.dump(state, stream)
    os.replace(STATE_FILE + '.tmp', STATE_FILE)
    for stale in (TARGET_FILE, ANSWER_FILE):
        try:
            os.remove(stale)
        except FileNotFoundError:
            pass


def cmd_reset(args):
    new_game_state(chess.Board(), args.arm_white)
    say('RESET', 'new game record: standard starting position, White to move. '
                'Moves, targets and previous failure cleared. No robot motion started.')
    say('NEXT', 'restore the physical pieces to the starting position; '
               'start play separately when ready. Calibration is unchanged.')
    return 0


def move_list(board):
    """The game so far in SAN, for the dashboard."""
    replay, out = board.root(), []
    for move in board.move_stack:
        out.append(replay.san(move))
        replay.push(move)
    return out


def said_now(path=None, linger_s=20.0):
    """The voice's current line for the page: {event, line, age_s, seconds}, or None when old."""
    try:
        with open(path or SAID_FILE) as stream:
            row = json.load(stream)
        age = time.time() - float(row['t'])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if age > float(row.get('seconds', 3.0)) + linger_s:
        return None
    return {'event': row.get('event'), 'line': row.get('line'), 'age_s': round(age, 1),
            'seconds': row.get('seconds')}


def serve_dashboard(port, latest, lan=False):
    """The browser dashboard: the page, the annotated video, and the state as JSON.

    Read-only apart from 'scan now': the arm is confirmed in the terminal.
    Bound to this computer unless ``lan`` (then anyone on the network can watch).
    """
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class Handler(BaseHTTPRequestHandler):
        def send(self, body, kind):
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802 - name fixed by http.server
            if self.path.startswith('/piece?'):
                import cv2
                from urllib.parse import parse_qs, urlsplit
                square = parse_qs(urlsplit(self.path).query).get('square', [''])[0]
                snapshot = latest.get('camera_snapshot')
                box = snapshot[1]['crops'].get(square) if snapshot else None
                if box is None:
                    self.send_error(404, 'No visible piece on this square')
                    return
                x, y, w, h = box
                crop = snapshot[0][y:y+h, x:x+w]
                self.send(cv2.imencode('.jpg', crop)[1].tobytes(), 'image/jpeg')
            elif self.path.startswith('/stream'):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        jpeg = latest.get('board_jpeg' if self.path == '/stream?view=board' else 'jpeg')
                        if jpeg:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                                             + jpeg + b'\r\n')
                        time.sleep(0.15)
                except (BrokenPipeError, ConnectionResetError):
                    return
            elif self.path.startswith('/state'):
                state = read_state()
                # How long since play last said it is alive: the operator page
                # greys its buttons out when the game process is gone.
                beat = state.get('beat')
                state['prompt_age_s'] = None if beat is None else round(time.time() - beat, 1)
                state['voice'] = said_now()
                self.send(json.dumps(dict(state, live=latest.get('live', {}))).encode(),
                          'application/json')
            else:
                with open(DASHBOARD_HTML, 'rb') as stream:
                    self.send(stream.read(), 'text/html; charset=utf-8')

        def do_POST(self):  # noqa: N802
            if self.path.startswith('/scan'):
                latest['scan_now'] = True
            elif self.path.startswith('/reset'):
                # The standing New game button when no game is running: the
                # same record reset as `lab_chess.py reset`, refused while a
                # chess command owns the record (answer that prompt instead).
                if self.client_address[0] not in ('127.0.0.1', '::1'):
                    self.send_error(403, 'Reset is accepted from this computer only')
                    return
                try:
                    with game_state_lock():
                        new_game_state(chess.Board(), read_state().get('arm') == 'white')
                except RuntimeError as error:
                    self.send_error(409, str(error))
                    return
            elif self.path.startswith('/ask'):
                # The operator's question for the robot's voice (chess_voice.py
                # answers it through the events file); an empty one asks for
                # something interesting about itself.
                if self.client_address[0] not in ('127.0.0.1', '::1'):
                    self.send_error(403, 'Questions are accepted from this computer only')
                    return
                length = int(self.headers.get('Content-Length') or 0)
                try:
                    data = json.loads(self.rfile.read(length) or b'{}')
                except ValueError:
                    data = {}
                question = str(data.get('question', '') if isinstance(data, dict) else '')[:200].strip()
                chess_gestures.log_event(EVENTS_FILE, 'ask', question=question,
                                         fen=read_state().get('fen'))
            elif self.path.startswith('/answer'):
                # The operator's click. Only from this computer: the projected
                # page is read-only for the audience, and arm motion is never
                # approved from across the network.
                if self.client_address[0] not in ('127.0.0.1', '::1'):
                    self.send_error(403, 'Operator answers are accepted from this computer only')
                    return
                length = int(self.headers.get('Content-Length') or 0)
                try:
                    data = json.loads(self.rfile.read(length) or b'{}')
                    answer = {'id': str(data['id']), 'answer': str(data.get('answer', ''))[:80],
                              'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
                except (ValueError, KeyError, TypeError):
                    self.send_error(400, 'expected {"id": ..., "answer": ...}')
                    return
                os.makedirs(os.path.dirname(ANSWER_FILE), exist_ok=True)
                with open(ANSWER_FILE + '.tmp', 'w') as stream:
                    json.dump(answer, stream)
                os.replace(ANSWER_FILE + '.tmp', ANSWER_FILE)
            self.send(b'{}', 'application/json')

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('0.0.0.0' if lan else '127.0.0.1', port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def planned_move(board, move):
    """What the arm is about to do, for the views: from, to, and whether a piece goes to the box."""
    return {'from': chess.square_name(move.from_square), 'to': chess.square_name(move.to_square),
            'capture': board.is_capture(move), 'san': board.san(move)}


def announce_target(name):
    """Tell ``watch`` where the arm is heading (a square, or 'box')."""
    os.makedirs(os.path.dirname(TARGET_FILE), exist_ok=True)
    with open(TARGET_FILE, 'w') as stream:
        json.dump({'target': name, 'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}, stream)


def tip_status(geometry, tips, target, tolerance_m=0.005):
    """Where the fingertips are against the target: square under them, error, verdict."""
    x, y, z = (float(v) for v in tips)
    names = [f'{"abcdefgh"[f]}{r + 1}' for f in range(8) for r in range(8)]
    centres = {n: np.array(geometry.square_xy(n)) for n in names}
    under = min(names, key=lambda n: np.hypot(*(centres[n] - (x, y))))
    if np.hypot(*(centres[under] - (x, y))) > 0.75 * geometry.square_m:
        under = None                                   # off the board
    status = {'under': under, 'above_board_mm': round((z - geometry.z_at(x, y)) * 1000)}
    if target == BOX and geometry.box is not None:
        goal = np.asarray(geometry.box[:2], float)
    elif target and SQUARE.match(target):
        goal = centres[target]
    else:
        return dict(status, target=None, error_mm=None, ok=None)
    error = float(np.hypot(*(goal - (x, y))))
    return dict(status, target=target, error_mm=round(error * 1000, 1),
                ok=error <= tolerance_m)


def draw_board_state(out, geometry, px, view):
    """What the camera sees against what the game says, and the arm's next move.

    * unknown depth stays on the companion board, leaving the camera unobstructed;
    * disagreement: red outline; detected pieces: open green corner brackets;
    * piece-type estimates stay on the companion board, not on the real pieces;
    * the planned move: a blue arrow, and an orange one to the box for a capture.
    """
    import cv2
    squares, colours = view.get('squares') or {}, view.get('colours') or {}
    expected = view.get('expected')

    def corners(quad, colour):
        # Short strokes at square corners leave the physical piece untouched.
        quad = np.round(quad.mean(axis=0) + (quad - quad.mean(axis=0)) * 0.78).astype(int)
        for i, vertex in enumerate(quad):
            for neighbour in (quad[(i - 1) % 4], quad[(i + 1) % 4]):
                end = np.round(vertex + 0.16 * (neighbour - vertex)).astype(int)
                cv2.line(out, tuple(vertex), tuple(end), colour, 1, cv2.LINE_AA)

    for name, entry in squares.items():
        f, r = ord(name[0]) - ord('a'), int(name[1]) - 1
        quad = px([geometry.point(u / 8, v / 8) for u, v in
                   ((f, r), (f + 1, r), (f + 1, r + 1), (f, r + 1))])
        if not entry['seen']:
            continue
        wrong = False
        if expected is not None:
            piece = expected.piece_at(chess.parse_square(name))
            wrong = (piece is not None) != entry['occupied'] or (
                piece is not None and name in colours and colours[name] != piece.color)
            if wrong:
                corners(quad, (65, 65, 225))
        if entry['occupied'] and not wrong:
            corners(quad, (80, 210, 130))
    planned = view.get('planned')
    if planned:
        a, b = (px([[*geometry.square_xy(s), geometry.z_at(*geometry.square_xy(s))]])[0]
                for s in (planned['from'], planned['to']))
        if planned.get('capture') and geometry.box is not None:
            (bx, by), = px([geometry.box])
            cv2.arrowedLine(out, tuple(int(v) for v in b), (int(bx), int(by)), (0, 140, 255), 4,
                            cv2.LINE_AA, tipLength=0.08)
        cv2.arrowedLine(out, tuple(int(v) for v in a), tuple(int(v) for v in b), (255, 120, 0), 5,
                        cv2.LINE_AA, tipLength=0.2)


def board_closeup(frame, geometry, base_from_camera, matrix, dist, with_bounds=False):
    """Frame the calibrated board plus headroom for pieces; never change CV inputs."""
    import cv2
    def result(bounds):
        x, y, w, h = bounds
        crop = frame[y:y+h, x:x+w]
        return (crop, bounds) if with_bounds else crop
    full = [0, 0, frame.shape[1], frame.shape[0]]
    camera_from_base = np.linalg.inv(base_from_camera)
    points = []
    for u, v in ((0, 0), (1, 0), (1, 1), (0, 1)):
        for height in (0, 0.12):
            point = np.array(geometry.point(u, v), dtype=float)
            point[2] += height
            points.append(point)
    points = np.asarray(points)
    camera_points = points @ camera_from_base[:3, :3].T + camera_from_base[:3, 3]
    if not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
        return (frame, full) if with_bounds else frame
    pixels = cv2.projectPoints(points, cv2.Rodrigues(camera_from_base[:3, :3])[0],
                               camera_from_base[:3, 3], matrix, np.asarray(dist, float))[0].reshape(-1, 2)
    if not np.isfinite(pixels).all():
        return (frame, full) if with_bounds else frame
    low, high = pixels.min(axis=0), pixels.max(axis=0)
    centre = (low + high) / 2
    width = max((high[0] - low[0]) * 1.25, (high[1] - low[1]) * 1.25 * 1.6)
    height = width / 1.6
    h, w = frame.shape[:2]
    width, height = min(w, width), min(h, height)
    x = int(np.clip(centre[0] - width / 2, 0, w - width))
    y = int(np.clip(centre[1] - height / 2, 0, h - height))
    crop = frame[y:y + int(height), x:x + int(width)]
    return result([x, y, int(width), int(height)]) if min(crop.shape[:2]) >= 32 else result(full)


def camera_annotations(frame, geometry, base_from_camera, matrix, dist, view, bounds):
    """Pixel coordinates for read-only browser overlays and raw object thumbnails."""
    import cv2
    transform = np.linalg.inv(base_from_camera)
    def px(points):
        return cv2.projectPoints(np.asarray(points, float), cv2.Rodrigues(transform[:3, :3])[0],
                                 transform[:3, 3], matrix, np.asarray(dist, float))[0].reshape(-1, 2)
    h, w = frame.shape[:2]
    data = dict(width=w, height=h, bounds=bounds, quads={}, crops={}, labels=[])
    for f in range(8):
        for r in range(8):
            name = f'{"abcdefgh"[f]}{r+1}'
            quad = px([geometry.point(u/8, v/8) for u, v in
                       ((f, r), (f+1, r), (f+1, r+1), (f, r+1))])
            data['quads'][name] = np.round(quad, 1).tolist()
            entry = view['squares'].get(name)
            if entry and entry['seen'] and entry['occupied']:
                x, y = entry['centre'][:2]
                z = geometry.z_at(x, y)
                top = float(np.clip(entry['top_m'], 0.01, 0.15))
                radius = geometry.square_m * 0.32
                points = px([[x+dx, y+dy, z+dz] for dx in (-radius, radius)
                             for dy in (-radius, radius) for dz in (0, top+0.008)])
                low = np.maximum(0, np.floor(points.min(axis=0)-4)).astype(int)
                high = np.minimum([w, h], np.ceil(points.max(axis=0)+4)).astype(int)
                if np.all(high-low > 2):
                    data['crops'][name] = [*low.tolist(), *(high-low).tolist()]
    for i in range(8):
        for text, u, v in (("abcdefgh"[i], (i+.5)/8, -.055), (str(i+1), -.055, (i+.5)/8)):
            data['labels'].append([text, *np.round(px([geometry.point(u, v)])[0], 1).tolist()])
    return data


def draw_watch(bgr, geometry, base_from_camera, matrix, dist, tips, status, view=None):
    """The overlay plus the target square (green: on it, red: not) and the fingertips."""
    import cv2
    # The calibration grid belongs in the setup view; it obscures a live game.
    out = bgr.copy()
    camera_from_base = np.linalg.inv(base_from_camera)
    rvec, tvec = cv2.Rodrigues(camera_from_base[:3, :3])[0], camera_from_base[:3, 3]

    def px(points):
        flat = np.asarray(points, float).reshape(-1, 3)
        return np.round(cv2.projectPoints(flat, rvec, tvec, matrix, np.asarray(dist, float))[0]
                        .reshape(-1, 2)).astype(int)

    if view:
        draw_board_state(out, geometry, px, view)

    colour = {True: (0, 200, 0), False: (0, 0, 255), None: (255, 200, 0)}[status.get('ok')]
    target = status.get('target')
    if target and SQUARE.match(target):
        f, r = ord(target[0]) - ord('a'), int(target[1]) - 1
        quad = px([geometry.point(u / 8, v / 8) for u, v in
                   ((f, r), (f + 1, r), (f + 1, r + 1), (f, r + 1))])
        cv2.polylines(out, [quad], True, colour, 3, cv2.LINE_AA)
    if tips is not None:
        tip, = px([tips])
        floor, = px([[tips[0], tips[1], geometry.z_at(tips[0], tips[1])]])
        cv2.line(out, tuple(tip), tuple(floor), colour, 2, cv2.LINE_AA)   # straight down
        cv2.circle(out, tuple(floor), 7, colour, 2, cv2.LINE_AA)
        cv2.circle(out, tuple(tip), 5, colour, -1, cv2.LINE_AA)
    return out


class BackgroundScan:
    """Stereo scans for the live view, in a thread so the video keeps running."""

    def __init__(self, geometry, base_from_camera, conf, mode, every_s):
        import threading
        self.every, self.busy, self.last = every_s, False, 0.0
        self.squares, self.lock, self.threading = {}, threading.Lock(), threading
        self.geometry, self.camera = geometry, None
        if every_s:
            import chess_vision
            self.cv = chess_vision
            self.camera = chess_vision.BoardCamera(geometry, base_from_camera, conf, mode)

    def offer(self, pair, tips, now=False):
        if self.camera is None or self.busy or (not now and time.time() - self.last < self.every):
            return
        self.busy = True
        self.threading.Thread(target=self._scan, args=(pair, tips), daemon=True).start()

    def _scan(self, pair, tips):
        try:
            points = self.camera.points(pair[0], pair[1])
            if tips is not None:
                points = self.cv.without_robot(points, tips)
            squares = self.cv.scan(points, self.camera)
            with self.lock:
                self.squares = squares
        except Exception as error:   # noqa: BLE001 - a failed scan must not stop the video
            say('WARNING', f'live scan: {error}')
        finally:
            self.last, self.busy = time.time(), False

    def view(self):
        """What draw_board_state needs: latest scan, piece colours, game state."""
        state = read_state()
        expected = chess.Board(state['fen']) if state.get('fen') else None
        with self.lock:
            squares = dict(self.squares)
        colours = {}
        if squares and self.camera is not None:
            colours = self.cv.piece_colours(squares, expected or chess.Board(None))
        return {'squares': squares, 'colours': colours, 'expected': expected,
                'planned': state.get('planned'), 'age_s': time.time() - self.last}

    @staticmethod
    def summary(view):
        """The scan for the dashboard: per square occupied/height/seen/white, and the mismatches."""
        squares, colours, expected = view['squares'], view['colours'], view['expected']
        compact, wrong = {}, []
        for name, e in squares.items():
            compact[name] = {'o': int(e['occupied']), 's': bool(e['seen']),
                             't': round(1000 * e['top_m']) if e['occupied'] else None,
                             'w': colours.get(name)}
            if expected is not None and e['seen']:
                piece = expected.piece_at(chess.parse_square(name))
                if (piece is not None) != e['occupied'] or (
                        piece is not None and name in colours and colours[name] != piece.color):
                    wrong.append(name)
        return {'squares': compact, 'wrong': sorted(wrong) if expected is not None else None,
                'scan_age_s': view['age_s'] if squares else None}


def cmd_watch(args):
    """Live camera view in the browser: green when the fingertips are on the target."""
    import types
    import cv2
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    import lab_tooling
    from lab_table_calibration import camera_settings, quaternion_matrix
    from ur7e_perception.camera import Camera, zed_factory_calibration
    geometry = BoardGeometry.load(args.geometry)
    with open(args.geometry) as stream:
        saved = json.load(stream).get('camera')
    if saved is None:
        say('BLOCKED', 'no camera pose yet: run overlay once')
        return 2
    base_from_camera = np.array(saved['base_from_camera'])
    matrix, dist, _ = zed_factory_calibration(args.conf, saved['mode'])
    camera = Camera(**camera_settings(types.SimpleNamespace(device=zed_device(args.device))))
    tool0_to_tips = lab_tooling.tool0_to_tips()
    rclpy.init()
    node = rclpy.create_node('lab_chess_watch')
    buffer = Buffer()
    TransformListener(buffer, node)
    latest = {}
    serve_dashboard(args.port, latest, args.lan)
    say('WATCH', {'open': f'http://localhost:{args.port}', 'stop': 'Ctrl+C',
                  'network': 'whole network (--lan)' if args.lan else 'this computer only'})
    shared = 0.0
    scanner = BackgroundScan(geometry, base_from_camera, args.conf, saved['mode'],
                             None if args.no_scan else args.scan_every)
    heights = {s: round(1000 * float(v['height_m']))
               for s, v in load_pieces(args.pieces)['pieces'].items()}
    geometry_stamp = os.path.getmtime(args.geometry)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.0)
            if os.path.getmtime(args.geometry) != geometry_stamp:     # track --apply, fit
                geometry.__dict__.update(BoardGeometry.load(args.geometry).__dict__)
                geometry_stamp = os.path.getmtime(args.geometry)
            pair = camera.read_pair()
            if pair is None:
                continue
            frame = pair[0]
            if pair[1] is not None and time.time() - shared > 0.5:
                # The camera has one owner; scan/track/--vision read the pair from here.
                np.save(PAIR_SHM + '.tmp.npy', np.stack(pair))
                os.replace(PAIR_SHM + '.tmp.npy', PAIR_SHM)
                shared = time.time()
            tips = None
            if buffer.can_transform('base_link', 'tool0', Time()):
                tf = buffer.lookup_transform('base_link', 'tool0', Time()).transform
                pose = np.eye(4)
                pose[:3, :3] = quaternion_matrix(tf.rotation.x, tf.rotation.y,
                                                 tf.rotation.z, tf.rotation.w)
                pose[:3, 3] = (tf.translation.x, tf.translation.y, tf.translation.z)
                tips = (pose @ tool0_to_tips)[:3, 3]
            target = None
            if os.path.exists(TARGET_FILE):
                with open(TARGET_FILE) as stream:
                    target = json.load(stream).get('target')
            status = (tip_status(geometry, tips, target) if tips is not None else {})
            if pair[1] is not None:
                scanner.offer(pair, tips, now=latest.pop('scan_now', False))
            view = scanner.view()
            out = draw_watch(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR), geometry,
                             base_from_camera, matrix, dist, tips, status, view)
            latest['jpeg'] = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            closeup, bounds = board_closeup(out, geometry, base_from_camera, matrix, dist, with_bounds=True)
            latest['board_jpeg'] = cv2.imencode('.jpg', closeup, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
            raw = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            annotations = camera_annotations(raw, geometry, base_from_camera, matrix, dist, view, bounds)
            latest['camera_snapshot'] = (raw, annotations)
            # Publish a complete snapshot once: HTTP readers must never see a
            # new scan before its camera geometry is attached (overlay flicker).
            latest['live'] = dict(scanner.summary(view), target=status, heights=heights,
                                  camera=annotations)
    except KeyboardInterrupt:
        return 0
    finally:
        camera.release()
        if os.path.exists(PAIR_SHM):
            os.remove(PAIR_SHM)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def cmd_overlay(args):
    import cv2
    from ur7e_perception.camera import ZED_MODES, zed_factory_calibration
    geometry = BoardGeometry.load(args.geometry)
    if args.image:
        bgr = cv2.imread(args.image)
    else:
        import types
        from lab_table_calibration import grab_frames
        frame = grab_frames(types.SimpleNamespace(device=zed_device(args.device)), 1.5)[-1]
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    size = (bgr.shape[1], bgr.shape[0])
    mode = next((m for m, s in ZED_MODES.items() if s == size), None)
    if mode is None:
        say('BLOCKED', f'{size[0]}x{size[1]} is not a ZED left-eye mode')
        return 2
    if mode == 'VGA':
        say('WARNING', 'the ZED delivers 672x376: it is on a USB 2 port. Use a USB 3 port')
    matrix, dist, _ = zed_factory_calibration(args.conf, mode)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    try:
        position = (chess.Board() if args.fen == 'start'
                    else chess.Board(args.fen) if args.fen else chess.Board(None))
        with open(args.geometry) as stream:
            saved = json.load(stream).get('camera')
        previous = None if saved is None else np.array(saved['base_from_camera'])[:3, 3]
        # With a pose already saved, look for the grid where the board was last
        # seen first (2026-10-13: the whole-image detector picked up floor and
        # table patterns on this view and the fit was refused); the whole-image
        # search stays as the fallback and for a first calibration.
        regional = [] if saved is None else board_region_grids(
            geometry, gray, matrix, dist, np.array(saved['base_from_camera']), margin_px=80)
        try:
            if not regional:
                raise RuntimeError('no grid near the saved board')
            base_from_camera, detected, rms, used = locate_camera(
                geometry, gray, matrix, dist, position, args.flip, previous, grids=regional)
        except RuntimeError as regional_error:
            say('NOTE', f'board-region search: {regional_error}; searching the whole image')
            base_from_camera, detected, rms, used = locate_camera(
                geometry, gray, matrix, dist, position, args.flip, previous)
    except RuntimeError as error:
        say('BLOCKED', str(error))
        return 1
    out = draw_overlay(bgr, geometry, base_from_camera, matrix, dist, detected)
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    cv2.imwrite(args.output, out)
    side_px = np.linalg.norm(detected[-1, 0] - detected[0, 0]) / (detected.shape[0] - 1)
    with open(args.geometry) as stream:
        data = json.load(stream)
    data['camera'] = {'base_from_camera': np.round(base_from_camera, 5).tolist(), 'mode': mode,
                      'rms_px': round(rms, 2), 'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
    with open(args.geometry, 'w') as stream:
        json.dump(data, stream, indent=2)
    say('OVERLAY', {'image': os.path.relpath(args.output), 'mode': mode,
                    'grid_corners_used': f'{used} of 49',
                    'grid_vs_touches_rms_px': round(rms, 2),
                    'about_mm': round(rms / side_px * geometry.square_m * 1000, 1),
                    'camera_xyz': [round(v, 3) for v in base_from_camera[:3, 3]],
                    'check': 'a1 must be labelled on the real a1; if not, run again with --flip'})
    return 0


def best_square(corners):
    """The true square closest to four touched corners: (corners 4x3, each touch's miss in m).

    A board is made square; four touches each a millimetre or two off are not.
    The square keeps the touched tilt (heights from the touches' plane).
    """
    c = np.asarray(corners, dtype=float)
    p = c[:, 0] + 1j * c[:, 1]
    q = np.array([0, 1, 1 + 1j, 1j])                 # a1, h1, h8, a8 (counter-clockwise)
    a = np.sum((p - p.mean()) * np.conj(q - q.mean())) / np.sum(np.abs(q - q.mean()) ** 2)
    fitted = a * (q - q.mean()) + p.mean()
    plane = BoardGeometry(c)
    out = np.array([[z.real, z.imag, plane.z_at(z.real, z.imag)] for z in fitted])
    return out, np.abs(p - fitted)


PAD_BELOW_GRIP_M = 0.005     # fingertip ends this far below the grip point (RG2 pads)


def tip_above_base(size, finger_reach=0.028):
    """Fingertip height above the piece's base when holding it."""
    if 'grip_m' in size:
        return max(0.0, float(size['grip_m']) - PAD_BELOW_GRIP_M)
    return max(0.0, float(size['height_m']) - finger_reach)


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

    def __init__(self, lab, geometry, pieces, board=None, vision=None):
        self.lab, self.geometry, self.pieces = lab, geometry, pieces
        self.board = board if board is not None else chess.Board()
        self.intent = None
        self.expected_width_mm = None
        self.vision = vision        # Vision: piece centre and height from the camera
        # Speed (2026-10-14): a capture is two steps; the arm parks at home
        # only after the last one (park_after), and OBSERVE never moves when
        # the arm already stands where the next step starts.
        self.park_after = True
        self._retreated = False
        self.parked_off_home = False

    # The lookup.

    def parse(self, utterance):
        try:
            frm, to = parse_step(utterance)
        except ValueError as error:
            raise Refused('not_a_step', str(error))
        self.intent = Intent('pick_and_place', frm, to, f'{frm} -> {to}')
        self._retreated = False
        return self.intent

    def detect(self, query):
        role = 'pick' if query == self.intent.target_query else 'place'
        if role == 'pick' and self.board.piece_at(chess.parse_square(query)) is None:
            raise StageError('empty_square', f'{query} is empty on the board as I know it')
        return Detection(f'{role}:{query}', 1.0)

    def locate(self, detection):
        role, name = detection.capture_id.split(':', 1)
        announce_target(name)
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
        top = self.geometry.z_at(x, y)
        if role == 'place':
            # 0, not the board thickness: the release height already carries
            # it, through the pick's tip height.
            return Target((x, y, top), yaw, {'height_m': 0.0, 'axis_ratio': 100.0})
        piece = self.board.piece_at(chess.parse_square(name))
        if self.vision is not None:
            return self.seen_target(name, piece, table_z, yaw)
        size = self.pieces['pieces'][piece.symbol().lower()]
        measured = 'grip_m' in size or bool(self.pieces.get('measured'))
        self.expected_width_mm = float(size['width_mm']) if measured else None
        thickness = max(0.0, top - table_z(x, y))
        detail = {'height_m': round(thickness + float(size['height_m']), 4),
                  'axis_ratio': 100.0, 'piece': chess.piece_name(piece.piece_type)}
        if 'grip_m' in size:
            detail['tip_above_table_m'] = round(thickness + tip_above_base(size), 4)
        return Target((x, y, top + float(size['height_m'])), yaw, detail)

    def seen_target(self, name, piece, table_z, yaw):
        """Pick target from the camera: where the piece really stands, and how tall it is."""
        squares, _, _ = self.vision.look(executor_tips(self.lab.executor))
        entry = squares[name]
        if not entry['occupied']:
            raise StageError('not_found', f'the camera sees no piece on {name}')
        sx, sy = self.geometry.square_xy(name)
        x, y = entry['centre'][:2]
        off = math.hypot(x - sx, y - sy)
        if off > 0.4 * self.geometry.square_m:
            raise StageError('off_square', f'the piece on {name} stands {off * 1000:.0f} mm from '
                                           'the centre: straighten it, then go again')
        top = float(entry['top_m'])
        thickness = max(0.0, self.geometry.z_at(x, y) - table_z(x, y))
        # The game record knows which piece stands here. With the set measured
        # (lab_chess.yaml, spec S5) the grip height and the expected jaw width
        # come from the table, and the camera supplies the centre and the
        # presence; its height is only a sanity check on the record. An
        # unmeasured set grips at 3/4 of the seen height, no width check.
        size = self.pieces['pieces'].get(piece.symbol().lower(), {}) if piece else {}
        if 'grip_m' in size and 'width_mm' in size:
            grip = max(0.008, float(size['grip_m']) - PAD_BELOW_GRIP_M)
            self.expected_width_mm = float(size['width_mm'])
            source = chess.piece_name(piece.piece_type)
            if abs(top - float(size['height_m'])) > 0.015:
                say('WARNING', f'{name}: the camera reads {top * 1000:.0f} mm tall, a '
                               f'{source} is {float(size["height_m"]) * 1000:.0f}: is the '
                               'right piece standing there?')
        else:
            grip = max(0.008, self.vision.cv.GRIP_FRACTION * top - PAD_BELOW_GRIP_M)
            self.expected_width_mm = None      # unknown width: the empty-close check stays
            source = 'camera'
        say('SEEN', {'square': name, 'top_mm': round(top * 1000, 1),
                     'off_centre_mm': [round((x - sx) * 1000, 1), round((y - sy) * 1000, 1)],
                     'fingertips_above_board_mm': round(grip * 1000, 1), 'grip_from': source})
        return Target((x, y, self.geometry.z_at(x, y) + top), yaw,
                      {'height_m': round(thickness + top, 4), 'axis_ratio': 100.0,
                       'tip_above_table_m': round(thickness + grip, 4),
                       'piece': chess.piece_name(piece.piece_type) if piece else 'piece'})

    def grasp(self):
        """A width far from the piece's known width is a miss, whatever the jaws felt."""
        result = self.lab.grasp()
        expected = self.expected_width_mm
        if expected is None or not result.holding or getattr(
                getattr(self.lab, 'executor', None), 'plan_only', False):
            return result               # plan-only: the stand-in gripper's width means nothing
        tolerance = float(self.pieces.get('width_tolerance_mm', 6.0))
        if abs(result.width_mm - expected) > tolerance:
            self.lab.held_width = None
            return GraspResult(False, result.width_mm,
                               dict(result.detail, expected_mm=expected, reason='width_off'))
        return result

    # The proven stages, untouched apart from when OBSERVE and HOME move at all.

    def at_home(self, tolerance_rad=0.05):
        """Is the arm standing at chess_home (within tolerance on every joint)?"""
        executor = self.lab.executor
        try:
            home = executor.poses[HOME]
            now = executor.joints_of(executor.fresh(settle_s=0.3))
        except (AttributeError, KeyError, TypeError):
            return False
        return all(abs(now[j] - home[j]) <= tolerance_rad for j in home)

    def observe(self):
        """OBSERVE moves nothing when the arm already stands where the step starts.

        The camera reads watch's frames, so the arm has no pose to take for a
        picture; at home, or at the previous step's back-up hover (a capture),
        APPROACH plans from where the arm is, with collision checking.
        """
        if self.parked_off_home or self.at_home():
            say('OBSERVE', 'skipped: the arm is where this step starts')
            return
        self.lab.observe()

    def plan(self, intent, target, place):
        return self.lab.plan(intent, target, place)

    def approach(self):
        self.lab.approach()

    def lift(self):
        self.lab.lift()

    def retreat(self):
        """Carry, lower, let go, back up; at the box the jaws open wide.

        2026-10-14: a knight held at 30.7 mm was "released" by opening to the
        32 mm the jaws use between neighbours on the board: one millimetre of
        play, and the knight stayed wedged. Over the box nothing is in the
        way, so open to 70 mm; on a square, as wide as the held width plus a
        finger's worth, still inside the diagonal gap to the neighbours.
        """
        config, usual = self.lab.config, self.lab.config['open_mm']
        held = getattr(self.lab, 'held_width', None)
        if self.intent is not None and self.intent.place_query == BOX:
            config['open_mm'] = BOX_RELEASE_MM
        elif held:
            config['open_mm'] = max(usual, min(float(held) + 10.0, SQUARE_RELEASE_MAX_MM))
        try:
            self.lab.retreat()
        finally:
            config['open_mm'] = usual
        self._retreated = True

    def release(self):
        self.lab.release()

    def home(self):
        """Park at home after the move's last step; between a capture's two
        steps stay at the back-up hover. Any failure path still goes home:
        _retreated is only set by a successful RETREAT."""
        if self._retreated and not self.park_after:
            say('HOME', 'skipped: the next step starts from here')
            self.parked_off_home = True
            return
        self.lab.home()
        self.parked_off_home = False

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

def use_board_surface(executor, geometry, thickness_m):
    """Measure everything from the touched board, not lab_table.json.

    lab_table.json (the table plane, the camera, the tool length) belongs to the
    project-1 robot. On the chess arm the surface under the board is the touched
    top minus the board's ruler thickness; the collision table goes there, and the
    project-1 stand's obstacles and camera keep-out are dropped.
    """
    thickness_m = float(thickness_m)
    executor.table_z = lambda x, y: geometry.z_at(x, y) - thickness_m
    # The collision table is level: put it under the board's lowest corner.
    surface = float(geometry.corners[:, 2].min()) - thickness_m
    executor.table_surface_z, executor.anchor, executor.calibration = surface, None, None
    executor.lab_obstacles = lambda _surface: []
    return surface


HOME = 'chess_home'


def open_session(args, board, need_home=True):
    """Executor, lab adapters, chess adapters and orchestrator (needs the ROS graph)."""
    import rclpy
    import lab_pick
    from ur7e_orchestrator.workflow import Orchestrator
    geometry = BoardGeometry.load(args.geometry)
    pieces = load_pieces(args.pieces)
    taught = {}
    for path in (lab_pick.SEED_POSES_FILE, args.poses_file):
        if path and os.path.exists(path):
            with open(path) as stream:
                taught.update(json.load(stream))
    if need_home and HOME not in taught:
        # The console's home/observe were taught on another arm and another
        # table (2026-10-10: a 5.5 rad base swing away); chess has its own.
        raise ValueError(f'no {HOME} taught on this arm: Freedrive the gripper straight '
                         'down, beside the board on the robot side (about 10 cm out, 15 cm '
                         'up), then run python3 scripts/lab_chess.py teach-home')
    rclpy.init()
    executor, lab, _ = lab_pick.build(args)
    # Ten segments a step: a 1 s settle before each plan was a quarter of
    # the step. The stationarity check in fresh() still guards every move.
    executor.settle_s = 0.3
    if pieces.get('board_thickness_m') is not None:
        use_board_surface(executor, geometry, pieces['board_thickness_m'])
    if HOME in executor.poses:
        lab.config['home_pose'] = lab.config['observe_pose'] = HOME
    lab.config['open_mm'] = float(pieces.get('open_mm', lab.config['open_mm']))
    sizes = pieces['pieces'].values()
    tallest = max(float(p['height_m']) for p in sizes)
    # A carried piece's base hangs tip_above_base below the fingertips, which
    # hover args.hover above its own top.
    clearance = min(args.hover + float(p['height_m']) - tip_above_base(p, lab.config['finger_reach'])
                    for p in sizes) - tallest
    if clearance < 0.03:
        say('WARNING', f'a carried piece clears the tallest piece by {clearance * 1000:.0f} mm; '
                       'raise --hover')
    # The RG2 reads 9.3 mm closed on nothing; lab_pick's 6 mm would call that a hold.
    lab.config['miss_width_mm'] = EMPTY_CLOSE_MM + 3.0
    vision = Vision(geometry, args.device) if getattr(args, 'vision', False) else None
    if vision is not None:
        check_board(geometry, board, args)
    adapters = ChessAdapters(lab, geometry, pieces, board, vision)
    def on_event(event):
        lab_pick.on_event(event)
        if event.get('event') == 'stage_end':
            fields = {'stage': {'name': event['stage'], 'outcome': event['outcome'],
                                'reason': event.get('reason')}}
            width = (event.get('detail') or {}).get('width_mm')
            if event['stage'] == 'grasp' and width is not None:
                fields['grip_mm'] = width
            publish_state(**fields)

    orchestrator = Orchestrator(adapters, log_dir=args.log_dir, on_event=on_event)
    return executor, lab, adapters, orchestrator


def close_session(session):
    import rclpy
    executor, _, _, orchestrator = session
    orchestrator.close()
    executor.destroy_node()
    executor.motion.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def park_arm(session):
    """Send the arm home if a skipped HOME left it at a hover (declined or failed step)."""
    _, lab, adapters, _ = session
    if not adapters.parked_off_home:
        return
    say('HOME', 'parking the arm')
    try:
        adapters.park_after, adapters._retreated = True, False
        adapters.home()
    except (StageError, Exception) as error:   # noqa: BLE001 - report, the operator decides
        say('WARNING', f'could not park the arm: {error}')


def run_steps(session, steps, auto_yes, confirm_steps=True):
    """Run each (from, to) through the orchestrator; return how many finished.

    The arm parks at home after the last step only: between the two steps of
    a capture it waits at the box's hover, where the next pick starts.
    ``confirm_steps=False``: the move was approved as a whole ("everyone
    clear"); no prompt per step.
    """
    _, _, adapters, orchestrator = session
    for done, (frm, to) in enumerate(steps):
        answer = ask_operator('step', f'{frm} -> {to}: go?',
                              [option(GO, f'Go: {frm} → {to}', 'go'), option(STOP, 'Stop', 'stop')],
                              auto=GO if auto_yes or not confirm_steps else None)
        if answer != GO:
            say('RESULT', {'outcome': 'declined', 'step': f'{frm} {to}'})
            return done
        adapters.park_after = done == len(steps) - 1
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
    corners = [touches[c]['tip'] for c in CORNERS]
    if args.square:
        corners, misses = best_square(corners)
        say('SQUARE', {'touch_off_square_mm': dict(zip(('a1', 'h1', 'h8', 'a8'),
                                                       np.round(misses * 1000, 1).tolist()))})
        if misses.max() > 0.003:
            say('BLOCKED', 'a touch is more than 3 mm off the best square: re-touch it')
            return 1
    geometry = BoardGeometry(corners, box,
                             notes={'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
                                    'touches': touches, 'square': bool(args.square)})
    complaints = geometry.complaints()
    for text in complaints:
        say('WARNING', text)
    if complaints and not args.force:
        say('BLOCKED', 're-touch the corner that disagrees (or --force to keep these)')
        return 1
    geometry.save(args.geometry)
    say('FIT', {'square_mm': round(geometry.square_m * 1000, 1),
                'top_z': round(geometry.top_z, 4),
                'tilt_deg': round(geometry.tilt_deg(), 2),
                'file_yaw_deg': round(math.degrees(geometry.file_yaw()), 1),
                'box': None if box is None else [round(v, 4) for v in box],
                'file': os.path.relpath(args.geometry)})
    return 0


def cmd_shift(args):
    """Slide the whole calibration (corners and box) by millimetres in base_link.

    The honest fix for a board that is off everywhere by the same amount is to
    re-touch the corners. This is the ten-second fallback when that cannot be
    done: measure the miss with ``goto --touch`` on two squares (the fingertip
    is 10 mm over where the arm thinks the centre is) and move the board the
    other way. The camera pose stays: it is relative to the robot, and ``watch``
    reloads the geometry on its own.
    """
    geometry = BoardGeometry.load(args.geometry)
    delta = np.array([args.dx, args.dy, args.dz], float) / 1000
    size = float(np.linalg.norm(delta))
    if size > 0.03:
        say('BLOCKED', f'{size * 1000:.0f} mm is more than 30 mm: re-touch the corners instead')
        return 1
    before = geometry.square_xy('e4')
    moved = BoardGeometry(geometry.corners + delta,
                          None if geometry.box is None else geometry.box + delta, geometry.notes)
    moved.notes[f'shift_{time.strftime("%Y%m%d-%H%M%S")}'] = (
        f'slid by {[args.dx, args.dy, args.dz]} mm: {args.why or "operator measured a miss"}; '
        're-touch the corners to replace this')
    moved.save(args.geometry)
    after = moved.square_xy('e4')
    say('SHIFT', {'mm': [args.dx, args.dy, args.dz], 'e4_before': [round(v, 4) for v in before],
                  'e4_after': [round(v, 4) for v in after], 'file': os.path.relpath(args.geometry),
                  'next': 'goto e4 --touch to check; watch reloads the geometry by itself'})
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


def cmd_teach_home(args):
    """Save the arm's present joints as chess_home, if the gripper is placed for it."""
    from lab_pick import tool_yaw
    session = open_session(args, chess.Board(), need_home=False)
    executor, _, adapters, _ = session
    try:
        geometry = adapters.geometry
        pose = executor.fk(executor.fresh())
        x, y = pose.position.x, pose.position.y
        _, tilt = tool_yaw(pose)
        above = pose.position.z - executor.tooling['tcp_to_tips_m'] - geometry.z_at(x, y)
        # Home is where the camera looks at the board from (2026-10-11): parked over
        # the middle, the gripper hid Black's back rank and made d8/e8 read empty.
        # So: beside the board, not over it, and not far from it.
        import chess_vision
        u, v = chess_vision.board_uv(geometry, [[x, y]])[0]
        outside = max(-u, u - 1, -v, v - 1, 0) * 8 * geometry.square_m   # metres past the edge
        problems = [text for bad, text in (
            (tilt > 5, f'the gripper is tilted {tilt:.0f} deg: point it straight down'),
            (above < 0.08, f'the fingertips are {above * 1000:.0f} mm over the board: '
                           'at least 80 mm, 150 is good'),
            (outside < 0.04, 'the gripper is over the board, where it hides pieces from the '
                             'camera: park it beside the board on the robot side, 4-20 cm out'),
            (outside > 0.20, f'the gripper is {outside * 100:.0f} cm from the board: '
                             'come within 20 cm'),
        ) if bad]
        if problems:
            for text in problems:
                say('BLOCKED', text)
            return 1
        joints = executor.teach(HOME)
        say('TAUGHT', {'pose': HOME, 'file': os.path.relpath(executor.poses_file),
                       'fingertips_over_board_mm': round(above * 1000),
                       'joints_rad': {j: round(v, 3) for j, v in joints.items()}})
        return 0
    finally:
        close_session(session)


def cmd_gesture(args):
    """One gesture (or all) from home: plan-only by default, the arm with --execute."""
    names = list(chess_gestures.GESTURES) if args.name == 'all' else [args.name]
    session = open_session(args, chess.Board())
    executor, lab, adapters, _ = session
    try:
        for name in names:
            if not args.execute:
                points, report = chess_gestures.plan(executor, adapters.geometry, name)
                say('GESTURE', report if points else f'{name}: skipped: {report}')
                continue
            if not args.yes:
                ask_operator('clear', f'Gesture {name} from home. Everyone clear of the arm?',
                             [option(GO, 'Go', 'go'), option(STOP, 'Skip', 'stop')])
            event = next(e for e, (g, _) in chess_gestures.EVENT_GESTURE.items() if g == name)
            chess_gestures.perform(event, executor, lab, adapters.geometry, executor.joint_rate,
                                   say, claps=not args.no_claps)
        return 0
    finally:
        close_session(session)


def cmd_goto(args):
    from lab_pick import tool_yaw
    session = open_session(args, chess.Board())
    executor, lab, adapters, _ = session
    try:
        geometry = adapters.geometry
        x, y = geometry.square_xy(args.square)
        announce_target(args.square)
        thickness = max(0.0, geometry.z_at(x, y) - executor.table_z(x, y))
        yaw, _ = tool_yaw(executor.fk(executor.fresh()))
        say('TARGET', {'square': args.square, 'xy': [round(x, 4), round(y, 4)],
                       'board_top_z': round(geometry.z_at(x, y), 4),
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
            if os.path.exists(TARGET_FILE):
                os.remove(TARGET_FILE)
        return 0
    finally:
        close_session(session)


def cmd_move(args):
    # One move at a time carries on from the last recorded position (the one the
    # dashboard shows), not the opening: 2026-10-11 a3a4 was refused as illegal
    # because every command started over. --fen start forces the opening.
    recorded = read_state().get('fen')
    if args.fen == 'start' or (not args.fen and not recorded):
        board = chess.Board()
    else:
        board = chess.Board(args.fen or recorded)
        if not args.fen:
            say('POSITION', {'from': 'the last recorded move', 'fen': board.fen()})
    try:
        move = parse_move(board, args.move)
    except ValueError as error:
        # A single test move is about the piece, not whose turn the record says it is.
        board.turn = not board.turn
        try:
            move = parse_move(board, args.move)
        except ValueError:
            say('BLOCKED', str(error))
            return 2
        say('TURN', f"played as {'White' if board.turn else 'Black'}: the recorded game "
                    "had the other side to move")
    steps, note = steps_for(board, move)
    say('MOVE', {'san': board.san(move), 'steps': [f'{a} {b}' for a, b in steps], 'note': note})
    if args.steps_only:
        return 0
    publish_state(fen=board.fen(), planned=planned_move(board, move), moves=move_list(board))
    session = open_session(args, board)
    try:
        try:
            done = run_steps(session, steps, args.yes)
        except (NewGame, QuitGame):            # the page's standing buttons: a single move just stops
            done = 0
        park_arm(session)
        if done == len(steps):
            if note:
                say('NOTE', note)
            board.push(move)
        publish_state(fen=board.fen(), planned=None, moves=move_list(board))
        return 0 if done == len(steps) else 1
    finally:
        close_session(session)


GAME_OPTIONS = [option(NEW_GAME, 'New game', 'warn'), option(QUIT, 'Quit', 'stop')]
CONFIRM_SECONDS = 8.0        # with --auto-confirm: "I saw e4" stands unless the operator objects within this
AUTO_CONFIRM = False         # set by play --auto-confirm; off by default since the Bxa6 game (2026-10-14)
WATCH_EVERY_S = 2.5          # how often the board is read while the student thinks
STABLE_READINGS = 3          # agreeing readings (about 7 s) before the camera proposes a move


def confident_margin(how):
    """Absolute reading: a plain move shows two changed squares, so a margin
    of 1 is a coin toss. Diff reading: a capture only vacates its square, so
    it can beat a quiet move from the same square by exactly 1 (2026-10-14,
    Nxc7+ read right and refused); only a tie (0) is unsure there."""
    return 1 if how == 'diff' else 2


def read_student_move(vision, executor, board):
    """One reading of the board: (move, margin, how, squares).

    With a baseline scan (after the arm's move, or the game's start) the move
    is read from what CHANGED; without one, from the absolute picture.
    """
    squares, _, _ = vision.look(executor_tips(executor))
    colours = vision.cv.piece_colours(squares, board)
    if vision.baseline is not None:
        move, margin, _ = vision.cv.infer_move_diff(board, vision.baseline, squares, colours)
        return move, margin, 'diff', squares
    move, margin, _ = vision.cv.infer_move(board, squares, colours)
    return move, margin, 'absolute', squares


class BoardWatcher:
    """Reads the board every few seconds while the student prompt waits.

    Answers the prompt by itself once STABLE_READINGS readings in a row agree
    on a confident move: the operator then confirms. A board being touched
    reads differently from one scan to the next, and a piece held in the
    hand reads like a capture, so a single reading never counts and a
    capture also needs the colour flip (infer_move_diff).
    """
    SEEN = '\x00seen'

    def __init__(self, vision, executor, board):
        self.vision, self.executor, self.board = vision, executor, board
        self.last_at, self.previous, self.agreed, self.result = 0.0, None, 0, None

    def poll(self):
        if time.time() - self.last_at < WATCH_EVERY_S:
            return None
        self.last_at = time.time()
        try:
            move, margin, how, squares = read_student_move(self.vision, self.executor, self.board)
        except Exception as error:    # noqa: BLE001 - a bad frame is not an answer
            say('WARNING', f'watching the board: {error}')
            return None
        if move is None or margin < confident_margin(how):
            self.previous, self.agreed = None, 0
            return None
        if self.previous != move:
            self.previous, self.agreed = move, 1
            return None
        self.agreed += 1
        if self.agreed < STABLE_READINGS:
            return None
        self.result = (move, margin, how, squares)
        return self.SEEN


def student_move(vision, executor, board, side):
    """The student's move: read from the board (--vision) or typed; None = ask again.

    Raises NewGame / QuitGame from the operator's buttons.
    """
    if vision is not None:
        options = [option('', 'Read the board', 'go')] + GAME_OPTIONS
        text = f'Student ({side}): make your move. I watch the board; or read it now, or type the move'
    else:
        options = list(GAME_OPTIONS)
        text = f'Student ({side}): type the move, e.g. e5 or e7e5'
    watcher = BoardWatcher(vision, executor, board) if vision is not None else None
    answer = ask_operator('student_move', text, options, allow_text=True,
                          poll=watcher.poll if watcher else None)
    if answer and answer != BoardWatcher.SEEN:
        try:
            return parse_move(board, answer)
        except ValueError as error:
            say('REFUSED', str(error))
            return None
    if vision is None:
        return None
    if answer == BoardWatcher.SEEN:
        move, margin, how, squares = watcher.result
    else:
        move, margin, how, squares = read_student_move(vision, executor, board)
    seen = {'move': board.san(move) if move else None, 'margin': margin, 'how': how,
            'runners_up': []}
    say('SEEN', seen)
    chess_gestures.log_event(EVENTS_FILE, 'seen', **seen)
    if move is None or margin < confident_margin(how):
        print(board_text(squares))
        say('UNSURE', 'two moves explain the board equally well: type the move')
        chess_gestures.log_event(EVENTS_FILE, 'unsure', **seen)
        return None
    # The operator confirms. With --auto-confirm the reading stands after a
    # few seconds unless objected to; off by default since a bishop held in
    # the hand was read as Bxa6 and stood (2026-10-14).
    check = ask_operator('confirm_move', f'I saw {board.san(move)}. Right?',
                         [option(GO, f'Yes, {board.san(move)}', 'go'),
                          option(STOP, 'No: read again')] + GAME_OPTIONS, allow_text=True,
                         timeout_s=CONFIRM_SECONDS if AUTO_CONFIRM else None,
                         default=GO if AUTO_CONFIRM else None)
    if check == GO:
        return move
    if check == STOP:
        return None
    try:
        return parse_move(board, check)
    except ValueError as error:
        say('REFUSED', str(error))
        return None


def check_position(vision, executor, board):
    """After the arm's move: does the board look like the game says? Warn if not.

    The scan (after any fix by hand) becomes the baseline the student's next
    move is read against.
    """
    squares, _, _ = vision.look(executor_tips(executor))
    wrong = [name for name, e in squares.items() if e['seen']
             and e['occupied'] != (board.piece_at(chess.parse_square(name)) is not None)]
    if wrong:
        print(board_text(squares))
        say('CHECK', {'squares_not_as_expected': wrong,
                      'hint': 'fix the board by hand to match the game, then continue'})
        ask_operator('board_fixed', f'The camera disagrees on {", ".join(wrong)}: '
                                    'fix the board to match the game record',
                     [option(GO, 'Board fixed', 'go')])
        squares, _, _ = vision.look(executor_tips(executor))
    else:
        say('CHECK', 'board matches the game')
    vision.baseline = squares


def capture_agrees(vision, executor, board, move):
    """Does the camera see the student's piece where the arm is about to capture?

    Returns True to go ahead (also when the camera cannot tell), False when
    the operator chose to take the student's last move back.
    """
    target = chess.square_name(move.to_square)
    if board.is_en_passant(move):
        target = chess.square_name(chess.square(chess.square_file(move.to_square),
                                                chess.square_rank(move.from_square)))
    try:
        squares, _, _ = vision.look(executor_tips(executor))
    except Exception as error:      # noqa: BLE001 - no picture, no objection
        say('WARNING', f'capture check: {error}')
        return True
    seen_white = vision.cv.piece_colours(squares, board).get(target)
    entry = squares.get(target, {})
    if seen_white is None or not entry.get('seen'):
        return True
    student_is_white = not board.turn            # the arm is the side to move
    if seen_white == student_is_white:           # the piece there is the student's: fine
        return True
    colour = 'white' if seen_white else 'black'
    say('CHECK', {'square': target, 'camera_sees': colour, 'record_expects': 'the student\'s piece'})
    answer = ask_operator('record_mismatch',
                          f'I am about to take on {target}, but the camera sees a {colour} piece '
                          'there, my own colour. Was the last student move entered wrong?',
                          [option('u', 'Yes: take it back and re-enter it', 'go'),
                           option(GO, 'No, the camera is wrong: go on', 'stop')] + GAME_OPTIONS)
    return answer != 'u'


def baseline_scan(vision, executor):
    """The scan the first student move is read against (a new game, a resumed one)."""
    if vision is None:
        return
    try:
        vision.baseline, _, _ = vision.look(executor_tips(executor))
    except Exception as error:      # noqa: BLE001 - no baseline: absolute reading until the arm moves
        say('WARNING', f'no baseline scan: {error}')
        vision.baseline = None


def arm_move(board, move, session, args):
    """Run the arm's move (every step approved by the operator). Returns when the board is right."""
    steps, note = steps_for(board, move)
    say('ARM', {'san': board.san(move), 'steps': [f'{a} {b}' for a, b in steps]})
    if session is not None:
        ask_operator('clear', f'Robot plays {board.san(move)} ({" then ".join(f"{a} → {b}" for a, b in steps)}). '
                              'Everyone clear of the board?',
                     [option(GO, 'Clear: let the arm move', 'go')] + GAME_OPTIONS,
                     auto=GO if args.yes else None)
        remaining = list(steps)
        while remaining:
            # "Everyone clear" approved the whole move; --careful asks per step too.
            done = run_steps(session, remaining, args.yes, confirm_steps=args.careful)
            remaining = remaining[done:]
            if not remaining:
                break
            park_arm(session)                # a stopped move never leaves the arm over the box
            answer = ask_operator('failed', f'The arm did not finish {remaining[0][0]} → {remaining[0][1]}',
                                  [option('r', 'Retry this step', 'go'),
                                   option('h', 'I finished the move by hand')] + GAME_OPTIONS)
            if answer == 'h':
                break
    if note:
        say('NOTE', note)
        if session is not None:
            ask_operator('swap', note, [option(GO, 'Swapped', 'go')], auto=GO if args.yes else None)


def react(session, args, event, **detail):
    """A game event: logged for the voice (events.jsonl), acted out by the arm (gestures)."""
    chess_gestures.log_event(EVENTS_FILE, event, **detail)
    say('EVENT', dict(event=event, **detail))
    if session is None or getattr(args, 'no_gestures', False):
        return
    executor, lab, adapters, _ = session
    chess_gestures.perform(event, executor, lab, adapters.geometry, executor.joint_rate, say)


def play_game(board, arm, engine, session, args):
    """One game to the end. Raises NewGame / QuitGame when the operator says so."""
    while not board.is_game_over(claim_draw=True):
        print(f'\n{board}\n')
        side = 'White' if board.turn == chess.WHITE else 'Black'
        if board.turn != arm:
            vision = session[2].vision if session is not None else None
            executor = session[0] if session is not None else None
            move = student_move(vision, executor, board, side)
            if move is None:
                continue
            san, capture = board.san(move), board.is_capture(move)
            say('STUDENT', san)
            board.push(move)
            publish_state(fen=board.fen(), planned=None, moves=move_list(board))
            if capture:
                react(session, args, 'lost', san=san, fen=board.fen())
            continue
        publish_state(thinking=True)
        move = engine.choose(board)
        publish_state(planned=planned_move(board, move), thinking=False)
        if session is not None and session[2].vision is not None and board.is_capture(move):
            # Before taking a piece, the camera must agree it is the student's.
            # 2026-10-14: a misread student move put a white bishop on a6 in
            # the record; the arm then removed its own pawn from a6.
            if not capture_agrees(session[2].vision, session[0], board, move):
                board.pop()
                publish_state(fen=board.fen(), planned=None, moves=move_list(board))
                say('UNDO', 'the student\'s last move was taken back; enter it again')
                continue
        arm_move(board, move, session, args)
        san, capture = board.san(move), board.is_capture(move)
        board.push(move)
        publish_state(fen=board.fen(), planned=None, moves=move_list(board))
        if session is not None and session[2].vision is not None:
            check_position(session[2].vision, session[0], board)
        if board.is_game_over(claim_draw=True):
            break
        if capture:
            react(session, args, 'captured', san=san, fen=board.fen())
        elif board.is_check():
            react(session, args, 'check', san=san, fen=board.fen())
    result = board.result(claim_draw=True)
    say('GAME', {'result': result, 'moves': len(board.move_stack)})
    publish_state(result=result)
    arm_result = {'1-0': chess.WHITE, '0-1': chess.BLACK}.get(result)
    react(session, args, 'draw' if arm_result is None else 'won' if arm_result == arm else 'lost_game',
          result=result, moves=len(board.move_stack))
    return result


def cmd_play(args):
    """The match: games one after another until the operator quits.

    Every game starts from the opening unless --resume (the recorded position,
    e.g. after play was restarted). New game / Quit are offered on every prompt,
    so a new group at the board is one click, with the arm parked.
    """
    arm_white = args.arm_white
    if args.resume and not args.fen:
        recorded = read_state()
        if not recorded.get('fen'):
            say('BLOCKED', 'nothing to resume: no recorded game')
            return 2
        # Replay the recorded moves so the move list survives; a record whose
        # moves do not reach its position falls back to the bare position.
        board = chess.Board()
        try:
            for san in recorded.get('moves') or []:
                board.push_san(san)
            if board.fen() != recorded['fen']:
                raise ValueError('moves and position disagree')
        except ValueError:
            board = chess.Board(recorded['fen'])
        arm_white = recorded.get('arm') == 'white'
        say('POSITION', {'from': 'the recorded game', 'fen': board.fen(),
                         'arm': 'white' if arm_white else 'black'})
    else:
        board = chess.Board(args.fen) if args.fen else chess.Board()
    engine = Engine(args.engine, args.skill, args.think, args.seed)
    say('ENGINE', engine.name)
    global AUTO_CONFIRM
    AUTO_CONFIRM = bool(getattr(args, 'auto_confirm', False))
    if args.resume and not args.fen:
        publish_state(planned=None, prompt=None, result=None, thinking=False)
    else:
        new_game_state(board, arm_white)
    session = None if args.steps_only else open_session(args, board)
    try:
        while True:
            arm = chess.WHITE if arm_white else chess.BLACK
            if session is not None:
                session[2].board = board          # the adapter answers DETECT from this board
            try:
                if not board.move_stack:
                    react(session, args, 'greeting', arm='white' if arm_white else 'black')
                if session is not None:
                    baseline_scan(session[2].vision, session[0])
                result = play_game(board, arm, engine, session, args)
                ask_operator('game_over', f'Game over: {result} after {len(board.move_stack)} moves',
                             GAME_OPTIONS)
                say('GAME', 'over; waiting for the operator')
                continue
            except NewGame:
                say('GAME', 'new game: set the pieces up; the record starts over')
                if session is not None:
                    park_arm(session)
                board = chess.Board()
                new_game_state(board, arm_white)
            except QuitGame:
                say('GAME', 'stopped by the operator')
                return 0
    finally:
        publish_state(prompt=None, thinking=False)
        engine.close()
        if session is not None:
            park_arm(session)
            close_session(session)


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--geometry', default=GEOMETRY_FILE, help='board geometry (lab_chess.json)')
    cli.add_argument('--pieces', default=PIECES_FILE, help='piece table (lab_chess.yaml)')
    sub = cli.add_subparsers(dest='command', required=True)

    p = sub.add_parser('reset', help='reset the game record to the starting position; no robot motion')
    p.add_argument('--arm-white', action='store_true', help='robot plays White (default: Black)')
    p.set_defaults(run=cmd_reset)

    p = sub.add_parser('touch', help='record the fingertip on a board corner or the box rim')
    p.add_argument('--corner', required=True, choices=CORNERS + (BOX,))
    p.add_argument('--robot-ip', default=os.environ.get('ROBOT_IP', '192.168.56.101'))
    p.add_argument('--tool-index', type=int,
                   help='OnRobot device index of the RG2 (default: find it by $RG2_SERIAL)')
    p.set_defaults(run=cmd_touch)
    p = sub.add_parser('overlay', help='draw the calibrated squares over a ZED image (empty board)')
    p.add_argument('--device', type=int, help='ZED video device (default: the saved one)')
    p.add_argument('--image', help='use this image instead of grabbing one')
    p.add_argument('--conf', default=ZED_CONF, help='the ZED factory calibration file')
    p.add_argument('--flip', action='store_true', help='a1 landed elsewhere: take the runner-up')
    p.add_argument('--fen', help="pieces on the board: 'start', or a FEN (default: empty)")
    p.add_argument('--output', default=OVERLAY_FILE)
    p.set_defaults(run=cmd_overlay)
    p = sub.add_parser('scan', help='look at the board: pieces, heights, centres (stereo)')
    p.add_argument('--device', type=int, help='ZED video device (default: the saved one)')
    p.add_argument('--conf', default=ZED_CONF, help='the ZED factory calibration file')
    p.add_argument('--no-robot', action='store_true', help='do not ask the driver where the arm is')
    p.set_defaults(run=cmd_scan)
    p = sub.add_parser('track', help='has the board moved? --apply follows it')
    p.add_argument('--device', type=int, help='ZED video device (default: the saved one)')
    p.add_argument('--conf', default=ZED_CONF, help='the ZED factory calibration file')
    p.add_argument('--fen', help='pieces on the board (helps the grid match; default: empty)')
    p.add_argument('--apply', action='store_true', help='move the calibration with the board')
    p.set_defaults(run=cmd_track)
    p = sub.add_parser('watch', help='live camera view: green when the arm is on its target')
    p.add_argument('--device', type=int, help='ZED video device (default: the saved one)')
    p.add_argument('--conf', default=ZED_CONF, help='the ZED factory calibration file')
    p.add_argument('--port', type=int, default=8091)
    p.add_argument('--scan-every', type=float, default=2.0,
                   help='seconds between stereo scans for the piece markers')
    p.add_argument('--no-scan', action='store_true', help='video and target only')
    p.add_argument('--lan', action='store_true',
                   help='serve the dashboard to the whole network (default: this computer)')
    p.set_defaults(run=cmd_watch)
    p = sub.add_parser('fit', help='the four touches -> lab_chess.json')
    p.add_argument('--force', action='store_true', help='keep touches that disagree')
    p.add_argument('--square', action='store_true',
                   help='use the true square closest to the touches (each within 3 mm)')
    p.set_defaults(run=cmd_fit)
    p = sub.add_parser('squares', help='print every square centre (no robot)')
    p.set_defaults(run=cmd_squares)
    p = sub.add_parser('shift', help='slide the calibration by mm (fallback for a constant miss; '
                                     're-touching the corners is the real fix)')
    p.add_argument('--dx', type=float, default=0.0, help='mm along base x')
    p.add_argument('--dy', type=float, default=0.0, help='mm along base y')
    p.add_argument('--dz', type=float, default=0.0, help='mm up')
    p.add_argument('--why', help='what was measured, for the notes')
    p.set_defaults(run=cmd_shift)

    try:
        from lab_pick import add_session_arguments      # needs ROS; the three commands below do too
    except ImportError as error:
        add_session_arguments = None
        reason = f'(unavailable here: {error.name or error})'
    arm = []
    p = sub.add_parser('teach-home', help='save the present arm pose as chess_home')
    p.set_defaults(run=cmd_teach_home)
    arm.append(p)
    p = sub.add_parser('gesture', help='one gesture from home (plan-only unless --execute)')
    p.add_argument('name', choices=list(chess_gestures.GESTURES) + ['all'])
    p.add_argument('--no-claps', action='store_true', help='no jaw claps')
    p.set_defaults(run=cmd_gesture)
    arm.append(p)
    p = sub.add_parser('goto', help='hover over a square; --touch points at it 10 mm up')
    p.add_argument('square')
    p.add_argument('--touch', action='store_true')
    p.add_argument('--stay', action='store_true', help='do not return home afterwards')
    p.set_defaults(run=cmd_goto)
    arm.append(p)
    p = sub.add_parser('move', help='one move through the orchestrator (plan-only by default)')
    p.add_argument('move', help='SAN or UCI, e.g. Nf3 or g1f3')
    p.add_argument('--fen', help="position to move from (default: the last recorded one; "
                                 "'start' for the opening)")
    p.add_argument('--steps-only', action='store_true', help='print the steps, no robot')
    p.set_defaults(run=cmd_move)
    arm.append(p)
    p = sub.add_parser('play', help='the match: prompts on the operator page '
                                    '(http://localhost:8091/#operator) and in this terminal')
    p.add_argument('--fen', help='start from this position (default: the opening)')
    p.add_argument('--resume', action='store_true',
                   help='carry on from the recorded game (log/chess/state.json) after a restart')
    p.add_argument('--arm-white', action='store_true',
                   help='the arm plays White (default: Black)')
    p.add_argument('--careful', action='store_true',
                   help='ask before every step as well as before the move '
                        '(first runs after a calibration change)')
    p.add_argument('--no-gestures', action='store_true',
                   help='no wrist gestures on captures, checks and the result')
    p.add_argument('--auto-confirm', action='store_true',
                   help='"I saw e4" stands after a few seconds unless the operator objects '
                        '(default: the operator clicks Yes)')
    p.add_argument('--engine', default='stockfish', help='UCI engine binary, or none')
    p.add_argument('--skill', type=int, default=1, help='Stockfish Skill Level 0..20')
    p.add_argument('--think', type=float, default=0.5, help='seconds per engine move')
    p.add_argument('--seed', type=int, help='random mover seed')
    p.add_argument('--steps-only', action='store_true', help='play in the terminal, no robot')
    p.set_defaults(run=cmd_play)
    arm.append(p)
    for p in arm:
        p.add_argument('--vision', action='store_true',
                       help='camera finds each piece: its real centre and height')
        p.add_argument('--device', type=int, help='ZED video device (default: the saved one)')
        if add_session_arguments is not None:
            add_session_arguments(p)
            # Chess carries lower than the project-1 picks (0.12): fingertips 8 cm over
            # the carried piece's top. open_session warns when that is too low to clear
            # the tallest piece in lab_chess.yaml. Rates (2026-10-14): 0.4 rad/s
            # travel, 0.12 beside the pieces; the pendant slider scales both.
            # 0.4 shook the arm at segment boundaries (2026-10-14); 0.32 with
            # the ramped starts in retime.py is the operator's "80 %".
            p.set_defaults(hover=0.08, joint_rate=0.32, contact_rate=0.10)
        else:
            p.add_argument('--yes', action='store_true')
            p.description = f'{p.prog}: robot options {reason}'
    args = cli.parse_args()
    if getattr(args, 'command', '') in ('teach-home', 'goto', 'move', 'play', 'gesture') \
            and add_session_arguments is None \
            and not getattr(args, 'steps_only', False):
        say('BLOCKED', f'{args.command} needs the ROS graph {reason}')
        return 2
    try:
        if args.command in ('reset', 'play', 'move', 'goto'):
            with game_state_lock():
                return args.run(args)
        return args.run(args)
    except (FileNotFoundError, ValueError, RuntimeError) as error:
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
