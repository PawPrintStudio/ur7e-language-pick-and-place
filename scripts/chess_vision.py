"""The ZED's view of the chessboard: board shift, square occupancy, piece height and centre.

Built on two things this session already proved: the touched board
(``lab_chess.json``: corners, tilt) and the camera's pose in the robot base
frame (``lab_chess.py overlay``, camera assumed rigidly mounted). With both
known, every stereo point can be put on the board in millimetres.

Why depth and not the image alone
---------------------------------
The camera looks at the board from about 45 degrees. In the image a tall
piece covers the squares behind it, so "which squares look different" mixes
up pieces and their neighbours. In 3-D the same piece is a cluster of points
standing on ITS square: one stereo scan gives which squares are occupied,
how tall each piece is (no per-piece table) and where its axis really is
(people do not centre pieces).

What makes the stereo good enough (measured 2026-10-10, 0.85 m from the board)
--------------------------------------------------------------------------------
* The factory lens tilt had drifted: rows 3-34 px apart. ``fit_row_rotation``
  re-fits it from matched features (0.65 px after), once per camera session.
* Disparity read 39.6 px low, uniformly (spread < 1 px). The board's true
  position is known, so every scan measures that offset on the board itself
  and removes it. Heights are then relative to the board seen in the same
  frame: board noise 1.4 mm (IQR 2.8 mm), a 40 mm pawn read 41.7 mm.
* The robot is removed with its own pose (a cylinder around the tool axis).
* SGBM invents small floating clusters on the repeating squares. A real piece
  is seen from the side: points from near the board up to its top. Clusters
  with nothing low, or too few points for their distance, are dropped.

No ROS in here; ``lab_chess.py`` grabs the frames and the robot pose.
"""
import math

import cv2
import numpy as np

from ur7e_perception.stereo import StereoRig, fit_row_rotation, match_features

FILES = 'abcdefgh'
GRIP_FRACTION = 0.75      # close the pads at 3/4 of the measured height (pawn: 30 of 40 mm)
MIN_TOP_M = 0.015         # anything lower is board noise or a flat object
MAX_PIECE_M = 0.11        # no chess piece is taller: points above are the arm or a person
ROBOT_RADIUS_M = 0.045    # tool axis -> outside of the RG2 and its cable
VISIBLE_SIDE_BIAS = 2 / math.pi   # centroid of a half-cylinder's visible arc, in radii
# What scans use. From 45 deg above the camera sees each piece's head all round,
# not just its front: 2026-10-11, six hand-placed pieces read 0.3-0.9 mm off
# centre uncorrected and 4-8 mm (all away from the camera) with 2/pi.
DEPTH_BIAS = 0.0


def square_name(f, r):
    return f'{FILES[f]}{r + 1}'


class BoardCamera:
    """Stereo points of the board in the robot base frame, heights above the board."""

    def __init__(self, geometry, base_from_camera, conf, mode='FHD', depth_range=None):
        self.geometry = geometry
        self.base_from_camera = np.asarray(base_from_camera, float)
        self.camera_from_base = np.linalg.inv(self.base_from_camera)
        if depth_range is None:
            # The disparity search must bracket the board wherever the camera is.
            # The ZED reads disparity about 40 px low (corrected after matching,
            # see ``points``), which makes the far edge look up to 1.5x further
            # than it is: 2026-10-13, with the camera raised to 0.84 m, a fixed
            # (0.4, 1.6) window lost the three far files of the board.
            reach = np.linalg.norm(np.asarray(geometry.corners, float)
                                   - self.base_from_camera[:3, 3], axis=1)
            depth_range = (0.6 * reach.min(), 1.8 * reach.max())
        self.rig = StereoRig.from_zed_conf(conf, mode, depth_range=depth_range)
        self.refined = None

    def refine(self, left, right):
        """Re-fit the lens tilt from this pair (drifts; once per camera session is enough)."""
        self.rig, report = fit_row_rotation(self.rig, *match_features(left, right))
        self.refined = report
        return report

    def _rectified_px(self, points_base):
        """Rectified-left pixel (x, y) and expected disparity of base-frame points."""
        cam = (np.c_[points_base, np.ones(len(points_base))] @ self.camera_from_base.T)[:, :3]
        rect = cam @ self.rig.R1.T
        x = self.rig.P1[0, 0] * rect[:, 0] / rect[:, 2] + self.rig.P1[0, 2]
        y = self.rig.P1[1, 1] * rect[:, 1] / rect[:, 2] + self.rig.P1[1, 2]
        return x, y, self.rig.focal_px * self.rig.baseline_m / rect[:, 2]

    def points(self, left, right, above_m=0.12):
        """Stereo points over the board: dict of base xyz, height, board uv, colour, stats."""
        if self.refined is None:
            self.refine(left, right)
        g = self.geometry
        outline = np.array([g.point(u, v) for u, v in ((0, 0), (1, 0), (1, 1), (0, 1))])
        x, y, _ = self._rectified_px(np.r_[outline, outline + [0, 0, above_m]])
        region = (x.min() - 10, y.min() - 10, x.max() + 10, y.max() + 10)
        disp, (lx, ty) = self.rig.disparity(left, right, region)
        # The disparity offset, measured on the board itself (see module notes).
        grid = np.array([g.point(u, v) for u in np.linspace(0.02, 0.98, 40)
                         for v in np.linspace(0.02, 0.98, 40)])
        bx, by, expected = self._rectified_px(grid)
        bx, by = np.round(bx - lx).astype(int), np.round(by - ty).astype(int)
        inside = (bx >= 0) & (by >= 0) & (bx < disp.shape[1]) & (by < disp.shape[0])
        measured = disp[by[inside], bx[inside]] - expected[inside]
        if np.isfinite(measured).sum() < 200:
            raise RuntimeError('too little of the board matched in stereo: is it in view and lit?')
        offset = float(np.nanmedian(measured))
        ys, xs = np.nonzero(np.isfinite(disp))
        hom = np.c_[xs + lx, ys + ty, disp[ys, xs] - offset, np.ones(len(xs))] @ self.rig.Q.T
        cam = (hom[:, :3] / hom[:, 3:4]) @ self.rig.R1
        base = (np.c_[cam, np.ones(len(cam))] @ self.base_from_camera.T)[:, :3]
        height = base[:, 2] - (g.plane[0] + g.plane[1] * base[:, 0] + g.plane[2] * base[:, 1])
        uv = board_uv(g, base[:, :2])
        window = (lx, ty, lx + disp.shape[1], ty + disp.shape[0])
        colour = self.rig.rectify(left, left, window)[0][ys, xs]
        flat = (uv >= 0).all(1) & (uv < 1).all(1) & (np.abs(height) < 0.01)
        return {'xyz': base, 'height': height, 'uv': uv, 'rgb': colour,
                'offset_px': offset,
                'board_noise_mm': float(1000 * np.subtract(*np.percentile(height[flat], [75, 25])))
                if flat.any() else None}

    def expected_pixels(self, xy, width_m=0.016, height_m=0.03):
        """Image area (px) of a small upright piece standing at ``xy``: scales the size test."""
        x0, y0 = xy
        z = self.geometry.z_at(x0, y0)
        cam_xy = self.base_from_camera[:2, 3]
        across = np.array([-(y0 - cam_xy[1]), x0 - cam_xy[0]])
        across /= np.linalg.norm(across)
        side = across * width_m / 2
        pts = np.array([[x0 - side[0], y0 - side[1], z], [x0 + side[0], y0 + side[1], z],
                        [x0, y0, z + height_m]])
        px, py, _ = self._rectified_px(pts)
        return abs(px[1] - px[0]) * abs(py[2] - py[0])


def board_uv(geometry, xy):
    """Board coordinates: u along the files (a=0 .. h=1), v along the ranks (1=0 .. 8=1)."""
    c = geometry.corners[:, :2]
    frame = np.c_[c[1] - c[0], c[3] - c[0]]
    return (np.asarray(xy, float) - c[0]) @ np.linalg.inv(frame).T


def without_robot(points, tips):
    """Drop the points that are the gripper (tool straight down, fingertips at ``tips``)."""
    xyz = points['xyz']
    robot = ((np.hypot(xyz[:, 0] - tips[0], xyz[:, 1] - tips[1]) < ROBOT_RADIUS_M)
             & (xyz[:, 2] > tips[2] - 0.015))
    return {k: (v[~robot] if isinstance(v, np.ndarray) and len(v) == len(xyz) else v)
            for k, v in points.items()}


def piece_centre(xyz, height, top, camera_xy, depth_bias=VISIBLE_SIDE_BIAS):
    """Axis of a piece from its points, in the base XY plane.

    Across the line of sight the visible outline is symmetric about the axis:
    the midpoint of its two edges is unbiased. Along the line of sight the
    camera sees only its own half of a round piece, whose points sit about
    2r/pi in front of the axis (r from the width across): ``depth_bias``,
    to be checked against a piece placed at a known spot.
    """
    band = height > 0.5 * top
    xy = xyz[band, :2] if band.sum() >= 10 else xyz[:, :2]
    rough = np.median(xy, axis=0)
    away = rough - np.asarray(camera_xy, float)
    away /= np.linalg.norm(away)
    across = np.array([-away[1], away[0]])
    side = xy @ across
    lo, hi = np.percentile(side, 5), np.percentile(side, 95)
    radius = (hi - lo) / 2
    along = np.median(xy @ away) + min(depth_bias * radius, 0.01)
    return away * along + across * (lo + hi) / 2


def scan(points, board_camera, min_points=120, size_fraction=0.25, low_fraction=0.12):
    """Per square: occupied or not, top height, centre, colour; plus coverage of the empties."""
    g = board_camera.geometry
    xyz, h, uv, rgb = points['xyz'], points['height'], points['uv'], points['rgb']
    on = (uv >= 0).all(1) & (uv < 1).all(1)
    fr = np.floor(uv * 8).astype(int)
    camera_xy = board_camera.base_from_camera[:2, 3]
    squares = {}
    for f in range(8):
        for r in range(8):
            name = square_name(f, r)
            here = on & (fr[:, 0] == f) & (fr[:, 1] == r)
            raised = here & (h > 0.008) & (h < MAX_PIECE_M)
            surface = int((here & (np.abs(h) < 0.006)).sum())
            entry = {'occupied': False, 'surface_points': surface, 'points': int(raised.sum())}
            if raised.sum() >= min_points:
                hh = h[raised]
                top = float(np.percentile(hh, 95))
                low = float(np.mean(hh < 0.5 * top))
                need = max(min_points,
                           size_fraction * board_camera.expected_pixels(g.square_xy(name)))
                if top >= MIN_TOP_M and low >= low_fraction and raised.sum() >= need:
                    body = raised & (h > 0.3 * top) & (h < 0.9 * top)
                    entry.update(occupied=True, top_m=top,
                                 centre=piece_centre(xyz[raised], hh, top, camera_xy,
                                                     DEPTH_BIAS).tolist(),
                                 rgb=np.median(rgb[body], axis=0).tolist() if body.any() else None)
            # A square neither seen bare nor holding a piece is hidden (robot, tall piece).
            entry['seen'] = entry['occupied'] or surface >= 40
            squares[name] = entry
    return squares


def piece_colours(squares, position):
    """Label occupied squares white/black from the colours of the pieces we know about."""
    import chess
    known = {chess.WHITE: [], chess.BLACK: []}
    for name, entry in squares.items():
        piece = position.piece_at(chess.parse_square(name))
        if entry['occupied'] and entry.get('rgb') is not None and piece is not None:
            known[piece.color].append(entry['rgb'])
    if not known[chess.WHITE] or not known[chess.BLACK]:
        # One side only (a test position): brightness alone, lighter = white.
        return {n: (sum(e['rgb']) > 3 * 128) for n, e in squares.items()
                if e['occupied'] and e.get('rgb') is not None}
    white, black = (np.median(known[c], axis=0) for c in (chess.WHITE, chess.BLACK))
    return {n: bool(np.linalg.norm(np.subtract(e['rgb'], white))
                    < np.linalg.norm(np.subtract(e['rgb'], black)))
            for n, e in squares.items() if e['occupied'] and e.get('rgb') is not None}


def infer_move(position, squares, colours=None):
    """The legal move whose result best matches the scan: (move, margin, ranking).

    Each candidate position is compared square by square with what was seen:
    occupied or empty, and the colour where occupied. Hidden squares count
    for nothing. A margin of 0 means two moves explain the scan equally well.
    """
    import chess
    colours = colours or {}
    ranking = []
    for move in position.legal_moves:
        after = position.copy()
        after.push(move)
        score = 0
        for name, entry in squares.items():
            if not entry['seen']:
                continue
            piece = after.piece_at(chess.parse_square(name))
            if (piece is not None) != entry['occupied']:
                score -= 1
            elif piece is not None and name in colours and colours[name] != piece.color:
                score -= 1
        ranking.append((score, move))
    ranking.sort(key=lambda s: -s[0])
    if not ranking:
        return None, 0, []
    margin = ranking[0][0] - (ranking[1][0] if len(ranking) > 1 else -99)
    return ranking[0][1], margin, ranking[:5]


def infer_move_diff(position, before, after):
    """The legal move whose occupancy CHANGES best match two scans: (move, margin, ranking).

    ``before`` is the scan after the arm's last move (the student has not
    touched the board), ``after`` the scan after the student's move. Only
    squares seen in both count, and only whether they changed: a back-rank
    square the camera always misses, or always reads the same, says nothing
    either way. That is what the absolute comparison (``infer_move``) gets
    wrong on this board (spec S6).
    """
    import chess
    both = [n for n, e in before.items() if e['seen'] and after.get(n, {}).get('seen')]
    observed = {n: before[n]['occupied'] != after[n]['occupied'] for n in both}
    ranking = []
    for move in position.legal_moves:
        result = position.copy()
        result.push(move)
        score = 0
        for name in both:
            square = chess.parse_square(name)
            expected = (position.piece_at(square) is not None) != (result.piece_at(square) is not None)
            if expected != observed[name]:
                score -= 1
        ranking.append((score, move))
    ranking.sort(key=lambda s: -s[0])
    if not ranking:
        return None, 0, []
    margin = ranking[0][0] - (ranking[1][0] if len(ranking) > 1 else -99)
    return ranking[0][1], margin, ranking[:5]


def board_shift(base_from_camera, apparent_base_from_camera):
    """How the board moved (4x4, base frame), from the camera pose the grid now implies.

    The grid is matched against the OLD board corners. If the board moved by
    B, that fit returns a camera C' with inv(C') = inv(C) @ B, so B = C @ inv(C').
    """
    return np.asarray(base_from_camera) @ np.linalg.inv(apparent_base_from_camera)


def planar_motion(dx, dy, turn_rad, centre):
    """4x4: turn about the vertical through ``centre``, then slide by (dx, dy)."""
    c, s = math.cos(turn_rad), math.sin(turn_rad)
    motion = np.eye(4)
    motion[:2, :2] = [[c, -s], [s, c]]
    motion[:2, 3] = np.asarray(centre[:2]) - motion[:2, :2] @ np.asarray(centre[:2]) + (dx, dy)
    return motion


def board_motion(world, pixels, base_from_camera, matrix, dist, centre):
    """How the board slid and turned on the table, seen by the FIXED camera: (4x4, rms px).

    ``world`` are grid corners where the calibration has them, ``pixels`` where
    the camera sees them now. Only three numbers are free (x, y, turn): far
    better posed than re-solving the camera's six from a partial grid.
    """
    from scipy.optimize import least_squares
    camera_from_base = np.linalg.inv(base_from_camera)
    rvec = cv2.Rodrigues(camera_from_base[:3, :3])[0]
    tvec = camera_from_base[:3, 3]
    world = np.asarray(world, float)
    pixels = np.asarray(pixels, float)
    dist = np.asarray(dist, float)

    def residual(p):
        moved = np.ascontiguousarray(shifted_corners(world, planar_motion(*p, centre)))
        return (cv2.projectPoints(moved, rvec, tvec, matrix, dist)[0].reshape(-1, 2)
                - pixels).ravel()

    fit = least_squares(residual, np.zeros(3), x_scale=[0.01, 0.01, 0.01])
    rms = float(np.sqrt(np.mean(fit.fun.reshape(-1, 2) ** 2 * 2)))
    return planar_motion(*fit.x, centre), rms


def shifted_corners(corners, shift):
    c = np.asarray(corners, float)
    return (np.c_[c, np.ones(len(c))] @ np.asarray(shift).T)[:, :3]


def shift_size(shift, centre):
    """(how far the board centre moved in mm, rotation in deg about vertical)."""
    shift = np.asarray(shift)
    moved = shifted_corners([centre], shift)[0] - np.asarray(centre, float)
    return (float(1000 * np.hypot(moved[0], moved[1])),
            float(math.degrees(math.atan2(shift[1, 0], shift[0, 0]))))
