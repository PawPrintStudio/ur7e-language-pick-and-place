"""Tabletop localization from ONE ordinary camera, anchored to a printed board.

Why this exists
---------------
The pipeline was designed around a depth camera: a mask plus a depth image
gives a 3-D point directly. A laptop webcam has no depth, so one pixel is a
*ray*, not a point -- the object could be anywhere along it. What turns the
ray back into a point is a known surface: everything we pick sits on the
table, and a ray meets a plane exactly once.

So the camera has to know where the table is. A printed ChArUco board taped
to the table tells it, in every single image:

1. The board's chessboard corners are found in the image (sub-pixel).
2. ``solvePnP`` recovers the camera pose relative to the board -- the board's
   real size is known, so the answer is in metres.
3. Once, at calibration time, the robot's fingertips touch the board's four
   outer corners. That fixes where the board is in ``base_link``.

Camera -> board comes from the picture, board -> robot from the touches, so
camera -> robot is their product. Because step 2 is repeated per capture, a
bumped laptop re-solves itself; only moving the *board* needs new touches.

What one camera cannot see is height. A tall object's silhouette covers its
footprint plus a shadow on the table *behind* it (further from the camera);
taken at face value the object looks longer than it is and its centre slides
away from the camera. If the height is known the shadow's length and
direction are known too, and it can be subtracted (:func:`table_footprint`).
The height comes from the caller (``object_height``), which is the honest
limit of this workaround and the reason the depth camera is still the
designed sensor.

No ROS here: this module is pure geometry and is unit-tested on rendered
images (``test/test_monocular.py``).
"""
from dataclasses import dataclass, field
import json
import math

import cv2
import numpy as np

from .core import Grasp, PerceptionError, fit_calibration, rigid_transform
from .fiducials import dictionary

# 5 x 7 squares of 35 mm = 175 x 245 mm: fits US Letter and A4 with margins.
COLUMNS, ROWS = 5, 7
SQUARE_M, MARKER_M = 0.035, 0.026


def table_board(square_m=SQUARE_M, marker_m=MARKER_M, columns=COLUMNS, rows=ROWS):
    """Return the ChArUco board that is taped to the table."""
    return cv2.aruco.CharucoBoard_create(columns, rows, square_m, marker_m, dictionary())


def touch_points(square_m=SQUARE_M, columns=COLUMNS, rows=ROWS):
    """Return the four outer chessboard corners in the board frame (4x3, m).

    Order is counter-clockwise seen from above the paper, starting at the
    board origin; the printable sheet numbers them 1-4 in this order.
    """
    w, h = columns * square_m, rows * square_m
    return np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]], dtype=float)


def detect_corners(rgb, board):
    """Return {corner_id: pixel} for the board's chessboard corners in one image."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary())
    if ids is None or len(ids) < 2:
        return {}
    count, points, point_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if not count:
        return {}
    return {int(i): p.astype(float) for i, p in zip(point_ids.ravel(), points.reshape(-1, 2))}


def detect_board(rgb, board, minimum=8):
    """Find the board's chessboard corners. Return (pixels Nx2, board_xyz Nx3).

    ``rgb`` may be one image or a list of images from a camera that did not
    move: corners seen in at least half of them are averaged, which divides
    the detection noise by roughly the square root of the number of frames.
    """
    frames = rgb if isinstance(rgb, (list, tuple)) else [rgb]
    seen = {}
    for frame in frames:
        for corner_id, pixel in detect_corners(frame, board).items():
            seen.setdefault(corner_id, []).append(pixel)
    if not seen:
        raise PerceptionError('table board not visible')
    kept = sorted(i for i, pixels in seen.items() if 2 * len(pixels) >= len(frames))
    if len(kept) < minimum:
        raise PerceptionError(f'only {len(kept)} board corners visible; need {minimum} '
                              '(is the arm or an object covering the board?)')
    pixels = np.array([np.median(seen[i], axis=0) for i in kept])
    xyz = np.asarray(board.chessboardCorners, dtype=float)[kept]
    return pixels, xyz


def calibrate_intrinsics(frames, board, size):
    """Estimate focal length and radial distortion from several board views.

    One view of a small flat board barely constrains the focal length (the
    test suite shows a 9 % error from a perfectly good image). Views from
    different angles and distances fix that. Returns (focal_px, dist, rms_px,
    views_used); ``dist`` is OpenCV's 5-element vector with only k1 estimated.
    """
    all_points, all_ids = [], []
    for frame in frames:
        found = detect_corners(frame, board)
        if len(found) >= 10:
            ids = sorted(found)
            all_ids.append(np.array(ids, dtype=np.int32).reshape(-1, 1))
            all_points.append(np.array([found[i] for i in ids], dtype=np.float32)
                              .reshape(-1, 1, 2))
    if len(all_points) < 6:
        raise PerceptionError(f'only {len(all_points)} usable board views; need at least 6 '
                              'from clearly different angles')
    guess = intrinsics(size[0], size)
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT
             | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST
             | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    rms, k, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
        all_points, all_ids, board, size, guess, np.zeros(5), flags=flags)
    if not np.isfinite(rms) or rms > 2.0:
        raise PerceptionError(f'intrinsic calibration did not converge (RMS {rms:.2f} px)')
    return float(k[0, 0]), dist.ravel().tolist(), float(rms), len(all_points)


def intrinsics(focal_px, size):
    """Pinhole matrix with the principal point at the image centre."""
    width, height = size
    return np.array([[focal_px, 0, width / 2], [0, focal_px, height / 2], [0, 0, 1]],
                    dtype=float)


def board_pose(pixels, xyz, k):
    """Return (camera_from_board 4x4, RMS reprojection error in pixels).

    A flat target has two mathematically valid poses; IPPE returns both and we
    keep the one that reprojects best with the camera on the printed side.
    """
    found, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        xyz.reshape(-1, 1, 3), pixels.reshape(-1, 1, 2), k, None, flags=cv2.SOLVEPNP_IPPE)
    best = None
    for rvec, tvec in zip(rvecs, tvecs) if found else []:
        transform = np.eye(4)
        transform[:3, :3] = cv2.Rodrigues(rvec)[0]
        transform[:3, 3] = tvec.ravel()
        # Camera centre in board coordinates must be above the paper (z > 0).
        if (-transform[:3, :3].T @ transform[:3, 3])[2] <= 0:
            continue
        projected = cv2.projectPoints(xyz, rvec, tvec, k, None)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((projected - pixels) ** 2, axis=1))))
        if best is None or rms < best[1]:
            best = (transform, rms)
    if best is None:
        raise PerceptionError('board pose could not be estimated')
    return best


def estimate_focal(pixels, xyz, size):
    """Estimate the focal length (pixels) from one oblique view of the board.

    Nobody calibrated this webcam, but a flat pattern seen at an angle shows
    perspective (far squares look smaller), and how strong that effect is
    depends on the focal length. So: try focal lengths, keep the one whose
    best pose reprojects the corners most exactly. Assumes square pixels, a
    centred principal point and no lens distortion -- fine for a laptop
    webcam, and the reprojection error reported tells you if it is not.
    """
    width = size[0]
    low, high = 0.4 * width, 3.0 * width

    def cost(focal):
        try:
            return board_pose(pixels, xyz, intrinsics(focal, size))[1]
        except PerceptionError:
            return float('inf')

    # Coarse scan (the cost is not guaranteed unimodal), then golden section.
    grid = np.geomspace(low, high, 40)
    costs = [cost(f) for f in grid]
    i = int(np.argmin(costs))
    a, b = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
    ratio = (math.sqrt(5) - 1) / 2
    c, d = b - ratio * (b - a), a + ratio * (b - a)
    for _ in range(40):
        if cost(c) < cost(d):
            b, d = d, c
            c = b - ratio * (b - a)
        else:
            a, c = c, d
            d = a + ratio * (b - a)
    focal = (a + b) / 2
    return float(focal), cost(focal)


@dataclass
class View:
    """What one image says about the camera: intrinsics and pose vs the board."""

    k: np.ndarray
    camera_from_board: np.ndarray
    rms_px: float
    corners: int

    @property
    def board_from_camera(self):
        """Return the camera pose expressed in the board frame."""
        return np.linalg.inv(self.camera_from_board)


def observe_board(rgb, board, focal_px=None, max_rms_px=2.5):
    """Detect the board in ``rgb`` (one image or a list) and solve the camera pose.

    Without ``focal_px`` the focal length is estimated from this single view,
    which is quick but loose; pass the value from :func:`calibrate_intrinsics`.
    """
    first = rgb[0] if isinstance(rgb, (list, tuple)) else rgb
    size = (first.shape[1], first.shape[0])
    pixels, xyz = detect_board(rgb, board)
    if focal_px is None:
        focal_px, _ = estimate_focal(pixels, xyz, size)
    k = intrinsics(focal_px, size)
    transform, rms = board_pose(pixels, xyz, k)
    if rms > max_rms_px:
        raise PerceptionError(f'board reprojection error {rms:.1f} px is too high '
                              '(blurred image, bent paper, or wrong focal length)')
    return View(k, rigid_transform(transform), rms, len(pixels))


@dataclass
class TableCalibration:
    """Where the board is in the robot's frame, plus the camera's focal length."""

    base_from_board: np.ndarray
    square_m: float = SQUARE_M
    marker_m: float = MARKER_M
    focal_px: float = 0.0
    dist: list = field(default_factory=lambda: [0.0] * 5)
    camera_matrix: list = None  # raw 3x3 (e.g. a factory calibration); None = ideal
    # Fixed-camera mode: a 4x4 camera pose in the robot frame, found once by
    # watching the robot move a held object (scripts/lab_camera_calibration.py).
    # No board is needed in view, but a bumped camera must be recalibrated.
    # ``base_from_board`` then just describes the table plane.
    camera_to_base: list = None
    image_size: tuple = (1280, 720)
    residual_mm: list = field(default_factory=list)
    source: str = 'physical'
    notes: dict = field(default_factory=dict)

    def board(self):
        """Return the board object matching the measured print."""
        return table_board(self.square_m, self.marker_m)

    def undistort(self, rgb):
        """Remove lens distortion so the pinhole maths downstream is exact.

        The output is the image an ideal camera would have taken: same focal
        length, no distortion, principal point at the image centre. That is
        the camera model every function in this module assumes.
        """
        if not self.focal_px or (not any(self.dist) and self.camera_matrix is None):
            return rgb
        size = (rgb.shape[1], rgb.shape[0])
        if getattr(self, '_maps_size', None) != size:
            ideal = intrinsics(self.focal_px, size)
            source = ideal if self.camera_matrix is None else np.array(self.camera_matrix)
            self._maps = cv2.initUndistortRectifyMap(
                source, np.array(self.dist, dtype=float), None, ideal, size, cv2.CV_16SC2)
            self._maps_size = size
        return cv2.remap(rgb, self._maps[0], self._maps[1], cv2.INTER_LINEAR)

    def save(self, path):
        """Write the calibration as JSON."""
        data = {key: value for key, value in self.__dict__.items() if not key.startswith('_')}
        data.update(base_from_board=np.asarray(self.base_from_board).tolist(),
                    image_size=list(self.image_size))
        with open(path, 'w') as stream:
            json.dump(data, stream, indent=2)
            stream.write('\n')

    @classmethod
    def load(cls, path):
        """Read a calibration written by :meth:`save` and validate it."""
        with open(path) as stream:
            data = json.load(stream)
        data['base_from_board'] = rigid_transform(data['base_from_board'])
        data['image_size'] = tuple(data['image_size'])
        if data.get('camera_to_base') is not None:
            data['camera_to_base'] = rigid_transform(data['camera_to_base']).tolist()
        return cls(**data)

    def fixed_view(self, size):
        """Return the stored camera pose as a :class:`View` (fixed-camera mode)."""
        if self.camera_to_base is None or not self.focal_px:
            raise PerceptionError('calibration has no fixed camera pose')
        board_from_camera = np.linalg.inv(self.base_from_board) @ np.array(self.camera_to_base)
        return View(intrinsics(self.focal_px, size), np.linalg.inv(board_from_camera), 0.0, 0)


def fit_board_anchor(tip_points_base, square_m=SQUARE_M):
    """Fit base_from_board from the four fingertip touches (base_link, m).

    ``tip_points_base`` is 4x3 in the order of :func:`touch_points`. Returns
    (base_from_board, per-point residuals in metres). The fit is rigid (no
    scale), so a residual of several millimetres means a touch was off or the
    print is not the size the software thinks it is.
    """
    return fit_calibration(touch_points(square_m), np.asarray(tip_points_base, dtype=float))


def camera_to_base(view, base_from_board):
    """Chain camera -> board (this image) with board -> robot (the touches)."""
    return rigid_transform(np.asarray(base_from_board) @ view.board_from_camera)


def rays_to_plane(pixels, k, cam_to_base, plane_point, plane_normal):
    """Intersect pixel rays with a plane. Return Nx3 points in the base frame.

    Rays that run parallel to the plane or hit it behind the camera are
    returned as NaN rows; callers must drop them.
    """
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
    directions = np.column_stack(((pixels[:, 0] - k[0, 2]) / k[0, 0],
                                  (pixels[:, 1] - k[1, 2]) / k[1, 1],
                                  np.ones(len(pixels))))
    directions = directions @ cam_to_base[:3, :3].T
    origin = cam_to_base[:3, 3]
    normal = np.asarray(plane_normal, dtype=float)
    denominator = directions @ normal
    with np.errstate(divide='ignore', invalid='ignore'):
        scale = ((np.asarray(plane_point, dtype=float) - origin) @ normal) / denominator
    scale[(np.abs(denominator) < 1e-9) | (scale <= 0)] = np.nan
    return origin + directions * scale[:, None]


def project(points_base, k, cam_to_base):
    """Project base-frame points into the image. Return Nx2 pixels."""
    base_to_cam = np.linalg.inv(cam_to_base)
    points = np.asarray(points_base, dtype=float).reshape(-1, 3)
    camera = points @ base_to_cam[:3, :3].T + base_to_cam[:3, 3]
    return np.column_stack((k[0, 0] * camera[:, 0] / camera[:, 2] + k[0, 2],
                            k[1, 1] * camera[:, 1] / camera[:, 2] + k[1, 2]))


def visible_table(k, cam_to_base, base_from_board, size, inset=0.04):
    """Return the patch of table the camera sees, as a base-frame XY polygon."""
    width, height = size
    dx, dy = inset * width, inset * height
    corners = [(dx, dy), (width - dx, dy), (width - dx, height - dy), (dx, height - dy)]
    points = rays_to_plane(corners, k, cam_to_base, base_from_board[:3, 3],
                           base_from_board[:3, 2])
    if not np.isfinite(points).all():
        raise PerceptionError('the camera sees above the table horizon; tilt it down')
    return points[:, :2]


def localize_on_table(mask, k, cam_to_base, base_from_board, object_height=0.03,
                      polygon=None, min_points=50):
    """Turn an instance mask into a top-down grasp without a depth image.

    The mask's silhouette is laid onto the table, the height shadow is
    removed (:func:`table_footprint`), and the footprint that remains gives
    the centre, the long axis (yaw) and the size. The returned point is on
    the object's top surface, matching :func:`core.localize`; ``extent`` is
    (length, width) of the footprint in metres.

    ``object_height=None`` means "unknown, compact object": the centre is
    then taken from the silhouette's near edge and its width across the
    view, which needs no height at all, and the returned z is the table.
    """
    if object_height is not None and (
            not np.isfinite(object_height) or not 0 <= object_height <= 0.3):
        raise PerceptionError('object height must be 0..0.3 m')
    selected = np.asarray(mask).astype(np.uint8)
    if selected.sum() < min_points:
        raise PerceptionError('mask too small to localize')
    footprint, grid_origin, cell = table_footprint(
        selected, k, cam_to_base, base_from_board, object_height or 0.0)
    rows, cols = np.nonzero(footprint)
    if len(rows) < min_points:
        raise PerceptionError(
            f'nothing is left of the object after removing a {object_height * 1000:.0f} mm '
            'height shadow: its height in the objects file is too large')
    # Board-frame metres of every footprint cell. Cells are equal areas of
    # table, so their mean is an area centroid with no bias toward the camera.
    xy = np.column_stack((cols, rows)) * cell + grid_origin
    if object_height is None:
        # Height unknown: use the near edge. The silhouette's edge nearest the
        # camera is where the object meets the table (no shadow there), and
        # across the viewing direction the silhouette is not smeared at all;
        # for a compact object (round or square footprint) the centre is half
        # that width behind the near edge. Measured on the lab hat 2026-10-02.
        camera = (np.linalg.inv(base_from_board) @ np.append(cam_to_base[:3, 3], 1.0))[:2]
        radial = xy.mean(axis=0) - camera
        radial /= np.linalg.norm(radial)
        tangent = np.array([-radial[1], radial[0]])
        along_r, along_t = (xy - camera) @ radial, (xy - camera) @ tangent
        near = np.percentile(along_r, 2)
        t0, t1 = np.percentile(along_t, 2), np.percentile(along_t, 98)
        width = t1 - t0 + cell
        center = camera + radial * (near + width / 2) + tangent * (t0 + t1) / 2
        keep = along_r <= near + width  # the footprint, shadow dropped
        xy = xy[keep] if keep.sum() >= min_points else xy
        object_height = 0.0
    else:
        center = xy.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov(xy.T))
    ratio = float(values[-1] / max(values[0], 1e-12))
    major, minor = vectors[:, -1], vectors[:, 0]
    along, across = (xy - center) @ major, (xy - center) @ minor
    extent = (float(np.ptp(along)) + cell, float(np.ptp(across)) + cell)
    rotation, origin = base_from_board[:3, :3], base_from_board[:3, 3]
    top = origin + rotation @ [center[0], center[1], object_height]
    if polygon is not None:
        polygon = np.asarray(polygon, dtype=np.float32)
        if cv2.pointPolygonTest(polygon, (float(top[0]), float(top[1])), False) < 0:
            raise PerceptionError('object is outside the workspace')
    axis = rotation @ [major[0], major[1], 0.0]
    yaw = (math.atan2(axis[1], axis[0]) + math.pi / 2) % math.pi - math.pi / 2
    # Variance ratio 2 = side ratio ~1.4: below that the long axis is noise.
    if ratio < 2.0:
        yaw = 0.0
    quaternion = np.array([math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0])
    grasp = Grasp(top, quaternion, yaw, int(len(rows)), ratio)
    grasp.extent = extent
    return grasp


def table_footprint(mask, k, cam_to_base, base_from_board, object_height, cell=0.001):
    """Return the object's footprint on the table as a metric grid.

    Seen from the side, a standing object covers its footprint *and* a
    "shadow" behind it: every point at height z lands on the table shifted
    directly away from the camera, by an amount proportional to z. For an
    object with vertical sides the silhouette on the table is therefore the
    footprint swept along one short line segment -- and sweeping can be
    undone: keep only the cells whose whole segment lies inside the
    silhouette (a morphological erosion). What remains is the footprint.

    Returns (grid uint8, grid origin xy in board metres, cell size). The grid
    is in the board frame, where the table is exactly the plane z = 0.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    outline = max(contours, key=cv2.contourArea).reshape(-1, 2)
    origin, normal = base_from_board[:3, 3], base_from_board[:3, 2]
    points = rays_to_plane(outline, k, cam_to_base, origin, normal)
    if not np.isfinite(points).all():
        raise PerceptionError('mask does not project onto the table')
    base_to_board = np.linalg.inv(base_from_board)
    xy = (points @ base_to_board[:3, :3].T + base_to_board[:3, 3])[:, :2]
    camera = base_to_board[:3, :3] @ cam_to_base[:3, 3] + base_to_board[:3, 3]
    if camera[2] <= object_height + 0.05:
        raise PerceptionError('camera is not above the object')
    # Shadow of the top face relative to the footprint (similar triangles).
    shadow = (xy.mean(axis=0) - camera[:2]) * object_height / (camera[2] - object_height)
    margin = 0.01
    grid_origin = xy.min(axis=0) - margin
    size = np.ceil((xy.max(axis=0) - grid_origin + margin) / cell).astype(int)
    if size.max() > 1500:
        raise PerceptionError('object silhouette is implausibly large')
    silhouette = np.zeros((size[1], size[0]), np.uint8)
    cv2.fillPoly(silhouette, [np.round((xy - grid_origin) / cell).astype(np.int32)], 1)
    footprint = silhouette.copy()
    steps = int(math.ceil(np.linalg.norm(shadow) / cell))
    for index in range(1, steps + 1):
        dx, dy = -shadow / cell * index / steps
        shift = np.array([[1, 0, dx], [0, 1, dy]], dtype=np.float32)
        footprint &= cv2.warpAffine(silhouette, shift, (int(size[0]), int(size[1])),
                                    flags=cv2.INTER_NEAREST)
    return footprint, grid_origin, cell


def color_mask(rgb, bbox, margin=0.25):
    """Segment the object inside a detection box by colour (GrabCut).

    The depth path separates object from table by height. Without depth the
    cue is appearance: pixels just outside the box are assumed to be table,
    and GrabCut grows a foreground region that differs from them.
    """
    height, width = rgb.shape[:2]
    x0, y0, x1, y1 = [float(value) for value in bbox]
    if not np.isfinite([x0, y0, x1, y1]).all() or x1 - x0 < 4 or y1 - y0 < 4:
        raise PerceptionError('empty detection box')
    mx, my = margin * (x1 - x0), margin * (y1 - y0)
    rx0, ry0 = max(0, int(x0 - mx)), max(0, int(y0 - my))
    rx1, ry1 = min(width, int(math.ceil(x1 + mx))), min(height, int(math.ceil(y1 + my)))
    roi = cv2.cvtColor(rgb[ry0:ry1, rx0:rx1], cv2.COLOR_RGB2BGR)
    rect = (max(1, int(x0) - rx0), max(1, int(y0) - ry0),
            min(int(x1 - x0), roi.shape[1] - 2), min(int(y1 - y0), roi.shape[0] - 2))
    labels = np.zeros(roi.shape[:2], np.uint8)
    cv2.grabCut(roi, labels, rect, np.zeros((1, 65)), np.zeros((1, 65)), 5,
                cv2.GC_INIT_WITH_RECT)
    foreground = ((labels == cv2.GC_FGD) | (labels == cv2.GC_PR_FGD)).astype(np.uint8)
    count, components, stats, _ = cv2.connectedComponentsWithStats(foreground)
    if count <= 1:
        raise PerceptionError('no foreground separable from the table by colour')
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = stats[largest, cv2.CC_STAT_AREA]
    if area < 0.05 * rect[2] * rect[3]:
        raise PerceptionError('foreground is too small inside the detection box')
    mask = np.zeros((height, width), np.uint8)
    mask[ry0:ry1, rx0:rx1] = components == largest
    return mask


# Hue ranges on OpenCV's 0-179 scale. Red wraps around zero.
COLOR_HUES = {
    'red': [(0, 8), (170, 179)], 'orange': [(9, 20)], 'yellow': [(21, 34)],
    'green': [(40, 85)], 'blue': [(95, 130)], 'purple': [(131, 160)],
}
COLOR_NAMES = tuple(COLOR_HUES) + ('white',)


def color_blob(rgb, color, region=None, min_area=150):
    """Find the largest saturated blob of a named colour. Return (mask, bbox).

    A fast, deterministic detector for coloured blocks: no model, a few
    milliseconds. ``region`` is an optional uint8 mask of where to look
    (the visible table), so a red laptop sticker is not a red block.
    """
    if color not in COLOR_NAMES:
        raise PerceptionError(f'no colour rule for "{color}"')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    selected = np.zeros(hsv.shape[:2], np.uint8)
    if color == 'white':
        # White has no hue: it is "bright and not colourful".
        selected = cv2.inRange(hsv, (0, 0, 175), (179, 70, 255))
    # Saturation floor: a black anodised breadboard reads as dark navy
    # (hue 110, S ~100, V ~70) and swallowed a whole plate as "blue" on
    # 2026-10-02; coloured toys and blocks sit well above S 150.
    floor = (150, 80) if color == 'blue' else (120, 80)
    for low, high in COLOR_HUES.get(color, []):
        selected |= cv2.inRange(hsv, (low, floor[0], floor[1]), (high, 255, 255))
    if region is not None:
        selected &= region
    selected = cv2.morphologyEx(selected, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, components, stats, _ = cv2.connectedComponentsWithStats(selected)
    areas = stats[1:, cv2.CC_STAT_AREA] if count > 1 else np.array([])
    order = np.argsort(areas)[::-1]
    if not len(order) or areas[order[0]] < min_area:
        raise PerceptionError(f'no {color} object on the table')
    if len(order) > 1 and areas[order[1]] > 0.5 * areas[order[0]]:
        raise PerceptionError(f'more than one {color} object; be more specific')
    index = int(order[0]) + 1
    x, y, w, h = stats[index, :4]
    return (components == index).astype(np.uint8), (x, y, x + w, y + h)


def marker_center(rgb, marker_id):
    """Return the pixel centre and corners of one ArUco marker (verification)."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary())
    matches = [] if ids is None else np.flatnonzero(ids.ravel() == marker_id).tolist()
    if len(matches) != 1:
        raise PerceptionError(f'marker {marker_id} missing or seen more than once')
    quad = corners[matches[0]].reshape(4, 2).astype(float)
    return quad.mean(axis=0), quad


def printable_sheets(dpi=300, page_inches=(8.5, 11.0)):
    """Render the pages to print: the table board, then verification markers.

    Returns a list of uint8 grayscale page images at ``dpi``. Corner numbers
    are placed by detecting the rendered board, so the labels agree with the
    board frame by construction rather than by convention.
    """
    page_w, page_h = int(page_inches[0] * dpi), int(page_inches[1] * dpi)
    per_square = int(round(SQUARE_M / 0.0254 * dpi))
    board = table_board()
    image = board.draw((COLUMNS * per_square, ROWS * per_square), marginSize=0, borderBits=1)
    page = np.full((page_h, page_w), 255, np.uint8)
    x0, y0 = (page_w - image.shape[1]) // 2, (page_h - image.shape[0]) // 2 - dpi // 8
    page[y0:y0 + image.shape[0], x0:x0 + image.shape[1]] = image
    pixels, xyz = detect_board(cv2.cvtColor(page, cv2.COLOR_GRAY2RGB), board)
    homography, _ = cv2.findHomography(xyz[:, :2], pixels)
    corners = cv2.perspectiveTransform(touch_points()[:, :2].reshape(-1, 1, 2), homography)
    middle = np.array([x0 + image.shape[1] / 2, y0 + image.shape[0] / 2])
    tick = dpi // 4
    for number, corner in enumerate(corners.reshape(-1, 2), start=1):
        cx, cy = [int(round(value)) for value in corner]
        cv2.line(page, (cx - tick, cy), (cx + tick, cy), 0, 3)
        cv2.line(page, (cx, cy - tick), (cx, cy + tick), 0, 3)
        outward = (corner - middle) / np.linalg.norm(corner - middle)
        label = corner + outward * dpi * 0.42
        cv2.putText(page, str(number), (int(label[0]) - 20, int(label[1]) + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, 0, 5)
    cv2.putText(page, 'UR7e table board - print at 100% (Actual size). '
                '5 squares must measure 175 mm.', (x0, page_h - dpi // 3),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)

    markers = np.full((page_h, page_w), 255, np.uint8)
    side = int(round(0.05 / 0.0254 * dpi))  # 50 mm black square
    for index, marker_id in enumerate((40, 41, 42, 43)):
        mx = page_w // 4 + (index % 2) * page_w // 2 - side // 2
        my = page_h // 4 + (index // 2) * page_h // 2 - side // 2
        markers[my:my + side, mx:mx + side] = cv2.aruco.drawMarker(dictionary(), marker_id, side)
        cv2.putText(markers, f'marker {marker_id} (50 mm) - cut out with a white border',
                    (mx - side // 2, my + side + dpi // 3), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
    return [page, markers]
