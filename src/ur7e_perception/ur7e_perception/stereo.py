"""Object height from BOTH lenses of a ZED, without the Stereolabs SDK.

Why this exists
---------------
``monocular.py`` localizes objects with one camera by intersecting pixel rays
with the table. Its honest limit is height: one camera cannot tell a tall
object from a long one, so heights come from a lookup table. A ZED has two
lenses 120 mm apart, and two views of the same point *do* fix its distance.
The SDK that normally turns the pair into depth needs an NVIDIA GPU; the
geometry does not, so it is done here with OpenCV on the CPU.

How two images become a distance
--------------------------------
A point seen by the left lens at column ``x`` appears in the right lens a
little further left, at ``x - d``. The shift ``d`` (the *disparity*, pixels)
is large for near points and zero at infinity::

    depth = focal_px * baseline_m / d

Finding ``d`` means searching the right image for the patch around each left
pixel. That search is only practical if it runs along one image row, which
real lenses do not give you: they distort, and the two sensors are not
perfectly parallel. *Rectification* fixes both. From the factory calibration
(each lens's intrinsics and distortion, plus the small rotation and the
translation between them) ``cv2.stereoRectify`` computes two virtual pinhole
cameras that share one image plane and have exactly parallel axes; warping
the raw images into those cameras puts every scene point on the same row in
both. Then semi-global block matching (``cv2.StereoSGBM``) does the row
search.

Three camera models meet in this module; keeping them apart is most of the
work:

* **raw left** -- the pixels the sensor delivers (distorted).
* **ideal left** -- what ``TableCalibration.undistort`` produces and every
  function in ``monocular.py`` assumes: same optical centre and orientation
  as raw left, no distortion, principal point at the image centre. Masks
  arrive in this image.
* **rectified left** -- the virtual camera used for matching. Same optical
  centre again, but rotated by ``R1`` (a fraction of a degree) and with its
  own pinhole matrix ``P1``.

All three share one optical centre, so moving a pixel between ideal and
rectified is a pure rotation of its ray: the homography
``P1[:, :3] @ R1 @ inv(K_ideal)``. Reconstructed points come out in the
rectified frame and are rotated back with ``R1.T`` into the raw/ideal left
camera frame (OpenCV optical: x right, y down, z forward), which is the frame
``cam_to_base`` describes.

What limits the answer
----------------------
* **Resolution.** One pixel of disparity is ``depth**2 / (focal * baseline)``
  metres of depth: about 5 mm at 0.8 m for this camera at 1080p, 2 mm at
  0.5 m, 11 mm at 1.2 m. SGBM interpolates to a fraction of a pixel, so the
  noise is smaller than that, but a *systematic* pixel (calibration drift) is
  not averaged away. Keep the camera close.
* **Texture.** Matching needs something to match. A plain table top yields
  few or no disparities; that is fine for the object's height (see
  :func:`object_height`) but means the table itself is usually not measured.
* **Calibration.** The factory file describes the camera as it left the
  factory. :meth:`StereoRig.rectification_error` measures, on any image pair,
  how well it still holds.

No ROS here: pure geometry, unit-tested on rendered stereo pairs
(``test/test_stereo.py``).
"""
import configparser
import math

import cv2
import numpy as np

from .camera import zed_factory_calibration
from .core import PerceptionError
from .monocular import intrinsics

# SGBM compares 5x5 patches; the two penalties are the usual 8 and 32 times
# the patch area (small for a one-pixel disparity change between neighbours,
# large for a jump), which lets surfaces slant but discourages speckle.
BLOCK = 5
# Extra rectified pixels matched around the requested ones, so the object's
# own pixels are never at the edge of the matching window.
PAD = 24


class StereoRig:
    """Both lenses of one ZED, from its factory calibration file."""

    def __init__(self, k_left, dist_left, k_right, dist_right, rotation, translation,
                 image_size, depth_range=(0.35, 3.0)):
        """Rectify two calibrated cameras that look roughly the same way.

        ``rotation`` (3x3) and ``translation`` (3, metres) follow OpenCV's
        stereo convention ``x_right = rotation @ x_left + translation``: for a
        right camera 120 mm to the right, ``translation`` is ``(-0.12, 0, 0)``.
        ``depth_range`` (metres) bounds the disparity search; a narrower range
        is faster and leaves less room for false matches.
        """
        self.k_left = np.asarray(k_left, dtype=float)
        self.k_right = np.asarray(k_right, dtype=float)
        self.dist_left = np.asarray(dist_left, dtype=float).ravel()
        self.dist_right = np.asarray(dist_right, dtype=float).ravel()
        self.rotation = np.asarray(rotation, dtype=float)
        self.translation = np.asarray(translation, dtype=float).reshape(3)
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.depth_range = (float(depth_range[0]), float(depth_range[1]))
        if not 0 < self.depth_range[0] < self.depth_range[1]:
            raise PerceptionError('depth_range must be (near, far) with 0 < near < far')
        # alpha=0 zooms the virtual cameras in just enough that every rectified
        # pixel comes from real image content (no black wedges to mis-match).
        # The price is a strip of a few dozen pixels at the raw image border
        # that the rectified image does not show.
        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
            self.k_left, self.dist_left, self.k_right, self.dist_right, self.image_size,
            self.rotation, self.translation, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.focal_px = float(self.P1[0, 0])
        # P2's last column is -focal * baseline for a camera displaced along x.
        self.baseline_m = float(-self.P2[0, 3] / self.P2[0, 0])
        if not self.baseline_m > 1e-3 or abs(self.P2[1, 3]) > 1e-9:
            raise PerceptionError('not a left/right stereo pair: the second camera must be '
                                  'to the right of the first (translation x negative)')
        # For every rectified pixel, where to read the raw image (float maps:
        # they are sliced to rectify only the window an object needs).
        self.maps_left = cv2.initUndistortRectifyMap(
            self.k_left, self.dist_left, self.R1, self.P1, self.image_size, cv2.CV_32FC1)
        self.maps_right = cv2.initUndistortRectifyMap(
            self.k_right, self.dist_right, self.R2, self.P2, self.image_size, cv2.CV_32FC1)

    @classmethod
    def from_zed_conf(cls, path, mode='FHD', rational=None, depth_range=(0.35, 3.0)):
        """Build the rig from a Stereolabs ``SN<serial>.conf`` factory file.

        The ``[STEREO]`` section gives the right lens relative to the left:
        ``Baseline``, ``TY``, ``TZ`` in millimetres and ``RX``, ``CV``
        ("convergence", the rotation about Y) and ``RZ`` in radians. They are
        used as OpenCV's ``T = (-Baseline, TY, TZ)`` and the rotation vector
        ``(RX, CV, RZ)``, the convention of Stereolabs' own OpenCV example.
        What one real (blurred, distant) frame from this camera could confirm:
        the sign of ``RX`` by row alignment (27 px of raw vertical offset
        drops to about 1 px; flipping ``RX`` leaves 23 px, ignoring the
        rotation 11 px), the sign of ``CV`` by disparities staying positive
        (flipped, half of them go negative, i.e. "beyond infinity"), and
        ``RZ`` weakly (flipped, the row error grows across the image).
        ``TY``/``TZ`` are half a millimetre and invisible on a distant scene;
        at table range a wrong ``TY`` sign would cost about a pixel of row
        alignment, so run :meth:`rectification_error` on a sharp table frame.

        ``rational`` picks the distortion model: ``None`` (default) takes
        whatever :func:`camera.zed_factory_calibration` returns, so both eyes
        use the same model as the single-camera pipeline that produced the
        "ideal left" image; ``False`` forces the five-coefficient model of
        the per-mode section, ``True`` requires ``[LEFT_DISTO]``.
        """
        k_left, dist_left, size = zed_factory_calibration(path, mode, 'LEFT')
        k_right, dist_right, _ = zed_factory_calibration(path, mode, 'RIGHT')
        parser = configparser.ConfigParser()
        parser.read(path)
        if rational is True and (len(dist_left) != 8 or len(dist_right) != 8):
            raise PerceptionError(f'{path} has no [LEFT_DISTO]/[RIGHT_DISTO] rational model')
        if rational is False:
            names = ('k1', 'k2', 'p1', 'p2', 'k3')
            dist_left, dist_right = [[float(parser[f'{side}_CAM_{mode}'][key]) for key in names]
                                     for side in ('LEFT', 'RIGHT')]
        if 'STEREO' not in parser:
            raise PerceptionError(f'{path} has no [STEREO] section')
        stereo = parser['STEREO']   # configparser lower-cases the keys

        def value(name):
            # Rotations are stored per resolution (RX_FHD); TY/TZ are in some files too.
            text = stereo.get(f'{name}_{mode}'.lower(), stereo.get(name.lower(), '0'))
            return float(text)

        if value('Baseline') <= 0:
            raise PerceptionError(f'{path} has no usable Baseline in [STEREO]')
        rotation = cv2.Rodrigues(np.array([value('RX'), value('CV'), value('RZ')]))[0]
        translation = np.array([-value('Baseline'), value('TY'), value('TZ')]) / 1000.0
        return cls(k_left, dist_left, k_right, dist_right, rotation, translation, size,
                   depth_range)

    def disparity_limits(self):
        """Return (lowest, count) of the disparities searched, in pixels.

        Disparity is ``focal * baseline / depth``, so the far limit of
        ``depth_range`` gives the smallest disparity and the near limit the
        largest. SGBM wants the count as a multiple of 16.
        """
        scale = self.focal_px * self.baseline_m
        lowest = max(1, int(math.floor(scale / self.depth_range[1])))
        highest = int(math.ceil(scale / self.depth_range[0]))
        return lowest, 16 * int(math.ceil((highest - lowest + 1) / 16))

    def depth_per_pixel(self, depth_m):
        """Return how many metres of depth one pixel of disparity is worth at ``depth_m``.

        From ``depth = focal * baseline / d``: a change of one pixel in ``d``
        changes the depth by ``depth**2 / (focal * baseline)``. It grows with
        the square of the distance, which is why the camera should be close.
        """
        return depth_m ** 2 / (self.focal_px * self.baseline_m)

    def ideal_to_rectified(self, pixels, focal_ideal_px):
        """Map Nx2 pixels of the ideal left image into the rectified left image.

        Both are pinhole views from the same optical centre, so a pixel's ray
        is un-projected with the ideal matrix, rotated by ``R1`` and projected
        with ``P1``: one 3x3 homography, no depth needed.
        """
        pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
        k_ideal = intrinsics(focal_ideal_px, self.image_size)
        homography = self.P1[:, :3] @ self.R1 @ np.linalg.inv(k_ideal)
        mapped = np.column_stack((pixels, np.ones(len(pixels)))) @ homography.T
        return mapped[:, :2] / mapped[:, 2:3]

    def rectify(self, left_raw, right_raw, window=None):
        """Warp a raw pair into the rectified cameras. Return (left, right).

        ``window`` = (x0, y0, x1, y1) in rectified pixels warps only that
        part, which is what makes one object cheap: the lookup maps are
        simply sliced.
        """
        width, height = self.image_size
        for image in (left_raw, right_raw):
            if image.shape[:2] != (height, width):
                raise PerceptionError(f'stereo images must be {width}x{height} per eye, '
                                      f'got {image.shape[1]}x{image.shape[0]}')
        x0, y0, x1, y1 = window if window is not None else (0, 0, width, height)
        rows, cols = slice(y0, y1), slice(x0, x1)
        return tuple(
            cv2.remap(image, maps[0][rows, cols], maps[1][rows, cols], cv2.INTER_LINEAR)
            for image, maps in ((left_raw, self.maps_left), (right_raw, self.maps_right)))

    def disparity(self, left_raw_rgb, right_raw_rgb, region=None):
        """Match the pair. Return (disparity float32, (x, y) of its top-left pixel).

        The disparity image is in rectified-left pixels; NaN marks pixels
        with no trustworthy match. ``region`` = (x0, y0, x1, y1) in rectified
        pixels limits the work to that box plus a margin. The box is widened
        to the *left* by the largest disparity searched, because the match
        for a left pixel lies that far to the left in the right image. (Things
        at the very left of the left image are simply not in the right
        image; no amount of matching recovers those.)

        Three filters decide what "trustworthy" means:

        * uniqueness -- the best match must beat the runner-up by 10 %,
          which removes blank and repetitive surfaces;
        * left-right consistency -- matching right-to-left must land on the
          same pixel (within one), which removes surfaces only one eye sees;
        * speckle -- small islands of disparity unlike their surroundings
          are deleted.
        """
        width, height = self.image_size
        lowest, count = self.disparity_limits()
        x0, y0, x1, y1 = (0, 0, width, height) if region is None else region
        # SGBM gives no answer for the first (lowest + count) columns of what
        # it is handed, so the window starts that far to the left ...
        start = int(x0) - PAD - lowest - count
        window = (max(0, start), max(0, int(y0) - PAD),
                  min(width, int(x1) + PAD + 1), min(height, int(y1) + PAD + 1))
        left, right = self.rectify(_gray(left_raw_rgb), _gray(right_raw_rgb), window)
        # ... and where the image itself ends first, black columns stand in,
        # so that objects near the left border are still matched.
        missing = max(0, -start)
        if missing:
            left, right = [cv2.copyMakeBorder(image, 0, 0, missing, 0, cv2.BORDER_CONSTANT,
                                              value=0) for image in (left, right)]
        matcher = cv2.StereoSGBM_create(
            minDisparity=lowest, numDisparities=count, blockSize=BLOCK,
            P1=8 * BLOCK * BLOCK, P2=32 * BLOCK * BLOCK, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=1,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        fixed = matcher.compute(left, right)[:, missing:]   # int16: disparity * 16
        result = fixed.astype(np.float32) / 16.0
        result[fixed < lowest * 16] = np.nan     # SGBM's "no match" is lowest - 1
        # A match further left than the right image's first column was a
        # match with the black stand-in columns, not with the scene.
        columns = window[0] + np.arange(result.shape[1], dtype=np.float32)
        with np.errstate(invalid='ignore'):
            result[result > columns] = np.nan
        return result, window[:2]

    def points_for_pixels(self, left_raw_rgb, right_raw_rgb, pixels, focal_ideal_px):
        """Return the 3-D point seen at each given pixel of the ideal left image.

        ``pixels`` is Nx2 (u, v) in the IDEAL left image: the raw left image
        undistorted to a pinhole with focal length ``focal_ideal_px`` and the
        principal point at the image centre (``TableCalibration.undistort``).
        The result is Nx3 in the raw left camera's optical frame, metres;
        rows are NaN where no reliable disparity was found (blank surface,
        hidden from the right eye, outside the rectified image, or nearer or
        farther than ``depth_range``).

        Each point lies exactly on the ray of the pixel that was asked for;
        only its distance along that ray comes from the stereo match (the
        disparity of the nearest rectified pixel).
        """
        pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
        points = np.full((len(pixels), 3), np.nan)
        width, height = self.image_size
        rectified = self.ideal_to_rectified(pixels, focal_ideal_px)
        nearest = np.round(rectified)
        inside = (np.isfinite(rectified).all(axis=1)
                  & (nearest[:, 0] >= 0) & (nearest[:, 0] < width)
                  & (nearest[:, 1] >= 0) & (nearest[:, 1] < height))
        if not inside.any():
            return points
        nearest = nearest[inside].astype(int)
        region = (*nearest.min(axis=0), *nearest.max(axis=0))
        disparity, (left_edge, top_edge) = self.disparity(left_raw_rgb, right_raw_rgb, region)
        found = disparity[nearest[:, 1] - top_edge, nearest[:, 0] - left_edge]
        # Q turns (x, y, disparity, 1) into homogeneous (X, Y, Z, W): written
        # out, Z = focal * baseline / d, X = (x - cx) * Z / focal, same for Y.
        homogeneous = np.column_stack(
            (rectified[inside], found, np.ones(len(found)))) @ self.Q.T
        with np.errstate(invalid='ignore', divide='ignore'):
            in_rectified = homogeneous[:, :3] / homogeneous[:, 3:4]
        # Rectified frame -> raw left frame is R1.T; for row vectors p @ R1.
        points[inside] = in_rectified @ self.R1
        return points

    def rectification_error(self, left_raw_rgb, right_raw_rgb, patch=65, step=32,
                            max_dy=8, min_texture=60.0, min_score=0.9, max_patches=400):
        """Measure how well the calibration lines the two images up row for row.

        After a perfect rectification a scene point has the *same row* in
        both images and a positive column shift. This checks it directly:
        patches of the rectified left image are searched for in the rectified
        right image in two dimensions, and the vertical part of each best
        match is the error. SGBM assumes that error is zero; beyond roughly a
        pixel it starts losing matches.

        Large patches rather than corner features, because this has to work
        on ordinary, even blurred, frames: only patches with texture in both
        directions are used (an edge can slide along itself and fake a
        vertical shift), blown-out highlights are skipped, and the match
        must correlate better than ``min_score``. A calibration error is
        smooth across the image, so a big patch loses nothing and averages
        the matching noise down; the search is limited to ``max_dy`` rows,
        so a grossly wrong calibration shows up as "too few matches" or as
        an error of several pixels, not as an exact number.

        Returns a dict: ``median_abs_dy`` and ``median_dy`` (pixels; a
        non-zero ``median_dy`` is a constant offset, i.e. the two lenses have
        tilted relative to each other since calibration), ``median_disparity``
        and ``positive_fraction`` (disparities must be positive: negative ones
        mean swapped eyes or a wrong sign in the calibration), and ``patches``.
        ``median_abs_dy`` includes the matching noise of the frame it was
        measured on: use a sharp, still frame with texture at working range.
        """
        left, right = self.rectify(_gray(left_raw_rgb), _gray(right_raw_rgb))
        width, height = self.image_size
        half = patch // 2
        lowest, count = self.disparity_limits()
        reach = lowest + count
        # Smaller eigenvalue of the gradient structure tensor: large only
        # where the patch has edges in two different directions.
        gx = cv2.Sobel(left, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(left, cv2.CV_32F, 0, 1, ksize=3)
        box = (patch, patch)
        a, b, c = cv2.blur(gx * gx, box), cv2.blur(gy * gy, box), cv2.blur(gx * gy, box)
        texture = (a + b) / 2 - np.sqrt(((a - b) / 2) ** 2 + c ** 2)
        brightest = cv2.dilate(left, np.ones(box, np.uint8))
        centres = [(x, y)
                   for y in range(half + max_dy, height - half - max_dy, step)
                   for x in range(half, width - half - max_dy, step)
                   if texture[y, x] >= min_texture and brightest[y, x] <= 250]
        centres = centres[::max(1, int(math.ceil(len(centres) / max_patches)))]
        shifts = []
        for x, y in centres:
            start = max(0, x - half - reach)
            search = right[y - half - max_dy:y + half + max_dy + 1,
                           start:x + half + max_dy + 1]
            template = left[y - half:y + half + 1, x - half:x + half + 1]
            score = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
            _, best, _, (column, row) = cv2.minMaxLoc(score)
            if best < min_score or not (0 < column < score.shape[1] - 1
                                        and 0 < row < score.shape[0] - 1):
                continue
            row_exact = row + _peak_offset(score[row - 1:row + 2, column])
            column_exact = column + _peak_offset(score[row, column - 1:column + 2])
            shifts.append(((x - half) - (start + column_exact), max_dy - row_exact))
        if len(shifts) < 10:
            raise PerceptionError(f'only {len(shifts)} matched patches: the images have too '
                                  'little texture (or are not a stereo pair) to judge '
                                  'the rectification')
        shifts = np.array(shifts)
        return {'median_abs_dy': float(np.median(np.abs(shifts[:, 1]))),
                'median_dy': float(np.median(shifts[:, 1])),
                'median_disparity': float(np.median(shifts[:, 0])),
                'positive_fraction': float((shifts[:, 0] > 0).mean()),
                'patches': int(len(shifts))}


def object_height(points_cam, cam_to_base, plane_point, plane_normal, percentile=90,
                  min_points=30, max_height_m=0.3, table_points_cam=None):
    """Return (height of the object's top above the table in metres, points used).

    ``points_cam`` is Nx3 from :meth:`StereoRig.points_for_pixels` for the
    object's mask pixels (NaN rows allowed), ``cam_to_base`` the 4x4 pose of
    the left camera in the robot base frame, ``plane_point``/``plane_normal``
    the table plane in the base frame with the normal pointing up.

    Every point is moved into the base frame and its signed distance to the
    plane taken along the normal. Points below the table or more than
    ``max_height_m`` above it cannot belong to something lying on the table
    and are dropped; fewer than ``min_points`` survivors raises
    :class:`~ur7e_perception.core.PerceptionError` rather than guessing.

    Why a percentile and not the mean or the maximum: a mask seen from the
    side covers the object's top *and* the sides facing the camera, plus a
    rim of table. The sides hold every height from zero up to the top, so
    the mean lands somewhere in the middle. The top is the highest thing
    there is, but the single highest point is whatever mismatch happened to
    stick out. The 90th percentile sits inside the top surface as long as the
    top supplies more than a tenth of the points, and tolerates up to a tenth
    of them being wild. Two consequences worth knowing:

    * it reads slightly high (by under one noise sigma), because it is an
      upper quantile of a noisy surface;
    * if the top is blank (no texture, no disparities) and only the textured
      sides were matched, it reads low. Edges and print on real objects
      usually save this; a featureless glossy lid will not be measured.

    The plane normally comes from the board calibration, so the result also
    carries any disagreement between "where stereo says things are" and
    "where the board says the table is" (one pixel of disparity is 5 mm at
    0.8 m). ``table_points_cam`` removes that: pass stereo points of bare
    table next to the object (or of the printed board, which is richly
    textured) and their median distance from the plane is subtracted first,
    making the height the difference of two stereo measurements taken side by
    side, in which a common error cancels.
    """
    heights = _heights_above(points_cam, cam_to_base, plane_point, plane_normal)
    if not len(heights):
        raise PerceptionError('no stereo depth on the object (blank surface, hidden from '
                              'the right lens, or outside the depth range)')
    if table_points_cam is not None:
        table = _heights_above(table_points_cam, cam_to_base, plane_point, plane_normal)
        if len(table) < min_points:
            raise PerceptionError(f'only {len(table)} stereo points on the table reference; '
                                  f'need {min_points} (a plain table has too little texture)')
        heights = heights - np.median(table)
    kept = heights[(heights >= 0.0) & (heights <= max_height_m)]
    if len(kept) < min_points:
        raise PerceptionError(
            f'only {len(kept)} stereo points lie between the table and '
            f'{max_height_m * 1000:.0f} mm above it; need {min_points} '
            f'({int((heights < 0).sum())} are below the table, '
            f'{int((heights > max_height_m).sum())} too high)')
    return float(np.percentile(kept, percentile)), int(len(kept))


def _heights_above(points_cam, cam_to_base, plane_point, plane_normal):
    """Return the signed distances (m) of the finite camera-frame points from the plane."""
    points = np.asarray(points_cam, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    cam_to_base = np.asarray(cam_to_base, dtype=float)
    normal = np.asarray(plane_normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    in_base = points @ cam_to_base[:3, :3].T + cam_to_base[:3, 3]
    return (in_base - np.asarray(plane_point, dtype=float)) @ normal


def _peak_offset(three):
    """Return where a maximum really is, from the score at it and its two neighbours.

    The true peak lies between samples; a parabola through the three values
    puts it within +-0.5 of the middle one.
    """
    curvature = three[0] - 2 * three[1] + three[2]
    return 0.5 * (three[0] - three[2]) / curvature if curvature < 0 else 0.0


def _gray(image):
    """Return the brightness image (matching ignores colour); grey input passes through."""
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if image.ndim == 3 else image
