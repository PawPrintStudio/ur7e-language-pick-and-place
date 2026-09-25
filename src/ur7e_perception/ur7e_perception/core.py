"""Registered RGB-D geometry; no ROS, GPU, or camera dependency."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


class PerceptionError(ValueError):
    """A capture or target is unsafe or insufficient for localization."""


@dataclass
class Frame:
    rgb: np.ndarray
    depth: np.ndarray  # optical z in metres, registered to rectified RGB
    k: np.ndarray
    stamp: float
    frame_id: str = 'camera_optical_frame'

    def validate(self):
        """Require rectified, registered images and finite pinhole intrinsics."""
        if self.rgb.dtype != np.uint8 or self.rgb.ndim != 3 or self.rgb.shape[2] != 3:
            raise PerceptionError('RGB must be HxWx3 uint8')
        if self.depth.shape != self.rgb.shape[:2]:
            raise PerceptionError('depth must be registered to RGB dimensions')
        if not np.issubdtype(self.depth.dtype, np.floating):
            raise PerceptionError('depth must be floating point metres')
        if (self.k.shape != (3, 3) or not np.isfinite(self.k).all()
                or self.k[0, 0] <= 0 or self.k[1, 1] <= 0
                or not np.allclose(self.k[2], [0, 0, 1])
                or abs(self.k[0, 1]) > 1e-8 or abs(self.k[1, 0]) > 1e-8):
            raise PerceptionError('invalid pinhole intrinsics')
        if not self.frame_id or not math.isfinite(self.stamp):
            raise PerceptionError('missing frame or invalid timestamp')


@dataclass
class Detection:
    mask: np.ndarray
    bbox: tuple  # x0,y0,x1,y1, exclusive upper bounds
    confidence: float
    label: str


@dataclass
class Grasp:
    position: np.ndarray
    quaternion: np.ndarray  # xyzw, tool z down
    yaw: float
    points: int
    axis_ratio: float


def rigid_transform(value):
    """Reject reflections, scaled transforms, and non-finite calibration."""
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)):
        raise PerceptionError('invalid camera-to-base rigid transform')
    return t


def check_fresh(frame, now, max_age=2.0):
    """Use the same clock for capture and evaluation, including rosbag /clock."""
    if not math.isfinite(now) or not 0 <= now - frame.stamp <= max_age:
        raise PerceptionError('capture is stale or from a different clock')


def localize(frame, detection, camera_to_base, polygon, table_z=0.0,
             max_z=0.35, min_points=20, max_depth=3.0):
    """Project an instance mask, trim depth outliers, and fit its XY axis.

    The returned point lies on the observed top surface. Finger insertion
    depth and approach clearance belong to the motion policy, not perception.
    Near-isotropic objects use a deterministic yaw of zero.
    """
    frame.validate()
    t = rigid_transform(camera_to_base)
    if (not np.isfinite([table_z, max_z, max_depth]).all()
            or table_z >= max_z or max_depth <= .05 or min_points < 3):
        raise PerceptionError('invalid localization bounds')
    mask = np.asarray(detection.mask)
    polygon = np.asarray(polygon, dtype=np.float32)
    if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
            or not np.isfinite(polygon).all() or cv2.contourArea(polygon) <= 1e-8):
        raise PerceptionError('invalid workspace polygon')
    if mask.shape != frame.depth.shape or not np.isin(mask, [0, 1, 255]).all():
        raise PerceptionError('invalid instance mask')
    if not 0 <= detection.confidence <= 1:
        raise PerceptionError('invalid detection confidence')
    selected = mask.astype(bool)
    valid = selected & np.isfinite(frame.depth) & (frame.depth > 0.05)
    valid &= frame.depth < max_depth
    if valid.sum() < min_points or valid.sum() < 0.5 * selected.sum():
        raise PerceptionError('insufficient valid depth in mask')
    v, u = np.nonzero(valid)
    z = frame.depth[v, u].astype(float)
    median = np.median(z)
    mad = np.median(np.abs(z - median))
    keep = np.abs(z - median) <= max(0.005, 4.5 * mad)
    u, v, z = u[keep], v[keep], z[keep]
    if len(z) < min_points:
        raise PerceptionError('insufficient inlier depth')
    k = frame.k
    xyz = np.column_stack(((u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z))
    xyz = xyz @ t[:3, :3].T + t[:3, 3]
    center = np.median(xyz, axis=0)
    # Check the whole visible target, not just its center.
    if any(cv2.pointPolygonTest(polygon, tuple(map(float, p)), False) < 0
           for p in xyz[:, :2]):
        raise PerceptionError('object extends outside workspace')
    if center[2] < table_z or center[2] > max_z:
        raise PerceptionError('target below table or above workspace')
    values, vectors = np.linalg.eigh(np.cov(xyz[:, :2].T))
    ratio = float(values[-1] / max(values[0], 1e-12))
    axis = vectors[:, -1]
    yaw = (math.atan2(axis[1], axis[0]) + math.pi/2) % math.pi - math.pi/2
    if ratio < 1.2:
        yaw = 0.0
    # Rz(yaw) Rx(pi): z down, x along the object's principal axis.
    q = np.array([math.cos(yaw/2), math.sin(yaw/2), 0.0, 0.0])
    return Grasp(center, q, yaw, len(z), ratio)


def fit_calibration(camera_points, base_points):
    """Fit camera-to-base from paired 3-D fiducials (at least 4, non-collinear)."""
    a, b = np.asarray(camera_points, float), np.asarray(base_points, float)
    if (a.shape != b.shape or a.ndim != 2 or a.shape[1] != 3 or len(a) < 4
            or not np.isfinite(a).all() or not np.isfinite(b).all()):
        raise PerceptionError('need at least four finite paired 3-D points')
    ac, bc = a-a.mean(0), b-b.mean(0)
    if np.linalg.matrix_rank(ac, tol=1e-5) < 2 or np.linalg.matrix_rank(bc, tol=1e-5) < 2:
        raise PerceptionError('degenerate calibration points')
    u, _, vt = np.linalg.svd(ac.T @ bc)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, b.mean(0) - r @ a.mean(0)
    residual = np.linalg.norm(a @ r.T + t[:3, 3] - b, axis=1)
    if residual.max() >= 0.01:
        raise PerceptionError('calibration residual exceeds 1 cm')
    return rigid_transform(t), residual


def verify_marker(camera_points, expected_base_points, transform, tolerance=0.01):
    """Independent held-out marker/touch checks; every point must pass."""
    t = rigid_transform(transform)
    a, b = np.asarray(camera_points, float), np.asarray(expected_base_points, float)
    if a.shape != b.shape or a.ndim != 2 or a.shape[1] != 3 or len(a) < 5:
        raise PerceptionError('verification requires at least five paired observations')
    errors = np.linalg.norm(a @ t[:3, :3].T + t[:3, 3] - b, axis=1)
    if not np.isfinite(errors).all() or np.any(errors >= tolerance):
        raise PerceptionError('calibration verification failed; recalibrate before picking')
    return errors
