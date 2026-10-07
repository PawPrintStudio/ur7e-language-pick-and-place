"""Where a fixed stereo camera is, from a target the robot carries.

The question
------------
A pick needs object positions in the robot's frame (``base_link``). A stereo
camera measures them in its *own* frame, and that frame moves every time the
camera is bumped or re-aimed. What connects the two is one rigid transform,
``cam_to_base``, and finding it is "eye-to-hand calibration".

The robot already knows, from its joint encoders and its own dimensions,
exactly where its flange (``tool0``) is: pose ``(R_i, t_i)`` at every stop
``i``. If the gripper holds a small target, the target's centre sits at a
fixed but unmeasured offset ``x`` in the flange frame, so in the base frame it
is at ``R_i @ x + t_i``. The stereo camera sees the same centre at ``p_i`` in
its own frame. One pose gives three equations::

    R_cb @ p_i + t_cb  =  R_i @ x + t_i

with nine unknowns: the camera's rotation and position (six) and the target
offset (three). A handful of poses over-determine them. The offset only
shows up if the flange *turns* between poses -- with a constant orientation
``R_i @ x`` is a constant that merges into ``t_cb`` -- so the wave tilts and
yaws the wrist, and :func:`solve` reports how well each unknown was pinned.

A tenth unknown, optional: stereo depth comes from disparity, and a small
constant error in disparity (a lens toe-in the factory file no longer
describes) scales every distance along its ray. The robot is a ruler here:
its kinematics are exact to a fraction of a millimetre, so the same
equations measure that disparity offset too.

How it is solved
----------------
1. **Linear start.** Treat the camera rotation as an arbitrary 3x3 matrix
   ``M``: then the equations are linear in (``M``, ``t_cb``, ``x``), fifteen
   unknowns, and ordinary least squares solves them with no initial guess.
   The nearest true rotation to ``M`` (by SVD) is the start.
2. **Robust refinement.** Nonlinear least squares over the real parameters
   (rotation vector, ``t_cb``, ``x``, and the disparity offset) with a
   soft-L1 loss, so one bad detection cannot drag the answer; detections
   that still disagree by more than a few times the typical residual are
   dropped and the solve repeated.

No ROS here: pure geometry, unit-tested on synthetic poses
(``test/test_handeye.py``).
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .core import PerceptionError


def correct_depth(points_cam, disparity_offset_px, focal_px, baseline_m):
    """Rescale camera-frame points for a constant disparity error.

    Measured disparity ``d = f*B/Z``; if the true one is ``d - offset``, the
    true depth is ``f*B / (f*B/Z - offset)``. The point stays on its ray, so
    the whole vector scales by the same factor.
    """
    points = np.asarray(points_cam, dtype=float).reshape(-1, 3)
    if not disparity_offset_px:
        return points.copy()
    fb = focal_px * baseline_m
    depth = points[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        scale = (fb / (fb / depth - disparity_offset_px)) / depth
    return points * scale[:, None]


def _linear_start(points_cam, rotations, translations, target_drop=None):
    """Return (R_cb, t_cb, x) from the relaxed linear problem.

    With ``target_drop`` the target's tool-z coordinate is known and moves
    to the right-hand side; only its x and y are unknowns.
    """
    free = 3 if target_drop is None else 2
    rows, rhs = [], []
    for p, rot, t in zip(points_cam, rotations, translations):
        for j in range(3):
            row = np.zeros(12 + free)
            row[3 * j:3 * j + 3] = p          # M[j, :] @ p
            row[9 + j] = 1.0                  # + t_cb[j]
            row[12:] = -rot[j, :free]         # - R_i[j, :] @ x
            rows.append(row)
            rhs.append(t[j] + (0.0 if target_drop is None else rot[j, 2] * target_drop))
    solution, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
    u, _, vt = np.linalg.svd(solution[:9].reshape(3, 3))
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:      # a reflection is not a camera pose
        rotation = u @ np.diag([1, 1, -1]) @ vt
    x = solution[12:] if target_drop is None else np.append(solution[12:], target_drop)
    return rotation, solution[9:12], x


def solve(points_cam, tool_poses, focal_px=None, baseline_m=None, fit_disparity=False,
          loss_scale_m=0.003, outlier_factor=4.0, min_poses=6, target_drop=None,
          target_on_axis=False):
    """Solve the camera pose from stereo target positions and flange poses.

    ``target_drop`` (m), if given, fixes how far along tool0's z axis the
    target sits (measured with a ruler). Then the wrist only needs to *yaw*
    between stops -- spin about its own vertical axis, which moves nothing
    but the last joint -- instead of tilting.

    ``points_cam``: Nx3, target centre in the left camera's optical frame (m).
    ``tool_poses``: N 4x4 ``base_link -> tool0`` transforms at the same stops.
    ``fit_disparity`` also estimates a constant disparity offset (needs
    ``focal_px`` and ``baseline_m`` of the rectified rig).

    Returns a dict: ``cam_to_base`` (4x4), ``target_in_tool`` (3,),
    ``disparity_offset_px``, ``residuals_mm`` (per pose, NaN for dropped),
    ``inliers`` (bool per pose), ``rms_mm`` over inliers, and ``sigma`` --
    the 1-sigma uncertainty of the target offset (mm, per axis) and of the
    disparity offset, from the fit's Jacobian. A large ``sigma`` means the
    poses did not turn the wrist enough to separate that unknown.

    ``target_on_axis`` (needs ``target_drop``) also pins the target on tool0's
    axis, so only the camera pose is fitted: for a handful of stops, which
    cannot separate a sideways target offset from the camera pose.
    """
    points = np.asarray(points_cam, dtype=float).reshape(-1, 3)
    poses = np.asarray(tool_poses, dtype=float).reshape(-1, 4, 4)
    if len(points) != len(poses):
        raise PerceptionError('need one flange pose per target observation')
    if fit_disparity and not (focal_px and baseline_m):
        raise PerceptionError('fit_disparity needs focal_px and baseline_m')
    finite = np.isfinite(points).all(axis=1)
    inliers = finite.copy()
    if inliers.sum() < min_poses:
        raise PerceptionError(f'only {int(inliers.sum())} usable observations; need {min_poses}')
    rotations, translations = poses[:, :3, :3], poses[:, :3, 3]

    if target_on_axis and target_drop is None:
        raise PerceptionError('target_on_axis needs target_drop')
    free = 0 if target_on_axis else 3 if target_drop is None else 2

    def unpack(params):
        if target_on_axis:
            x = np.array([0.0, 0.0, target_drop])
        elif target_drop is None:
            x = params[6:9]
        else:
            x = np.append(params[6:8], target_drop)
        offset = params[6 + free] if fit_disparity else 0.0
        return Rotation.from_rotvec(params[:3]).as_matrix(), params[3:6], x, offset

    def residual(params, keep):
        rot_cb, t_cb, x, offset = unpack(params)
        p = correct_depth(points[keep], offset, focal_px, baseline_m) if offset else points[keep]
        predicted = np.einsum('nij,j->ni', rotations[keep], x) + translations[keep]
        return ((p @ rot_cb.T + t_cb) - predicted).ravel()

    for _ in range(4):
        rot0, t0, x0 = _linear_start(points[inliers], rotations[inliers],
                                     translations[inliers], target_drop)
        start = np.concatenate([Rotation.from_matrix(rot0).as_rotvec(), t0, x0[:free]]
                               + ([[0.0]] if fit_disparity else []))
        fit = least_squares(residual, start, args=(inliers,), loss='soft_l1',
                            f_scale=loss_scale_m)
        errors = np.full(len(points), np.nan)
        errors[finite] = np.linalg.norm(
            residual(fit.x, finite).reshape(-1, 3), axis=1)
        typical = max(np.median(errors[inliers]), 0.5 * loss_scale_m)
        keep = finite & (errors <= outlier_factor * typical)
        if keep.sum() < min_poses:
            break
        if np.array_equal(keep, inliers):
            break
        inliers = keep

    rot_cb, t_cb, x, offset = unpack(fit.x)
    cam_to_base = np.eye(4)
    cam_to_base[:3, :3], cam_to_base[:3, 3] = rot_cb, t_cb
    # Parameter covariance from the Jacobian at the solution, scaled by the
    # residual variance: the textbook Gauss-Newton estimate.
    jac = fit.jac
    dof = max(1, jac.shape[0] - jac.shape[1])
    variance = float(np.sum(residual(fit.x, inliers) ** 2) / dof)
    try:
        covariance = np.linalg.inv(jac.T @ jac) * variance
        spread = np.sqrt(np.clip(np.diag(covariance), 0, None))
    except np.linalg.LinAlgError:
        spread = np.full(jac.shape[1], np.inf)
    residuals = np.where(inliers, errors, np.nan) * 1000.0
    return {'cam_to_base': cam_to_base, 'target_in_tool': x,
            'disparity_offset_px': float(offset), 'inliers': inliers,
            'residuals_mm': residuals,
            'rms_mm': float(np.sqrt(np.nanmean(residuals[inliers] ** 2))),
            'sigma': {'target_in_tool_mm': (np.append(spread[6:6 + free], [0.0] * (3 - free))
                                            * 1000.0).tolist(),
                      'disparity_offset_px': float(spread[6 + free]) if fit_disparity else 0.0}}


def sphere_centre(surface_point, ray_pixel_dir, radius_m):
    """Move a measured front-surface point of a ball back to the ball's centre.

    Stereo sees the side of the target that faces the camera, so the median
    of its points lies about one radius in front of the centre, along the
    viewing ray. Left uncorrected, that constant offset in the camera frame
    would end up inside the camera position.
    """
    direction = np.asarray(ray_pixel_dir, dtype=float)
    direction = direction / np.linalg.norm(direction)
    return np.asarray(surface_point, dtype=float) + radius_m * direction
