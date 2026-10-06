"""Eye-to-hand calibration from a carried target, on synthetic robot poses."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from ur7e_perception.core import PerceptionError
from ur7e_perception.handeye import correct_depth, solve, sphere_centre
from ur7e_perception.synthetic_table import look_at

FOCAL, BASELINE = 1079.0, 0.1199
TARGET = np.array([0.012, -0.008, 0.31])          # held centre in the tool0 frame
CAMERA = look_at([0.75, 0.45, 0.35], [0.24, 0.08, 0.05])  # about 0.8 m away, looking down


def flange_poses(turn=True, yaw_only=False):
    """Top-down flange poses over the plate; with ``turn`` the wrist yaws and tilts."""
    down = Rotation.from_euler('x', 180, degrees=True)
    turns = [(0, 0, 0), (0, 0, 60), (0, 0, -60), (25, 0, 0), (-25, 0, 0), (0, 25, 0),
             (0, -25, 0), (15, 15, 45)]
    if yaw_only:
        turns = [(0, 0, 0), (0, 0, 35), (0, 0, 0), (0, 0, -35)]
    poses = []
    index = 0
    for z in (0.27, 0.36, 0.45):
        for x in (0.19, 0.24, 0.29):
            for y in (-0.02, 0.08, 0.18):
                pose = np.eye(4)
                extra = turns[index % len(turns)] if turn else (0, 0, 0)
                pose[:3, :3] = (down * Rotation.from_euler('xyz', extra, degrees=True)
                                ).as_matrix()
                pose[:3, 3] = (x, y, z)
                poses.append(pose)
                index += 1
    return np.array(poses)


def observe(poses, cam_to_base=CAMERA, noise_m=0.0, seed=0):
    """Return the target centre in the camera frame at each pose."""
    rng = np.random.default_rng(seed)
    in_base = np.einsum('nij,j->ni', poses[:, :3, :3], TARGET) + poses[:, :3, 3]
    base_to_cam = np.linalg.inv(cam_to_base)
    points = in_base @ base_to_cam[:3, :3].T + base_to_cam[:3, 3]
    return points + rng.normal(0, noise_m, points.shape)


def pose_error(found, true):
    """Return (position error m, rotation error deg) between two 4x4 poses."""
    delta = np.linalg.inv(true) @ found
    angle = np.degrees(np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()))
    return float(np.linalg.norm(delta[:3, 3])), float(angle)


def test_exact_observations_give_the_exact_camera_and_target():
    poses = flange_poses()
    result = solve(observe(poses), poses)
    position, angle = pose_error(result['cam_to_base'], CAMERA)
    assert position < 1e-6 and angle < 1e-5
    assert np.allclose(result['target_in_tool'], TARGET, atol=1e-6)
    assert result['inliers'].all() and result['rms_mm'] < 1e-3


def test_millimetre_noise_gives_millimetre_answers():
    poses = flange_poses()
    result = solve(observe(poses, noise_m=0.001, seed=3), poses)
    position, angle = pose_error(result['cam_to_base'], CAMERA)
    assert position < 0.003 and angle < 0.3
    assert np.linalg.norm(result['target_in_tool'] - TARGET) < 0.004
    assert 0.5 < result['rms_mm'] < 2.5
    assert max(result['sigma']['target_in_tool_mm']) < 3.0


def test_wrong_detections_are_dropped_not_averaged_in():
    poses = flange_poses()
    points = observe(poses, noise_m=0.0005, seed=4)
    points[5] += (0.06, 0.0, 0.0)       # the detector locked onto something else
    points[17] += (0.0, -0.04, 0.03)
    result = solve(points, poses)
    assert not result['inliers'][5] and not result['inliers'][17]
    assert result['inliers'].sum() == len(poses) - 2
    position, angle = pose_error(result['cam_to_base'], CAMERA)
    assert position < 0.002 and angle < 0.2


def test_missing_detections_are_skipped():
    poses = flange_poses()
    points = observe(poses)
    points[[2, 9, 20]] = np.nan
    result = solve(points, poses)
    assert result['inliers'].sum() == len(poses) - 3
    assert np.isnan(result['residuals_mm'][[2, 9, 20]]).all()
    assert pose_error(result['cam_to_base'], CAMERA)[0] < 1e-6


def test_a_disparity_offset_is_measured_with_the_robot_as_ruler():
    poses = flange_poses()
    true_points = observe(poses, noise_m=0.0003, seed=5)
    # The matcher reports every disparity 1.5 px too large: things look nearer.
    measured = correct_depth(true_points, -1.5, FOCAL, BASELINE)
    plain = solve(measured, poses)
    fitted = solve(measured, poses, focal_px=FOCAL, baseline_m=BASELINE, fit_disparity=True)
    assert abs(fitted['disparity_offset_px'] - 1.5) < 0.2
    assert fitted['rms_mm'] < plain['rms_mm']
    assert pose_error(fitted['cam_to_base'], CAMERA)[0] < 0.003


def test_a_wrist_that_never_turns_cannot_reveal_the_target_offset():
    poses = flange_poses(turn=False)
    result = solve(observe(poses, noise_m=0.0005, seed=6), poses)
    # Points still line up (the offset hides inside the camera position) ...
    assert result['rms_mm'] < 1.5
    # ... but the solver says it could not tell where the target sits.
    assert max(result['sigma']['target_in_tool_mm']) > 50.0


def test_too_few_observations_raise():
    poses = flange_poses()[:5]
    with pytest.raises(PerceptionError):
        solve(observe(poses), poses)
    with pytest.raises(PerceptionError):
        solve(observe(poses), poses[:4])


def test_disparity_correction_round_trips_and_keeps_rays():
    points = np.array([[0.1, -0.05, 0.8], [-0.2, 0.1, 1.2]])
    there = correct_depth(points, 2.0, FOCAL, BASELINE)
    back = correct_depth(there, -2.0, FOCAL, BASELINE)
    assert np.allclose(back, points)
    assert np.allclose(np.cross(there, points), 0, atol=1e-12)   # same ray
    assert (there[:, 2] > points[:, 2]).all()    # less disparity -> farther


def test_sphere_centre_is_one_radius_behind_the_surface():
    centre = sphere_centre([0.0, 0.0, 0.78], [0.0, 0.0, 2.0], 0.02)
    assert np.allclose(centre, [0.0, 0.0, 0.80])


def test_yaw_alone_needs_the_measured_drop_and_then_suffices():
    # 2026-10-06: tilting the wrist on the real arm was stopped twice by hand;
    # spinning wrist 3 in place is the safe motion. Yaw cannot reveal how far
    # down the tool axis the target hangs -- but a ruler can.
    poses = flange_poses(yaw_only=True)
    points = observe(poses, noise_m=0.0005, seed=8)
    blind = solve(points, poses)
    assert max(blind['sigma']['target_in_tool_mm']) > 50.0
    told = solve(points, poses, target_drop=TARGET[2])
    position, angle = pose_error(told['cam_to_base'], CAMERA)
    assert position < 0.002 and angle < 0.2
    assert np.linalg.norm(told['target_in_tool'][:2] - TARGET[:2]) < 0.002
    assert told['target_in_tool'][2] == TARGET[2]
    assert told['sigma']['target_in_tool_mm'][2] == 0.0
    # A wrong ruler reading moves the camera by about the same amount.
    off = solve(points, poses, target_drop=TARGET[2] + 0.01)
    assert 0.007 < pose_error(off['cam_to_base'], CAMERA)[0] < 0.013


def test_a_known_drop_still_works_with_the_disparity_correction():
    poses = flange_poses(yaw_only=True)
    measured = correct_depth(observe(poses, noise_m=0.0003, seed=9), -1.0, FOCAL, BASELINE)
    result = solve(measured, poses, focal_px=FOCAL, baseline_m=BASELINE, fit_disparity=True,
                   target_drop=TARGET[2])
    assert abs(result['disparity_offset_px'] - 1.0) < 0.3
    assert pose_error(result['cam_to_base'], CAMERA)[0] < 0.004
