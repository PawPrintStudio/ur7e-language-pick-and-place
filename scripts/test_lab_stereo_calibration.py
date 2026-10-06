"""The stereo wave's motion rules, checked without a robot."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import lab_stereo_calibration as cal

JOINTS = cal.ARM_JOINTS + cal.WRIST_JOINTS


def angle_deg(a, b):
    """Rotation angle between two tool orientations given as xyz Euler degrees."""
    ra, rb = (Rotation.from_euler('xyz', turn, degrees=True) for turn in (a, b))
    return np.degrees((ra.inv() * rb).magnitude())


def test_every_wave_move_either_translates_or_turns_in_place():
    stops = cal.wave_stops([0.27, 0.34], [0.0, 0.08, 0.16], [0.32, 0.40])
    assert len(stops) == 24
    for (xyz_a, turn_a, _), (xyz_b, turn_b, kind) in zip(stops, stops[1:]):
        if kind == 'turn':
            assert xyz_a == xyz_b                     # rotation without travel
        else:
            assert turn_a == turn_b                   # travel without rotation
            assert np.linalg.norm(np.subtract(xyz_a, xyz_b)) <= 0.081


def test_orientation_steps_are_small_and_the_cycle_closes():
    steps = cal.ORIENTATIONS + cal.ORIENTATIONS[:1]
    for a, b in zip(steps, steps[1:]):
        assert angle_deg(a, b) <= 36.0
    # Yaw only: never a tilt, which swings the forearm (2026-10-06).
    turns = np.array(cal.ORIENTATIONS)
    assert not turns[:, :2].any() and np.ptp(turns[:, 2]) > 0


def joints(**changes):
    value = {name: 0.0 for name in JOINTS}
    value.update(changes)
    return value


def test_a_small_wrist_step_passes():
    arm, wrist = cal.check_move(joints(), [joints(wrist_3_joint=0.3), joints(wrist_3_joint=0.61)])
    assert arm == 0.0 and wrist == pytest.approx(0.61)


def test_a_swing_out_and_back_is_caught_mid_path():
    # Ends where it started, but the elbow went 0.8 rad out on the way.
    path = [joints(elbow_joint=0.4), joints(elbow_joint=0.8), joints()]
    with pytest.raises(ValueError, match='arm joints'):
        cal.check_move(joints(), path)


def test_a_hundred_degree_wrist_swing_is_refused():
    # The 2026-10-06 move that had to be stopped by hand swung the wrist this far.
    with pytest.raises(ValueError, match='wrist'):
        cal.check_move(joints(), [joints(wrist_3_joint=np.radians(100))])
