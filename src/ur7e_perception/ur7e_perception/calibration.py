"""Eye-to-hand calibration from robot poses and observed ChArUco board poses."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .core import PerceptionError, rigid_transform, verify_marker


def eye_to_hand(base_from_gripper, camera_from_board):
    """Solve the fixed camera pose; the ChArUco board is rigid on the gripper.

    OpenCV's eye-in-hand equation is reused by inverting each robot pose.
    The output is base_from_camera. Require rotational excitation about
    multiple axes; pure translation/single-axis data are underconstrained.
    """
    if len(base_from_gripper) != len(camera_from_board) or len(base_from_gripper) < 6:
        raise PerceptionError('need at least six paired robot/board poses')
    robot = [rigid_transform(t) for t in base_from_gripper]
    board = [rigid_transform(t) for t in camera_from_board]
    excitation = np.array([cv2.Rodrigues(robot[0][:3, :3].T @ p[:3, :3])[0].ravel() for p in robot[1:]])
    if np.linalg.svd(excitation, compute_uv=False)[1] < .1:
        raise PerceptionError('calibration needs rotation about at least two axes')
    inverse = [np.linalg.inv(t) for t in robot]
    r, p = cv2.calibrateHandEye(
        [t[:3, :3] for t in inverse], [t[:3, 3] for t in inverse],
        [t[:3, :3] for t in board], [t[:3, 3] for t in board],
        method=cv2.CALIB_HAND_EYE_PARK)
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = r, p.ravel()
    rigid_transform(result)
    attachments = [np.linalg.inv(g) @ result @ b for g, b in zip(robot, board)]
    reference = attachments[0]
    translation_error = max(np.linalg.norm(t[:3, 3]-reference[:3, 3]) for t in attachments)
    rotation_error = max(np.linalg.norm(cv2.Rodrigues(reference[:3, :3].T @ t[:3, :3])[0]) for t in attachments)
    if translation_error >= .01 or rotation_error >= .05:
        raise PerceptionError('inconsistent board attachment or calibration observations')
    return result, translation_error, rotation_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('observations', help='JSON paired 4x4 poses and independent marker checks')
    parser.add_argument('output', help='New JSON verification artifact; refuses overwrite')
    args = parser.parse_args()
    data = json.loads(Path(args.observations).read_text())
    t, translation, rotation = eye_to_hand(data['base_from_gripper'], data['camera_from_board'])
    checks = data['verification']
    errors = verify_marker(checks['camera_points'], checks['base_points'], t)
    result = dict(source=data['source'], camera_to_base=t.tolist(),
                  camera_points=checks['camera_points'], base_points=checks['base_points'],
                  fit_translation_m=translation, fit_rotation_rad=rotation,
                  verification_errors_m=errors.tolist())
    with open(args.output, 'x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(f'Calibration and {len(errors)} independent checks passed; max error {errors.max():.6f} m')


if __name__ == '__main__':
    main()
