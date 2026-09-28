"""ChArUco observations and a permanent workspace calibration marker."""
import cv2
import numpy as np

from .core import PerceptionError, rigid_transform


def dictionary():
    """Use one explicit dictionary for printed boards and workspace markers."""
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def charuco_board(columns=5, rows=7, square_m=.04, marker_m=.03):
    """Build the metric gripper board (print at 100%, verify with a ruler)."""
    return cv2.aruco.CharucoBoard_create(columns, rows, square_m, marker_m, dictionary())


def charuco_pose(frame, board=None):
    """Return camera_from_board for a rectified RGB capture, without moving."""
    frame.validate()
    board = charuco_board() if board is None else board
    gray = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary())
    if ids is None:
        raise PerceptionError('ChArUco board not visible')
    count, points, corner_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board, cameraMatrix=frame.k, distCoeffs=np.zeros(5))
    if count < 6:
        raise PerceptionError('insufficient ChArUco corners')
    found, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        points, corner_ids, board, frame.k, np.zeros(5), None, None)
    if not found:
        raise PerceptionError('ChArUco pose could not be estimated')
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = tvec.ravel()
    return rigid_transform(transform)


def check_workspace_marker(frame, camera_to_base, base_from_marker,
                           marker_id=42, marker_size_m=.04):
    """Reject a missing marker or camera drift at each physical capture.

    Check orientation as well as position: rotating the camera about a marker
    center could leave that center unchanged while moving every grasp point.
    """
    frame.validate()
    expected = rigid_transform(base_from_marker)
    transform = rigid_transform(camera_to_base)
    if not np.isfinite(marker_size_m) or marker_size_m <= 0:
        raise PerceptionError('marker size must be positive metres')
    gray = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary())
    matches = [] if ids is None else np.flatnonzero(ids.ravel() == marker_id).tolist()
    if len(matches) != 1:
        raise PerceptionError('workspace calibration marker missing or ambiguous')
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        [corners[matches[0]]], marker_size_m, frame.k, np.zeros(5))
    observed = np.eye(4)
    observed[:3, :3] = cv2.Rodrigues(rvecs[0])[0]
    observed[:3, 3] = tvecs[0].ravel()
    actual = transform @ observed
    translation = float(np.linalg.norm(actual[:3, 3]-expected[:3, 3]))
    rotation = float(np.linalg.norm(cv2.Rodrigues(expected[:3, :3].T @ actual[:3, :3])[0]))
    if not np.isfinite([translation, rotation]).all() or translation >= .01 or rotation >= .05:
        raise PerceptionError('workspace marker indicates calibration drift')
    return translation, rotation


def main():
    """Print reference patterns or extract one synchronized board observation."""
    import argparse
    import json
    from pathlib import Path
    from .synthetic import load_frame
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    printing = commands.add_parser('print')
    printing.add_argument('directory')
    observe = commands.add_parser('observe')
    observe.add_argument('capture')
    observe.add_argument('robot_pose', help='JSON: stamp, base_from_gripper (4x4)')
    observe.add_argument('output')
    args = parser.parse_args()
    if args.command == 'print':
        directory = Path(args.directory)
        directory.mkdir(parents=True, exist_ok=False)
        cv2.imwrite(str(directory/'charuco_200x280mm.png'), charuco_board().draw((2000, 2800)))
        marker = np.full((600, 600), 255, np.uint8)
        marker[100:500, 100:500] = cv2.aruco.drawMarker(dictionary(), 42, 400)
        cv2.imwrite(str(directory/'marker42_black_square_40mm.png'), marker)
        print('Print board at 200 x 280 mm; marker black square at 40 mm. Verify with a ruler.')
        return
    frame = load_frame(args.capture)
    robot = json.loads(Path(args.robot_pose).read_text())
    if not np.isfinite(robot['stamp']) or abs(robot['stamp']-frame.stamp) > .04:
        raise PerceptionError('robot pose and camera observation are not synchronized')
    transform = rigid_transform(robot['base_from_gripper'])
    result = dict(stamp=frame.stamp, base_from_gripper=transform.tolist(),
                  camera_from_board=charuco_pose(frame).tolist())
    with open(args.output, 'x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(f'Saved synchronized ChArUco observation: {args.output}')


if __name__ == '__main__':
    main()
