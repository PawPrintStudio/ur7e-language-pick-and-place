"""Use rendered printable fiducials to exercise actual OpenCV detection."""
import cv2
import numpy as np
import pytest

from ur7e_perception.core import Frame, PerceptionError
from ur7e_perception.fiducials import dictionary, charuco_board, charuco_pose, check_workspace_marker
from ur7e_perception.synthetic import CAMERA_TO_BASE


def frame(gray):
    h, w = gray.shape
    k = np.array([[300., 0, (w-1)/2], [0, 300., (h-1)/2], [0, 0, 1]])
    return Frame(cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB), np.ones((h, w), np.float32), k, 0.)


def test_printed_workspace_marker_and_drift():
    gray = np.full((240, 320), 255, np.uint8)
    gray[70:170, 110:210] = cv2.aruco.drawMarker(dictionary(), 42, 100)
    expected = np.eye(4)
    expected[:3, 3] = [.45, 0, .7]
    error, angle = check_workspace_marker(frame(gray), CAMERA_TO_BASE, expected, marker_size_m=.1)
    assert error < .005 and angle < .01
    bumped = CAMERA_TO_BASE.copy()
    bumped[0, 3] += .02
    with pytest.raises(PerceptionError, match='drift'):
        check_workspace_marker(frame(gray), bumped, expected, marker_size_m=.1)
    with pytest.raises(PerceptionError, match='missing'):
        check_workspace_marker(frame(np.full_like(gray, 255)), CAMERA_TO_BASE, expected)


def test_printed_charuco_pose():
    gray = np.full((360, 400), 255, np.uint8)
    gray[40:320, 100:300] = charuco_board().draw((200, 280))
    pose = charuco_pose(frame(gray))
    assert abs(pose[2, 3]-.3) < .005
    with pytest.raises(PerceptionError):
        charuco_pose(frame(np.full_like(gray, 255)))
