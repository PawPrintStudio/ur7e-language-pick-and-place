"""Eye-to-hand recovery from independently composed frame chains."""
import cv2
import numpy as np
import pytest

from ur7e_perception.calibration import eye_to_hand
from ur7e_perception.core import PerceptionError
from ur7e_perception.synthetic import CAMERA_TO_BASE


def observations():
    rng = np.random.default_rng(15)
    attachment = np.eye(4)
    attachment[:3, 3] = [.02, -.01, .15]
    robot, board = [], []
    for _ in range(15):
        g = np.eye(4)
        g[:3, :3] = cv2.Rodrigues(rng.uniform(-.8, .8, 3))[0]
        g[:3, 3] = rng.uniform([.3, -.2, .2], [.6, .2, .4])
        robot.append(g)
        board.append(np.linalg.inv(CAMERA_TO_BASE) @ g @ attachment)
    return robot, board


def test_eye_to_hand_recovers_transform():
    robot, board = observations()
    result, error, rotation = eye_to_hand(robot, board)
    assert np.allclose(result, CAMERA_TO_BASE, atol=1e-8)
    assert error < 1e-8 and rotation < 1e-8


def test_bad_calibration_data_rejected():
    robot, board = observations()
    with pytest.raises(PerceptionError):
        eye_to_hand(robot[:3], board[:3])
    with pytest.raises(PerceptionError):
        eye_to_hand([robot[0]]*15, board)
    board[3][0, 3] += .1
    with pytest.raises(PerceptionError):
        eye_to_hand(robot, board)
