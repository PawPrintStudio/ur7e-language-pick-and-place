"""Plain USB (UVC) camera access, including a ZED used without its SDK.

A ZED 2i plugged into a machine without the Stereolabs SDK (which needs an
NVIDIA GPU) is still an ordinary USB camera: it delivers both lenses side by
side in one wide frame. Taking the left half gives a good 1920x1080 colour
camera. What the SDK would normally add -- lens undistortion -- is done here
from the factory calibration file Stereolabs publishes per serial number::

    https://calib.stereolabs.com/?SN=<serial>
"""
import configparser

import cv2
import numpy as np

from .core import PerceptionError

# Per-eye image size for each ZED resolution name used in the calibration file.
ZED_MODES = {'2K': (2208, 1242), 'FHD': (1920, 1080), 'HD': (1280, 720), 'VGA': (672, 376)}


class Camera:
    """One V4L2 camera; ``left_half`` crops a side-by-side stereo frame."""

    def __init__(self, device=0, width=1280, height=720, fourcc='MJPG', left_half=False):
        self.left_half = left_half
        self.capture = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width * (2 if left_half else 1))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.capture.isOpened():
            raise PerceptionError(f'cannot open camera {device} (already in use?)')

    def read(self):
        """Return the next frame as RGB, or None if the camera gave nothing."""
        pair = self.read_pair()
        return None if pair is None else pair[0]

    def read_pair(self):
        """Return (left RGB, right RGB or None) from one exposure.

        Both halves of a side-by-side frame are captured at the same instant,
        which is what makes them usable as a stereo pair.
        """
        ok, frame = self.capture.read()
        if not ok:
            return None
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if not self.left_half:
            return rgb, None
        half = rgb.shape[1] // 2
        return rgb[:, :half], rgb[:, half:]

    def release(self):
        """Close the device so another process can open it."""
        self.capture.release()


def zed_factory_calibration(path, mode='FHD', side='LEFT'):
    """Read one eye's intrinsics from a Stereolabs ``SN<serial>.conf`` file.

    Returns (camera_matrix 3x3, dist coefficients, (width, height)). The
    eight-coefficient rational model in ``[LEFT_DISTO]`` is used when the
    file has it, otherwise the five-coefficient one in the per-mode section.
    """
    if mode not in ZED_MODES:
        raise PerceptionError(f'unknown ZED mode {mode}; use one of {sorted(ZED_MODES)}')
    parser = configparser.ConfigParser()
    if not parser.read(path):
        raise PerceptionError(f'cannot read ZED calibration file {path}')
    section = f'{side}_CAM_{mode}'
    if section not in parser:
        raise PerceptionError(f'{path} has no [{section}] section')
    cam = {key: float(value) for key, value in parser[section].items()}
    matrix = np.array([[cam['fx'], 0, cam['cx']], [0, cam['fy'], cam['cy']], [0, 0, 1]])
    dist = [cam['k1'], cam['k2'], cam['p1'], cam['p2'], cam['k3']]
    rational = f'{side}_DISTO'
    if rational in parser:
        d = {key: float(value) for key, value in parser[rational].items()}
        dist = [d['k1'], d['k2'], d['p1'], d['p2'], d['k3'], d['k4'], d['k5'], d['k6']]
    return matrix, dist, ZED_MODES[mode]
