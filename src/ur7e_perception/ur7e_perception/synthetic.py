"""Seeded RGB-D scenes with independent metric ground truth and noise."""

import cv2
import numpy as np

from .core import Frame

CAMERA_TO_BASE = np.array([[1., 0, 0, .45], [0, -1., 0, 0],
                           [0, 0, -1., 1.], [0, 0, 0, 1.]])
WORKSPACE = [[.1, -.35], [.8, -.35], [.8, .35], [.1, .35]]


def scene(seed=0, stamp=0.0, color='red', noise=0.001, dropout=0.02):
    """Render an overhead block; labels never enter the detector input."""
    rng = np.random.default_rng(seed)
    h, w, f = 240, 320, 300.
    k = np.array([[f, 0, w/2], [0, f, h/2], [0, 0, 1.]])
    center = np.array([rng.uniform(.32, .58), rng.uniform(-.16, .16), rng.uniform(.03, .09)])
    yaw = rng.uniform(-1.3, 1.3)
    length, width = rng.uniform(.07, .11), rng.uniform(.025, .045)
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [length/2, width/2]
    r = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    xy = corners @ r.T + center[:2]
    z = 1.-center[2]
    pixels = np.column_stack(((xy[:, 0]-.45)*f/z+w/2, -xy[:, 1]*f/z+h/2))
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, np.rint(pixels).astype('int32'), 1)
    rgb = np.full((h, w, 3), [90, 80, 70], dtype=np.uint8)
    rgb[mask > 0] = {'red': [220, 35, 25], 'green': [25, 220, 35], 'blue': [25, 35, 220]}[color]
    depth = np.ones((h, w), np.float32)
    depth[mask > 0] = z
    depth += rng.normal(0, noise, (h, w)).astype('float32')
    depth[rng.random((h, w)) < dropout] = np.nan
    return Frame(rgb, depth, k, stamp), center, yaw, mask


def save_frame(path, frame):
    """Portable capture: no pickle, explicit metre depth and optical frame."""
    frame.validate()
    np.savez_compressed(path, rgb=frame.rgb, depth=frame.depth, k=frame.k,
                        stamp=frame.stamp, frame_id=frame.frame_id)


def load_frame(path):
    with np.load(path, allow_pickle=False) as data:
        frame = Frame(data['rgb'], data['depth'], data['k'],
                      float(data['stamp']), str(data['frame_id']))
    frame.validate()
    return frame
