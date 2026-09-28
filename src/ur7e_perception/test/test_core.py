"""Metric and rejection regressions independent of model weights and ROS."""
import math

import numpy as np
import pytest

from ur7e_perception.backends import ColorFixtureBackend, foreground_mask
from ur7e_perception.core import (
    PerceptionError, check_fresh, fit_calibration, localize, rigid_transform, verify_marker,
)
from ur7e_perception.synthetic import CAMERA_TO_BASE, WORKSPACE, scene, save_frame, load_frame


@pytest.mark.parametrize('seed', range(30))
def test_metric_geometry(seed):
    frame, expected, yaw, _ = scene(seed)
    detection = ColorFixtureBackend().detect(frame, 'red block')
    grasp = localize(frame, detection, CAMERA_TO_BASE, WORKSPACE)
    assert np.linalg.norm(grasp.position-expected) < .005
    error = abs((grasp.yaw-yaw+math.pi/2) % math.pi-math.pi/2)
    assert error < .15
    assert np.linalg.norm(grasp.quaternion) == pytest.approx(1)
    assert np.allclose(grasp.quaternion[2:], [0, 0])


@pytest.mark.parametrize('fault', ['nan', 'zero', 'negative', 'far', 'partial', 'mask',
                                  'intrinsics', 'units', 'workspace', 'below', 'above'])
def test_rejections(fault):
    frame, _, _, _ = scene()
    detection = ColorFixtureBackend().detect(frame, 'red block')
    transform, workspace = CAMERA_TO_BASE.copy(), WORKSPACE
    if fault in ('nan', 'zero', 'negative', 'far'):
        frame.depth[:] = {'nan': np.nan, 'zero': 0, 'negative': -1, 'far': 10}[fault]
    elif fault == 'partial':
        v, u = np.nonzero(detection.mask)
        frame.depth[v[:len(v)*3//4], u[:len(v)*3//4]] = np.nan
    elif fault == 'mask':
        detection.mask = np.zeros((2, 2))
    elif fault == 'intrinsics':
        frame.k[0, 0] = 0
    elif fault == 'units':
        frame.depth = (np.nan_to_num(frame.depth)*1000).astype('uint16')
    elif fault == 'workspace':
        workspace = [[0, 0], [.1, 0], [.1, .1], [0, .1]]
    elif fault == 'below':
        transform[2, 3] -= .2
    elif fault == 'above':
        transform[2, 3] += .4
    with pytest.raises(PerceptionError):
        localize(frame, detection, transform, workspace)


def test_depth_outliers_and_segmentation():
    frame, expected, _, _ = scene()
    d = ColorFixtureBackend().detect(frame, 'red block')
    v, u = np.nonzero(d.mask)
    frame.depth[v[::10], u[::10]] = 2.5
    g = localize(frame, d, CAMERA_TO_BASE, WORKSPACE)
    assert np.linalg.norm(g.position-expected) < .005
    mask = foreground_mask(frame, d.bbox)
    assert np.logical_and(mask, d.mask).sum()/d.mask.sum() > .8
    frame.depth[:] = 1
    with pytest.raises(PerceptionError, match='foreground'):
        foreground_mask(frame, d.bbox)


def test_stale_future_missing_and_ambiguous():
    frame = scene(stamp=100)[0]
    check_fresh(frame, 101)
    for now in [99, 103, float('nan')]:
        with pytest.raises(PerceptionError):
            check_fresh(frame, now)
    with pytest.raises(PerceptionError):
        ColorFixtureBackend().detect(frame, 'hammer')
    with pytest.raises(PerceptionError):
        ColorFixtureBackend().detect(frame, 'blue block')
    frame.rgb[:10, :10] = [255, 0, 0]
    with pytest.raises(PerceptionError, match='ambiguous'):
        ColorFixtureBackend().detect(frame, 'red block')


def test_calibration_fit_heldout_drift_and_degeneracy():
    rng = np.random.default_rng(10)
    camera = rng.uniform(-.2, .2, (15, 3)) + [0, 0, .9]
    base = camera @ CAMERA_TO_BASE[:3, :3].T + CAMERA_TO_BASE[:3, 3]
    transform, residual = fit_calibration(camera[:10], base[:10])
    assert residual.max() < 1e-10
    assert verify_marker(camera[10:], base[10:], transform).max() < 1e-10
    transform[0, 3] += .02
    with pytest.raises(PerceptionError):
        verify_marker(camera[10:], base[10:], transform)
    with pytest.raises(PerceptionError):
        fit_calibration(np.ones((5, 3)), np.ones((5, 3)))
    with pytest.raises(PerceptionError):
        rigid_transform(np.diag([1, 1, -1, 1]))


def test_roundtrip(tmp_path):
    frame = scene()[0]
    path = tmp_path/'capture.npz'
    save_frame(path, frame)
    loaded = load_frame(path)
    assert np.array_equal(frame.rgb, loaded.rgb)
    assert np.allclose(frame.depth, loaded.depth, equal_nan=True)
    assert frame.frame_id == loaded.frame_id


@pytest.mark.parametrize('options', [{'table_z': float('nan')}, {'max_z': -.1},
                                    {'max_depth': float('inf')}])
def test_invalid_bounds_rejected(options):
    frame = scene()[0]
    detection = ColorFixtureBackend().detect(frame, 'red block')
    with pytest.raises(PerceptionError, match='bounds'):
        localize(frame, detection, CAMERA_TO_BASE, WORKSPACE, **options)
