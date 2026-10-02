"""Single-camera localization, checked on images rendered from known geometry."""
import math

import cv2
import numpy as np
import pytest

from ur7e_perception import monocular as mono
from ur7e_perception.core import PerceptionError

from ur7e_perception.synthetic_table import (
    board_in_base, cuboid, FOCAL, look_at, render, SIZE)


@pytest.fixture(scope='module')
def scene():
    cam_to_base = look_at([-0.55, -0.85, 0.55], [-0.40, 0.0, 0.0])
    base_from_board = board_in_base()
    block = cuboid((-0.30, 0.12), (0.04, 0.04, 0.04), 0.0, base_from_board[2, 3])
    image, masks = render(cam_to_base, base_from_board, [(block, (200, 30, 30))])
    return cam_to_base, base_from_board, image, masks[0]


def test_printable_board_round_trips():
    pages = mono.printable_sheets(dpi=150)
    pixels, _ = mono.detect_board(cv2.cvtColor(pages[0], cv2.COLOR_GRAY2RGB),
                                  mono.table_board())
    assert len(pixels) == 24
    center, _ = mono.marker_center(cv2.cvtColor(pages[1], cv2.COLOR_GRAY2RGB), 42)
    assert np.isfinite(center).all()


def test_single_view_focal_is_only_approximate(scene):
    """One view of a small board: usable, but 10 % off is normal. Hence the next test."""
    cam_to_base, base_from_board, image, _ = scene
    view = mono.observe_board(image, mono.table_board())
    assert view.corners >= 20 and view.rms_px < 1.0
    assert abs(view.k[0, 0] - FOCAL) / FOCAL < 0.15


def test_multi_view_calibration_recovers_focal():
    base_from_board = board_in_base(z=0.0)
    middle = base_from_board[:3, 3] + base_from_board[:3, :3] @ [0.0875, 0.1225, 0]
    frames = []
    for angle in np.linspace(0, 2 * math.pi, 9)[:-1]:
        for radius, height in ((0.25, 0.35), (0.40, 0.25)):
            position = middle + [radius * math.cos(angle), radius * math.sin(angle), height]
            frames.append(render(look_at(position, middle), base_from_board)[0])
    focal, dist, rms, used = mono.calibrate_intrinsics(frames, mono.table_board(), SIZE)
    assert used >= 12 and rms < 1.0
    assert abs(focal - FOCAL) / FOCAL < 0.03
    assert abs(dist[0]) < 0.1


def test_camera_pose_recovered_with_calibrated_focal(scene):
    cam_to_base, base_from_board, image, _ = scene
    view = mono.observe_board(image, mono.table_board(), focal_px=FOCAL)
    recovered = mono.camera_to_base(view, base_from_board)
    assert np.linalg.norm(recovered[:3, 3] - cam_to_base[:3, 3]) < 0.02


def test_averaging_frames_keeps_only_stable_corners(scene):
    _, _, image, _ = scene
    blank = np.full_like(image, 170)
    pixels, _ = mono.detect_board([image, image, blank], mono.table_board())
    assert len(pixels) >= 20
    with pytest.raises(PerceptionError):
        mono.detect_board([image, blank, blank], mono.table_board())


def test_touch_fit_recovers_board_and_reports_residual():
    truth = board_in_base()
    tips = mono.touch_points() @ truth[:3, :3].T + truth[:3, 3]
    noisy = tips + np.array([[.001, 0, 0], [0, -.001, 0], [0, 0, .001], [-.001, 0, 0]])
    fitted, residual = mono.fit_board_anchor(noisy)
    assert np.allclose(fitted[:3, 3], truth[:3, 3], atol=0.002)
    assert residual.max() < 0.003
    with pytest.raises(PerceptionError):
        mono.fit_board_anchor(tips + np.array([[.03, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]))


def test_block_localized_within_6mm_when_height_is_known(scene):
    _, base_from_board, image, mask = scene
    view = mono.observe_board(image, mono.table_board(), focal_px=FOCAL)
    cam_to_base = mono.camera_to_base(view, base_from_board)
    grasp = mono.localize_on_table(mask, view.k, cam_to_base, base_from_board,
                                   object_height=0.04)
    assert np.linalg.norm(grasp.position[:2] - [-0.30, 0.12]) < 0.006
    assert abs(grasp.position[2] - (base_from_board[2, 3] + 0.04)) < 1e-6
    # The height shadow is gone: a cube looks like a cube again.
    assert grasp.yaw == 0.0
    assert all(abs(side - 0.04) < 0.008 for side in grasp.extent)


def test_overstated_height_is_refused_not_guessed(scene):
    _, base_from_board, image, mask = scene
    view = mono.observe_board(image, mono.table_board(), focal_px=FOCAL)
    cam_to_base = mono.camera_to_base(view, base_from_board)
    with pytest.raises(PerceptionError, match='height'):
        mono.localize_on_table(mask, view.k, cam_to_base, base_from_board,
                               object_height=0.12)


def test_ignoring_height_smears_away_from_camera(scene):
    """The reason object_height exists: without it the answer is biased."""
    _, base_from_board, image, mask = scene
    view = mono.observe_board(image, mono.table_board())
    cam_to_base = mono.camera_to_base(view, base_from_board)
    flat = mono.localize_on_table(mask, view.k, cam_to_base, base_from_board,
                                  object_height=0.0)
    assert np.linalg.norm(flat.position[:2] - [-0.30, 0.12]) > 0.012


def test_unknown_height_uses_the_near_edge(scene):
    """No height given: still within a centimetre for a compact object."""
    _, base_from_board, image, mask = scene
    view = mono.observe_board(image, mono.table_board(), focal_px=FOCAL)
    cam_to_base = mono.camera_to_base(view, base_from_board)
    grasp = mono.localize_on_table(mask, view.k, cam_to_base, base_from_board, None)
    assert np.linalg.norm(grasp.position[:2] - [-0.30, 0.12]) < 0.012
    assert abs(grasp.position[2] - base_from_board[2, 3]) < 1e-6
    # A square's principal axes are arbitrary, so sides read anywhere from
    # the side length to the diagonal; what matters is the shadow is gone.
    assert all(0.035 < side < 0.06 for side in grasp.extent)


def test_elongated_object_yaw(scene):
    cam_to_base, base_from_board, _, _ = scene
    yaw = math.radians(40)
    bar = cuboid((-0.32, 0.05), (0.14, 0.03, 0.02), yaw, base_from_board[2, 3])
    image, masks = render(cam_to_base, base_from_board, [(bar, (30, 60, 200))])
    view = mono.observe_board(image, mono.table_board(), focal_px=FOCAL)
    grasp = mono.localize_on_table(masks[0], view.k, mono.camera_to_base(view, base_from_board),
                                   base_from_board, object_height=0.02)
    assert abs(grasp.yaw - yaw) < math.radians(6)
    assert np.linalg.norm(grasp.position[:2] - [-0.32, 0.05]) < 0.008
    assert abs(grasp.extent[0] - 0.14) < 0.012 and abs(grasp.extent[1] - 0.03) < 0.010


def test_color_blob_and_grabcut_agree_with_truth(scene):
    _, _, image, truth = scene
    mask, bbox = mono.color_blob(image, 'red')
    overlap = (mask & truth).sum() / (mask | truth).sum()
    assert overlap > 0.9
    cut = mono.color_mask(image, bbox)
    assert (cut & truth).sum() / (cut | truth).sum() > 0.8
    with pytest.raises(PerceptionError):
        mono.color_blob(image, 'green')


def test_two_same_colored_objects_are_ambiguous(scene):
    cam_to_base, base_from_board, _, _ = scene
    z = base_from_board[2, 3]
    boxes = [(cuboid((-0.30, 0.12), (0.04, 0.04, 0.04), 0, z), (200, 30, 30)),
             (cuboid((-0.40, 0.20), (0.04, 0.04, 0.04), 0, z), (200, 30, 30))]
    image, _ = render(cam_to_base, base_from_board, boxes)
    with pytest.raises(PerceptionError, match='more than one'):
        mono.color_blob(image, 'red')


def test_object_outside_workspace_rejected(scene):
    _, base_from_board, image, mask = scene
    view = mono.observe_board(image, mono.table_board())
    cam_to_base = mono.camera_to_base(view, base_from_board)
    far = [[0.5, 0.5], [0.9, 0.5], [0.9, 0.9], [0.5, 0.9]]
    with pytest.raises(PerceptionError, match='outside'):
        mono.localize_on_table(mask, view.k, cam_to_base, base_from_board, 0.04, far)
    polygon = mono.visible_table(view.k, cam_to_base, base_from_board, SIZE)
    mono.localize_on_table(mask, view.k, cam_to_base, base_from_board, 0.04, polygon)


def test_missing_board_is_refused():
    blank = np.full((SIZE[1], SIZE[0], 3), 170, np.uint8)
    with pytest.raises(PerceptionError, match='not visible'):
        mono.observe_board(blank, mono.table_board())


def test_calibration_file_round_trip(tmp_path):
    calibration = mono.TableCalibration(board_in_base(), focal_px=FOCAL,
                                        residual_mm=[1.0, 0.5, 0.7, 0.9])
    path = tmp_path / 'table.json'
    calibration.save(path)
    loaded = mono.TableCalibration.load(path)
    assert np.allclose(loaded.base_from_board, calibration.base_from_board)
    assert loaded.focal_px == FOCAL and loaded.image_size == (1280, 720)
