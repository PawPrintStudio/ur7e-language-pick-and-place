"""Stereo height, checked on rendered pairs with known geometry and on one real frame.

The renderer here differs from ``synthetic_table.render`` in one way that
matters: every surface carries a random texture. Stereo matching finds a
point in the other image by its surroundings, so flat-coloured boxes (fine
for the single-camera tests) would give it nothing to find.
"""
from pathlib import Path

import cv2
import numpy as np
import pytest

from ur7e_perception import monocular as mono
from ur7e_perception.core import PerceptionError
from ur7e_perception.stereo import fit_row_rotation, locate_object, object_height, StereoRig
from ur7e_perception.synthetic_table import cuboid, look_at

REPO = Path(__file__).resolve().parents[3]
CONF = REPO / 'docs' / 'calibration' / 'zed2i_SN35717973.conf'
SAMPLE = REPO / 'log' / 'zed' / 'sbs_fhd_sample.png'    # log/ is gitignored

SIZE = (1920, 1080)
BASELINE = 0.119891
# Off-centre principal points, as on the real camera: the "ideal left" image
# (principal point at the centre) is then NOT the raw left image, so the
# ideal -> rectified mapping is really exercised.
K_LEFT = np.array([[1065.0, 0, 942.0], [0, 1065.0, 527.0], [0, 0, 1]])
K_RIGHT = np.array([[1068.0, 0, 919.0], [0, 1068.0, 510.0], [0, 0, 1]])
TABLE_Z = 0.012
PLANE = ([0.0, 0.0, TABLE_Z], [0.0, 0.0, 1.0])
BOX_XY, BOX_SIDE, BOX_YAW = (-0.40, 0.02), 0.06, np.radians(20)
CAMERA = look_at([-0.45, -0.62, 0.50], [-0.40, 0.0, 0.0])      # about 0.8 m from the box
TEXEL = 0.0005                                                  # metres per texture pixel


def speckle(rng, height, width, tint=(1.0, 1.0, 1.0)):
    """Random blotches at two scales, so every patch the matcher compares is unique."""
    def noise(cell, how):
        small = rng.integers(0, 256, (height // cell + 1, width // cell + 1)).astype(np.uint8)
        return cv2.resize(small, (width, height), interpolation=how)
    grey = cv2.addWeighted(noise(16, cv2.INTER_CUBIC), 0.4, noise(4, cv2.INTER_NEAREST), 0.6, 0)
    grey = cv2.GaussianBlur(grey, (0, 0), 0.8)
    return (grey[:, :, None] * np.array(tint)).astype(np.uint8)


def scene_faces(box_height, textured_table=True, box_xy=BOX_XY, seed=7):
    """Return [(4 world corners, texture)]: the table, then the box's sides, then its top."""
    rng = np.random.default_rng(seed)

    def face(corners, tint=(1.0, 1.0, 1.0)):
        corners = np.asarray(corners, dtype=float)
        width = int(np.linalg.norm(corners[1] - corners[0]) / TEXEL)
        height = int(np.linalg.norm(corners[3] - corners[0]) / TEXEL)
        return corners, speckle(rng, height, width, tint)

    x0, x1, y0, y1 = -1.05, 0.05, -0.45, 0.45
    table = face([[x0, y0, TABLE_Z], [x1, y0, TABLE_Z], [x1, y1, TABLE_Z], [x0, y1, TABLE_Z]])
    if not textured_table:
        table[1][:] = 128
    v = cuboid(box_xy, (BOX_SIDE, BOX_SIDE, box_height), BOX_YAW, TABLE_Z)
    # Corner order makes (c1 - c0) x (c3 - c0) the outward normal of each face.
    sides = [face(v[list(index)], (1.0, 0.6, 0.5))
             for index in ((0, 4, 5, 1), (4, 6, 7, 5), (6, 2, 3, 7), (2, 0, 1, 3))]
    return [table] + sides + [face(v[[1, 5, 7, 3]], (1.0, 0.6, 0.5))]


def render_eye(k, eye_to_base, faces):
    """Paint the faces a pinhole camera can see, back to front."""
    image = np.full((SIZE[1], SIZE[0], 3), 128, np.uint8)
    for corners, texture in faces:
        outward = np.cross(corners[1] - corners[0], corners[3] - corners[0])
        if outward @ (eye_to_base[:3, 3] - corners[0]) <= 0:
            continue                                    # this face looks away from the eye
        height, width = texture.shape[:2]
        source = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=float)
        warp, _ = cv2.findHomography(source, mono.project(corners, k, eye_to_base))
        layer = cv2.warpPerspective(texture, warp, SIZE, flags=cv2.INTER_LINEAR)
        cover = cv2.warpPerspective(np.full((height, width), 255, np.uint8), warp, SIZE,
                                    flags=cv2.INTER_NEAREST) > 0
        image[cover] = layer[cover]
    return image


def distort(image, k, dist):
    """Turn a pinhole image into what a lens with this distortion delivers."""
    grid = np.stack(np.meshgrid(np.arange(SIZE[0]), np.arange(SIZE[1])), -1)
    ideal = cv2.undistortPoints(grid.reshape(-1, 1, 2).astype(np.float64), k,
                                np.asarray(dist, dtype=float), P=k)
    ideal = ideal.reshape(SIZE[1], SIZE[0], 2).astype(np.float32)
    return cv2.remap(image, ideal[..., 0], ideal[..., 1], cv2.INTER_LINEAR,
                     borderValue=(128, 128, 128))


def render_pair(rig, box_height, textured_table=True, box_xy=BOX_XY):
    """Return the raw (left, right) images the rig's two lenses would deliver."""
    faces = scene_faces(box_height, textured_table, box_xy)
    right_from_left = np.eye(4)
    right_from_left[:3, :3], right_from_left[:3, 3] = rig.rotation, rig.translation
    right_to_base = CAMERA @ np.linalg.inv(right_from_left)
    left = render_eye(rig.k_left, CAMERA, faces)
    right = render_eye(rig.k_right, right_to_base, faces)
    if rig.dist_left.any() or rig.dist_right.any():
        left = distort(left, rig.k_left, rig.dist_left)
        right = distort(right, rig.k_right, rig.dist_right)
    return left, right


def ideal_pixels(rig, points_base, fill=True):
    """Pixels (Nx2) of the ideal left image covered by the hull of some base-frame points."""
    k_ideal = mono.intrinsics(rig.k_left[0, 0], SIZE)
    hull = cv2.convexHull(mono.project(points_base, k_ideal, CAMERA).astype(np.int32))
    mask = np.zeros((SIZE[1], SIZE[0]), np.uint8)
    cv2.fillConvexPoly(mask, hull, 1)
    return np.argwhere(mask)[:, ::-1] if fill else hull.reshape(-1, 2)


def box_pixels(rig, box_height, box_xy=BOX_XY):
    """Return the object's mask pixels, as the single-camera pipeline would hand them over."""
    return ideal_pixels(rig, cuboid(box_xy, (BOX_SIDE, BOX_SIDE, box_height), BOX_YAW, TABLE_Z))


def table_pixels(rig):
    """Return the pixels of a patch of bare table beside the box."""
    patch = [[-0.56, -0.05, TABLE_Z], [-0.50, -0.05, TABLE_Z],
             [-0.50, 0.01, TABLE_Z], [-0.56, 0.01, TABLE_Z]]
    return ideal_pixels(rig, np.array(patch))


def make_rig(name):
    """Zero-distortion rigs: exactly parallel, or tilted and offset like a real ZED."""
    if name == 'parallel':
        return StereoRig(K_LEFT, np.zeros(5), K_LEFT, np.zeros(5), np.eye(3),
                         [-BASELINE, 0, 0], SIZE)
    rotation = cv2.Rodrigues(np.array([0.011, 0.005, 0.002]))[0]
    return StereoRig(K_LEFT, np.zeros(5), K_RIGHT, np.zeros(5), rotation,
                     [-BASELINE, -0.0004, 0.0003], SIZE)


@pytest.fixture(scope='module')
def tilted():
    """Render the tilted rig looking at a 40 mm box once, for several tests."""
    rig = make_rig('tilted')
    return (rig,) + render_pair(rig, 0.04)


@pytest.mark.parametrize('name', ['parallel', 'tilted'])
@pytest.mark.parametrize('box_height', [0.04, 0.08])
def test_box_height_recovered_within_5mm(name, box_height):
    rig = make_rig(name)
    left, right = render_pair(rig, box_height)
    points = rig.points_for_pixels(left, right, box_pixels(rig, box_height), rig.k_left[0, 0])
    height, used = object_height(points, CAMERA, *PLANE)
    assert used > 1000
    assert abs(height - box_height) < 0.005


def test_table_is_reconstructed_on_the_table(tilted):
    """Absolute geometry: stereo points of the table land on the calibrated plane."""
    rig, left, right = tilted
    pixels = table_pixels(rig)
    points = rig.points_for_pixels(left, right, pixels, rig.k_left[0, 0])
    valid = np.isfinite(points).all(axis=1)
    assert valid.mean() > 0.9
    in_base = points[valid] @ CAMERA[:3, :3].T + CAMERA[:3, 3]
    assert abs(np.median(in_base[:, 2]) - TABLE_Z) < 0.002
    # Each point lies on the ray of the ideal pixel that was asked for.
    k_ideal = mono.intrinsics(rig.k_left[0, 0], SIZE)
    assert np.abs(mono.project(in_base, k_ideal, CAMERA) - pixels[valid]).max() < 1e-6


def test_blank_table_gives_no_points_rather_than_wrong_ones():
    rig = make_rig('parallel')
    left, right = render_pair(rig, 0.04, textured_table=False)
    points = rig.points_for_pixels(left, right, table_pixels(rig), rig.k_left[0, 0])
    assert np.isfinite(points).all(axis=1).mean() < 0.05
    with pytest.raises(PerceptionError):
        object_height(points, CAMERA, *PLANE)
    # The textured box on it is still measured.
    box = rig.points_for_pixels(left, right, box_pixels(rig, 0.04), rig.k_left[0, 0])
    assert abs(object_height(box, CAMERA, *PLANE)[0] - 0.04) < 0.005


def test_object_near_the_left_image_edge_is_still_measured():
    """The matcher is blind in its first few hundred columns unless they are padded."""
    rig = make_rig('tilted')
    box_xy = (-0.90, 0.02)
    pixels = box_pixels(rig, 0.04, box_xy)
    assert 150 < pixels[:, 0].mean() < 350          # well inside the unpadded blind strip
    left, right = render_pair(rig, 0.04, box_xy=box_xy)
    points = rig.points_for_pixels(left, right, pixels, rig.k_left[0, 0])
    height, used = object_height(points, CAMERA, *PLANE)
    assert used > 1000 and abs(height - 0.04) < 0.005


def test_pixels_outside_the_image_are_nan_and_wrong_size_is_refused(tilted):
    rig, left, right = tilted
    points = rig.points_for_pixels(left, right, [[-50.0, 300.0], [5000.0, 5000.0]],
                                   rig.k_left[0, 0])
    assert points.shape == (2, 3) and np.isnan(points).all()
    with pytest.raises(PerceptionError, match='1920x1080'):
        rig.points_for_pixels(left[:720, :1280], right[:720, :1280], [[640, 360]], 1065.0)


def test_cameras_in_the_wrong_order_are_refused():
    with pytest.raises(PerceptionError, match='right'):
        StereoRig(K_LEFT, np.zeros(5), K_LEFT, np.zeros(5), np.eye(3), [BASELINE, 0, 0], SIZE)


@pytest.mark.skipif(not CONF.exists(), reason='factory calibration file not present')
def test_from_zed_conf():
    rig = StereoRig.from_zed_conf(CONF)
    assert abs(rig.baseline_m - 0.1199) < 1e-4
    assert rig.image_size == (1920, 1080)
    for maps in (rig.maps_left, rig.maps_right):
        assert all(m.shape == (1080, 1920) and np.isfinite(m).all() for m in maps)
    assert rig.R1.shape == (3, 3) and rig.P1.shape == (3, 4) and rig.Q.shape == (4, 4)
    # One pixel of disparity is worth 5 mm of depth at about 0.8 m.
    assert 0.004 < rig.depth_per_pixel(0.8) < 0.006
    assert len(StereoRig.from_zed_conf(CONF, rational=False).dist_left) == 5
    assert StereoRig.from_zed_conf(CONF, mode='HD').image_size == (1280, 720)


@pytest.mark.skipif(not CONF.exists(), reason='factory calibration file not present')
def test_factory_rig_measures_height_through_real_lens_distortion():
    """The whole hand-over: raw pair -> monocular undistortion -> colour mask -> height.

    The pair is rendered through this camera's real distortion and lens
    offsets. The mask is then made the way the node makes it, on the image
    ``TableCalibration.undistort`` produces, which pins down the "ideal left"
    convention that ``points_for_pixels`` promises to understand.
    """
    rig = StereoRig.from_zed_conf(CONF)
    left, right = render_pair(rig, 0.04)
    focal = float(rig.k_left[0, 0])
    calibration = mono.TableCalibration(np.eye(4), focal_px=focal, dist=list(rig.dist_left),
                                        camera_matrix=rig.k_left.tolist(), image_size=SIZE)
    ideal = calibration.undistort(left).astype(int)
    reddish = (ideal[..., 0] > ideal[..., 2] + 10).astype(np.uint8)    # the box is tinted
    truth = np.zeros_like(reddish)
    truth[tuple(box_pixels(rig, 0.04)[:, ::-1].T)] = 1
    assert (reddish & truth).sum() / (reddish | truth).sum() > 0.9
    points = rig.points_for_pixels(left, right, np.argwhere(reddish)[:, ::-1], focal)
    height, used = object_height(points, CAMERA, *PLANE)
    assert used > 1000 and abs(height - 0.04) < 0.005


@pytest.mark.skipif(not (CONF.exists() and SAMPLE.exists()),
                    reason='needs the real side-by-side frame in log/zed (gitignored)')
def test_rectification_quality_on_a_real_frame():
    """Rows line up and disparities are positive on a frame from the real camera.

    The sample is motion-blurred and distant, so this is a coarse check of
    the sign conventions (unrectified, the rows are 27 px apart), not a
    certificate of sub-pixel calibration. Measured: 0.67 px with the 65 px
    patches used here, and it shrinks as the patch grows (1.3 px at 33 px,
    0.55-0.7 px at 97 px), which is how matching noise behaves; corner
    features (ORB, LK) scatter by 2.4 px on this frame. Re-run on a sharp
    frame of the table before trusting the last pixel.
    """
    frame = cv2.cvtColor(cv2.imread(str(SAMPLE)), cv2.COLOR_BGR2RGB)
    assert frame.shape == (1080, 3840, 3)
    left, right = frame[:, :1920], frame[:, 1920:]
    rig = StereoRig.from_zed_conf(CONF)
    result = rig.rectification_error(left, right)
    assert result['patches'] >= 30
    assert result['median_abs_dy'] < 1.0
    assert result['positive_fraction'] > 0.95 and result['median_disparity'] > 0
    # Applying the factory rotation backwards must be visibly worse (or unmatchable).
    backwards = StereoRig(rig.k_left, rig.dist_left, rig.k_right, rig.dist_right,
                          rig.rotation.T, rig.translation, SIZE)
    try:
        worse = backwards.rectification_error(left, right)['median_abs_dy']
    except PerceptionError:
        worse = float('inf')        # rows so far apart that nothing matched at all
    assert worse > 3.0


def cloud(heights, seed=0):
    """Camera-frame points whose heights above the test plane are ``heights``."""
    rng = np.random.default_rng(seed)
    heights = np.asarray(heights, dtype=float)
    in_base = np.column_stack((rng.uniform(-0.43, -0.37, len(heights)),
                               rng.uniform(-0.01, 0.05, len(heights)), TABLE_Z + heights))
    base_to_cam = np.linalg.inv(CAMERA)
    return in_base @ base_to_cam[:3, :3].T + base_to_cam[:3, 3]


def test_percentile_finds_the_top_among_sides_and_outliers():
    rng = np.random.default_rng(1)
    top = 0.05 + rng.normal(0, 0.001, 600)
    sides = rng.uniform(0, 0.05, 400)               # every height from table to top
    wild = np.array([0.20] * 20 + [0.50] * 20 + [-0.10] * 20)
    points = np.vstack((cloud(np.concatenate((top, sides, wild))), np.full((50, 3), np.nan)))
    height, used = object_height(points, CAMERA, *PLANE)
    assert abs(height - 0.05) < 0.003
    assert used == 600 + 400 + 20                   # NaN, below-table and >0.3 m dropped


def test_object_height_refuses_instead_of_guessing():
    with pytest.raises(PerceptionError, match='no stereo depth'):
        object_height(np.full((500, 3), np.nan), CAMERA, *PLANE)
    with pytest.raises(PerceptionError, match='only 10 '):
        object_height(cloud([0.04] * 10), CAMERA, *PLANE)
    with pytest.raises(PerceptionError, match='500 are below the table'):
        object_height(cloud([-0.02] * 500), CAMERA, *PLANE)
    with pytest.raises(PerceptionError, match='500 too high'):
        object_height(cloud([0.45] * 500), CAMERA, *PLANE)


def test_table_reference_cancels_a_common_depth_error():
    """Stereo and board disagree by 8 mm: wrong alone, right against stereo's own table."""
    rng = np.random.default_rng(2)
    bias = 0.008
    box = cloud(0.04 + bias + rng.normal(0, 0.0005, 500))
    table = cloud(bias + rng.normal(0, 0.0005, 500), seed=3)
    assert abs(object_height(box, CAMERA, *PLANE)[0] - 0.04) > 0.006
    height, _ = object_height(box, CAMERA, *PLANE, table_points_cam=table)
    assert abs(height - 0.04) < 0.002
    with pytest.raises(PerceptionError, match='table reference'):
        object_height(box, CAMERA, *PLANE, table_points_cam=table[:5])


def project_pair(rig, rotation, points):
    """Raw left/right pixels of camera-frame points for a right lens at ``rotation``."""
    left = cv2.projectPoints(points, np.zeros(3), np.zeros(3), rig.k_left, rig.dist_left)[0]
    right = cv2.projectPoints(points, cv2.Rodrigues(rotation)[0], rig.translation,
                              rig.k_right, rig.dist_right)[0]
    return left.reshape(-1, 2), right.reshape(-1, 2)


@pytest.mark.skipif(not CONF.exists(), reason='factory calibration file not present')
@pytest.mark.parametrize('tilt_deg', [0.3, 1.6])
def test_a_tilted_lens_is_re_measured_from_matched_points(tilt_deg):
    # 2026-10-06: today's frames sat 33 px apart in rows with the factory
    # rotation; refitting RX and RZ brought them to under a pixel.
    factory = StereoRig.from_zed_conf(CONF)
    rng = np.random.default_rng(7)
    depth = rng.uniform(0.5, 3.0, 400)
    pixels = np.column_stack((rng.uniform(150, 1770, 400), rng.uniform(100, 980, 400)))
    rays = np.column_stack(((pixels - factory.k_left[:2, 2]) / factory.k_left[0, 0],
                            np.ones(400)))
    points = rays * depth[:, None]
    true_rvec = cv2.Rodrigues(factory.rotation)[0].ravel() + np.radians([-tilt_deg, 0, 0.05])
    left, right = project_pair(factory, true_rvec, points)
    right += rng.normal(0, 0.3, right.shape)          # matching noise
    refined, report = fit_row_rotation(factory, left, right)
    assert report['before_median_abs_dy'] > 3.0
    assert report['after_median_abs_dy'] < 0.4
    found = cv2.Rodrigues(refined.rotation)[0].ravel()
    assert abs(np.degrees(found[0] - true_rvec[0])) < 0.02
    assert abs(np.degrees(found[2] - true_rvec[2])) < 0.05
    # Toe-in is not touched: rows cannot see it.
    assert found[1] == pytest.approx(cv2.Rodrigues(factory.rotation)[0].ravel()[1])


def box_mask(rig, box_height):
    """The detection mask (ideal left image) of the rendered box."""
    mask = np.zeros((SIZE[1], SIZE[0]), bool)
    pixels = box_pixels(rig, box_height)
    mask[pixels[:, 1], pixels[:, 0]] = True
    return mask


def test_stereo_locator_finds_the_top_centre_of_a_box(tilted):
    rig, left, right = tilted
    grasp = locate_object(rig, left, right, box_mask(rig, 0.04), rig.k_left[0, 0], CAMERA,
                          *PLANE)
    assert np.hypot(*(grasp.position[:2] - BOX_XY)) < 0.006
    assert abs(grasp.height - 0.04) < 0.005
    assert abs(grasp.position[2] - (TABLE_Z + 0.04)) < 0.005
    assert abs(grasp.table_offset) < 0.003
    assert grasp.axis_ratio < 2.0 and grasp.yaw == 0.0     # a square box has no long axis
    assert 0.04 < grasp.extent[0] < 0.09


def test_stereo_locator_height_survives_a_wrong_table_plane(tilted):
    # The calibration's table is 1 cm too high: the ring of real table next to
    # the box reads -1 cm, so the HEIGHT is still right; z follows the plane
    # the robot was told about, which is what the pick code plans against.
    rig, left, right = tilted
    plane = ([0.0, 0.0, TABLE_Z + 0.01], [0.0, 0.0, 1.0])
    grasp = locate_object(rig, left, right, box_mask(rig, 0.04), rig.k_left[0, 0], CAMERA,
                          *plane)
    assert abs(grasp.table_offset + 0.01) < 0.003
    assert abs(grasp.height - 0.04) < 0.005


def test_stereo_locator_refuses_a_blank_object():
    rig = make_rig('tilted')
    blank = np.full((SIZE[1], SIZE[0], 3), 128, np.uint8)
    with pytest.raises(PerceptionError):
        locate_object(rig, blank, blank, box_mask(rig, 0.04), rig.k_left[0, 0], CAMERA, *PLANE)
