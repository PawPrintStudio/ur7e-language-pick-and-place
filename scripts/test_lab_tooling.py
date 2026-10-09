"""Offline checks for the tool-stack geometry (lab_tooling.py). No ROS needed.

    python3 -m pytest -q scripts/test_lab_tooling.py
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lab_tooling  # noqa: E402

DEFAULTS = dict(lab_tooling.DEFAULTS)


def angle_deg(a, b):
    return math.degrees(math.acos(np.clip(np.dot(a, b), -1, 1)))


def test_each_tool_points_60_degrees_off_the_flange_axis_and_120_apart():
    rg2 = lab_tooling.rg2_mount(DEFAULTS)[:3, 2]
    soft = lab_tooling.soft_mount(DEFAULTS)[:3, 2]
    assert abs(angle_deg(rg2, [0, 0, 1]) - 60.0) < 1e-9
    assert abs(angle_deg(soft, [0, 0, 1]) - 60.0) < 1e-9
    assert abs(angle_deg(rg2, soft) - 120.0) < 1e-9


def test_rg2_tcp_matches_the_cad_estimate():
    tcp = lab_tooling.tool0_to_rg2_tcp(DEFAULTS)[:3, 3]
    # face (34.6, 0, 59.7) mm + 200 mm along (sin 60, 0, cos 60)
    assert np.allclose(tcp, [0.0346 + 0.2 * math.sin(math.radians(60)), 0.0,
                             0.0597 + 0.2 * 0.5], atol=1e-9)
    tips = lab_tooling.tool0_to_tips(DEFAULTS)[:3, 3]
    assert abs(np.linalg.norm(tips - tcp) - DEFAULTS['tcp_to_tips_m']) < 1e-9
    assert tips[0] > 0.2    # far outside the old 0.20 x 0.20 on-axis box


def test_changer_yaw_turns_everything_about_the_flange_axis():
    turned = lab_tooling.tool0_to_rg2_tcp(dict(DEFAULTS, changer_yaw_deg=90.0))[:3, 3]
    base = lab_tooling.tool0_to_rg2_tcp(DEFAULTS)[:3, 3]
    assert np.allclose(turned, [-base[1], base[0], base[2]], atol=1e-9)


def test_rg2_on_the_other_face_mirrors_x():
    mirrored = lab_tooling.tool0_to_rg2_tcp(dict(DEFAULTS, rg2_face='-x'))[:3, 3]
    base = lab_tooling.tool0_to_rg2_tcp(DEFAULTS)[:3, 3]
    assert np.allclose(mirrored, [-base[0], base[1], base[2]], atol=1e-9)


def test_unknown_face_is_refused(tmp_path):
    bad = tmp_path / 'tooling.yaml'
    bad.write_text("rg2_face: 'up'\n")
    with pytest.raises(ValueError, match='rg2_face'):
        lab_tooling.load(str(bad))


@pytest.mark.parametrize('deg', [(0, 0, 0), (10, -20, 30), (90, 45, -60)])
def test_rpy_round_trip(deg):
    r, p, y = (math.radians(v) for v in deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = lab_tooling.rot_y(math.degrees(p))
    rz = lab_tooling.rot_z(math.degrees(y))
    assert np.allclose(lab_tooling.rpy(rz @ ry @ rx), (r, p, y), atol=1e-9)


def test_urdf_has_every_link_hanging_from_tool0_and_a_bodiless_tcp():
    robot = ET.Element('robot', name='t')
    lab_tooling.add_to_urdf(robot, DEFAULTS)
    names = [link.get('name') for link in robot.findall('link')]
    assert names == list(lab_tooling.LINKS)
    assert {j.find('parent').get('link') for j in robot.findall('joint')} == {'tool0'}
    tcp = robot.find("link[@name='rg2_tcp']")
    assert tcp.find('collision') is None


def test_yaml_file_loads_with_an_explicit_verified_flag():
    # Checked on the arm 2026-10-08 (changer yaw 180, roll 90), so the file is
    # now verified; the gate itself is covered by the lab_pick tests.
    settings = lab_tooling.load()
    assert set(DEFAULTS) <= set(settings)
    assert isinstance(settings['verified'], bool)


def test_gazebo_and_moveit_model_uses_the_same_mount_as_the_lab_planner():
    """ur7e_bringup/urdf/dual_quick_changer.xacro must match lab_tooling.py."""
    import re
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src',
                        'ur7e_bringup', 'urdf', 'dual_quick_changer.xacro')
    text = open(path).read()

    def prop(name):
        return re.search(rf'name="{name}" value="([^"]+)"', text).group(1)

    assert float(prop('dqc_face_x')) == lab_tooling.FACE_CENTRE[0]
    assert float(prop('dqc_face_z')) == lab_tooling.FACE_CENTRE[2]
    assert prop('dqc_tilt') == '${pi / 3}' and lab_tooling.FACE_TILT_DEG == 60.0
    assert abs(float(prop('dqc_tool_side')) + 0.113 - lab_tooling.SOFT_GRIPPER_LENGTH) < 1e-9
