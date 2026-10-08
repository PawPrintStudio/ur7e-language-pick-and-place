"""The real tool stack on the UR7e flange: Dual Quick Changer, RG2, soft gripper.

One place for the geometry every planner and calibration script must agree on.
The OnRobot Dual Quick Changer v3 sits on the flange and carries two tools
splayed in a V (photos: docs/vendor/cad/photos/). Each tool face points 60 deg
off the flange axis, so **no tool points along tool0**. The RG2 is the one we
grip with; the soft gripper on the other face only has to be avoided.

Numbers measured from the manufacturer CAD (docs/vendor/cad/README.md), all in
tool0's frame (Z out of the flange, metres):

* changer: 128 x 71 x 97 mm block on the flange (OnRobot lists 127 x 71 x 96);
* tool faces: centres at (+-34.6, 0, 59.7) mm, normals (+-0.866, 0, 0.5);
* RG2: OnRobot's TCP is 200 mm out from its mount face (manual 8.3.1);
  the community RG2 mesh reaches 233.6 mm (all openings);
* soft gripper: 28.4 mm Quick Changer tool side + 113 mm body.

What the CAD cannot say is checked in the lab, and lives in ``lab_tooling.yaml``:
how the changer is turned on the flange (``changer_yaw_deg``), which face holds
the RG2, the RG2's roll on its face, and the exact TCP length. Until a person has
checked them on the arm and set ``verified: true``, ``lab_pick.py --execute``
refuses to move the real robot (simulation passes ``--allow-unverified-tooling``).

``python3 scripts/lab_tooling.py`` prints the frames and the URDF snippet.
"""
import math
import os

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
TOOLING_FILE = os.path.join(_HERE, 'lab_tooling.yaml')

# --- fixed by the CAD (Dual Quick Changer v3, STEP 'Dual QC_2-0') -------------------
CHANGER_SIZE = (0.128, 0.071, 0.097)        # x, y, z in tool0 (z along the flange axis)
FACE_CENTRE = (0.0346, 0.0, 0.0597)         # +x face; the -x face mirrors it
FACE_TILT_DEG = 60.0                        # each tool axis from the flange axis
# RG2 envelope in its mount frame (z out of the face), from the community
# collision mesh sampled over 0..110 mm opening: x +-72.6, y -39.4..35.9, z 0..233.6 mm.
RG2_REACH = 0.2336
RG2_HALF_WIDTH = 0.0726
SOFT_GRIPPER_LENGTH = 0.0284 + 0.113        # QC tool side + body (STEP bbox)
SOFT_GRIPPER_WIDTH = 0.094                  # widest side of the soft-gripper STEP bbox
PAD = 0.005                                 # collision padding on every box

DEFAULTS = {
    'verified': False,
    'changer_yaw_deg': 0.0,       # changer +x face direction, measured from tool0 +x about tool0 z
    'rg2_face': '+x',             # which changer face holds the RG2: '+x' or '-x'
    'rg2_roll_deg': 0.0,          # RG2 turned on its face, about its own axis
    'rg2_tcp_m': 0.200,           # OnRobot RG2 TCP along the tool axis from the mount face
    'tcp_to_tips_m': 0.034,       # closed fingertips beyond the TCP (conservative: mesh max)
}


def load(path=TOOLING_FILE):
    """Return the tooling settings: defaults overlaid with ``lab_tooling.yaml``."""
    settings = dict(DEFAULTS)
    if path and os.path.exists(path):
        with open(path) as stream:
            settings.update(yaml.safe_load(stream) or {})
    if settings['rg2_face'] not in ('+x', '-x'):
        raise ValueError(f"rg2_face must be '+x' or '-x', not {settings['rg2_face']!r}")
    return settings


# --- small transform helpers (numpy only, so launch files can import this) ---------

def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def transform(rotation=None, translation=(0, 0, 0)):
    matrix = np.eye(4)
    if rotation is not None:
        matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def rpy(matrix):
    """URDF roll-pitch-yaw (fixed X, then Y, then Z) of a rotation matrix."""
    r = np.asarray(matrix)[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(r[2, 0]) < 1 - 1e-9:
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock: put everything in yaw
        roll, yaw = 0.0, math.atan2(-r[0, 1], r[1, 1])
    return roll, pitch, yaw


# --- the frames ------------------------------------------------------------------------

def face_mount(sign, settings):
    """tool0 -> mount frame of the changer face on ``sign`` (+1: +x face, -1: -x face).

    The mount frame's z is the tool axis (out of the face), y is tool0's y
    turned with the changer, x completes it.
    """
    yaw = transform(rot_z(settings['changer_yaw_deg']))
    centre = (sign * FACE_CENTRE[0], FACE_CENTRE[1], FACE_CENTRE[2])
    return yaw @ transform(rot_y(sign * FACE_TILT_DEG), centre)


def rg2_sign(settings):
    return 1 if settings['rg2_face'] == '+x' else -1


def rg2_mount(settings):
    return face_mount(rg2_sign(settings), settings) @ transform(rot_z(settings['rg2_roll_deg']))


def soft_mount(settings):
    return face_mount(-rg2_sign(settings), settings)


def tool0_to_rg2_tcp(settings=None):
    """4x4 from tool0 to the RG2 TCP (z along the gripper, out of the fingers)."""
    settings = settings or load()
    return rg2_mount(settings) @ transform(translation=(0, 0, settings['rg2_tcp_m']))


def tool0_to_tips(settings=None):
    """4x4 from tool0 to the closed RG2 fingertips (same axes as the TCP)."""
    settings = settings or load()
    return tool0_to_rg2_tcp(settings) @ transform(
        translation=(0, 0, settings['tcp_to_tips_m']))


def tips_in_base(base_from_tool0, settings=None):
    """Closed-fingertip point in base_link for a base_link->tool0 4x4."""
    return (np.asarray(base_from_tool0) @ tool0_to_tips(settings))[:3, 3]


# --- URDF for the planner ----------------------------------------------------------------

LINKS = ('lab_changer', 'lab_rg2', 'lab_soft_gripper', 'rg2_tcp')


def urdf_elements(settings=None):
    """(name, parent, 4x4 joint origin, box size or None, 4x4 box origin) for each link.

    Every link hangs off tool0 with a fixed joint. ``rg2_tcp`` has no geometry:
    it is the frame the pick planner moves.
    """
    settings = settings or load()
    yaw = transform(rot_z(settings['changer_yaw_deg']))
    size = CHANGER_SIZE
    changer_box = ((size[0] + 2 * PAD, size[1] + 2 * PAD, size[2] + PAD),
                   transform(translation=(0, 0, (size[2] + PAD) / 2)))
    rg2_len = RG2_REACH + PAD
    # Square across the fingers: the RG2's roll on its face is a lab setting.
    rg2_box = ((2 * (RG2_HALF_WIDTH + PAD),) * 2 + (rg2_len,),
               transform(translation=(0, 0, rg2_len / 2)))
    soft_len = SOFT_GRIPPER_LENGTH + PAD
    soft_box = ((SOFT_GRIPPER_WIDTH + 2 * PAD,) * 2 + (soft_len,),
                transform(translation=(0, 0, soft_len / 2)))
    return [
        ('lab_changer', 'tool0', yaw) + changer_box,
        ('lab_rg2', 'tool0', rg2_mount(settings)) + rg2_box,
        ('lab_soft_gripper', 'tool0', soft_mount(settings)) + soft_box,
        ('rg2_tcp', 'tool0', tool0_to_rg2_tcp(settings), None, None),
    ]


def add_to_urdf(robot, settings=None):
    """Append the tool links and fixed joints to an ``xml.etree`` URDF root."""
    import xml.etree.ElementTree as ET

    def origin(parent, matrix):
        xyz = ' '.join(f'{v:.6f}' for v in matrix[:3, 3])
        angles = ' '.join(f'{v:.6f}' for v in rpy(matrix))
        ET.SubElement(parent, 'origin', xyz=xyz, rpy=angles)

    for name, parent, joint_origin, box, box_origin in urdf_elements(settings):
        link = ET.SubElement(robot, 'link', name=name)
        if box is not None:
            collision = ET.SubElement(link, 'collision')
            origin(collision, box_origin)
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'box', size=' '.join(f'{v:.4f}' for v in box))
        joint = ET.SubElement(robot, 'joint', name=f'{name}_joint', type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=name)
        origin(joint, joint_origin)


# Pairs the planner may ignore: rigidly joined, so a touch is the mounting itself.
ADJACENT = [(a, b) for a in ('lab_changer', 'lab_rg2', 'lab_soft_gripper')
            for b in ('tool0', 'flange', 'wrist_3_link')] + [
    ('lab_changer', 'lab_rg2'), ('lab_changer', 'lab_soft_gripper'),
    ('lab_rg2', 'lab_soft_gripper')]
TOUCH_LINKS = ['lab_changer', 'lab_rg2', 'lab_soft_gripper', 'tool0', 'flange', 'wrist_3_link']


def main():
    import xml.etree.ElementTree as ET
    settings = load()
    np.set_printoptions(precision=4, suppress=True)
    print('settings:', settings)
    tcp, tips = tool0_to_rg2_tcp(settings), tool0_to_tips(settings)
    print('tool0 -> rg2_tcp xyz (m):', tcp[:3, 3], ' axis:', tcp[:3, 2])
    print('tool0 -> fingertips xyz (m):', tips[:3, 3])
    tilt = math.degrees(math.acos(tcp[2, 2]))
    print(f'RG2 axis is {tilt:.1f} deg off the flange axis')
    robot = ET.Element('robot', name='preview')
    add_to_urdf(robot, settings)
    ET.indent(robot)
    print(ET.tostring(robot, encoding='unicode'))


if __name__ == '__main__':
    main()
