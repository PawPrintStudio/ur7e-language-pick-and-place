#!/usr/bin/env python3
"""Calibrate the ZED against the robot with BOTH lenses -- quick enough to redo after every move.

Why this replaces the single-camera wave (lab_camera_calibration.py)
--------------------------------------------------------------------
Two lenses measure distance, so every stop of the wave gives a 3-D point in
the camera's frame, not just a pixel. Three-dimensional pairs pin the camera
pose directly, and nothing else has to be measured by hand: the 10-02 wave
also needed a fingertip touchdown and a ``--table-z`` found by trial, both
because one camera cannot tell near from far.

Two more things become measurable, and both are reported:

* where the held target sits on the gripper (``target_in_tool``), which is
  why the wrist yaws and tilts between stops (see ur7e_perception.handeye);
* a constant error in stereo depth (``disparity_offset_px``), with the
  robot's own kinematics as the ruler.

Before the wave, ``rows`` re-measures how the two lenses are tilted against
each other from whatever the camera sees (2026-10-06: the factory numbers
left the two images 33 px apart in rows; refitted, under one pixel).

Steps (driver + ``lab_arm_moveit.launch.py ik:=kdl`` running, Play pressed,
tool pointing straight down, e.g. after ``go to front`` in the console)::

    python3 scripts/lab_stereo_calibration.py rows               # lens tilt, no motion
    python3 scripts/lab_stereo_calibration.py grip --width 75    # open; a ball between the pads
    python3 scripts/lab_stereo_calibration.py grip --width 0     # close on it
    python3 scripts/lab_stereo_calibration.py --execute wave     # ~3 min
    python3 scripts/lab_stereo_calibration.py solve              # camera pose + checks
    python3 scripts/lab_stereo_calibration.py finish             # -> scripts/lab_table.json
    python3 scripts/lab_stereo_calibration.py check              # live: robot drawn on the image

The target: bright, compact and roughly round (a ball is ideal). Stereo sees
the side facing the camera; the solve moves that surface point back by the
target's radius, measured from its size in the image.
"""
import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from lab_camera_calibration import (  # noqa: E402
    command_grip, moving_target, TARGET_COLORS, wave_poses, white_blobs)
from lab_table_calibration import read_json, write_json  # noqa: E402

from ur7e_perception import handeye  # noqa: E402
from ur7e_perception import monocular as mono  # noqa: E402
from ur7e_perception.camera import Camera, zed_factory_calibration  # noqa: E402
from ur7e_perception.stereo import fit_row_rotation, match_features, StereoRig  # noqa: E402

SESSION = os.path.join(_HERE, '..', 'log', 'stereo_calibration')
ROWS = os.path.join(SESSION, 'rows.json')
OBSERVATIONS = os.path.join(SESSION, 'observations.json')
SOLUTION = os.path.join(SESSION, 'solution.json')
CALIBRATION_FILE = os.path.join(_HERE, 'lab_table.json')
CONF = os.path.join(_HERE, '..', 'docs', 'calibration', 'zed2i_SN35717973.conf')
DEFAULT_CAMERA = {'device': 2, 'width': 1920, 'height': 1080, 'fourcc': 'YUYV',
                  'left_half': True}
# Wrist orientations (degrees about tool0's own x, y, z) the wave steps
# through: without turns the target's offset on the gripper cannot be told
# apart from the camera's position. Neighbours differ by one small step (a
# 35 deg yaw or a 12 deg tilt), and the list closes on itself.
#
# 2026-10-06, first try: turns of +-50 deg yaw and 20 deg tilt, changed
# *while* travelling to the next stop. The second move swung the wrist 100
# deg at once and the arm headed for itself; Nikola stopped it. Hence:
# rotations happen in place, one small step at a time, and every planned
# move is checked joint by joint (``check_move``) before it may run.
# Yaw only (wrist 3 spinning in place: the rest of the arm does not move).
# How far down the tool axis the target hangs is measured with a ruler and
# given to `solve --target-drop` instead of being solved from tilts.
ORIENTATIONS = [(0, 0, 0), (0, 0, 35), (0, 0, 0), (0, 0, -35)]
ARM_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint')
WRIST_JOINTS = ('wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
# Per-move limits: the big joints barely move during a wave, the wrist only
# by about one orientation step (35 deg = 0.61 rad) plus what a translation needs.
MAX_ARM_STEP_RAD = 0.35
MAX_WRIST_STEP_RAD = 0.9
# The approach from the start pose to the first stop is a straight line with
# the tool kept vertical; it may move the big joints further than a wave step.
MAX_APPROACH_ARM_RAD = 1.2
# ... and wrist 3 counter-rotates by as much as the base pans, to keep the
# gripper pointing the same way in the room.
MAX_APPROACH_WRIST_RAD = 1.2
# The held target as the planner should see it: a box under the fingertips
# (tool0 frame; the RG2's closed tips are 0.258 m from the flange). Sized
# for the blue hat held by its brim (about 15 cm across, 10 cm deep).
TARGET_BOX = {'size': (0.17, 0.17, 0.12), 'center_z': 0.258 + 0.06}


def wave_stops(xs, ys, heights):
    """Return [(xyz, orientation, kind)] for the wave.

    Positions snake through the grid (one short step at a time); at each one
    the target is seen in the orientation the arm arrived with, then the
    wrist steps in place to the next orientation and it is seen again. So
    every move is either a pure translation or one small pure rotation.
    """
    stops, k = [], 0
    for xyz in wave_poses(xs, ys, heights):
        stops.append((xyz, ORIENTATIONS[k % len(ORIENTATIONS)], 'move'))
        k += 1
        stops.append((xyz, ORIENTATIONS[k % len(ORIENTATIONS)], 'turn'))
    return stops


def check_move(start, points, arm_limit=MAX_ARM_STEP_RAD, wrist_limit=MAX_WRIST_STEP_RAD):
    """Return (largest arm-joint change, largest wrist-joint change) in rad; raise if too big.

    Measured from the start to *every* waypoint, not just the end: a move
    can swing out and come back.
    """
    arm = max(abs(p[j] - start[j]) for p in points for j in ARM_JOINTS)
    wrist = max(abs(p[j] - start[j]) for p in points for j in WRIST_JOINTS)
    if arm > arm_limit or wrist > wrist_limit:
        raise ValueError(f'refused: arm joints {arm:.2f} rad (limit {arm_limit}), '
                         f'wrist {wrist:.2f} rad (limit {wrist_limit})')
    return arm, wrist


def say(tag, payload):
    import json
    text = payload if isinstance(payload, str) else json.dumps(payload, default=str)
    print(f'{tag:8} {text}', flush=True)


def camera_settings():
    """Camera device settings: the current calibration file's, else the ZED defaults."""
    saved = read_json(CALIBRATION_FILE, {}) or {}
    return dict((saved.get('notes') or {}).get('camera') or DEFAULT_CAMERA)


def lenses():
    """Return (StereoRig with the session's lens tilt, left-eye undistorter, ideal focal)."""
    rig = StereoRig.from_zed_conf(CONF, depth_range=(0.3, 2.5))
    rows = read_json(ROWS, None)
    if rows:
        rig = rig.with_rotation(cv2.Rodrigues(np.array(rows['rotation_rvec']))[0])
    k_left, dist_left, size = zed_factory_calibration(CONF, 'FHD', 'LEFT')
    focal = float(k_left[0, 0])
    shell = mono.TableCalibration(np.eye(4), focal_px=focal, dist=list(dist_left),
                                  camera_matrix=k_left.tolist(), image_size=size)
    return rig, shell, focal


def grab_pair(camera, count=6):
    """Return the newest (left, right) raw RGB pair after flushing the driver's buffer."""
    pair = None
    for _ in range(count):
        latest = camera.read_pair()
        pair = latest if latest is not None else pair
    if pair is None or pair[1] is None:
        raise SystemExit('camera returned no stereo pair (is left_half set?)')
    return pair


def command_rows(args):
    rig = StereoRig.from_zed_conf(CONF, depth_range=(0.3, 2.5))
    camera = Camera(**camera_settings())
    try:
        left, right = grab_pair(camera, 10)
    finally:
        camera.release()
    pixels_left, pixels_right = match_features(left, right)
    refined, report = fit_row_rotation(rig, pixels_left, pixels_right)
    report['stamp'] = time.strftime('%Y-%m-%dT%H:%M:%S')
    os.makedirs(SESSION, exist_ok=True)
    write_json(ROWS, report)
    quality = refined.rectification_error(left, right, step=24, min_texture=20.0,
                                          min_score=0.85, max_dy=40, max_patches=800)
    say('ROWS', {'matches': report['matches'],
                 'row_error_px_before': round(report['before_median_abs_dy'], 2),
                 'row_error_px_after': round(report['after_median_abs_dy'], 2),
                 'p90_after': round(report['after_p90_abs_dy'], 2),
                 'dense_check_px': round(quality['median_abs_dy'], 2),
                 'tilt_change_deg': [round(math.degrees(a - b), 3) for a, b in
                                     zip(report['rotation_rvec'], report['factory_rvec'])]})
    if report['after_median_abs_dy'] > 1.0:
        say('WARN', 'rows still more than a pixel apart: aim at a scene with more texture '
                    'and run rows again before trusting stereo depth')


def tool_matrix(pose):
    """4x4 base_link -> tool0 from a geometry_msgs Pose."""
    from scipy.spatial.transform import Rotation
    q = pose.orientation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return matrix


def make_executor(args):
    import lab_pick
    saved = read_json(CALIBRATION_FILE, {}) or {}
    tips = ((saved.get('notes') or {}).get('touch') or {}).get('tips', {}).get('tool0_z')
    # The planner's table: where the closed fingertips met the plate, one
    # gripper length below the flange (a robot measurement: survives camera moves).
    surface = (tips if tips is not None else args.tips_z) - lab_pick.APPROX_GRIPPER_LENGTH
    executor = lab_pick.PickExecutor(
        None, max_speed_percent=args.max_speed_percent, joint_rate=args.joint_rate,
        max_excursion=2.5, plan_only=not args.execute, table_surface_z=surface)
    executor.load_poses(lab_pick.SEED_POSES_FILE)
    return executor


def attach_target(executor, attach=True):
    """Tell the planner about the held target, or forget it again.

    The planner knows the gripper (a box on the flange, lab_arm_moveit.launch.py)
    but not what it holds: without this, a path may sweep the hanging target
    through the arm or an obstacle and still count as collision-free.
    """
    import lab_pick
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from moveit_msgs.srv import ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive

    executor.ensure_table()
    held = AttachedCollisionObject(link_name='tool0', touch_links=[
        'lab_gripper_envelope', 'tool0', 'flange', 'wrist_3_link'])
    held.object.id = 'lab_held_target'
    held.object.header.frame_id = 'tool0'
    req = ApplyPlanningScene.Request()
    req.scene.is_diff = True
    req.scene.robot_state.is_diff = True
    if attach:
        held.object.operation = CollisionObject.ADD
        pose = lab_pick.Pose()
        pose.position.z, pose.orientation.w = TARGET_BOX['center_z'], 1.0
        held.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                                 dimensions=list(TARGET_BOX['size']))]
        held.object.primitive_poses = [pose]
    else:
        # Detaching alone would leave the box behind in the world.
        held.object.operation = CollisionObject.REMOVE
        gone = CollisionObject(id='lab_held_target', operation=CollisionObject.REMOVE)
        req.scene.world.collision_objects = [gone]
    req.scene.robot_state.attached_collision_objects = [held]
    if not executor.call(ApplyPlanningScene, '/apply_planning_scene', req).success:
        raise SystemExit('the planning scene refused the held-target change')


def with_joints(state, joints):
    """Return a copy of a RobotState with some joints moved (for chained dry runs)."""
    from copy import deepcopy
    moved = deepcopy(state)
    moved.joint_state.position = [float(joints.get(name, value)) for name, value in
                                  zip(moved.joint_state.name, moved.joint_state.position)]
    return moved


def plan_pose_joint(executor, state, pose):
    """IK seeded from ``state``, then a collision-checked straight line in joint space.

    2026-10-06: /compute_cartesian_path turned a 6 mm step from calib_start
    into 2020 waypoints that swung the shoulder a full turn (the gates
    refused it). Wave steps are a few cm, so a joint-space line from the
    nearest IK solution follows nearly the same tool path without that risk.
    """
    from lab_jog import JOINTS, JogError
    from moveit_msgs.srv import GetPositionIK
    from geometry_msgs.msg import PoseStamped
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'ur_manipulator'
    req.ik_request.robot_state = state
    req.ik_request.avoid_collisions = True
    req.ik_request.ik_link_name = 'tool0'
    req.ik_request.pose_stamped = PoseStamped(pose=pose)
    req.ik_request.pose_stamped.header.frame_id = 'base_link'
    req.ik_request.timeout.nanosec = 200_000_000
    res = executor.call(GetPositionIK, '/compute_ik', req)
    if res.error_code.val != 1:
        raise JogError(f'no collision-free IK for the stop (MoveIt code {res.error_code.val})')
    solved = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return executor.plan_joint_line(state, {j: solved[j] for j in JOINTS})


def command_capture(args):
    """Freedrive calibration: the operator moves the arm, Enter records a stop.

    Sends no motion at all (2026-10-06, after the wave's stops: the RG2
    fingers kept meeting the forearm in poses the model passed). Writes the
    same observations file as ``wave``, so ``solve`` works unchanged. Vary
    position across the view AND the tool's tilt/yaw between stops.
    """
    executor = make_executor(args)
    camera = Camera(**camera_settings())
    _, shell, _ = lenses()
    os.makedirs(SESSION, exist_ok=True)
    observations = []
    say('CAPTURE', 'freedrive the arm with the target held; Enter = record, q = finish')
    while True:
        if input(f'[{len(observations)} recorded] Enter to record, q to finish: ').strip() == 'q':
            break
        first = executor.joints_of(executor.fresh(settle_s=0.2))
        time.sleep(0.5)
        state = executor.fresh(settle_s=0.2)
        if max(abs(first[j] - executor.joints_of(state)[j]) for j in first) > 0.002:
            say('SKIP', 'the arm is still moving; hold it still and press Enter again')
            continue
        tool = tool_matrix(executor.fk(state))
        left, right = grab_pair(camera)
        index = len(observations)
        names = (f'capture_{index:02d}_left.jpg', f'capture_{index:02d}_right.jpg')
        for name, image in zip(names, (left, right)):
            cv2.imwrite(os.path.join(SESSION, name), cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_JPEG_QUALITY, 97])
        blobs = white_blobs(shell.undistort(left), color=args.target_color, min_area=300,
                            max_area=80000)
        observations.append({'index': index, 'turn': [0, 0, 0], 'tool0': tool.tolist(),
                             'left': names[0], 'right': names[1]})
        write_json(OBSERVATIONS, {'color': args.target_color, 'observations': observations})
        say('SEEN', {'index': index, 'tool0_xyz': [round(v, 4) for v in tool[:3, 3]],
                     'target_colour_blobs': len(blobs)})
    say('DONE', {'observations': len(observations), 'file': OBSERVATIONS})


def command_wave(args):
    import lab_pick
    from lab_jog import JogError
    from lab_path_audit import check_gap
    from scipy.spatial.transform import Rotation

    executor = make_executor(args)
    camera = Camera(**camera_settings()) if args.execute else None
    _, shell, _ = lenses()
    os.makedirs(SESSION, exist_ok=True)
    observations = []
    attached = False
    try:
        state = executor.fresh()
        if args.from_pose:
            # Plan-only: rehearse the wave from a named pose without going there.
            if args.execute:
                raise SystemExit('--from-pose is for plan-only rehearsals')
            if args.from_pose not in executor.poses:
                raise SystemExit(f'no pose called {args.from_pose}: {sorted(executor.poses)}')
            state = with_joints(state, executor.poses[args.from_pose])
        start = executor.fk(state)
        yaw, tilt = lab_pick.tool_yaw(start)
        if tilt > 3.0:
            raise SystemExit(f'tool is tilted {tilt:.1f} deg; start from a top-down pose '
                             '(console: "go to front")')
        q = start.orientation
        reference = Rotation.from_quat([q.x, q.y, q.z, q.w])
        if not args.no_target_box:
            attach_target(executor)
            attached = True
        stops = wave_stops(args.xs, args.ys, args.heights)
        approached = False
        say('WAVE', {'stops': len(stops), 'turns': not args.no_turns,
                     'start_tool0_xyz': [round(v, 3) for v in (
                         start.position.x, start.position.y, start.position.z)],
                     'target_box': not args.no_target_box, 'execute': bool(args.execute),
                     'limits_rad': {'arm': MAX_ARM_STEP_RAD, 'wrist': MAX_WRIST_STEP_RAD}})
        for index, (xyz, turn, kind) in enumerate(stops):
            if args.no_turns:
                turn = (0, 0, 0)
            quat = (reference * Rotation.from_euler('xyz', turn, degrees=True)).as_quat()
            pose = lab_pick.Pose()
            pose.position.x, pose.position.y, pose.position.z = xyz
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w) = quat
            if args.execute:
                state = executor.fresh()
            try:
                points = plan_pose_joint(executor, state, pose)
                arm, wrist = check_move(
                    executor.joints_of(state), points,
                    *((MAX_ARM_STEP_RAD, MAX_WRIST_STEP_RAD) if approached else
                      (MAX_APPROACH_ARM_RAD, MAX_APPROACH_WRIST_RAD)))
                gap = check_gap(executor, state, points)
                approached = True
            except (JogError, ValueError) as error:
                say('SKIP', {'stop': index, 'kind': kind, 'xyz': list(xyz), 'turn': list(turn),
                             'why': str(error)[:140]})
                if not approached:
                    # Never let a later stop become the approach: it would
                    # travel and turn in one move.
                    say('ABORT', 'the approach to the first stop was refused')
                    break
                continue
            line = {'stop': index, 'kind': kind, 'xyz': list(xyz), 'turn': list(turn),
                    'arm_rad': round(arm, 2), 'wrist_rad': round(wrist, 2),
                    'gripper_arm_gap_m': round(gap, 3)}
            if not args.execute:
                say('PLAN', line)
                state = with_joints(state, points[-1])
                continue
            try:
                executor.execute(state, points, executor.joint_rate)
            except JogError as error:
                # A failed execution is a protective stop or a refusal at the
                # robot: never carry on to the next stop after one.
                say('ABORT', {'stop': index, 'why': str(error)[:160]})
                break
            time.sleep(0.6)
            actual = executor.fk(executor.fresh(settle_s=0.4))
            left, right = grab_pair(camera)
            names = (f'wave_{index:02d}_left.jpg', f'wave_{index:02d}_right.jpg')
            for name, image in zip(names, (left, right)):
                cv2.imwrite(os.path.join(SESSION, name), cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                            [cv2.IMWRITE_JPEG_QUALITY, 97])
            observations.append({'index': index, 'turn': list(turn),
                                 'tool0': tool_matrix(actual).tolist(),
                                 'left': names[0], 'right': names[1]})
            blobs = white_blobs(shell.undistort(left), color=args.target_color, min_area=300,
                                max_area=80000)
            say('SEEN', dict(line, tool0_xyz=[round(v, 4) for v in tool_matrix(actual)[:3, 3]],
                             target_colour_blobs=len(blobs)))
            write_json(OBSERVATIONS, {'color': args.target_color, 'observations': observations})
    finally:
        if attached:
            try:
                attach_target(executor, attach=False)
            except SystemExit as error:
                say('WARN', str(error))
        if camera is not None:
            camera.release()
        executor.destroy_node()
        executor.motion.destroy_node()
    say('DONE', {'observations': len(observations), 'file': os.path.normpath(OBSERVATIONS)})


def target_mask(ideal_rgb, bbox, color):
    """Pixels (Nx2, u v) of the target colour inside a blob's bounding box."""
    _, loose = TARGET_COLORS[color]
    x, y, w, h = bbox
    hsv = cv2.cvtColor(ideal_rgb[y:y + h, x:x + w], cv2.COLOR_RGB2HSV)
    mask = cv2.morphologyEx(cv2.inRange(hsv, loose[0], loose[1]), cv2.MORPH_OPEN,
                            np.ones((5, 5), np.uint8))
    rows, cols = np.nonzero(mask)
    return np.column_stack((cols + x, rows + y)).astype(float)


def measure_targets(data, rig, shell, focal, radius_m=None):
    """Return per observation: dict(centre pixel, surface point, radius estimate) or None."""
    observations = data['observations']
    frames = []
    for obs in observations:
        left = cv2.cvtColor(cv2.imread(os.path.join(SESSION, obs['left'])), cv2.COLOR_BGR2RGB)
        right = cv2.cvtColor(cv2.imread(os.path.join(SESSION, obs['right'])), cv2.COLOR_BGR2RGB)
        ideal = shell.undistort(left)
        obs['blobs'] = white_blobs(ideal, color=data['color'], min_area=300, max_area=80000)
        frames.append((left, right, ideal))
    centers = moving_target(observations)
    found = []
    for obs, center, (left, right, ideal) in zip(observations, centers, frames):
        if center is None:
            found.append(None)
            continue
        blob = min(obs['blobs'], key=lambda b: math.dist(b['center'], center))
        pixels = target_mask(ideal, blob['bbox'], data['color'])
        points = rig.points_for_pixels(left, right, pixels, focal)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) < 30:
            found.append({'center': center, 'stereo_points': int(len(points))})
            continue
        surface = np.median(points, axis=0)
        # A round blob of area A has radius sqrt(A/pi) pixels; at depth Z one
        # pixel is Z/focal metres.
        estimate = math.sqrt(blob['area'] / math.pi) * surface[2] / focal
        found.append({'center': center, 'surface': surface, 'radius_estimate_m': estimate,
                      'stereo_points': int(len(points))})
    radii = [f['radius_estimate_m'] for f in found if f and 'surface' in f]
    radius = radius_m if radius_m is not None else (float(np.median(radii)) if radii else 0.0)
    for f in found:
        if f and 'surface' in f:
            f['centre'] = handeye.sphere_centre(f['surface'], f['surface'], radius)
    return found, radius


def command_solve(args):
    data = read_json(OBSERVATIONS, None)
    if not data:
        raise SystemExit('no observations: run `--execute wave` first')
    rig, shell, focal = lenses()
    found, radius = measure_targets(data, rig, shell, focal, args.radius)
    observations = data['observations']
    usable = [(obs, f) for obs, f in zip(observations, found) if f and 'centre' in f]
    say('TARGETS', {'stops': len(observations), 'with_stereo_centre': len(usable),
                    'radius_mm': round(radius * 1000, 1),
                    'no_detection': [o['index'] for o, f in zip(observations, found) if not f],
                    'no_depth': [o['index'] for o, f in zip(observations, found)
                                 if f and 'centre' not in f]})
    if len(usable) < 8:
        raise SystemExit(f'only {len(usable)} stops have a target with stereo depth; need 8')
    points = np.array([f['centre'] for _, f in usable])
    poses = np.array([obs['tool0'] for obs, _ in usable])
    result = handeye.solve(points, poses, focal_px=rig.focal_px, baseline_m=rig.baseline_m,
                           fit_disparity=not args.no_disparity, target_drop=args.target_drop)
    cam_to_base = result['cam_to_base']
    k = mono.intrinsics(focal, rig.image_size)
    solution = {
        'cam_to_base': cam_to_base.tolist(),
        'target_in_tool': result['target_in_tool'].tolist(),
        'disparity_offset_px': result['disparity_offset_px'],
        'rms_mm': round(result['rms_mm'], 2),
        'residuals_mm': {str(obs['index']): (None if np.isnan(r) else round(float(r), 1))
                         for (obs, _), r in zip(usable, result['residuals_mm'])},
        'sigma': result['sigma'], 'radius_m': radius, 'stops_used': int(result['inliers'].sum()),
        'stops_seen': len(usable), 'focal_px': focal, 'image_size': list(rig.image_size),
        'rows': read_json(ROWS, None), 'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
    write_json(SOLUTION, solution)
    # Contact sheet: detected centre (green) vs where the solved model puts
    # the target (red cross). The cross on the blob is the whole check.
    base_to_cam = np.linalg.inv(cam_to_base)
    tiles = []
    for (obs, f), keep in zip(usable, result['inliers']):
        tool = np.array(obs['tool0'])
        predicted = tool[:3, :3] @ result['target_in_tool'] + tool[:3, 3]
        in_cam = base_to_cam[:3, :3] @ predicted + base_to_cam[:3, 3]
        u, v = (k @ in_cam)[:2] / in_cam[2]
        image = shell.undistort(cv2.imread(os.path.join(SESSION, obs['left'])))
        cx, cy = int(f['center'][0]), int(f['center'][1])
        cv2.circle(image, (cx, cy), 16, (0, 255, 0) if keep else (0, 165, 255), 2)
        cv2.drawMarker(image, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
        x0, y0 = max(0, cx - 160), max(0, cy - 120)
        crop = image[y0:y0 + 240, x0:x0 + 320]
        tile = np.zeros((240, 320, 3), np.uint8)
        tile[:crop.shape[0], :crop.shape[1]] = crop
        cv2.putText(tile, f'{obs["index"]} {solution["residuals_mm"][str(obs["index"])]} mm',
                    (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        tiles.append(tile)
    while len(tiles) % 6:
        tiles.append(np.zeros((240, 320, 3), np.uint8))
    sheet = np.vstack([np.hstack(tiles[i:i + 6]) for i in range(0, len(tiles), 6)])
    cv2.imwrite(os.path.join(SESSION, 'contact_sheet.jpg'), sheet)
    position = cam_to_base[:3, 3]
    say('SOLVED', {'rms_mm': solution['rms_mm'], 'stops_used': solution['stops_used'],
                   'stops_seen': solution['stops_seen'],
                   'camera_xyz_m': [round(float(v), 3) for v in position],
                   'camera_to_plate_m': round(float(np.hypot(*(position[:2] - [0.24, 0.08]))), 2),
                   'target_in_tool_mm': [round(v * 1000, 1) for v in solution['target_in_tool']],
                   'target_sigma_mm': [round(v, 1) for v in
                                       result['sigma']['target_in_tool_mm']],
                   'disparity_offset_px': round(solution['disparity_offset_px'], 2),
                   'disparity_sigma_px': round(result['sigma']['disparity_offset_px'], 2),
                   'residuals_mm': solution['residuals_mm'],
                   'sheet': os.path.normpath(os.path.join(SESSION, 'contact_sheet.jpg'))})
    if solution['rms_mm'] > 5.0 or max(result['sigma']['target_in_tool_mm']) > 10.0:
        say('WARN', 'weak solve: check the contact sheet (crosses off their blobs = wrong '
                    'detections) and that the wrist really turned between stops')


def command_finish(args):
    """Write scripts/lab_table.json in TRUE base_link coordinates.

    The pick code's contract (lab_pick.PickExecutor) is unchanged: the table
    plane plus ``tool_length`` is the tool0 height at which the closed
    fingertips touch the plate (the robot-measured 0.1485). What changes is
    that every number is now a real base_link height: the plate is one
    gripper length below that contact height, and ``tool_length`` is the
    gripper length itself.
    """
    import lab_pick

    solution = read_json(SOLUTION, None)
    if not solution:
        raise SystemExit('no solution: run `solve` first')
    previous = read_json(CALIBRATION_FILE, {}) or {}
    notes_before = previous.get('notes') or {}
    touch = notes_before.get('touch') or {}
    tips = touch.get('tips', {}).get('tool0_z', args.tips_z)
    rig, shell, focal = lenses()
    table = np.eye(4)
    table[2, 3] = tips - lab_pick.APPROX_GRIPPER_LENGTH
    calibration = mono.TableCalibration(
        table, focal_px=focal, dist=list(shell.dist), camera_matrix=shell.camera_matrix,
        camera_to_base=solution['cam_to_base'], image_size=tuple(solution['image_size']),
        residual_mm=[],
        notes={'method': 'stereo eye-to-hand: carried target, wrist turns',
               'tool_length': lab_pick.APPROX_GRIPPER_LENGTH,
               'camera': camera_settings(), 'intrinsics': os.path.basename(CONF),
               'touch': touch or {'tips': {'tool0_z': tips}},
               'stereo': {'conf': os.path.relpath(CONF, os.path.join(_HERE, '..')),
                          'rotation_rvec': (solution.get('rows') or {}).get('rotation_rvec'),
                          'disparity_offset_px': solution['disparity_offset_px'],
                          'focal_px': rig.focal_px, 'baseline_m': rig.baseline_m},
               'handeye': {key: solution[key] for key in
                           ('rms_mm', 'stops_used', 'stops_seen', 'target_in_tool',
                            'sigma', 'radius_m', 'stamp')},
               'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')})
    calibration.save(args.output)
    say('WROTE', {'output': os.path.normpath(args.output),
                  'table_plane_z': round(float(table[2, 3]), 4),
                  'tool_length': lab_pick.APPROX_GRIPPER_LENGTH, 'tips_touch_z': tips,
                  'camera_xyz_m': [round(v, 3) for v in
                                   np.array(solution['cam_to_base'])[:3, 3].tolist()]})


# The arm's joint frames in chain order; lines between their origins run
# along the links. tool0 is the flange face.
CHAIN = ('base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link',
         'wrist_2_link', 'wrist_3_link', 'tool0')


def command_check(args):
    """Draw the robot -- from its joint angles and its own dimensions -- over the camera image.

    If the calibration is right, the drawn skeleton runs along the real arm
    and the drawn target sits on the held object, wherever the arm is. A
    camera that has been bumped shows up at once as a drawing that has slid
    off the robot. 's' saves the frame, 'q' quits.
    """
    import rclpy
    from rclpy.node import Node
    import tf2_ros

    calibration = mono.TableCalibration.load(CALIBRATION_FILE)
    if calibration.camera_to_base is None:
        raise SystemExit(f'{CALIBRATION_FILE} has no camera pose')
    base_to_cam = np.linalg.inv(np.array(calibration.camera_to_base))
    size = tuple(calibration.image_size)
    k = mono.intrinsics(calibration.focal_px, size)
    target = (calibration.notes.get('handeye') or {}).get('target_in_tool')
    plate_z = float(calibration.base_from_board[2, 3])
    node = Node('lab_stereo_check')
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    camera = Camera(**calibration.notes.get('camera', DEFAULT_CAMERA))

    def pixel(point):
        p = base_to_cam[:3, :3] @ np.asarray(point, float) + base_to_cam[:3, 3]
        if p[2] < 0.05:
            return None
        u, v = (k @ p)[:2] / p[2]
        return (int(round(u)), int(round(v))) if abs(u) < 1e4 and abs(v) < 1e4 else None

    window = 'stereo calibration check (s saves, q quits)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 720)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            rgb = camera.read()
            if rgb is None:
                continue
            image = cv2.cvtColor(calibration.undistort(rgb), cv2.COLOR_RGB2BGR)
            # Plate grid, 5 cm squares, over the front plate.
            for value in np.arange(0.10, 0.36, 0.05):
                for line in ([(value, -0.10), (value, 0.25)], [(0.10, value - 0.20),
                                                               (0.35, value - 0.20)]):
                    a, b = (pixel((x, y, plate_z)) for x, y in line)
                    if a and b:
                        cv2.line(image, a, b, (0, 200, 255), 1)
            frames = {}
            for name in CHAIN:
                try:
                    t = buffer.lookup_transform('base_link', name, rclpy.time.Time())
                except tf2_ros.TransformException:
                    continue
                tr, rot = t.transform.translation, t.transform.rotation
                frames[name] = (np.array([tr.x, tr.y, tr.z]),
                                np.array([rot.x, rot.y, rot.z, rot.w]))
            points = [pixel(frames[name][0]) for name in CHAIN if name in frames]
            for a, b in zip(points, points[1:]):
                if a and b:
                    cv2.line(image, a, b, (255, 0, 255), 4)
            for at in points:
                if at:
                    cv2.circle(image, at, 7, (255, 255, 255), -1)
            text = f'{len(frames)}/{len(CHAIN)} robot frames'
            if target is not None and 'tool0' in frames:
                from scipy.spatial.transform import Rotation
                position, quat = frames['tool0']
                at = pixel(Rotation.from_quat(quat).as_matrix() @ np.array(target) + position)
                if at:
                    cv2.drawMarker(image, at, (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
                    text += '; red cross = where the held target should be'
            cv2.putText(image, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5)
            cv2.putText(image, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.imshow(window, image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                path = os.path.join(SESSION, 'check.jpg')
                cv2.imwrite(path, image)
                say('SAVED', os.path.normpath(path))
            if key == ord('q'):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
        node.destroy_node()


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--robot-ip', default='192.168.56.101')
    cli.add_argument('--execute', action='store_true', help='really move (default: plan only)')
    cli.add_argument('--max-speed-percent', type=float, default=50.0)
    cli.add_argument('--joint-rate', type=float, default=0.09)
    cli.add_argument('--tips-z', type=float, default=0.1485,
                     help='tool0 z with closed fingertips on the plate, if no file has it')
    commands = cli.add_subparsers(dest='command', required=True)
    commands.add_parser('rows', help='re-measure the lens tilt from the current view'
                        ).set_defaults(run=command_rows)
    grip = commands.add_parser('grip', help='open or close the jaws (for the hand-over)')
    grip.add_argument('--width', type=float, required=True, help='mm; 0 closes on the object')
    grip.add_argument('--force', type=float, default=10.0)
    grip.set_defaults(run=command_grip)
    wave = commands.add_parser('wave', help='carry the target through the view, turning the wrist')
    # x from 0.27: the held target hangs below the robot's mounting plane, and
    # the platform it is bolted to reaches x = 0.14 (lab_obstacles.yaml).
    wave.add_argument('--xs', type=float, nargs='+', default=[0.27, 0.34],
                      help='tool0 x positions, m, base_link (the plate ends at x ~0.30)')
    wave.add_argument('--ys', type=float, nargs='+', default=[0.0, 0.08, 0.16])
    wave.add_argument('--heights', type=float, nargs='+', default=[0.32, 0.40],
                      help='tool0 heights, m (fingertips on the plate at tool0 0.1485; the '
                           'held target hangs about 0.10 below them)')
    wave.add_argument('--no-turns', action='store_true',
                      help='keep the tool straight down (target offset then unmeasurable)')
    wave.add_argument('--no-target-box', action='store_true',
                      help='do not add the held target to the planning scene')
    wave.add_argument('--from-pose', default='',
                      help='plan-only: rehearse the wave as if starting at this named pose')
    wave.add_argument('--target-color', choices=sorted(TARGET_COLORS), default='blue')
    wave.set_defaults(run=command_wave)
    capture = commands.add_parser('capture', help='freedrive calibration: you move the arm, Enter records')
    capture.add_argument('--target-color', choices=sorted(TARGET_COLORS), default='blue')
    capture.set_defaults(run=command_capture)
    solve = commands.add_parser('solve', help='camera pose from the wave')
    solve.add_argument('--radius', type=float, default=None,
                       help='target radius, m (default: measured from the images)')
    solve.add_argument('--target-drop', type=float, required=True,
                       help='flange face to the target centre along the tool axis, m '
                            '(ruler; the hat by its brim was ~0.312 on 2026-10-02)')
    solve.add_argument('--no-disparity', action='store_true',
                       help='do not estimate a stereo depth correction')
    solve.set_defaults(run=command_solve)
    finish = commands.add_parser('finish', help='write the calibration file')
    finish.add_argument('--output', default=CALIBRATION_FILE)
    finish.set_defaults(run=command_finish)
    commands.add_parser('check', help='live overlay of the robot on the camera image'
                        ).set_defaults(run=command_check)
    args = cli.parse_args()
    needs_ros = args.command in ('wave', 'check', 'capture')
    if needs_ros:
        import rclpy
        rclpy.init()
    try:
        return args.run(args) or 0
    finally:
        if needs_ros:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
