#!/usr/bin/env python3
"""Calibrate a fixed camera against the robot with nothing printed.

The idea ("eye-to-hand calibration with a held object")
-------------------------------------------------------
The robot always knows where its own flange (``tool0``) is. If the gripper
holds something the camera can find easily -- here a white object -- then
every pose of the arm gives one pair: *a pixel* (where the camera sees the
object) and *a 3-D position* (where the robot says the flange is). A dozen
pairs spread through the workspace are enough to solve for the one camera
pose that explains all of them (``solvePnP``; the lens model is already known
from the factory calibration).

Notice what is being measured: the camera learns to answer "where must
``tool0`` be for the gripper to hold an object at this pixel?" -- which is
exactly the question a pick asks. The unknown distance from the flange to the
held object never has to be measured: it is the same in every pair, so it
drops out. (For that to hold, every pose uses the same top-down orientation.)

One more number is needed, the height of the table, and the robot finds it
by feel: ``touchdown`` lowers the held object in small steps until the wrist
force sensor reports contact. Then it lets go -- and now the object lies at a
position the robot knows exactly, so the camera's answer can be checked
without a ruler (``lab_benchmark.py pick --area`` repeats that check on every
run).

Steps (driver + ``lab_arm_moveit.launch.py ik:=kdl`` running, Play pressed)::

    python3 scripts/lab_camera_calibration.py grip --width 75      # open
    #   ... hold the object between the pads, bottom flush with the fingertips
    python3 scripts/lab_camera_calibration.py grip --width 0       # close on it
    python3 scripts/lab_camera_calibration.py wave --execute       # ~4 min
    python3 scripts/lab_camera_calibration.py solve                # camera pose + QA image
    python3 scripts/lab_camera_calibration.py touchdown --pixel U V --execute
    #   -> writes scripts/lab_table.json, releases the object, backs up

The camera must not move afterwards. If it is bumped, run this again.
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
from geometry_msgs.msg import WrenchStamped
import numpy as np
import rclpy

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import lab_pick  # noqa: E402
from lab_jog import JogError  # noqa: E402
from lab_table_calibration import CAMERA_FILE, read_json, write_json  # noqa: E402

from ur7e_perception import monocular as mono  # noqa: E402
from ur7e_perception.camera import Camera  # noqa: E402

SESSION = os.path.join(_HERE, '..', 'log', 'calibration')
OBSERVATIONS = os.path.join(SESSION, 'observations.json')
SOLUTION = os.path.join(SESSION, 'camera_pose.json')
CALIBRATION_FILE = os.path.join(_HERE, 'lab_table.json')


def say(tag, payload):
    text = payload if isinstance(payload, str) else json.dumps(payload, default=str)
    print(f'{tag:8} {text}', flush=True)


def camera_model():
    """Return (camera settings, TableCalibration shell that undistorts) from the saved lens."""
    saved = read_json(CAMERA_FILE, {})
    if not saved.get('focal_px'):
        raise SystemExit('no lens model saved: run lab_table_calibration.py zed|intrinsics first')
    shell = mono.TableCalibration(np.eye(4), focal_px=float(saved['focal_px']),
                                  dist=saved.get('dist', [0.0] * 5),
                                  camera_matrix=saved.get('camera_matrix'))
    return saved, shell


def grab(camera, shell, count=4):
    """Return the newest undistorted frame after flushing the driver's buffer."""
    frame = None
    for _ in range(count):
        latest = camera.read()
        frame = latest if latest is not None else frame
    if frame is None:
        raise SystemExit('camera returned no frame')
    return shell.undistort(frame)


TARGET_COLORS = {
    # colour: (strict HSV range, loose HSV range) -- OpenCV hue 0-179
    'white': (((0, 0, 232), (179, 45, 255)), ((0, 0, 215), (179, 45, 255))),
    'blue': (((100, 150, 70), (130, 255, 255)), ((95, 110, 50), (135, 255, 255))),
    'red': (((0, 150, 80), (8, 255, 255)), ((0, 110, 60), (10, 255, 255))),
    'green': ((((40, 120, 60)), (85, 255, 255)), ((35, 90, 40), (90, 255, 255))),
}


def white_blobs(rgb, min_area=1800, max_area=60000, color='white'):
    """Return compact blobs of the target colour as dicts (centre, area, bbox).

    Two thresholds, found on the lab frames of 2026-10-02. A *strict* one
    (a white plug saturates the sensor, V = 255; aluminium, wall and skin do
    not; a blue toy is far more saturated than the robot's blue-grey caps)
    says which blobs are the real thing; a *looser* one gives each of those
    its full outline, shaded faces included, so the centroid does not jump
    when the lighting on one face changes. The wide morphological opening
    cuts thin bridges (cables, glints) between neighbours.
    """
    strict, loose = TARGET_COLORS[color]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    kernel = np.ones((9, 9), np.uint8)
    core = cv2.morphologyEx(cv2.inRange(hsv, strict[0], strict[1]), cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(cv2.inRange(hsv, loose[0], loose[1]), cv2.MORPH_OPEN, kernel)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    blobs = []
    for index in range(1, count):
        x, y, w, h, area = stats[index]
        if not min_area <= area <= max_area or max(w, h) > 2.2 * min(w, h):
            continue
        if area < 0.5 * w * h:  # ragged or hollow: not a box-like object
            continue
        if np.count_nonzero(core[labels == index]) < 0.25 * min_area:
            continue
        blobs.append({'center': [float(centroids[index][0]), float(centroids[index][1])],
                      'area': int(area), 'bbox': [int(x), int(y), int(w), int(h)]})
    return blobs


def moving_target(observations):
    """Pick, per observation, the white blob that is the held object.

    Two facts single it out. Scenery does not move: a blob found at the same
    pixel in several frames is the wall, a label, the robot's base. And the
    held object hangs *below* the gripper, whose own pale housing is the
    only other white thing that follows the arm -- so of the moving blobs
    the lowest in the image is the object. Mistakes that survive this (a
    hand in the frame, a merged blob) are thrown out later as outliers by
    the pose solver. Returns one centre (or None) per observation.
    """
    chosen = []
    for obs in observations:
        moving = []
        for blob in obs['blobs']:
            repeats = sum(1 for other in observations if other is not obs and any(
                math.dist(blob['center'], b['center']) < 12 for b in other['blobs']))
            if repeats < 2:
                moving.append(blob)
        chosen.append(max(moving, key=lambda b: b['center'][1])['center']
                      if moving else None)
    return chosen


def solve_pose(pixels, points, k):
    """Return (cam_to_base, inlier mask, per-point reprojection error px)."""
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 1, 2)
    points = np.asarray(points, dtype=float).reshape(-1, 1, 3)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        points, pixels, k, None, reprojectionError=12.0, iterationsCount=2000,
        confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers) < 6:
        raise SystemExit('camera pose could not be solved: too few consistent detections')
    keep = inliers.ravel()
    ok, rvec, tvec = cv2.solvePnP(points[keep], pixels[keep], k, None, rvec, tvec,
                                  useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    projected = cv2.projectPoints(points, rvec, tvec, k, None)[0].reshape(-1, 2)
    errors = np.linalg.norm(projected - pixels.reshape(-1, 2), axis=1)
    base_to_cam = np.eye(4)
    base_to_cam[:3, :3] = cv2.Rodrigues(rvec)[0]
    base_to_cam[:3, 3] = tvec.ravel()
    mask = np.zeros(len(points), dtype=bool)
    mask[keep] = True
    return np.linalg.inv(base_to_cam), mask, errors, projected


def wave_poses(xs, ys, heights):
    """Snake through a box grid of tool0 positions (base_link, metres).

    The snake order keeps each move short: one grid step, never a jump back
    across the workspace.
    """
    poses = []
    for layer, z in enumerate(heights):
        plane = []
        for column, x in enumerate(xs):
            line = [(x, y, z) for y in ys]
            plane.extend(line if column % 2 == 0 else line[::-1])
        poses.extend(plane if layer % 2 == 0 else plane[::-1])
    return poses


def make_executor(args):
    touch = (read_json(SOLUTION, None) or {}).get('touch', {})
    surface = None
    if 'tips' in touch:
        # The planner's table: where the fingertips met the plate, one
        # gripper length below the flange. Until then, the base plane.
        surface = touch['tips']['tool0_z'] - lab_pick.APPROX_GRIPPER_LENGTH
    executor = lab_pick.PickExecutor(
        None, max_speed_percent=args.max_speed_percent, joint_rate=args.joint_rate,
        max_excursion=2.5, plan_only=not args.execute, table_surface_z=surface)
    executor.load_poses(lab_pick.SEED_POSES_FILE)
    return executor


def command_grip(args):
    gripper = lab_pick.Rg2(args.robot_ip)
    state = gripper.move(args.width, args.force)
    say('GRIP', {'target_mm': args.width, 'width_mm': round(state['width'], 1),
                 'depth_mm': round(state['depth'], 1),
                 'grip_detected': state['grip_detected']})


def command_wave(args):
    saved, shell = camera_model()
    executor = make_executor(args)
    camera = Camera(**saved['camera'])
    os.makedirs(SESSION, exist_ok=True)
    observations = []
    try:
        start = executor.fk(executor.fresh())
        yaw, tilt = lab_pick.tool_yaw(start)
        if tilt > 3.0:
            raise SystemExit(f'tool is tilted {tilt:.1f} deg; start from a top-down pose')
        targets = wave_poses(args.xs, args.ys, args.heights)
        say('WAVE', {'poses': len(targets), 'yaw_deg': round(math.degrees(yaw), 1),
                     'execute': bool(args.execute)})
        for index, (x, y, z) in enumerate(targets):
            pose = lab_pick.Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation = start.orientation
            try:
                report = executor.move_pose(f'wave_{index:02d}', pose)
            except JogError as error:
                say('SKIP', {'pose': index, 'xyz': [round(v, 3) for v in (x, y, z)],
                             'why': str(error)[:120]})
                if 'did not succeed' in str(error):
                    # The robot itself stopped (2026-10-06: protective stops
                    # C153). Never carry on to the next pose after that.
                    say('ABORT', 'execution failed; stopping the wave')
                    break
                continue
            if not args.execute:
                say('PLAN', {'pose': index, 'xyz': [round(v, 3) for v in (x, y, z)],
                             'seconds': report['nominal_seconds']})
                continue
            time.sleep(0.6)
            actual = executor.fk(executor.fresh(settle_s=0.4)).position
            frame = grab(camera, shell)
            path = os.path.join(SESSION, f'wave_{index:02d}.jpg')
            cv2.imwrite(path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            blobs = white_blobs(frame, color=args.target_color)
            observations.append({'index': index, 'tool0': [actual.x, actual.y, actual.z],
                                 'image': os.path.basename(path), 'blobs': blobs})
            say('SEEN', {'pose': index, 'tool0': [round(v, 4) for v in
                                                  (actual.x, actual.y, actual.z)],
                         'white_blobs': len(blobs)})
            write_json(OBSERVATIONS, {'yaw': yaw, 'size': [frame.shape[1], frame.shape[0]],
                                      'focal_px': shell.focal_px, 'color': args.target_color,
                                      'observations': observations})
    finally:
        camera.release()
        executor.destroy_node()
        executor.motion.destroy_node()
    say('DONE', {'observations': len(observations), 'file': os.path.normpath(OBSERVATIONS)})


def command_solve(args):
    data = read_json(OBSERVATIONS, None)
    if not data:
        raise SystemExit('no observations: run `wave --execute` first')
    observations = data['observations']
    for obs in observations:
        # Re-detect from the saved frames, so a detector fix needs no new motion.
        image = cv2.imread(os.path.join(SESSION, obs['image']))
        obs['blobs'] = white_blobs(cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
                                   color=data.get('color', 'white'))
    centers = moving_target(observations)
    overrides = read_json(os.path.join(SESSION, 'overrides.json'), {})
    for key, value in overrides.items():  # {"7": [u, v]} or {"7": null} to drop a pose
        for position, obs in enumerate(observations):
            if obs['index'] == int(key):
                centers[position] = value
    used = [(obs, center) for obs, center in zip(observations, centers) if center]
    if len(used) < 8:
        raise SystemExit(f'only {len(used)} poses have a usable detection; need 8')
    k = mono.intrinsics(data['focal_px'], tuple(data['size']))
    cam_to_base, inliers, errors, projected = solve_pose(
        [center for _, center in used], [obs['tool0'] for obs, _ in used], k)
    rms = float(np.sqrt(np.mean(errors[inliers] ** 2)))
    position = cam_to_base[:3, 3]
    down = math.degrees(math.asin(max(-1.0, min(1.0, -cam_to_base[2, 2]))))
    result = {'camera_to_base': cam_to_base.tolist(), 'focal_px': data['focal_px'],
              'size': data['size'], 'yaw': data['yaw'], 'poses_used': int(inliers.sum()),
              'poses_seen': len(used), 'rms_px': round(rms, 2),
              'max_inlier_px': round(float(errors[inliers].max()), 2),
              'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
    # A touchdown measures the robot against the table, not the camera, so
    # it stays valid across camera re-solves (a bumped camera, for instance).
    previous = read_json(SOLUTION, {}) or {}
    if previous.get('touch'):
        result['touch'] = previous['touch']
    write_json(SOLUTION, result)
    # Contact sheet: one crop per pose, detection (green) and model (red cross).
    tiles = []
    for (obs, center), good, model in zip(used, inliers, projected):
        image = cv2.imread(os.path.join(SESSION, obs['image']))
        cx, cy = int(center[0]), int(center[1])
        cv2.circle(image, (cx, cy), 14, (0, 255, 0) if good else (0, 165, 255), 2)
        cv2.drawMarker(image, (int(model[0]), int(model[1])), (0, 0, 255),
                       cv2.MARKER_CROSS, 24, 2)
        x0, y0 = max(0, cx - 160), max(0, cy - 120)
        crop = image[y0:y0 + 240, x0:x0 + 320]
        tile = np.zeros((240, 320, 3), np.uint8)
        tile[:crop.shape[0], :crop.shape[1]] = crop
        cv2.putText(tile, f'{obs["index"]}', (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)
        tiles.append(tile)
    while len(tiles) % 6:
        tiles.append(np.zeros((240, 320, 3), np.uint8))
    sheet = np.vstack([np.hstack(tiles[i:i + 6]) for i in range(0, len(tiles), 6)])
    cv2.imwrite(os.path.join(SESSION, 'contact_sheet.jpg'), sheet)
    say('SOLVED', {'rms_px': result['rms_px'], 'max_inlier_px': result['max_inlier_px'],
                   'poses_used': result['poses_used'], 'poses_seen': result['poses_seen'],
                   'camera_xyz': [round(float(v), 3) for v in position],
                   'looking_down_deg': round(down, 1),
                   'errors_px': [round(float(e), 1) for e in errors],
                   'sheet': os.path.normpath(os.path.join(SESSION, 'contact_sheet.jpg'))})


class ForceProbe:
    """Average wrist force over a short window (driver's F/T broadcaster)."""

    def __init__(self, node):
        self.node, self.samples = node, []
        node.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench',
                                 lambda m: self.samples.append(
                                     (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)), 50)

    def read(self, seconds=0.4):
        self.samples.clear()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if len(self.samples) < 20:
            raise JogError('no wrist force data; is the driver running?')
        return np.mean(self.samples, axis=0)


def command_touchdown(args):
    solution = read_json(SOLUTION, None)
    if not solution:
        raise SystemExit('no camera pose: run `solve` first')
    saved, _ = camera_model()
    cam_to_base = np.array(solution['camera_to_base'])
    k = mono.intrinsics(solution['focal_px'], tuple(solution['size']))
    executor = make_executor(args)
    probe = ForceProbe(executor)
    try:
        start = executor.fk(executor.fresh())
        if args.kind == 'tips':
            # Closed jaws are the gripper's longest state: the reference length.
            closed = lab_pick.Rg2(args.robot_ip).move(0.0, 10.0)
            say('GRIP', {'closed_width_mm': round(closed['width'], 1),
                         'depth_mm': round(closed['depth'], 1)})
        if args.xy:
            x, y = args.xy
        elif args.pixel:
            # Where a held object would be on a plane a little above the table.
            point = mono.rays_to_plane([args.pixel], k, cam_to_base,
                                       [0, 0, args.guess_z], [0, 0, 1.0])[0]
            x, y = float(point[0]), float(point[1])
        else:
            x, y = start.position.x, start.position.y
        say('TARGET', {'xy': [round(x, 4), round(y, 4)], 'start_z': args.start_z,
                       'floor_z': args.floor_z})

        def pose_at(z):
            pose = lab_pick.Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation = start.orientation
            return pose

        hover = pose_at(max(args.start_z, args.travel_z))
        # A purely vertical move is checked by the force probe, not the
        # planner: the gripper envelope is already "in" the planner's table
        # whenever the fingertips are near the real one.
        vertical = math.hypot(start.position.x - x, start.position.y - y) < 0.02
        say('MOVE', executor.move_pose('above', hover, avoid_collisions=not vertical))
        say('MOVE', executor.move_pose('start', pose_at(args.start_z), avoid_collisions=False))
        if not args.execute:
            say('PLAN', 'plan-only: stopping before the descent')
            return
        baseline = probe.read()
        z, contact = args.start_z, None
        while z - args.step >= args.floor_z:
            z -= args.step
            executor.move_pose('probe', pose_at(z), avoid_collisions=False, rate=0.03)
            force = float(np.linalg.norm(probe.read() - baseline))
            say('PROBE', {'tool0_z': round(z, 4), 'force_change_n': round(force, 2)})
            if force > args.contact_force:
                contact = z
                break
        if contact is None:
            raise SystemExit(f'no contact down to z={args.floor_z}; raise --start-z/--floor-z '
                             'deliberately if the table is lower')
        # Contact was felt between this step and the previous one.
        touch_z = contact + args.step / 2
        solution.setdefault('touch', {})[args.kind] = {
            'tool0_z': touch_z, 'xy': [x, y], 'force_n': round(force, 2),
            'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
        write_json(SOLUTION, solution)
        say('TOUCH', {'kind': args.kind, 'tool0_z_at_contact': round(touch_z, 4)})
        # Take the load off before anything else: back up one step.
        executor.move_pose('unload', pose_at(touch_z + 0.004), avoid_collisions=False,
                           rate=0.03)
        if args.kind == 'object':
            state = lab_pick.Rg2(args.robot_ip).move(args.release_width, 10.0)
            say('GRIP', {'released_at_xy': [round(x, 4), round(y, 4)],
                         'width_mm': round(state['width'], 1)})
        say('MOVE', executor.move_pose('back_up', pose_at(args.travel_z),
                                       avoid_collisions=False))
    finally:
        executor.destroy_node()
        executor.motion.destroy_node()


def command_finish(args):
    """Write the calibration file from the camera pose and the two touchdowns.

    Heights live in the frame the wave defined, where a point's z is "the
    tool0 height at which the gripper would hold the calibration object's
    centre there". In that frame:

    * the table surface is the object touchdown height minus the distance
      from the object's tracked centre to its lowest point (as it was held);
    * ``tool_length`` is how far above that surface tool0 is when the closed
      fingertips rest on the table -- measured directly by the tips touchdown.
    """
    solution = read_json(SOLUTION, None)
    touch = (solution or {}).get('touch', {})
    if 'tips' not in touch or ('object' not in touch and args.table_z is None):
        raise SystemExit('need `touchdown --kind tips` and either `--kind object` or --table-z')
    saved, _ = camera_model()
    table = np.eye(4)
    table[2, 3] = (args.table_z if args.table_z is not None
                   else touch['object']['tool0_z'] - args.center_to_bottom)
    tool_length = touch['tips']['tool0_z'] - table[2, 3]
    calibration = mono.TableCalibration(
        table, focal_px=saved['focal_px'], dist=saved.get('dist', [0.0] * 5),
        camera_matrix=saved.get('camera_matrix'),
        camera_to_base=solution['camera_to_base'], image_size=tuple(solution['size']),
        residual_mm=[],
        notes={'method': 'held-object wave + force touchdowns',
               'tool_length': float(tool_length), 'camera': saved.get('camera'),
               'intrinsics': saved.get('source'), 'grasp_yaw': solution['yaw'],
               'touch': touch, 'center_to_bottom_m': args.center_to_bottom,
               'wave': {key: solution[key] for key in
                        ('rms_px', 'max_inlier_px', 'poses_used', 'poses_seen')},
               'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')})
    calibration.save(args.output)
    say('WROTE', {'output': os.path.normpath(args.output),
                  'table_plane_z': round(float(table[2, 3]), 4),
                  'tool_length': round(float(tool_length), 4),
                  'tips_touch_z': round(touch['tips']['tool0_z'], 4)})


def command_map(args):
    """Save a camera frame with the robot's XY grid drawn on a horizontal plane.

    The quickest way to see whether a calibration is sane, and to read off
    base_link coordinates of things on the table (obstacles, a free spot).
    """
    solution = read_json(SOLUTION, None)
    if not solution:
        raise SystemExit('no camera pose: run `solve` first')
    saved, shell = camera_model()
    camera = Camera(**saved['camera'])
    try:
        frame = grab(camera, shell, count=8)
    finally:
        camera.release()
    cam_to_base = np.array(solution['camera_to_base'])
    k = mono.intrinsics(solution['focal_px'], tuple(solution['size']))
    image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    height, width = image.shape[:2]

    def pixel(x, y):
        point = mono.project([[x, y, args.plane_z]], k, cam_to_base)[0]
        behind = (np.linalg.inv(cam_to_base) @ [x, y, args.plane_z, 1.0])[2] <= 0.05
        return None if behind else (int(point[0]), int(point[1]))

    ticks = np.arange(-0.2, 0.71, 0.1)
    for value in ticks:
        for line in ([(value, t) for t in np.arange(-0.2, 0.71, 0.02)],
                     [(t, value) for t in np.arange(-0.2, 0.71, 0.02)]):
            points = [pixel(x, y) for x, y in line]
            for a, b in zip(points, points[1:]):
                if a and b and all(-2000 < c < 4000 for c in a + b):
                    cv2.line(image, a, b, (0, 255, 255), 1)
    for x in ticks:
        for y in ticks:
            at = pixel(x, y)
            if at and 0 <= at[0] < width and 0 <= at[1] < height:
                cv2.putText(image, f'{x:.1f},{y:.1f}', (at[0] + 3, at[1] - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
                cv2.putText(image, f'{x:.1f},{y:.1f}', (at[0] + 3, at[1] - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    os.makedirs(SESSION, exist_ok=True)
    path = os.path.join(SESSION, 'map.jpg')
    cv2.imwrite(path, image)
    say('MAP', {'image': os.path.normpath(path), 'plane_z': args.plane_z})


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--robot-ip', default='192.168.56.101')
    cli.add_argument('--execute', action='store_true', help='really move (default: plan only)')
    cli.add_argument('--max-speed-percent', type=float, default=50.0)
    cli.add_argument('--joint-rate', type=float, default=0.09)
    commands = cli.add_subparsers(dest='command', required=True)
    grip = commands.add_parser('grip', help='open or close the jaws (for the hand-over)')
    grip.add_argument('--width', type=float, required=True, help='mm; 0 closes on the object')
    grip.add_argument('--force', type=float, default=10.0)
    grip.set_defaults(run=command_grip)
    wave = commands.add_parser('wave', help='move the held object through the workspace')
    # Defaults: the clear front plate of the lab stand, in front of the two
    # toggle clamps (2026-10-02). The lowest layer carries the held object a
    # few centimetres above the table, so keep the grid off anything taller.
    wave.add_argument('--xs', type=float, nargs='+', default=[0.19, 0.24, 0.29],
                      help='tool0 x positions, m, base_link (the plate ends at x ~0.30)')
    wave.add_argument('--ys', type=float, nargs='+', default=[-0.02, 0.08, 0.18],
                      help='tool0 y positions, m, base_link')
    wave.add_argument('--heights', type=float, nargs='+', default=[0.27, 0.36, 0.45],
                      help='tool0 heights, m (fingertips are about 0.26 m lower; the '
                           'plate is at tool0 0.1485)')
    wave.add_argument('--target-color', choices=sorted(TARGET_COLORS), default='blue',
                      help='colour of the held object')
    wave.set_defaults(run=command_wave)
    solve = commands.add_parser('solve', help='camera pose from the wave observations')
    solve.set_defaults(run=command_solve)
    grid = commands.add_parser('map', help='draw the robot XY grid on a camera frame')
    grid.add_argument('--plane-z', type=float, default=0.26,
                      help='height of the plane to draw on, in the calibration frame '
                           '(the table is about one gripper length above zero)')
    grid.set_defaults(run=command_map)
    touch = commands.add_parser('touchdown', help='find the table by force, then let go')
    touch.add_argument('--kind', choices=('object', 'tips'), default='object',
                       help='object: lower the held object, then release it; '
                            'tips: lower the empty, closed fingertips')
    touch.add_argument('--xy', type=float, nargs=2, metavar=('X', 'Y'),
                       help='base_link point to come down at (default: straight down)')
    touch.add_argument('--pixel', type=float, nargs=2, metavar=('U', 'V'),
                       help='or an image point to come down at')
    touch.add_argument('--guess-z', type=float, default=0.245,
                       help='rough tool0 height with the object on the table, m')
    touch.add_argument('--travel-z', type=float, default=0.40)
    touch.add_argument('--start-z', type=float, default=0.22,
                       help='tool0 height where the slow probing starts, m')
    touch.add_argument('--floor-z', type=float, default=0.10,
                       help='never probe below this tool0 height, m (the lab plate was '
                            'found at 0.1485)')
    touch.add_argument('--step', type=float, default=0.003)
    touch.add_argument('--contact-force', type=float, default=5.0, help='N')
    touch.add_argument('--release-width', type=float, default=80.0)
    touch.set_defaults(run=command_touchdown)
    finish = commands.add_parser('finish', help='write the calibration file')
    finish.add_argument('--center-to-bottom', type=float, default=0.032,
                        help='tracked centre of the held object to its lowest point, m')
    finish.add_argument('--table-z', type=float, default=None,
                        help='override the table plane height (calibration frame), m')
    finish.add_argument('--output', default=CALIBRATION_FILE)
    finish.set_defaults(run=command_finish)
    args = cli.parse_args()
    needs_ros = args.command in ('wave', 'touchdown')
    if needs_ros:
        rclpy.init()
    try:
        args.run(args)
    except JogError as error:
        say('BLOCKED', str(error))
        return 1
    finally:
        if needs_ros and rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
