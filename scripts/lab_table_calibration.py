#!/usr/bin/env python3
"""Calibrate one webcam against the robot using a printed board on the table.

This replaces task 2.5's eye-to-hand procedure for a session without the
depth camera. The idea, in the order you run it:

``print``
    Write the board PDF. Print at 100 %, tape it flat on the table where the
    camera sees it and the arm reaches it. Measure five squares (175 mm).

``zed --conf FILE``  or  ``intrinsics``
    Tell the software about the lens. For a ZED used as a plain USB camera,
    ``zed`` reads the factory calibration (download it once from
    https://calib.stereolabs.com/?SN=<serial>). For any other webcam,
    ``intrinsics`` measures the focal length: move the camera slowly for
    ~20 s so the board is seen from different angles and distances, and the
    script solves for the focal length that explains all the views. Then put
    the camera where it will stay.

``aim``
    A live view in the browser (http://localhost:8089) for placing the
    camera: it marks the board corners it finds and says how far away and
    how steeply the camera looks down. Aim for 30 degrees or more.

``touch --corner N``  (N = 1..4, the numbers printed at the board's corners)
    Tell the robot where the board is. Close the gripper, jog the fingertips
    onto corner N (pendant Move tab, RG2 straight down: the flange tilts ~60 deg), run this.
    It reads tool0 from TF and stores the fingertip point; if the jaws are
    not fully closed it asks the gripper how much shorter it is at that
    width and corrects for it. No motion is commanded: the script only
    listens.

``fit``
    Solve board -> base_link from the four touches and write the calibration
    file the perception node loads. The residual it prints is how well four
    hand-placed touches agree with a rigid 175 x 245 mm rectangle: under
    3 mm is good, over 10 mm is refused.

``check``
    Look through the camera with the finished calibration and save an
    annotated picture: the base_link axes drawn on the table, the reachable
    region tinted. If the axes sit where the robot's base is, it worked.

Why touches at all? The camera can measure where the board is relative to
*itself*; only the robot can say where the board is relative to *the robot*.
Four touches join the two. Everything after that is geometry.
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'ur7e_perception'))
from ur7e_perception import monocular as mono  # noqa: E402
from ur7e_perception.camera import Camera, ZED_MODES, zed_factory_calibration  # noqa: E402
from ur7e_perception.core import PerceptionError  # noqa: E402

CAMERA_FILE = os.path.join(_HERE, '.lab_camera.json')
TOUCH_FILE = os.path.join(_HERE, '.lab_touches.json')
CALIBRATION_FILE = os.path.join(_HERE, 'lab_table.json')
DOCS = os.path.join(_HERE, '..', 'docs', 'calibration')


def read_json(path, default):
    if os.path.exists(path):
        with open(path) as stream:
            return json.load(stream)
    return default


def write_json(path, data):
    with open(path, 'w') as stream:
        json.dump(data, stream, indent=2)
        stream.write('\n')


def camera_settings(args):
    """Camera settings: the saved ones (``zed``/``intrinsics``), then CLI overrides."""
    settings = dict(device=0, width=1280, height=720, fourcc='MJPG', left_half=False)
    settings.update(read_json(CAMERA_FILE, {}).get('camera') or {})
    if args.device is not None:
        settings['device'] = args.device
    return settings


def grab_frames(args, seconds, every=0.4):
    """Return RGB frames sampled over ``seconds`` from the camera."""
    try:
        camera = Camera(**camera_settings(args))
    except PerceptionError as error:
        raise SystemExit(f'{error} (is the perception node using it?)')
    frames, last, end = [], 0.0, time.time() + seconds
    try:
        for _ in range(10):  # let auto-exposure settle
            camera.read()
        while time.time() < end or not frames:
            frame = camera.read()
            if frame is not None and time.time() - last >= every:
                frames.append(frame)
                last = time.time()
    finally:
        camera.release()
    if float(frames[-1].std()) < 2.0:
        raise SystemExit('camera image is blank: open the privacy shutter')
    return frames


def saved_calibration_shell():
    """A TableCalibration holding only the camera model (for undistorting)."""
    camera = read_json(CAMERA_FILE, {})
    return mono.TableCalibration(
        np.eye(4), focal_px=float(camera.get('focal_px', 0.0)),
        dist=camera.get('dist', [0.0] * 5), camera_matrix=camera.get('camera_matrix'))


def command_zed(args):
    """Use the ZED's factory calibration instead of calibrating by hand."""
    matrix, dist, size = zed_factory_calibration(args.conf, args.mode)
    settings = dict(device=args.device if args.device is not None else 2,
                    width=size[0], height=size[1], fourcc='YUYV', left_half=True)
    write_json(CAMERA_FILE, dict(
        focal_px=float(matrix[0, 0]), dist=dist, camera_matrix=matrix.tolist(),
        image_size=list(size), camera=settings, source=os.path.basename(args.conf),
        square_mm=args.square_mm, stamp=time.strftime('%Y-%m-%dT%H:%M:%S')))
    fov = math.degrees(2 * math.atan(size[0] / 2 / matrix[0, 0]))
    print(json.dumps(dict(focal_px=round(float(matrix[0, 0]), 1),
                          horizontal_fov_deg=round(fov, 1), camera=settings,
                          distortion_terms=len(dist))))


def command_print(args):
    from PIL import Image
    os.makedirs(DOCS, exist_ok=True)
    pages = [Image.fromarray(page) for page in mono.printable_sheets()]
    path = os.path.join(DOCS, 'table_board_letter.pdf')
    pages[0].save(path, 'PDF', resolution=300.0, save_all=True, append_images=pages[1:])
    print(f'wrote {os.path.normpath(path)}: print at 100 %, measure 5 squares = 175 mm')


def command_intrinsics(args):
    board = mono.table_board(args.square_mm / 1000, args.square_mm / 1000 * 26 / 35)
    print(f'capturing for {args.seconds:.0f} s: move the camera around the board...')
    settings = camera_settings(args)
    frames = grab_frames(args, args.seconds)
    size = (frames[0].shape[1], frames[0].shape[0])
    focal, dist, rms, used = mono.calibrate_intrinsics(frames, board, size)
    write_json(CAMERA_FILE, dict(focal_px=focal, dist=dist, image_size=list(size),
                                 camera=settings, rms_px=rms, views=used,
                                 square_mm=args.square_mm,
                                 stamp=time.strftime('%Y-%m-%dT%H:%M:%S')))
    fov = math.degrees(2 * math.atan(size[0] / 2 / focal))
    print(json.dumps(dict(focal_px=round(focal, 1), horizontal_fov_deg=round(fov, 1),
                          k1=round(dist[0], 4), rms_px=round(rms, 3), views=used)))


def tool_pose(samples=10, timeout=5.0):
    """Return base_link -> tool0 as a 4x4, averaged, from the driver's TF."""
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node('lab_table_touch')
    buffer = Buffer()
    TransformListener(buffer, node)
    poses, deadline = [], time.monotonic() + timeout
    try:
        while len(poses) < samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not buffer.can_transform('base_link', 'tool0', Time()):
                continue
            t = buffer.lookup_transform('base_link', 'tool0', Time()).transform
            q = t.rotation
            rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
            poses.append((np.array([t.translation.x, t.translation.y, t.translation.z]),
                          rotation))
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if len(poses) < samples:
        raise SystemExit('no base_link -> tool0 transform: is the driver running?')
    positions = np.array([p for p, _ in poses])
    if np.ptp(positions, axis=0).max() > 0.0005:
        raise SystemExit('the arm is moving; let it settle and run this again')
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = poses[-1][1], positions.mean(axis=0)
    return transform


def quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def command_touch(args):
    pose = tool_pose()
    # The RG2's fingers swing on a parallelogram: the wider the jaws, the
    # shorter the gripper. It reports that shortening ("depth", mm) itself,
    # so a touch made with open jaws is corrected to the closed-jaw length
    # that every later height is expressed in.
    depth_mm, width_mm = 0.0, None
    try:
        import socket
        import xmlrpc.client
        socket.setdefaulttimeout(3.0)  # an unreachable robot must not hang a touch
        state = xmlrpc.client.ServerProxy(
            f'http://{args.robot_ip}:41414/').rg_get_all_variables(args.tool_index)
        depth_mm, width_mm = float(state['depth']), float(state['width'])
        if not (0 <= depth_mm <= 30 and 0 <= width_mm <= 110):
            raise ValueError(f'implausible gripper state: depth {depth_mm}, width {width_mm}')
    except Exception as error:  # noqa: B902 - any failure means "assume closed"
        depth_mm, width_mm = 0.0, None
        print(f'NOTE: could not read the gripper ({error}); assuming closed jaws')
    # Fingertip from the real tool stack (lab_tooling.py): the RG2 sits 60 deg
    # off tool0 Z on the Dual Quick Changer, so the tips are NOT on the flange
    # axis. Open jaws are shorter by ``depth`` along the RG2's own axis.
    import lab_tooling
    tips = np.asarray(pose) @ lab_tooling.tool0_to_tips()
    tip = tips[:3, 3] - tips[:3, 2] * depth_mm / 1000
    # Tilt of the RG2's own axis, not the flange's: with the changer the
    # flange is ~60 deg off vertical whenever the RG2 points straight down.
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -tips[2, 2]))))
    touches = read_json(TOUCH_FILE, {})
    touches[str(args.corner)] = dict(tip=tip.tolist(), tool0=pose.tolist(),
                                     tilt_deg=tilt, tool_length=args.tool_length,
                                     jaw_width_mm=width_mm, jaw_depth_mm=depth_mm)
    write_json(TOUCH_FILE, touches)
    print(json.dumps(dict(corner=args.corner, tip_xyz=[round(v, 4) for v in tip],
                          tilt_deg=round(tilt, 1), jaw_width_mm=width_mm,
                          jaw_depth_mm=depth_mm, recorded=sorted(touches))))
    if tilt > 5:
        print(f'WARNING: the RG2 is tilted {tilt:.1f} deg from straight down; the fingertip '
              'point then depends on tcp_to_tips_m being exact. Prefer a vertical RG2.')


def command_fit(args):
    touches = read_json(TOUCH_FILE, {})
    missing = [n for n in '1234' if n not in touches]
    if missing:
        raise SystemExit(f'corners not touched yet: {", ".join(missing)}')
    square = args.square_mm / 1000
    tips = np.array([touches[n]['tip'] for n in '1234'])
    try:
        base_from_board, residual = mono.fit_board_anchor(tips, square)
    except PerceptionError as error:
        sides = [np.linalg.norm(tips[i] - tips[(i + 1) % 4]) * 1000 for i in range(4)]
        raise SystemExit(f'{error}. Measured side lengths {np.round(sides, 1)} mm; expected '
                         f'{mono.COLUMNS * args.square_mm:g}, {mono.ROWS * args.square_mm:g}, '
                         f'{mono.COLUMNS * args.square_mm:g}, {mono.ROWS * args.square_mm:g}. '
                         'Re-touch the corner that disagrees.')
    camera = read_json(CAMERA_FILE, {})
    calibration = mono.TableCalibration(
        base_from_board, square_m=square, marker_m=square * 26 / 35,
        focal_px=float(camera.get('focal_px', 0.0)),
        dist=camera.get('dist', [0.0] * 5), camera_matrix=camera.get('camera_matrix'),
        image_size=tuple(camera.get('image_size', (1280, 720))),
        residual_mm=[round(float(r) * 1000, 2) for r in residual],
        notes=dict(stamp=time.strftime('%Y-%m-%dT%H:%M:%S'), touches=touches,
                   camera=camera.get('camera'),
                   intrinsics=camera.get('source') or camera.get('stamp')
                   or 'single-view estimate per capture'))
    calibration.save(args.output)
    normal = base_from_board[:3, 2]
    print(json.dumps(dict(
        output=os.path.normpath(args.output),
        residual_mm=calibration.residual_mm,
        table_z_m=round(float(base_from_board[2, 3]), 4),
        board_origin_xyz=[round(float(v), 4) for v in base_from_board[:3, 3]],
        table_tilt_deg=round(math.degrees(math.acos(min(1.0, abs(normal[2])))), 2),
        focal_px=round(calibration.focal_px, 1))))
    if not calibration.focal_px:
        print('NOTE: no intrinsics step was run; focal length will be estimated from each '
              'image (looser). Run `intrinsics` for better accuracy away from the board.')


def command_check(args):
    calibration = mono.TableCalibration.load(args.calibration)
    frames = [calibration.undistort(f) for f in grab_frames(args, 1.5, every=0.2)]
    view = mono.observe_board(frames, calibration.board(),
                              focal_px=calibration.focal_px or None)
    cam_to_base = mono.camera_to_base(view, calibration.base_from_board)
    anchor = calibration.base_from_board
    image = frames[-1].copy()
    # Tint the part of the table the arm can reach.
    ys, xs = np.mgrid[0:image.shape[0]:6, 0:image.shape[1]:6]
    points = mono.rays_to_plane(np.column_stack((xs.ravel(), ys.ravel())), view.k,
                                cam_to_base, anchor[:3, 3], anchor[:3, 2])
    reach = np.hypot(points[:, 0], points[:, 1]).reshape(ys.shape)
    tint = cv2.resize(((reach > 0.2) & (reach < 0.8)).astype(np.uint8),
                      (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
    image[tint > 0] = (image[tint > 0] * 0.75 + np.array([0, 90, 255]) * 0.25).astype(np.uint8)
    # base_link axes at the table point under the base origin, 10 cm long.
    table_z = anchor[2, 3]
    axes = np.array([[0, 0, table_z], [0.1, 0, table_z], [0, 0.1, table_z],
                     [0, 0, table_z + 0.1]])
    try:
        pixels = mono.project(axes, view.k, cam_to_base).astype(int)
        for end, color in ((1, (255, 0, 0)), (2, (0, 255, 0)), (3, (0, 0, 255))):
            cv2.line(image, tuple(pixels[0]), tuple(pixels[end]), color, 3)
    except (ValueError, OverflowError):
        pass
    corners = mono.touch_points(calibration.square_m)
    corner_px = mono.project(corners @ anchor[:3, :3].T + anchor[:3, 3], view.k, cam_to_base)
    for number, pixel in enumerate(corner_px.astype(int), start=1):
        cv2.circle(image, tuple(pixel), 6, (255, 255, 0), 2)
        cv2.putText(image, str(number), (pixel[0] + 8, pixel[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    os.makedirs(os.path.dirname(os.path.abspath(args.image)), exist_ok=True)
    cv2.imwrite(args.image, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    camera_position = cam_to_base[:3, 3]
    print(json.dumps(dict(
        image=os.path.normpath(args.image), board_corners_seen=view.corners,
        reprojection_rms_px=round(view.rms_px, 2), focal_px=round(float(view.k[0, 0]), 1),
        camera_xyz_base_m=[round(float(v), 3) for v in camera_position],
        camera_height_above_table_m=round(float(camera_position[2] - table_z), 3))))


def command_look(args):
    """Save one frame and say whether the board is visible (no touches needed)."""
    shell = saved_calibration_shell()
    frames = [shell.undistort(f) for f in grab_frames(args, 1.0, every=0.2)]
    os.makedirs(os.path.dirname(os.path.abspath(args.image)), exist_ok=True)
    cv2.imwrite(args.image, cv2.cvtColor(frames[-1], cv2.COLOR_RGB2BGR))
    board = mono.table_board(args.square_mm / 1000, args.square_mm / 1000 * 26 / 35)
    report = dict(image=os.path.normpath(args.image),
                  board_corners_seen=len(mono.detect_corners(frames[-1], board)),
                  size=[frames[-1].shape[1], frames[-1].shape[0]])
    try:
        view = mono.observe_board(frames, board, focal_px=shell.focal_px or None)
        position = view.board_from_camera[:3, 3]
        report.update(reprojection_rms_px=round(view.rms_px, 2),
                      camera_height_above_board_m=round(float(position[2]), 3),
                      camera_distance_to_board_m=round(float(np.linalg.norm(position)), 3))
    except PerceptionError as error:
        report['board'] = str(error)
    print(json.dumps(report))


def command_aim(args):
    """Live view for placing the camera: shows what it sees and whether the board is found."""
    shell = saved_calibration_shell()
    board = mono.table_board(args.square_mm / 1000, args.square_mm / 1000 * 26 / 35)
    try:
        camera = Camera(**camera_settings(args))
    except PerceptionError as error:
        raise SystemExit(f'{error} (is the perception node using it?)')
    latest = {}
    server = serve_mjpeg(args.port, latest)
    print(f'open http://localhost:{args.port} in a browser; Ctrl-C to stop', flush=True)
    end = time.time() + args.seconds
    try:
        while time.time() < end:
            frame = camera.read()
            if frame is None:
                continue
            frame = shell.undistort(frame)
            found = mono.detect_corners(frame, board)
            view = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            for pixel in found.values():
                cv2.circle(view, (int(pixel[0]), int(pixel[1])), 6, (0, 255, 0), 2)
            text, color = f'board corners: {len(found)}/24', (0, 0, 255)
            if len(found) >= 8:
                try:
                    pose = mono.observe_board(frame, board, focal_px=shell.focal_px or None)
                    position = pose.board_from_camera[:3, 3]
                    down = math.degrees(math.asin(position[2] / np.linalg.norm(position)))
                    text += (f'   camera {np.linalg.norm(position):.2f} m away, '
                             f'{position[2]:.2f} m above the table, looking down {down:.0f} deg')
                    color = (0, 200, 0) if down >= 30 else (0, 165, 255)
                except PerceptionError as error:
                    text += f'   ({error})'
            cv2.putText(view, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 6)
            cv2.putText(view, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2)
            ok, jpeg = cv2.imencode('.jpg', cv2.resize(view, None, fx=0.6, fy=0.6),
                                    [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                latest['jpeg'] = jpeg.tobytes()
    finally:
        camera.release()
        server.shutdown()


def serve_mjpeg(port, latest):
    """Serve the newest JPEG in ``latest['jpeg']`` as a browser-viewable stream.

    A web page instead of a GUI window: the container has no access to the
    desktop's display, but any browser can open http://localhost:<port>.
    """
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    import threading

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802 - name fixed by http.server
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    jpeg = latest.get('jpeg')
                    if jpeg:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                                         + jpeg + b'\r\n')
                    time.sleep(0.15)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--device', type=int, default=None,
                     help='V4L2 camera index (default: the saved one, else 0)')
    cli.add_argument('--square-mm', type=float, default=None,
                     help='measured size of one printed square (5 squares / 5); '
                          'remembered from the zed/intrinsics step, else 35')
    commands = cli.add_subparsers(dest='command', required=True)
    commands.add_parser('print').set_defaults(run=command_print)
    zed = commands.add_parser('zed', help='use a ZED as a USB camera + factory calibration')
    zed.add_argument('--conf', required=True, help='SN<serial>.conf from calib.stereolabs.com')
    zed.add_argument('--mode', default='FHD', choices=sorted(ZED_MODES))
    zed.set_defaults(run=command_zed)
    intrinsics = commands.add_parser('intrinsics')
    intrinsics.add_argument('--seconds', type=float, default=20.0)
    intrinsics.set_defaults(run=command_intrinsics)
    touch = commands.add_parser('touch')
    touch.add_argument('--corner', type=int, choices=(1, 2, 3, 4), required=True)
    touch.add_argument('--tool-length', type=float, default=0.20,
                       help='recorded with each touch for the tool0-based height contract. '
                            'The fingertip position itself now comes from lab_tooling.py '
                            '(RG2 on the Dual Quick Changer, 60 deg off tool0 Z)')
    touch.add_argument('--robot-ip', default='192.168.56.101')
    touch.add_argument('--tool-index', type=int, default=2, help='OnRobot device index')
    touch.set_defaults(run=command_touch)
    fit = commands.add_parser('fit')
    fit.add_argument('--output', default=CALIBRATION_FILE)
    fit.set_defaults(run=command_fit)
    check = commands.add_parser('check')
    check.add_argument('--calibration', default=CALIBRATION_FILE)
    check.add_argument('--image', default=os.path.join(DOCS, 'check.jpg'))
    check.set_defaults(run=command_check)
    look = commands.add_parser('look')
    look.add_argument('--image', default=os.path.join(DOCS, 'look.jpg'))
    look.set_defaults(run=command_look)
    aim = commands.add_parser('aim', help='live browser view for placing the camera')
    aim.add_argument('--seconds', type=float, default=600.0)
    aim.add_argument('--port', type=int, default=8089)
    aim.set_defaults(run=command_aim)
    args = cli.parse_args()
    if args.square_mm is None:
        # One measurement, used by every step: a board the software thinks is
        # 1 % bigger than the print puts every object 1 % further away.
        args.square_mm = float(read_json(CAMERA_FILE, {}).get('square_mm', 35.0))
    args.run(args)


if __name__ == '__main__':
    main()
