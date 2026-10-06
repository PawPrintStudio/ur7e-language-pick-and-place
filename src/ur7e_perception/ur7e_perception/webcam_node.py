"""Perception services from one webcam: same contract as the RGB-D node.

``/perception/detect_object`` and ``/perception/locate_object`` behave exactly
as they do in :mod:`ur7e_perception.nodes` (immutable capture IDs, explicit
``success``/``reason``), so everything downstream -- the pick demo, the
orchestrator -- is unchanged. What differs is where 3-D comes from: not a
depth image but the printed table board (see :mod:`ur7e_perception.monocular`)
-- or, when the calibration carries a ``stereo`` section (written by
``scripts/lab_stereo_calibration.py``), both lenses of a ZED used as one USB
camera (see :func:`ur7e_perception.stereo.locate_object`).

Per detection request the node:

1. takes the most recent few frames (the camera thread keeps them fresh);
2. finds the board and solves the camera pose *now*, so a nudged laptop is
   corrected automatically and a hidden board is a refusal, not a guess;
3. runs the detector (``owlv2`` open-vocabulary model, ``color`` for coloured
   blocks, or ``auto``), keeping only candidates that sit on the reachable
   table and not on the board itself;
4. stores image + camera pose + mask under a capture ID for ``locate``.

The special query ``marker 42`` finds an ArUco marker instead: a flat target
whose centre is known exactly, used for the touch-point verification.
"""
from collections import deque, OrderedDict
import json
import os
import re
import threading
import time
import uuid

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ur7e_interfaces.srv import DetectObject, LocateObject
import yaml

from . import monocular as mono
from .camera import Camera
from .core import Detection, PerceptionError
from .stereo import locate_object, StereoRig

MARKER_QUERY = re.compile(r'^marker (\d{1,2})$')


def looks_blank(rgb):
    """Return True for a privacy-shutter frame: no texture at all."""
    return float(rgb.std()) < 2.0


class ObjectHeights:
    """Heights by object name: the one thing a single camera cannot measure."""

    def __init__(self, path, default):
        self.default = None if default is None or default < 0 else float(default)
        self.table = {}
        if path:
            with open(path) as stream:
                data = yaml.safe_load(stream) or {}
            fallback = data.get('default_height_m', default)
            self.default = None if fallback is None or fallback < 0 else float(fallback)
            self.table = {str(k).lower(): (None if v is None or float(v) < 0 else float(v))
                          for k, v in (data.get('objects') or {}).items()}

    def lookup(self, query):
        """Return the height for the longest known name in ``query`` (None = unknown).

        Unknown means the locator falls back to the near-edge estimate, which
        needs no height and is good to about a centimetre for compact objects.
        """
        matches = [name for name in self.table if name in query.lower()]
        return self.table[max(matches, key=len)] if matches else self.default


class Owlv2Candidates:
    """OWLv2 open-vocabulary boxes; segmentation is done by colour afterwards."""

    def __init__(self, threshold, device='cpu', model='google/owlv2-base-patch16-ensemble'):
        import torch
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.torch, self.device, self.threshold = torch, device, threshold
        self.processor = Owlv2Processor.from_pretrained(model)
        self.model = Owlv2ForObjectDetection.from_pretrained(model).to(device).eval()

    def boxes(self, rgb, query):
        """Return [(score, (x0, y0, x1, y1)), ...] best first."""
        from PIL import Image as PilImage
        inputs = self.processor(text=[[f'a photo of a {query}']],
                                images=PilImage.fromarray(rgb), return_tensors='pt')
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        with self.torch.inference_mode():
            outputs = self.model(**inputs)
        result = self.processor.post_process_grounded_object_detection(
            outputs, target_sizes=[rgb.shape[:2]], threshold=self.threshold)[0]
        pairs = [(float(score), tuple(box.detach().cpu().tolist()))
                 for score, box in zip(result['scores'], result['boxes'])]
        return sorted(pairs, reverse=True)


class WebcamPerceptionNode(Node):
    """Detect and locate tabletop objects with a single calibrated webcam."""

    def __init__(self):
        super().__init__('perception_node')
        defaults = {
            'device': -1, 'calibration': '', 'backend': 'auto', 'threshold': 0.10,
            'default_height': 0.03, 'objects_file': '', 'average_frames': 5,
            'max_frame_age': 1.0, 'max_capture_age': 60.0, 'base_frame': 'base_link',
            'reach_min': 0.20, 'reach_max': 0.80, 'snapshot_dir': '',
            'publish_rate': 3.0, 'ambiguity_ratio': 0.85, 'replay_image': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.get = lambda name: self.get_parameter(name).value
        if not self.get('calibration'):
            raise PerceptionError('set the calibration parameter '
                                  '(scripts/lab_table_calibration.py writes the file)')
        self.calibration = mono.TableCalibration.load(self.get('calibration'))
        # A still image instead of a camera: for rehearsing without the lab.
        # Only then may the calibration be synthetic -- a live camera with a
        # made-up calibration would send the arm to made-up places.
        self.replay = None
        if self.get('replay_image'):
            image = cv2.imread(self.get('replay_image'))
            if image is None:
                raise PerceptionError('replay_image could not be read')
            self.replay = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif self.calibration.source != 'physical':
            raise PerceptionError('the webcam node needs a physical table calibration')
        self.board = self.calibration.board()
        # Board mode re-solves the camera pose from the printed board in every
        # capture; fixed mode trusts a pose stored in the calibration file.
        self.has_board = self.calibration.camera_to_base is None
        self.heights = ObjectHeights(self.get('objects_file'), self.get('default_height'))
        self.owl = None
        if self.get('backend') in ('owlv2', 'auto'):
            self.get_logger().info('loading OWLv2 (first run downloads ~600 MB)...')
            self.owl = Owlv2Candidates(self.get('threshold'))

        self.frames = deque(maxlen=max(1, int(self.get('average_frames'))))
        self.lock = threading.Lock()
        self.running = True
        self.capture = None
        if self.replay is None:
            # Camera settings travel with the calibration (it is only valid
            # for the lens and resolution it was made with); `device` may be
            # overridden because USB numbering changes when a cable moves.
            settings = dict(device=0, width=1280, height=720, fourcc='MJPG', left_half=False)
            settings.update(self.calibration.notes.get('camera') or {})
            if self.get('device') >= 0:
                settings['device'] = self.get('device')
            self.capture = Camera(**settings)
            self.get_logger().info(f'camera: {settings}')
        self.stereo = self.stereo_rig(settings if self.replay is None else {})
        threading.Thread(target=self._grab, daemon=True).start()

        self.bridge = CvBridge()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb', qos_profile_sensor_data)
        self.overlay_pub = self.create_publisher(Image, '/perception/annotated', latched)
        self.pose_pub = self.create_publisher(PoseStamped, '/perception/grasp_pose', latched)
        self.detail_pub = self.create_publisher(String, '/perception/last_result', latched)
        self.captures = OrderedDict()
        self.create_timer(1.0 / self.get('publish_rate'), self._publish)
        self.create_service(DetectObject, '/perception/detect_object', self.detect)
        self.create_service(LocateObject, '/perception/locate_object', self.locate)
        mode = 'stereo (both ZED lenses)' if self.stereo else 'single-camera table mode'
        self.get_logger().info(f'Ready: backend={self.get("backend")}, {mode}')

    def stereo_rig(self, settings):
        """Return (rig, disparity offset) if the calibration is a stereo one, else None."""
        stereo = self.calibration.notes.get('stereo')
        if not stereo or not settings.get('left_half'):
            return None
        here = os.path.dirname(os.path.abspath(self.get('calibration')))
        candidates = [stereo['conf'], os.path.join(here, stereo['conf']),
                      os.path.join(here, '..', stereo['conf'])]
        conf = next((path for path in candidates if os.path.exists(path)), None)
        if conf is None:
            raise PerceptionError(f'stereo calibration names {stereo["conf"]}, not found')
        rig = StereoRig.from_zed_conf(conf, depth_range=(0.3, 2.5))
        if stereo.get('rotation_rvec'):
            rig = rig.with_rotation(cv2.Rodrigues(np.array(stereo['rotation_rvec']))[0])
        return rig, float(stereo.get('disparity_offset_px') or 0.0)

    # --- camera ---------------------------------------------------------------

    def _grab(self):
        while self.running:
            if self.replay is not None:
                rgb = self.replay
                time.sleep(0.1)
                right = None
            else:
                rgb, right = self.capture.read_pair() or (None, None)
                if rgb is None:
                    time.sleep(0.1)
                    continue
            with self.lock:
                self.frames.append((time.time(), rgb, right))

    def recent_frames(self):
        """Return (undistorted frames younger than ``max_frame_age``, stamp, raw pair).

        Frames come oldest first; the raw (left, right) pair is the newest
        one's, for stereo (``right`` is None for a single camera).
        """
        now = time.time()
        with self.lock:
            fresh = [rgb for stamp, rgb, _ in self.frames
                     if now - stamp <= self.get('max_frame_age')]
            stamp = self.frames[-1][0] if self.frames else 0.0
            pair = self.frames[-1][1:] if self.frames else (None, None)
        if not fresh:
            raise PerceptionError('no fresh camera frame')
        if looks_blank(fresh[-1]):
            raise PerceptionError('camera image is blank (privacy shutter closed?)')
        size = (fresh[-1].shape[1], fresh[-1].shape[0])
        if self.calibration.focal_px and size != tuple(self.calibration.image_size):
            # A focal length in pixels only means something at one resolution.
            raise PerceptionError(f'camera delivers {size}, calibration was made at '
                                  f'{tuple(self.calibration.image_size)}')
        return [self.calibration.undistort(rgb) for rgb in fresh], stamp, pair

    def _publish(self):
        with self.lock:
            latest = self.frames[-1] if self.frames else None
        if latest is None:
            return
        # Undistorted, so masks and boxes from the services line up with it.
        message = self.bridge.cv2_to_imgmsg(self.calibration.undistort(latest[1]), 'rgb8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'camera_optical_frame'
        self.rgb_pub.publish(message)

    # --- geometry helpers -----------------------------------------------------

    def on_table(self, pixel, k, cam_to_base):
        """Project one pixel onto the table plane; return base XYZ."""
        anchor = self.calibration.base_from_board
        return mono.rays_to_plane([pixel], k, cam_to_base, anchor[:3, 3], anchor[:3, 2])[0]

    def pickable(self, pixel, k, cam_to_base):
        """Return None if the pixel is a plausible target location, else why not."""
        point = self.on_table(pixel, k, cam_to_base)
        if not np.isfinite(point).all():
            return 'not on the table'
        reach = float(np.hypot(point[0], point[1]))
        if not self.get('reach_min') <= reach <= self.get('reach_max'):
            return f'out of reach ({reach:.2f} m from the base)'
        if not self.has_board:
            return None
        anchor = self.calibration.base_from_board
        on_board = np.linalg.inv(anchor) @ np.append(point, 1.0)
        width = mono.COLUMNS * self.calibration.square_m
        height = mono.ROWS * self.calibration.square_m
        if -0.01 <= on_board[0] <= width + 0.01 and -0.01 <= on_board[1] <= height + 0.01:
            return 'on the calibration board'
        return None

    # --- detectors ------------------------------------------------------------

    def detect_marker(self, rgb, marker_id):
        center, quad = mono.marker_center(rgb, marker_id)
        mask = np.zeros(rgb.shape[:2], np.uint8)
        cv2.fillConvexPoly(mask, quad.astype(np.int32), 1)
        x0, y0 = quad.min(axis=0)
        x1, y1 = quad.max(axis=0)
        return Detection(mask, (x0, y0, x1, y1), 1.0, f'marker {marker_id}')

    def detect_color(self, rgb, query, k, cam_to_base):
        colors = [name for name in mono.COLOR_NAMES if name in query.lower().split()]
        if len(colors) != 1:
            raise PerceptionError('the colour detector needs exactly one colour word '
                                  f'({", ".join(mono.COLOR_NAMES)})')
        hsv_region = self.reach_region(rgb.shape[:2], k, cam_to_base)
        mask, bbox = mono.color_blob(rgb, colors[0], hsv_region)
        return Detection(mask, bbox, 1.0, query)

    def reach_region(self, shape, k, cam_to_base, step=8):
        """Image mask of pixels whose table point is reachable and off the board."""
        height, width = shape
        ys, xs = np.mgrid[0:height:step, 0:width:step]
        pixels = np.column_stack((xs.ravel(), ys.ravel()))
        anchor = self.calibration.base_from_board
        points = mono.rays_to_plane(pixels, k, cam_to_base, anchor[:3, 3], anchor[:3, 2])
        reach = np.hypot(points[:, 0], points[:, 1])
        good = np.isfinite(reach) & (reach >= self.get('reach_min'))
        good &= reach <= self.get('reach_max')
        local = (np.column_stack((points, np.ones(len(points))))
                 @ np.linalg.inv(anchor).T)
        board_w = mono.COLUMNS * self.calibration.square_m
        board_h = mono.ROWS * self.calibration.square_m
        if self.has_board:
            good &= ~((local[:, 0] > -0.01) & (local[:, 0] < board_w + 0.01)
                      & (local[:, 1] > -0.01) & (local[:, 1] < board_h + 0.01))
        coarse = good.reshape(ys.shape).astype(np.uint8) * 255
        return cv2.resize(coarse, (width, height), interpolation=cv2.INTER_NEAREST)

    def detect_owl(self, rgb, query, k, cam_to_base):
        candidates = []
        rejected = []
        image_area = rgb.shape[0] * rgb.shape[1]
        for score, box in self.owl.boxes(rgb, query):
            x0, y0, x1, y1 = box
            if (x1 - x0) * (y1 - y0) > 0.25 * image_area:
                rejected.append('box covers a quarter of the image')
                continue
            # The bottom-centre of a box is where the object meets the table.
            reason = self.pickable(((x0 + x1) / 2, y1), k, cam_to_base)
            if reason:
                rejected.append(reason)
                continue
            candidates.append((score, box))
        if not candidates:
            detail = f' ({rejected[0]})' if rejected else ''
            raise PerceptionError(f'target not detected{detail}')
        best_score, best = candidates[0]
        for score, box in candidates[1:]:
            a, b = np.array(best), np.array(box)
            overlap = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])).prod()
            union = (a[2:] - a[:2]).prod() + (b[2:] - b[:2]).prod() - overlap
            distinct = overlap / max(union, 1e-9) < 0.5
            if distinct and score >= self.get('ambiguity_ratio') * best_score:
                raise PerceptionError('multiple plausible targets; clarify query')
        return Detection(mono.color_mask(rgb, best), best, best_score, query)

    # --- services -------------------------------------------------------------

    def detect(self, request, response):
        start = time.perf_counter()
        try:
            query = request.query.strip()
            if not query or len(query) > 120 or '\n' in query:
                raise PerceptionError('query must be a short noun phrase')
            frames, stamp, pair = self.recent_frames()
            rgb = frames[-1]
            if self.has_board:
                focal = self.calibration.focal_px or None
                view = mono.observe_board(frames, self.board, focal_px=focal)
            else:
                view = self.calibration.fixed_view((rgb.shape[1], rgb.shape[0]))
            cam_to_base = mono.camera_to_base(view, self.calibration.base_from_board)
            marker = MARKER_QUERY.match(query.lower())
            backend = self.get('backend')
            if marker:
                detection, height = self.detect_marker(rgb, int(marker.group(1))), 0.0
            else:
                height = self.heights.lookup(query)
                if backend == 'color':
                    detection = self.detect_color(rgb, query, view.k, cam_to_base)
                elif backend == 'owlv2':
                    detection = self.detect_owl(rgb, query, view.k, cam_to_base)
                else:
                    try:
                        detection = self.detect_owl(rgb, query, view.k, cam_to_base)
                    except PerceptionError as owl_error:
                        try:
                            detection = self.detect_color(rgb, query, view.k, cam_to_base)
                        except PerceptionError:
                            raise owl_error
            capture_id = uuid.uuid4().hex
            self.captures[capture_id] = dict(
                rgb=rgb, k=view.k, cam_to_base=cam_to_base, detection=detection,
                height=height, stamp=stamp, query=query, rms_px=view.rms_px, pair=pair)
            while len(self.captures) > 8:
                self.captures.popitem(last=False)
            response.mask = self.bridge.cv2_to_imgmsg(detection.mask.astype('uint8') * 255,
                                                      'mono8')
            response.mask.header.frame_id = 'camera_optical_frame'
            response.bbox = [float(value) for value in detection.bbox]
            response.confidence = float(detection.confidence)
            response.capture_id, response.success = capture_id, True
        except Exception as error:  # noqa: B902 - a service must answer, not die
            response.success, response.reason = False, str(error)
        response.latency_ms = (time.perf_counter() - start) * 1000
        return response

    def locate(self, request, response):
        try:
            if request.capture_id not in self.captures:
                raise PerceptionError('unknown or evicted capture ID')
            capture = self.captures[request.capture_id]
            if time.time() - capture['stamp'] > self.get('max_capture_age'):
                raise PerceptionError('capture is stale; detect again')
            anchor = self.calibration.base_from_board
            if self.stereo and capture['pair'][1] is not None:
                rig, offset = self.stereo
                grasp = locate_object(
                    rig, capture['pair'][0], capture['pair'][1], capture['detection'].mask,
                    self.calibration.focal_px, capture['cam_to_base'], anchor[:3, 3],
                    anchor[:3, 2], disparity_offset_px=offset)
                capture['height'] = grasp.height
            else:
                grasp = mono.localize_on_table(
                    capture['detection'].mask, capture['k'], capture['cam_to_base'],
                    anchor, capture['height'])
            reach = float(np.hypot(grasp.position[0], grasp.position[1]))
            if not self.get('reach_min') <= reach <= self.get('reach_max'):
                raise PerceptionError(f'object is {reach:.2f} m from the base: out of reach')
            response.pose.header.frame_id = self.get('base_frame')
            response.pose.header.stamp = self.get_clock().now().to_msg()
            p, q = response.pose.pose.position, response.pose.pose.orientation
            p.x, p.y, p.z = map(float, grasp.position)
            q.x, q.y, q.z, q.w = map(float, grasp.quaternion)
            response.valid_points, response.axis_ratio = grasp.points, grasp.axis_ratio
            response.success = True
            self.pose_pub.publish(response.pose)
            height = capture['height'] or 0.0
            detail = dict(capture_id=request.capture_id, query=capture['query'],
                          xyz=[round(float(v), 4) for v in grasp.position],
                          yaw=round(grasp.yaw, 4), height_m=height,
                          extent_m=[round(v, 4) for v in grasp.extent],
                          table_z=round(float(grasp.position[2] - height), 4),
                          board_rms_px=round(capture['rms_px'], 2),
                          stereo=bool(self.stereo and capture['pair'][1] is not None),
                          table_offset_m=round(float(getattr(grasp, 'table_offset', 0.0)), 4),
                          confidence=round(capture['detection'].confidence, 3))
            self.detail_pub.publish(String(data=json.dumps(detail)))
            self.annotate(capture, grasp, request.capture_id)
        except Exception as error:  # noqa: B902
            response.success, response.reason = False, str(error)
        return response

    def annotate(self, capture, grasp, capture_id):
        """Publish (and optionally save) the frozen capture with what was found."""
        overlay = capture['rgb'].copy()
        selected = capture['detection'].mask.astype(bool)
        overlay[selected] = (overlay[selected] * .4 + np.array([0, 255, 80]) * .6).astype('uint8')
        x0, y0, x1, y1 = map(int, capture['detection'].bbox)
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 255, 80), 2)
        pixel = mono.project([grasp.position], capture['k'], capture['cam_to_base'])[0]
        cv2.drawMarker(overlay, (int(pixel[0]), int(pixel[1])), (255, 0, 0),
                       cv2.MARKER_CROSS, 24, 2)
        text = (f'{capture["query"]}: {capture["detection"].confidence:.2f}  '
                f'xyz=({grasp.position[0]:+.3f}, {grasp.position[1]:+.3f}, '
                f'{grasp.position[2]:+.3f}) m')
        cv2.putText(overlay, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, 'rgb8'))
        directory = self.get('snapshot_dir')
        if directory:
            os.makedirs(directory, exist_ok=True)
            bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
            cv2.imwrite(os.path.join(directory, f'{capture_id}.jpg'), bgr)
            cv2.imwrite(os.path.join(directory, 'latest.jpg'), bgr)
            # Everything needed to redo this localization offline (e.g. to
            # refine the table height against a known object position).
            np.savez_compressed(
                os.path.join(directory, 'latest_capture.npz'),
                mask=capture['detection'].mask.astype(np.uint8), k=capture['k'],
                cam_to_base=capture['cam_to_base'], height=capture['height'] or -1.0,
                query=capture['query'], xyz=np.asarray(grasp.position))

    def destroy_node(self):
        """Stop the camera thread before the node goes away."""
        self.running = False
        time.sleep(0.2)
        if self.capture is not None:
            self.capture.release()
        super().destroy_node()


def main():
    """Run the webcam perception node."""
    rclpy.init()
    node = WebcamPerceptionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
