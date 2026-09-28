"""ROS transport with synchronized RGB-D and immutable detection captures."""

from collections import OrderedDict
from threading import Lock
import time
import uuid

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ur7e_interfaces.srv import DetectObject, LocateObject

from .backends import make_backend
from .core import Frame, PerceptionError, check_fresh, localize, rigid_transform
from .synthetic import load_frame, scene


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PerceptionNode(Node):
    """Detection and locator services share exact timestamped capture storage."""

    def __init__(self):
        super().__init__('perception_node')
        defaults = {
            'backend': 'owlv2', 'device': 'cpu', 'threshold': 0.15,
            'camera_frame': 'camera_optical_frame', 'base_frame': 'base_link',
            'camera_to_base': [0.0]*16, 'workspace': [0.0]*8,
            'table_z': 0.0, 'max_z': 0.35, 'max_capture_age': 15.0,
            'simulation': False, 'calibration_verification': '',
            'calibration_marker_id': 42, 'calibration_marker_size': .04,
            'base_from_marker': [0.0]*16,
            'owl_engine': '', 'sam_encoder': '', 'sam_decoder': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.get = lambda name: self.get_parameter(name).value
        self.transform = rigid_transform(np.array(self.get('camera_to_base')).reshape(4, 4))
        if not self.get('simulation'):
            # A physical session must supply held-out verification evidence.
            import json
            from .core import verify_marker
            with open(self.get('calibration_verification')) as stream:
                evidence = json.load(stream)
            if evidence.get('source') != 'physical':
                raise PerceptionError('physical mode requires physical calibration evidence')
            verify_marker(evidence['camera_points'], evidence['base_points'], self.transform)
            rigid_transform(np.array(self.get('base_from_marker')).reshape(4, 4))
        name = self.get('backend')
        if name == 'fixture' and not self.get('simulation'):
            raise PerceptionError('fixture backend is simulation only')
        kwargs = {}
        if name == 'owlv2':
            kwargs = dict(device=self.get('device'), threshold=self.get('threshold'))
        if name == 'nanoowl':
            kwargs = {key: self.get(key) for key in ('owl_engine', 'sam_encoder', 'sam_decoder', 'threshold')}
        self.backend = make_backend(name, **kwargs)
        self.bridge = CvBridge()
        display_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.overlay_pub = self.create_publisher(Image, '/perception/annotated', display_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/perception/grasp_pose', display_qos)
        self.lock = Lock()
        self.latest = None
        self.captures = OrderedDict()
        self.sensor_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.subs = [message_filters.Subscriber(
            self, cls, topic, qos_profile=qos_profile_sensor_data, callback_group=self.sensor_group)
            for cls, topic in [(Image, '/camera/rgb'), (Image, '/camera/depth'),
                               (CameraInfo, '/camera/camera_info')]]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, 10, 0.04)
        self.sync.registerCallback(self.capture)
        self.create_service(DetectObject, '/perception/detect_object', self.detect,
                            callback_group=self.service_group)
        self.create_service(LocateObject, '/perception/locate_object', self.locate,
                            callback_group=self.service_group)
        self.get_logger().info(f'Ready: {name}, simulation={self.get("simulation")}')

    def capture(self, rgb, depth, info):
        try:
            frames = {m.header.frame_id for m in (rgb, depth, info)}
            if frames != {self.get('camera_frame')}:
                raise PerceptionError('RGB/depth/intrinsics optical frames disagree')
            if (info.width, info.height) != (rgb.width, rgb.height):
                raise PerceptionError('camera info dimensions disagree')
            # P describes the rectified image. Some drivers retain raw D/K
            # beside P even on the image_rect topic.
            k = np.array(info.p).reshape(3, 4)[:, :3]
            if k[0, 0] <= 0:
                if any(abs(x) > 1e-8 for x in info.d):
                    raise PerceptionError('rectified RGB/intrinsics required')
                k = np.array(info.k).reshape(3, 3)
            if depth.encoding not in ('32FC1', '16UC1'):
                raise PerceptionError('depth encoding must be 32FC1 metres or 16UC1 mm')
            d = self.bridge.imgmsg_to_cv2(depth).astype('float32')
            if depth.encoding == '16UC1':
                d *= 0.001
            frame = Frame(self.bridge.imgmsg_to_cv2(rgb, 'rgb8').copy(), d,
                          k, seconds(rgb.header.stamp), rgb.header.frame_id)
            frame.validate()
            with self.lock:
                self.latest = frame
        except Exception as error:
            with self.lock:
                self.latest = None
            self.get_logger().warning(str(error))

    def detect(self, request, response):
        start = time.perf_counter()
        try:
            query = request.query.strip()
            if not query or len(query) > 120 or '\n' in query:
                raise PerceptionError('query must be a short noun phrase')
            with self.lock:
                frame = self.latest
            if frame is None:
                raise PerceptionError('no synchronized capture')
            check_fresh(frame, self.get_clock().now().nanoseconds/1e9, 2.0)
            if not self.get('simulation'):
                from .fiducials import check_workspace_marker
                check_workspace_marker(
                    frame, self.transform, np.array(self.get('base_from_marker')).reshape(4, 4),
                    self.get('calibration_marker_id'), self.get('calibration_marker_size'))
            detection = self.backend.detect(frame, query)
            check_fresh(frame, self.get_clock().now().nanoseconds/1e9, self.get('max_capture_age'))
            capture_id = uuid.uuid4().hex
            self.captures[capture_id] = (frame, detection)
            while len(self.captures) > 8:
                self.captures.popitem(last=False)
            response.mask = self.bridge.cv2_to_imgmsg(detection.mask.astype('uint8')*255, 'mono8')
            response.mask.header.frame_id = frame.frame_id
            response.mask.header.stamp.sec = int(frame.stamp)
            response.mask.header.stamp.nanosec = int((frame.stamp-int(frame.stamp))*1e9)
            response.bbox = [float(x) for x in detection.bbox]
            response.confidence = float(detection.confidence)
            response.capture_id, response.success = capture_id, True
            # Display the immutable inference capture, not a later live frame.
            import cv2
            overlay = frame.rgb.copy()
            selected = detection.mask.astype(bool)
            overlay[selected] = (overlay[selected]*.4 + np.array([0, 255, 80])*.6).astype('uint8')
            x1, y1, x2, y2 = map(int, detection.bbox)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 80), 1)
            cv2.putText(overlay, f'{query}: {detection.confidence:.2f}', (5, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
            message = self.bridge.cv2_to_imgmsg(overlay, 'rgb8')
            message.header = response.mask.header
            self.overlay_pub.publish(message)
        except Exception as error:
            response.success, response.reason = False, str(error)
        response.latency_ms = (time.perf_counter()-start)*1000
        return response

    def locate(self, request, response):
        try:
            if request.capture_id not in self.captures:
                raise PerceptionError('unknown or evicted capture ID')
            frame, detection = self.captures[request.capture_id]
            check_fresh(frame, self.get_clock().now().nanoseconds/1e9, self.get('max_capture_age'))
            grasp = localize(frame, detection, self.transform,
                             np.array(self.get('workspace')).reshape(-1, 2),
                             table_z=self.get('table_z'), max_z=self.get('max_z'))
            response.pose.header.frame_id = self.get('base_frame')
            response.pose.header.stamp.sec = int(frame.stamp)
            response.pose.header.stamp.nanosec = int((frame.stamp-int(frame.stamp))*1e9)
            p, q = response.pose.pose.position, response.pose.pose.orientation
            p.x, p.y, p.z = map(float, grasp.position)
            q.x, q.y, q.z, q.w = map(float, grasp.quaternion)
            response.valid_points, response.axis_ratio = grasp.points, grasp.axis_ratio
            response.success = True
            self.pose_pub.publish(response.pose)
        except Exception as error:
            response.success, response.reason = False, str(error)
        return response


class RgbdSource(Node):
    """Publish a synthetic scene or NPZ recording through normal sensor topics."""

    def __init__(self):
        super().__init__('rgbd_source')
        self.declare_parameter('replay_file', '')
        self.declare_parameter('seed', 0)
        path = self.get_parameter('replay_file').value
        self.frame = load_frame(path) if path else scene(self.get_parameter('seed').value)[0]
        self.bridge = CvBridge()
        self.pubs = [self.create_publisher(cls, topic, qos_profile_sensor_data)
                     for cls, topic in [(Image, '/camera/rgb'), (Image, '/camera/depth'),
                                        (CameraInfo, '/camera/camera_info')]]
        # Source is explicitly a replay clock: restamp all three messages together.
        self.create_timer(1/15, self.publish)
        self.tf = StaticTransformBroadcaster(self)
        if not path:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'base_link', self.frame.frame_id
            t.transform.translation.x, t.transform.translation.z = 0.45, 1.0
            t.transform.rotation.x = 1.0
            self.tf.sendTransform(t)

    def publish(self):
        frame = self.frame
        rgb = self.bridge.cv2_to_imgmsg(frame.rgb, 'rgb8')
        depth = self.bridge.cv2_to_imgmsg(frame.depth, '32FC1')
        info = CameraInfo()
        info.height, info.width = frame.depth.shape
        info.k = frame.k.flatten().tolist()
        info.p = np.column_stack((frame.k, np.zeros(3))).flatten().tolist()
        info.r = np.eye(3).flatten().tolist()
        info.distortion_model = 'plumb_bob'
        stamp = self.get_clock().now().to_msg()
        for pub, msg in zip(self.pubs, (rgb, depth, info)):
            msg.header.stamp, msg.header.frame_id = stamp, frame.frame_id
            pub.publish(msg)


def spin(cls):
    rclpy.init()
    node = cls()
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


def perception_main():
    spin(PerceptionNode)


def source_main():
    spin(RgbdSource)
