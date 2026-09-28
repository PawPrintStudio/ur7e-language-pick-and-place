"""Exercise actual ROS message conversions and cached service contracts."""
import numpy as np
import pytest
from pathlib import Path


@pytest.fixture
def node():
    rclpy = pytest.importorskip('rclpy')
    from ur7e_perception.nodes import PerceptionNode
    config = Path(__file__).resolve().parents[1]/'config'/'synthetic.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    value = PerceptionNode()
    yield value
    value.destroy_node()
    rclpy.shutdown()


def messages(node):
    from sensor_msgs.msg import CameraInfo
    from ur7e_perception.synthetic import scene
    frame = scene()[0]
    rgb = node.bridge.cv2_to_imgmsg(frame.rgb, 'rgb8')
    depth = node.bridge.cv2_to_imgmsg(frame.depth, '32FC1')
    info = CameraInfo()
    info.height, info.width = frame.depth.shape
    info.k = frame.k.flatten().tolist()
    stamp = node.get_clock().now().to_msg()
    for msg in (rgb, depth, info):
        msg.header.stamp, msg.header.frame_id = stamp, frame.frame_id
    return rgb, depth, info


def test_capture_id_keeps_original_depth(node):
    from ur7e_interfaces.srv import DetectObject, LocateObject
    rgb, depth, info = messages(node)
    node.capture(rgb, depth, info)
    result = node.detect(DetectObject.Request(query='red block'), DetectObject.Response())
    assert result.success, result.reason
    before = node.locate(LocateObject.Request(capture_id=result.capture_id), LocateObject.Response())
    assert before.success
    # A later capture has a different object height. It must not affect the old ID.
    changed = node.bridge.imgmsg_to_cv2(depth).copy()-.1
    next_depth = node.bridge.cv2_to_imgmsg(changed, '32FC1')
    next_depth.header = depth.header
    node.capture(rgb, next_depth, info)
    after = node.locate(LocateObject.Request(capture_id=result.capture_id), LocateObject.Response())
    assert after.pose.pose.position.z == before.pose.pose.position.z
    frame, _ = node.captures[result.capture_id]
    frame.stamp -= 30
    stale = node.locate(LocateObject.Request(capture_id=result.capture_id), LocateObject.Response())
    assert not stale.success and not stale.pose.header.frame_id


def test_frame_mismatch_and_units(node):
    rgb, depth, info = messages(node)
    mm = (np.nan_to_num(node.bridge.imgmsg_to_cv2(depth))*1000).astype('uint16')
    message = node.bridge.cv2_to_imgmsg(mm, '16UC1')
    message.header = depth.header
    node.capture(rgb, message, info)
    assert .9 < float(np.median(node.latest.depth)) < 1.01
    message.header.frame_id = 'wrong_optical_frame'
    node.capture(rgb, message, info)
    assert node.latest is None


def test_service_failure_and_eviction(node):
    from ur7e_interfaces.srv import DetectObject, LocateObject
    missing = node.detect(DetectObject.Request(query='red block'), DetectObject.Response())
    assert not missing.success and not missing.capture_id
    node.capture(*messages(node))
    ids = []
    for _ in range(9):
        result = node.detect(DetectObject.Request(query='red block'), DetectObject.Response())
        assert result.success
        ids.append(result.capture_id)
    assert len(node.captures) == 8
    gone = node.locate(LocateObject.Request(capture_id=ids[0]), LocateObject.Response())
    assert not gone.success
