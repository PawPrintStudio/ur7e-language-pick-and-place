#!/usr/bin/env python3
"""Live ROS contract checks: synchronized input, capture IDs, stale rejection."""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from ur7e_interfaces.srv import DetectObject, LocateObject


def call(node, client, request):
    assert client.wait_for_service(timeout_sec=30), 'service not available'
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)
    assert future.done(), 'service timeout'
    return future.result()


def main():
    rclpy.init()
    node = Node('perception_smoke')
    detect = node.create_client(DetectObject, '/perception/detect_object')
    locate = node.create_client(LocateObject, '/perception/locate_object')
    try:
        request = DetectObject.Request(query='red block')
        result = call(node, detect, request)
        deadline = time.monotonic()+15
        while not result.success and time.monotonic() < deadline:
            time.sleep(.2)
            result = call(node, detect, request)
        assert result.success, result.reason
        assert result.mask.encoding == 'mono8' and result.mask.width > 0
        found = call(node, locate, LocateObject.Request(capture_id=result.capture_id))
        assert found.success, found.reason
        assert found.pose.header.frame_id == 'base_link'
        p = found.pose.pose.position
        assert np.isfinite([p.x, p.y, p.z]).all() and p.z > 0
        assert not call(node, locate, LocateObject.Request(capture_id='missing')).success
        assert not call(node, detect, DetectObject.Request(query='purple elephant')).success
        assert not call(node, detect, DetectObject.Request(query='')).success
        print(f'PASS ROS detect -> immutable capture -> locate: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
