#!/usr/bin/env python3
"""Save one frame from the running perception node's ``/camera/rgb`` topic.

The camera device can only be open in one process, so while the perception
node runs, this is how a person (or a log) gets a picture of the scene.
"""
import argparse
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('output', nargs='?', default='log/zed/look.jpg')
    cli.add_argument('--timeout', type=float, default=5.0)
    args = cli.parse_args()
    rclpy.init()
    node = rclpy.create_node('lab_snapshot')
    frames = []
    node.create_subscription(Image, '/camera/rgb', frames.append, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    while not frames and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not frames:
        print('no frame on /camera/rgb (is the perception node running?)')
        return 1
    rgb = CvBridge().imgmsg_to_cv2(frames[-1], 'rgb8')
    cv2.imwrite(args.output, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    print(args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
