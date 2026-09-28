#!/usr/bin/env python3
"""Save one synchronized canonical RGB-D capture for camera-free replay."""
import argparse
from pathlib import Path
import time

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from ur7e_perception.core import Frame
from ur7e_perception.synthetic import save_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='New .npz path, also writes RGB .png beside it')
    args = parser.parse_args()
    path = Path(args.output)
    if path.exists() or path.with_suffix('.png').exists():
        parser.error('refusing to overwrite a capture')
    rclpy.init()
    node = Node('perception_snapshot')
    bridge = CvBridge()
    frames = []

    def receive(rgb, depth, info):
        if len({m.header.frame_id for m in (rgb, depth, info)}) != 1:
            return
        d = bridge.imgmsg_to_cv2(depth).astype('float32')
        if depth.encoding == '16UC1':
            d *= .001
        elif depth.encoding != '32FC1':
            return
        k = np.array(info.p).reshape(3, 4)[:, :3]
        if k[0, 0] <= 0:
            k = np.array(info.k).reshape(3, 3)
        frame = Frame(bridge.imgmsg_to_cv2(rgb, 'rgb8'), d, k,
                      rgb.header.stamp.sec+rgb.header.stamp.nanosec*1e-9, rgb.header.frame_id)
        frame.validate()
        frames.append(frame)

    subs = [message_filters.Subscriber(node, cls, topic, qos_profile=qos_profile_sensor_data)
            for cls, topic in [(Image, '/camera/rgb'), (Image, '/camera/depth'),
                               (CameraInfo, '/camera/camera_info')]]
    sync = message_filters.ApproximateTimeSynchronizer(subs, 10, .04)
    sync.registerCallback(receive)
    try:
        deadline = time.monotonic()+20
        while not frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
        if not frames:
            raise RuntimeError('no synchronized capture within 20 seconds')
        save_frame(path, frames[0])
        cv2.imwrite(str(path.with_suffix('.png')), cv2.cvtColor(frames[0].rgb, cv2.COLOR_RGB2BGR))
        print(f'Saved {frames[0].rgb.shape}, optical frame {frames[0].frame_id}: {path}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
