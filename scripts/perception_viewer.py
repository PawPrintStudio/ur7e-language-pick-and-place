#!/usr/bin/env python3
"""Live camera/depth beside the last immutable detector capture (display only)."""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    node = Node('perception_viewer')
    bridge = CvBridge()
    images = {}
    pose_text = ['Awaiting DETECT -> LOCATE']
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(msg, key):
        if key == 'depth':
            depth = bridge.imgmsg_to_cv2(msg).astype('float32')
            if msg.encoding == '16UC1':
                depth *= .001
            valid = np.isfinite(depth) & (depth > 0)
            scaled = (np.clip(np.nan_to_num(depth), 0, 1.5)/1.5*255).astype('uint8')
            image = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            image[~valid] = 0
        else:
            image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        images[key] = image

    for key, topic, qos in [('rgb', '/camera/rgb', qos_profile_sensor_data),
                            ('depth', '/camera/depth', qos_profile_sensor_data),
                            ('detection', '/perception/annotated', retained)]:
        node.create_subscription(Image, topic, lambda msg, key=key: receive(msg, key), qos)

    def located(msg):
        p = msg.pose.position
        pose_text[0] = f'Base XYZ: {p.x:.3f}, {p.y:.3f}, {p.z:.3f} m'

    node.create_subscription(PoseStamped, '/perception/grasp_pose', located, retained)
    title = 'Task 2 | RGB-D and OWLv2 perception'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 720, 580)
    cv2.moveWindow(title, 0, 510)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.03)
            panels = []
            for key, label in [('rgb', 'LIVE RGB'), ('depth', 'LIVE DEPTH: 0-1.5 m'),
                               ('detection', 'LAST DETECTION (frozen capture)'), ('info', 'SIMULATION ONLY')]:
                panel = cv2.resize(images.get(key, np.zeros((240, 320, 3), dtype='uint8')), (360, 270))
                cv2.rectangle(panel, (0, 0), (360, 27), (25, 25, 25), -1)
                cv2.putText(panel, label, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
                if key == 'info':
                    for i, line in enumerate(['Gazebo overhead camera', 'OWLv2 open-vocabulary detector', 'Green overlay = selected mask', pose_text[0], 'Grasp uses captured RGB + depth', 'Physical validation is pending']):
                        cv2.putText(panel, line, (8, 60+i*31), cv2.FONT_HERSHEY_SIMPLEX, .43, (220, 225, 230), 1)
                panels.append(panel)
            cv2.imshow(title, np.vstack([np.hstack(panels[:2]), np.hstack(panels[2:])]))
            if cv2.waitKey(1) & 0xff == 27:
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
