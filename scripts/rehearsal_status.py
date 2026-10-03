#!/usr/bin/env python3
"""Publish the robot status topics that mock hardware does not have.

``lab_jog.JogExecutor.gate()`` refuses to move unless the safety mode is
NORMAL, the External Control program is running, and the speed slider is
inside the approved window. On the real robot the driver publishes those from
the controller; with ros2_control mock hardware nobody does. This node says
"all good, slider at 100 %" so a rehearsal can execute -- which is exactly
why it must never run in the lab's ROS domain.
"""
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
from ur_dashboard_msgs.msg import SafetyMode


def main():
    """Publish fake status until interrupted."""
    if os.environ.get('ROS_DOMAIN_ID') == '42':
        sys.exit('refusing to fake robot status in the lab domain (42)')
    rclpy.init()
    node = Node('rehearsal_status')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    running = node.create_publisher(Bool, '/io_and_status_controller/robot_program_running',
                                    latched)
    safety = node.create_publisher(SafetyMode, '/io_and_status_controller/safety_mode', latched)
    scaling = node.create_publisher(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                                    10)
    running.publish(Bool(data=True))
    safety.publish(SafetyMode(mode=SafetyMode.NORMAL))
    node.create_timer(0.1, lambda: scaling.publish(Float64(data=100.0)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
