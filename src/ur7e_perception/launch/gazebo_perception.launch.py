"""Add rendered RGB-D to the existing Fortress world without a second robot."""
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable, TimerAction, ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    package = get_package_share_directory('ur7e_perception')
    gazebo = get_package_share_directory('ur7e_gazebo')
    tree = ET.parse(os.path.join(gazebo, 'worlds', 'pick_place_table.sdf'))
    world = tree.getroot().find('world')
    plugin = ET.SubElement(world, 'plugin', {
        'filename': 'ignition-gazebo-sensors-system', 'name': 'ignition::gazebo::systems::Sensors'})
    ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    world.append(ET.parse(os.path.join(package, 'config', 'overhead_camera.sdf')).getroot().find('model'))
    # Runtime file only; the original physics/gripper world stays the source of truth.
    fd, path = tempfile.mkstemp(prefix='ur7e-rgbd-', suffix='.sdf')
    with os.fdopen(fd, 'wb') as stream:
        tree.write(stream, encoding='utf-8', xml_declaration=True)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/overhead/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/overhead/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/overhead/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
        remappings=[('/overhead/image', '/camera/rgb'),
                    ('/overhead/depth_image', '/camera/depth'),
                    ('/overhead/camera_info', '/camera/camera_info')],
        parameters=[{'use_sim_time': True,
                     'qos_overrides./camera/rgb.publisher.reliability': 'best_effort',
                     'qos_overrides./camera/depth.publisher.reliability': 'best_effort',
                     'qos_overrides./camera/camera_info.publisher.reliability': 'best_effort'}])
    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join([
            os.path.dirname(get_package_share_directory('onrobot_description')),
            os.path.dirname(get_package_share_directory('ur_description')),
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gazebo, 'launch', 'ur7e_gz.launch.py')),
            launch_arguments={'world_file': path, 'gazebo_gui': LaunchConfiguration('gazebo_gui')}.items()),
        bridge,
        # Fortress latches once during startup. The base launch detaches at
        # 8 seconds; then restore the block before declaring the demo ready.
        TimerAction(period=9.0, actions=[ExecuteProcess(cmd=[
            'ign', 'service', '-s', '/world/pick_place_table/set_pose',
            '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000', '--req',
            'name: "pick_object", position: {x: 0.45, y: -0.15, z: 0.04}, orientation: {w: 1.0}',
        ])]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.45', '--z', '1.0', '--qx', '0.7071067811865476',
                        '--qy', '-0.7071067811865476', '--qz', '0', '--qw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'overhead_camera/link/rgbd']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', 'perception.launch.py')),
            launch_arguments={'external_camera': 'true', 'use_sim_time': 'true',
                              'config': os.path.join(package, 'config', 'gazebo.yaml'),
                              'backend': LaunchConfiguration('backend')}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('backend', default_value='fixture'),
        OpaqueFunction(function=setup),
    ])
