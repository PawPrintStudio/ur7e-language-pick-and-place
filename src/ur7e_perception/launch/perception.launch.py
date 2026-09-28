"""Camera-free defaults; external mode consumes the same canonical topics."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('ur7e_perception'), 'config', 'synthetic.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('backend', default_value='fixture'),
        DeclareLaunchArgument('external_camera', default_value='false'),
        DeclareLaunchArgument('replay_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/rgb'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth'),
        DeclareLaunchArgument('info_topic', default_value='/camera/camera_info'),
        Node(package='ur7e_perception', executable='rgbd_source',
             condition=UnlessCondition(LaunchConfiguration('external_camera')),
             parameters=[{'replay_file': LaunchConfiguration('replay_file'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='ur7e_perception', executable='perception_node', output='screen',
             remappings=[('/camera/rgb', LaunchConfiguration('rgb_topic')),
                         ('/camera/depth', LaunchConfiguration('depth_topic')),
                         ('/camera/camera_info', LaunchConfiguration('info_topic'))],
             parameters=[LaunchConfiguration('config'),
                         {'backend': LaunchConfiguration('backend'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
