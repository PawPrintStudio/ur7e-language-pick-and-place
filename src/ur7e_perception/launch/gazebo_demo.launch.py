"""Complete software workstation: one Gazebo robot, camera, MoveIt and motion."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    def include(package, filename, arguments):
        path = os.path.join(get_package_share_directory(package), 'launch', filename)
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=arguments.items())
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='fixture'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        include('ur7e_perception', 'gazebo_perception.launch.py',
                {'backend': LaunchConfiguration('backend'),
                 'gazebo_gui': LaunchConfiguration('gazebo_gui')}),
        include('ur7e_pick_place_bringup', 'ur7e_moveit.launch.py', {
            'launch_rviz': LaunchConfiguration('launch_rviz'), 'use_sim_time': 'true',
            'moveit_controllers_file': 'config/ur7e_gz_moveit_controllers.yaml'}),
        Node(package='ur7e_motion', executable='motion_node', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
