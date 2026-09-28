"""
Task 1.8: bring up the UR7e + RG2 in Ignition Gazebo 6 (Fortress).

Table, pick/place objects, real physics, the DetachableJoint grasp latch.

Same seam as tier 1/2 (docs/SIMULATION.md): everything above ros2_control —
MoveIt2 (ur7e_pick_place_bringup/launch/ur7e_moveit.launch.py), motion_node,
scripts/pick_place_demo.py — is unmodified. This launch file only stands up
what's *behind* that seam differently: Gazebo instead of mock hardware or the
real driver.

Headless by default (`gazebo_gui:=false`, `-s` — server only, no render
window) so this runs in CI/a container with no display; set `gazebo_gui:=true`
on a workstation to actually watch it.

The DetachableJoint startup workaround (see ur7e_gazebo/urdf/ur7e_gz.urdf.xacro's
comment for how this was confirmed, not assumed): the plugin welds
`pick_object` to the gripper THE INSTANT the world starts, not on the first
"attach" message. `initial_detach` below publishes one "detach" a few seconds
after spawn — long enough for the world, robot, and controllers to all be up
— so the object starts free on the table like task 1.7 expects. It's a
work-around, not a clean API, because the plugin doesn't offer a "start
detached" option.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    kinematics_parameters_file = LaunchConfiguration("kinematics_parameters_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur7e"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument(
            "kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur7e_bringup"), "config", "ur7e_calibration.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_gui", default_value="false",
            description="Headless by default (server only, -s) — set true "
            "on a workstation with a display to actually watch it.",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur7e_gazebo"), "worlds", "pick_place_table.sdf"]
            ),
        ),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur7e_gazebo"), "urdf", "ur7e_gz.urdf.xacro"]),
            " ",
            "ur_type:=", ur_type, " ",
            "tf_prefix:=", tf_prefix, " ",
            "kinematics_parameters_file:=", kinematics_parameters_file, " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 3 ", world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 3 ", world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "ur7e_gz",
            "-allow_renaming", "true",
        ],
    )

    # /clock bridge — everything downstream (MoveIt, motion_node) runs with
    # use_sim_time:=true and needs Gazebo's simulated clock, not the wall clock.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # ROS -> Gazebo bridges for the grasp latch, so scripts/pick_place_demo.py
    # can trigger attach/detach with a plain rclpy publisher instead of
    # shelling out to `ign topic`.
    grasp_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/pick_object/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/pick_object/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    gripper_action_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
    )

    # Startup-only workaround for DetachableJoint's "starts attached"
    # default — see this file's module docstring. `ign topic` (not a ROS
    # bridge round-trip) because at t=0 nothing has necessarily subscribed
    # to bridge the ROS side yet; talking Gazebo Transport directly is more
    # robust for a one-shot startup message.
    initial_detach = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=["ign", "topic", "-t", "/pick_object/detach",
                     "-m", "ignition.msgs.Empty", "-p", "unused: true"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            gz_sim_gui,
            gz_sim_headless,
            gz_spawn_entity,
            clock_bridge,
            grasp_bridge,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            gripper_action_controller_spawner,
            initial_detach,
        ]
    )
