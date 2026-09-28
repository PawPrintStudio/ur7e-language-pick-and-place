"""
Phase 1 bringup (arm + RG2 gripper): a fork of the vendored launch file.

This forks, rather than wraps, ``ur_onrobot_control/launch/start_robot.launch.py``
(src/vendor, D6 in docs/ARCHITECTURE.md, pinned via ur7e.repos).

Why a fork and not ``IncludeLaunchDescription`` (the pattern ``ur7e_bringup``
uses for the bare-arm driver launch): that file hardcodes two things as
internal ``PathJoinSubstitution``s with no launch argument to override them —
the controllers YAML path and the kinematics parameters file passed to the
xacro ``Command``. ``IncludeLaunchDescription`` treats an included launch
file as opaque; there's no way to reach in and change a Node's parameter list
from outside. Forking is exactly the move the vendored file itself made on
top of ``ur_control.launch.py`` for the same reason — see its own top-of-file
comment in ``src/vendor/UR_OnRobot_ROS2/ur_onrobot_control/launch/``.

Our deltas from the vendored file, all below:

* ``kinematics_parameters_file`` — NEW argument, defaulting to our robot's
  factory calibration (reused from ``ur7e_bringup/config/ur7e_calibration.yaml``,
  task 0.4). The vendored file never passes this to the xacro ``Command`` at
  all, so it silently falls back to generic UR7e kinematics — the exact
  session-2 mistake ``ur7e_bringup``'s own header explains (checksum
  mismatch, every TCP pose wrong).
* ``ur_type`` / ``onrobot_type`` defaults changed to ``ur7e`` / ``rg2`` (ours
  are ``ur3e`` / ``rg2`` upstream).
* controllers file points at ``config/ur7e_controllers.yaml`` (this package):
  the vendored ``ur_onrobot_controllers.yaml`` plus our
  ``gripper_action_controller`` (task 1.2's GripperCommand action) — see that
  file's header for why it's a full copy instead of a merged overlay.
* ``gripper_action_controller`` added to the active-controllers spawn list.
* ``controller_spawner_timeout`` default raised 10s -> 120s (URSim's
  controller interface can take >10s after power-on — same fix
  ``ur7e_bringup`` already made for the bare-arm launch, same reasoning).
"""

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    kinematics_parameters_file = LaunchConfiguration("kinematics_parameters_file")

    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "onrobot_type:=",
            onrobot_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "name:=",
            "ur_onrobot",
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "kinematics_parameters_file:=",
            kinematics_parameters_file,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ur7e_pick_place_bringup"), "config", "ur7e_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_onrobot_description"), "rviz", "view_robot.rviz"]
    )

    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "config",
            ur_type.perform(context) + "_update_rate.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))
        ),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"robot_ip": robot_ip},
        ],
    )

    tool_communication_node = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": 54321,
                "device_name": "/tmp/ttyUR",
            }
        ],
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
        "gripper_action_controller",  # task 1.2 — our delta on the vendored list
    ]
    controllers_inactive = [
        "finger_width_controller",
        "finger_width_trajectory_controller",
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
    ]
    if activate_joint_controller.perform(context) == "true":
        controllers_active.append(initial_joint_controller.perform(context))
        controllers_inactive.remove(initial_joint_controller.perform(context))

    if use_fake_hardware.perform(context) == "true":
        controllers_active.remove("tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    nodes_to_start = [
        control_node,
        ur_control_node,
        dashboard_client_node,
        robot_state_helper_node,
        tool_communication_node,
        controller_stopper_node,
        urscript_interface,
        robot_state_publisher_node,
        rviz_node,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3", "ur3e", "ur5", "ur5e", "ur7e", "ur10", "ur10e", "ur16e", "ur20", "ur30",
            ],
            default_value="ur7e",
        ),
        DeclareLaunchArgument(
            "onrobot_type",
            description="Type/series of used OnRobot gripper.",
            choices=["rg2", "rg6"],
            default_value="rg2",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            description="Robot controller IP. Default is the lab robot; the "
            "URSim container uses the same address (sim/ursim/docker-compose.yml).",
            default_value="192.168.56.101",
        ),
        DeclareLaunchArgument(
            "kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur7e_bringup"), "config", "ur7e_calibration.yaml"]
            ),
            description="Factory kinematics calibration of OUR robot (task 0.4). "
            "Only override to bring up a different physical arm.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for multi-robot "
            "setups. If changed, joint names in the controllers config must match.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Sim tier 1: mocked hardware, no robot needed.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="See ur7e_bringup's launch file for the evaluation of "
            "this vs the pendant Play ritual — same trade-off applies here.",
        ),
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="120",
            description="Upstream default (10s) loses the race against a "
            "freshly booted URSim; see ur7e_bringup's launch file for the "
            "same fix and reasoning.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
            description="Initially loaded arm controller.",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate the initial arm controller.",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false",
            description="Headless by default, matching ur7e_bringup.",
        ),
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true",
            description="Launch the UR dashboard client (real robot / URSim only).",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
