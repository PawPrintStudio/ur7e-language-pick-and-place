"""
MoveIt2 for the combined UR7e + RG2 (task 1.4). Built with
``moveit_configs_utils.MoveItConfigsBuilder`` — the standard modern way to
assemble a MoveIt launch from files that live in more than one package —
rather than forking the vendored ``ur_onrobot_moveit_config`` launch file the
way ``ur7e_pick_place.launch.py`` forks ``start_robot.launch.py``. That fork
was necessary there because two internal paths had no override argument at
all; here, ``MoveItConfigsBuilder`` already gives us a clean per-file
override, so there's nothing to fork. Also a better one to read to learn the
current MoveIt2 idiom (the vendored file predates ``MoveItConfigsBuilder``
being the recommended pattern).

Our deltas on the vendored MoveIt config (all pulled in below by file path):

* ``robot_description_semantic`` -> our ``ur7e_pick_place.srdf.xacro``
  (includes the vendored SRDF unchanged, adds the ``observe`` named pose).
* ``robot_description_kinematics`` -> our ``ur7e_kinematics.yaml``
  (``pick_ik``, task 1.4's decided solver, D6).
* Everything else (joint limits, trajectory-execution controllers, RViz
  config) is reused unmodified from ``ur_onrobot_moveit_config`` — no reason
  to fork what isn't changing. The OMPL pipeline config isn't reused at all:
  we don't override it, so ``MoveItConfigsBuilder`` falls back to its own
  generic default (a deliberate simplification — no delta on any of D6's
  decisions to preserve there).

``MoveItConfigsBuilder``'s ``file_path`` arguments are resolved as plain
``pathlib.Path`` joins at graph-construction time (see its own module
docstring), not launch-time substitutions — that's why this file wraps
everything in ``launch_setup``/``OpaqueFunction`` and calls ``.perform(context)``
on the launch arguments that affect *which file* gets loaded, rather than
handing it ``PathJoinSubstitution``/``LaunchConfiguration`` objects directly
the way a plain ``Node(parameters=[...])`` call could.

Also starts ``planning_scene.py`` (this package): the table + demo object
collision geometry (task 1.3's table collision, and the hardcoded poses
task 1.7's scripted pick will use), added as MoveIt ``CollisionObject``s at
startup rather than baked into the URDF — easy to move without touching the
robot description, and it's how a real deployment adds workspace furniture
anyway (the geometry isn't rigidly attached to the robot).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    kinematics_parameters_file = LaunchConfiguration("kinematics_parameters_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Files we reuse unmodified from the vendored MoveIt config (D6) —
    # resolved to an absolute path so MoveItConfigsBuilder (constructed
    # against OUR package, below) can still reach them.
    vendored_moveit_share = get_package_share_directory("ur_onrobot_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("ur_onrobot", package_name="ur7e_pick_place_bringup")
        .robot_description(
            file_path="urdf/ur7e_pick_place.urdf.xacro",
            mappings={
                "ur_type": ur_type,
                "onrobot_type": onrobot_type,
                "tf_prefix": tf_prefix,
                "use_fake_hardware": "true",
                "kinematics_parameters_file": kinematics_parameters_file,
            },
        )
        .robot_description_semantic(
            file_path="srdf/ur7e_pick_place.srdf.xacro",
            mappings={"name": "ur_onrobot", "prefix": tf_prefix},
        )
        .robot_description_kinematics(file_path="config/ur7e_kinematics.yaml")
        .joint_limits(file_path=os.path.join(vendored_moveit_share, "config", "joint_limits.yaml"))
        .trajectory_execution(
            file_path=LaunchConfiguration("moveit_controllers_file").perform(context)
        )
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        arguments=["-d", os.path.join(vendored_moveit_share, "rviz", "view_robot.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    planning_scene_node = Node(
        package="ur7e_pick_place_bringup",
        executable="planning_scene",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Placeholder overhead-camera TF (task 1.3) — exact mount offset is
    # undecided until task 2.1 (hardware). Kept obviously provisional rather
    # than silently baked into the URDF; move this into ur7e_description once
    # 2.1 measures the real mount.
    camera_placeholder_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_placeholder_tf",
        # xyz (metres) rpy (radians): 1 m over the table centre, looking down.
        # PLACEHOLDER — see task 2.1.
        arguments=["0", "0", "1.0", "0", "1.5708", "0", "base_link", "camera_placeholder"],
    )

    return [move_group_node, rviz_node, planning_scene_node, camera_placeholder_tf]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur7e"),
        DeclareLaunchArgument("onrobot_type", default_value="rg2"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument(
            "kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur7e_bringup"), "config", "ur7e_calibration.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true",
            description="Workstation default true (unlike the headless arm "
            "bringup) — MoveIt's RViz motion-planning plugin is the primary "
            "way to develop and sanity-check plans.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "moveit_controllers_file",
            default_value="config/ur7e_moveit_controllers.yaml",
            description="Path (relative to this package) to MoveIt's "
            "trajectory-execution controllers config. Tier 1/2 (real driver "
            "controller names) use the default; tier 3 (Gazebo) passes "
            "config/ur7e_gz_moveit_controllers.yaml — see ur7e_gazebo/README.md "
            "for why the controller name differs (unscaled vs scaled JTC).",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
