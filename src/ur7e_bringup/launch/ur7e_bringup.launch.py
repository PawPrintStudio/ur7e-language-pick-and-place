"""
Single-command bringup for our UR7e (task 0.5).

This wraps the stock ``ur_control.launch.py`` from ``ur_robot_driver`` rather
than reimplementing it: upstream maintains the hard parts (controller spawning,
dashboard client, URScript resources), and we would only drift out of date by
copying them.  What this wrapper adds is *our* invariants:

* ``ur_type`` is pinned to ``ur7e`` — it is not an argument, because launching
  this repo against any other arm is always a mistake.
* ``kinematics_params_file`` defaults to the factory calibration extracted from
  our robot (lab session 1).  Lab session 2 proved why this must be baked in:
  without it the driver computes FK from the *default* UR7e kinematics and
  every TCP pose is silently wrong (checksum ``calib_127880...`` vs our robot's
  ``calib_124458...``).  A forgettable CLI flag is how that mistake happens.
* ``robot_ip`` defaults to the lab robot (192.168.56.101).  The URSim container
  is deliberately given the same IP on its Docker network (sim/ursim/), so
  tier-2 sim runs with identical arguments to the real lab.

Backends (architecture D7):
  real robot   ros2 launch ur7e_bringup ur7e_bringup.launch.py
  tier 1 mock  ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true
  tier 2 URSim ros2 launch ur7e_bringup ur7e_bringup.launch.py   # after docker compose up
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="Robot controller IP. Default is the lab robot; the URSim "
            "container uses the same address (sim/ursim/docker-compose.yml). "
            "Ignored when use_mock_hardware is true.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Sim tier 1: run the driver's description against mocked "
            "hardware — controllers and topics exist, no robot needed. Gates CI.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="false (default): the validated ritual — a person at the "
            "pendant starts the External Control program with Play, which is an "
            "intentional arming step. true: the driver injects and starts the "
            "URScript program itself (remote arming; requires Remote Control "
            "mode on the pendant). See the package README for the evaluation.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Headless by default: the Jetson runs this over SSH. "
            "Set true on a workstation to visualize.",
        ),
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="120",
            description="Upstream defaults to 10s, which loses the race when the "
            "controller hardware comes up slowly — observed against a freshly "
            "booted URSim (interface ready ~12s after driver start, spawners "
            "already dead). A patient timeout costs nothing when things are "
            "fast and saves the bringup when they are not.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur7e_bringup"), "config", "ur7e_calibration.yaml"]
            ),
            description="Factory kinematics calibration of OUR robot. Only "
            "override to bring up a different physical arm.",
        ),
    ]

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur7e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            # The Humble driver predates the upstream mock-hardware rename and
            # still calls this "fake". We expose the modern name (matching
            # ARCHITECTURE.md D7) and map it here; drop the mapping when we
            # move past Humble.
            "use_fake_hardware": LaunchConfiguration("use_mock_hardware"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "controller_spawner_timeout": LaunchConfiguration(
                "controller_spawner_timeout"
            ),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [ur_control_launch])
