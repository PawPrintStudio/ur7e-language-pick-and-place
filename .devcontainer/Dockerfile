# Devcontainer image (task 0.8): the full dev stack for any member, from
# `git clone`. Based on osrf/ros:humble-desktop-full (athackst pattern) —
# desktop-full brings RViz and the GUI stack that plain ros:humble (CI image)
# deliberately lacks.
FROM osrf/ros:humble-desktop-full

# Project stack, mirroring the Jetson install (RUNBOOK B3) plus sim extras:
#   ros-humble-ur              driver + calibration + moveit config (Jetson)
#   ros-humble-moveit          motion planning (phase 1)
#   ros-humble-pick-ik         phase-1 IK solver (D6/ARCHITECTURE.md task 1.4)
#   ros-humble-ros-gz          Gazebo Fortress bridge (sim tier 3)
#   ros-humble-ur-simulation-gz  Gazebo-ready UR7e (gz_ros2_control), the arm
#                              base for tier 3 — see ur7e_gazebo/README.md
#   ros-humble-gz-ros2-control   sim ros2_control plugin the RG2 gripper's
#                              sim joints are wired under (tier 3)
#   libnet1-dev                Modbus RTU/TCP dep for the vendored gripper
#                              hardware_interface (src/vendor, ur7e.repos) —
#                              only needed to build it, not to exercise it in
#                              sim
#   rmw-cyclonedds-cpp         the RMW the Jetson runs — same middleware everywhere
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ur \
    ros-humble-moveit \
    ros-humble-pick-ik \
    ros-humble-ros-gz \
    ros-humble-ur-simulation-gz \
    ros-humble-gz-ros2-control \
    libnet1-dev \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    gdb \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Non-root user: files created in the bind-mounted workspace stay owned by
# you, not root (the classic Docker-on-Linux paper cut).
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

# Shell environment for every terminal in the container.
# ROS_DOMAIN_ID + RMW match the Jetson (RUNBOOK, SSH path section): a laptop
# on the Jetson's hotspot sees its topics without any per-shell exports.
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo '[ -f /workspaces/*/install/setup.bash ] && source /workspaces/*/install/setup.bash' >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc \
    && echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
