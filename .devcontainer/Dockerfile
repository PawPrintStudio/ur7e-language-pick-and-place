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
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    python3-opencv \
    python3-numpy \
    python3-pytest \
    xvfb \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    gdb \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# pip-only dependencies, learned in the lab 2026-09-30 (RUNBOOK):
#   anthropic  - the language backend's SDK; no rosdep key, so not in any
#                package.xml (see arm_language/backends/claude.py).
#   brotli     - Ubuntu 22.04's apt python3-brotli 1.0.9 makes the SDK's HTTP
#                client fail to decompress responses ("Connection error",
#                cause "process() takes no keyword arguments"); >=1.1 fixes it.
#   pytest>=7  - the SDK's anyio pulls a pytest plugin that needs >=7; apt's
#                6.2.5 then fails to even collect ("No module _pytest.scope").
# Installed system-wide here (before USER switches) so every user in the
# image has them; --break-system-packages is not needed on 22.04.
RUN pip3 install --no-cache-dir "anthropic>=1.10" "brotli>=1.1" "pytest>=7,<9"

# Real-hardware perception (2026-10-08): the webcam node's default
# `backend:=auto` loads OWLv2 with no fallback, so without these it crashes
# at start; the colour backend alone cannot find "headphones".
#   torch (CPU wheel)  - the laptop drives the ZED; no CUDA in the container.
#   transformers<5     - Owlv2Processor.post_process_grounded_object_detection
#                        as webcam_node.py calls it.
#   numpy<2            - apt's python3-opencv and cv_bridge are built against
#                        numpy 1.x; a 2.x pulled in by pip breaks both imports.
#   pillow>=9.1        - apt's Pillow 9.0.1 satisfies a bare "pillow", but
#                        transformers' OWLv2 needs Image.Resampling (9.1+);
#                        without the bound the import fails (2026-10-08).
RUN pip3 install --no-cache-dir --extra-index-url https://download.pytorch.org/whl/cpu \
    torch "transformers>=4.40,<5" "pillow>=9.1" "numpy<2"

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

# Mount point for the `ur7e-hf-cache` volume (devcontainer.json): created here
# so the volume is owned by this user, and OWLv2's ~600 MB of weights survive
# container rebuilds instead of downloading again.
RUN mkdir -p /home/$USERNAME/.cache/huggingface

# Shell environment for every terminal in the container.
# ROS_DOMAIN_ID + RMW match the Jetson (RUNBOOK, SSH path section): a laptop
# on the Jetson's hotspot sees its topics without any per-shell exports.
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo '[ -f /workspaces/*/install/setup.bash ] && source /workspaces/*/install/setup.bash' >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc \
    && echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
