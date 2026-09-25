#!/usr/bin/env bash
# Full simulator regression; run in the devcontainer with a built workspace.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source "${PERCEPTION_INSTALL:-install}/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
export ROS_LOCALHOST_ONLY=1
output="${PERCEPTION_OUTPUT:-/tmp/ur7e-gazebo-perception}"
mkdir -p "$output"
launcher=()
if [[ -z "${DISPLAY:-}" ]]; then
  launcher=(xvfb-run -a)
  export LIBGL_ALWAYS_SOFTWARE=1
fi
setsid "${launcher[@]}" ros2 launch ur7e_perception gazebo_demo.launch.py \
  backend:="${PERCEPTION_BACKEND:-fixture}" > "$output/launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
# Base world performs its initial detach/reset at 8/9 seconds.
sleep 12
python3 scripts/perception_ros_smoke.py | tee "$output/services.log"
python3 scripts/perception_pick_demo.py --execute-simulation | tee "$output/pick.log"
