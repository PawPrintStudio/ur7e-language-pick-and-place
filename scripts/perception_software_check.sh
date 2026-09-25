#!/usr/bin/env bash
# Reproducible Humble software gate; isolated build tree and ROS domain.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
export ROS_LOCALHOST_ONLY=1
build_dir="${PERCEPTION_BUILD_DIR:-/tmp/ur7e-perception-check}"
colcon build --build-base "$build_dir/build" --install-base "$build_dir/install" \
  --packages-select ur7e_interfaces ur7e_perception
source "$build_dir/install/setup.bash"
set -u
python3 -m pytest -q src/ur7e_perception/test
ros2 run ur7e_perception perception_eval --output "$build_dir/evaluation" \
  --export-frame "$build_dir/replay.npz"
setsid ros2 launch ur7e_perception perception.launch.py > "$build_dir/ros.log" 2>&1 &
launch_pid=$!
player_pid=''
cleanup() {
  if [[ -n "$player_pid" ]]; then
    kill -TERM -- "-$player_pid" 2>/dev/null || true
    wait "$player_pid" 2>/dev/null || true
  fi
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
python3 scripts/perception_ros_smoke.py
# Record canonical topics, stop the camera, and prove the same services work
# from recorded messages using the replay clock. Each run gets a new bag path.
bag_path="$build_dir/bag-$(date +%s)-$$"
set +e
timeout -s INT 3 bash scripts/perception_capture.sh record "$bag_path" > "$build_dir/record.log" 2>&1
record_status=$?
set -e
[[ "$record_status" == 124 || "$record_status" == 0 ]]
test -s "$bag_path/metadata.yaml"
cleanup
setsid ros2 launch ur7e_perception perception.launch.py \
  external_camera:=true use_sim_time:=true > "$build_dir/replay.log" 2>&1 &
launch_pid=$!
setsid ros2 bag play "$bag_path" --clock --rate 0.2 \
  --qos-profile-overrides-path "$(ros2 pkg prefix ur7e_perception)/share/ur7e_perception/config/bag_qos.yaml" \
  > "$build_dir/player.log" 2>&1 &
player_pid=$!
python3 scripts/perception_ros_smoke.py
echo 'PASS recorded RGB-D replay with camera stopped'
