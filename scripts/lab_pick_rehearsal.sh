#!/usr/bin/env bash
# Rehearse the whole language-directed pick without the lab (sim tier 1).
#
# What is real here: the intent parser, the orchestrator, the webcam
# perception node (fed a rendered image instead of a camera), MoveIt planning
# with KDL, and the trajectory path through scaled_joint_trajectory_controller.
# What is faked: the robot (ros2_control mock hardware), the gripper, and the
# three status topics the execution gate reads (safety mode, program running,
# speed scaling) -- mock hardware has no dashboard to publish them.
#
# It proves the software chain end to end and that the plan is reachable for
# an arm in the `ready` pose. It says nothing about grasp quality or
# calibration accuracy; those are lab results.
#
# Usage (inside the dev container, workspace built and sourced):
#   bash scripts/lab_pick_rehearsal.sh ["pick up the red block"]
set -eo pipefail
cd "$(dirname "$0")/.."
SENTENCE="${1:-pick up the red block}"
OUT="${REHEARSAL_OUTPUT:-/tmp/lab-pick-rehearsal}"
export ROS_DOMAIN_ID="${REHEARSAL_DOMAIN:-87}" ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
mkdir -p "$OUT"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
start() {  # start <logname> <command...>: own process group, so cleanup kills children
  local name="$1"; shift
  setsid "$@" >"$OUT/$name.log" 2>&1 &
  pids+=("$!")
}

python3 -m ur7e_perception.synthetic_table "$OUT/scene.png" "$OUT/table.json"

# Mock hardware starts upright; the OBSERVE stage swings it to `ready` first
# (a 3.6 rad wrist move, hence the wide --max-excursion below).
start driver ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true \
  launch_rviz:=false
sleep 8
start moveit ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl
start perception ros2 run ur7e_perception webcam_perception_node --ros-args \
  -p calibration:="$OUT/table.json" -p replay_image:="$OUT/scene.png" \
  -p backend:=color -p objects_file:="$PWD/scripts/lab_objects.yaml" \
  -p snapshot_dir:="$OUT/perception"
start status python3 scripts/rehearsal_status.py
sleep 10

python3 scripts/lab_pick.py --calibration "$OUT/table.json" --log-dir "$OUT/runs" \
  --say "$SENTENCE" --execute --yes --fake-gripper --max-speed-percent 100 \
  --max-excursion 4.0 --poses-file "$OUT/poses.json"
