#!/usr/bin/env bash
# Sim tier 1 smoke test (task 0.7): prove the bringup launch produces a working
# control stack with NO robot attached — mock hardware only.
#
# What it checks, and why these three things:
#   1. /joint_states publishes        -> the hardware interface is alive
#   2. scaled JTC reports "active"    -> the controller chain assembled
#   3. a FollowJointTrajectory goal   -> the full action path works end-to-end
#      succeeds                          (the same interface MoveIt uses later)
#
# Step 3 is the sim twin of lab session 2's "first commanded motion".
# Runs locally (see docs/SIMULATION.md) and in CI on every push.

# No `set -e`/`set -u`: we handle failures explicitly (cleanup must always
# run), and ROS setup.bash files reference unset variables, so -u breaks them.

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${TMPDIR:-/tmp}/tier1_bringup.log"
TIMEOUT_S=90

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS_ROOT/install/setup.bash"
else
    echo "FATAL: $WS_ROOT/install/setup.bash not found - run 'colcon build' first" >&2
    exit 1
fi

echo "== tier 1 smoke: launching bringup with mock hardware (log: $LOG)"
ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    # SIGINT first — `ros2 launch`'s clean Ctrl-C path. But don't trust it:
    # a launch process that ignores it would hang this script (and CI with
    # it, until the job timeout), so escalate to TERM/KILL after a grace
    # period. Observed in practice on the first container run.
    kill -INT "$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 10); do
        kill -0 "$LAUNCH_PID" 2>/dev/null || return 0
        sleep 1
    done
    kill -TERM "$LAUNCH_PID" 2>/dev/null
    sleep 3
    kill -KILL "$LAUNCH_PID" 2>/dev/null
    return 0
}

fail() {
    echo "== FAIL: $1" >&2
    echo "== last 40 lines of launch log:" >&2
    tail -40 "$LOG" >&2
    cleanup
    exit 1
}

echo "== waiting for scaled_joint_trajectory_controller to become active (max ${TIMEOUT_S}s)"
deadline=$((SECONDS + TIMEOUT_S))
until ros2 control list_controllers 2>/dev/null \
        | grep "scaled_joint_trajectory_controller" | grep -q "active"; do
    if [ "$SECONDS" -ge "$deadline" ]; then
        fail "controller did not become active within ${TIMEOUT_S}s"
    fi
    kill -0 "$LAUNCH_PID" 2>/dev/null || fail "launch process died during startup"
    sleep 2
done
echo "   controller active."

echo "== checking /joint_states publishes"
timeout 15 ros2 topic echo /joint_states --once >/dev/null \
    || fail "/joint_states did not publish within 15s"
echo "   joint states flowing."

echo "== sending a FollowJointTrajectory goal (the sim twin of lab session 2's first motion)"
# Single waypoint, 3s duration. Mock hardware starts at the description's
# initial positions; a modest absolute target keeps interpolation well inside
# the controller's 0.2 rad path tolerance (the one that aborts with -4 on
# real hardware, see RUNBOOK 4a).
GOAL='{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                  wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [
      { positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.05],
        time_from_start: { sec: 3 } }
    ]
  }
}'
RESULT=$(timeout 60 ros2 action send_goal \
    /scaled_joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "$GOAL" 2>&1)
echo "$RESULT" | grep -q "SUCCEEDED" \
    || { echo "$RESULT" >&2; fail "trajectory goal did not succeed"; }
echo "$RESULT" | grep -q "error_code: 0" \
    || { echo "$RESULT" >&2; fail "trajectory finished but error_code != 0"; }
echo "   trajectory SUCCEEDED (error_code 0)."

cleanup
echo "== tier 1 smoke test PASSED"
