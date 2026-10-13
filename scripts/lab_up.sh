#!/usr/bin/env bash
# The proven real-arm flow (docs/LAB_2026-10-08_REAL_E2E.md) in one command:
# driver -> MoveIt (KDL) -> perception -> lab_console --pick, each started in
# the background and waited on until it is really ready, then the console in
# the foreground. Quit the console (or Ctrl+C) and everything it started stops.
#
# Run it twice, once per side:
#   bash scripts/lab_up.sh        # on the HOST: link up, API key, start VS Code
#   bash scripts/lab_up.sh        # in the DEV CONTAINER: the whole stack + console
#
#   LAB_SPEED=25 bash scripts/lab_up.sh                # console speed cap (default 100: the pendant slider is the speed)
#   bash scripts/lab_up.sh --backend keyword           # extra args go to lab_console
#   ZED_DEVICE=2 bash scripts/lab_up.sh                # skip the ZED auto-detect
#   KEEP_STACK=true bash scripts/lab_up.sh             # leave driver/MoveIt/perception up
#   CHESS=true bash scripts/lab_up.sh                  # driver + MoveIt only, no camera; Ctrl+C stops them
#
# It assumes nothing moved since 10-08: lab_table.json (camera pose),
# lab_tooling.yaml (verified) and .console_poses.json (taught observe) are
# reused as they are. It never presses Play for you and never bypasses a check.
set -eo pipefail
cd "$(dirname "$0")/.."
ROBOT_IP=192.168.56.101
# The container has no ping. Opening the dashboard port (29999) works on
# both sides and also proves the controller itself is answering.
robot_reachable() { timeout 3 bash -c "exec 3<>/dev/tcp/$ROBOT_IP/29999" 2>/dev/null; }

# ---------------------------------------------------------------- host side
if [[ ! -f /.dockerenv ]]; then
  echo "== host: bringing up the robot link"
  nmcli con up ur-link
  robot_reachable || { echo "== no reply from $ROBOT_IP: check the cable and the pendant network" >&2; exit 2; }
  echo "   robot answers at $ROBOT_IP"
  # VS Code hands the folder to an already-running window, which keeps the
  # environment it started with: the API key would never reach the container.
  if pgrep -x code >/dev/null; then
    echo "== VS Code is already running. Quit it completely (all windows), then rerun this." >&2
    exit 2
  fi
  [[ -f .env ]] && { set -a; source .env; set +a; }
  [[ -n "${ANTHROPIC_API_KEY:-}" ]] || echo "   warning: no ANTHROPIC_API_KEY; only --backend keyword will work"
  code .
  echo "== in VS Code: Reopen in Container, then in its terminal:  bash scripts/lab_up.sh"
  exit 0
fi

# ----------------------------------------------------------- container side
# The host's .venv (Python 3.12) gets auto-activated in container terminals
# too; ROS here is 3.10. Drop it from PATH for everything this script starts.
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(tr ':' '\n' <<<"$PATH" | grep -vxF "$VIRTUAL_ENV/bin" | paste -sd:)"
  unset VIRTUAL_ENV
  echo "   ignoring the active .venv (host Python) for this run"
fi
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/lab_env.sh
set -u
SPEED="${LAB_SPEED:-100}"
KEEP="${KEEP_STACK:-false}"
CHESS="${CHESS:-false}"
OUT="log/lab/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
echo "== logs: $OUT"

robot_reachable || { echo "== no reply from $ROBOT_IP: on the host, run  nmcli con up ur-link" >&2; exit 2; }
# VS Code only forwards the key when it was started from a shell that had it;
# the repo's .env is mounted here too, so read it directly when it's missing.
if [[ -z "${ANTHROPIC_API_KEY:-}" && -f .env ]]; then set -a; source .env; set +a; fi
[[ -n "${ANTHROPIC_API_KEY:-}" ]] || echo "   warning: no ANTHROPIC_API_KEY in the container; --backend claude will fail"

# The ZED moves between /dev/video0 and video2 on replug. Find its capture
# node (index 0) by name rather than trusting the number in lab_table.json.
# Chess plays from a square table and needs no camera.
if [[ "$CHESS" != true ]]; then
  zed="${ZED_DEVICE:-}"
  if [[ -z "$zed" ]]; then
    for d in /sys/class/video4linux/video*; do
      if grep -q ZED "$d/name" 2>/dev/null && [[ "$(cat "$d/index")" == 0 ]]; then zed="${d##*video}"; break; fi
    done
  fi
  [[ -n "$zed" && -e "/dev/video$zed" ]] || { echo "== no ZED camera found: replug it into a USB-3 port (ls /dev/video*)" >&2; exit 2; }
  echo "   ZED on /dev/video$zed"
fi

# Two drivers means two controller managers fighting over the arm.
if pgrep -f "ur7e_bringup.launch.py|lab_arm_moveit.launch.py|webcam_perception_node" >/dev/null; then
  echo "== part of the stack is already running:" >&2
  pgrep -af "ur7e_bringup.launch.py|lab_arm_moveit.launch.py|webcam_perception_node" >&2
  echo "   stop it first (Ctrl+C its terminal, or pkill -f <name>), then rerun." >&2
  exit 2
fi

pids=()
start() {  # start <name> <command...>: own process group, log to $OUT/<name>.log
  local name="$1"; shift
  setsid "$@" >"$OUT/$name.log" 2>&1 &
  pids+=("$!")
}
cleanup() {
  if [[ "$KEEP" == true ]]; then
    echo "== stack left running (KEEP_STACK=true). Stop it with:  pkill -INT -f ur7e_bringup.launch.py; pkill -INT -f lab_arm_moveit.launch.py; pkill -INT -f webcam_perception_node"
    return
  fi
  echo "== stopping perception, MoveIt, driver"
  for (( i=${#pids[@]}-1; i>=0; i-- )); do kill -INT -- "-${pids[i]}" 2>/dev/null || true; done
  sleep 3
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT

wait_until() {  # wait_until <seconds> <what> <log> <test...>
  local seconds="$1" what="$2" log="$3"; shift 3
  local deadline=$((SECONDS + seconds))
  until "$@" >/dev/null 2>&1; do
    if (( SECONDS >= deadline )); then
      echo "== TIMEOUT after ${seconds}s waiting for $what. Last lines of $OUT/$log.log:" >&2
      tail -n 25 "$OUT/$log.log" >&2; exit 3
    fi
    sleep 2
  done
  echo "   ready: $what"
}
has_service() { timeout 20 ros2 service list | grep -qx "$1"; }
controller_active() {
  timeout 20 ros2 control list_controllers | grep scaled_joint_trajectory_controller | grep -q active
}
in_log() { grep -q "$2" "$OUT/$1.log"; }

echo "== 1/4 driver"
start driver ros2 launch ur7e_bringup ur7e_bringup.launch.py launch_rviz:=false
wait_until 90 "scaled_joint_trajectory_controller active" driver controller_active

echo
echo "   >>> Pendant: set the speed slider (it is the speed; cap ${SPEED} %), then press PLAY. <<<"
echo
wait_until 180 "pendant Play (reverse interface connected)" driver \
  in_log driver "Robot connected to reverse interface"

echo "== 2/4 MoveIt (KDL)"
start moveit ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl
wait_until 120 "MoveIt (/compute_cartesian_path)" moveit has_service /compute_cartesian_path

if [[ "$CHESS" == true ]]; then
  echo "== chess stack up. In another terminal:  python3 scripts/lab_chess.py ...  (docs/CHESS.md)"
  echo "   Ctrl+C here stops MoveIt and the driver."
  wait -n "${pids[@]}" || true   # returns on Ctrl+C or when either one dies
  exit 0
fi

echo "== 3/4 perception (OWLv2 load takes a while on CPU)"
start perception env OMP_NUM_THREADS=4 ros2 run ur7e_perception webcam_perception_node --ros-args \
  -p device:="$zed" \
  -p calibration:="$PWD/scripts/lab_table.json" -p objects_file:="$PWD/scripts/lab_objects.yaml" \
  -p snapshot_dir:="$PWD/log/perception" -p backend:=auto -p reach_min:=0.08 -p reach_max:=0.5 \
  -p default_height:=-1.0
wait_until 300 "perception (/perception/detect_object)" perception has_service /perception/detect_object

echo "== 4/4 console  (objects on the plate beside the board, never on it; hand near the E-stop)"
set +e
python3 scripts/lab_console.py --pick --execute --max-speed-percent "$SPEED" --backend claude \
  --joint-rate 0.09 --max-excursion 2.5 "$@"
status=$?
set -e
exit "$status"
