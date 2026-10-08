#!/usr/bin/env bash
# The real-arm pipeline (scripts/lab_pick.py: language -> orchestrator -> perception
# -> MoveIt -> controller) end to end in Gazebo, with the Gazebo and RViz windows
# open so you can watch. Six scored scenarios, the same as lab_pick_rehearsal.sh,
# but checked against Gazebo's ground truth: the block must really rise and
# really end up where it should.
#
#   bash scripts/gazebo_pick_rehearsal.sh              # all six, windows open
#   GAZEBO_GUI=false bash scripts/gazebo_pick_rehearsal.sh     # headless
#   bash scripts/gazebo_pick_rehearsal.sh "pick up the red block"   # one sentence
#   REHEARSAL_BACKEND=claude bash scripts/gazebo_pick_rehearsal.sh  # Claude parser
#
# Run inside the devcontainer with the workspace built. Gazebo is left running
# at the end (KEEP_GAZEBO=false to stop it). Exit 0 only if every scenario passes.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${REHEARSAL_DOMAIN:-84}" ROS_LOCALHOST_ONLY=1
if [[ "$ROS_DOMAIN_ID" == 42 ]]; then echo "refusing to run in the lab domain 42" >&2; exit 2; fi
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
# Software OpenGL by default: with the GPU driver unavailable in the container
# ("MESA: failed to load driver: iris", 2026-10-08) Gazebo's camera sensor hangs
# the server at start-up. LIBGL_ALWAYS_SOFTWARE=0 to use the GPU where it works.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
GUI="${GAZEBO_GUI:-true}"
BACKEND="${REHEARSAL_BACKEND:-keyword}"
KEEP="${KEEP_GAZEBO:-true}"
OUT="/tmp/gazebo-pick-rehearsal/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT/runs"
echo "== logs: $OUT"

if pgrep -f "ign gazebo" >/dev/null; then
  echo "== Gazebo is already running. Stop it first (Ctrl+C its launch), then rerun." >&2
  exit 2
fi

pids=()
start() {  # start <name> <command...>: own process group, log to $OUT/<name>.log
  local name="$1"; shift
  setsid "$@" >"$OUT/$name.log" 2>&1 &
  pids+=("$!")
}
stop_all() {
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 3
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  pkill -f "ign gazebo" 2>/dev/null || true
}
cleanup() {
  [[ -n "${watch_pid:-}" ]] && kill "$watch_pid" 2>/dev/null || true
  if [[ "$KEEP" == true ]]; then
    echo "== Gazebo left running (KEEP_GAZEBO=true). Stop it with:  pkill -f 'ign gazebo'; pkill -f gazebo_demo; pkill -f rehearsal_status"
  else
    stop_all
  fi
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
  timeout 20 ros2 control list_controllers | grep joint_trajectory_controller | grep -q active
}
status_up() { timeout 20 ros2 node list | grep -qx /rehearsal_status; }

reset_block() {  # release the latch, then put the block back on its mark
  ros2 topic pub --times 3 /pick_object/detach std_msgs/msg/Empty "{}" >/dev/null 2>&1 || true
  sleep 1
  ign service -s /world/pick_place_table/set_pose --reqtype ignition.msgs.Pose \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req 'name: "pick_object", position: {x: 0.45, y: -0.15, z: 0.04}, orientation: {w: 1.0}' \
    >/dev/null
  sleep 2
}

echo "== starting Gazebo (GUI: $GUI), MoveIt, perception, RViz, fake robot status"
start gazebo ros2 launch ur7e_perception gazebo_demo.launch.py backend:=fixture \
  gazebo_gui:="$GUI" launch_rviz:="$GUI"
wait_until 180 "Gazebo trajectory controller active" gazebo controller_active
wait_until 120 "MoveIt (/compute_cartesian_path)" gazebo has_service /compute_cartesian_path
wait_until 120 "perception (/perception/detect_object)" gazebo has_service /perception/detect_object
start status python3 scripts/rehearsal_status.py
wait_until 60 "fake robot status" status status_up
echo "   waiting 12 s for the world's start-up latch release"
sleep 12

# expected outcome | expected reason | ground-truth check | extra args | sentence
SCENE_WS=(0.25 -0.35 0.70 0.35)
LAB_WS=(0.14 -0.07 0.30 0.22)
if [[ $# -gt 0 ]]; then
  SCENARIOS=("succeeded|ok|lifted||$1")
else
  SCENARIOS=(
    "succeeded|ok|lifted_back||pick up the red block"
    "succeeded|ok|on_target||put the red block on the blue block"
    "failed|not_found|untouched||pick up the green block"
    "refused|action_not_allowed|untouched||go up 2 cm"
    "refused|target_not_noun_phrase|untouched||pick up the red block and throw it"
    "failed|outside_workspace|untouched|lab|pick up the red block"
  )
fi

check_truth() {  # check_truth <kind> <watch.json>: print PASS/FAIL reason
  python3 - "$1" "$2" <<'EOF'
import json, math, sys
kind, path = sys.argv[1], sys.argv[2]
r = json.load(open(path))
if not r['final']:
    print('FAIL no block pose recorded'); sys.exit()
x, y, z = r['final']; top = r['max_z']
start, target = (0.45, -0.15), (0.45, 0.20)
lifted = top is not None and top > 0.12
msg = f'block max z {top:.3f} m, ended at ({x:.3f}, {y:.3f}, {z:.3f})'
if kind == 'lifted':
    ok = lifted
elif kind == 'lifted_back':
    ok = lifted and math.dist((x, y), start) < 0.03 and z < 0.06
elif kind == 'on_target':
    ok = lifted and math.dist((x, y), target) < 0.04 and z < 0.07
else:  # untouched
    ok = not lifted and math.dist((x, y), start) < 0.01
print(('PASS ' if ok else 'FAIL ') + msg)
EOF
}

results=()
n=0
for line in "${SCENARIOS[@]}"; do
  IFS='|' read -r want_outcome want_reason truth extra sentence <<<"$line"
  n=$((n + 1))
  ws=("${SCENE_WS[@]}"); [[ "$extra" == lab ]] && ws=("${LAB_WS[@]}")
  echo
  echo "== [$n/${#SCENARIOS[@]}] \"$sentence\"  (expect $want_outcome / $want_reason; block: $truth)"
  reset_block
  python3 scripts/gazebo_block_watch.py "$OUT/watch-$n.json" &
  watch_pid=$!
  sleep 2
  log="$OUT/scenario-$n.log"
  set +e
  python3 scripts/lab_pick.py --sim gazebo --execute --yes --backend "$BACKEND" \
    --calibration scripts/gazebo_table.json --poses-file scripts/gazebo_poses.json \
    --workspace "${ws[@]}" --max-speed-percent 100 --max-excursion 4.0 \
    --log-dir "$OUT/runs" --say "$sentence" 2>&1 | grep --line-buffered -v multicast | tee "$log"
  set -e
  sleep 2
  kill "$watch_pid" 2>/dev/null || true; wait "$watch_pid" 2>/dev/null || true; watch_pid=""
  result=$(grep '^RESULT' "$log" | tail -1 | sed 's/^RESULT *//')
  got=$(python3 -c "import json,sys; r=json.loads(sys.argv[1]); print(r['outcome'], r['reason'])" "$result" 2>/dev/null || echo "none none")
  truth_line=$(check_truth "$truth" "$OUT/watch-$n.json")
  if [[ "$got" == "$want_outcome $want_reason" && "$truth_line" == PASS* ]]; then
    verdict=PASS
  else
    verdict=FAIL
  fi
  echo "   $verdict  got ${got/ / / }   |   Gazebo: ${truth_line#* }"
  results+=("$verdict  [$n] \"$sentence\"  ->  ${got/ / / }  |  ${truth_line#* }")
done

echo
echo "== summary ($BACKEND parser, logs in $OUT)"
failures=0
for r in "${results[@]}"; do echo "   $r"; [[ "$r" == FAIL* ]] && failures=$((failures + 1)); done
exit $(( failures > 0 ))
