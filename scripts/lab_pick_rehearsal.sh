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
# It proves the software chain end to end and that the plans are reachable for
# an arm in the `ready` pose. It says nothing about grasp quality or
# calibration accuracy; those are lab results.
#
# By default it runs a small suite: each sentence below goes through
# scripts/lab_pick.py in its own process, and the run's outcome and reason
# code are compared with what the workflow is supposed to do. The happy path
# alone is not a test of a pick workflow -- refusals, retries and the
# workspace gate are where it earns its keep.
#
# Usage (inside the dev/lab container, workspace built and sourced):
#   bash scripts/lab_pick_rehearsal.sh                          # the suite
#   bash scripts/lab_pick_rehearsal.sh "pick up the red block"  # one sentence, must succeed
#   REHEARSAL_BACKEND=claude bash scripts/lab_pick_rehearsal.sh # LLM parser (needs ANTHROPIC_API_KEY)
#   REHEARSAL_KEEP=1 bash scripts/lab_pick_rehearsal.sh         # leave the stack up afterwards
#
# Exit code: 0 only if every scenario ended as expected.
set -eo pipefail
cd "$(dirname "$0")/.."

BACKEND="${REHEARSAL_BACKEND:-keyword}"
OUT="${REHEARSAL_OUTPUT:-/tmp/lab-pick-rehearsal}/$(date +%Y%m%d-%H%M%S)"
export ROS_DOMAIN_ID="${REHEARSAL_DOMAIN:-87}" ROS_LOCALHOST_ONLY=1
if [[ "$ROS_DOMAIN_ID" == 42 ]]; then
  echo "refusing to rehearse in the lab domain (42): the fake status node would" >&2
  echo "tell a real robot's gate that everything is fine" >&2
  exit 2
fi
# Same participant-index range as scripts/lab_env.sh: the driver launch alone
# starts ~8 DDS participants, and with Cyclone's default range the 11th
# process on this machine (here: perception, status, lab_pick) fails with
# "Failed to find a free participant index".
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
if [[ "$BACKEND" == claude && -z "${ANTHROPIC_API_KEY:-}" ]]; then
  echo "REHEARSAL_BACKEND=claude needs ANTHROPIC_API_KEY in this shell" >&2
  exit 2
fi
mkdir -p "$OUT"
echo "== logs: $OUT"

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
wait_until() {  # wait_until <seconds> <what> <log name> <test command...>
  local seconds="$1" what="$2" log="$3"; shift 3
  local deadline=$((SECONDS + seconds))
  until "$@" >/dev/null 2>&1; do
    if (( SECONDS >= deadline )); then
      echo "== TIMEOUT after ${seconds}s waiting for $what. Last lines of $OUT/$log.log:" >&2
      tail -n 25 "$OUT/$log.log" >&2
      exit 3
    fi
    sleep 2
  done
  echo "   ready: $what"
}
controller_active() {
  ros2 control list_controllers | grep scaled_joint_trajectory_controller | grep -q active
}
has_service() { ros2 service list | grep -qx "$1"; }
# The gate needs the fake status before the first pick (2026-10-08: scenario 1
# raced it and aborted). Its topics are latched, so the node being up is enough.
status_up() { ros2 node list | grep -qx /rehearsal_status; }

# --- the scene ---------------------------------------------------------------------------
# A rendered webcam view of the table with a red block at (-0.42, 0.22) and a
# blue block at (-0.50, 0.02) m, plus the synthetic calibration that matches it.
python3 -m ur7e_perception.synthetic_table "$OUT/scene.png" "$OUT/table.json" \
  | tee "$OUT/scene.json"
# lab_pick's default workspace gate is the lab's raised plate (x 0.14..0.30),
# where the synthetic blocks are not. This rectangle covers the synthetic
# table; the `outside_workspace` scenario below checks the lab gate itself.
SCENE_WORKSPACE=(-0.60 -0.10 -0.30 0.35)
LAB_WORKSPACE=(0.14 -0.07 0.30 0.22)

# --- the stack ---------------------------------------------------------------------------
echo "== starting mock-hardware driver, MoveIt, perception, fake status"
# Mock hardware starts upright; the OBSERVE stage swings it to `ready` first
# (a 3.6 rad wrist move, hence the wide --max-excursion below).
start driver ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true \
  launch_rviz:=false
wait_until 90 "scaled_joint_trajectory_controller active" driver controller_active
start moveit ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl
start perception ros2 run ur7e_perception webcam_perception_node --ros-args \
  -p calibration:="$OUT/table.json" -p replay_image:="$OUT/scene.png" \
  -p backend:=color -p objects_file:="$PWD/scripts/lab_objects.yaml" \
  -p snapshot_dir:="$OUT/perception"
start status python3 scripts/rehearsal_status.py
wait_until 90 "MoveIt (/apply_planning_scene)" moveit has_service /apply_planning_scene
wait_until 90 "MoveIt (/compute_cartesian_path)" moveit has_service /compute_cartesian_path
wait_until 60 "perception (/perception/detect_object)" perception \
  has_service /perception/detect_object
wait_until 60 "fake robot status (safety mode)" status status_up

# --- the scenarios -----------------------------------------------------------------------
# expected outcome | expected reason | extra lab_pick arguments | sentence
if [[ $# -gt 0 ]]; then
  SCENARIOS=("succeeded|ok||$1")
else
  SCENARIOS=(
    "succeeded|ok||pick up the red block"
    "succeeded|ok||put the red block on the blue block"
    # Nothing green in the scene: DETECT misses, the workflow re-observes once,
    # misses again, sends the arm home and reports not_found.
    "failed|not_found||pick up the green block"
    # A jog is a valid sentence but not a pick: refused before any motion.
    "refused|action_not_allowed||go up 2 cm"
    # Not a noun phrase the detector could look for: refused before any motion.
    "refused|target_not_noun_phrase||pick up the red block and throw it"
    # The lab's own workspace gate: the synthetic block is outside the lab
    # plate, so LOCATE must refuse it and the arm must go home.
    "failed|outside_workspace|--workspace ${LAB_WORKSPACE[*]}|pick up the red block"
  )
fi

passed=0
failed=0
summary="$OUT/summary.txt"
: >"$summary"
n=0
for scenario in "${SCENARIOS[@]}"; do
  n=$((n + 1))
  IFS='|' read -r want_outcome want_reason extra sentence <<<"$scenario"
  # shellcheck disable=SC2206  # word-split the extra arguments on purpose
  extra_args=($extra)
  if [[ ${#extra_args[@]} -eq 0 ]]; then
    extra_args=(--workspace "${SCENE_WORKSPACE[@]}")
  fi
  log="$OUT/scenario-$n.log"
  echo
  echo "== [$n/${#SCENARIOS[@]}] \"$sentence\"  (expect $want_outcome / $want_reason)"
  set +e
  python3 scripts/lab_pick.py --calibration "$OUT/table.json" --log-dir "$OUT/runs" \
    --backend "$BACKEND" --say "$sentence" --execute --yes --fake-gripper --allow-unverified-tooling \
    --max-speed-percent 100 --max-excursion 4.0 --poses-file "$OUT/poses.json" \
    "${extra_args[@]}" 2>&1 | tee "$log"
  set -e
  # lab_pick prints one "RESULT {json}" line per sentence.
  verdict=$(python3 - "$log" "$want_outcome" "$want_reason" <<'PY'
import json, sys
log, want_outcome, want_reason = sys.argv[1:4]
lines = [line for line in open(log) if line.startswith('RESULT')]
if not lines:
    blocked = [line.strip() for line in open(log) if line.startswith('BLOCKED')]
    print('FAIL no RESULT line' + (f' ({blocked[-1]})' if blocked else ' (crashed? see log)'))
    sys.exit()
result = json.loads(lines[-1].split(None, 1)[1])
got = f"{result['outcome']} / {result['reason']}"
ok = result['outcome'] == want_outcome and result['reason'] == want_reason
print(f"{'PASS' if ok else 'FAIL'} got {got} in {result['duration_s']} s, "
      f"retries {result['retries']}, run {result['run_id']}")
PY
)
  echo "   $verdict"
  printf '%-4s  %-55s  %s\n' "${verdict%% *}" "\"$sentence\"" "${verdict#* }" >>"$summary"
  if [[ "$verdict" == PASS* ]]; then passed=$((passed + 1)); else failed=$((failed + 1)); fi
done

echo
echo "== summary ($BACKEND backend): $passed passed, $failed failed"
cat "$summary"
echo "== run logs (one JSONL per run): $OUT/runs   perception snapshots: $OUT/perception"

if [[ -n "${REHEARSAL_KEEP:-}" ]]; then
  echo
  echo "== REHEARSAL_KEEP: the stack stays up. In another container shell:"
  echo "   export ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI='$CYCLONEDDS_URI'"
  echo "   python3 scripts/lab_pick.py --calibration $OUT/table.json --log-dir $OUT/runs \\"
  echo "     --workspace ${SCENE_WORKSPACE[*]} --execute --yes --fake-gripper --allow-unverified-tooling \\"
  echo "     --max-speed-percent 100 --max-excursion 4.0 --poses-file $OUT/poses.json \\"
  echo "     --say \"pick up the blue block\""
  echo "   Ctrl-C here stops everything."
  wait
fi
[[ $failed -eq 0 ]]
