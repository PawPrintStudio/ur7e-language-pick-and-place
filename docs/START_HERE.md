# START HERE: running the natural-language pick yourself

One page that explains what the system is, which environment to use, and every
command needed to run "pick up the red block" end to end in simulation, with
the reason for each. Everything here was run on 2026-10-07 in the VS Code
devcontainer. The other docs are reference material or dated session logs;
the map at the bottom says which to open when.

## 1. What the system does, in five layers

```
"pick up the red block"
  ① arm_language        sentence → structured command {action: pick, target_query: "red block"}
                         (keyword parser offline, or Claude with an API key)
  ② ur7e_orchestrator   the state machine: PARSE → OBSERVE → DETECT → LOCATE → PLAN
                         → APPROACH → GRASP → LIFT → RETREAT → HOME, with retries and safe aborts
  ③ ur7e_perception     image → "red block at (x, y, z)" in the robot's base frame
                         (colour backend in sim; OWLv2 open-vocabulary in the lab)
  ④ MoveIt (KDL IK)     target pose → collision-free joint trajectory
  ⑤ ros2_control        trajectory → joints (real UR driver in the lab, mock hardware in sim)
```

`scripts/lab_pick.py` is the program that ties ①–⑤ together. It is the same
program in sim and in the lab; only what sits underneath changes.

Each layer talks to the next only through ROS topics, services and actions.
That is why a still image can stand in for the camera, and mock joints for the
robot, without changing `lab_pick.py`.

## 2. The three simulation tiers, and what each one proves

| Tier | Underneath | You see | Proves | Does not prove |
|---|---|---|---|---|
| **1: mock hardware** (this guide) | joints echo commands; camera = rendered still image; `FakeRg2` gripper | the arm moving in RViz, **no objects** | parsing, the state machine, perception maths, every plan is reachable | grasp, physics, calibration |
| 2: URSim | UR's own controller simulator | the UR teach-pendant UI | driver handshake, protective-stop handling | perception, grasp |
| 3: Gazebo | physics + rendered RGB-D camera | arm, table and blocks in 3D | a camera render and objects that move | grasp quality (the latch is a stand-in) |

Rule: **sim never signs off grasp quality or calibration.** Only the real arm does.

Why RViz is empty in tier 1: the blocks exist only in `/tmp/sim/scene.png` and
its calibration `table.json`. Nothing publishes them to RViz. The arm still
goes to the located point. Add a **TF** display in RViz and watch `tool0`,
or run `ros2 run tf2_ros tf2_echo base_link tool0`.

## 3. The environment: the VS Code devcontainer

| Use | When |
|---|---|
| **Devcontainer** (`.devcontainer/`) | Sim and everyday development. Built from the repo's Dockerfile: ROS Humble + UR + MoveIt, with `anthropic`/`pytest` preinstalled |
| `docker run … ur7e-task2:visual` ([COLD_START.md](COLD_START.md)) | Lab days: needs the ZED, video devices and the OWLv2 weights |

Start it:

1. On the host: `xhost +local:` (lets the container open RViz windows).
2. `cd ~/Projects/Makerspace/AI_arm && code .` and choose **Reopen in Container**.
   Open the **repo root**, not a sub-folder. Opening `src/arm_language`
   alone creates a second, unrelated devcontainer.
3. Check you are inside: `whoami; pwd` → `ros`, `/workspaces/AI_arm`.
   A `!` command typed into Claude Code runs on the **host**, not here.

Build once (and after pulling new code):

```bash
colcon build --symlink-install --packages-select ur7e_interfaces arm_interfaces arm_language ur7e_orchestrator ur7e_perception ur7e_bringup
source install/setup.bash
```

- `colcon build` compiles our packages; the `*_interfaces` ones generate the
  message and service types the others import.
- `--symlink-install` links the Python files, so edits apply without a rebuild.
- New terminals source ROS and `install/` automatically (`~/.bashrc`). The
  terminal you built in needs the `source` line once.

## 4. The one line every terminal needs

```bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

Why:

- **`ROS_DOMAIN_ID`** decides which ROS processes find each other. The
  devcontainer's `.bashrc` sets **42**, the lab's domain (laptop ↔ Jetson).
  Sim uses **87** so a sim process can never send a trajectory to a real arm
  on the same network. A terminal left on 42 simply cannot see the others:
  `lab_pick.py` then fails with `No fresh joint state -- is the driver running?`
- **`ROS_LOCALHOST_ONLY=1`** keeps discovery on this machine. The two settings
  are deliberately redundant. The harmless warning
  `selected interface "lo" is not multicast-capable` comes from this one.
- **`CYCLONEDDS_URI`** raises Cyclone's limit of about 10 ROS processes per
  machine; the full stack has more. Without it: `Failed to find a free
  participant index`.

Check with `echo $ROS_DOMAIN_ID` → `87`.

Do not `source scripts/lab_env.sh` for sim: it sets domain 42.
Do not activate `.venv` for ROS work: ROS nodes need the container's system Python.

## 5. Run the full flow by hand (tier 1)

Five terminals. Start each with the line from section 4, and wait for the
ready message before starting the next.

**T1: the arm (mock driver + RViz).** Ready when you see
`Configured and activated scaled_joint_trajectory_controller` and RViz shows the arm.

```bash
ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true launch_rviz:=true
```

**T2: the planner.** Ready when you see `You can start planning now!`

```bash
ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl
```

KDL is the IK solver that has worked on the real arm (not pick_ik).

**T3: the "camera".** Ready when you see `Ready: backend=color`

```bash
mkdir -p /tmp/sim && python3 -m ur7e_perception.synthetic_table /tmp/sim/scene.png /tmp/sim/table.json
ros2 run ur7e_perception webcam_perception_node --ros-args -p calibration:=/tmp/sim/table.json -p replay_image:=/tmp/sim/scene.png -p backend:=color -p objects_file:=$PWD/scripts/lab_objects.yaml -p snapshot_dir:=/tmp/sim/perception
```

The first line draws a table with coloured blocks and writes a calibration
that matches it exactly. The node then serves that image as if it were the
ZED. Open `/tmp/sim/scene.png` to see what the robot "sees".

**T4: the fake robot status.** Keep it running.

```bash
python3 scripts/rehearsal_status.py
```

`lab_pick.py` refuses to move unless the robot reports running, 100 % speed
and no protective stop. The real arm publishes that itself; this script fakes it.

**T5: say the sentence.** Store the long command once:

```bash
P="python3 scripts/lab_pick.py --calibration /tmp/sim/table.json --log-dir /tmp/sim/runs --workspace -0.60 -0.10 -0.30 0.35 --max-excursion 4.0 --poses-file /tmp/sim/poses.json"
```

- `--workspace` matches the synthetic table. Without it the lab plate limits
  apply and every pick fails with `outside_workspace`.
- `--max-excursion 4.0` allows the long first move from the mock start pose.

- `--allow-unverified-tooling` (sim only): picks plan the RG2's own TCP
  (`rg2_tcp`) because the RG2 sits 60° off the flange on the Dual Quick Changer
  ([vendor/cad/README.md](vendor/cad/README.md)). Real `--execute` is refused
  until that mounting is checked on the arm (`scripts/lab_tooling.yaml`,
  `verified: true`); this flag allows it in simulation.

First an executed run, which also parks the arm at `ready`:

```bash
$P --say "pick up the red block" --execute --yes --fake-gripper --allow-unverified-tooling --max-speed-percent 100
```

Then plan-only works too (it computes every trajectory and sends none):

```bash
$P --say "pick up the red block"
```

Why the order: PLAN checks the arm's **current** pose and needs the gripper
pointing down. Plan-only never sends the move to `ready`, so on a fresh mock arm
(gripper sideways) it fails with `tool_not_vertical`. On the real arm you
always start parked at `ready`, so this does not show there.

### Reading the output

Each stage prints one line: `STAGE {outcome, seconds, detail}`.

| Line | What to look for |
|---|---|
| `PARSE` | `action: pick, target_query: "red block"`. Layer ① worked |
| `OBSERVE` / `MOVE stage: ready` | the arm goes to the top-down `ready` pose (`waypoints: 0` if already there) |
| `DETECT` | `confidence`; the `capture_id` links it to LOCATE |
| `LOCATE` | `xyz` in metres in `base_link`. The red block should be ≈ (−0.42, 0.22, 0.04) |
| `PLAN` | every segment proven before any motion; tip-above-table and hover heights |
| `APPROACH` → `RETREAT` | the motion stages; `executed: false` means plan-only |
| `GRASP` | `holding: true` is from `FakeRg2`. It always "holds"; never trust it in sim |
| `RESULT` | `outcome` + `reason`. This is the line to report |

Run logs: `/tmp/sim/runs/*.jsonl`.

### The other sentences, and what each should do

| Sentence | Expected `RESULT` | What it proves |
|---|---|---|
| pick up the red block | `succeeded / ok` | the whole chain |
| put the red block on the blue block | `succeeded / ok` | two detections, place height |
| pick up the green block | `failed / not_found` | one re-observe, then a courtesy HOME (the scene has no green block) |
| go up 2 cm | `refused / action_not_allowed` | a jog is not a pick; no motion |
| pick up the red block and throw it | `refused / target_not_noun_phrase` | the validator refuses; no motion |

### Stopping

Ctrl+C in T5…T1 (reverse order). If T1 is restarted, the mock arm is back at
its start pose, so run an executed pick before a plan-only one again.

## 6. The scripted version, and the checks

Same stack, started and scored automatically: six sentences, PASS/FAIL each,
exit code 0 only if all pass. Use it after code changes, once you know the
by-hand flow.

```bash
bash scripts/lab_pick_rehearsal.sh                       # keyword parser
set -a; source .env; set +a; REHEARSAL_BACKEND=claude bash scripts/lab_pick_rehearsal.sh   # Claude parser
```

Logs: `/tmp/lab-pick-rehearsal/<timestamp>/`. On a `TIMEOUT`, read the log
tail printed under it before rerunning.

Faster checks, no ROS graph needed, cheapest first:

```bash
(cd src/ur7e_orchestrator && python3 -m pytest -q test/test_workflow.py)
(cd src/arm_language && python3 -m pytest -q test)
(cd src/ur7e_perception && python3 -m pytest -q test)
python3 -m pytest -q scripts/test_lab_pick.py
(cd src/ur7e_orchestrator && python3 -m ur7e_orchestrator.cli "pick up the red block" --fail grasp:miss)
bash scripts/tier1_smoke_test.sh
```

The `cli` line runs the state machine against mock adapters. Try `--fail`
with `detect:not_found`, `grasp:miss` and `lift:safety` to see the retries
and the safe abort (no HOME after a safety stop).

## 7. When something fails

| Symptom | Cause / what to do |
|---|---|
| `No module named 'ur7e_perception'` | Not in the container, or not built/sourced. `whoami; pwd`, then section 3 |
| `No fresh joint state -- is the driver running?` | T1 down, or this terminal is on domain 42. `echo $ROS_DOMAIN_ID` |
| `Failed to find a free participant index` | No `CYCLONEDDS_URI` in this terminal (section 4) |
| `tool_not_vertical` on plan-only | Fresh mock arm; run one `--execute` pick first |
| `lab_tooling.yaml is not verified` | `--execute` without the tooling check. In sim add `--allow-unverified-tooling`; on the arm, do the check in [vendor/cad/README.md](vendor/cad/README.md) |
| `outside_workspace` on the red block | `--workspace` missing |
| `BLOCKED … execution gate closed` | T4 (`rehearsal_status.py`) not running |
| PLAN `unreachable` after many runs, or "invalid bounds" | Wrist wound up on the mock arm. Restart T1 |
| RViz does not open | `xhost +local:` not run on the host |

## 8. The other docs: which to open when

| Doc | Open it for |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | The decisions D1–D7 and why |
| [SIMULATION.md](SIMULATION.md) | Tiers 2 and 3 in detail (URSim, Gazebo) |
| [COLD_START.md](COLD_START.md) | Lab day: container with camera, network, power-on |
| [RUNBOOK.md](RUNBOOK.md) | Real-arm procedures and recovery (long; search it) |
| [DEMO_CONSOLE.md](DEMO_CONSOLE.md) | `lab_console.py`, the interactive demo console |
| [SINGLE_CAMERA_PERCEPTION.md](SINGLE_CAMERA_PERCEPTION.md) | How perception and table calibration work |
| [vendor/cad/README.md](vendor/cad/README.md) | Manufacturer CAD (UR7e, Dual Quick Changer, tool side, soft gripper) and the measured changer geometry |
| `LAB_<date>_*.md` | Session logs: what happened that day. History, not instructions |
| [TASK2_SOFTWARE.md](TASK2_SOFTWARE.md), [TASK3_SOFTWARE.md](TASK3_SOFTWARE.md) | Perception and language deliverables, including the Gazebo demos |

Known open items (from [LAB_2026-10-07_SIM_E2E.md](LAB_2026-10-07_SIM_E2E.md)):
the demo console cannot be rehearsed in sim yet (gap 5); "quickly" stays in
the pick target (gap 6); Gazebo runs the language parser but not
`lab_pick.py` (gap 7); the Gazebo docs disagree on whether the grasp latch
works (finding 7).
