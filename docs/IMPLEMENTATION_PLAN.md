# Implementation Plan

Five phases, each ending in something demonstrable on real hardware. Phases map 1:1 to GitHub milestones; tasks map 1:1 to issues on the project board. Decisions referenced as D1–D7 and questions Q1–Q9 are in [ARCHITECTURE.md](ARCHITECTURE.md).

**Guiding principles:**

1. Motion before perception, perception before language. Each layer is validated in isolation before the layer above depends on it.
2. **This is a learning platform.** Makerspace members should be able to study any module in isolation. Concretely: every package ships with a README explaining *the concept* (not just usage) — "what is hand-eye calibration and why", "how a ros2_control chain works"; code favors readability over cleverness (Python-first, type hints, no premature abstraction); each phase's demo doubles as a teaching checkpoint someone can re-run from the runbook.

---

## Phase 0 — Foundations (platform + robot link)

Goal: the Jetson reliably commands the UR7e with correct kinematics, and the repo has a working dev loop.

| # | Task | Acceptance criteria |
|---|---|---|
| 0.1 | Verify PolyScope version on the pendant; record SW version, update if < 5.9.4 (Q9) | Version documented in repo; driver compatibility confirmed |
| 0.2 | Flash JetPack 6.2.x, enable MAXN Super mode, install native ROS 2 Humble + `ros-humble-ur` | `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur7e` connects to the robot |
| 0.3 | Network setup: static IPs on Jetson↔robot Ethernet link, document laptop→Jetson SSH path | Documented in `docs/RUNBOOK.md`; survives reboot |
| 0.4 | Extract factory kinematics with `ur_calibration`; wire `kinematics_params_file` into our bringup launch | Calibration YAML committed; TCP pose spot-check vs pendant < 1 mm |
| 0.5 | Bringup package: single launch file (driver + calibration + our params), headless-mode option evaluated vs External Control URCap | One command brings the robot to "ready"; startup runbook written |
| 0.6 | Validate motion baseline: run keyboard teleop from the reference repo against our bringup | Jog works; protective-stop recovery procedure exercised and documented |
| 0.7 | Sim tiers 1+2 (D7): `use_mock_hardware:=true` config; URSim Docker (`ursim_e-series:5.26`, `ROBOT_MODEL=UR7`) + External Control URCap against the real driver; CI (colcon build + lint + tier-1 smoke test) on GitHub Actions | CI green; a trajectory runs in URSim through the real driver |
| 0.8 | Devcontainer: `osrf/ros:humble-desktop-full`-based VSCode devcontainer (athackst pattern) so any member on Linux/Windows(WSLg) gets the full stack; document the macOS headless path (rosbags + Foxglove) | A fresh machine reaches "RViz shows the arm" from `git clone` in < 30 min |

## Phase 1 — Deterministic pick-and-place (no vision, no language)

Goal: the arm picks a known object from a hardcoded pose with a real gripper, under MoveIt2, safely. This proves the entire motion+gripper stack.

> **2026-09-24 status (remote session, no lab access):** 1.3–1.5 and the
> software half of 1.2 are built and **verified against sim tier 1** (mock
> hardware) — see `src/ur7e_pick_place_bringup`, `src/ur7e_motion`,
> `src/ur7e_interfaces`. 1.7's full scripted sequence runs end-to-end against
> tier 1 (`scripts/pick_place_demo.py`, exit 0, every stage logged). **Tier 3
> (Gazebo, `src/ur7e_gazebo`) now exists and its motion half is verified**:
> MoveIt2 → motion_node → real Gazebo physics confirmed driving the arm
> end-to-end. Its grasp latch and gripper actuation are not yet working —
> investigated in detail, root cause understood, not resolved this session
> (see `src/ur7e_gazebo/README.md`). 1.6 (`src/ur7e_safety_monitor`) is
> written but only verifiable against tier 2 (URSim) or the real robot — not
> yet run. **Still lab-only:** 1.1 (physical RS-485/URCap bench wiring), the
> hardware leg of 1.2 (real gripper), 1.6's verification, and 1.7's
> ≥9/10-real-runs acceptance. Details and how-to-run in each new package's
> README. `docs/SIMULATION.md` and `docs/RUNBOOK.md` have the session's full
> record.

| # | Task | Acceptance criteria |
|---|---|---|
| 1.1 | RG2 v2 ROS bench setup on the **direct tool connector** (decided — D6): install UR RS485 Daemon URCap, disable the OnRobot URCap, Tool I/O "Controlled by User" @ 24 V, verify `/tmp/ttyUR` bridge from the Jetson | Gripper opens/closes from a test script over the bridge — **lab-only, not started** |
| 1.2 | `gripper_node`: `GripperCommand` action wrapping an OnRobot RG2 driver (`tonydle/OnRobot_ROS2_Driver` serial or `ABC-iRobotics/onrobot-ros2` TCP per 1.1) | Open/close/width/force from CLI works on real gripper — **software done & sim-verified** (`gripper_action_controller` in `ur7e_pick_place_bringup`, a stock `position_controllers/GripperActionController` over the vendored Modbus `hardware_interface`); **real-gripper verification is lab-only** |
| 1.3 | Combined URDF/xacro: UR7e + RG2 (start from `tonydle/UR_OnRobot_ROS2`) + table collision geometry + static overhead-camera frame; static TCP/payload set (≈[0,0,200 mm], 0.78 kg + 0.2 kg QC) | `robot_state_publisher` + RViz shows correct model; TCP verified against pendant — **done & sim-verified** (URDF parses, full kinematic tree confirmed via `check_urdf`; table + pick/place objects added as a MoveIt planning scene, not baked into the URDF — see `ur7e_pick_place_bringup/README.md`); **TCP-vs-pendant check is lab-only** |
| 1.4 | MoveIt2 config for combined model (base: `UR_OnRobot_ROS2` / `ur_moveit_config`); **`pick_ik` as IK solver** (kinematics.yaml); named poses: `home`, `observe` (clear of camera view) | Plans execute on robot via scaled JTC; collision with table prevented in test — **done & sim-verified**: `pick_ik/PickIkPlugin` confirmed live via `ros2 param get`, `home`/`observe` both reachable, and the table collision is real (a demo run that got too close to the table failed to plan until the sequence was routed clear of it — see RUNBOOK) |
| 1.5 | `motion_node`: motion primitives (goto named pose, approach-above(pose), cartesian descend/lift, retreat) **built on `pymoveit2`** (official `moveit_py` is not available on Humble binaries) | Each primitive callable as an action; unit-tested against mock hardware — **done & sim-verified**: all five primitives exercised individually and as part of the full 1.7 sequence over `ExecutePrimitive` (`ur7e_interfaces`); named poses resolved live from the running SRDF, not hardcoded |
| 1.6 | `safety_monitor`: watch `safety_mode`/`robot_program_running`/speed scaling; abort-to-IDLE on protective stop; assisted recovery service (Q8) | Induced protective stop → clean abort, logged; recovery service restores "ready" — **written, not yet run**: needs tier 2 (URSim) or the real robot, since mock hardware has no dashboard/safety system to watch |
| 1.7 | **Demo: scripted pick** of one object at a taped, hardcoded pose → lift → place at second pose → home | ≥ 9/10 success over 10 consecutive runs — **sim leg done**: `scripts/pick_place_demo.py` runs the full sequence end-to-end against tier 1 (exit 0); **the ≥9/10-real-runs acceptance is lab-only** |
| 1.8 | Gazebo sim world (D7 tier 3, motion half): `ur_simulation_gz` (`ur_type:=ur7e`) + RG2 xacro attached with sim inertials/mimic joints + DetachableJoint grasp latch + table/objects world | The 1.7 scripted pick sequence runs end-to-end in Gazebo on a machine with no lab access — **motion half done & verified** (`src/ur7e_gazebo`: MoveIt2 → motion_node → real Gazebo physics confirmed end-to-end, arm moves under actual dynamics); **grasp latch (DetachableJoint) and reliable gripper actuation in sim are open** — root-caused in detail, not yet resolved, see `src/ur7e_gazebo/README.md` |

## Phase 2 — Perception (see and locate)

Goal: given a noun phrase, return an accurate grasp pose in `base_link`.

| # | Task | Acceptance criteria |
|---|---|---|
| 2.1 | ZED 2i platform setup: ZED SDK 5.2+/JetPack 6.2 install, pre-optimize neural-depth TensorRT models (slow first run), build rigid overhead mount ~1 m over the table | `ZED_Diagnostic` clean; camera streams; mount doesn't flex |
| 2.2 | `zed-ros2-wrapper` bringup tuned for the 8GB Nano: HD720@15, `NEURAL_LIGHT`, positional tracking off + `depth_stabilization: 0`, point cloud off; rosbag capture tooling | Registered RGB-D topics ≥ 15 FPS; GPU ≤ ~40% during capture (`tegrastats`); bags recorded for offline dev |
| 2.3 | NanoOWL + NanoSAM runtime via jetson-containers; `perception_node` `DetectObject` service (D3) | Query "hammer" on live capture → correct mask; latency and `tegrastats` memory recorded |
| 2.4 | `locator_node`: mask + depth → centroid + principal axis → top-down grasp `PoseStamped`; workspace-bounds rejection | On a marker of known position: localization error measured and < 1 cm |
| 2.5 | Hand-eye calibration: ChArUco board on the gripper + easy_handeye2 **eye-to-hand**; publish static camera→base transform; touch-point verification gate + workspace calibration-check marker (D4) | Arm touches detected marker within 1 cm, 5/5 attempts |
| 2.6 | Perception validation harness: scripted eval over ~10 makerspace objects (tools, blocks) with success/latency report | Detection ≥ 8/10 objects; report committed |
| 2.7 | Sim camera + dev perception backend (D7 tier 3, vision half): Fortress `rgbd_camera` at the overhead pose bridged via `ros_gz_bridge` (optical-frame TF, sensor QoS); pluggable `perception_node` backend — HuggingFace OWLv2 on dev machines, NanoOWL on Jetson, same service | Full PARSE→…→GRASP pipeline dry-runs in Gazebo without lab or Jetson |

## Phase 3 — Language front-end

Goal: free-form text becomes a validated structured command.

| # | Task | Acceptance criteria |
|---|---|---|
| 3.1 | `intent_parser` service: LLM → strict JSON schema (`action`, `target_query`, `modifiers`, `place_target`); cloud backend first, backend interface swappable (D5) | 20-utterance test set parses correctly incl. rejections of non-pick requests |
| 3.2 | Command console: CLI node to submit text commands + watch workflow stage progress | Usable end-to-end entry point for demos |
| 3.3 | Guardrails: whitelist of actions, confidence threshold, "did you mean" echo before motion (configurable) | Unknown/unsafe requests refused with clear message; no motion on parse failure |

## Phase 4 — Orchestration: the standardized workflow

Goal: "pick up the hammer" works end-to-end, repeatably, with defined failure behavior.

| # | Task | Acceptance criteria |
|---|---|---|
| 4.1 | `orchestrator` state machine implementing the stage pipeline with per-stage timeouts, run IDs, structured logging (§1.2 of architecture) | Dry-run mode traverses all stages against mocks in CI |
| 4.2 | Failure/retry policy: not-found → re-observe once; grasp-miss detection (gripper closed to min width) → single retry; protective stop → abort (Q8). If retry logic outgrows the flat FSM, migrate the orchestrator to `py_trees_ros` (Humble apt — the sanctioned upgrade path, architecture §5) | Each failure path exercised on hardware and logged |
| 4.3 | **End-to-end demo**: 3 distinct objects by name, from voice-of-user text | Video + logs committed |
| 4.4 | Repeatability benchmark: 20-run protocol per object, success rate + per-stage timing dashboard/report | ≥ 80% end-to-end success; report in repo |
| 4.5 | `docs/RUNBOOK.md` final: cold-start to demo in one page (power-on order, pendant steps, launch commands, recovery) | A teammate reproduces the demo from the runbook alone |

## Phase 5 — Stretch / hardening

Not scheduled; pull in as capacity allows.

- Voice input via whisper.cpp (CUDA) feeding the same `intent_parser`
- "Place it in/on X" — place-target detection reusing the perception path
- Local LLM backend (Llama 3.2 3B via ollama) for offline operation
- Multi-object disambiguation ("the *red* screwdriver") using modifier-aware re-ranking
- Continuous scene memory (object persistence between picks)
- PREEMPT_RT kernel or core isolation if reverse-interface drops are observed

---

## Sequencing notes

- **All hardware is on hand** (RG2 v2 + ZED 2i + an unused Compute Box on the shelf as gripper fallback): tasks 1.1 and 2.1 are bench-setup tasks that can start during phase 0 — 2.1 doesn't depend on the arm being ROS-controlled at all.
- Phase 2 software (2.3, 2.4) can start on rosbags/laptop before the camera decision lands on hardware.
- Phase 3 is independent of phases 1–2 and can proceed in parallel; it's pure software.
- **Remote contributors** (no lab access) are unblocked by 0.8 + 1.8 + 2.7: after those land, every pipeline component except real grasping and calibration can be developed and integration-tested in simulation (D7). Sim results never sign off grasp quality — the hardware demos do.
- The critical path is: 0.2 → 0.4 → 1.3 → 1.4 → 1.5 → 1.7 → 2.5 → 4.1 → 4.3.
