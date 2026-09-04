# Implementation Plan

Five phases, each ending in something demonstrable on real hardware. Phases map 1:1 to GitHub milestones; tasks map 1:1 to issues on the project board. Decisions referenced as D1–D7 and questions Q1–Q9 are in [ARCHITECTURE.md](ARCHITECTURE.md).

**Guiding principle:** motion before perception, perception before language. Each layer is validated in isolation before the layer above depends on it.

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
| 0.7 | Dev workflow: URSim in Docker on x86 laptop + `use_mock_hardware` config; CI (colcon build + lint) on GitHub Actions | CI green; a MoveIt-less trajectory runs in URSim |

## Phase 1 — Deterministic pick-and-place (no vision, no language)

Goal: the arm picks a known object from a hardcoded pose with a real gripper, under MoveIt2, safely. This proves the entire motion+gripper stack.

| # | Task | Acceptance criteria |
|---|---|---|
| 1.1 | **Decision: select and order gripper** (Q2/D6 — recommend Robotiq Hand-E via tool RS-485) | Decision recorded; hardware ordered |
| 1.2 | `gripper_node` with pluggable backend (`GripperCommand` action; RS-485/Modbus RTU and flange-IO backends) | Open/close/force from CLI works on real gripper |
| 1.3 | Combined URDF/xacro: UR7e + gripper + camera mount + table collision geometry | `robot_state_publisher` + RViz shows correct model; TCP offset configured |
| 1.4 | MoveIt2 config for combined model (from `ur_moveit_config` base); named poses: `home`, `observe` | Plans execute on robot via scaled JTC; collision with table prevented in test |
| 1.5 | `motion_node`: motion primitives (goto named pose, approach-above(pose), cartesian descend/lift, retreat) | Each primitive callable as an action; unit-tested against mock hardware |
| 1.6 | `safety_monitor`: watch `safety_mode`/`robot_program_running`/speed scaling; abort-to-IDLE on protective stop; assisted recovery service (Q8) | Induced protective stop → clean abort, logged; recovery service restores "ready" |
| 1.7 | **Demo: scripted pick** of one object at a taped, hardcoded pose → lift → place at second pose → home | ≥ 9/10 success over 10 consecutive runs |

## Phase 2 — Perception (see and locate)

Goal: given a noun phrase, return an accurate grasp pose in `base_link`.

| # | Task | Acceptance criteria |
|---|---|---|
| 2.1 | **Decision: select and order depth camera** (Q3/D4 — recommend Orbbec Gemini 335) + print/procure wrist mount | Decision recorded; hardware ordered |
| 2.2 | Camera bringup on Jetson: ROS2 driver, aligned RGB-D at observe pose, rosbag capture tooling | Aligned RGB-D topics at ≥ 15 FPS; bags recorded for offline dev |
| 2.3 | NanoOWL + NanoSAM runtime via jetson-containers; `perception_node` `DetectObject` service (D3) | Query "hammer" on live capture → correct mask; latency and `tegrastats` memory recorded |
| 2.4 | `locator_node`: mask + depth → centroid + principal axis → top-down grasp `PoseStamped`; workspace-bounds rejection | On a marker of known position: localization error measured and < 1 cm |
| 2.5 | Hand-eye calibration: ChArUco + easy_handeye2, eye-in-hand; publish static transform; touch-point verification gate (D4) | Arm touches detected marker within 1 cm, 5/5 attempts |
| 2.6 | Perception validation harness: scripted eval over ~10 makerspace objects (tools, blocks) with success/latency report | Detection ≥ 8/10 objects; report committed |

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
| 4.2 | Failure/retry policy: not-found → re-observe once; grasp-miss detection (gripper closed to min width) → single retry; protective stop → abort (Q8) | Each failure path exercised on hardware and logged |
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

- **Long-lead items first:** tasks 1.1 (gripper) and 2.1 (camera) are ordering decisions — file them on day one; shipping time overlaps with phase 0 work.
- Phase 2 software (2.3, 2.4) can start on rosbags/laptop before the camera decision lands on hardware.
- Phase 3 is independent of phases 1–2 and can proceed in parallel; it's pure software.
- The critical path is: 0.2 → 0.4 → 1.3 → 1.4 → 1.5 → 1.7 → 2.5 → 4.1 → 4.3.
