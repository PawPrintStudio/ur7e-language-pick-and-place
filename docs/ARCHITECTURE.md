# Architecture & Decisions

Language-directed pick-and-place on a UR7e, with a Jetson Orin Nano as the single compute node. This document records the stack, the reasoning behind each choice, the questions we resolved during planning, and the risks we know about.

## 1. System overview

One Jetson Orin Nano runs everything: the UR ROS 2 driver, perception, language parsing, and orchestration. The UR7e control box is connected to the Jetson over Ethernet; a laptop reaches the Jetson over WiFi/SSH for development (same topology as the keyboard-controller project).

### Node graph

| Node | Package (planned) | Role | Key interfaces |
|---|---|---|---|
| `ur_robot_driver` | upstream `ur_robot_driver` | Hardware interface to UR7e (RTDE 500 Hz) | `scaled_joint_trajectory_controller`, `io_and_status_controller`, `dashboard_client` |
| `motion_node` | `arm_motion` | MoveIt2 planning + motion primitives (observe pose, approach, descend, lift, home) | Action: `ExecuteMotionPrimitive`; uses MoveGroup |
| `gripper_node` | `arm_gripper` | Pluggable gripper backend (flange-IO or RS-485) | Action: `GripperCommand` |
| `perception_node` | `arm_perception` | NanoOWL detect + NanoSAM segment on demand | Service: `DetectObject(query) → mask, bbox, confidence` |
| `locator_node` | `arm_perception` | Mask + aligned depth → 3D grasp pose in `base_link` | Service: `LocateGrasp(mask) → PoseStamped` |
| `intent_parser` | `arm_language` | Free-form text → structured command JSON via LLM | Service: `ParseIntent(text) → {action, target_query, modifiers}` |
| `orchestrator` | `arm_orchestrator` | The standardized pick workflow state machine | Action: `PickCommand(text)`; consumes all of the above |
| `safety_monitor` | `arm_orchestrator` | Watches `safety_mode`, `robot_program_running`, speed scaling; drives recovery | `dashboard_client` services |

### The standardized workflow

`IDLE → PARSE → OBSERVE → DETECT → LOCATE → PLAN → APPROACH → GRASP → LIFT → RETREAT → HOME`

Design rules that make it repeatable:

1. **Fixed observe pose.** Every run starts by moving to the same joint-space observe pose before capturing an image. Perception always sees the workspace from the same viewpoint → comparable detections, stable calibration validity.
2. **Stage contracts.** Each stage has typed inputs/outputs, a timeout, and an explicit outcome (`OK`, `RETRY`, `ABORT`) logged with a run ID. A failed run tells you *which stage* failed and why.
3. **Grasp strategy is fixed:** top-down grasp at the segmented object's centroid, gripper yaw aligned to the mask's principal axis, pre-grasp hover at +10 cm, straight-line descend, close, straight-line lift. No 6-DoF grasp inference in v1 — deterministic beats clever for repeatability.
4. **Workspace is bounded.** A configured table-plane polygon in `base_link`; any grasp pose outside it (or below the table plane) is rejected before planning.

## 2. Decisions (with rationale)

### D1. OS/base: JetPack 6.2.x + native ROS 2 Humble

JetPack 6.2 = Ubuntu 22.04 = Tier-1 Humble target; `sudo apt install ros-humble-ur` works on arm64 (buildfarm binaries exist). JetPack 6.2 also unlocks MAXN "Super" mode (67 TOPS, +50% memory bandwidth) on the same hardware — free performance we want for perception. The keyboard-controller repo already validated native Humble on this Jetson. JetPack 5 would force containers for Humble; JetPack 7 pairs with Jazzy — wrong distro for the ecosystem we're using (Isaac/NanoOWL tooling targets Humble).

Perception models run from **jetson-containers** Docker images (NanoOWL/NanoSAM have exact CUDA/TensorRT version needs), bridged to the native ROS graph. Everything else runs native.

### D2. Motion: MoveIt2 + `scaled_joint_trajectory_controller`

The keyboard controller proved raw FollowJointTrajectory works, but pick-and-place needs collision-aware planning (table, camera, gripper). `ur_moveit_config` gives us MoveIt2 out of the box; we extend it with a combined URDF (arm + gripper + camera mount). The scaled JTC is the right execution backend: it respects the pendant speed slider and pauses/resumes cleanly through safeguard stops — exactly the behavior we want during development with people nearby. Cartesian straight-line segments (descend/lift) via MoveIt cartesian path planning. No `moveit_servo` in v1 — nothing in this workflow needs realtime servoing.

**Mandatory step the teleop repo skipped:** extract the robot's factory kinematics with `ur_calibration` and feed it to the driver (`kinematics_params_file`). Without it, TCP poses are off by centimeters — fatal for vision-guided grasping, invisible for keyboard jogging.

### D3. Perception: NanoOWL + NanoSAM, open-vocabulary, on-demand

NanoOWL (TensorRT OWL-ViT) is the proven open-vocab detector on Orin-class hardware, with an official ROS2 Humble node; NanoSAM turns the detection into a clean instance mask for grasp-point computation. Both fit the 8GB Nano. We run detection **on demand** (one capture per pick, at the observe pose), not as a continuous stream — this sidesteps most of the FPS and memory-contention concerns; even 2–5 FPS is irrelevant when we need one good frame.

NanoOWL takes **noun phrases**, not sentences — which is exactly why the LLM intent parser exists (D5).

Rejected: FoundationPose / Isaac Manipulator (memory-heavy, Orin NX/AGX territory); Grounding DINO (too heavy / licensing); YOLO-World (fixed-vocabulary reparameterization defeats the open-vocabulary point).

### D4. Camera: on-camera-depth sensor, wrist-mounted (eye-in-hand)

**Recommendation: Orbbec Gemini 335.** Depth computed on-camera (zero Jetson GPU cost — matters on 8GB shared memory), mature ROS2 Humble driver, used in NVIDIA's own Isaac workflows. RealSense D435 is acceptable if one is already on hand, but needs a source build with `-DFORCE_RSUSB_BACKEND=ON` and has documented enumeration problems on JetPack 6 — budget fiddling time.

**Mounting: wrist (eye-in-hand), capturing from the fixed observe pose.** At ~0.4 m standoff depth error is ~2× better than an overhead mount at ~1 m, the hand-eye transform is flange-relative (mechanically stable, calibrate once), and no external camera rig has to stay rigid. Cost: cable management along the arm and a printed mount. Fallback: fixed overhead mount (eye-to-hand) if cabling proves painful — same software, different calibration parent frame.

Hand-eye calibration via ChArUco board + `easy_handeye2` (or MoveIt's hand-eye calibration), verified with a touch-point test (arm touches a detected marker; error must be < 1 cm).

### D5. Language: LLM intent parser, cloud-first with local fallback interface

Free-form request → strict JSON: `{action: "pick", target_query: "hammer", modifiers: {color: "red", ...}, place_target: null}`. `target_query` (a noun phrase) is what NanoOWL consumes.

**Cloud API (Claude) is the v1 default**: far better language robustness, zero GPU/RAM footprint on the Nano (the crunch resource), and 1–3 s latency is nothing next to a ~20 s pick cycle. The parser node hides the backend behind its service interface, so a local model (Llama 3.2 3B via jetson-containers/ollama, ~28 tok/s on the Nano) can be swapped in later for offline demos. Voice input (whisper.cpp, CUDA build) is a stretch phase — text CLI first, since it exercises the identical pipeline.

### D6. Gripper: pluggable backend; recommend Robotiq via tool RS-485

Two viable attachment paths on the e-Series tool flange:

- **Tool RS-485 bridge** (recommended): driver's `use_tool_communication:=true` exposes the flange RS-485 as a virtual tty on the Jetson; run a Robotiq (Hand-E or 2F-85) Modbus RTU driver against it. Adjustable stroke/force → wider range of objects.
- **Flange digital-IO gripper**: cheapest/simplest (spring or pneumatic, toggled via `io_and_status_controller/set_io`) — binary open/close only.

The `gripper_node` exposes one `GripperCommand` action either way, so the hardware decision (open question Q2) doesn't block any software work. Note: vendor URCap gripper control is **not** usable mid-External-Control — the ROS-side serial/IO route is the correct one.

### D7. Simulation & dev workflow

URSim's Docker image is **x86-only** — it cannot run on the Jetson. Development loop: URSim + full ROS stack on an x86 laptop (driver treats URSim like a real robot, including dashboard + External Control), `use_mock_hardware:=true` for pure-kinematics CI. Perception development against recorded rosbags of real captures.

## 3. Resolved open questions

| # | Question | Resolution |
|---|---|---|
| Q1 | Native ROS or Docker on the Jetson? | Native Humble for ROS graph; jetson-containers only for the model runtimes (D1). |
| Q2 | Which gripper? | **Open hardware decision** — tracked as an issue; software unblocked by the pluggable `gripper_node` (D6). Recommend Robotiq Hand-E. |
| Q3 | Which depth camera, mounted where? | Orbbec Gemini 335 recommended, wrist-mounted, captures from fixed observe pose (D4). Final purchase tracked as an issue. |
| Q4 | LLM local or cloud? | Cloud-first behind a swappable service interface (D5). |
| Q5 | Full 6-DoF grasp planning? | No — fixed top-down grasp strategy for v1 (§1.3). Revisit only if object set demands it. |
| Q6 | MoveIt2 or extend the teleop trajectory approach? | MoveIt2 (D2). The teleop node remains useful as a manual jog/recovery tool. |
| Q7 | Continuous perception or on-demand? | On-demand single capture per pick (D3). |
| Q8 | How do we handle protective stops autonomously? | `safety_monitor` node: detect via `safety_mode`/`robot_program_running`, recover via `dashboard_client/unlock_protective_stop` (+ mandatory ~5 s robot-enforced delay) then `resend_robot_program`/play — but **never auto-resume motion**; the orchestrator aborts the run and returns to IDLE. A human re-issues the command. |
| Q9 | PolyScope 5 or X on our unit? | **Must verify on the pendant** (tracked as a phase-0 task). UR7e ships as either. Driver needs ≥5.9.4 (PolyScope 5) or ≥10.7.0 (PolyScope X); PolyScope X changes URCap handling and was implicated in a Jetson-specific velocity-limit issue (driver issue #1859). |

## 4. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| 500 Hz RTDE deadline misses on a loaded Jetson ("connection to reverse interface dropped") | Motion aborts mid-pick | Perception is on-demand (GPU/CPU quiet during motion); pin CPU governor, consider isolating a core for the driver; evaluate the passthrough trajectory controller (robot-side interpolation) if drops persist |
| Jetson + PolyScope X velocity-limit bug (driver issue #1859) | Commands ignored | Verify PolyScope version first (Q9); prefer PolyScope 5 path; track upstream issue |
| 8 GB shared RAM: ROS + TensorRT engines + depth + (local LLM?) | OOM, thrash | Cloud LLM in v1; load perception engines once, lazily; swap on zram; measure with `tegrastats` as a phase-2 acceptance gate |
| RealSense on JetPack 6 enumeration issues | Camera dead on arrival | Prefer Orbbec (D4); if RealSense, RSUSB source build + known-good SDK version |
| Depth/hand-eye error stack-up > gripper tolerance | Missed grasps | Touch-point calibration verification gate (< 1 cm); grasp strategy uses centroid of a *segmented mask*, not bbox center; wide-stroke gripper |
| Skipped kinematics calibration | Centimeter TCP error | `ur_calibration` extraction is a phase-0 blocking task (D2) |
| URSim not available on Jetson | Slower on-robot iteration | x86 laptop URSim loop + mock-hardware CI (D7) |

## 5. Reuse from ur7e-ros2-keyboard-controller

- The **driver launch + pendant/External-Control startup ritual** (documented and validated) — becomes our runbook baseline.
- `keyboard_teleop.py` — kept as a **manual jog & recovery tool** alongside the autonomous stack; its current-pose-as-first-point trajectory trick is the known-good pattern for direct JTC goals.
- Network topology (laptop → WiFi → Jetson → Ethernet → UR7e).
- Its gaps define our phase 0: no kinematics calibration, no MoveIt, no safety monitoring, committed build artifacts, no CI.
