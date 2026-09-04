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

1. **Fixed observe pose.** Every run starts by parking the arm at the same joint-space observe pose — clear of the fixed overhead camera's view — before capturing an image. The camera always sees the unoccluded workspace from the same viewpoint → comparable detections, stable calibration validity.
2. **Stage contracts.** Each stage has typed inputs/outputs, a timeout, and an explicit outcome (`OK`, `RETRY`, `ABORT`) logged with a run ID. A failed run tells you *which stage* failed and why.
3. **Grasp strategy is fixed:** top-down grasp at the segmented object's centroid, gripper yaw aligned to the mask's principal axis, pre-grasp hover at +10 cm, straight-line descend, close, straight-line lift. No 6-DoF grasp inference in v1 — deterministic beats clever for repeatability.
4. **Workspace is bounded.** A configured table-plane polygon in `base_link`; any grasp pose outside it (or below the table plane) is rejected before planning.

## 2. Decisions (with rationale)

### D1. OS/base: JetPack 6.2.x + native ROS 2 Humble

JetPack 6.2 = Ubuntu 22.04 = Tier-1 Humble target; `sudo apt install ros-humble-ur` works on arm64 (buildfarm binaries exist). JetPack 6.2 also unlocks MAXN "Super" mode (67 TOPS, +50% memory bandwidth) on the same hardware — free performance we want for perception. The keyboard-controller repo already validated native Humble on this Jetson. JetPack 5 would force containers for Humble; JetPack 7 pairs with Jazzy — wrong distro for the ecosystem we're using (Isaac/NanoOWL tooling targets Humble).

Perception models run from **jetson-containers** Docker images (NanoOWL/NanoSAM have exact CUDA/TensorRT version needs), bridged to the native ROS graph. Everything else runs native.

### D2. Motion: MoveIt2 + `scaled_joint_trajectory_controller`

The keyboard controller proved raw FollowJointTrajectory works, but pick-and-place needs collision-aware planning (table, camera, gripper). `ur_moveit_config` gives us MoveIt2 out of the box; we extend it with a combined URDF (arm + gripper + camera mount). The scaled JTC is the right execution backend: it respects the pendant speed slider and pauses/resumes cleanly through safeguard stops — exactly the behavior we want during development with people nearby. Cartesian straight-line segments (descend/lift) via MoveIt cartesian path planning. No `moveit_servo` in v1 — nothing in this workflow needs realtime servoing.

**Python access to MoveIt: `pymoveit2`** (AndrejOrsula). The official `moveit_py` bindings don't exist on Humble binaries (MoveIt 2.7+, Iron onward), and building MoveIt from source would drag us off the LTS line. pymoveit2 is the de-facto standard pure-Python layer over stock `ros-humble-moveit`: pose/joint goals, cartesian paths, collision objects, attach/detach — everything `motion_node` needs. **IK solver: `pick_ik`** (PickNik, Humble apt) instead of default KDL — a one-line `kinematics.yaml` change with better behavior near e-Series wrist singularities.

**Mandatory step the teleop repo skipped:** extract the robot's factory kinematics with `ur_calibration` and feed it to the driver (`kinematics_params_file`). Without it, TCP poses are off by centimeters — fatal for vision-guided grasping, invisible for keyboard jogging.

### D3. Perception: NanoOWL + NanoSAM, open-vocabulary, on-demand

NanoOWL (TensorRT OWL-ViT) is the proven open-vocab detector on Orin-class hardware, with an official ROS2 Humble node; NanoSAM turns the detection into a clean instance mask for grasp-point computation. Both fit the 8GB Nano. We run detection **on demand** (one capture per pick, at the observe pose), not as a continuous stream — this sidesteps most of the FPS and memory-contention concerns; even 2–5 FPS is irrelevant when we need one good frame.

NanoOWL takes **noun phrases**, not sentences — which is exactly why the LLM intent parser exists (D5).

Rejected: FoundationPose / Isaac Manipulator (memory-heavy, Orin NX/AGX territory); Grounding DINO (too heavy / licensing); YOLO-World (fixed-vocabulary reparameterization defeats the open-vocabulary point).

### D4. Camera: ZED 2i (project hardware), fixed overhead mount (eye-to-hand)

**The project uses a Stereolabs ZED 2i** (decided 2026-09-04). Jetson support is first-class: ZED SDK 5.2/5.3 targets JetPack 6.2, and `zed-ros2-wrapper` officially supports Humble. The trade-off vs on-camera-depth sensors: the ZED computes depth **on the Jetson GPU**, so its configuration must be tuned to coexist with NanoOWL on the 8GB Nano:

- **Depth mode `NEURAL_LIGHT`** (~36% GPU at 30 FPS on Orin-class hardware, ideal range 0.3–5 m, <1% error to 3 m). `NEURAL` eats ~88% GPU — off the table alongside detection.
- **Sequential, not concurrent**: grab frame → run detection → look up depth. Our on-demand capture design (Q7) already implies this.
- Disable positional tracking **and** `depth_stabilization: 0` (stabilization silently re-enables tracking), no point-cloud publishing (biggest CPU sink — we use the registered depth image), HD720 @ 15 FPS is plenty.
- `depth/depth_registered` is registered to the RGB viewpoint by construction — no manual alignment step.

**Mounting: fixed overhead/tripod (eye-to-hand) at ~0.8–1.2 m over the table.** The ZED 2i is the wrong shape for a wrist: 230 g, 175 mm wide, stiff locking USB3 cable, and — decisively — **0.3 m minimum depth**, which is exactly where a wrist camera sits during pre-grasp. At ~1 m overhead it's in every depth mode's sweet spot (~≤10 mm error). The arm parks at an observe pose *outside the camera's view* before each capture so it never occludes the scene. Requires a rigid mount (bumping it invalidates calibration — makerspace hazard; add a calibration-check step to the runbook).

Hand-eye calibration via ChArUco board on the gripper flange + `easy_handeye2` (eye-to-hand mode), verified with a touch-point test (arm touches a detected marker; error must be < 1 cm).

### D5. Language: LLM intent parser, cloud-first with local fallback interface

Free-form request → strict JSON: `{action: "pick", target_query: "hammer", modifiers: {color: "red", ...}, place_target: null}`. `target_query` (a noun phrase) is what NanoOWL consumes.

**Cloud API (Claude) is the v1 default**: far better language robustness, zero GPU/RAM footprint on the Nano (the crunch resource), and 1–3 s latency is nothing next to a ~20 s pick cycle. The parser node hides the backend behind its service interface, so a local model (Llama 3.2 3B via jetson-containers/ollama, ~28 tok/s on the Nano) can be swapped in later for offline demos. Voice input (whisper.cpp, CUDA build) is a stretch phase — text CLI first, since it exercises the identical pipeline.

### D6. Gripper: OnRobot RG2 v2 (project hardware)

**The project uses an OnRobot RG2 v2** (decided 2026-09-04): 0–110 mm adjustable stroke, 3–40 N adjustable force, 2 kg force-fit payload, 0.78 kg, mounted via the OnRobot Quick Changer. It holds grip force on power loss — a nice safety property. Both control routes are physically available (the makerspace has an unused Compute Box; the gripper currently runs on the direct tool connector). **Primary route: direct tool connector** — keeps the existing wiring, and the best-fit ROS2 stack is purpose-built for it:

- **Direct tool connector (primary)** — Modbus RTU over the flange RS-485 at 1 M baud (supported by the v2 hardware revision). Requires UR's **RS485 Daemon URCap** (ToolComm Forwarder → virtual `/tmp/ttyUR` on the Jetson) alongside External Control — those two coexist by design. Driver: `tonydle/OnRobot_ROS2_Driver` (serial mode, ros2_control — the gripper appears as a `finger_width` joint in RViz/MoveIt); its companion **`tonydle/UR_OnRobot_ROS2`** ships a combined UR+RG2 URDF, controllers, and MoveIt config (`onrobot_type:=rg2`) — our phase-1 URDF/MoveIt starting point.
- **Compute Box (fallback / teaching rig)** — Modbus TCP over Ethernet, fully independent of External Control, zero URCap involvement; the gripper can be driven from any laptop without the robot — useful as a standalone teaching/debug station, and the escape hatch if the serial bridge proves flaky. Humble driver: `ABC-iRobotics/onrobot-ros2` (Python) or `tonydle/OnRobot_ROS2_Driver` (TCP mode). Cost: re-cabling the gripper to an external cable along the arm.

**Either way, the OnRobot URCap must be disabled** — it seizes Tool I/O control and its RS-485 daemon conflicts with the forwarder. Tool I/O is set to "Controlled by User", 24 V. TCP/payload must be configured statically by us (URCap auto-update is off): TCP ≈ [0, 0, 200 mm], CoG ≈ [0, 0, 64 mm], mass 0.78 kg + ~0.2 kg Quick Changer.

The 2 kg force-fit payload bounds the object set (fine for hand tools; a sledgehammer is out).

### D7. Simulation & remote development: three tiers

Development must not require the lab (decided 2026-09-04). Three simulation tiers, each faithful to a different layer of the stack. The enabling design rule: **everything above the controller interface is backend-agnostic** — the trajectory controller name and the perception backend are launch parameters, so identical code runs against sim or hardware.

| Tier | What | Faithful to | Runs on |
|---|---|---|---|
| 1 — Mock hardware | Real driver description with `use_mock_hardware:=true` — exposes the scaled JTC + MoveIt with no robot | Kinematics, planning, orchestrator logic | Anywhere incl. CI on every PR |
| 2 — URSim | Official `universalrobots/ursim_e-series` Docker (pin ≥5.26, `ROBOT_MODEL=UR7`) + External Control URCap + the **real** `ur_robot_driver` | Driver behavior: bringup, dashboard services, protective-stop recovery | x86 Linux/Windows only (amd64 image; not the Jetson, not Apple Silicon) |
| 3 — Gazebo Fortress | `ros-humble-ur-simulation-gz` (apt) with `ur_type:=ur7e` + our world | The full language → detect → locate → pick pipeline with physics and a simulated camera | Linux/Windows dev machines |

Tier 3 needs three pieces of glue from us (no turnkey RG2+Gazebo integration exists):
1. **RG2 in sim**: attach `tonydle/OnRobot_ROS2_Description`'s xacro macro to `tool0`, add sim-tuned inertials and mimic finger joints under `gz_ros2_control`.
2. **Grasp latch**: two-finger contact physics in Gazebo is notoriously unstable (slip/jitter) — use the Fortress **DetachableJoint** pattern (weld object to finger on grasp, detach on release), as `gezp/universal_robot_ign` does. Consequence: **sim validates pipeline logic, never grasp quality** — hardware gates (1.7, 4.4) remain the acceptance authority.
3. **Overhead camera**: Fortress `rgbd_camera` sensor at the calibrated camera pose, bridged via `ros_gz_bridge` (mind: static TF for the ROS optical-frame convention, sensor-data QoS overrides, depth aligned to RGB by construction).

**Perception off-Jetson** (extends D3): `perception_node` gets a pluggable backend — HuggingFace **OWLv2** (`transformers`) on dev machines (CPU is fine at ~1–5 s/frame for our on-demand captures) vs NanoOWL TensorRT on the Jetson — identical service interface either way.

**Portability**: VSCode devcontainer on `osrf/ros:humble-desktop-full` (athackst/vscode_ros2_workspace pattern). Linux and Windows (WSL2 + WSLg) are first-class GUI targets; macOS members work headless: rosbags, `gz sim -s`, Foxglove Studio instead of RViz.

Known sim/real deltas (don't overfit): sim's default controller is the **unscaled** `joint_trajectory_controller` (speed scaling needs real hardware); sim depth is noise-free; DetachableJoint grasps always "succeed".

Rejected: **Isaac Sim** — RTX GPU per seat, no UR7e asset yet, DIY ros2_control bridge; photorealism buys nothing for a scripted pick. Revisit only for synthetic training data at scale.

## 3. Resolved open questions

| # | Question | Resolution |
|---|---|---|
| Q1 | Native ROS or Docker on the Jetson? | Native Humble for ROS graph; jetson-containers only for the model runtimes (D1). |
| Q2 | Which gripper? | **OnRobot RG2 v2** (chosen by the team; on hand). Control route: **direct tool RS-485** primary — matches current wiring and the `tonydle` stack; the on-shelf Compute Box is the fallback (D6). |
| Q3 | Which depth camera, mounted where? | **ZED 2i** (chosen by the team). Fixed overhead mount at ~1 m — its 0.3 m min depth and 175 mm width rule out the wrist (D4). |
| Q4 | LLM local or cloud? | Cloud-first behind a swappable service interface (D5). |
| Q5 | Full 6-DoF grasp planning? | No — fixed top-down grasp strategy for v1 (§1.3). Revisit only if object set demands it. |
| Q6 | MoveIt2 or extend the teleop trajectory approach? | MoveIt2 (D2). The teleop node remains useful as a manual jog/recovery tool. |
| Q7 | Continuous perception or on-demand? | On-demand single capture per pick (D3). |
| Q8 | How do we handle protective stops autonomously? | `safety_monitor` node: detect via `safety_mode`/`robot_program_running`, recover via `dashboard_client/unlock_protective_stop` (+ mandatory ~5 s robot-enforced delay) then `resend_robot_program`/play — but **never auto-resume motion**; the orchestrator aborts the run and returns to IDLE. A human re-issues the command. |
| Q10 | Can members develop without the lab? | Yes — three-tier sim strategy (D7): mock-hardware CI, URSim for driver fidelity, Gazebo Fortress for the full pipeline; devcontainer for any OS. Sim never signs off grasp quality — hardware gates do. |
| Q9 | PolyScope 5 or X on our unit? | **Resolved 2026-09-04: PolyScope 5.23** (read at the pendant). Above the 5.9.4 driver minimum; the PolyScope X URCap changes and Jetson velocity bug (#1859) don't apply. Pin URSim to the matching `5.23` tag. |

## 4. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| 500 Hz RTDE deadline misses on a loaded Jetson ("connection to reverse interface dropped") | Motion aborts mid-pick | Perception is on-demand (GPU/CPU quiet during motion); pin CPU governor, consider isolating a core for the driver; evaluate the passthrough trajectory controller (robot-side interpolation) if drops persist |
| ~~Jetson + PolyScope X velocity-limit bug (driver issue #1859)~~ | — | **Retired 2026-09-04**: unit confirmed PolyScope 5.23 (Q9) |
| 8 GB shared RAM: ROS + TensorRT engines + depth + (local LLM?) | OOM, thrash | Cloud LLM in v1; load perception engines once, lazily; swap on zram; measure with `tegrastats` as a phase-2 acceptance gate |
| ZED depth competes with NanoOWL for the 8GB GPU | OOM / starved inference | NEURAL_LIGHT mode, sequential capture→detect, tracking/point-cloud disabled (D4); build TensorRT engines one at a time with swap enabled (OWL-ViT engine build is known to exhaust 8GB); `tegrastats` gate in phase 2 |
| Overhead camera mount gets bumped (makerspace!) | Silent calibration drift → missed grasps | Rigid mount, calibration-check marker in the workspace, touch-point re-verification step in the runbook |
| Depth/hand-eye error stack-up > gripper tolerance | Missed grasps | Touch-point calibration verification gate (< 1 cm); grasp strategy uses centroid of a *segmented mask*, not bbox center; wide-stroke gripper |
| Skipped kinematics calibration | Centimeter TCP error | `ur_calibration` extraction is a phase-0 blocking task (D2) |
| URSim not available on Jetson | Slower on-robot iteration | x86 laptop URSim loop + mock-hardware CI (D7) |
| Sim-to-real gap breeds false confidence (welded grasps, noise-free depth, unscaled controller) | "Works in sim" ships broken | Deltas documented in D7; hardware demos (1.7, 4.3, 4.4) are the only acceptance authority; controller name parameterized |

## 5. Build-vs-adopt survey (2026-09)

Verdicts on the popular drop-in candidates, so nobody re-litigates them without new information:

| Candidate | Verdict | Reason |
|---|---|---|
| `pymoveit2` | **Adopted** (task 1.5) | Only maintained Python-on-Humble MoveIt layer; `moveit_py` needs MoveIt 2.7+ (not on Humble binaries) |
| `pick_ik` IK plugin | **Adopted** (task 1.4) | Humble apt, one-line config change, better near-singularity behavior than KDL |
| `py_trees` / `py_trees_ros` | **Deferred** (task 4.2's upgrade path) | v1's linear workflow is clearer as an explicit ~100-line FSM (and more teachable); adopt when retry/recovery logic starts nesting |
| MoveIt Task Constructor | Reference only | In Humble apt but effectively C++-only on ROS2 (Python bindings never ported); heavy stage machinery for one fixed grasp sequence |
| Learned grasping (GG-CNN, contact_graspnet, GPD, AnyGrasp) | Skip | Abandonware / no ROS2 wrappers / TF-on-Jetson pain / AnyGrasp is machine-locked closed source; centroid+PCA top-down is what shipping tabletop demos use |
| LLM-ROS frameworks (ROSA, RAI, rosgpt) | Skip | Ops-chat/agent frameworks, not intent parsers; one direct LLM call with a JSON schema is smaller and clearer |
| LangSAM / GroundingDINO+SAM | Skip on this hardware | Several GB + seconds/frame on an 8GB Nano; NanoOWL+NanoSAM is the same capability built for Orin |

## 6. Reuse from ur7e-ros2-keyboard-controller

- The **driver launch + pendant/External-Control startup ritual** (documented and validated) — becomes our runbook baseline.
- `keyboard_teleop.py` — kept as a **manual jog & recovery tool** alongside the autonomous stack; its current-pose-as-first-point trajectory trick is the known-good pattern for direct JTC goals.
- Network topology (laptop → WiFi → Jetson → Ethernet → UR7e).
- Its gaps define our phase 0: no kinematics calibration, no MoveIt, no safety monitoring, committed build artifacts, no CI.
