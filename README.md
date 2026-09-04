# UR7e Language-Directed Pick-and-Place

Natural-language pick-and-place for a **Universal Robots UR7e**: say or type *"pick up the hammer"* and the arm finds the object, plans a grasp, and picks it up — through a standardized, repeatable workflow.

Built on **ROS 2 Humble** running natively on an **NVIDIA Jetson Orin Nano** (JetPack 6.2.x), extending the proven driver + network setup from [ur7e-ros2-keyboard-controller](https://github.com/PawPrintStudio/ur7e-ros2-keyboard-controller).

## How it works

```
 "pick up the hammer"
        │
        ▼
┌───────────────────┐   noun-phrase query   ┌──────────────────────┐
│  Intent Parser    │──────────────────────▶│  Perception          │
│  (LLM → JSON cmd) │      "hammer"         │  NanoOWL + NanoSAM   │
└───────────────────┘                       │  + depth camera      │
                                            └──────────┬───────────┘
                                                       │ 3D grasp pose (base frame,
                                                       │ via hand-eye calibration)
                                                       ▼
┌───────────────────┐   FollowJointTrajectory   ┌──────────────────┐
│  Task Orchestrator│──────────────────────────▶│  Motion (MoveIt2 │
│  (state machine)  │   scaled_joint_traj_ctrl  │  + ur_robot_drv) │
└───────────────────┘                           └────────┬─────────┘
        │                                                │
        ▼                                                ▼
┌───────────────────┐                           ┌──────────────────┐
│  Safety Monitor   │                           │  Gripper Node    │
│  (dashboard/IO)   │                           │  (pluggable HW)  │
└───────────────────┘                           └──────────────────┘
```

Every pick follows the same fixed stage sequence (the "standardized workflow"):

`IDLE → PARSE → OBSERVE → DETECT → LOCATE → PLAN → APPROACH → GRASP → LIFT → RETREAT → HOME`

Each stage has explicit entry checks, logged outcomes, and defined failure/retry behavior, so runs are comparable and debuggable.

## Hardware

| Component | Choice | Status |
|---|---|---|
| Arm | UR7e (e-Series, 7.5 kg payload, 850 mm reach) | on hand |
| Compute | Jetson Orin Nano 8GB, JetPack 6.2.x, MAXN Super mode | on hand |
| Depth camera | Stereolabs ZED 2i — fixed overhead mount (~1 m), NEURAL_LIGHT depth | decided |
| Gripper | OnRobot RG2 v2 (110 mm stroke, 3–40 N) — Modbus via Compute Box or tool RS-485 | decided |

## Working without the robot

Lab access is not required for most development ([architecture D7](docs/ARCHITECTURE.md)):

- **Tier 1 — mock hardware**: MoveIt + RViz against `use_mock_hardware:=true`; runs anywhere, gates CI.
- **Tier 2 — URSim**: the official UR controller simulator in Docker, driven by the *real* ROS driver (x86 Linux/Windows).
- **Tier 3 — Gazebo**: full physics world — UR7e + RG2 + simulated overhead RGB-D camera — running the entire language → detect → pick pipeline, with OWLv2 standing in for the Jetson's NanoOWL.

A VSCode devcontainer gives any member the full stack from `git clone`. Only grasping quality and calibration require the physical setup.

## Documentation

- [Architecture & decisions](docs/ARCHITECTURE.md) — the full stack, resolved open questions, risks
- [Implementation plan](docs/IMPLEMENTATION_PLAN.md) — phased plan; mirrors the GitHub issues/project board

## Project goal

Beyond the demo itself, this repo is a **learning platform** for makerspace members: every package documents the concept it embodies (hand-eye calibration, ros2_control, open-vocabulary detection), the code favors readability over cleverness, and each phase ends in a demo anyone can reproduce from the runbook.

## Status

Planning phase. See the project board for live task status.
