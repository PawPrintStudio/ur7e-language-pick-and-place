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
| Depth camera | Orbbec Gemini 335 (recommended) or RealSense D435 | **decision open** |
| Gripper | Robotiq Hand-E / 2F-85 via tool RS-485 (recommended) or flange-IO gripper | **decision open** |

## Documentation

- [Architecture & decisions](docs/ARCHITECTURE.md) — the full stack, resolved open questions, risks
- [Implementation plan](docs/IMPLEMENTATION_PLAN.md) — phased plan; mirrors the GitHub issues/project board

## Status

Planning phase. See the project board for live task status.
