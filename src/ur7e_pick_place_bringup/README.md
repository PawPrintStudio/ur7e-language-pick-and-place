# ur7e_pick_place_bringup

Phase 1's bringup: everything needed to get the UR7e + RG2 gripper under
MoveIt2, in one command, in three flavors — real robot, tier-1 mock hardware,
tier-2 URSim (see [docs/SIMULATION.md](../../docs/SIMULATION.md)).

## What's ours vs. vendored

Task 1.3/1.4's URDF and MoveIt config aren't written from scratch here —
[ARCHITECTURE.md decision D6](../../docs/ARCHITECTURE.md) already points at
`tonydle/UR_OnRobot_ROS2` (+ its `OnRobot_ROS2_Driver` / `OnRobot_ROS2_Description`
companions) as the starting point, and it turns out to be genuinely complete:
a working combined URDF, `ros2_control` controllers (including a real Modbus
gripper `hardware_interface`), and a MoveIt2 config, all MIT-licensed. Pulled
in at build time via `vcs` (`ur7e.repos`, `scripts/vendor_import.sh`) into
`src/vendor/` — never committed, same treatment the project already gives
`ur_robot_driver` (apt, not vendored).

This package holds only the delta:

| File | What it changes | Why |
|---|---|---|
| `launch/ur7e_pick_place.launch.py` | Fork of the vendored `start_robot.launch.py` | That file hardcodes its controllers-YAML path and never passes `kinematics_parameters_file` to the xacro — no launch argument reaches either, so wrapping with `IncludeLaunchDescription` can't override them. Forking is the same move the vendored file itself made on top of upstream `ur_control.launch.py`. |
| `config/ur7e_controllers.yaml` | Full copy of the vendored `ros2_control` controllers config + `gripper_action_controller` | Task 1.2's `GripperCommand` action, as a stock `position_controllers/GripperActionController` — no custom driver node, since the vendored `onrobot_driver` `hardware_interface` already exposes `finger_width` as a normal ros2_control joint. |
| `config/ur7e_moveit_controllers.yaml` | New — MoveIt's own trajectory-execution controller list | Not reused from the vendored `ur_onrobot_moveit_config/config/controllers.yaml`: that file's flat layout only works via the vendored launch file's manual nesting, while `MoveItConfigsBuilder.trajectory_execution()` needs the nesting already in the file. Also trimmed to the one controller we spawn active (`scaled_joint_trajectory_controller`) — MoveIt shouldn't drive the gripper's `GripperCommand` action through trajectory execution. |
| `srdf/ur7e_pick_place.srdf.xacro` | Includes the vendored SRDF unchanged, adds one `group_state` | `observe`, clear of the future overhead camera (task 2.1) — everything else (`home`, the gripper `open`/`closed` states, the `ur_onrobot_manipulator`/`ur_onrobot_gripper` groups) was already there. |
| `config/ur7e_kinematics.yaml` | Overrides the vendored `kinematics.yaml` | `pick_ik` instead of KDL — task 1.4/D6's decided solver. |
| `urdf/ur7e_pick_place.urdf.xacro` | Thin pass-through to the vendored combined xacro | Lets `ur7e_moveit.launch.py`'s `MoveItConfigsBuilder` load `robot_description` by file path + xacro mappings, the same way it loads everything else. |
| `ur7e_pick_place_bringup/planning_scene.py` | New node | Task 1.3's table collision geometry + task 1.7's pick/place object poses, added as MoveIt `CollisionObject`s at startup — not baked into the URDF, so re-measuring against the real table (lab session) is a one-file edit. |

If you bump the `ur7e.repos` pin and something here stops making sense, diff
against the newly-vendored file — everywhere above marks the intentional
delta with a comment.

## What's a placeholder

- The overhead-camera TF (`camera_placeholder_tf` in `ur7e_moveit.launch.py`)
  is a guess. Task 2.1 (camera hardware) is what replaces it with a measured
  mount offset.
- `planning_scene.py`'s table/object poses are "taped, hardcoded pose"
  per task 1.7's own acceptance criterion — real numbers come from a lab
  session with a tape measure, not this laptop.
- `gripper_action_controller`'s `max_effort` in the controllers YAML is a
  placeholder; the Modbus driver's actual force-control register isn't
  exercised until task 1.1's bench setup.

## Run

```bash
# Tier 1 — mock hardware, no robot needed:
ros2 launch ur7e_pick_place_bringup ur7e_pick_place.launch.py use_fake_hardware:=true
ros2 launch ur7e_pick_place_bringup ur7e_moveit.launch.py

# Tier 2 — URSim (sim/ursim, docs/SIMULATION.md), same commands minus use_fake_hardware:
ros2 launch ur7e_pick_place_bringup ur7e_pick_place.launch.py
ros2 launch ur7e_pick_place_bringup ur7e_moveit.launch.py

# Real robot: same as tier 2, robot_ip defaults to the lab robot.
```

Open RViz's Motion Planning panel (or pass `launch_rviz:=true`) to see the
table + pick/place objects and plan interactively before anything is wired
to `motion_node` ([../ur7e_motion](../ur7e_motion)).
