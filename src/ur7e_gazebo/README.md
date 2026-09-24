# ur7e_gazebo

Task 1.8 — sim tier 3 (Gazebo Fortress / Ignition Gazebo 6): the UR7e + RG2
running under real physics, not mocked/mirrored setpoints. This is the piece
[docs/ARCHITECTURE.md decision D7](../../docs/ARCHITECTURE.md) says doesn't
exist anywhere turnkey ("no turnkey RG2+Gazebo integration exists") — this
package is that integration.

## What's verified working

- The combined arm+gripper URDF spawns and both `ros2_control` hardware
  interfaces (`ign_ros2_control/IgnitionSystem`, one for the arm via
  `ur_description`'s own `sim_ignition` branch, one we wrote for the
  gripper) initialize successfully.
- `joint_trajectory_controller` and `gripper_action_controller` both
  activate.
- **MoveIt2 → motion_node → real Gazebo physics, end to end**: sent
  `goto_named home` through the exact same `ExecutePrimitive` action tier 1
  uses, and the arm moved under actual simulated dynamics — confirmed by
  reading `/joint_states` before and after (start pose → `home`'s joint
  values, matching what tier 1 does, except this time gravity/inertia/motor
  response are real, not mirrored).

This is the core deliverable: everything above `ros2_control` — MoveIt2,
`motion_node`, `scripts/pick_place_demo.py` — runs completely unmodified
against Gazebo. Confirmed, not assumed.

## What's not working yet, and exactly what's known about why

### The grasp latch (DetachableJoint) doesn't attach

The plan (D7 tier-3 item 2) was: weld `pick_object` to the gripper via
Ignition Gazebo 6's `DetachableJoint` system plugin on command, since real
two-finger contact physics in Gazebo is notoriously unstable. This was
investigated hard, in three stages, each with a live-tested, confirmed
finding — not a guess:

1. **Plugin declared on the robot** (`parent_link=onrobot_base_link`,
   `child_model=pick_object`) — failed immediately: `Link with name
   onrobot_base_link not found in model ur7e_gz`. Tried a shallower link
   (`tool0`, pure arm, nothing to do with the gripper addition) — identical
   failure. Confirmed via Ignition Gazebo 6's own shipped reference world
   (`/usr/share/ignition/ignition-gazebo6/worlds/detachable_joint.sdf`,
   which uses a *statically world-loaded* model, not a *dynamically spawned*
   one) that a DetachableJoint plugin declared on a model spawned at launch
   time via `ros_gz_sim create` cannot resolve **any** of its own links by
   name — an ECM timing issue independent of which link or how deep.

2. **Plugin flipped onto `pick_object`** instead (present in the world from
   load time, so its own link resolves) with the robot as the lazily-checked
   child — this got further: the "child model" error changed from a hard
   failure to a per-frame retry warning, and once the robot actually spawned,
   the model-level lookup (`ur7e_gz`) succeeded. But the **link**-level
   lookup (`onrobot_base_link`) inside that model still fails, every frame.

3. **Root-caused to sdformat's fixed-joint lumping**: `onrobot_base_link` is
   attached to its parent (`tool0`) by nothing but a fixed joint, and
   sdformat's default URDF→SDF conversion merges ("lumps") any link reached
   only through fixed joints into its nearest ancestor for physics-engine
   efficiency — so `onrobot_base_link` likely never exists as an independent
   SDF entity at all, which is consistent with `tool0` failing identically
   in stage 1 (it's *also* reached only via fixed joints further up the
   chain). Confirmed sdformat ships the documented escape hatch for this
   (`disableFixedJointLumping` — verified present via `strings` on the
   installed `libsdformat` .so, not assumed) and added it
   (`ur7e_gz.urdf.xacro`'s `<gazebo reference="onrobot_base_link">` block).
   **This did not resolve it** — the link still isn't found after adding the
   tag. Left in place since it's the textbook-correct fix and does no harm;
   whether it's applied too late in the pipeline, needs a different link
   scoped, or there's a second lumping boundary in play is the open
   question for whoever picks this up next.

**Practical effect today:** `pick_object` sits on the table under real
gravity/friction, but nothing welds it to the gripper — a grasp attempt in
Gazebo won't transport the object. The `/pick_object/attach` and
`/pick_object/detach` ROS↔Gazebo topic bridges (`launch/ur7e_gz.launch.py`)
are wired and ready for whenever the link-resolution issue is fixed; no
other part of this package needs to change.

### The gripper doesn't reliably reach a commanded width

Sending a `GripperCommand` goal is accepted and the controller runs, but the
joint reports `stalled: true, reached_goal: false` and doesn't visibly move
to the target width. Not yet root-caused — candidates worth checking first:
`position_proportional_gain` (`ign_ros2_control` logged it defaulting to
`0.1` for the arm's interface; the gripper's own hardware interface may need
an explicit, different gain for its much smaller prismatic range), the
`finger_width_mock_link`'s very small placeholder mass/inertia (added only
to survive SDF conversion — see next section — may be too light for stable
control), or friction/joint-limit interaction with the (currently untested)
mimic finger joints.

### Simplifications, clearly flagged

- **`finger_width_mock_link` needed a nonzero inertial** to survive SDF
  conversion at all (confirmed live: sdformat drops any link with no mass,
  taking its parent joint with it — `gz_ros2_control` then logs "Skipping
  joint ... which is not in the gazebo model", exactly what happened before
  this fix). Can't add it to the vendored `rg2_macro.xacro` without editing
  vendored source, so `ur7e_gz.urdf.xacro` reimplements that one macro
  (`finger_joint`) with the added `<inertial>`, calling every other piece of
  `onrobot_rg2` unmodified — see that file's own comment for the exact diff.
- **Mimic finger joints untested.** Whether the visual fingers (left/right
  knuckle, inner finger) actually track `finger_width` under Ignition
  Gazebo 6 hasn't been checked — doesn't affect grasp behavior either way
  (that's the DetachableJoint's job, not finger contact), but expect it to
  need its own investigation.
- **The world's table/object poses are the same "taped, hardcoded, re-measure
  in a lab session" numbers as `ur7e_pick_place_bringup/planning_scene.py`**
  (kept in sync manually — see `worlds/pick_place_table.sdf`'s header).

## Run

```bash
ros2 launch ur7e_gazebo ur7e_gz.launch.py gazebo_gui:=false   # headless; true on a workstation
ros2 launch ur7e_pick_place_bringup ur7e_moveit.launch.py launch_rviz:=false \
    use_sim_time:=true moveit_controllers_file:=config/ur7e_gz_moveit_controllers.yaml
ros2 run ur7e_motion motion_node --ros-args -p use_sim_time:=true
```

Then anything that already works against tier 1 — `ros2 action send_goal
/execute_primitive ...`, `scripts/pick_place_demo.py` — runs the same way,
now against real physics. The full pick-and-place demo will run to
completion motion-wise; the grasp step just won't visibly hold the object
until the DetachableJoint issue above is resolved.

**Process hygiene note, learned the hard way this session:** `ign gazebo`
processes do not always die cleanly from a `pkill` pattern match against the
`ros2 launch` wrapper — a stale second instance running alongside a fresh
one produces deeply confusing, inconsistent results (a `DetachableJoint`
finding that appeared to "fix itself" on a later run turned out to be two
Gazebo processes answering queries at once). If Gazebo debugging output
looks inconsistent between runs, `ps aux | grep "ign gazebo"` before trusting
it — kill everything and start clean rather than trusting an incremental
`pkill`.
