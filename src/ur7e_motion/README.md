# ur7e_motion

`motion_node` (task 1.5): a ROS 2 action server, `execute_primitive`
(`ur7e_interfaces/action/ExecutePrimitive`), that turns five motion
primitives into `pymoveit2` calls.

## The concept: why pymoveit2, and what it buys us

MoveIt2's official Python bindings (`moveit_py`) need MoveIt 2.7+, which
isn't on Humble's binary release — see
[ARCHITECTURE.md §5](../../docs/ARCHITECTURE.md). `pymoveit2` is a
community layer over the same `move_group` actions/services any MoveIt
client uses (`MoveGroup` action, `compute_fk`, `apply_planning_scene`, …) —
nothing here talks to MoveIt any differently than RViz's own Motion Planning
plugin does.

The one piece worth understanding if you're new to MoveIt: **named poses
aren't hardcoded here.** `RobotDescription.from_node(node, remote_node_name="move_group")`
fetches the *running* `move_group`'s URDF/SRDF parameters and parses them, so
`primitives.goto_named()` resolves `home`/`observe` from whatever
`ur7e_pick_place_bringup/srdf/ur7e_pick_place.srdf.xacro` currently says.
Edit a named pose there; nothing here needs to change.

## The five primitives (`primitives.py`)

| Primitive | What it does |
|---|---|
| `goto_named` | Joint-space move to an SRDF `group_state` by name. |
| `approach_above` | Cartesian move to `z_offset` metres above a given `target_pose` (keeps its orientation — top-down grasp only in v1, ARCHITECTURE.md D3). |
| `cartesian_descend` | Straight down `z_offset` m from wherever the arm currently is. |
| `cartesian_lift` | Straight up `z_offset` m. |
| `retreat` | Same motion as lift, kept as a separate primitive because it means something different in a sequence (leaving a grasp site vs. transporting an object) — see the action definition's own comment. |

Each is a plain function taking a live `MoveIt2` instance — read any one
standalone, no class state to trace through.

## Run

Needs `ur7e_pick_place_bringup`'s MoveIt launch already running (tier 1, 2,
or 3 — motion_node doesn't know or care which).

```bash
ros2 run ur7e_motion motion_node
ros2 action send_goal /execute_primitive ur7e_interfaces/action/ExecutePrimitive \
  "{primitive: goto_named, named_pose: home}"
```
