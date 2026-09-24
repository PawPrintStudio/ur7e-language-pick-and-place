# ur7e_safety_monitor

Task 1.6: the node that notices when the robot has stopped moving on its own
and makes sure nothing else keeps waiting on a goal that's never coming back.

## The concept: why this needs its own node

Every other Phase 1 node (`motion_node`, the gripper action controller,
`scripts/pick_place_demo.py`) assumes the happy path — send a goal, it
completes. A protective stop breaks that assumption *silently* from any one
node's point of view: the action just... never finishes. `safety_monitor`
is the one thing in the stack whose job is watching for that and reacting,
so nothing else has to.

It does two things:

1. **Watches** `/io_and_status_controller/safety_mode`
   (`ur_dashboard_msgs/SafetyMode`) and `robot_program_running`. On a
   transition into a stopped mode (protective stop, e-stop, violation, …) it
   cancels any in-flight goal on `motion_node`, the gripper action
   controller, and the raw trajectory controller, and publishes
   `/ur7e/safety_event` so anything downstream (the demo script, later the
   Phase 4 orchestrator) can react instead of hanging.
2. **Offers `~/recover`** (`std_srvs/Trigger`): calls
   `/dashboard_client/unlock_protective_stop`, then reports — it does not
   perform — the two remaining manual steps. A ROS node can't press Play on
   the pendant or restart the driver process for you, and skipping the
   driver restart is exactly the wedge documented in
   [docs/SIMULATION.md](../../docs/SIMULATION.md)'s tier-2 recovery drill
   (RUNBOOK, 2026-09-17): the trajectory controller holds its stale
   pre-stop command, and reconnecting without a restart loops forever
   vetoing it.

## Verified against

Tier 2 (URSim) only — `safety_mode` doesn't exist under tier-1 mock hardware
(there's no dashboard, no safety system to simulate), so this node has
nothing to watch there. See [docs/SIMULATION.md](../../docs/SIMULATION.md)
for the exact protective-stop drill this was exercised against.

## Run

```bash
ros2 run ur7e_safety_monitor safety_monitor_node
ros2 topic echo /ur7e/safety_event
# after inducing a protective stop (docs/SIMULATION.md drill):
ros2 service call /safety_monitor/recover std_srvs/srv/Trigger
```
