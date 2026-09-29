# Direct-Ethernet hardware acceptance, 2026-09-28

## End-of-day summary

The operator ended the lab session. Both session containers
(`ur7e-lab-20260928` and `ur7e-showcase-20260928`) were stopped and verified
exited. The unrelated development container and Portainer were left alone.
The final robot-network check timed out; no power-off or physical emergency-stop
state is claimed. The last successful hardware read showed an idle, healthy
RG2 at 58.4 mm, with the arm program stopped during gripper acceptance.

Completed evidence:

- Actual UR7e reached over laptop Ethernet, with matching factory calibration.
- Real wrist motions and 5 cm vertical lift/return, at operator-approved speed;
  MoveIt execution and rejection of a simulated obstructed state verified.
- Actual RG2 identified, actuated from pendant, then from laptop native API,
  then through ROS `GripperCommand`; width feedback, bounded force commands,
  rejection and immediate cancellation checked.
- Humble safety cancellation and motion-result handling repaired; software
  tests passed. A physical protective-stop recovery drill was not performed.
- One complete RViz/mock pick/place showcase with kinematic object attachment;
  object ended approximately 1.3 mm from the simulation placement target.
  A later replay failed on the final return-to-observe plan. No repeatability
  claim is made, and this does not establish Gazebo grasp physics.
- Hardware-independent parser, motion, and perception checks were reviewed
  separately from physical acceptance; their detailed results are below.

Issue reconciliation for this session: **#9, #12, #21, #31 closed** against
their respective acceptance evidence. These remaining items stay open:

| Issue | Remaining acceptance |
|---|---|
| #8 | Raw RS485 route still returns invalid width; native OnRobot route works. |
| #10 | Validate actual RG2 fingertip TCP, mounting orientation, and payload against pendant; current default was QC Tool Side. |
| #11 | Validate combined model/table and real `home`/`observe` poses; arm-only diagnostics are partial evidence. |
| #13 | Perform the supervised physical stop/abort/recovery drill. |
| #14 | Integrate native gripper backend with combined bringup, measure object/table poses, then complete >=9/10 physical pick/place runs. |
| #30 | Resolve Gazebo gripper actuation and object attachment; RViz kinematic attachment is a separate simulation tier. |
| #15–20 | Jetson/camera bringup, real calibration and perception acceptance remain incomplete. |
| #22–28 | Console/guardrails/orchestration/end-to-end acceptance remains separate from parser and component tests. |

Next-session priorities are TCP/payload validation, native-backend integration,
the physical stop/recovery check, and a first supervised object pick. Technical
uncertainties to resolve are native-vs-raw serial ownership, the raw unit-67
width discrepancy, and the simulation return-to-observe planning failure.
No further pendant actions were requested at closeout.

This follow-up uses the physical UR7e and attached empty RG2, with the laptop
replacing the unavailable Jetson. This is real robot testing. Jetson-specific
camera and platform acceptance remains separate.

The operator subsequently deferred gripper installation/testing to a future
session and requested all feasible arm-only checks instead.

## Connection and initial motion

- Starting revision: `fe68a06` on `main`.
- Laptop: NetworkManager profile `ur-link`, `enp7s0`, `192.168.56.1/24`.
- Robot: `192.168.56.101`, reached directly through `enp7s0`, not a Docker bridge.
- Driver: `ur7e-dev:latest`, container `ur7e-lab-20260928`, host networking,
  ROS domain 42, CycloneDDS, existing workspace build.
- Launch: `ros2 launch ur7e_bringup ur7e_bringup.launch.py`.
- Dashboard: PolyScope 5.23.0.1218630, robot RUNNING, safety NORMAL,
  `/programs/ros2_external_control.urp`, local pendant control.
- Calibration checksum `calib_12445833238222042106` matched successfully.
- Operator confirmed clear workspace, pendant attendance, attached empty RG2;
  pressed Play after driver startup. Dashboard then reported program running.
- `demo_01_nudge.py` submitted wrist_3 +0.05 rad and return, with the explicit
  t=0 trajectory anchor. Client logged `SUCCESSFUL (error_code=0)`.
- **Acceptance caveat:** operator initially reported 10%, but telemetry during
  this run read 50%. A cancel-all request was issued; the client reported
  success near the same time and the cancel service acknowledged one goal.
  Therefore this is not recorded as a clean 10% return-to-start acceptance.
  Initial wrist_3 was approximately -3.582 rad; a subsequent stationary
  sample was -3.57319 rad. No further motion was queued.
- Operator subsequently set 10%, verified as `10.0` on the speed-scaling topic.

Follow-up diagnosis found that `MotionClient.run()` previously checked only
`error_code`, not the action's terminal status: a cancelled action with its
default zero error code could print SUCCESSFUL. It now requires both
`STATUS_SUCCEEDED` and error code zero; the nudge script exits nonzero on
failure. Four regression cases pass, including CANCELLED/ABORTED with zero
error code. Treat the first run above as interrupted/ambiguous, not accepted.

A clean nudge with the corrected check passed at 10%, from ROS timestamp
1790634565.513 to 1790634765.524 (about 200 seconds for the nominal 20-second
trajectory). It completed with successful terminal status and zero error code.

The shared ROS CLI daemon returned `!rclpy.ok()` errors during inspection.
Explicit message types and `--no-daemon` yielded live samples. This was a CLI
inspection problem; the trajectory client and driver remained connected.

## Gripper gate

The operator checked Settings -> System -> URCaps and reported only
UR Connect, OnRobot, and External Control. **RS485 Daemon is missing**.
Tool telemetry reports 24 V,
about 0.021 A, and about 22.5 C. TCP port 54321 refuses connections; installation
alone has not established a working forwarder. Pendant daemon status and
inbound-port restrictions can be checked after installation. The official
`rs485-1.0.urcap` installer is bundled in the ROS driver's resources directory.

Use the [official forwarding setup](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap#usage).
Check daemon startup and specifically port 54321; do not remove unrelated
network restrictions. Do not start combined arm/gripper bringup until the
bridge is available: its serial retry previously blocked controller startup.

## TCP investigation

With one arm-only robot_state_publisher, `base -> tool0` was approximately
(-0.062, -0.118, 0.441) m. Driver TCP pose was
(-0.0910306, -0.1130907, 0.4495683) m. Orientations matched up to quaternion
sign. The position difference is about 30 mm along tool Z, consistent with a
configured offset. The earlier roughly 20 cm discrepancy was not reproduced.
This is preliminary: confirm the pendant's active TCP and then measure with
full precision before accepting combined-gripper TCP accuracy.

## Safety monitor repair

The source called `ActionClient.cancel_all_goals`, absent on ROS 2 Humble.
It now uses each action's CancelGoal service with zero UUID/timestamp,
reports asynchronous cancellation responses, subscribes to latched program
status, and instructs restart-driver then Play after unlocking. Recovery
awaits the dashboard response without recursively spinning the node.

Validation: four package tests passed (ROS action cancellation, ROS recovery
service, flake8, pep257). Protocol tests explicitly use domain 143, separate
from hardware domain 42. A separate action client submitted a goal; the
monitor cancelled it after an injected protective-stop callback. This is a
software protocol test, **not an induced physical protective stop**. Task 1.6
remains open until the real stop, abort, and manual recovery drill passes.

## Issue reconciliation

- Closed #12 / 1.5: its stated acceptance is standalone primitives and
  mock-hardware CI. Existing mock execution evidence covers all five
  primitives, current package tests pass (7/7), and CI on `fe68a06` is green:
  https://github.com/PawPrintStudio/ur7e-language-pick-and-place/actions/runs/36475436786.
  This does not claim a physical grasp or all production gripper-TCP motions.
- Closed #21 / 3.1: committed live-Claude report has 29/29 correct cases,
  zero wrong answers and zero contract violations; acceptance requires 20.
  Evidence: `language-evidence/claude-corpus-acceptance.json`.
- Closed #31 / 2.7: committed fixture and OWLv2 Gazebo logs both report
  `PARSE->OBSERVE->DETECT->LOCATE->PLAN->GRASP->LIFT` and
  `motion_actions_succeeded: true`. This issue explicitly requires simulation.
  Evidence: `perception-evidence/gazebo{,_owlv2}/pick.log`.
- Real gripper, combined TCP, MoveIt hardware execution/collision checks,
  physical stop recovery, and 9/10 pick-and-place remain unaccepted.
- GitHub CLI can close issues, but lacks `read:project`; Projects-column
  synchronization has not been verified.

Next checkpoints: working RS485 forwarding; empty-gripper open/close and
width/force checks; confirmed TCP/payload; collision scene matching the bench;
small MoveIt motion; protective-stop recovery; ten recorded physical pick/place
runs. Larger motion also requires an open starting posture: this session's
initial elbow was approximately -2.768 rad (deeply folded).

## Gripper-deferred checks

Independent software reruns in isolated ROS domains/containers:

| Check | Result |
|---|---|
| Language package pytest | 98 passed, 9 live-API cases skipped; no credentials supplied |
| Offline keyword corpus | 27/27 scored, 2 LLM-only cases skipped; zero contract violations |
| Motion primitive package pytest | 7 passed |
| Shared trajectory-result regression | 4 passed |
| Perception CPU image | 55 passed; 30/30 synthetic scenes; live detect/locate and camera-stopped bag replay passed; container exit 0 |
| Connected physical camera | No ZED enumerated over USB; real-camera acceptance unavailable |

The arm-only MoveIt launcher reads the live driver's calibrated URDF and adds
a conservative fixed collision envelope for the physically attached gripper.
It creates no fake gripper feedback, hardware controller, or extra TF publisher.
The first planning-only check refused the folded starting pose because the
envelope overlapped the forearm and upper arm. No MoveIt trajectory was sent.
Operator-assisted Freedrive to an open pose was requested. The diagnostic
script defaults to planning-only; `--execute` additionally requires fresh
stationary feedback, NORMAL safety, running External Control, and by default
at most 10% speed and a maximum 0.05-rad excursion for every planned joint.
The explicitly selected `--visible` mode permits up to 0.25 rad per joint for
a 5 cm lift and return. `--max-speed-percent 50` selects the higher ceiling
approved by the operator during this visit; the default remains 10%.

After operator repositioning, the initial coarse envelope still blocked
planning. The replacement was derived from RG2 collision meshes over 111
openings (0..110 mm), including mimic joints and the mounting transform.
Tool0-frame mesh bounds were x +/-72.62 mm, y -39.43..35.89 mm,
z 0..233.59 mm. The fixed 160 x 90 x 280 mm envelope adds mounting allowance
and padding; it does not claim to measure the gripper's current opening.

With this envelope, the open starting pose was collision-free. A wrist plan
and complete vertical path succeeded. Adding a virtual obstacle overlapping
the tool caused state-validity failure and planner rejection (code -26);
removing it restored validity. No colliding trajectory was executed.

Physical MoveIt execution through `/execute_trajectory` and the scaled JTC:
wrist +0.02 rad, wrist return, 5 mm commanded lift, and commanded descent all
passed on a complete rerun. All four returned action status 4 and MoveIt code
1; worst measured joint endpoint error on that run was 0.000043 rad.
The first descent was held at planning because the relative jump heuristic
truncated the short path to 71.4%. A stationary A/B check reproduced 71.4%
with relative threshold 2, versus 100% using absolute bounds; maximum planned
waypoint change was only 0.001371 rad. The diagnostic now uses absolute jump
checking, an independent 0.01-rad waypoint audit, and the excursion cap.
The separate return descent measured 4.06 mm for 5 mm requested, consistent
with the current IK's 1 mm position tolerance; joint tracking itself was tight.

A stationary read-only FK/driver-pose comparison after repositioning found
tool-local offset (0.0000051, -0.0000006, 0.0306072) m and orientation difference
0.00202 degrees. This supports a 30.6 mm configured TCP offset; pendant
configuration and full gripper TCP validation are still outstanding.

## Visible motion at operator-approved 50% speed

The first 5 cm attempt reached the controller endpoint but the diagnostic
reported a telemetry gate failure and did not execute its return. The old
diagnostic did not record which gate failed, so the precise cause is unknown.
Subsequent checks found NORMAL safety, running External Control, zero joint
velocity, and a measured 49.34 mm rise. The operator then explicitly approved
50%; live speed telemetry confirmed 50.0%. Cancellation diagnostics now report
the failing telemetry values, cancellation response, and terminal status when
available, without claiming cancellation merely because it was requested.

A fresh lift/return from the stationary raised position completed with:

- Full collision-checked paths in both directions and joint excursions below
  0.139 rad (0.25-rad cap); maximum waypoint change below 0.0032 rad.
- Both executions returned action status 4 (SUCCEEDED), MoveIt error code 1.
- Measured lift: (0.269, 0.061, 49.092) mm.
- Maximum joint endpoint errors: 0.0000286 rad outbound, 0.0000126 rad return.
- Return position error: 0.509 mm relative to this run's starting position.
- Command nominal joint-rate cap remained 0.02 rad/s; only the approved
  pendant speed ceiling changed from 10% to 50%.

This is real arm hardware execution over direct laptop Ethernet. It does not
complete gripper, full combined TCP/named-pose, or physical protective-stop
acceptance. The robot returned to the start of the second run, approximately
5 cm above the pose preceding the first larger lift.

## Gripper setup resumed: pendant detection confirmed

The operator resumed gripper setup after the arm demonstration. The photo
`/home/nikola/Pictures/gripper2.HEIC` shows Installation -> OnRobot Setup with
`Tool connector - RG2` selected and `Detected device(s): RG2`. This confirms
pendant-side identification over the tool connector; it does not yet establish
successful jaw actuation or laptop/ROS gripper communication.

The screen's default TCP device is `QC Tool Side (1)`, not `RG2 (2)`.
Full fingertip TCP/payload validation remains pending. An earlier photo showed
an incorrectly added `RG2-FT Grip` program node with width N/A; the correct
standard-RG2 command is `RG Grip`. The next check is an isolated empty-jaw
pendant test with depth compensation disabled, before further arm motion.

The operator subsequently reported successful jaw motion using the pendant
buttons and a current width of 54.4 mm. This passes the operator-observed
pendant actuation check. Requested-width accuracy, ROS action completion,
grasp detection, and object retention have not yet been measured.

The operator installed the official RS485 1.0 URCap from the verified USB
copy and restarted PolyScope. Port 54321 became reachable. Initial tool
output was 0 V; the operator changed it to 24 V and saved. A read-only
primary-interface snapshot then confirmed communication enabled, 1,000,000
baud, parity 2 (even), one stop bit, RX idle 1.5, TX idle 3.5, output modes
[0, 1, 1], and 24 V. External Control remained stopped, with NORMAL safety.

The read-only RTU diagnostic received no response from units 65 or 66, but
unit 67 returned CRC-valid register replies. Registers 267 and 275 both read
0xFF55 (-17.1 mm as signed data); status 268 and fingertip offset 258 were
zero. This does not match the last pendant-reported 54.4 mm and is not accepted
as usable feedback. Function 43 device identification returned exception 1
(unsupported function). No RTU write or gripper motion command was sent.
Device identity/firmware and the address discrepancy remain under diagnosis;
do not mark ROS gripper control complete from bridge reachability alone.

Selecting OnRobot's `No connection` and cycling tool power 0 V -> 24 V did
not resolve the discrepancy: unit 65 still timed out and unit 67 still
returned -17.1 mm with status zero. Actual tool communication remained
enabled at 1M/even/1, with 24 V and approximately 0.082 A reported. Returning
temporarily to OnRobot ownership was requested to retrieve the actual device
model/firmware before further diagnosis. Do not send a gripper goal using
the invalid unit-67 width or assume that address 67 is the attached gripper.

Restoring OnRobot ownership and `Tool connector` restored detection. The
operator's `/home/nikola/Pictures/DeviceInfo.HEIC` shows RG2 serial 1000042561,
firmware 1.0.8.600361162, system health `Ok`, and grip counter 2572. Selected
IP is localhost; no Compute Box version is shown. The screenshot also shows
speed 100%; the operator was asked to restore the approved 50% ceiling before
any further arm test. Actual native tool configuration still reports
1M/even/1 and 24 V. These observations do not establish a firmware defect or
compatibility diagnosis; a USB copy of the working generated program was
requested to inspect its device initialization.

## Native OnRobot interface and real ROS gripper acceptance

The USB export was copied and verified under
`/home/nikola/Downloads/ur7e-lab-export-20260928/`: `11142025.script`,
`11142025.urp`, `default.installation`, and `default.variables`.
The generated script identifies OnRobot URCap 6.5.0, tool index 2 (secondary),
and the native XML-RPC endpoint `http://localhost:41414/`. Its grip call is
`rg_grip(tool_index, width_mm, force_n)`. The exported program also contains
External Control and a subsequent full-close grip command; it was inspected,
not executed. Tool index 2 is consistent with the secondary address but does
not explain the invalid raw RTU feedback or prove that disabling the native
daemon would resolve it.

The endpoint is reachable from the laptop at `http://192.168.56.101:41414/`.
Its introspection confirmed getter and command signatures. Read-only discovery
reported RG2, product code 32, device ID 2, serial 1000042561, firmware
1.0.8.600361162, and zero status/warning/error. After the operator set 59 mm,
three native samples returned exactly 59.0 mm, idle and healthy. Dashboard
reported NORMAL safety, powered robot, and program stopped.

`scripts/lab_rg2_native.py` is read-only by default. Its supervised demo checks
the device serial/model, stopped arm program, safety state, finite/in-range
feedback, <=20 mm width changes, <=10 N requested force, and completion within
8 seconds. Three settled samples within 1 mm are required. Errors/cancellation
attempt native `rg_stop` and verify idle; an unconfirmed stop is a failure.
These are software checks, not safety-rated interlocks.

Real native demo results, with the operator confirming both visible movements:

| Target | Requested force | Actual width | Time | Busy observed |
|---|---|---|---|---|
| 70 mm | 10 N | 70.5 mm | 0.800 s | yes |
| 59 mm | 10 N | 58.4000 mm | 0.917 s | yes |

`scripts/lab_rg2_action.py` exposes the same action name and units as the
existing project controller: `/gripper_action_controller/gripper_cmd`,
`control_msgs/action/GripperCommand`, width in metres and effort limit in N.
It uses the native backend, preserves OnRobot ownership, and intentionally
requires a stopped arm program. It is a standalone bench adapter, not yet
integrated into the combined hardware launch. Limits are 40–80 mm targets,
<=20 mm per move, and 0<force<=10 N. Measured effort is unavailable and is
reported as NaN. No joint-state or grasp-retention claim is made.

Real ROS action results through a CLI-launched rclpy client in domain 42:

| Goal | Result |
|---|---|
| 120 mm / 5 N | rejected before motion |
| 70 mm / 5 N | SUCCEEDED (4), reached_goal=true, actual 70.2000 mm; 13 feedback samples |
| 59 mm / 10 N | SUCCEEDED (4), reached_goal=true, actual 58.4000 mm; 9 feedback samples |
| 70 mm / 11 N | rejected before motion |
| 70 mm / 5 N, immediate cancel | cancellation accepted, CANCELED (5), reached_goal=false, idle confirmed at 58.4000 mm |
| 59 mm / 10 N after cancellation | SUCCEEDED (4), actual 58.4000 mm |

The immediate cancellation did not establish a mid-motion stopping distance.
Requested force settings were transmitted, not independently measured. The
reported operation counter remained 2573, so it is not used as execution proof.
Arm program remained stopped; no arm command was issued during these checks.
Eight native diagnostic unit tests cover invalid feedback, bounds, gates,
settled completion, cancellation, and stop failures. Raw RTU tests remain
separate. Original #8 raw-bridge acceptance is still open. #9's standalone
real-action width/force-command acceptance is satisfied using the native
backend. Combined TCP/payload, production launch integration, simultaneous
arm/gripper operation, and object pick/place remain unverified.

Run in the lab container, after sourcing ROS and workspace setup:

```sh
ROS_DOMAIN_ID=42 python3 scripts/lab_rg2_action.py
```

Do not run this alongside the stock gripper action controller or another
gripper command process. Keep the empty gripper supervised and leave the arm
program stopped. The gripper remains at approximately 58.4 mm after the tests.

After acceptance, the temporary ROS bench adapter was stopped and its process
exit verified. A final native read confirmed width 58.4000 mm, busy=false,
speed=0, and no reported errors. GitHub #9 was closed with the exact lab limits
and uncommitted-code status disclosed; #8 received a progress update and stays
open. The GitHub Project board column could not be verified with the available
token scope.

## RViz showcase after gripper acceptance

The simulation used a separate Docker container with `--network none`, private
IPC, ROS domain 84, localhost-only DDS, mock hardware explicitly enabled,
robot IP 127.0.0.1, and dashboard client disabled. This prevented simulator
commands from reaching the connected hardware/domain 42.

The original `pick_place_demo.py` reached the grasp but failed at lift because
the fingers collided with `pick_object`; it had not attached that object to
the planning scene. Added `scripts/showcase_sim.py`, guarded to run only in
a network-disabled Docker container/domain 84. It attaches the object to
`gripper_tcp`, permits contact with the two inner fingers, and detaches at
the actual transformed release pose. The gripper closes to the object's
30 mm width. The placeholder destination collision box is removed because it
represents the empty placement location. Table/arm collision checking remains
enabled. This is kinematic scene attachment, not contact/grasp physics.

One full run completed all stages and returned home. The released object's
reported world position was (0.450469, 0.200591, 0.041027) m versus target
(0.45, 0.20, 0.04) m. Raw successful output is retained in
`evidence/2026-09-28/showcase-complete.log`. A subsequent replay carried and
released the object but failed to submit the `observe` return motion; its
output is retained in `evidence/2026-09-28/showcase-replay.log`. This failure
was not investigated further after the operator ended the lab session.
`scripts/showcase.rviz` provides a cleaner live-state view with planning
ghosts hidden; the revised view was launched but not visually reverified at
closeout. Neither #14's physical repetition criteria nor #30's Gazebo physics
criteria are fulfilled by this showcase.
