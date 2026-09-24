# Simulation & Working Without the Robot

How to develop for this project with no lab access. The design and rationale
live in [ARCHITECTURE.md, decision D7](ARCHITECTURE.md); this is the hands-on
guide. Each tier is faithful to a *different layer* of the stack — pick the
lowest tier that exercises what you're changing:

| Tier | Faithful to | Use it when you touch | Status |
|---|---|---|---|
| 1 — Mock hardware | Kinematics, controllers, planning, orchestration logic | Launch files, MoveIt, nodes above the driver | **working** (gates CI) |
| 2 — URSim | Driver ↔ controller behavior: bringup handshake, dashboard, protective stop | Bringup, recovery logic, anything talking to URControl | **working** (this doc) |
| 3 — Gazebo | Physics + camera: the full language → detect → pick pipeline | Perception, grasp sequencing | **motion working, grasp latch open** (`src/ur7e_gazebo`) |

One rule to internalize: **sim results never sign off grasp quality or
calibration** — those are validated on hardware only (plan tasks 1.7, 4.4).
Sim exists to make everything *around* them fast and lab-free.

## Getting a dev environment (devcontainer)

The repo ships a VSCode devcontainer (task 0.8): full ROS 2 Humble desktop +
UR stack + MoveIt + Gazebo bridge, matching the Jetson's middleware settings
(`ROS_DOMAIN_ID=42`, CycloneDDS).

1. Install Docker and VS Code with the *Dev Containers* extension.
2. `git clone` this repo, open the folder in VS Code, accept **"Reopen in
   Container"** (first build downloads a few GB — one-time).
3. In the container terminal:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true launch_rviz:=true
```

RViz showing the arm = your environment works (the task 0.8 acceptance test).

- **Linux:** run `xhost +local:` on the host once per session so the
  container may use your X server.
- **Windows:** use WSL2; WSLg provides the display automatically.
- **macOS:** no usable X/GPU path — work headless (build, tests, rosbags,
  `gz sim -s`) and use [Foxglove Studio](https://foxglove.dev) instead of
  RViz. URSim is amd64-only, so tier 2 is also out on Apple Silicon; tier 1
  and CI cover you.

## Tier 1 — mock hardware (runs anywhere, gates CI)

The real driver's robot description is loaded with mocked hardware: every
controller, topic, and action exists — `scaled_joint_trajectory_controller`,
`/joint_states`, TF — but setpoints are simply mirrored back as states. No
physics, no URControl. Perfect for launch plumbing, MoveIt, and node logic;
useless for timing or contact.

```bash
ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true
```

**Phase 1 (arm + RG2 gripper + MoveIt2) also runs fully on tier 1** — this is
what "hardware-free development" concretely means for the pick-and-place
stack, not just the bare-arm bringup above. First, one-time setup:

```bash
bash scripts/vendor_import.sh   # pulls the third-party UR+RG2 packages, D6 — see ur7e.repos
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

Then, three terminals:

```bash
ros2 launch ur7e_pick_place_bringup ur7e_pick_place.launch.py use_fake_hardware:=true
ros2 launch ur7e_pick_place_bringup ur7e_moveit.launch.py launch_rviz:=true
ros2 run ur7e_motion motion_node
```

...and task 1.7's scripted pick, end to end, with the table collision object
enforced and `pick_ik` actually solving IK (verified this session — see
`docs/RUNBOOK.md` and `docs/IMPLEMENTATION_PLAN.md`'s Phase 1 status note):

```bash
python3 scripts/pick_place_demo.py
```

**Known mock-hardware quirk:** setpoints are mirrored straight back as state
with no continuous-joint wrapping — after many relative Cartesian moves in
one long-lived process, a wrist joint can accumulate to a value like `6.28`
rad (two full turns) instead of wrapping to `~0`. Planning FROM such a state
can fail outright. It's a mock-hardware artifact (a real driver wraps
continuous joints), not a planning bug — if a primitive starts failing to
"submit" after a long ad-hoc testing session, restart the bringup stack
before assuming the code is broken.

The smoke test codifies "tier 1 works" (CI runs it on every push — task 0.7):

```bash
bash scripts/tier1_smoke_test.sh
```

It asserts the controller goes active, `/joint_states` flows, and a
`FollowJointTrajectory` goal succeeds — the same action interface MoveIt and
the orchestrator use later, and the sim twin of lab session 2's first
commanded motion.

## Tier 2 — URSim (the real controller software, simulated)

URSim is Universal Robots' own controller simulator: the same URControl that
runs in the cabinet, with a PolyScope GUI in your browser. The ROS driver
cannot tell the difference — which is the point. Rehearse here what you
intend to do in the lab: bringup, dashboard calls, **protective-stop
recovery** (task 0.6's drill).

Pinned to `5.23` = our robot's PolyScope version (RUNBOOK session 1);
simulating a different controller version than the real robot invites
works-in-sim-only bugs. amd64 only: x86 Linux/Windows, not the Jetson, not
Apple Silicon.

**Network design worth understanding:** the container gets the *real robot's
IP* (`192.168.56.101`) on a private Docker subnet whose gateway — your
machine — is `192.168.56.1`, the Jetson's lab address. So the bringup
command, and the External Control URCap's target IP, are byte-identical
between sim and lab. What you rehearse is what you type in the lab.

```bash
cd sim/ursim
./get_urcap.sh            # once: fetch the External Control URCap
docker compose up -d
```

Open PolyScope at <http://localhost:6080/vnc.html>, then (first run only):

1. **Install the URCap** — our pinned 5.23 image does *not* auto-install
   mounted URCaps (newer image tags do). ☰ (top-right) → **Settings → System →
   URCaps** → **+** → the picker opens in the programs folder; enter `urcaps/`
   and select `externalcontrol-1.0.5.urcap` → it lands under Active URCaps →
   **Restart**. Restarting PolyScope **exits the container** (PolyScope is its
   main process): run `docker compose up -d` again and reconnect the VNC page.
   The install persists, so this is truly once.
2. Confirm the robot is powered on (bottom-left status → ON → START).
3. Create a program: **Program → URCaps → External Control** (a single
   "Control by 192.168.56.1" node — the URCap's Installation defaults are
   already our host IP and port 50002). Save it (☰ → Save All). PolyScope
   pops an on-screen keyboard for the name; program names are safest as
   plain letters, e.g. `externalcontrol`.

Then from your workspace (host or devcontainer, Linux):

```bash
ros2 launch ur7e_bringup ur7e_bringup.launch.py    # same command as the lab
```

Press **Play** in PolyScope — the driver log must print *"Robot connected to
reverse interface. Ready to receive control commands."* (the same line lab
session 2 recorded on the real robot).

Two log lines that are **expected** against URSim and not against the lab robot:

- *"calibration parameters ... don't match"* — our bringup bakes in the *real*
  robot's factory calibration, and URSim is a generic UR7. Harmless here;
  on the real robot this same message would mean something is genuinely wrong.
- A slow first bringup: URSim's controller interface can take >10 s to come up
  after power-on, which is why our launch raises `controller_spawner_timeout`
  to 120 s (upstream default 10 s lost this race).

Now send motion, e.g. the smoke test's trajectory, or drill the
protective-stop recovery:

To *provoke* a protective stop (the dashboard can only unlock one, not cause
one), use a **position** limit, not a velocity: Safety → Joint Limits (needs
the safety password, `easybot1` in our sim), restrict e.g. the base's range so
a slow, legal trajectory crosses the line, apply (forces a power cycle), then
send that trajectory. Two dead ends, learned the hard way (rehearsal session,
RUNBOOK 2026-09-17):

- A "too fast" trajectory (short `time_from_start`) does **not** cause a
  protective stop: the External Control URCap's own velocity guard vetoes the
  setpoints first (*"Ignoring commands until a valid command is received"*
  popup; goal aborts; program keeps running; recovery = the popup's Continue).
- URSim with factory-preset safety limits didn't enforce speed limits at all
  in our tests — another reason sim never signs off safety.

(The on-screen red button is an *emergency* stop, a different category with a
different recovery — worth trying separately to see the difference.)

Then recover — all three steps, in order:

```bash
ros2 service call /dashboard_client/unlock_protective_stop std_srvs/srv/Trigger
# (real robot enforces ~5 s before unlock is accepted; URSim mimics this)
```

then **restart the driver** (Ctrl+C the bringup, relaunch): after a mid-goal
protective stop the trajectory controller holds its stale pre-stop command,
and reconnecting without a restart wedges the arm in an endless veto loop.
Then re-press Play (our default non-headless ritual) to restore control, and
verify with a small legal goal. A goal *rejected* at submission (vs *aborted*
mid-flight) means External Control isn't running — Play wasn't pressed.

Done for the day: `docker compose down` — your External Control program
survives it (persisted in the `ursim_programs` volume), so the first-run
PolyScope setup never has to be repeated.

Windows note: `--network=host` in the devcontainer doesn't reach the URSim
bridge network from inside Docker Desktop; either run the driver in WSL2
directly, or attach the devcontainer to the URSim network:
`docker network connect ursim_ursim_net <devcontainer-name>`.

## Tier 3 — Gazebo (motion working, grasp latch open)

Ignition Gazebo 6 (Fortress) — `src/ur7e_gazebo`. Real physics: gravity,
inertia, actual simulated motor response, as opposed to tier 1's mirrored
setpoints.

```bash
ros2 launch ur7e_gazebo ur7e_gz.launch.py gazebo_gui:=false   # true on a workstation with a display
ros2 launch ur7e_pick_place_bringup ur7e_moveit.launch.py launch_rviz:=false \
    use_sim_time:=true moveit_controllers_file:=config/ur7e_gz_moveit_controllers.yaml
ros2 run ur7e_motion motion_node --ros-args -p use_sim_time:=true
```

Everything above `ros2_control` — MoveIt2, `motion_node`,
`scripts/pick_place_demo.py` — is the exact same code tier 1 uses; only
what's *behind* `ros2_control` changed. Verified: sending
`goto_named home` through `ExecutePrimitive` moves the arm under real
simulated dynamics (checked `/joint_states` before/after against actual
physics, not a mirror).

**Not yet working:** the DetachableJoint grasp latch (D7 tier-3 item 2 —
weld the object to the gripper on command) and reliable gripper
actuation in sim. Both were investigated in real depth this session — not
guessed at — and the findings are written up in full in
`src/ur7e_gazebo/README.md`, including the specific sdformat behavior
(fixed-joint lumping) that's the leading suspect for the grasp latch issue.
Known sim/real deltas from D7 in ARCHITECTURE.md (grasp always "succeeds" in
sim once the latch works, sim depth is noise-free, unscaled controller) still
apply once it does.

## What runs where (quick reference)

| Environment | Tier 1 | Tier 2 (URSim) | Tier 3 (Gazebo) | RViz |
|---|---|---|---|---|
| Linux x86 | ✔ | ✔ | ✔ (when built) | ✔ |
| Windows (WSL2 + WSLg) | ✔ | ✔ | ✔ (when built) | ✔ |
| macOS (incl. Apple Silicon) | ✔ | ✗ (amd64 image) | headless only | ✗ → Foxglove |
| CI (GitHub Actions) | ✔ (smoke test) | ✗ (needs URCap click-through) | ✗ | ✗ |
| Jetson | ✔ (but why) | ✗ (arm64) | ✗ (no GPU headroom) | ✗ (headless) |
