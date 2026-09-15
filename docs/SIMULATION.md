# Simulation & Working Without the Robot

How to develop for this project with no lab access. The design and rationale
live in [ARCHITECTURE.md, decision D7](ARCHITECTURE.md); this is the hands-on
guide. Each tier is faithful to a *different layer* of the stack — pick the
lowest tier that exercises what you're changing:

| Tier | Faithful to | Use it when you touch | Status |
|---|---|---|---|
| 1 — Mock hardware | Kinematics, controllers, planning, orchestration logic | Launch files, MoveIt, nodes above the driver | **working** (gates CI) |
| 2 — URSim | Driver ↔ controller behavior: bringup handshake, dashboard, protective stop | Bringup, recovery logic, anything talking to URControl | **working** (this doc) |
| 3 — Gazebo | Physics + camera: the full language → detect → pick pipeline | Perception, grasp sequencing | planned (issues #30, #31) |

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

1. Confirm the robot is powered on (bottom-left status → ON → START).
2. Create a program: **Program → URCaps → External Control**; set Host IP to
   `192.168.56.1` (already the URCap default port 50002). Save it.

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

```bash
# trigger a protective stop from the dashboard, then recover:
ros2 service call /dashboard_client/unlock_protective_stop std_srvs/srv/Trigger
# (real robot enforces ~5 s before unlock is accepted; URSim mimics this)
# then re-press Play (our default non-headless ritual) to restore control.
```

Windows note: `--network=host` in the devcontainer doesn't reach the URSim
bridge network from inside Docker Desktop; either run the driver in WSL2
directly, or attach the devcontainer to the URSim network:
`docker network connect ursim_ursim_net <devcontainer-name>`.

## Tier 3 — Gazebo (planned)

Lands with issues [#30](https://github.com/PawPrintStudio/ur7e-language-pick-and-place/issues/30)
(world + RG2 + grasp latch) and [#31](https://github.com/PawPrintStudio/ur7e-language-pick-and-place/issues/31)
(overhead RGB-D camera + OWLv2 perception backend). The devcontainer already
ships the Gazebo Fortress bridge (`ros-humble-ros-gz`) in anticipation.

## What runs where (quick reference)

| Environment | Tier 1 | Tier 2 (URSim) | Tier 3 (Gazebo) | RViz |
|---|---|---|---|---|
| Linux x86 | ✔ | ✔ | ✔ (when built) | ✔ |
| Windows (WSL2 + WSLg) | ✔ | ✔ | ✔ (when built) | ✔ |
| macOS (incl. Apple Silicon) | ✔ | ✗ (amd64 image) | headless only | ✗ → Foxglove |
| CI (GitHub Actions) | ✔ (smoke test) | ✗ (needs URCap click-through) | ✗ | ✗ |
| Jetson | ✔ (but why) | ✗ (arm64) | ✗ (no GPU headroom) | ✗ (headless) |
