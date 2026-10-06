# Cold start to a language-directed pick (one page)

The whole stack on **Nikola's laptop** (no Jetson): UR7e over the direct
Ethernet link, RG2 through the OnRobot URCap, ZED 2i used as a plain USB
camera, everything ROS in one Docker container. Verified 2026-10-02.
Every command below is a copy-paste; "pendant" steps are the only manual ones.

> **2026-10-06: do not run the §3 calibration wave from `front` with an
> object in the gripper.** Four protective stops (C153) came from that start
> pose: in it the gripper hangs ~10 cm from the arm's own forearm, and the
> approach swings the forearm over the held object. Clear the plate and
> start from an open, taught pose, as in
> [LAB_2026-10-06_STEREO_CALIBRATION.md](LAB_2026-10-06_STEREO_CALIBRATION.md)
> ("Next session"). `lab_stereo_calibration.py` now refuses such moves.

## 0. Power-on order (5 min)

1. Robot controller on → pendant: **Power on**, **Brake release** (robot mode RUNNING).
2. Ethernet cable laptop ↔ robot. On the laptop: `nmcli con up ur-link` (laptop
   becomes 192.168.56.1; `ping 192.168.56.101` must answer).
3. ZED on a USB-3 port, pointed at the pick area (see §3). Do not touch it after calibration.
4. Pendant: load `ros2_external_control.urp`, speed slider at the level you
   are prepared to watch (25 % for anything new, 70 % for a rehearsed demo).

## 1. Container and robot driver (3 min)

```bash
cd ~/Projects/Makerspace/AI_arm
docker start ur7e-lab-20261002 || docker run -d --init --name ur7e-lab-20261002 --network host \
  --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 \
  --device-cgroup-rule 'c 81:* rmw' --group-add video \
  -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$PWD:/workspaces/AI_arm" -w /workspaces/AI_arm ur7e-task2:visual sleep infinity
docker exec ur7e-lab-20261002 bash -lc 'pip install --user -q anthropic "brotli>=1.1" "pytest>=7,<9"'
```

Then, in **four** container terminals (`docker exec -it ur7e-lab-20261002 bash`,
and in each: `source install/setup.bash && source scripts/lab_env.sh` — the
second file is not optional, see "gotchas"):

```bash
# T1 driver (wait for "Configured and activated scaled_joint_trajectory_controller")
ros2 launch ur7e_bringup ur7e_bringup.launch.py launch_rviz:=false
# pendant: press Play. If a goal is later REJECTED with "Controller is not running":
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [scaled_joint_trajectory_controller], strictness: 1, activate_asap: true}"
# T2 MoveIt (KDL for 1 mm Cartesian steps)
ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl
# T3 perception (only after the calibration in §3 exists)
OMP_NUM_THREADS=4 ros2 run ur7e_perception webcam_perception_node --ros-args \
  -p calibration:=$PWD/scripts/lab_table.json -p objects_file:=$PWD/scripts/lab_objects.yaml \
  -p snapshot_dir:=$PWD/log/perception -p backend:=auto -p reach_min:=0.08 -p reach_max:=0.5 \
  -p default_height:=-1.0
```

Quick checks: `python3 scripts/lab_rg2_native.py` (gripper answers),
`python3 scripts/lab_snapshot.py log/zed/look.jpg` (camera frame via the node).

## 2. First motion of the day

```bash
python3 scripts/lab_console.py --execute --max-speed-percent 25 --max-excursion 2.5 --say "go to front"
```

`front` is the top-down hub over the pick area; `observe` (taught per session
with `/teach observe` in the console) is 10 cm higher and is where every pick
starts and ends.

## 3. Camera calibration (6 min, only when the camera has moved)

No printed board needed. The robot waves an object it holds; the camera
learns where the robot is.

```bash
python3 scripts/lab_table_calibration.py zed --conf docs/calibration/zed2i_SN35717973.conf   # once per camera
python3 scripts/lab_camera_calibration.py grip --width 75         # hand over a blue or white object
python3 scripts/lab_camera_calibration.py grip --width 5          # ... it closes on it
python3 scripts/lab_camera_calibration.py --execute --max-speed-percent 50 wave --target-color blue \
    --xs 0.19 0.24 0.29 --ys -0.02 0.08 0.18 --heights 0.27 0.36 0.45
python3 scripts/lab_camera_calibration.py solve                   # want: RMS < 5 px, >= 12 poses
python3 scripts/lab_camera_calibration.py --execute touchdown --kind tips --xy 0.22 0.12 \
    --travel-z 0.40 --start-z 0.22 --floor-z 0.10 --step 0.003    # closed fingertips feel the plate
python3 scripts/lab_camera_calibration.py finish --table-z 0.2025 # see note
python3 scripts/lab_camera_calibration.py map                     # log/calibration/map.jpg: grid on the plate
```

Note on `--table-z`: it is "tool0 height when the held object's centre is at
the plate" in the frame the wave defined. Get it right by having the robot set
the object down at a known spot and comparing with `scripts/lab_detect.py`
(the 2026-10-02 value for the hat held by its brim was 0.2025; the hat's
centre hangs 5.4 cm below the fingertips). The tips touchdown (0.1485) does
not depend on the camera and survives camera bumps.

## 4. The demo

```bash
# one sentence, scripted
python3 scripts/lab_pick.py --execute --max-speed-percent 70 --yes --open-mm 108 --force 15 \
    --tip-clearance 0.025 --say "pick up the blue hat"
# the console: typed or spoken sentences, jogs and picks
python3 scripts/lab_console.py --pick --execute --max-speed-percent 70 --backend claude \
    --open-mm 108 --tip-clearance 0.025 --voice-inbox
# on the HOST, for voice: .venv-voice/bin/python scripts/voice_input.py --auto --wake robot
# repeatability: N runs, object set down at a new random spot each time
python3 scripts/lab_benchmark.py pick --say "pick up the blue hat" --runs 10 \
    --area 0.21 -0.03 0.26 0.10 --execute --max-speed-percent 70 --yes --open-mm 108 \
    --force 15 --tip-clearance 0.025 --hover 0.15
```

## 5. Recovery

| Symptom | Do |
|---|---|
| Protective stop (arm hit something) | pendant Unlock → kill and relaunch T1 → Play → re-activate the controller if a goal is REJECTED |
| "grip lost" popup, program stopped | dismiss, Play. Cause: empty jaws closed to 0 mm register as a grip; never command < 5 mm empty |
| RG2 `safety_failed` / `s1_triggered` | pendant OnRobot toolbar → reset (tool power cycle also works); then open/close once |
| `Failed to find a free participant index` | a process was started without `scripts/lab_env.sh` |
| Camera moved | §3 again (the tips touchdown can be skipped) |
| Link down after a robot reboot | `nmcli con up ur-link`, then as RUNBOOK 2026-09-30 |

Gotchas that cost time on 2026-10-02: the pick plate is a *raised* piece
11 cm below the robot's platform, and the clamp area behind it is lower
still — objects go on the raised front piece only; the RG2's fingers slope
inward above the pads, so wide round objects need `--tip-clearance 0.025`
and `--open-mm 108`; the gripper reports `busy` while squeezing a slipping
object and `width` shrinks although it is still held.
