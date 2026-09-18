# Runbook

Living document. Each lab session appends its verified findings; the final one-page cold-start guide is task 4.5.

## Lab session 1 — Phase 0 platform + robot link (issues #1–#4)

State going in: Jetson confirmed on L4T R36.4.7 (JetPack 6.2.x, Ubuntu 22.04) — **no reflash needed**. Pendant confirmed **PolyScope 5.23** (issue #1 done). URSim should be pinned to the matching tag: `universalrobots/ursim_e-series:5.23` — simulating a different controller version than the real robot invites works-in-sim-only bugs.

### A. At the pendant (no SSH) — issue #1

1. Record PolyScope software version: Settings → System (fill into the table below).
   - Driver needs ≥ 5.9.4 (PolyScope 5) or ≥ 10.7.0 (PolyScope X). If PolyScope X → read ur_robot_driver issue #1859 before phase 1.
2. Record the robot's IP: Settings → System → Network (and whether it's static).
3. Check installed URCaps: is **External Control** present? Open its settings and record the configured Host IP + port (should be the Jetson's Ethernet IP, port 50002).
4. Record whether the pendant is in Local or Remote mode.

### B. On the Jetson (one SSH session, copy-paste blocks)

**B1. System verification + Super mode**

```bash
lsb_release -d            # expect Ubuntu 22.04
apt show nvidia-jetpack 2>/dev/null | grep Version
free -h && df -h /        # headroom check
sudo nvpmodel -q          # current power mode
grep -B1 POWER_MODEL_DEFAULT /etc/nvpmodel.conf | head; grep '< POWER_MODEL' /etc/nvpmodel.conf
```

Pick the **MAXN SUPER** mode id from the listed modes and set it (persists across reboots):

```bash
sudo nvpmodel -m <MAXN_SUPER_MODE_ID>
```

**B2. Inventory the existing install** (the keyboard-teleop project ran here — reuse, don't duplicate)

```bash
ls /opt/ros/ 2>/dev/null
apt list --installed 2>/dev/null | grep -E 'ros-humble-(ur$|ur-|desktop|moveit)' | head -20
ls ~/ur_ws/src 2>/dev/null
```

**B3. Install/update the driver stack** (needs internet on the Jetson — if apt can't resolve, put the Jetson's WiFi on an internet-connected network first; the robot Ethernet link is unaffected)

```bash
sudo apt update && sudo apt install -y ros-humble-ur ros-humble-moveit
```

**B4. Network link to the robot** — issue #3

Robot IP confirmed at the pendant: **192.168.56.101** (the teleop-era config — keep it; changing both ends of a working link invites debugging). The Jetson's Ethernet side must be on the same subnet:

```bash
ip -br addr                   # identify the Ethernet interface (eth0/enP...) and its IP
ping -c 3 192.168.56.101
```

If the Jetson's Ethernet isn't already static on 192.168.56.x, set it (why: DHCP with no server on a direct cable loses the address on reboot):

```bash
sudo nmcli con add type ethernet ifname <ETH_IFACE> con-name ur-link ip4 192.168.56.1/24
sudo nmcli con up ur-link
```

Then re-run the ping. Also confirm the pendant's Network screen says **Static**, not DHCP.

**B5. Driver smoke test** — issue #2 acceptance

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur7e robot_ip:=192.168.56.101 launch_rviz:=false
```

(If `ur_type:=ur7e` is rejected, the packages are stale — re-run B3 and note it.)

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

Expect `scaled_joint_trajectory_controller [active]`. Then on the pendant: open/create the External Control program (Host IP = Jetson's Ethernet IP from B4) and press **Play**. The driver log should print that the robot is connected to the reverse interface.

**B6. Factory kinematics extraction** — issue #4 (do this while connected; takes seconds)

```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.56.101 target_filename:="$HOME/ur7e_calibration.yaml"
head -5 ~/ur7e_calibration.yaml
```

Back on the laptop (still on the private network), pull the file so it can be committed:

```bash
scp jetson@ubuntu.local:~/ur7e_calibration.yaml ./config/ur7e_calibration.yaml
```

### C. Record before leaving the lab

| Item | Value |
|---|---|
| PolyScope version | **5.23** (PolyScope 5 — ≥5.9.4 driver minimum met; PolyScope X concerns retired) |
| Local/Remote mode | **Manual** (3-mode pendant: Manual / Automatic / Remote Control → Remote Control is enabled on this unit; stay Manual for session 1) |
| External Control URCap installed? Host IP/port | **Yes, active** — Host IP `192.168.56.1`, Custom port `50002` (driver default), Host Name `192.168.56.1`. Also active: UR Connect. → Jetson eth must hold 192.168.56.1 (B4). |
| JetPack version (apt) | **6.2.1+b38** (L4T R36.4.7, Ubuntu 22.04.5) |
| MAXN SUPER mode id used | id **2** — was already the active mode |
| Ethernet iface + Jetson IP / robot IP | **enP8p1s0** — Jetson 192.168.56.1/24 ↔ robot 192.168.56.101 (static, NM profile "Wired connection 1", `ipv4.method: manual`, autoconnect — no separate `ur-link` profile was needed) |
| `ur_type:=ur7e` accepted? | **Yes** — Humble binary driver loaded hardware `ur7e`; dashboard reports robot version 5.23.0.0; RTDE v2 @ 500 Hz |
| Controllers active | `scaled_joint_trajectory_controller`, `joint_state_broadcaster`, `io_and_status_controller`, `speed_scaling_state_broadcaster`, `force_torque_sensor_broadcaster`, `tcp_pose_broadcaster`, `ur_configuration_controller`, `friction_model_controller` (others loaded inactive) |
| Calibration YAML extracted? | **Yes** — committed as `config/ur7e_calibration.yaml`, since moved to `src/ur7e_bringup/config/` where the bringup launch installs and defaults to it (hash calib_12445833238222042106). TCP spot-check vs pendant pending (issue #4 acceptance). |
| Anything that errored (paste text) | apt offline on robot network (expected — DNS unavailable; install over WiFi) |
| Notable | `/opt/ros` has **humble and rolling** — ensure shells source humble. **No `ros-humble-ur*` was installed** and `~/ur_ws/src` holds only `ur_dev_bringup` → the teleop-era driver never ran from this Jetson via apt; fresh install required. ~937 GB disk, 3.7 GB swap present. |

**Networking notes (learned the hard way):** from the laptop, address the Jetson as `jetson@ubuntu.local` — the `.local` suffix uses mDNS (the Jetson answers for itself via avahi), which works on the private network where plain DNS has no entry for it. The Jetson keeps two links at once: built-in Ethernet -> robot (192.168.56.1), USB-Ethernet adapter -> internet for package installs; keep that adapter with the robot kit.

**Session 1 watch list:**
- **Checksum question (verify in session 2):** driver (launched WITHOUT our kinematics file) printed `calib_12788084448423163542` and warned of calibration mismatch; our extracted YAML carries `calib_12445833238222042106`. Expected explanation: the printed value is the default kinematics file's hash, ours is the robot's true one. Definitive test: relaunch with `kinematics_params_file:=.../config/ur7e_calibration.yaml` — the mismatch ERROR must disappear. If it persists: re-extract and investigate before trusting any TCP pose.
- `Could not enable FIFO RT scheduling policy` — stock kernel denies RT priority to the control thread. Harmless until "reverse interface dropped" appears under load; fixes (rtprio limits / lowlatency kernel / core isolation) are catalogued in ARCHITECTURE.md risks.
- Robot logged error code `C210A0` at driver startup, then went NORMAL/RUNNING. Watch for recurrence.
- External Control **Play was not exercised** in session 1 — driver bringup verified, motion path not yet. First item of session 2.

## Lab session 2 — results (issues #3, #4; run remotely over SSH, 2026-09-04)

Done entirely from the laptop over SSH — no pendant access. Plan items 1–3 executed; item 4 (first motion) needs a human at the arm and is queued for the next lab visit.

### 1. Checksum question — CLOSED (issue #4)

Relaunched the stock driver launch with our extracted calibration wired in:

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur7e robot_ip:=192.168.56.101 launch_rviz:=false kinematics_params_file:=/home/jetson/ur7e_calibration.yaml
```

Driver log now prints `Calibration checksum: 'calib_12445833238222042106'` → **“Calibration checked successfully.”** — the session-1 mismatch ERROR is gone, and the log has zero ERROR lines. Confirms the hypothesis: the session-1 `calib_12788084448423163542` was the hash of the *default* kinematics file shipped with `ur_description`; our YAML carries the robot's true calibration. Every future launch must pass `kinematics_params_file` (this is why task 0.5's bringup launch bakes it in). Verified twice — before and after the reboot test.

### 2. TCP spot-check — FK validated to <0.1 mm; pendant glance still wanted (issue #4)

Why this works without the pendant: `/tcp_pose_broadcaster/pose` reports the **controller's** TCP (via RTDE — the same number the pendant Move screen displays), while TF `base → tool0` is **our URDF + calibration** computing FK from `/joint_states`. Comparing them checks the calibration chain against the robot's own ground truth, with more digits than the pendant shows.

At the parked pose (joints ≈ [-1.0505, -2.5947, -2.3083, -5.6924, 5.5277, 5.3739] rad):

| Source | x | y | z (m, `base` frame) |
|---|---|---|---|
| TF `base→tool0` (our FK) | -0.101107 | -0.222188 | 0.230576 |
| Controller TCP (RTDE) | -0.131047 | -0.225229 | 0.225140 |

Orientations match to <1e-4 (quaternion). The translation gap, rotated into the tool frame, is **[0.01, 0.03, 30.58] mm** — a pure +30.6 mm offset along tool-Z with ≤0.03 mm residual in x/y. That is a **TCP offset configured on the pendant**, and 30.5 mm is exactly the OnRobot Quick Changer height (the QC sits on the flange; cf. task 1.3's `0.2 kg QC` payload note). So the FK chain agrees with the controller to hundredths of a millimeter once the pendant TCP is accounted for.

Remaining to fully close #4 (next lab visit, 2 min at the pendant):
- Confirm Installation → General → TCP shows z ≈ 30.5 mm (and note the exact value + payload).
- Repeat the comparison at 1–2 more arm poses (freedrive the arm, re-run the TF-vs-topic readout). One pose can't mathematically distinguish "constant tool-frame offset" from a coincidental FK error; a second orientation does.

### 3. Reboot survival — PASSED (issue #3 closed)

`sudo reboot` at 17:16; ~90 s later everything self-recovered with nobody touching anything:

- WiFi hotspot `urjetson` back up, laptop re-associated automatically → SSH path restored
- `enP8p1s0` static 192.168.56.1/24 restored (NM profile "Wired connection 1", `manual` + autoconnect)
- USB-Ethernet internet adapter (`enxa0cec8b70c31`, 10.102.52.55) back
- Robot ping 0.4 ms; driver relaunched cleanly with calibration OK
- MAXN_SUPER (id 2) persisted across the reboot

### 4. First commanded motion — DONE (coordinated: Nikola at the pendant, commands from the laptop)

Protocol: speed slider to 10%, External Control program → Play (arm correctly did **not** move on Play — it just opens the command channel), hand on e-stop. Driver logged `Robot connected to reverse interface. Ready to receive control commands.`

Motion: `FollowJointTrajectory` goal to `/scaled_joint_trajectory_controller/follow_joint_trajectory` — wrist_3 +0.05 rad over 3 s, back over 3 s (time-stretched ~10× by the 10% slider, as scaled JTC is designed to do). Result: **error_code 0 (SUCCESSFUL)**, final position within 0.0005 rad of start. This validates the full chain (laptop → Jetson → driver → reverse interface → robot) that MoveIt will use. Script kept at `~/first_motion.py` on the Jetson.

Gotchas learned (matter for task 1.6's `safety_monitor`):
- `/io_and_status_controller/robot_program_running` is **latched (TRANSIENT_LOCAL), publishes only on change** — a default-QoS subscriber never sees the stored value. Subscribe with `durability=TRANSIENT_LOCAL`.
- `/speed_scaling_state_broadcaster/speed_scaling` read `10.0` with the slider at 10% (and `0.0` while no program runs) — treat it as percent-scaled, verify before using it as a 0–1 factor.

Still open from task 0.6: interactive keyboard-teleop jog, and the protective-stop recovery exercise.

### 4a. Motion envelope finding — robot refuses trajectories above ~0.018 rad/s joint speed (MUST FIX before real trajectories)

After the first success we attempted a 4-joint "nod" sequence (±10–20° per joint). The robot **never moved**: pendant showed a velocity-limit complaint (exact code TBD), the driver-side desired position crept ahead of the motionless joint until the scaled JTC's 0.2 rad path tolerance tripped → `error_code -4, Aborted due to state tolerance violation`. Reproduced twice with the identical signature (wrist_3, error 0.200 rad). Meanwhile a 1.1° micro-move and the original 2.9° move — commanded ~10–25× *slower* — both succeeded with sub-millidegree tracking.

| Motion | wrist_3 actual speed | Result |
|---|---|---|
| +2.9° @10% slider | ~0.002 rad/s | success |
| +1.1° micro-test | ~0.0007 rad/s | success |
| +20° nod @20% slider | ~0.018 rad/s | robot never moved, -4 abort |
| +20° repeat | ~0.018 rad/s | identical abort |

0.018 rad/s is glacial — no sane configured limit sits there. Working hypothesis (was already on the session-1 watch list): **no RT scheduling** (`Could not enable FIFO RT scheduling policy`) lets the 500 Hz reverse-interface stream stall for tens of ms; on resume the setpoint arrives as a jump whose *implied instantaneous velocity* trips URControl's guard. Faster trajectories → proportionally bigger jump per stall → threshold behavior exactly as observed. Dashboard state was PLAYING/RUNNING/NORMAL throughout — the channel itself never dropped.

Next steps (session 3, before any teleop):
1. Read the exact pendant log entry (code + text) for the velocity complaint — distinguishes URControl runtime guard vs safety-limit event (check Safety → Joint Limits too, in case teleop-era limits are set absurdly low).
2. Apply the catalogued RT fix on the Jetson (ARCHITECTURE.md risks): `rtprio` limits for the driver user (`/etc/security/limits.d/`, e.g. `jetson - rtprio 99`) so the FIFO warning disappears; retest the same +20° nod. If it then tracks, hypothesis confirmed and closed.
3. If not RT: bisect speed (2×, 4×, 8× the known-good 0.002 rad/s) to measure the actual threshold, and test with explicit waypoint velocities (cubic interpolation) to rule out linear-interpolation velocity steps at waypoints.

Until resolved, keep commanded joint speeds ≤0.002 rad/s (known good) for any verification moves.

### SSH path (task 0.3 documentation)

- Laptop joins the Jetson's own WiFi hotspot, SSID **`urjetson`** → `ssh jetson@10.42.0.1`. This works with no lab infrastructure at all.
- **Gotcha:** while on the hotspot, the laptop's *internet* also routes through the Jetson (default route via 10.42.0.1 → Jetson's USB-Ethernet uplink). Rebooting the Jetson therefore cuts the laptop's internet until the hotspot returns and the laptop re-associates.
- After the reboot the Jetson's SSH host key verification tripped on a stale `known_hosts` entry (old ECDSA entry vs freshly negotiated RSA). Fix: `ssh-keygen -R 10.42.0.1`, reconnect, and sanity-check you're on the Jetson (hostname `ubuntu`, `~/ur7e_calibration.yaml` present).
- Alternative from the robot-side private network: `jetson@ubuntu.local` via mDNS (session 1 note).
- ROS env on the Jetson (must match to see topics): `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (set in `~/.bashrc`; export explicitly in non-interactive SSH commands, since `.bashrc` returns early for non-interactive shells).

### Session 2 watch list

- `C210A0` **recurs at every driver connect** (WARN via `robot_state_helper`, robot proceeds to RUNNING). Reproducible, so far benign; still worth identifying the code's meaning at the pendant log next visit.
- `Could not enable FIFO RT scheduling policy` still present (expected — unchanged stock kernel). Becomes real work only if reverse-interface drops appear under motion load.
- Driver runs ad-hoc via `nohup ... > ~/ses2_driver.log` — fine for lab sessions; task 0.5's bringup owns making this a single command (and eventually a service).

## Workstation session — non-lab infrastructure (2026-09-15, no robot)

Tasks 0.5 (software half), 0.7, and 0.8 landed from the laptop, verified in a `ros:humble` Docker container (the CI image):

- **`src/ur7e_bringup`** — single launch entrypoint wrapping the stock driver launch; pins `ur_type:=ur7e`, bakes in the calibration YAML (now at `src/ur7e_bringup/config/`), exposes `use_mock_hardware` / `headless_mode` / `robot_ip` / `launch_rviz`. Headless-vs-URCap evaluation written up in the package README — recommendation: keep the URCap + Play ritual; **lab session 3 confirms** and closes #5.
- **Gotcha for the books:** the Humble apt driver still names the mock flag `use_fake_hardware` (the `use_mock_hardware` rename is post-Humble). Our launch exposes the modern name and maps internally.
- **Sim tier 1 + CI (#7):** `scripts/tier1_smoke_test.sh` (controller active → `/joint_states` → FollowJointTrajectory succeeds — the sim twin of session 2's first motion) wired into GitHub Actions (`.github/workflows/ci.yml`: rosdep → colcon build → lint → smoke).
- **Sim tier 2 (#7):** `sim/ursim/docker-compose.yml`, pinned `ursim_e-series:5.23` (= our PolyScope; tag verified on Docker Hub). Container gets the real robot's IP 192.168.56.101, host is 192.168.56.1 — sim and lab commands identical. First-run PolyScope steps in `docs/SIMULATION.md`. Trajectory-through-URSim acceptance still to be run on an x86 machine with the driver installed (devcontainer or CI machine — this laptop has no ROS).
- **Devcontainer (#29):** `.devcontainer/` on `osrf/ros:humble-desktop-full`, Jetson-matching middleware env (`ROS_DOMAIN_ID=42`, CycloneDDS). "RViz shows the arm in <30 min from clone" acceptance needs a first member run.

## URSim rehearsal session (2026-09-17, workstation, pre-lab-3)

Executed lab session 3's item 0: full bringup + protective-stop drill against
URSim tier 2 (driver in the devcontainer — also its first real member run,
counts toward #29's acceptance). **Tier-2 acceptance from #7 is met:** a
`FollowJointTrajectory` goal ran through the real driver against URSim
(base +30°, elbow +15°, out-and-back, SUCCEEDED, returned within 0.02°).

Setup potholes fixed in the docs as we hit them (see `docs/SIMULATION.md`
and `sim/ursim/docker-compose.yml` diffs): the 5.23 image does **not**
auto-install mounted URCaps (manual Settings → System → URCaps → + install,
once); PolyScope's URCap "Restart" exits the container (`docker compose up -d`
again); the URCap file picker is sandboxed to the programs dir (compose now
mounts `./urcaps` inside it). URSim safety password was unset — set to
`easybot1` (sim only, nothing sensitive).

### Four findings that matter for the lab and for tasks 1.2/1.6

1. **The External Control URCap has its own URScript-side velocity guard,
   below the safety board.** A trajectory demanding more than the joint
   velocity limit in a 2 ms setpoint step is not executed and does not fault
   the robot: PolyScope pops *"External Control speed limit … Ignoring
   commands until a valid command is received"*, the driver aborts the goal,
   the program keeps running. Recovery is the popup's **Continue** button —
   no dashboard unlock involved. **This is very likely the mechanism behind
   session 2's "refuses trajectories above ~0.018 rad/s at 10% slider"
   finding** — same veto signature, and now reproducible in sim (issue #6).

2. **URSim with factory-preset safety limits did not enforce speed limits at
   all.** A 100° base swing in 0.4 s (~375°/s peak, TCP ~2.7 m/s) executed
   cleanly despite nominal 191°/s joint and 1.5 m/s TCP limits. Only after
   setting a *custom* tool-speed limit (160 mm/s) did the guard/monitoring
   engage. **Never let sim absolve a trajectory's safety** — matches the
   standing "sim never signs off" rule, now with evidence.

3. **A protective stop can be provoked reliably via a restricted joint
   position limit** (Safety → Joint Limits, base min −30°, then a slow legal
   trajectory across the line). Velocity-based provocations get vetoed by
   finding 1 before the safety board ever sees them. The stop fired ~2°
   before the boundary (stopping distance), program halted, goal ABORTED.

4. **After a protective stop aborts a goal mid-flight, re-pressing Play is
   not enough — restart the driver.** The trajectory controller holds its
   stale pre-stop command; on reconnect it demands the arm jump to it, the
   URCap guard vetoes every 2 ms cycle (popup counter climbing thousands),
   and the connection is wedged. Full recovery ritual, verified twice:
   `unlock_protective_stop` (≥5 s after the stop) → **Ctrl+C and relaunch
   the bringup** → Play in PolyScope → verify with a small legal goal.
   Task 1.6's `safety_monitor` must encode this. Also learned: a goal
   **rejected** (vs aborted) means the controller refused it at submission —
   with our stack that's "External Control isn't running", i.e. nobody
   pressed Play.

Pendant safety checksum (top-right, 4+4 hex, e.g. `52AA F631`) changes with
every safety-config apply — quick visual check for "which safety config is
loaded", useful at the real pendant too.

## Lab session 3 — plan (issues #4 close-out, #5, #6)

0. *Before the lab:* rehearse the protective-stop recovery flow in URSim (`docs/SIMULATION.md`, tier 2) — the lab visit then only confirms real-robot behavior instead of discovering the procedure.
1. At the pendant: read Installation → TCP (expect z ≈ 30.5 mm, the Quick Changer) + payload; note exact values into this file.
2. TCP comparison at 2 more poses (freedrive between them): `ros2 run tf2_ros tf2_echo base tool0` vs `ros2 topic echo /tcp_pose_broadcaster/pose --once` — < 1 mm after subtracting the pendant TCP closes #4.
3. Keyboard-teleop jog through our bringup (task 0.6 remainder — first *trajectory* motion already done in session 2); exercise protective-stop recovery and document it.
4. Task 0.5 close-out: `ros2 launch ur7e_bringup ur7e_bringup.launch.py` against the real robot — one command to "ready" (the package landed in the workstation session; this is its hardware acceptance).
