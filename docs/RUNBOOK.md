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
| Ethernet iface + Jetson IP / robot IP | |
| `ur_type:=ur7e` accepted? | **Yes** — Humble binary driver loaded hardware `ur7e`; dashboard reports robot version 5.23.0.0; RTDE v2 @ 500 Hz |
| Controllers active | `scaled_joint_trajectory_controller`, `joint_state_broadcaster`, `io_and_status_controller`, `speed_scaling_state_broadcaster`, `force_torque_sensor_broadcaster`, `tcp_pose_broadcaster`, `ur_configuration_controller`, `friction_model_controller` (others loaded inactive) |
| Calibration YAML extracted? | **Yes** — committed as `config/ur7e_calibration.yaml` (hash calib_12445833238222042106). Wire into bringup via `kinematics_params_file` (task 0.5); TCP spot-check vs pendant pending (issue #4 acceptance). |
| Anything that errored (paste text) | apt offline on robot network (expected — DNS unavailable; install over WiFi) |
| Notable | `/opt/ros` has **humble and rolling** — ensure shells source humble. **No `ros-humble-ur*` was installed** and `~/ur_ws/src` holds only `ur_dev_bringup` → the teleop-era driver never ran from this Jetson via apt; fresh install required. ~937 GB disk, 3.7 GB swap present. |

**Networking notes (learned the hard way):** from the laptop, address the Jetson as `jetson@ubuntu.local` — the `.local` suffix uses mDNS (the Jetson answers for itself via avahi), which works on the private network where plain DNS has no entry for it. The Jetson keeps two links at once: built-in Ethernet -> robot (192.168.56.1), USB-Ethernet adapter -> internet for package installs; keep that adapter with the robot kit.

**Session 1 watch list:**
- **Checksum question (verify in session 2):** driver (launched WITHOUT our kinematics file) printed `calib_12788084448423163542` and warned of calibration mismatch; our extracted YAML carries `calib_12445833238222042106`. Expected explanation: the printed value is the default kinematics file's hash, ours is the robot's true one. Definitive test: relaunch with `kinematics_params_file:=.../config/ur7e_calibration.yaml` — the mismatch ERROR must disappear. If it persists: re-extract and investigate before trusting any TCP pose.
- `Could not enable FIFO RT scheduling policy` — stock kernel denies RT priority to the control thread. Harmless until "reverse interface dropped" appears under load; fixes (rtprio limits / lowlatency kernel / core isolation) are catalogued in ARCHITECTURE.md risks.
- Robot logged error code `C210A0` at driver startup, then went NORMAL/RUNNING. Watch for recurrence.
- External Control **Play was not exercised** in session 1 — driver bringup verified, motion path not yet. First item of session 2.

## Lab session 2 — plan (issues #3, #4, #5, #6)

1. Bringup launch (task 0.5) passing `kinematics_params_file` → confirm the calibration-mismatch error is GONE (closes the checksum question, most of #4).
2. TCP spot-check: `tcp_pose_broadcaster` output vs pendant Move-screen TCP readout at 2–3 arm poses; < 1 mm closes #4.
3. Reboot the Jetson once → confirm eth static config and robot link come back (closes #3).
4. External Control Play + keyboard-teleop jog through our bringup (task 0.6) — first commanded motion; hand on the e-stop, speed slider low.
