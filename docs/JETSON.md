# The chess game on the Jetson

The game as demoed on 2026-10-14, moved from the laptop's dev container to the
makerspace Jetson (hostname `ubuntu`, 16 GB, ROS Humble native; set up
2026-10-17). **The Jetson is shared:** one `jetson` account for every project,
so everything of ours lives in `~/chess_arm`, `~/.bashrc` stays untouched, and
system changes are simulated first and listed at the end with their undo.

| Machine | Runs |
|---|---|
| Jetson | driver + MoveIt (T1), camera + dashboard `watch` (T2), `play` (T3), voice (T4), the audience monitor on its DisplayPort |
| Laptop (optional) | the operator page over WiFi |

**Network.** The laptop joins the Jetson's WiFi hotspot `JetsonArm` and
reaches it at `ssh jetson@10.42.0.1` (the laptop's internet then runs through
the Jetson). That leaves the Jetson's one Ethernet port `eno1` for the wall
drop during setup and for the robot during a game. First connection after
the September unit: SSH warns that the host key changed (same address,
different machine); compare the fingerprint on the Jetson's screen
(`ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`), then
`ssh-keygen -R 10.42.0.1` on the laptop.

## Setup (once, `eno1` on the wall drop for internet)

### 1. Look before touching (read-only)

```bash
ls /home; who; df -h / | tail -1; free -h | head -2
grep -h ROS_DOMAIN_ID /home/*/.bashrc; ls -d ~/chess_arm
which vcs colcon stockfish tmux; ss -ltn | grep 8091
cat /proc/device-tree/model; echo; head -1 /etc/nv_tegra_release; ls /opt/ros
python3 -c "import cv2, numpy; print('cv2', cv2.__version__, 'numpy', numpy.__version__)"
```

Needed: `humble` in `/opt/ros`, 6 GB free, no one else on ROS domain 42
(else change it in `scripts/lab_env.sh`), port 8091 free. Note the cv2 and
numpy versions: nothing below may replace them.

Found 2026-10-17: Orin NX 16 GB, JetPack 6.2.1 (R36.4.7). Another project's
service `openmanipulator-xbox` starts at boot on ROS domain 0 (ours is 42),
and an `ollama` service holds 127.0.0.1:11434. numpy 1.26.4 and
opencv-python 4.11 are in `~/.local` (someone's pip installs, shared by every
program of the account).

### 2. Our checkout

```bash
git clone https://github.com/PawPrintStudio/ur7e-language-pick-and-place.git ~/chess_arm
cd ~/chess_arm
```

### 3. System packages: simulate, then install

`apt install` can upgrade packages another project uses (a newer MoveIt
pulls newer `ros-humble-*` core packages). Simulate first; nothing is
installed:

```bash
sudo apt-get update
sudo apt-get install -s --no-upgrade ros-humble-ur ros-humble-moveit ros-humble-rmw-cyclonedds-cpp \
  ros-humble-cv-bridge ros-humble-message-filters ros-humble-rosbag2-transport ros-humble-ros2controlcli \
  python3-scipy python3-yaml python3-pip libnet1-dev python3-colcon-common-extensions python3-vcstool \
  stockfish tmux | grep -E '^(Inst|Remv)'
```

`Inst pkg (new)` adds a package: harmless to others. `Inst pkg [old] (new)`
**upgrades** one already there and `Remv` removes one: agree those with the
other devs first. Then the same command without `-s` and the `grep`, and
write the list in the table at the end.

### 4. Python packages, inside the checkout

```bash
python3 -m pip install --target .pydeps "chess>=1.10"
```

Not the system Python and not `~/.local`, which every program of the shared
account would see. `scripts/chess_env.sh` adds `.pydeps` to `PYTHONPATH`.
numpy, OpenCV and PyYAML stay the system ones: never pip-install them here.

### 5. Build

```bash
source /opt/ros/humble/setup.bash
bash scripts/vendor_import.sh
colcon build --symlink-install --packages-skip ur7e_gazebo --parallel-workers 2
```

`vendor_import.sh` fetches the UR + RG2 drivers and pymoveit2 into
`src/vendor`. `ur7e_gazebo` needs Gazebo, which the robot computer does not;
two workers leave the box usable for others while it builds.

### 6. The demo's calibration

The live files are not in git (one placement of board and camera); the
2026-10-14 set is archived and copied into place on purpose (`-n`: never over
an existing one):

```bash
cp -n docs/evidence/chess-2026-10-14/{lab_chess.json,.lab_chess_touches.json,.console_poses.json,lab_table.json,.lab_camera.json} scripts/
```

### 7. Check without the arm

```bash
source ~/chess_arm/scripts/chess_env.sh
python3 -c "import chess, cv2, rclpy; print('ok')"; which stockfish
python3 scripts/lab_chess.py squares
```

`chess_env.sh` prints `Chess terminal ready`; `which stockfish` must find
`/usr/games/stockfish` (without it `play` silently picks random moves).

Verified 2026-10-17: apt history shows no upgrades or removals; 15 packages
built; Stockfish answers; `squares` prints the 10-14 board; 76 chess tests
pass (`cd scripts && PYTHONPATH=$PYTHONPATH:../src/ur7e_perception python3 -m
pytest -p no:launch_testing -p no:launch_ros test_lab_chess.py
test_chess_operator.py test_chess_vision.py test_chess_gestures.py`);
`CHESS=true bash scripts/lab_up.sh` takes the native path and stops at the
robot check.

### 8. Camera and dashboard, still without the arm

ZED into a Jetson USB-3 port (`lsusb`: STEREOLABS), then on the Jetson
`python3 scripts/lab_chess.py watch`, and on the laptop the tunnel from
"Screens" below: http://localhost:8091/ shows the live view.

## Each game

The step-by-step version to keep open at the arm: [JETSON_GAME_WORKBOOK.md](JETSON_GAME_WORKBOOK.md).

### Network at the arm

The pendant's External Control host is `192.168.1.2` and the robot is
`192.168.1.10`, so the Jetson holds `192.168.1.2` on `eno1`, cabled to the
robot. One profile, added once (it changes no other profile; on the list at
the end):

```bash
sudo nmcli con add type ethernet ifname eno1 con-name ur-link-chess ipv4.method manual ipv4.addresses 192.168.1.2/24 autoconnect no
```

Each session: robot cable into `eno1`, laptop on `JetsonArm`, `ssh
jetson@10.42.0.1`, then `sudo nmcli con up ur-link-chess`. `autoconnect no`
keeps the wired profile the others use as the default after a reboot.
`chess_env.sh` calls this address `LAPTOP_IP`: "the computer the pendant
connects back to", now the Jetson.

### Terminals (Jetson)

In `tmux`, so an SSH drop does not stop the driver: `tmux new -s chess`, three
windows (Ctrl+B C; Ctrl+B 0/1/2 to switch; Ctrl+B D to leave it running;
`tmux attach -t chess` to return), and in each:

```bash
source ~/chess_arm/scripts/chess_env.sh
```

Then the walkthrough's lines unchanged: T1 `CHESS=true LAB_SPEED=100 bash
scripts/lab_up.sh` (pendant Play when it asks), T2 `python3
scripts/lab_chess.py watch`, T3 `track`, `scan`, `play ...`
(CHESS_OPERATOR_WALKTHROUGH.md sections 4 to 7).

### Screens (laptop)

The operator page answers only the computer it runs on; a tunnel makes the
laptop's browser count as the Jetson:

```bash
ssh -N -L 8091:127.0.0.1:8091 jetson@10.42.0.1
```

Then http://localhost:8091/#operator and the TV view http://localhost:8091/,
exactly as before.

### Monitor and voice (Jetson)

A regular monitor on the Jetson's DisplayPort. Sound goes to the monitor's
speakers over DP if it has them (`--sink hdmi`, the default), else to a USB
speaker (`--sink usb`; `pactl list short sinks` names it). A monitor has no
TV audio delay: `--lead 0`. On the Jetson's desktop, Chromium full screen (F11) at
http://localhost:8091/. The voice in a fourth tmux window:

```bash
python3 scripts/chess_voice.py --model llama3.2:3b --lead 0
```

The first line must say `AUDIO ... paplay --device=...` with the sink you meant. The
model is the one the Jetson's `ollama` service already has (another
project's; nothing downloaded); a slow answer falls back to a canned line as
on the laptop. Piper is the copy in `~/.local`. The voice and jingle files
are not in git: copied once from the laptop's `log/chess/voices` and
`log/chess/sfx` (2026-10-17). Verified that day without a screen: 11 voice
tests, a line synthesized and played on the HDMI sink, model 1.9 s warm.

## What we changed on the Jetson

Fill in as it happens; each line with its undo.

| Change | Undo |
|---|---|
| `~/chess_arm` (checkout, build, `.pydeps`) | `rm -rf ~/chess_arm` |
| apt (2026-10-17, simulated: 0 upgraded, 0 removed): `ros-humble-rmw-cyclonedds-cpp libnet1-dev stockfish tmux` (+7 dependencies) | `sudo apt-get remove` the same four, then `sudo apt-get autoremove` |
| laptop key in `~/.ssh/authorized_keys` (line ending `nikola@nikola-LOQ`) | delete that line |
| NM profile `ur-link-chess` | `sudo nmcli con delete ur-link-chess` |
