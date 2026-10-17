# The chess game on the Jetson

The game as demoed on 2026-10-14, moved from the laptop's dev container to the
makerspace Jetson (Yahboom Orin Nano, JetPack 6.2 = Ubuntu 22.04, ROS Humble
native). **The Jetson is shared with other projects:** everything of ours lives
in `~/chess_arm`, system changes are checked first and listed at the end so
they can be undone.

| Machine | Runs |
|---|---|
| Jetson | driver + MoveIt (T1), camera + dashboard `watch` (T2), `play` (T3) |
| Laptop | the two browser windows (operator, TV), `chess_voice.py` (Ollama on its GPU, TV sound) |

The laptop keeps what it did on the host side before; the Jetson replaces the
dev container. Nothing on the Jetson needs a monitor.

## 1. Internet for the Jetson through the laptop (laptop)

The Jetson's single port `eno1` cabled to the laptop. The laptop shares its
WiFi; the Jetson's existing DHCP profile takes an address without changes.

```bash
ip -br link                     # the laptop's Ethernet name (enp7s0 on 09-28)
sudo nmcli con add type ethernet ifname enp7s0 con-name jetson-share ipv4.method shared ipv6.method ignore autoconnect no
sudo nmcli con up jetson-share
ip neigh show dev enp7s0        # after 20 s: the Jetson's 10.42.0.x
ssh jetson@10.42.0.x
```

Undo: `sudo nmcli con down jetson-share`. WiFi is not affected.

## 2. Look before touching (Jetson, read-only)

```bash
whoami; ls /home; who; df -h / | tail -1; free -h | head -2
nmcli -t -f NAME,DEVICE,AUTOCONNECT con show
grep -h ROS_DOMAIN_ID /home/*/.bashrc 2>/dev/null
ls -d ~/ur7e_ws ~/chess_arm 2>/dev/null
which stockfish vcs tmux colcon; ss -ltn | grep -E ':8091\b'
```

Needed: 6 GB free disk, no other project on ROS domain 42 (else change it in
`scripts/lab_env.sh` for our side only), port 8091 free. `~/ur7e_ws` is the
September clone: leave it as it is.

## 3. Our checkout (Jetson)

```bash
git clone -b claude/chess-arm-single-rg2 https://github.com/PawPrintStudio/ur7e-language-pick-and-place.git ~/chess_arm
cd ~/chess_arm
```

A new folder, so nobody's files change.

## 4. System packages: simulate, then ask (Jetson)

`apt install` can upgrade libraries another project uses (a newer MoveIt
pulls newer `ros-humble-*` core packages). Simulate first, nothing is
installed:

```bash
sudo apt-get update
sudo apt-get install -s --no-upgrade ros-humble-ur ros-humble-moveit ros-humble-rmw-cyclonedds-cpp \
  ros-humble-cv-bridge ros-humble-message-filters ros-humble-rosbag2-transport ros-humble-ros2controlcli \
  python3-scipy python3-yaml libnet1-dev python3-colcon-common-extensions python3-vcstool stockfish tmux \
  | grep -E '^(Inst|Remv)'
```

`Inst pkg (new)` is a new package: harmless to others. `Inst pkg [old] (new)`
**upgrades** something already there and `Remv` removes it: agree those with
the other devs first. Then the same line without `-s`, and record it below.

## 5. Python packages, inside the checkout (Jetson)

```bash
python3 -m pip install --target .pydeps "chess>=1.10"
```

Not the system Python and not `~/.local`, which every program of the `jetson`
user would see. `scripts/chess_env.sh` adds `.pydeps` to `PYTHONPATH`. numpy,
OpenCV (Yahboom's CUDA build) and PyYAML are the system ones: do not
pip-install them.

## 6. Build (Jetson)

```bash
source /opt/ros/humble/setup.bash
bash scripts/vendor_import.sh                 # UR + RG2 + pymoveit2 into src/vendor
colcon build --symlink-install --packages-skip ur7e_gazebo --parallel-workers 2
```

`ur7e_gazebo` needs Gazebo, which a robot computer does not. Two workers keep
the box usable for others while it builds (about 15 min).

## 7. The demo's calibration (laptop → Jetson)

Not in git (they describe one placement of board and camera, `.gitignore`):

```bash
cd ~/Projects/Makerspace/AI_arm
scp scripts/lab_chess.json scripts/.lab_chess_touches.json scripts/.console_poses.json \
    scripts/lab_table.json scripts/.lab_camera.json jetson@10.42.0.x:chess_arm/scripts/
```

`chess_home` is in `.console_poses.json`. If anything moved since 10-14,
re-touch anyway (CHESS.md, "Calibration order when something moved").

## 8. Check without the arm (Jetson)

```bash
source ~/chess_arm/scripts/chess_env.sh
python3 -c "import chess, cv2, rclpy; print('ok', cv2.__version__)"
python3 scripts/lab_chess.py squares          # board geometry from lab_chess.json
```

## 9. At the arm

**Network.** The pendant's External Control host is `192.168.1.2` and the
robot is `192.168.1.10`, so the Jetson must hold `192.168.1.2`. With one port
that is `eno1`; a small switch lets the laptop stay reachable: robot, Jetson
and laptop on the switch, laptop at `192.168.1.3`.

```bash
# Jetson, once (adds a profile, changes none; ask first, see the list below)
sudo nmcli con add type ethernet ifname eno1 con-name ur-link-chess ipv4.method manual ipv4.addresses 192.168.1.2/24 autoconnect no
# laptop, once
sudo nmcli con add type ethernet ifname enp7s0 con-name chess-lan ipv4.method manual ipv4.addresses 192.168.1.3/24 autoconnect no
```

Each session, in this order (the SSH session drops at step 1, expected):

1. Jetson, still over the 10.42 cable: `sudo nmcli con up ur-link-chess`
2. Cables: Jetson `eno1`, robot and laptop into the switch
3. Laptop: `sudo nmcli con up chess-lan`, then `ssh jetson@192.168.1.2`

After a Jetson reboot `ur-link-chess` is down again (`autoconnect no`, so the
others' wired profile keeps the port by default): repeat from step 1 with the
laptop cable, or a monitor and keyboard.

`chess_env.sh` calls this address `LAPTOP_IP`: it means "the computer the
pendant connects back to", now the Jetson. The ZED goes in a Jetson USB-3
port (`lsusb`: STEREOLABS).

**Terminals.** On the Jetson in `tmux` (an SSH drop must not kill the driver):
`tmux new -s chess`, three windows, `source ~/chess_arm/scripts/chess_env.sh`
in each, then the walkthrough's T1, T2, T3 lines unchanged (sections 4 to 7).

**Screens, from the laptop.** The operator page only answers 127.0.0.1; a
tunnel makes the laptop's browser count as the Jetson itself:

```bash
ssh -N -L 8091:127.0.0.1:8091 jetson@192.168.1.2
```

Then http://localhost:8091/#operator and the TV view at http://localhost:8091/
exactly as before.

**Voice, on the laptop.** It reads the game's events and writes the line the
page shows, both in the Jetson's `log/chess`; mount it (`sudo apt install
sshfs` on the laptop once):

```bash
mkdir -p ~/jetson-chess && sshfs -o cache=no jetson@192.168.1.2:chess_arm/log/chess ~/jetson-chess
.venv-voice/bin/python scripts/chess_voice.py --events ~/jetson-chess/events.jsonl
```

Undo: `fusermount -u ~/jetson-chess`.

## What we changed on the Jetson

Fill in as it happens; each line with its undo.

| Change | Undo |
|---|---|
| `~/chess_arm` (checkout, build, `.pydeps`) | `rm -rf ~/chess_arm` |
| apt packages: _list from step 4_ | `sudo apt-get remove <list>` |
| NM profile `ur-link-chess` | `sudo nmcli con delete ur-link-chess` |
