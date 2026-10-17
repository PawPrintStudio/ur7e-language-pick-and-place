# Workbook: a chess game on the arm, run from the Jetson

Offline from start to finish: nothing below needs the internet. The Jetson
runs everything natively (no container): the driver and MoveIt, the camera
and dashboard, the game, the voice, and the audience page on its monitor.
The laptop is only the operator screen, over the Jetson's WiFi. The day's
detail and the "why" behind each check: CHESS_OPERATOR_WALKTHROUGH.md;
setup history: JETSON.md.

| Thing | Address / name |
|---|---|
| Robot (pendant) | `192.168.1.10`, External Control host `192.168.1.2` |
| Jetson on the robot cable | `192.168.1.2` (profile `ur-link-chess` on `eno1`) |
| Jetson WiFi hotspot | `JetsonArm`, Jetson = `10.42.0.1` |
| Login | `ssh jetson@10.42.0.1` (laptop key, no password) |
| Our folder | `~/chess_arm` (ROS domain 42; another project uses domain 0) |
| Dashboard | Jetson `http://localhost:8091/`, operator `/#operator` |

## A. Once, before the first game (Jetson still online)

- [ ] `cd ~/chess_arm && git pull` (this workbook and the voice fix arrive)
- [ ] The robot-link profile, typed by you (sudo); it adds a profile and
      changes none, and `autoconnect no` leaves the others' wired profile the
      default after a reboot:
      ```bash
      sudo nmcli con add type ethernet ifname eno1 con-name ur-link-chess ipv4.method manual ipv4.addresses 192.168.1.2/24 autoconnect no
      ```
- [ ] Sound: monitor plugged into the Jetson's DisplayPort, then
      `cd ~/chess_arm && python3 scripts/chess_voice.py --say greeting --model llama3.2:3b --lead 0`.
      Heard it: the sink is `hdmi` (default). Silence: plug a USB speaker,
      `pactl list short sinks` shows its name, use `--sink usb` from now on.

## B. Cables and power

- [ ] Robot Ethernet cable into the Jetson's `eno1` (out of the wall drop)
- [ ] ZED into a Jetson **USB-3** port (blue); mount, board and box where
      they were calibrated
- [ ] Monitor (audience) on the Jetson's DisplayPort; speaker if needed
- [ ] Jetson powered; laptop joins WiFi `JetsonArm`
- [ ] Robot on, brakes released, pendant in Remote/Local as for the demo,
      the **External Control** program loaded, host `192.168.1.2`, tool and
      payload as on 10-14

## C. Network and terminals (laptop → Jetson)

```bash
ssh jetson@10.42.0.1
```
```bash
sudo nmcli con up ur-link-chess
```
```bash
ping -c2 192.168.1.10
```

Replies = the Jetson sees the robot. Then the terminals, inside tmux so a
WiFi drop does not stop the arm's software:

```bash
tmux new -s chess
```

Make four windows: Ctrl+B then C, three times. Ctrl+B then 0 to 3 switches,
Ctrl+B then D leaves them all running, `tmux attach -t chess` returns. In
**each** window:

```bash
source ~/chess_arm/scripts/chess_env.sh
```

Expect `Chess terminal ready ... Robot: 192.168.1.10 | Laptop: 192.168.1.2 | ROS domain: 42`
(`Laptop` is the old name for "the computer the pendant calls back": the Jetson).

## D. Window 0: driver and MoveIt

```bash
CHESS=true LAB_SPEED=100 bash scripts/lab_up.sh
```

- [ ] `ready: scaled_joint_trajectory_controller configured`
- [ ] At `>>> Pendant: ... press PLAY`: everyone clear, slider at **25 %**
      for the first game on the Jetson, then press Play
- [ ] `ready: pendant Play` / `controller active` / `MoveIt` / `== chess stack up.`

Leave it running. `no reply from 192.168.1.10`: step C (cable, profile up).
Timeout at Play: wait for its cleanup, rerun.

## E. Window 1: camera and dashboard

```bash
python3 scripts/lab_chess.py watch
```

- [ ] Prints `WATCH` and `http://localhost:8091`
- [ ] On the Jetson's monitor: Chromium at `http://localhost:8091/`, F11 (broken snaps: JETSON.md "Open")
      for full screen. Live video, grid on the board, a1 in the right corner

`no ZED`: replug into a blue USB-3 port, `ls /dev/video*`.

## F. Laptop: the operator page

In a laptop terminal (keep it open; it shows nothing):

```bash
ssh -N -L 8091:127.0.0.1:8091 jetson@10.42.0.1
```

Browser on the laptop: `http://localhost:8091/#operator`. The page only
takes clicks from the Jetson itself; the tunnel makes the laptop count as it.

## G. Window 2: calibration check (every session)

Pieces at the start position, arm clear of the camera's view.

- [ ] If the board, stand, camera or tool moved since 10-14, **re-touch**:
      `touch --corner 1` (a1), `2` (h1), `3` (h8), `4` (a8), `touch --corner box`,
      then `fit --square` (all as `python3 scripts/lab_chess.py ...`).
      Freedrive stops the External Control program: **Play on the pendant
      again after each touch**.
- [ ] Fingertip test, arm moves slowly to e4 and touches it:
      ```bash
      python3 scripts/lab_chess.py goto e4 --touch --execute
      ```
      Within 3 mm of the centre. A constant miss everywhere: re-touch (or,
      as a patch, `shift --dx --dy` in mm).
- [ ] Camera agrees with the board:
      ```bash
      python3 scripts/lab_chess.py overlay --fen start
      ```
      ```bash
      python3 scripts/lab_chess.py track --fen 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'
      ```
      ```bash
      python3 scripts/lab_chess.py scan
      ```
      `track` under 1.5 mm and 0.3°; `scan` reads the start position.

## H. Window 3: the voice

```bash
python3 scripts/chess_voice.py --model llama3.2:3b --lead 0
```

- [ ] First line `AUDIO ... paplay --device=...` (add `--sink usb` if the
      sound is on a USB speaker). It speaks once per game event; the game
      never waits for it. Ctrl+C stops it, the game goes on.

## I. Window 2: the game

```bash
python3 scripts/lab_chess.py play --vision --execute --max-speed-percent 100 --skill 20 --hover 0.10 --careful
```

- [ ] First line `ENGINE stockfish` (`random mover` = Stockfish not found:
      re-source `chess_env.sh`)
- [ ] `--careful` adds a Go per step: keep it for the first game on the
      Jetson, drop it once a full move ran cleanly. `--skill 1` for a
      beatable robot.

Each turn, on the laptop's operator page (or typed in window 2):

1. The student (White) makes one move and lets go of the piece.
2. `I saw ...` appears: check the real board, **Yes**, **No**, or type the
   move (`e2e4`).
3. Read the robot's reply; **Clear: let the arm move** only when everyone is
   clear (with `--careful`, a **Go** per step).
4. When the arm is home, check the real board before the next move.

After the first clean move at 25 %: slider up on the pendant (100 % is the
demo setting). **New game** resets between students (pieces back by hand,
board untouched); **Quit** stops `play`.

## J. Shut down

1. **Quit** on the operator page (or `q` in window 2)
2. Ctrl+C in window 3 (voice), window 1 (camera), then window 0 (driver);
   wait for `stopping ... driver`
3. Pendant: stop the program
4. `exit` each window (or `tmux kill-session -t chess`)
5. Jetson back to normal: `sudo nmcli con down ur-link-chess`, robot cable
   out, wall drop back into `eno1`

Ctrl+C is not an emergency stop: the pendant's E-stop is.

## When something goes wrong

| Symptom | Fix |
|---|---|
| `ssh: connect ... 10.42.0.1` fails | Laptop not on `JetsonArm`; or the Jetson rebooted, wait a minute |
| `no reply from 192.168.1.10` | Cable in `eno1`? `sudo nmcli con up ur-link-chess`; pendant IP `192.168.1.10` |
| Timeout waiting for Play | External Control host must be `192.168.1.2`; rerun window 0 after cleanup |
| Arm stops after a touch | Freedrive ended External Control: Play on the pendant |
| Operator buttons grey, "no game running" | `play` stopped: look at window 2 |
| Operator page does not load on the laptop | The tunnel terminal (F) closed: run it again |
| Page loads but clicks do nothing | Open it through the tunnel (`localhost`), not `10.42.0.1:8091` |
| `ENGINE random mover` | `source ~/chess_arm/scripts/chess_env.sh` again in that window |
| No sound | `pactl list short sinks`; `--sink usb` or `--sink hdmi`; monitor volume |
| Picks miss sideways by the same amount | Re-touch the corners (G) |
| A move read wrong | **No** and type the move; never confirm with a piece still in a hand |
| After a Jetson reboot | `ur-link-chess` is down again: section C from the top |
