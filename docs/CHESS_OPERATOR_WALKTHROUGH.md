# Ro-Boko operator walkthrough — October 14, 2026

> **Physical execution on hold (October 13):** the b7 → b5 attempt contacted
> the board and aborted during APPROACH with a protective stop. Do not resume
> T3 execution or reset/retry using this guide until the contact cause and
> recovery clearance are established. Camera-only viewing remains available.
> See [incident evidence](CHESS_B7_CONTACT.md).

Follow this sequence in **VS Code Dev Containers** on the prepared laptop.
**HOST** means a normal laptop terminal; **CONTAINER** means an integrated
terminal in the VS Code window connected to the dev container. Three
terminals remain open: **T1 driver**, **T2 camera**, **T3 game**. Project the
browser at **http://localhost:8091/**; keep operator prompts on the laptop.

The stack uses the laptop, ROS 2 Humble container, UR7e, single RG2 and ZED
camera. Stockfish chooses moves, python-chess maintains legal state, and CPU
stereo measures the board. No Jetson, Raspberry Pi, language-model API key,
package upgrade or container rebuild is needed for this prepared setup.

## Readiness as of October 13

We connected to the real robot, observed NORMAL safety status, brought up the
driver, and ran the real camera/dashboard. The launcher's configured-controller
check passed after replacing its unavailable `ros2 control` CLI dependency.
31 chess/vision tests, 5 dashboard checks and 7 isolated readiness cases passed.

**A full new physical game and complete student-to-robot cycle have not been
demonstrated in this session.** The driver later timed out waiting for pendant
Play. After the camera mount was raised, its pose was refitted on October 13
using all 49 inner board corners (1.18 px fit error). The operator confirmed
orientation; robot-board coordinates were preserved. Calibration maintenance
is documented separately in [CHESS_CAMERA_RECALIBRATION.md](CHESS_CAMERA_RECALIBRATION.md).
The fresh board-position check passed (0.7 mm / 0.07 degrees), but three
stationary scans detected only 25–27 of 32 pieces, with persistent back-rank
misses. Resolve these perception limits and complete the physical rehearsal
before claiming full camera-guided play is ready. The latest regression run
had 34 passes and one live-view drawing assertion failure; this does not
replace the physical acceptance checks below.

## 0. Prepare the physical setup

Connect laptop power, robot Ethernet, ZED USB 3 and the projector. Secure the
camera at its final position with the entire board visible. Keep the board,
robot base and captured-piece box at their taught positions; mark them.
Confirm the actual tool and payload on the pendant. Keep an operator at the
pendant, with students clear until invited. Prevent laptop sleep during play.

Using the established lab procedure, park the arm clear of the board view.
Do not teach a new robot home just because the camera moved.

**This camera-only procedure assumes the board, base, tool and box did not
move.** If they did, stop and use the appropriate board/tool calibration
workflow in [CHESS.md](CHESS.md). Fitting a camera against stale board
coordinates does not recover the board's true position relative to the robot.

If a prior run is active: type `quit` at its game prompt, then Ctrl+C its
camera terminal, then Ctrl+C its driver terminal. Wait for shutdown before
starting again. Never run duplicate drivers or camera viewers.

Today's persistent terminals can be reopened from HOST, if needed:

```bash
tmux attach-session -t chess-camera-live
```

```bash
tmux attach-session -t chess-driver-live
```

These are optional recovery commands. A shell prompt means that terminal's
foreground demo process has already ended. A fresh run does not require tmux.

## 1. HOST — activate Ethernet and open VS Code

In a normal laptop terminal, run:

```bash
cd /home/nikola/Projects/Makerspace/AI_arm
export UR_LINK=ur-link-chess ROBOT_IP=192.168.1.10 LAPTOP_IP=192.168.1.2
nmcli con up "$UR_LINK"
lsusb
code .
```

| Command | What it does / what to expect |
|---|---|
| `cd ...` | Selects the checkout shared with the container. |
| `export ...` | Selects the chess robot's addresses and wired profile. |
| `nmcli con up ...` | Activates the robot network; expect successful activation. |
| `lsusb` | Lists USB devices; look for STEREOLABS/ZED. |
| `code .` | Opens this checkout in VS Code. |

If ZED is missing, reseat its USB cable without moving the mount. Merely
seeing `/dev/video0` is insufficient: today it initially belonged to the
laptop webcam. Use USB 3; resolve any USB 2/VGA warning before calibration.

In VS Code, press **Ctrl+Shift+P → Dev Containers: Reopen in Container**.
Wait for setup to finish and verify the bottom-left remote indicator says
**Dev Container**. If already connected to this project's dev container,
keep that window and skip reopening.

The old VS Code container previously failed to start because of a stale
`/run/user/1000/wayland-0` mount. If that exact error recurs, do not rebuild
blindly during demo setup. Start the prepared fallback using the appendix,
then use **Dev Containers: Attach to Running Container…**, select
`ur7e-chess-showcase`, and open `/workspaces/AI_arm`. All subsequent work can
still use VS Code terminals. Do not launch a second stack in another container.

## 2. Open and prepare three container terminals

In the connected VS Code window, use **Terminal → New Terminal** three times.
Rename them **T1 Driver**, **T2 Camera**, and **T3 Game**. These terminals are
already inside the container: do not run `docker exec` in them. Use Bash
(select the bash terminal profile if necessary).

Run this in **each** integrated terminal:

```bash
source /workspaces/AI_arm/scripts/chess_env.sh
```

This script enters the project, removes an inherited host virtualenv, loads
ROS and the built project, applies shared ROS discovery settings, and sets
the chess robot/laptop addresses. Expect `Chess terminal ready`.
Use **source**, not `bash`: the settings must remain in your current terminal.
It does not start a driver, camera, calibration or motion.
Label the windows T1, T2 and T3 so you can find them during a demonstration.

**Where each thing runs.** The prompt tells you which side you are on:
`ros@nikola-LOQ:/workspaces/AI_arm$` is the container, `~/Pr/Makerspace/AI_arm`
is the laptop itself. The container reads the repo folder live, so a `git
pull` needs no rebuild, but a **running** `watch`, `play` or voice keeps its
old code until restarted.

| Where | What |
|---|---|
| Container (T1, T2, T3) | `lab_up.sh`, `watch`, `play`, `gesture`, `goto`, `touch`/`fit`/`overlay` |
| Laptop (T4) | `.venv-voice/bin/python scripts/chess_voice.py` (sound card, GPU), the two browser windows |

A container terminal from any laptop terminal: `docker exec -it <container> bash`
(`docker ps` names it), then `cd /workspaces/AI_arm`.

## 3. Day-of setup — reuse the accepted calibration

Start with the camera calibration completed and accepted during preparation.
Keep the camera mount, board, robot base, tool and box at their verified
positions. Restore pieces to the standard start without moving the board.

**Before the first game, re-touch the board.** The saved corners are the
October 11 touches shifted in Z by hand after the stand was lowered; on
October 13 picks missed sideways by the same amount everywhere while the
camera agreed with the grid. With T1 up, in T3 (order and checks in
[CHESS.md, "Calibration order"](CHESS.md#calibration-order-when-something-moved)):

```bash
python3 scripts/lab_chess.py touch --corner 1   # a1; then 2 (h1), 3 (h8), 4 (a8), box
python3 scripts/lab_chess.py fit --square
python3 scripts/lab_chess.py goto e4 --touch --execute   # fingertip within 3 mm of the centre
python3 scripts/lab_chess.py overlay --fen start          # camera refit; check a1 in overlay.jpg
```

Then start the viewer below and check the grid alignment, a1 orientation and
fresh scan readings. If the mount moves later, use the
[camera maintenance guide](CHESS_CAMERA_RECALIBRATION.md). If re-touching is
impossible and `goto --touch` shows a constant miss, `shift --dx --dy` (mm)
moves the whole board the other way; it is a patch, not a calibration.

## 4. T1 — start the robot driver and MoveIt

On the pendant, load the established **External Control** program, verify
its host is **192.168.1.2** and confirm tool/payload. The demo software permits
the pendant slider up to 100%; the operator controls the actual setting.
Do not substitute an unrelated saved robot program.

In **T1 CONTAINER**:

```bash
CHESS=true LAB_SPEED=100 bash scripts/lab_up.sh
```

This starts the driver, checks that its controller is configured, waits for
pendant Play, checks that the controller is active, and starts MoveIt.
Chess mode skips the general object detector and language-model console.

When the terminal requests **PLAY**, verify everyone is clear and press
**Play on the pendant**. The launcher allows 180 seconds for the connection;
if it times out, wait for cleanup and rerun T1 when ready at the pendant.

Expected milestones:

```text
ready: scaled_joint_trajectory_controller configured
ready: pendant Play (reverse interface connected)
ready: scaled_joint_trajectory_controller active
ready: MoveIt (/compute_cartesian_path)
== chess stack up.
```

Leave T1 running. An API-key warning does not affect chess. In chess mode,
`LAB_SPEED=100` does not set the pendant slider or the game's approval ceiling;
the T3 command sets that ceiling explicitly. The trajectory rate remains
0.09 rad/s with the existing 0.10 rad/s nominal joint safety limit and minimum
waypoint timing. A 100% slider runs those trajectories without slider reduction;
it does not mean the robot's maximum rated speed. Safety scaling can still reduce
actual speed. Collision clearance and a physical rehearsal remain required.

## 5. T2 — start the live camera and dashboard

Using the calibration accepted during preparation, run in **T2 CONTAINER**:

```bash
python3 scripts/lab_chess.py watch
```

Expect `WATCH` and `http://localhost:8091`. Leave T2 running: it owns the
camera, shares stereo frames with the game, and serves the dashboard.
Open **http://localhost:8091/** on the laptop/projector. Use Ctrl+0 to reset
zoom, then the page's Full screen button if useful.

Check fresh video, updating scan age, a1/grid alignment and piece visibility.
Striped squares mean unknown depth, not empty. “Camera sees” estimates piece
types from height; the legal game record has a separate role. Before `play`,
the record may still be from October 11: its difference count is not a count
of invisible squares.

## 5b. T4 on the HOST — the robot's voice (optional)

In a **host** terminal, not the container (the prompt must not say
`/workspaces`; the script refuses inside the container), with the TV on:

```bash
.venv-voice/bin/python scripts/chess_voice.py
```

It speaks one line per game event (greeting, captures, checks, the result)
from a local model, or a canned line when the model is slow, through the
HDMI output (the TV). The first line must say `AUDIO` with `pw-play --target`;
if it says `aplay`, the TV sink was not found: `wpctl status`. Ctrl+C stops
it; the game does not depend on it. Setup commands are in CHESS.md,
"Reactions".

## 6. T3 — read-only checks before a game

With the physical pieces at the standard start and the arm clear of the
board view, run in **T3 CONTAINER**:

```bash
python3 scripts/lab_chess.py track --fen 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'
python3 scripts/lab_chess.py scan
```

`track` reports apparent board displacement without applying it. Unlike
`overlay`, its `--fen` takes an actual FEN, not the word `start`. `scan`
reports occupancy, heights and centres. Both use T2's shared frames and
neither commands a chess move.

The code considers below 1.5 mm and 0.3 degrees OK; this is not independent
proof of physical calibration accuracy. A camera newly fitted against this
board cannot prove that the board itself stayed in its robot-taught position.
Check the physical position marks too.

If alignment is wrong, relevant pieces remain unobserved, or board position
is unchecked/displaced, resolve it before execution. Do not accept a
“Follow it?” prompt just to dismiss a stale-camera error.

## 7. T3 — start the student versus robot game

Confirm T1 says `chess stack up`, T2 is live, calibration is accepted and
the physical pieces are at the normal start. In **T3 CONTAINER**:

```bash
python3 scripts/lab_chess.py play --vision --execute --max-speed-percent 100 --skill 20 --hover 0.10
```

| Option | Meaning |
|---|---|
| `play` | Starts a new standard game (`--resume` carries on from the recorded one). Every prompt is also a button on **http://localhost:8091/#operator** on this laptop. |
| `--vision` | Reads student moves (watching the board while the prompt waits) and measures piece centres/heights for picking. |
| `--execute` | Enables real motion after operator confirmations. |
| `--max-speed-percent 100` | Permits live speed scaling up to 100%; does not set the pendant slider or remove trajectory safety limits. |
| `--skill 20` | Stockfish at full strength; `--skill 1` for a beatable robot. |
| `--hover 0.10` | Carries pieces 10 cm over the tallest piece (8 cm warns). |
| `--careful` | A Go per step as well as per move: first runs after a calibration change. |
| `--no-gestures` | No wrist reactions. |

Student is White; robot is Black. Keep confirmations; **do not add `--yes`**.
If startup warns that board position could not be checked, stop and resolve
it rather than interpreting a continued prompt as readiness.
The first line must read `ENGINE stockfish`. `ENGINE random mover` means
Stockfish is missing: `sudo apt-get install -y stockfish` in the container,
re-source `chess_env.sh`, and start `play` again.

### Each turn

Open **http://localhost:8091/#operator** in a second browser window on the
laptop (not the projected one: the projected page shows the same game with no
controls). Every prompt below appears there as buttons and in T3 as text;
answer in either place.

1. At the student prompt, invite one legal physical White move. The camera
   reads the board every few seconds; do not press anything until the
   student has let go of the piece.
2. `I saw ...` appears by itself once three readings agree (or click **Read
   the board**). Look at the real board, then click **Yes**, or **No**, or
   type the actual move, such as `e2e4`. Never confirm while a piece is
   still in the student's hand.
3. Read the robot's reply. Click **Clear: let the arm move** only when
   everyone is clear; that approves the whole move (a capture is two steps).
   With `play --careful` there is also a **Go** per step.
5. Wait for completion and check the real board before inviting another move.
   Resolve any post-move disagreement honestly (**Board fixed**).

If the buttons are grey and the page says *no game running*, `play` is not
running or has stopped: look at T3.

Typing the student move at the initial prompt is a fallback for ambiguous
recognition, not a fix for bad calibration; robot picking still uses vision.
The dashboard's scan button requests measurement only, never arm motion.

## 8. Physical rehearsal and acceptance

Use the final mount position, board, pieces and audience boundary. Rehearse
the full student → camera → confirm → robot reply → board-check cycle.
Check open-jaw clearance beside neighbours, stable grasp and return to the
taught chess home. Rehearse captures/box and carried-piece clearance if they
will be shown.

The piece table remains `measured: false` with some placeholder values. Do
not mark it measured without measurements. Scans and offline tests do not
prove every possible chess move is physically reliable. Use a short guided
exchange unless broader play is rehearsed; reject unsafe/unrehearsed physical
actions before execution.

- [ ] Overlay accepted: a1, directions and all four corners checked.
- [ ] Driver, pendant Play and MoveIt ready with correct tool/payload.
- [ ] Fresh scans and student move recognition checked.
- [ ] Real robot reply completed with stable grasp and neighbour clearance.
- [ ] Physical board agrees after the reply.
- [ ] Capture/box checked if included.
- [ ] Reset procedure and projector readability checked.

## 9. Reset between groups and shut down

Click **New game** on the operator page (or type `new` at the student prompt):
the record restarts at the standard position, the arm stays parked, `play`
keeps running. Reset the pieces to the standard start without moving the
board, box or mount, then invite the next student. **Quit** stops `play`.

Only when `play` is not running and the display still shows an old game,
clear it from the T3 shell:

```bash
python3 scripts/lab_chess.py reset
```

This starts a fresh **record**: standard position, White to move, robot Black.
It clears move history, the planned target, old failure status and grip reading.
The dashboard updates automatically; allow its next scan to update comparisons.
It does not move pieces, start the engine, enable motion or change calibration.
Use `reset --arm-white` only when the next game also uses `play --arm-white`.
An active chess command from this updated version blocks reset; quit it first.
For a game already running before this update, quit it before using reset too.

Once physical execution is cleared for use, rerun the T3 `play` command for the
next group. Starting `play` also clears stale status from the previous game.

At the end: quit T3, Ctrl+C T2, then Ctrl+C T1. Wait for cleanup. Ctrl+C is
not an emergency stop; use pendant stop controls as appropriate. After the
processes stop, use **Dev Containers: Reopen Folder Locally** to leave the
container workspace; that action alone is not a guarantee the container has
stopped. If using the prepared fallback container, explicitly stop it on HOST:

```bash
docker stop ur7e-chess-showcase
```

That command applies only to the named fallback container, not necessarily
the container VS Code created for the project. It stops software, not robot
power. Follow the lab's normal
robot shutdown procedure separately.

## 10. Quick troubleshooting

| Symptom | Action |
|---|---|
| No ZED | Check HOST `lsusb`; reseat USB 3 cable without moving mount. |
| Camera busy / port occupied | Stop the old viewer; run only one `watch`. Stop it before `overlay`. |
| Unknown entire file/rank after camera moved | Stop demo startup and use the separate camera maintenance guide. |
| Live updates paused | Inspect T2/USB; require fresh video and scan age. |
| Many differences before `play` | Check the old game record and actual perception; new `play` resets the record. |
| Robot unreachable | Check Ethernet, `ur-link-chess` and pendant IP `192.168.1.10`. |
| Timeout waiting for Play | Verify External Control host `192.168.1.2`; rerun T1 after cleanup. |
| Stack already running | Stop it normally in its terminal before another launch. |
| Missing `chess_home` | Resolve the taught pose through the lab procedure; do not substitute the old arm's home. |
| Motion failure/protective stop | Keep people clear, inspect the cause, and do not repeatedly retry. |
| Failed-motion prompt offers `h` | Use only after the operator actually completed the remaining move by hand. |
| Operator page button says "Sent" but nothing happens, or "not served by this watch" | `watch` is older than the page: Ctrl+C and restart it in T2, reload the page. |
| Voice speaks but no jingle, or speaks every line twice | Restart `chess_voice.py` after pulling; make sure only one is running (`pgrep -af chess_voice`). |
| `ModuleNotFoundError: chess` | That terminal is the laptop, not the container. |
| Gesture "skipped: execution gate closed" | The pendant program is not running (after a stop): press Play, run it again. |
| `I saw …` shows a capture the student did not make | A piece held in the hand; click No, let the student finish, read again. |

T1 logs: `log/lab/<timestamp>/driver.log` and `moveit.log`. Calibration image:
`log/chess/overlay.jpg`. Board/camera geometry: `scripts/lab_chess.json`.
Game record: `log/chess/state.json`.

## Optional software checks and fallback

In a sourced CONTAINER terminal, before a live match:

```bash
python3 -m pytest -q -p no:cacheprovider scripts/test_lab_chess.py scripts/test_chess_vision.py
```

On HOST, from the project directory:

```bash
node scripts/test_chess_dashboard.cjs
```

October 13 results were 31 and 5 passing tests. These are software evidence,
not calibration or physical clearance proof.

For an explicitly labelled software-only fallback, stop the real match and
run in the sourced CONTAINER:

```bash
python3 scripts/lab_chess.py play --steps-only --engine none
```

This issues no robot moves and uses a random legal opponent. It changes the
game record, so reset before resuming real play.

## Appendix: prepared Docker fallback (optional)

Use this only if the normal Dev Containers startup fails. On HOST:

```bash
docker start ur7e-chess-showcase
```

Then in VS Code choose **Dev Containers: Attach to Running Container…** →
`ur7e-chess-showcase`, open `/workspaces/AI_arm`, and follow section 2.
This fallback uses the same cached image and checkout, avoiding the old
container's stale Wayland mount. If the named container does not exist,
resolve that preparation issue rather than substituting an arbitrary image.

For a shell-only fallback instead of VS Code, run this from each HOST terminal:

```bash
docker exec -it -w /workspaces/AI_arm \
  -e ROBOT_IP=192.168.1.10 -e LAPTOP_IP=192.168.1.2 \
  ur7e-chess-showcase bash
```

Then source the environment from section 2. Choose one workflow; do not run
duplicate driver/camera processes across containers.

## Five-minute presenter flow

Explain **see → think → move → check**. Compare the camera estimate with the
game record; invite a student move and predictions; demonstrate the confirmed
physical reply with everyone clear; then compare the board again. Ask what
else a chess program needs to manipulate real pieces. Keep interaction within
the moves and piece types you have physically rehearsed.
