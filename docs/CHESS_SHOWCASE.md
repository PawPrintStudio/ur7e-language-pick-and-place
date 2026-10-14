# Robot Chess Lab · laptop showcase, October 14

> **Physical demo on hold:** October 13 b7 pickup contacted the board.
> [Incident evidence and unresolved checks](CHESS_B7_CONTACT.md).

**For the day-of operator command sequence, use
[CHESS_OPERATOR_WALKTHROUGH.md](CHESS_OPERATOR_WALKTHROUGH.md).**
Camera recalibration is a separate preparation procedure in
[CHESS_CAMERA_RECALIBRATION.md](CHESS_CAMERA_RECALIBRATION.md). Today's live
camera and software checks do not constitute a completed physical match.

Use the proven laptop, existing ROS 2 Humble devcontainer, UR7e with the
single RG2, and ZED 2i over USB 3. Chess uses local rules, Stockfish and CPU
stereo; no language-model API key or internet is needed once the container
and dependencies are present. Do not rebuild or migrate hardware for the show.

## 1. Laptop and pendant

Plug in laptop power, robot Ethernet, camera and projector. Disable automatic
sleep for the presentation and silence notifications. Mark the board, camera
and captured-piece box positions so they cannot drift between groups.

The documented **chess arm** uses robot `192.168.1.10`, laptop `192.168.1.2`,
and NetworkManager profile `ur-link-chess`. These differ from the original
pick-and-place arm. Check the pendant network and External Control host before
using these values; they must match the robot you are physically using.

On the **laptop host**, in a terminal:

```bash
cd ~/Projects/Makerspace/AI_arm
export UR_LINK=ur-link-chess ROBOT_IP=192.168.1.10 LAPTOP_IP=192.168.1.2
nmcli con up "$UR_LINK"
docker start ur7e-chess-showcase
```

The prepared `ur7e-chess-showcase` container uses the exact cached image from
the working VS Code devcontainer. It starts an idle shell environment only.
This avoids the saved VS Code container's obsolete `/run/user/1000/wayland-0`
mount, which prevented it from starting during the October 13 check. No image
rebuild, package upgrade or change to the old container is needed.

Open **three laptop terminals**. In each, enter the prepared container:

```bash
docker exec -it ur7e-chess-showcase bash
```

Then in each container terminal:

```bash
cd /workspaces/AI_arm
if [ -n "${VIRTUAL_ENV:-}" ]; then deactivate; fi
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/lab_env.sh
export ROBOT_IP=192.168.1.10 LAPTOP_IP=192.168.1.2
```

Explicit exports avoid inherited old robot addresses. Use the container's
Python, not the laptop's `.venv`. VS Code can still edit the same checkout;
these terminal commands do not require VS Code to manage the container.

Pendant: confirm the installed tool and payload, load the established External
Control program, and verify its host is `192.168.1.2`. The software permits the
pendant slider up to 100%; the operator controls the actual setting. Keep the
operator at the pendant, with access to the stop controls.

## 2. Three terminals, one browser

**T1 — driver and MoveIt:**

```bash
CHESS=true LAB_SPEED=100 bash scripts/lab_up.sh
```

When prompted, press **Play on the pendant**. Wait for `chess stack up`.
Leave this terminal running. An API-key warning is irrelevant to chess.
`LAB_SPEED` alone does not set the chess process's speed cap; T3 does that.

**T2 — camera and dashboard:**

```bash
python3 scripts/lab_chess.py watch
```

The camera is auto-detected. If necessary use `watch --device N`, with the
actual ZED capture device number. Keep one `watch` process: it owns the camera
and shares stereo frames with the game. Open **http://localhost:8091** in the
laptop browser and project it. Use the page's **Full screen** button.
Reset browser zoom with Ctrl+0 first; use the actual projector to choose a
readable zoom. On narrow windows the page intentionally stacks and scrolls.

Before a game, verify the overlay's a1 and grid line up with the real board,
scan age updates, and all pieces are visible. A striped square means unknown,
not empty; the game record and camera estimate have different jobs.

**T3 — interactive game, after the physical rehearsal checks below:**

```bash
python3 scripts/lab_chess.py play --vision --execute --max-speed-percent 100 --skill 1
```

This starts a **new standard position**, student White, robot Black. Reset the
physical pieces first. Keep confirmations enabled; do not add `--yes`.
The 100% approval ceiling does not set the pendant slider. Chess still requests
0.09 rad/s within the existing 0.10 rad/s joint safety envelope and minimum
waypoint timing; robot safety scaling also remains active. This is not maximum
rated robot speed or proof of safe clearance.

Every prompt below is also a button on **http://localhost:8091/#operator**
in a second browser window on the laptop (the projected page has no
controls; answers are accepted from this computer only). **New game** there
restarts the record for the next group without leaving `play`.

1. Invite one student to make a legal White move when the presenter permits it.
2. Student steps back; the **operator** clicks **Read the board** (or Enter in T3).
3. Read the detected move and confirm it, or type the real move (`e2e4`, `Nf3`).
4. Confirm everyone is clear before approving the robot reply and each step.
5. Let the arm finish; check the physical board before inviting the next move.

**Check again** on the audience page requests a camera scan only. If camera
move recognition is uncertain, type the move at the existing prompt; this
preserves the same game and camera-assisted robot picking.

## 3. Final rehearsal — physical evidence still required

The current piece table remains `measured: false`; several widths are still
placeholders. Do not flip that flag without measurements. Offline tests and
a polished dashboard do not establish that every chess move is physically
reliable. Use the same board, pieces, tool and positions that were rehearsed.

- Confirm current board/camera calibration and the taught chess home. If the
  setup moved, use the board touch/fit and camera overlay workflow in
  [CHESS.md](CHESS.md); do not compensate by guessing offsets or raising limits.
- Rehearse the intended opening and replies at low speed with the actual
  audience boundary, including jaw clearance beside neighbouring pieces.
- Rehearse a capture only if you intend to show it: carried-piece clearance,
  destination clearance and the captured-piece box all matter.
- Check a missed/ambiguous camera reading can be corrected by typing the move.
- Verify projector readability, then run the whole student → camera → confirm
  → robot → board-check cycle. Keep the group interaction short enough to use
  moves and piece types you have actually validated.

If broader chess play has not been rehearsed, present a short guided exchange
rather than promise an unrestricted full match. An operator can reject an
unrehearsed move before physical execution and reset for the next group.

## 4. A five-minute interaction

**0:00 — Invite a prediction.** “This robot has to see, think and move. Which
part do you think is hardest?” Point to the game record and camera board.

**0:45 — See.** Point to the camera estimate and game record. “One is a
measurement; one remembers the rules and moves. Why might they disagree?”
Compare the same square in both views. Explain that a striped square means
the camera does not know, rather than that the square is empty.

**1:30 — Your move.** Have a student choose a rehearsed legal move. Let the
group predict the robot's reply while the operator checks the detected move.

**2:30 — Move.** Watch the physical robot. Ask students to count the smaller
actions hidden inside one chess move, then check the two board views again.

**4:00 — Reflect.** “Could a great chess program pick up a piece without a
camera, calibration and motor control?” Invite an idea for another robot job.

## 5. Reset, stop and fallback

- Between groups: at a student input prompt type `quit`; let the current action
  finish before approaching. Reset physical pieces and rerun T3. `play` starts
  a new game; it does not silently resume the old position.
- If motion fails, use the pendant stop controls as appropriate, keep people
  clear, and inspect the terminal. Do not repeatedly retry a protective stop.
  The `h` option means the operator really completed the remaining move by
  hand; never use it merely to dismiss an error.
- Dashboard says **Live updates paused**: the last board may be stale. Inspect
  T2 and reconnect before relying on it. A connected page is not proof of
  fresh camera frames; watch scan age and visible camera motion as well.
- End normally: quit T3, Ctrl+C T2, then Ctrl+C T1. T1 stops the processes it
  started. Terminal Ctrl+C is not a replacement for the pendant stop controls.
  After all three have stopped, run `docker stop ur7e-chess-showcase` on the host.
- If physical play cannot continue, stop the live game and explain the learning
two board views. A terminal-only fallback is available with
  `python3 scripts/lab_chess.py play --steps-only --engine none`; label it clearly
  as a software demonstration. It changes the recorded game, so reset before
  resuming real play.

## Offline check before packing

In the sourced devcontainer, without launching the robot:

```bash
python3 -m pytest -q -p no:cacheprovider scripts/test_lab_chess.py scripts/test_chess_vision.py
python3 scripts/lab_chess.py watch --help
python3 scripts/lab_chess.py play --help
```

Bring laptop charger, Ethernet/USB cables, display adapter, the measured board
and pieces, captured-piece box, position-marking tape and this runbook.

## October 13 earlier offline preparation evidence

- Prepared container: `ur7e-chess-showcase`, using the cached devcontainer image
  `sha256:dc2f82fb30408fd3b4443ad6dccdc637b7ec76ca39f026bcd4df6c3941f698fa`.
  It was left idle; only its shell/sleep and Docker init were running.
- Chess, OpenCV, NumPy, YAML and ROS imports succeeded; Stockfish and the board
  and ZED calibration files were present. File presence is not calibration validation.
- **31 offline chess/vision tests passed**, using that image with networking
  disabled and the repository read-only. Tests now isolate saved target/state
  output in temporary files rather than writing to the demo's live target.
- **5 dashboard regression checks passed** with
  `node scripts/test_chess_dashboard.cjs` on the host: unknown depth, board
  disagreement, connection loss/recovery, scan failure and failed-motion status.
- Browser inspection used an explicitly labeled simulated board. Learning
  buttons, answer disclosure and scan-request feedback were checked.
- No robot or camera was attached for this verification. Real camera alignment,
  motion/clearance, projector readability and the interactive physical cycle
  remain the final rehearsal gate. No physical motion was commanded.
