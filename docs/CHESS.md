# Phase 2: a chess match against a student

**October 14 showcase: [laptop launch commands and presenter guide](CHESS_SHOWCASE.md).**
Use the existing laptop devcontainer, single RG2 and live camera dashboard.
The Pi section below is historical planning, not tomorrow's setup.

Project 1 (language-directed picks) closed on 2026-10-10 on repeated real
picks. Phase 2: the UR7e plays a full game of chess against a student on a
real board, with the pieces it already knows how to pick. The deadline is
tight, so the order is a system that works, then one that sees, then one
that looks good. Tracked on
[UR7e Chess Match (phase 2)](https://github.com/orgs/PawPrintStudio/projects/3),
issues #53 to #59.

## Why chess is mostly solved already

| A pick in project 1 needed | In chess it is |
|---|---|
| the camera to find the object (DETECT, LOCATE) | a lookup: `e2` is at a known XY and the piece on it is a pawn of known height |
| a grasp height guessed from the silhouette | the piece table, `scripts/lab_chess.yaml` |
| a drop point | the destination square, or the box for a captured piece |
| the language model | python-chess for the rules, Stockfish for the move |

The motion stages (PLAN, APPROACH, GRASP, LIFT, RETREAT, HOME) ran on the arm
in project 1 and are reused unchanged: `scripts/lab_chess.py` hands the
orchestrator a square lookup in place of the camera (`ChessAdapters` wraps
`lab_pick.LabAdapters`). The camera comes back for one thing, reading the
student's move, and that is a before/after comparison of 64 small image
regions, not open-vocabulary detection.

## Three versions, in order

1. **Blind but playing** (C1 to C4, #53 to #56). Board corners touched by the
   fingertip give the square centres. The operator types the student's move,
   the engine answers, the arm executes it. This is a complete match.
2. **Seeing** (C5, C6, #57, #58). A photo from `observe` before and after the
   student's move; the squares that changed give the move. Re-centre a piece
   from the photo before picking it, since people do not centre pieces.
3. **Showing** (C7, #59). A page with the board, the stage lines and the photo
   for the people watching.

## Running it

Without the robot:

```
python3 -m pytest -q scripts/test_lab_chess.py                  # geometry, steps, targets, width check
python3 scripts/lab_chess.py move exd5 --steps-only --fen ...   # the physical steps of a move
python3 scripts/lab_chess.py play --steps-only --engine none    # the match loop in the terminal
python3 scripts/lab_chess.py squares                            # the centres from lab_chess.json
```

With the driver and MoveIt up (`CHESS=true bash scripts/lab_up.sh` in the
container: no camera, no OWLv2, no console), plan-only until `--execute`:

```
python3 scripts/lab_chess.py touch --corner 1     # fingertip on the a1 corner; then 2 (h1), 3 (h8), 4 (a8), box
python3 scripts/lab_chess.py fit                  # -> scripts/lab_chess.json (refuses touches that disagree)
python3 scripts/lab_chess.py goto e4 --touch      # hover, then point 10 mm over the square
python3 scripts/lab_chess.py move e2e4 --execute --max-speed-percent 25
python3 scripts/lab_chess.py play --execute --max-speed-percent 25
```

Dependencies: `pip install chess` and `apt install stockfish`; the dev
container has both. Without Stockfish the arm plays random legal moves,
captures first, which is enough to test the mechanics.

### On a Raspberry Pi 5

The dev container's usual base image is amd64 only. On the Pi (64-bit
Raspberry Pi OS or Ubuntu 24.04, Docker installed, 8 GB RAM or more) build it
from `ros:humble-ros-base`, which has no RViz. All the apt packages exist for
arm64.

```
export ROS_BASE_IMAGE=ros:humble-ros-base    # before VS Code starts: lab_up.sh on the host
bash scripts/lab_up.sh                       # host: ur-link, then Reopen in Container
colcon build --symlink-install               # in the container, first time
bash scripts/tier1_smoke_test.sh             # mock hardware: proves the image before the arm
CHESS=true bash scripts/lab_up.sh            # driver + MoveIt
```

Not yet tried on a Pi. Open questions: whether the UR driver needs a
PREEMPT_RT kernel on the Pi (watch for the velocity-veto popup), and the
`ur-link` NetworkManager profile (static 192.168.56.1) must be created there.

## Lab checklist, next session

1. **Pick the board.** Squares of 45 mm or more: the open jaws (50 mm)
   straddle a piece and must clear its neighbours, which sit one square away
   along the file and rank and 1.4 squares away diagonally. A club board is
   40 to 46 cm across, twice the plate used so far (22 by 18 cm). Before
   committing, check reach with a top-down RG2 at all four corners, plan-only
   (`goto a1`, `h1`, `h8`, `a8`), and the sweep (`lab_path_audit.py`). If the
   far rank is out of reach, turn the board so the arm plays along the long
   side of its workspace, or take a smaller board whose pieces the RG2 can
   still close on.
2. **Where it sits.** Beside the table board or on top of it: the table
   calibration is only used for the plane height, and version 1 takes no
   photo. Version 2 needs all 64 squares in the camera's view from `observe`;
   check that in the same session (`lab_snapshot.py`).
3. **Measure the set** with a ruler into `lab_chess.yaml` and set
   `measured: true`, which turns on the jaw-width check.
4. **Touch the corners, fit, verify** with `goto --touch` on a1, h1, h8, a8
   and e4: within 5 mm.
5. **First moves** plan-only, then `--execute --max-speed-percent 25`
   confirming each step: with the board empty except the moving piece, then
   with neighbours, then a capture to the box.
6. **A game** with the operator typing what the student played.

## The camera in the loop (2026-10-10, `scripts/chess_vision.py`)

Needs the corners fitted (`fit --square`) and the camera pose saved
(`overlay`), and the ZED **rigidly mounted**: everything below assumes the
camera never moves, so a moved camera reads as a moved board.

| Command | What it does |
|---|---|
| `scan` | Stereo depth over the board: which squares hold a piece, its height, its centre (off-centre mm). `?` = hidden (arm, tall piece). |
| `track [--apply]` | Has the board moved since it was calibrated? Fits the board's slide and turn from the grid the fixed camera sees; `--apply` moves the calibration with it (refuses beyond 30 mm / 5 deg). |
| `move ... --vision` | Board check at the start, then each pick goes to the piece's **seen** centre and grips at 3/4 of its **measured** height (no per-piece table). |
| `play ... --vision` | Also reads the student's move from the board (Enter after they move; confirm or type it) and checks the board after every arm move. |
| `watch` | Shares its frames, so all of the above work while the live view runs. |

Design decisions, measured on the real board at 0.85 m:

- **OpenCV stereo, not the ZED SDK.** No working GPU here, none on the Pi 5.
- **Lens tilt re-fitted every session** (`fit_row_rotation`): rows were 3-34 px apart, 0.65 px after.
- **Heights relative to the board seen in the same frame.** Disparity read 39.6 px low (constant); each scan measures that offset on the calibrated board and removes it. Board noise 1.4 mm; a 40 mm pawn reads 41-42 mm.
- **Pieces as 3-D points standing on a square**, not image regions: from 45 deg a tall piece covers the squares behind it in the image, never in 3-D. Clusters with no low points (stereo mismatches on the repeating squares) or too few points for their distance are dropped; the gripper is cut out with the arm's own pose.
- **Board motion solved with the camera fixed** (x, y, turn: 3 unknowns), not by re-solving the camera's 6 from a partial grid (1-2 cm loose along the line of sight).
- **Empty close = 9.3 mm** on this RG2: the generic miss check is now 12.3 mm (lab_pick's 6 mm called an empty close a hold).

- **Piece centres are not corrected for "seeing only the front".** From 45 deg the
  camera sees each head all round: hand-placed pieces read 0.2-1 mm off centre
  uncorrected, 4-8 mm with a 2r/pi correction (2026-10-11, `DEPTH_BIAS = 0`).
- **Heights read about 2 mm high** (95th percentile of a noisy top): pawn 41.7 for
  40, king 82 for about 80.

Not yet considered: the neighbouring pieces' positions when the jaws open (fixed
32 mm, along the diagonal), and a carry height from the tallest piece actually
standing (fixed `--hover`, 0.08 for chess). A moved camera still reads as a moved board.

## The turn protocol

The student and the arm share a table. The arm moves only after the operator
has entered the student's move and the student has stepped back; `play` asks
"everyone clear?" before every arm move. The pendant slider stays at the
rehearsed value; the operator keeps a hand by the stop. A failed step stops
the loop and asks: retry from that step, finished by hand, or quit.

## The operator console (2026-10-13)

Every question `play` asks is also a button on
**http://localhost:8091/#operator** (the same page `watch` serves; without
`#operator` it is the projected audience view and has no controls). The
terminal still works: whichever answers first wins. How it is wired:

- `play` writes the open question into `log/chess/state.json` (`prompt`: id,
  text, options) and a heartbeat every second; `watch` serves it at `/state`.
- A click POSTs `/answer` with the prompt's id; `watch` writes
  `log/chess/answer.json`; `play` reads it, drops it if the id is stale.
  Answers are accepted from this computer only (127.0.0.1): the projected
  page cannot move the arm, and neither can anything on the network.
- **New game** and **Quit** are on every student prompt and at game over:
  a new group is one click, the record restarts, the arm stays parked.
  `reset` is only needed when `play` is not running.
- `play --resume` carries on from the recorded game after a restart of T3.

Prompts, in order of a turn: *student move* (Read the board / type it /
New game / Quit), *I saw e4: right?* (Yes / No, read again / type it),
*Robot plays b5, everyone clear?* (Go), one *b7 → b5: go?* per step
(Go / Stop), and after a failed step *Retry / I finished it by hand*.
The audience page turns each of these into a headline ("Your move · White",
"Robot is about to play b5: stand clear", "Arm stopped during b5").

## Speed (2026-10-14, spec S1 in CHESS_IMPROVEMENTS.md)

A step took 72 s at the baseline. What changed, and where each knob lives:

- **Two rates.** `play`, `move`, `goto` default to `--joint-rate 0.4`
  (travel: hover, carry, home) and `--contact-rate 0.12` (descend, lift,
  lower, back up). The envelope in `scripts/motion/ur_motion.py` is 0.6 rad/s
  nominal; the pendant slider scales everything below it.
- **Waypoints thinned** before timing (`scripts/motion/retime.py`): MoveIt's
  0.003 rad steps under a 0.05 s floor meant the rate was never the limit.
- **No OBSERVE move** when the arm already stands where the step starts, **no
  HOME between a capture's two steps** (it waits at the box hover), and a
  0.3 s settle before each plan instead of 1 s.

First run after this change, in order: `goto e4` plan-only (read the new
nominal seconds), then `move e4 --fen start --vision --execute
--max-speed-percent 25`, then 100 %. `--joint-rate 0.09 --contact-rate 0.09`
restores the baseline speed if anything looks wrong.

## Calibration order when something moved

The board corners are the only robot-side truth; the camera pose is fitted
to them, and vision centres are checked against them, so camera and grid
agreeing proves nothing about the robot. 2026-10-13: the stand was lowered,
the corners were shifted in Z by hand (+60 mm, two notes in `lab_chess.json`)
and never re-touched in X and Y; picks then missed by the same amount
everywhere while the camera agreed with the grid within 2 mm.

1. `touch --corner 1..4` (and `box`), `fit --square`: the real fix, 10 min.
   Check with `goto e4 --touch`, `goto a1 --touch`, `goto h8 --touch`:
   the fingertip must be within 3 mm of the centre, 10 mm up.
2. `overlay --fen start`: refit the camera to the new corners; check a1 in
   `log/chess/overlay.jpg`. Restart `watch`.
3. If the corners cannot be re-touched: measure the miss with `goto --touch`
   on two squares and `shift --dx .. --dy ..` (mm, base frame, ≤ 30 mm) the
   other way; `watch` reloads the geometry. This is a patch, not a calibration.

The tool length in `lab_tooling.yaml` (247 mm flange to tips) is 35-40 mm
longer than the ruler says. It cancels between touch and move for a vertical
tool, so picks are unaffected, but every camera-versus-model fingertip
comparison and the collision model carry that error. Measure it with the
jaws closed before trusting such comparisons.

## The improvement list, mapped

| # | Note on the sheet | Where it goes |
|---|---|---|
| 1 | No continuous check, cannot adjust to displacement | C5 reads the board every turn; C6 re-centres before a pick |
| 2 | False positives: nothing checks that the gripper holds something | C4: expected jaw width per piece, retry once, then ask |
| 4 | The calibration board must be fully visible | Not needed for chess: the chessboard has its own geometry (C1). Deferred |
| 5 | No shape identification, everything a square | Not needed: the game state says which piece is on which square (C1) |
| 6 | Latency | No detector in the loop; not a problem |
| 7 | Actions not combinable (pick, then give) | C2: a move is pick then place; a capture is two of them |
| 8 | "Go to square" in the terminal | C1: `goto e4` |
| 9 | UI out of the terminal, showing the steps | C7, after the match works |
| 10 | Local model, for latency and cost | Deferred: the language model is not in the chess loop |
| 11 | Height only adjusted for known objects | C1: the piece table |
| 12 | Switch grippers by object | Not needed: every piece fits the RG2. Deferred |
| 13 | OWL on the GPU, or nano OWL | Not needed: no OWL in the chess loop |
| 14 | The gripper checks a location, not degrees | C4 uses the width the RG2 reports |
| top | Pick an object from the missing box | C2: the captured-pieces box |

## Decisions, and why

- **Reuse the orchestrator rather than write a chess pipeline.** Every stage
  after LOCATE was proven on the arm. The chess layer is one adapter that
  answers DETECT and LOCATE from a table, plus a game loop.
- **Blind moves first.** The arm's own moves need no camera; reading the
  student's move does. Typing it in costs ten seconds a move and makes
  version 1 a complete game, which is what the deadline needs.
- **Jaws along the diagonal.** The adapter reports the board's diagonal as
  the piece's "long axis", and the existing yaw logic closes the jaws across
  it, where the neighbours are furthest.
- **Heights from a table, not from the camera.** The silhouette height guess
  was the weak link of project 1 (#41). Here the piece on a square is known.
- **Reading a move is 64 colour regions, not a detector.** Empty, white or
  black per square, compared with the legal moves. Cheap, no model to load.
- **The board thickness is measured, not typed.** The corner touches give the
  board's top; the table calibration gives the surface under it; the
  difference goes into the fingertip height once, at the pick.
