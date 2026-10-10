# Phase 2: a chess match against a student

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

With the driver and MoveIt up (`scripts/lab_up.sh`; quit the console with
`KEEP_STACK=true`), plan-only until `--execute`:

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

## The turn protocol

The student and the arm share a table. The arm moves only after the operator
has entered the student's move and the student has stepped back; `play` asks
"everyone clear?" before every arm move. The pendant slider stays at the
rehearsed value; the operator keeps a hand by the stop. A failed step stops
the loop and asks: retry from that step, finished by hand, or quit.

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
