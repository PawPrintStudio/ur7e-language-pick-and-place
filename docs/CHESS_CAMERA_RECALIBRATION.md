# Camera recalibration maintenance guide

## October 13 accepted camera pose

The camera mount was raised while the board/base/box stayed fixed, as
confirmed by the operator. A conventional full-grid detector on the board
region found all 49 inner corners after the partial-grid detector selected
spurious off-board points. The existing pose-fit acceptance check passed:
1.18 px RMS (approximately 1.2 mm in the overlay's local image scale).
This image-space estimate is not a robot-motion accuracy guarantee.

The operator confirmed a1 at image lower-right, h1 upper-right, and a8
lower-left. Only the `camera` section of `scripts/lab_chess.json` changed.
Previous pose: `log/chess/calibration-backups/before-camera-refit-20261013.json`.
Accepted overlay: `log/chess/camera-candidate-overlay.jpg`.
Source image: `log/chess/calibration-empty.jpg`.
Detected corners: `log/chess/empty-grid-classic.npy`.
The empty-board stereo check found no false pieces, with h2 unknown and
3.7 mm reported board noise. Piece visibility and physical motion are
separate checks; a fitted camera pose does not close those gates.

The board-position checker now tries conventional grids within the projected
board region before its whole-image search. Pixel coordinates are translated
back to the original image; fit/motion thresholds are unchanged. A fresh
standard-position check passed at 0.7 mm apparent shift, 0.07 degrees and
0.53 px fit error using 15 corners. Regression run: 34 passed, one live-view
drawing assertion failed (`test_the_live_view_marks_pieces_differences_and_the_move`).
Both new board-region tests passed, including detection of a synthetic 5 mm
board shift and handling of a board outside the image.

**Remaining perception limitation:** three subsequent stationary scans found
27, 25 and 26 of the expected 32 occupied squares. All 16 pawns were found;
e8, f8, g1, g8 and h8 were missed throughout, with additional intermittent
back-rank misses. f8/h8 were consistently unknown; reported board noise was
about 4.2 mm. Thus the camera pose and board-position check are accepted,
but unrestricted camera-guided play is not signed off. No arm motion was
commanded during this recalibration.

The standard automatic command below may reproduce the partial-grid failure
on this view. Do not loosen its fit threshold to force acceptance. Preserve
the accepted fixed mount for the demo; if it moves again, repeat assisted
full-grid fitting and visual orientation verification before use.

## When calibration maintenance is needed

Use this only after the camera mount moves, outside the normal demo startup.
Robot motion must remain stopped. The taught board, robot base, tool and box
must remain unchanged; otherwise camera-only fitting is insufficient.

## Camera pose fitting

Do this before starting `watch` or `play`. If `watch` is already running,
Ctrl+C it and wait for the shell prompt: `overlay` opens the camera directly
and must not compete with the viewer. These calibration commands do not move
the robot and do not require the driver to be running.

Keep the board fixed and arrange the pieces in the standard starting
position, including correct king/queen placement. Keep hands and the arm
clear of the grid. In **T2 CONTAINER**:

```bash
mkdir -p log/chess/calibration-backups
cp scripts/lab_chess.json "log/chess/calibration-backups/lab_chess-$(date +%Y%m%d-%H%M%S).json"
python3 scripts/lab_chess.py overlay --fen start
```

The first two commands preserve the previous calibration. `overlay` fits
the camera pose against the saved robot-referenced chessboard, updates the
`camera` section of `scripts/lab_chess.json`, and writes
`log/chess/overlay.jpg`. **It saves the pose before you inspect the image.**
`--fen start` tells the fitter where pieces are; use it only when the physical
board really is in the starting position.

Inspect the image in the laptop file manager, or run on **HOST**:

```bash
xdg-open /home/nikola/Projects/Makerspace/AI_arm/log/chess/overlay.jpg
```

Check all four corners and grid lines across the whole board. The label
**a1 must sit on the real a1**, with the correct file/rank directions. A small
fit error is not enough if the orientation is wrong.

If orientation is reversed, run in **T2**, then reopen and inspect the image:

```bash
python3 scripts/lab_chess.py overlay --fen start --flip
```

Use `--flip` only for the orientation alternative, not to correct a poorly
fitting grid. If pieces prevent a fit, with the robot stopped remove them
without moving the board and use the empty-board variant:

```bash
python3 scripts/lab_chess.py overlay
```

Its orientation alternative is `overlay --flip`. Inspect it, then restore
the standard starting position. If the command says `BLOCKED`, or labels
and grid do not align, resolve the view before proceeding.

**Do not use `track --apply` to compensate for camera movement.** That command
assumes a fixed camera and changes the robot's board coordinates. Recalibration
of the moved camera uses `overlay`.

Leave the accepted mount fixed. Always restart `watch` after recalibration
so the depth scanner loads the new camera pose. Tomorrow, if nothing moved,
verify and reuse the saved calibration; if the camera moved, repeat this step.


After a successful fit, inspect the whole-board orientation and alignment,
then check several fresh stereo scans before resuming camera-guided motion.
