# b7 contact — October 13, 2026

The operator confirmed that the **fingertips** contacted the board during the first robot reply b7 → b5,
after White played e4. Physical execution is on hold; no recovery motion was
commanded during this investigation.

## Recorded evidence

- Run: `log/runs/366dcd11838b4ef491c1f8caec8d33b6.jsonl`.
- LOCATE: pick xyz = [0.092729, 0.472279, -0.064544] m.
- PLAN: fingertip height 43.8 mm above the underlying table; with the saved
  20 mm board thickness this is 23.8 mm above the board. Hover: 138 mm above
  the underlying table. These are model targets, not physical measurements.
- APPROACH failed with `motion_stopped`; no GRASP stage occurred. APPROACH
  includes both hover positioning and descent; this log does not distinguish
  which subsegment failed.
- Driver log `log/lab/20261013-203110/driver.log` reports PROTECTIVE_STOP and
  controller code C153A1. The code's meaning has not been independently decoded.
- A subsequent read-only live snapshot reported rg2_tcp xyz
  [0.092501, 0.474454, 0.004586] m, board plane z = -0.102918 m at that XY.
  With the configured 45 mm TCP-to-tip offset and near-downward orientation,
  the model places the fingertips approximately 62.5 mm above the board.
  This is a later snapshot, not a guaranteed capture of the instant of contact.
- That snapshot returned no modeled contacts, safety mode numeric 1,
  External Control running, and speed scaling 0%. It does not prove physical
  clearance or establish which part touched the board.

## Open investigation

A second read-only check used `measure_fingertip`, the same TF/tooling calculation
used to record board touches. It returned tip xyz [0.092456, 0.475019, -0.038623] m,
tilt 0.755 degrees, jaw width 33.6 mm and jaw shortening 1.8 mm: 64.3 mm above
the saved board plane. This agrees with the earlier FK-based estimate after
jaw shortening. The operator subsequently confirmed that the arm was not moved
after fingertip contact. The physical contact therefore contradicts the saved
geometry; these readings alone must not be used as a calibration correction.

The robot graph shows one robot-state publisher, the driver reports a matching
factory-calibration checksum, and tooling settings match the committed chess
setup. No root cause has yet been established. Tool verification has been revoked
(`scripts/lab_tooling.yaml`, `verified: false`) so a new real execution is refused
until board/tool geometry is reconciled. An already-running game must be quit;
it has already loaded its configuration and will not reread this flag per move.

Confirm the pendant message and whether the arm, board or
camera moved after contact. Reconcile physical tool geometry and board position
against the live model before proposing recovery or changing target heights.
The descent currently passes `avoid_collisions=False` and depends on the
configured fingertip floor. A successful plan is therefore not contact proof.
Do not compensate with a guessed Z offset or attribute this to camera height
overprediction without further evidence.

Read-only diagnostic: `log/chess/inspect_stopped_pose.py`. It reads joints,
forward kinematics and existing-scene validity without updating the scene or
commanding motion.

## After operator moved the hand

The operator subsequently moved the hand. A fresh read-only TF/gripper sample
gave fingertip z = +45.6 mm in base_link and 148.5 mm above the saved board
plane. These are different from the unmoved contact-pose measurements above.

Fresh stereo samples from two small visible fingertip patches had median
heights 118.6 and 113.9 mm above the saved board plane; their 10th–90th
percentiles spanned approximately 105–125 mm. Board depth IQR was 4.35 mm;
that is not a bound on fingertip accuracy. Reflective metal, edge pixels and
the board-referenced depth correction limit interpretation. These are rough
surface samples, not a calibrated fingertip position or a recovery target.

`log/chess/current-tip-comparison.jpg` shows the projected model tip in red
and depth-sampling patches in yellow. The model projection is visibly above
the real tips. Camera pose and disparity correction depend on the existing
board calibration, so this does not isolate tool-length error from frame error.
No calibration correction or motion was performed.

## Parked-pose repeatability check

After the operator parked the gripper away from the board, fresh image patches
were selected at the new fingertip locations. Two stationary measurements gave
one visible fingertip patch median heights of 137.8 and 139.4 mm relative to the
saved board plane (mean 138.6 mm). The model estimates were 171.13 and 171.14 mm.
The other fingertip patch had no valid depth samples in either capture. Thus
the observed discrepancy is approximately 32.5 mm at this pose, not a validated
64 mm correction. Surface-patch depth is not the precise lowest fingertip point;
missing depth and the shared board reference prevent using this average as a
motion calibration. The model assumes 247 mm flange-to-closed-tip length
(202 mm to TCP + 45 mm to tips), which still needs physical verification.
Tool verification remains false; no motion or offset changes were made.

## Operator-confirmed ruler estimate

The operator confirmed 210 mm total wrist-mounting-face-to-fingertip length
after reviewing the ruler image. The image estimate itself was approximately
210 mm with 20 mm uncertainty, and the jaws were open. It is recorded as a
candidate in `log/chess/calibration-backups/focused-20261013/measurement-candidate.json`.
It has not been substituted into the active model. In particular, it must not
be used with the old board touches or treated as a verified closed-jaw length.

## Camera fit to the offline candidate

Fresh empty-board image `log/chess/focused-empty.jpg` produced a complete
49-corner grid. Camera fitting against `board-candidate.json` passed the image
fit check at 1.51 px RMS (reported local scale approximately 1.4 mm).
The inspected overlay `log/chess/focused-candidate-overlay.jpg` places a1
lower-right, h1 upper-right, h8 upper-left and a8 lower-left, consistent with
the previously confirmed orientation. Camera data was saved only to the
offline candidate. Active robot calibration and revoked tooling verification
were unchanged. The fitted square dimensions still derive from the inconsistent
touches; image alignment does not validate their metric size or robot clearance.

## a1 recheck, 21:36 UTC

The original reference set was archived before replacing a1. The new four-point
square residuals are 2.46, 1.43, 2.54 and 3.23 mm; maximum improved from 5.11 mm
but still exceeds the existing 3 mm check. Plane residuals are about 0.11 mm.
Corner 4 (a8) is now the largest square-fit residual. The prior candidate and
its overlay are marked superseded; neither is active. No motion was commanded.
