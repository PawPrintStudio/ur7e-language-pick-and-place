# Chess: improvement specs (after the 2026-10-13 baseline game)

One spec per item, implemented one at a time, each with its own acceptance
test. Numbers come from `log/runs/*.jsonl` of the baseline game (9 successful
steps, 01:37–01:51 UTC on 2026-10-14).

## S1. Speed: a step in 20 s, not 72

**Now.** Median seconds per stage of one step (pick one square, place on another):

| OBSERVE | LOCATE | PLAN | APPROACH | GRASP | LIFT | RETREAT | HOME | total |
|---|---|---|---|---|---|---|---|---|
| 2.5 | 1.4 | 7.8 | 20.1 | 1.3 | 7.8 | 20.3 | 11.0 | 72 |

A capture is two steps: about 2.5 minutes of arm time per robot move.

**Why it is slow.** Three caps, all in software, all older than chess:

1. `MAX_JOINT_VEL = 0.10 rad/s` in `scripts/motion/ur_motion.py` (5.7°/s, a
   hard envelope) and `--joint-rate 0.09` (`lab_pick.py`). The UR7e's own
   joint limit is 3.1 rad/s; the pendant's safety settings cap tool speed
   separately.
2. `MIN_WAYPOINT_DT = 0.05 s` in `lab_jog.py` with MoveIt Cartesian plans of
   about 0.003 rad per waypoint: a 156-waypoint hover takes 7.8 s whatever
   the rate, because 0.003 / 0.09 = 0.034 s is already under the floor. The
   rate is not even the binding limit today.
3. `PLAN` dry-runs all five segments before anything moves (8 s); `HOME`
   plans a clear-up plus a joint-space move from scratch (11 s); `OBSERVE`
   goes home when the arm is already home (2.5 s).

**Design.**

- Two rates, not one: `travel_rate` for hover, carry, back-up, home
  (target 0.5 rad/s nominal) and `contact_rate` for descend, lift, lower
  (0.15 rad/s). Chess passes them; `lab_pick` keeps its defaults. The
  envelope in `ur_motion.py` rises to 0.6 rad/s, with the pendant's tool
  speed limit (Safety → Joint Limits / Tool speed) checked and recorded
  first: at 0.5 rad/s the TCP moves about 300 mm/s at full reach.
- Thin waypoints before retiming so each joint step is about
  `rate × MIN_WAYPOINT_DT` (0.025 rad at 0.5 rad/s): keep every n-th point
  plus the last. Or drop `MIN_WAYPOINT_DT` to 0.02 s; the scaled joint
  trajectory controller interpolates either way. Thinning is the safer
  change: the audit still sees every kept point.
- Chess skips OBSERVE (the camera reads `watch`'s frames, the arm is home at
  run start) and plans only approach + descend up front; lift, carry and
  home are planned when reached. Capture: no HOME between the box drop and
  the second pick (`run_steps` passes `home=False` for all but the last
  step).

**Expected.** Motion 60 s → about 12 s, PLAN 8 → 3, OBSERVE 2.5 → 0:
roughly 18–20 s per step, 40 s for a capture.

**Status 2026-10-14: implemented, not yet run on the arm.** Rates, thinning
(`scripts/motion/retime.py`, 6 tests), OBSERVE/HOME skipping (2 tests), the
0.3 s settle. PLAN still dry-runs every segment (kept on purpose: it is the
reachability proof before the jaws close); it should shrink with the shorter
settle. Measure with `scratchpad stages.py`-style per-stage medians after ten
moves at 100 %.

**Acceptance.** `goto e4` plan-only reports the new nominal seconds; ten
moves at 25 % slider then 100 % without a protective stop; grasp success
unchanged; the Gazebo rehearsal (`gazebo_pick_rehearsal.sh`) still 6/6.

**Risk.** Faster contact means harder protective stops: the descend and
lift stay slow, the hover stays 100 mm. The envelope change touches every
script that moves the arm; it is one constant, reviewed with the pendant
settings in hand.

## S2. One approval per move; a standing New game button

**Now.** A robot move is four clicks (Read, Yes, Clear, Go per step; a
capture is five). New game exists only on the student prompts; when `play`
is not running the page has no reset at all.

**Design.**

- `Clear: let the arm move` approves the whole move; the per-step `Go`
  returns only with `--careful` (first runs after a calibration change).
- A standing **New game** button on the operator page: while `play` runs it
  answers the current prompt with `new` (accepted on every prompt, including
  mid-move after the current step finishes); when `play` is not running it
  calls `POST /reset`, which `watch` serves by running the same
  `new_game_state` as `lab_chess.py reset`, refused if the record lock is held.
- The audience page shows the move list and whose turn it is.

**Acceptance.** A full move in two clicks; New game works in all three
states (student prompt, mid-move, no game); `reset` and the button leave the
same record.

**Status 2026-10-14: implemented** (`--careful`, standing New game / Quit,
`POST /reset`, move list; 4 new tests). Mid-move means between steps: the
arm finishes the step it is on, then parks.

## S3. Emotion: gestures and sounds on game events

**Design.** Events from the game loop (`play`): `captured` (the arm took a
piece), `lost` (its piece was taken), `check`, `won`, `lost_game`,
`thinking`, `greeting`. Each maps to a gesture and a sound.

- Gestures are wrist-only joint offsets from `chess_home` (wrist 1–3, ±0.35
  rad, 2–4 keyframes), run through the existing joint-line planner at the
  travel rate, collision-checked, only when the arm is at home. No teaching:
  wrist-only motion never reaches the board. Examples: *nod* (check),
  *shake* (lost), *wiggle* with three fast jaw open/close "claps"
  (captured; the RG2 clicks audibly), *droop* (lost game), *dance* (won).
  `--no-gestures` turns them off; they run while the student is thinking so
  they add no latency.
- Sounds play on the laptop (the container has no `/dev/snd`): `play`
  appends events to `log/chess/events.jsonl`; a host process
  `scripts/chess_voice.py` tails it and plays a clip or speaks a line (S4).

**Acceptance.** Each gesture plan-only from home passes the audit; on the
arm, no gesture approaches within 150 mm of the board plane; events logged
for a scripted game match the rules.

**Status 2026-10-14: implemented, not yet run on the arm**
(`scripts/chess_gestures.py`, 7 tests; events wired into `play`, 2 tests).
Clearance checks: 60 mm above the plane, 30 mm outside the footprint, at
every third planned point; a gesture is skipped whole, with the reason,
otherwise. First run: `play --steps-only` shows the events; on the arm,
watch the first `celebrate` with a hand by the stop.

## S4. Natural language, fully local: commentary and voice moves

**Hardware.** RTX 3050 6 GB, Ollama installed with `gemma4:e2b` and
`qwen3.5` (7 GB each, partly on CPU; a 3–4B model answers in about a second
on this GPU). No TTS or speech-to-text installed yet; `voice_input.py`
already does push-to-talk with faster-whisper on the host and a file inbox.

**Design, in this order.**

1. **Commentary.** `chess_voice.py` (host) reads `events.jsonl`, builds a
   prompt (last moves in SAN, Stockfish eval and best line from `play`,
   the event) and asks Ollama for one sentence in the robot's voice
   ("That knight was mine. I'll remember this."), then speaks it with
   Piper TTS (one downloaded voice, CPU, 0.3 s). Hard limits: 20 words, no
   second sentence, a 3 s timeout after which a canned line plays.
2. **Voice move entry.** Push-to-talk on the host (existing `voice_input.py`),
   transcript → `python-chess` parse of "knight f3", "e4", "castle"; the LLM
   only when the parse fails. The result lands in the student prompt's text
   box for the operator to confirm, never straight into the game.
3. **Questions.** "Why that move?" answered from the engine's principal
   variation, same pipeline, optional.

The backend is an Ollama HTTP call behind the existing `arm_language`
`Backend` protocol (`complete(utterance) -> str`), so the parser's
guardrails and replay tests apply.

**Acceptance.** Commentary within 2 s of the event on this laptop with
`watch` and `play` running; nothing in the game loop waits on it; a replay
file makes the tests deterministic.

## S5. Measure the set; grip by type

**Now.** `lab_chess.yaml` has placeholder heights and widths (`measured:
false`); with `--vision` each pick grips at 3/4 of the camera's height and
the width check is off (the camera does not know the type). The game record
does know the type on every square.

**Design.** Ten minutes with calipers per type: height, grip height (the
narrowest solid spot below the head), width there, base diameter → `grip_m`,
`width_mm`, `base_mm`, `measured: true`. The adapter then uses the record's
type for grip height and expected width, and the camera for the centre and
presence only. Carry height comes from the measured tallest piece.

**Acceptance.** A deliberate miss (piece removed after the scan) is reported
as `width_off` or `grasp_missed`; the knight's wider tolerance holds; the
`--hover` warning disappears.

**Status 2026-10-14: code done (2 tests), measurements pending.** A type
with `grip_m` + `width_mm` grips by the table and checks the width; the
camera's height is a sanity check against the record. Fill in
`lab_chess.yaml` with calipers, set `measured: true`.

## S6. Vision: use the known position

**Now.** Move reading already scores every legal move against the scan;
occupancy is absolute per square, so a back-rank piece the camera misses
(e8/f8/g8/h8 behind taller pieces) counts against the right answer.

**Design.** Compare the scan after the student's move with the scan before
it (both from `watch`), and score legal moves on the squares that
*changed*; squares that read the same, including persistent misses, count
for nothing. The record stays the prior for colour and for which squares
can change. A start-of-game scan of the known start position records
per-square noise and flags squares the camera cannot see, shown on the
operator page before the first move.

**Acceptance.** Replayed scan pairs from tonight's game read every move;
the `UNSURE` rate over a game drops (measure it: it is not logged today).

## S7. Also noticed

- `PLAN` doubled from 4 to 8 s tonight (`--hover 0.10` and the new home);
  covered by S1.
- The game loop does not log `SEEN`/`UNSURE`: add them to `events.jsonl`
  so S6 can be measured.
- Export the game as PGN at game over with a QR code on the audience page.
- An "attract mode" when nobody is playing: the engine plays itself,
  slowly, every few minutes, so the exhibit moves.
- ZED SDK depth (CUDA) on this laptop would beat OpenCV SGBM for the
  back rank; a separate project, not before S1–S6.
- Tool length 247 vs about 210 mm in `lab_tooling.yaml`: measure with
  jaws closed; it affects the collision model and camera-vs-model checks.
