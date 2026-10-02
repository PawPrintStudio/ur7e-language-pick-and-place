# ur7e_orchestrator

The standardized pick workflow (tasks 4.1 and 4.2): a pure-Python state
machine that takes one utterance ("pick up the red block") through the same
fixed stages every time, with per-stage timeouts, a run ID, a structured
log, and a small failure/retry policy.

It contains **no ROS code**. All real work (language model, camera, MoveIt,
gripper) happens behind one *adapter* object. This package ships the state
machine, the adapter interface, and a mock adapter; the real-robot adapter
lives elsewhere and plugs in unchanged.

> Status: verified against the mock adapters only (`pytest`, see below).
> Nothing in this package has moved the real arm yet.

## The concept: what a workflow state machine is, and why we want one

A *state machine* is a program that is always in exactly one named state
and has an explicit rule for which state comes next. Here the states are
the stages of a pick:

`IDLE → PARSE → OBSERVE → DETECT → LOCATE → PLAN → APPROACH → GRASP → LIFT → RETREAT → HOME`

You could write a pick as one long function instead. It would work on a good
day. The state machine earns its keep on the bad days:

- **Fixed stages make runs comparable.** If every run can only be "in
  DETECT" or "in APPROACH", then two failed runs can be lined up next to
  each other: same stage or not? That question has no answer when each run
  is a different tangle of function calls.
- **Stable reason codes make failures countable.** Every stage ends with a
  short snake_case `reason` (`ok`, `not_found`, `grasp_missed`,
  `protective_stop`, `stage_timeout`). The code never contains run-specific
  text, so after 50 runs you can ask "how many failed with `grasp_missed`?"
  with `grep`. The human explanation goes in a separate `message` field.
- **Recovery is a transition, not an afterthought.** "Detector found
  nothing, look again" is just an arrow back from DETECT to OBSERVE. Because
  it is an arrow in the same machine, it is logged, counted, and bounded the
  same way as everything else.
- **Safety rules are checkable.** "After a protective stop nothing moves"
  is one branch in one function (`_drive` in `workflow.py`), and one test.

Read the code in this order: `stages.py` (names and timeouts), `adapters.py`
(what the machine asks the robot to do), `workflow.py` (the machine),
`mock.py` and `cli.py` (the dry run).

## The stages

| Stage | Adapter calls | Safety check on entry | Default timeout |
|---|---|---|---|
| `IDLE` | none: recorded as the state every run starts from | | none |
| `PARSE` | `parse(utterance)` → `Intent` | | 30 s |
| `OBSERVE` | `observe()`: move the arm out of the camera's view | yes | 120 s |
| `DETECT` | `detect(query)`, once per target | | 60 s |
| `LOCATE` | `locate(detection)`, once per target | | 15 s |
| `PLAN` | `plan(intent, target, place)` | | 60 s |
| `APPROACH` | `approach()`: open jaws, hover, descend | yes | 180 s |
| `GRASP` | `grasp()` (and `release()` if it missed) | yes | 30 s |
| `LIFT` | `lift()` | yes | 120 s |
| `RETREAT` | `retreat()`: carry, release, back away | yes | 240 s |
| `HOME` | `home()` | yes | 180 s |

- **Pick and place.** When the parsed `Intent` has a `place_query`, DETECT
  and LOCATE each make two adapter calls (pick target, then place target)
  inside the one stage, and `plan()` receives the located place `Target`.
  Otherwise `place` is `None`.
- **Safety check.** Before each stage that moves hardware the machine calls
  `safety_ok()`. `(False, reason)` aborts the run with that reason.
- **Timeouts are per stage, not per call.** DETECT's 60 s covers both of its
  `detect()` calls together. Override any of them:
  `Orchestrator(adapters, timeouts={'approach': 90})`.
- **IDLE is an explicit record.** Every `RunRecord.stages` list starts with
  an `IDLE` entry, so a clean run lists all eleven stages exactly as written
  above.

## Outcomes

`Orchestrator.run(utterance)` never raises for a failed pick. It returns a
`RunRecord` whose `outcome` is one of:

| Outcome | Meaning | Where the arm is |
|---|---|---|
| `succeeded` | All stages ran; `reason` is `ok`. | Home. |
| `refused` | The parser refused, or the operator declined (`Refused`). | Where it was: no motion follows a refusal, not even HOME. |
| `failed` | A stage failed and the retry policy had nothing left to try. | See the retry policy below. |
| `aborted` | Safety abort, stage timeout, or Ctrl-C. `abort()` was called. | Wherever it stopped. Nothing further is commanded. |

## The failure and retry policy (task 4.2)

`RetryPolicy(reobserve_on_not_found=1, regrasp_on_miss=1)` — each number is
how many times *one run* may use that retry. `RunRecord.retries` reports how
many were used, under the same two names.

| What happened | What the machine does | If the retries are used up |
|---|---|---|
| `NotFound` in DETECT | Back to OBSERVE, then DETECT again. | `failed` / `not_found`, then a courtesy HOME. |
| GRASP returns `holding=False` | `release()` (inside GRASP), LIFT to get clear, then the whole pick again from OBSERVE. | `release()`, LIFT, HOME, then `failed` / `grasp_missed`. |
| `SafetyAbort` from any adapter call, or `safety_ok()` returns False | `abort()`, then stop. Outcome `aborted` with the abort's reason. | Never retried. No HOME. A human re-issues the command. |
| A stage exceeds its timeout | `abort()`, then stop. Outcome `aborted` / `stage_timeout`. | Never retried. See "Timeouts" below. |
| `Refused` | Stop. Outcome `refused`. | Never retried. |
| Any other `StageError` in OBSERVE … APPROACH | Courtesy HOME (recorded as a HOME stage), then `failed` with that error's reason. | — |
| Any other `StageError` in GRASP … HOME | **No motion.** `failed`; the message says the arm was left where it stopped. | — |
| Any other `StageError` in PARSE | No motion (nothing has moved yet). `failed`. | — |
| An exception that is not a `StageError` | Same as the three rows above, with reason `unexpected_error` and the exception's `repr` as the message. | — |

Why these choices:

- **Re-observe once on not-found.** A hand in the frame or a bad exposure is
  common and cheap to retry. A second miss is more likely a real absence,
  and repeating forever would hide it.
- **Open the jaws and lift before trying again.** After a miss the jaws are
  closed on nothing, right at table height. Opening them first means every
  later decision can rely on "after a miss the jaws are empty"; lifting
  straight up first means the next move does not sweep the table.
- **No homing at or after GRASP.** From GRASP on, the arm may be holding an
  object the machine knows nothing about (how heavy, how well gripped). A
  blind move home could drop or drag it. Standing still and asking for a
  human is the safe default.
- **No automatic motion after a safety abort.** This is architecture
  decision Q8: a protective stop means the robot met something it did not
  expect. The orchestrator stops, and a person decides what happens next.
- **If the courtesy HOME itself fails,** the run keeps its *first* reason
  (that is the one worth counting) and the message adds what went wrong in
  recovery.

## Timeouts and the worker thread

A Python function call cannot be interrupted from outside. So the
orchestrator runs every adapter call on **one persistent worker thread** and
waits for the result with a timeout. One thread, never a pool: the real
adapter owns ROS objects that must not be used from two threads.

When a stage times out the orchestrator *stops waiting*; it cannot make the
stuck call return. So it:

1. calls `adapters.abort()` (from a helper thread, because the worker is the
   thing that is stuck),
2. records the stage as `timeout` and ends the run as `aborted` /
   `stage_timeout`,
3. refuses every further `run()` with `OrchestratorBusy` until the stuck
   call finally returns. Two overlapping calls into the adapter would be
   worse than a refused command. `orchestrator.busy` tells you whether a
   `run()` would be refused.

Ctrl-C while `run()` is waiting is handled the same way: `abort()` is
called, the log gets its `run_end` event (`aborted` / `interrupted`), and
the `KeyboardInterrupt` then continues to the caller.

## The run log

Pass `log_dir=` to get one file per run, `<log_dir>/<run_id>.jsonl`, one
JSON object per line, flushed as it is written (so a run that crashes still
leaves a readable log up to its last finished stage):

```json
{"event": "run_start", "run_id": "0f3c…", "t": "2026-10-02T20:42:05.334+00:00", "utterance": "pick up the red block", "started": "…", "policy": {…}, "timeouts_s": {…}}
{"event": "stage_end", "run_id": "0f3c…", "t": "…", "stage": "DETECT", "attempt": 1, "outcome": "failed", "reason": "not_found", "duration_s": 1.204, "detail": {"pick": {"query": "red block"}, "error": "nothing there"}}
{"event": "run_end", "run_id": "0f3c…", "t": "…", "outcome": "succeeded", "reason": "ok", "message": "All stages completed.", "retries": {"reobserve_on_not_found": 1, "regrasp_on_miss": 0}, "duration_s": 41.7}
```

There is exactly one `stage_end` line per executed stage; a retried stage
appears again with `attempt` 2. A stage's `outcome` is `ok`, `failed`, or
`timeout` (`skipped` is reserved in the record format; this version never
writes it).

For live progress, pass `on_event=callable`: it receives the same dict as
each line is logged, on the thread that called `run()`.

## Run the dry run

No robot and no ROS graph needed; it runs the real state machine against
`MockAdapters`.

```bash
# After `colcon build`:
ros2 run ur7e_orchestrator dry_run "pick up the red block"

# Or straight from this directory, no ROS at all:
python3 -m ur7e_orchestrator.cli "pick up the red block and put it on the tray"
```

It prints one line per stage, then the final `RunRecord` as JSON, and exits
0 only if the run `succeeded`.

Inject failures with `--fail STAGE:KIND` (repeat it to fail later calls of
the same stage too) and watch the policy react:

```bash
dry_run "pick up the red block" --fail detect:not_found      # retried, succeeds
dry_run "pick up the red block" --fail detect:not_found --fail detect:not_found   # fails, exit 1
dry_run "pick up the red block" --fail grasp:miss            # re-grasp, succeeds
dry_run "pick up the red block" --fail lift:safety           # aborted, no HOME
dry_run "pick up the red block" --fail approach:timeout --log-dir /tmp/runs
```

Kinds: `not_found`, `refused`, `safety`, `error` (a plain `StageError`),
`crash` (an unexpected exception), `miss` (grasp only), `timeout`.

## Run the tests

```bash
cd src/ur7e_orchestrator
python3 -m pytest test            # lint tests need ament_flake8 / ament_pep257 (ROS image)
python3 -m pytest test/test_workflow.py   # plain Python 3.10, nothing else
```

## Writing an adapter

Subclass `Adapters` (`adapters.py`) and override every method:

```python
from ur7e_orchestrator.adapters import Adapters, Detection, NotFound
from ur7e_orchestrator.workflow import Orchestrator

class LabAdapters(Adapters):
    def detect(self, query):
        result = ...  # call the perception node
        if result is None:
            raise NotFound('not_found', f'no {query!r} in view')
        return Detection(capture_id=result.id, confidence=result.score)
    # ... parse, observe, locate, plan, approach, grasp, lift, retreat,
    #     home, release, safety_ok, abort

record = Orchestrator(LabAdapters(), log_dir='runs').run('pick up the red block')
```

The rules the state machine relies on:

- **Report failures by raising.** `Refused` (no motion may follow),
  `NotFound` (nothing or ambiguous), `SafetyAbort` (protective stop, e-stop,
  program stopped), or a plain `StageError(reason, message)` for anything
  else. Keep `reason` short, stable, and snake_case; put the specifics in
  `message`.
- **`grasp()` does not raise on a miss.** It returns
  `GraspResult(holding=False, …)`; the machine then calls `release()`.
- **Motion methods block until the motion is finished** (or has failed).
  The machine starts the next stage the moment a method returns.
- **Threading.** Every method is called on the same worker thread, one at a
  time — create your ROS objects there, or make sure they tolerate it. The
  exception is `abort()`: after a timeout it is called from a different
  thread *while your stuck method is still running*, so it must be safe to
  call concurrently and should return quickly (it gets 10 s).
- **`safety_ok()` returns `(ok, reason)`** and should be cheap; it runs six
  times per pick. When `ok` is False, `reason` becomes the run's reason
  code, so return a stable code such as `protective_stop`.
- **Results that reach the log must be JSON-able.** `plan()`'s summary and
  the `detail` dicts of `Target` and `GraspResult` are written to the log;
  anything JSON cannot encode is stored as its `repr`.
- **Call `orchestrator.close()`** when you are done with it, to stop the
  worker thread.
