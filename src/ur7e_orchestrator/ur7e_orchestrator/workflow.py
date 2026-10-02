"""
The workflow state machine: every pick runs the same stages in the same order.

``Orchestrator.run(utterance)`` walks the stages from ``stages.py``, calling
one ``Adapters`` object for all real work, and returns a ``RunRecord`` that
says how far the run got and why it stopped. Reading order for this file:

1. ``RetryPolicy``, ``StageRecord``, ``RunRecord`` - the data a run produces.
2. ``Orchestrator._drive`` and the two transition methods after it - the
   state machine itself: "this stage ended like that, so go there next".
3. ``Orchestrator._run_stage`` - what happens inside one stage (safety
   check, adapter calls, timeout, record, log line).
4. ``_Worker`` - the one thread adapter calls run on, so a call that hangs
   can be given up on without hanging the orchestrator too.
"""
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
import logging
from pathlib import Path
import queue
import threading
import time
import uuid

from ur7e_orchestrator.adapters import NotFound, Refused, SafetyAbort, StageError
from ur7e_orchestrator.stages import (
    ABORT_TIMEOUT_S,
    HOME_AFTER_FAILURE,
    MOTION_STAGES,
    next_stage,
    resolve_timeouts,
    Stage,
)

_LOG = logging.getLogger(__name__)

# Appended to the run message whenever a failure leaves the arm in place.
_LEFT_IN_PLACE = (
    'The arm was left where it stopped (it may be holding the object): '
    'recover it by hand.'
)


@dataclass(frozen=True)
class RetryPolicy:
    """How many times one run may retry each recoverable failure."""

    reobserve_on_not_found: int = 1   # DETECT found nothing -> OBSERVE again
    regrasp_on_miss: int = 1          # jaws closed on nothing -> pick again


@dataclass
class StageRecord:
    """What happened in one execution of one stage."""

    stage: str          # Stage value, e.g. 'DETECT'
    attempt: int        # 1 for the first time this stage ran in the run
    outcome: str        # 'ok' | 'failed' | 'timeout' | 'skipped'
    reason: str         # 'ok', or the stable code of what went wrong
    duration_s: float
    detail: dict = field(default_factory=dict)   # JSON-able adapter results


@dataclass
class RunRecord:
    """The full account of one ``Orchestrator.run()``."""

    run_id: str         # uuid4 hex; also the name of the log file
    utterance: str
    started: str        # ISO-8601 UTC wall time
    outcome: str = ''   # 'succeeded' | 'refused' | 'failed' | 'aborted'
    reason: str = ''    # stable snake_case code; 'ok' on success
    message: str = ''   # free-form explanation for a human
    stages: list = field(default_factory=list)    # StageRecord, execution order
    retries: dict = field(default_factory=dict)   # retries used, by policy name
    duration_s: float = 0.0

    def to_dict(self):
        """Return the record as plain dicts and lists."""
        return asdict(self)

    def to_json(self, indent=None):
        """Return the record as a JSON string."""
        return json.dumps(self.to_dict(), indent=indent)


class OrchestratorBusy(RuntimeError):
    """
    ``run()`` was refused because the orchestrator cannot take a run now.

    Either another ``run()`` is in progress, or an adapter call from an
    earlier run timed out and has still not returned.
    """


class _StageTimeout(Exception):
    """An adapter call did not return within the stage's time budget."""


class _GraspMissed(StageError):
    """GRASP closed the jaws on nothing; the jaws were opened again."""


class _Job:
    """One call handed to another thread, with a place to leave its result."""

    def __init__(self, label, function, args=()):
        self.label = label
        self._function = function
        self._args = args
        self._done = threading.Event()
        self._result = None
        self._error = None

    @property
    def finished(self):
        """Return True once the call has returned or raised."""
        return self._done.is_set()

    def execute(self):
        """Run the call on the current thread and store what came of it."""
        try:
            self._result = self._function(*self._args)
        except BaseException as error:
            # Carried across to the waiting thread and re-raised there.
            self._error = error
        finally:
            self._done.set()

    def wait(self, timeout_s):
        """Return the call's result, re-raise its error, or time out."""
        if not self._done.wait(timeout_s):
            raise _StageTimeout(self.label)
        if self._error is not None:
            raise self._error
        return self._result


class _Worker:
    """
    The single persistent thread that adapter calls run on.

    A plain function call cannot be interrupted from outside, so the only
    way to enforce a timeout is to make the call somewhere else and stop
    *waiting* for it. One thread, never a pool: the real adapter owns ROS
    objects that must not be touched from two threads.
    """

    def __init__(self):
        self._jobs = queue.Queue()
        self._latest = None
        self._thread = threading.Thread(
            target=self._loop, name='orchestrator-worker', daemon=True)
        self._thread.start()

    @property
    def stuck_in(self):
        """Return the label of the call still running, or '' when free."""
        job = self._latest
        return job.label if job is not None and not job.finished else ''

    def submit(self, job):
        """Queue ``job``; the caller then waits on it with a timeout."""
        self._latest = job
        self._jobs.put(job)

    def stop(self):
        """Ask the thread to exit after the call it is in (if any)."""
        self._jobs.put(None)

    def _loop(self):
        while True:
            job = self._jobs.get()
            if job is None:
                return
            job.execute()


class _Run:
    """Working state of one run: the record plus data passed between stages."""

    def __init__(self, record, log, started):
        self.record = record
        self.log = log              # open JSONL file, or None
        self.started = started      # clock reading at run start
        self.stage = Stage.IDLE     # the stage being executed
        self.deadline = started     # clock reading when that stage times out
        self.attempts = {}          # Stage -> times executed so far
        self.intent = None
        self.detections = {}        # 'pick' / 'place' -> Detection
        self.targets = {}           # 'pick' / 'place' -> Target
        # Set once the run is known to have failed and is only tidying up
        # (courtesy HOME): the (reason, message) to report at the end.
        self.failure = None
        # Set after a grasp miss: where the "get clear" LIFT leads next.
        self.after_lift = None

    def roles(self):
        """Return the (role, query) pairs to detect: pick, then place."""
        pairs = [('pick', self.intent.target_query)]
        if self.intent.place_query:
            pairs.append(('place', self.intent.place_query))
        return pairs

    def end(self, outcome, reason, message):
        """Write the verdict into the record."""
        self.record.outcome = outcome
        self.record.reason = reason
        self.record.message = message


class Orchestrator:
    """
    Run picks through the fixed stage sequence, one run at a time.

    ``timeouts`` overrides ``stages.DEFAULT_TIMEOUTS_S`` per stage (keys
    are ``Stage`` members or their names). ``log_dir`` turns on the JSONL
    run log. ``clock`` measures durations and stage deadlines; tests pass
    a fake one. ``on_event`` is called, on the thread that called
    ``run()``, with each event dict as it is logged.
    """

    def __init__(self, adapters, policy=None, timeouts=None, log_dir=None,
                 clock=time.monotonic, on_event=None, abort_timeout_s=ABORT_TIMEOUT_S):
        self._adapters = adapters
        self._policy = RetryPolicy() if policy is None else policy
        self._timeouts = resolve_timeouts(timeouts)
        self._log_dir = None if log_dir is None else Path(log_dir)
        self._clock = clock
        self.on_event = on_event
        self._abort_timeout_s = abort_timeout_s
        self._run_lock = threading.Lock()
        self._worker = None
        self._closed = False
        self._current_stage = Stage.IDLE
        self._handlers = {
            Stage.IDLE: self._idle,
            Stage.PARSE: self._parse,
            Stage.OBSERVE: self._observe,
            Stage.DETECT: self._detect,
            Stage.LOCATE: self._locate,
            Stage.PLAN: self._plan,
            Stage.APPROACH: self._approach,
            Stage.GRASP: self._grasp,
            Stage.LIFT: self._lift,
            Stage.RETREAT: self._retreat,
            Stage.HOME: self._home,
        }

    # ------------------------------------------------------------------
    # Public surface
    # ------------------------------------------------------------------

    @property
    def current_stage(self):
        """Return the stage being executed (IDLE between runs)."""
        return self._current_stage

    @property
    def busy(self):
        """Return True while ``run()`` would be refused."""
        stuck = self._worker is not None and bool(self._worker.stuck_in)
        return self._run_lock.locked() or stuck

    def run(self, utterance):
        """
        Run one pick for ``utterance`` and return its RunRecord.

        Failures of the pick itself never raise: they come back as the
        record's outcome and reason. ``OrchestratorBusy`` is raised only
        when the run cannot be started at all.
        """
        if not self._run_lock.acquire(blocking=False):
            raise OrchestratorBusy('another run() is in progress; one run at a time')
        try:
            self._claim_worker()
            run = self._begin(utterance)
            try:
                self._drive(run)
            except KeyboardInterrupt:
                # Ctrl-C lands here, on the waiting thread, while the worker
                # may still be moving the arm: stop the motion, close the
                # log, then let the interrupt continue to the caller.
                note = self._abort_motion()
                run.end('aborted', 'interrupted', 'Interrupted by the operator.' + note)
                raise
            finally:
                self._finish(run)
            return run.record
        finally:
            self._current_stage = Stage.IDLE
            self._run_lock.release()

    def close(self):
        """Stop the worker thread; the orchestrator cannot run afterwards."""
        self._closed = True
        if self._worker is not None:
            self._worker.stop()

    # ------------------------------------------------------------------
    # The state machine
    # ------------------------------------------------------------------

    def _drive(self, run):
        """Walk the stages until the run ends, following the transitions."""
        stage = Stage.IDLE
        while stage is not None:
            try:
                self._run_stage(run, stage)
            except _StageTimeout as error:
                limit = self._timeouts[stage]
                stage = self._abort_run(run, 'stage_timeout', (
                    f'{stage.value} did not finish within its {limit:g} s timeout '
                    f'(still waiting on {error}).'))
            except SafetyAbort as error:
                stage = self._abort_run(run, error.reason, f'{stage.value}: {error.message}')
            except Exception as error:
                stage = self._after_failure(run, stage, error)
            else:
                stage = self._after_success(run, stage)

    def _after_success(self, run, stage):
        """Return the stage to run after ``stage`` succeeded (None = done)."""
        if stage is Stage.HOME:
            if run.failure is None:
                run.end('succeeded', 'ok', 'All stages completed.')
            else:
                reason, message = run.failure
                run.end('failed', reason, f'{message} The arm was sent home.')
            return None
        if stage is Stage.LIFT and run.after_lift is not None:
            # This LIFT was the "get clear" move after a grasp miss.
            detour, run.after_lift = run.after_lift, None
            return detour
        return next_stage(stage)

    def _after_failure(self, run, stage, error):
        """Return the stage to run after ``stage`` failed (None = done)."""
        _, reason, message = _classify(error)

        if run.failure is not None:
            # Already failed and tidying up, and the tidy-up failed too.
            # Keep the first failure as the reason; stop commanding motion.
            first_reason, first_message = run.failure
            run.end('failed', first_reason, (
                f'{first_message} Then {stage.value} failed during recovery '
                f'({reason}: {message}). {_LEFT_IN_PLACE}'))
            return None

        if isinstance(error, Refused):
            # Wherever it is raised: no motion may follow, not even HOME.
            run.end('refused', reason, message)
            return None

        if isinstance(error, NotFound) and stage is Stage.DETECT:
            if self._take_retry(run, 'reobserve_on_not_found'):
                return Stage.OBSERVE

        if isinstance(error, _GraspMissed):
            # The GRASP stage already opened the jaws. Lift clear first,
            # then either try the whole pick again or give up and go home.
            if self._take_retry(run, 'regrasp_on_miss'):
                run.after_lift = Stage.OBSERVE
            else:
                run.failure = (reason, message)
                run.after_lift = Stage.HOME
            return Stage.LIFT

        if stage in HOME_AFTER_FAILURE:
            run.failure = (reason, message)
            return Stage.HOME

        if stage is Stage.PARSE:
            run.end('failed', reason, f'{message} Nothing was moved.')
        else:
            run.end('failed', reason, f'{message} {_LEFT_IN_PLACE}')
        return None

    def _abort_run(self, run, reason, message):
        """End the run as aborted: stop motion, command nothing further."""
        note = self._abort_motion()
        if run.failure is not None:
            first_reason, first_message = run.failure
            message += f' (While recovering from {first_reason}: {first_message})'
        run.end('aborted', reason, message + note)
        return None

    def _take_retry(self, run, name):
        """Use up one retry of kind ``name``; return False if none are left."""
        used = run.record.retries[name]
        if used >= getattr(self._policy, name):
            return False
        run.record.retries[name] = used + 1
        return True

    # ------------------------------------------------------------------
    # One stage
    # ------------------------------------------------------------------

    def _run_stage(self, run, stage):
        """
        Execute ``stage`` once and record how it went.

        The StageRecord and its log event are written whether the stage
        returns or raises; the exception then continues to ``_drive``,
        which decides where the run goes next.
        """
        attempt = run.attempts.get(stage, 0) + 1
        run.attempts[stage] = attempt
        run.stage = stage
        self._current_stage = stage
        started = self._clock()
        run.deadline = started + self._timeouts.get(stage, 0.0)
        outcome, reason, detail = 'ok', 'ok', {}
        try:
            if stage in MOTION_STAGES:
                self._check_safety(run)
            self._handlers[stage](run, detail)
        except BaseException as error:
            outcome, reason, detail['error'] = _classify(error)
            raise
        finally:
            record = StageRecord(
                stage=stage.value,
                attempt=attempt,
                outcome=outcome,
                reason=reason,
                duration_s=round(self._clock() - started, 3),
                detail=_jsonable(detail),
            )
            run.record.stages.append(record)
            self._emit(run, 'stage_end', **asdict(record))

    def _check_safety(self, run):
        """Ask the adapter whether it is safe to move; raise SafetyAbort if not."""
        ok, why = self._call(run, self._adapters.safety_ok)
        if not ok:
            raise SafetyAbort(why or 'safety_not_ok', (
                f'safety check failed before moving ({why or "no reason given"})'))

    def _call(self, run, method, *args):
        """Run one adapter method on the worker, within the stage's budget."""
        label = f'{run.stage.value}/{getattr(method, "__name__", "call")}()'
        remaining = run.deadline - self._clock()
        if remaining <= 0:
            # The stage's earlier calls used the whole budget: do not start
            # another one that nobody would wait for.
            raise _StageTimeout(label)
        job = _Job(label, method, args)
        self._worker.submit(job)
        return job.wait(remaining)

    # Stage handlers: each makes the stage's adapter call(s), keeps what
    # later stages need on ``run``, and notes JSON-able facts in ``detail``
    # (filled in as it goes, so a failure still shows what was done).

    def _idle(self, run, detail):
        """Do nothing: IDLE is recorded as the state every run starts from."""

    def _parse(self, run, detail):
        intent = self._call(run, self._adapters.parse, run.record.utterance)
        detail.update(asdict(intent))
        # Entry check for everything downstream: the stages after PARSE
        # assume one of the two known actions and a consistent place target.
        has_place = bool(intent.place_query)
        if intent.action not in ('pick', 'pick_and_place') or not intent.target_query:
            raise StageError('invalid_intent', f'The parser returned an unusable {intent!r}.')
        if has_place != (intent.action == 'pick_and_place'):
            raise StageError('invalid_intent', (
                f'action {intent.action!r} does not match place_query '
                f'{intent.place_query!r}.'))
        run.intent = intent

    def _observe(self, run, detail):
        self._call(run, self._adapters.observe)

    def _detect(self, run, detail):
        run.detections = {}
        for role, query in run.roles():
            detail[role] = {'query': query}
            detection = self._call(run, self._adapters.detect, query)
            run.detections[role] = detection
            detail[role].update(asdict(detection))

    def _locate(self, run, detail):
        run.targets = {}
        for role, detection in run.detections.items():
            target = self._call(run, self._adapters.locate, detection)
            run.targets[role] = target
            detail[role] = asdict(target)

    def _plan(self, run, detail):
        summary = self._call(
            run, self._adapters.plan,
            run.intent, run.targets['pick'], run.targets.get('place'))
        detail.update(summary or {})

    def _approach(self, run, detail):
        self._call(run, self._adapters.approach)

    def _grasp(self, run, detail):
        result = self._call(run, self._adapters.grasp)
        detail.update(asdict(result))
        if not result.holding:
            # Open the jaws again here, inside GRASP, so that every later
            # decision can rely on "after a miss the jaws are empty".
            self._call(run, self._adapters.release)
            detail['released'] = True
            raise _GraspMissed('grasp_missed', (
                f'The jaws closed on nothing (width {result.width_mm:g} mm).'))

    def _lift(self, run, detail):
        self._call(run, self._adapters.lift)

    def _retreat(self, run, detail):
        self._call(run, self._adapters.retreat)

    def _home(self, run, detail):
        self._call(run, self._adapters.home)

    # ------------------------------------------------------------------
    # Run bookkeeping: worker, abort, log
    # ------------------------------------------------------------------

    def _claim_worker(self):
        """Start the worker if needed; refuse the run if it is still stuck."""
        if self._closed:
            raise RuntimeError('this Orchestrator was closed; create a new one')
        if self._worker is None:
            self._worker = _Worker()
        stuck = self._worker.stuck_in
        if stuck:
            raise OrchestratorBusy(
                f'the worker thread is still inside {stuck} from an earlier run '
                'that was given up on; the adapter is not safe to call until it '
                'returns. Wait for it, or restart the process.')

    def _abort_motion(self):
        """Call ``abort()`` best effort; return a note for the run message."""
        job = _Job('abort()', self._adapters.abort)
        if self._worker.stuck_in:
            # The worker is the thing that is stuck, so abort() cannot run
            # there; this is the one adapter call made from another thread.
            threading.Thread(
                target=job.execute, name='orchestrator-abort', daemon=True).start()
        else:
            self._worker.submit(job)
        try:
            job.wait(self._abort_timeout_s)
        except _StageTimeout:
            return f' abort() did not return within {self._abort_timeout_s:g} s.'
        except Exception as error:
            return f' abort() itself raised {error!r}.'
        return ' abort() was called.'

    def _begin(self, utterance):
        """Create the run's record and log file and announce the run."""
        record = RunRecord(
            run_id=uuid.uuid4().hex,
            utterance=utterance,
            started=_utc_now(),
            retries={name: 0 for name in asdict(self._policy)},
        )
        log = None
        if self._log_dir is not None:
            self._log_dir.mkdir(parents=True, exist_ok=True)
            path = self._log_dir / f'{record.run_id}.jsonl'
            log = open(path, 'a', encoding='utf-8')
        run = _Run(record, log, self._clock())
        self._emit(
            run, 'run_start',
            utterance=utterance,
            started=record.started,
            policy=asdict(self._policy),
            timeouts_s={stage.value: limit for stage, limit in self._timeouts.items()},
        )
        return run

    def _finish(self, run):
        """Stamp the duration, log the verdict, and close the log file."""
        record = run.record
        if not record.outcome:
            # Only reachable through a bug in this file; say so in the log
            # rather than leave a run without a verdict.
            run.end('failed', 'unexpected_error', 'The orchestrator stopped without a verdict.')
        record.duration_s = round(self._clock() - run.started, 3)
        self._emit(
            run, 'run_end',
            outcome=record.outcome,
            reason=record.reason,
            message=record.message,
            retries=dict(record.retries),
            duration_s=record.duration_s,
        )
        if run.log is not None:
            run.log.close()
            run.log = None

    def _emit(self, run, name, **fields):
        """
        Log one event as a JSON line and hand it to ``on_event``.

        Each line is flushed as it is written, so a run that crashes still
        leaves a readable log up to its last finished stage. Neither a full
        disk nor a broken callback may stop a run that is moving hardware,
        so their errors are reported through ``logging`` and the run goes on.
        """
        event = {'event': name, 'run_id': run.record.run_id, 't': _utc_now(), **fields}
        if run.log is not None:
            try:
                run.log.write(json.dumps(event, default=repr) + '\n')
                run.log.flush()
            except OSError:
                _LOG.exception('could not write the run log; the run continues')
        if self.on_event is not None:
            try:
                self.on_event(event)
            except Exception:
                _LOG.exception('on_event callback raised; the run continues')


def _classify(error):
    """Return ``(stage_outcome, reason, message)`` for an exception."""
    if isinstance(error, _StageTimeout):
        return 'timeout', 'stage_timeout', f'no answer from {error}'
    if isinstance(error, StageError):
        return 'failed', error.reason, error.message
    if isinstance(error, KeyboardInterrupt):
        return 'failed', 'interrupted', 'interrupted by the operator'
    return 'failed', 'unexpected_error', repr(error)


def _jsonable(detail):
    """Return ``detail`` reduced to JSON types (tuples to lists, rest to repr)."""
    return json.loads(json.dumps(detail, default=repr))


def _utc_now():
    """Return the current wall-clock time as an ISO-8601 UTC string."""
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')
