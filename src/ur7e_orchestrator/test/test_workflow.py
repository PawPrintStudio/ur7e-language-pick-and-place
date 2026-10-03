"""
Dry-run tests: the workflow against MockAdapters, no ROS and no robot.

Each test scripts one situation (a refusal, a missed grasp, a protective
stop) and checks two things: the RunRecord the orchestrator reports, and
the exact adapter calls it made - because on the real arm every one of
those calls is a motion.
"""
from datetime import datetime, timedelta
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest

from ur7e_orchestrator.adapters import (
    Adapters,
    GraspResult,
    Intent,
    NotFound,
    Refused,
    SafetyAbort,
    StageError,
    Target,
)
from ur7e_orchestrator.cli import main as cli_main
from ur7e_orchestrator.mock import MockAdapters
from ur7e_orchestrator.stages import Stage, STAGE_ORDER
from ur7e_orchestrator.workflow import Orchestrator, OrchestratorBusy, RetryPolicy

PICK = 'pick up the red block'
PICK_AND_PLACE = 'pick up the red block and put it on the blue plate'

# The adapter calls of one clean pick, in order (safety checks left out).
HAPPY_ACTIONS = [
    'parse', 'observe', 'detect', 'locate', 'plan',
    'approach', 'grasp', 'lift', 'retreat', 'home',
]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def make():
    """Build (orchestrator, mock) pairs and stop their worker threads after."""
    built = []

    def factory(script=None, **kwargs):
        mock = MockAdapters(script)
        orchestrator = Orchestrator(mock, **kwargs)
        built.append(orchestrator)
        return orchestrator, mock

    yield factory
    for orchestrator in built:
        orchestrator.close()


def stage_names(record):
    """Return the stage names of a record in execution order."""
    return [stage.stage for stage in record.stages]


def actions(mock):
    """Return the adapter calls without the interleaved safety checks."""
    return [name for name in mock.calls if name != 'safety_ok']


def wait_until_free(orchestrator, limit_s=5.0):
    """Block until a stuck worker has returned (or fail the test)."""
    deadline = time.monotonic() + limit_s
    while orchestrator.busy:
        assert time.monotonic() < deadline, 'worker never came back'
        time.sleep(0.01)


# -- 1. happy path ---------------------------------------------------------

def test_happy_path_traverses_all_eleven_stages_in_order(make):
    """IDLE is an explicit first StageRecord, then the ten working stages."""
    orchestrator, mock = make()
    record = orchestrator.run(PICK)

    assert record.outcome == 'succeeded'
    assert record.reason == 'ok'
    assert stage_names(record) == [stage.value for stage in STAGE_ORDER]
    assert len(record.stages) == 11
    assert all(stage.outcome == 'ok' and stage.attempt == 1 for stage in record.stages)
    assert actions(mock) == HAPPY_ACTIONS
    assert record.retries == {'reobserve_on_not_found': 0, 'regrasp_on_miss': 0}
    assert orchestrator.current_stage is Stage.IDLE


def test_run_record_identifies_and_serializes_the_run(make):
    """A record carries a uuid4 hex id, a UTC start time, and round-trips JSON."""
    orchestrator, _ = make()
    first, second = orchestrator.run(PICK), orchestrator.run(PICK)

    assert len(first.run_id) == 32 and int(first.run_id, 16) >= 0
    assert first.run_id != second.run_id
    assert datetime.fromisoformat(first.started).utcoffset() == timedelta(0)
    decoded = json.loads(first.to_json())
    assert decoded['utterance'] == PICK
    assert decoded['stages'][1]['stage'] == 'PARSE'
    assert decoded['stages'][1]['detail']['target_query'] == 'red block'


def test_safety_is_checked_before_each_motion_stage_and_only_those(make):
    """Six stages move hardware; each is preceded by exactly one safety_ok."""
    orchestrator, mock = make()
    orchestrator.run(PICK)

    checked = [mock.calls[i + 1] for i, name in enumerate(mock.calls) if name == 'safety_ok']
    assert checked == ['observe', 'approach', 'grasp', 'lift', 'retreat', 'home']


# -- 2. pick and place -----------------------------------------------------

def test_pick_and_place_detects_and_locates_twice_and_plans_with_place(make):
    """DETECT and LOCATE each make two adapter calls inside one stage."""
    pick_at = Target(xyz=(0.4, 0.1, 0.02), yaw=0.0)
    place_at = Target(xyz=(0.5, -0.2, 0.0), yaw=1.57)
    orchestrator, mock = make({'locate': [pick_at, place_at]})
    record = orchestrator.run(PICK_AND_PLACE)

    assert record.outcome == 'succeeded'
    assert stage_names(record) == [stage.value for stage in STAGE_ORDER]
    assert [args for name, args in mock.call_log if name == 'detect'] == [
        ('red block',), ('blue plate',)]
    assert mock.calls.count('locate') == 2
    (intent, target, place), = [args for name, args in mock.call_log if name == 'plan']
    assert intent.action == 'pick_and_place'
    assert (target, place) == (pick_at, place_at)


def test_plain_pick_plans_with_no_place_target(make):
    """Without a place query, plan() receives place=None."""
    orchestrator, mock = make()
    orchestrator.run(PICK)

    (_, _, place), = [args for name, args in mock.call_log if name == 'plan']
    assert place is None
    assert mock.calls.count('detect') == 1


def test_not_found_on_the_place_target_fails_detect(make):
    """A not-found on either of the two detect calls is DETECT's not-found."""
    lost = NotFound('not_found', 'no blue plate')
    orchestrator, mock = make(
        {'detect': [None, lost]}, policy=RetryPolicy(reobserve_on_not_found=0))
    record = orchestrator.run(PICK_AND_PLACE)

    assert (record.outcome, record.reason) == ('failed', 'not_found')
    detect = record.stages[stage_names(record).index('DETECT')]
    assert detect.outcome == 'failed'
    assert detect.detail['pick']['confidence'] == 0.9
    assert detect.detail['place'] == {'query': 'blue plate'}


# -- 3. refusal ------------------------------------------------------------

def test_refusal_calls_nothing_but_parse(make):
    """A refused utterance must not cause any motion, not even a safety check."""
    orchestrator, mock = make()
    record = orchestrator.run('do a little dance')

    assert (record.outcome, record.reason) == ('refused', 'not_a_pick')
    assert mock.calls == ['parse']
    assert stage_names(record) == ['IDLE', 'PARSE']
    assert record.stages[-1].outcome == 'failed'


def test_refusal_after_parse_also_stops_all_motion(make):
    """Refused from a later stage (operator declined) skips even HOME."""
    orchestrator, mock = make({'plan': [Refused('operator_declined', 'operator said no')]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('refused', 'operator_declined')
    assert actions(mock) == ['parse', 'observe', 'detect', 'locate', 'plan']


def test_unusable_intent_fails_parse_without_motion(make):
    """PARSE checks the Intent before any stage acts on it."""
    orchestrator, mock = make({'parse': [Intent(action='wave', target_query='hand')]})
    record = orchestrator.run('wave')

    assert (record.outcome, record.reason) == ('failed', 'invalid_intent')
    assert mock.calls == ['parse']


# -- 4. not found -> re-observe -------------------------------------------

def test_not_found_once_reobserves_and_succeeds(make):
    """One NotFound in DETECT sends the run back to OBSERVE, once."""
    orchestrator, mock = make({'detect': [NotFound('not_found', 'nothing there'), None]})
    record = orchestrator.run(PICK)

    assert record.outcome == 'succeeded'
    assert record.retries['reobserve_on_not_found'] == 1
    assert stage_names(record) == [
        'IDLE', 'PARSE', 'OBSERVE', 'DETECT', 'OBSERVE', 'DETECT',
        'LOCATE', 'PLAN', 'APPROACH', 'GRASP', 'LIFT', 'RETREAT', 'HOME']
    first_detect, second_detect = [s for s in record.stages if s.stage == 'DETECT']
    assert (first_detect.outcome, first_detect.reason) == ('failed', 'not_found')
    assert first_detect.detail['error'] == 'nothing there'
    assert (second_detect.outcome, second_detect.attempt) == ('ok', 2)


def test_not_found_twice_fails_and_attempts_home(make):
    """With the retry used up the run fails, and the arm is sent home."""
    lost = NotFound('not_found', 'nothing there')
    orchestrator, mock = make({'detect': [lost, lost]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('failed', 'not_found')
    assert record.retries['reobserve_on_not_found'] == 1
    assert actions(mock) == ['parse', 'observe', 'detect', 'observe', 'detect', 'home']
    assert stage_names(record)[-2:] == ['DETECT', 'HOME']
    assert record.stages[-1].outcome == 'ok'


def test_retry_policy_zero_disables_reobserve(make):
    """RetryPolicy(reobserve_on_not_found=0) fails on the first NotFound."""
    orchestrator, mock = make(
        {'detect': [NotFound('not_found', 'nothing there')]},
        policy=RetryPolicy(reobserve_on_not_found=0))
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('failed', 'not_found')
    assert actions(mock) == ['parse', 'observe', 'detect', 'home']


# -- 5. grasp miss -> re-grasp --------------------------------------------

def test_grasp_miss_once_releases_lifts_and_picks_again(make):
    """A miss opens the jaws, lifts clear, and restarts from OBSERVE."""
    orchestrator, mock = make({'grasp': [GraspResult(False, 0.0), None]})
    record = orchestrator.run(PICK)

    assert record.outcome == 'succeeded'
    assert record.retries == {'reobserve_on_not_found': 0, 'regrasp_on_miss': 1}
    assert actions(mock) == [
        'parse', 'observe', 'detect', 'locate', 'plan', 'approach', 'grasp',
        'release', 'lift',
        'observe', 'detect', 'locate', 'plan', 'approach', 'grasp',
        'lift', 'retreat', 'home']
    missed = record.stages[stage_names(record).index('GRASP')]
    assert (missed.outcome, missed.reason) == ('failed', 'grasp_missed')
    assert missed.detail['holding'] is False and missed.detail['released'] is True


def test_grasp_miss_twice_fails_after_release_and_home(make):
    """With the re-grasp used up: release, lift clear, home, then fail."""
    miss = GraspResult(False, 0.0)
    orchestrator, mock = make({'grasp': [miss, miss]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('failed', 'grasp_missed')
    assert record.retries['regrasp_on_miss'] == 1
    assert mock.calls.count('grasp') == 2
    assert actions(mock)[-4:] == ['grasp', 'release', 'lift', 'home']
    assert 'retreat' not in mock.calls


# -- 6. and 7. safety abort ------------------------------------------------

def test_safety_abort_in_lift_aborts_without_retry_or_further_motion(make):
    """A protective stop ends the run on the spot: abort(), then nothing."""
    orchestrator, mock = make({'lift': [SafetyAbort('protective_stop', 'robot stopped')]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('aborted', 'protective_stop')
    assert actions(mock) == HAPPY_ACTIONS[:8] + ['abort']
    assert stage_names(record)[-1] == 'LIFT'
    assert record.retries == {'reobserve_on_not_found': 0, 'regrasp_on_miss': 0}


def test_safety_not_ok_before_approach_aborts_without_approaching(make):
    """The second safety check guards APPROACH; False there aborts the run."""
    orchestrator, mock = make({'safety_ok': [None, (False, 'protective_stop')]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('aborted', 'protective_stop')
    assert 'approach' not in mock.calls
    assert mock.calls[-2:] == ['safety_ok', 'abort']
    approach = record.stages[-1]
    assert (approach.stage, approach.outcome) == ('APPROACH', 'failed')


def test_safety_abort_during_courtesy_home_aborts(make):
    """Even while homing after a failure, a safety abort wins."""
    orchestrator, mock = make({
        'plan': [StageError('unreachable', 'target is outside the workspace')],
        'home': [SafetyAbort('emergency_stop', 'e-stop pressed')],
    })
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('aborted', 'emergency_stop')
    assert 'unreachable' in record.message
    assert actions(mock)[-2:] == ['home', 'abort']


# -- 8. stage timeout ------------------------------------------------------

def test_stage_timeout_aborts_and_refuses_runs_until_the_worker_returns(make):
    """A hung adapter call is abandoned; the orchestrator stays locked on it."""
    orchestrator, mock = make(
        {'approach': [('sleep', 0.6)]}, timeouts={Stage.APPROACH: 0.05})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('aborted', 'stage_timeout')
    assert record.stages[-1].stage == 'APPROACH'
    assert record.stages[-1].outcome == 'timeout'
    assert actions(mock) == HAPPY_ACTIONS[:6] + ['abort']

    # The worker is still asleep inside approach(): no new run may start.
    assert orchestrator.busy
    with pytest.raises(OrchestratorBusy, match='APPROACH/approach'):
        orchestrator.run(PICK)
    assert actions(mock) == HAPPY_ACTIONS[:6] + ['abort']

    # Once that call returns, the orchestrator is usable again.
    wait_until_free(orchestrator)
    assert orchestrator.run(PICK).outcome == 'succeeded'


def test_stage_budget_is_shared_by_the_calls_of_one_stage(make):
    """The timeout covers the whole stage, not each adapter call in it."""
    now = [0.0]

    def slow_detect(query):
        now[0] += 70.0   # the first detect() "takes" 70 s of DETECT's 60 s

    orchestrator, mock = make({'detect': [slow_detect]}, clock=lambda: now[0])
    record = orchestrator.run(PICK_AND_PLACE)

    assert (record.outcome, record.reason) == ('aborted', 'stage_timeout')
    assert mock.calls.count('detect') == 1
    assert mock.calls[-1] == 'abort'
    assert record.stages[-1].duration_s == 70.0


def test_a_second_run_is_refused_while_one_is_in_progress(make):
    """One run() at a time, also when called from another thread."""
    entered, leave = threading.Event(), threading.Event()

    def hold(*_):
        entered.set()
        leave.wait(5.0)

    orchestrator, _ = make({'observe': [hold]})
    records = []
    runner = threading.Thread(target=lambda: records.append(orchestrator.run(PICK)))
    runner.start()
    try:
        assert entered.wait(5.0)
        with pytest.raises(OrchestratorBusy, match='in progress'):
            orchestrator.run(PICK)
    finally:
        leave.set()
        runner.join(5.0)
    assert records[0].outcome == 'succeeded'


def test_ctrl_c_stops_motion_and_closes_the_log_before_propagating(make):
    """Ctrl-C during a stage calls abort() and still writes the run_end event."""
    if threading.current_thread() is not threading.main_thread():
        pytest.skip('SIGINT is only delivered to the main thread')

    def ctrl_c(*_):
        signal.pthread_kill(threading.main_thread().ident, signal.SIGINT)
        time.sleep(0.3)   # the "motion" is still running when Ctrl-C lands

    events = []
    orchestrator, mock = make({'approach': [ctrl_c]}, on_event=events.append)
    with pytest.raises(KeyboardInterrupt):
        orchestrator.run(PICK)

    assert actions(mock) == HAPPY_ACTIONS[:6] + ['abort']
    assert events[-1]['event'] == 'run_end'
    assert (events[-1]['outcome'], events[-1]['reason']) == ('aborted', 'interrupted')
    assert events[-2]['stage'] == 'APPROACH' and events[-2]['reason'] == 'interrupted'
    wait_until_free(orchestrator)


# -- 9. where a failure leaves the arm ------------------------------------

@pytest.mark.parametrize('method', ['grasp', 'lift', 'retreat'])
def test_failure_at_or_after_grasp_does_not_move_the_arm(make, method):
    """The arm may be holding something: no HOME, and the message says so."""
    orchestrator, mock = make({method: [StageError('motion_failed', 'controller rejected')]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('failed', 'motion_failed')
    assert 'home' not in mock.calls
    assert actions(mock)[-1] == method
    assert 'left where it stopped' in record.message


def test_unexpected_exception_is_reported_with_its_repr(make):
    """A bug in an adapter becomes unexpected_error, not a crash of run()."""
    orchestrator, mock = make({'lift': [ZeroDivisionError('boom')]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('failed', 'unexpected_error')
    assert "ZeroDivisionError('boom')" in record.message
    assert 'home' not in mock.calls


def test_failure_before_grasp_attempts_home(make):
    """Before GRASP the jaws are empty, so the arm is sent home."""
    orchestrator, mock = make({'plan': [StageError('unreachable', 'outside the workspace')]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('failed', 'unreachable')
    assert actions(mock) == ['parse', 'observe', 'detect', 'locate', 'plan', 'home']
    assert stage_names(record)[-2:] == ['PLAN', 'HOME']


def test_failure_in_parse_does_not_home(make):
    """Nothing has moved yet, so a parser fault must not start a motion."""
    orchestrator, mock = make({'parse': [StageError('parser_unavailable', 'no model')]})
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('failed', 'parser_unavailable')
    assert mock.calls == ['parse']


def test_failed_courtesy_home_keeps_the_original_reason(make):
    """If HOME fails too, the run still reports why it failed first."""
    orchestrator, mock = make({
        'plan': [StageError('unreachable', 'outside the workspace')],
        'home': [StageError('motion_failed', 'controller rejected')],
    })
    record = orchestrator.run(PICK)

    assert (record.outcome, record.reason) == ('failed', 'unreachable')
    assert 'motion_failed' in record.message
    assert record.stages[-1].stage == 'HOME' and record.stages[-1].outcome == 'failed'


def test_unimplemented_adapter_fails_instead_of_raising():
    """The Adapters base class raises NotImplementedError from every method."""
    orchestrator = Orchestrator(Adapters())
    try:
        record = orchestrator.run(PICK)
    finally:
        orchestrator.close()

    assert (record.outcome, record.reason) == ('failed', 'unexpected_error')
    assert 'NotImplementedError' in record.message


# -- 10. structured log ----------------------------------------------------

def test_jsonl_log_has_start_one_line_per_stage_and_end(make, tmp_path):
    """Every line parses; the callback sees the same events as the file."""
    seen = []
    orchestrator, _ = make(
        {'detect': [NotFound('not_found', 'nothing there'), None]},
        log_dir=tmp_path / 'runs', on_event=seen.append)
    record = orchestrator.run(PICK)

    lines = (tmp_path / 'runs' / f'{record.run_id}.jsonl').read_text().splitlines()
    events = [json.loads(line) for line in lines]
    assert events == json.loads(json.dumps(seen))
    assert [event['event'] for event in events] == (
        ['run_start'] + ['stage_end'] * len(record.stages) + ['run_end'])
    assert all(event['run_id'] == record.run_id for event in events)
    assert [event['stage'] for event in events[1:-1]] == stage_names(record)
    assert events[0]['utterance'] == PICK
    assert events[-1]['outcome'] == 'succeeded'
    assert events[-1]['retries'] == {'reobserve_on_not_found': 1, 'regrasp_on_miss': 0}


def test_each_run_gets_its_own_log_file(make, tmp_path):
    """The log file is named after the run id, one file per run."""
    orchestrator, _ = make(log_dir=tmp_path)
    first, second = orchestrator.run(PICK), orchestrator.run('sing')

    assert sorted(path.name for path in tmp_path.iterdir()) == sorted(
        [f'{first.run_id}.jsonl', f'{second.run_id}.jsonl'])
    refused = (tmp_path / f'{second.run_id}.jsonl').read_text().splitlines()
    assert json.loads(refused[-1])['outcome'] == 'refused'


def test_broken_on_event_callback_does_not_stop_the_run(make):
    """A console that fails to print must not strand the arm mid-run."""
    def broken(event):
        raise ValueError('console went away')

    orchestrator, _ = make(on_event=broken)
    assert orchestrator.run(PICK).outcome == 'succeeded'


# -- 11. CLI ---------------------------------------------------------------

def test_cli_exits_zero_on_the_happy_path(capsys, tmp_path):
    """The dry run prints stage lines, then the record as JSON."""
    assert cli_main([PICK, '--log-dir', str(tmp_path)]) == 0

    out = capsys.readouterr().out
    record = json.loads(out[out.index('\n{') + 1:])
    assert record['outcome'] == 'succeeded'
    assert [stage['stage'] for stage in record['stages']] == [s.value for s in STAGE_ORDER]
    assert (tmp_path / f'{record["run_id"]}.jsonl').exists()


def test_cli_exits_nonzero_when_detect_fails_twice(capsys):
    """Two injected not-founds use up the single re-observe."""
    code = cli_main([PICK, '--fail', 'detect:not_found', '--fail', 'detect:not_found'])

    out = capsys.readouterr().out
    assert code != 0
    assert json.loads(out[out.index('\n{') + 1:])['reason'] == 'not_found'


def test_cli_one_injected_not_found_is_retried_and_succeeds(capsys):
    """A single not-found is within the retry policy."""
    assert cli_main([PICK, '--fail', 'detect:not_found']) == 0
    assert 'not_found' in capsys.readouterr().out


def test_cli_rejects_an_unknown_failure_spec(capsys):
    """A typo in --fail is a usage error, not a silently clean run."""
    with pytest.raises(SystemExit) as exit_info:
        cli_main([PICK, '--fail', 'detect:explode'])
    assert exit_info.value.code == 2
    assert 'explode' in capsys.readouterr().err


def test_cli_process_exit_codes():
    """The real process exit code is 0 only for a succeeded run."""
    command = [sys.executable, '-m', 'ur7e_orchestrator.cli']
    happy = subprocess.run(
        command + [PICK], cwd=PACKAGE_ROOT, capture_output=True, text=True, timeout=60)
    failing = subprocess.run(
        command + [PICK, '--fail', 'detect:not_found', '--fail', 'detect:not_found'],
        cwd=PACKAGE_ROOT, capture_output=True, text=True, timeout=60)
    timed_out = subprocess.run(
        command + [PICK, '--fail', 'approach:timeout'],
        cwd=PACKAGE_ROOT, capture_output=True, text=True, timeout=60)

    assert happy.returncode == 0, happy.stderr
    assert failing.returncode == 1, failing.stderr
    assert timed_out.returncode == 1, timed_out.stderr
    assert '"reason": "stage_timeout"' in timed_out.stdout
