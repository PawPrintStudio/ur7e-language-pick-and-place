"""
Dry run: push one utterance through the workflow against the mock adapters.

No robot, no ROS graph. Use it to see what a run looks like stage by stage,
and to watch the retry policy react to an injected failure, for example::

    dry_run "pick up the red block" --fail detect:not_found
"""
import argparse
import sys

from ur7e_orchestrator.adapters import (
    GraspResult,
    NotFound,
    Refused,
    SafetyAbort,
    StageError,
)
from ur7e_orchestrator.mock import MockAdapters
from ur7e_orchestrator.stages import as_stage, DEFAULT_TIMEOUTS_S
from ur7e_orchestrator.workflow import Orchestrator

# What ``--fail STAGE:KIND`` injects into the next call of that stage's
# adapter method. Each value builds a fresh MockAdapters script entry.
FAILURE_KINDS = {
    'not_found': lambda: NotFound('not_found', 'injected: the detector found nothing'),
    'refused': lambda: Refused('refused', 'injected: refused'),
    'safety': lambda: SafetyAbort('protective_stop', 'injected: protective stop'),
    'error': lambda: StageError('injected_error', 'injected: the stage failed'),
    'crash': lambda: RuntimeError('injected: unexpected exception'),
    'miss': lambda: GraspResult(holding=False, width_mm=0.0),
    'timeout': lambda: ('sleep', _TIMEOUT_SLEEP_S),
}

# A 'timeout' failure shortens the stage's budget to this and sleeps longer.
_TIMEOUT_BUDGET_S = 0.2
_TIMEOUT_SLEEP_S = 1.0


def build_script(fail_specs):
    """Turn ``['detect:not_found', ...]`` into a mock script and timeouts."""
    script, timeouts = {}, {}
    for spec in fail_specs:
        stage_name, _, kind = spec.partition(':')
        stage = as_stage(stage_name)
        if stage not in DEFAULT_TIMEOUTS_S:
            raise ValueError(f'{stage.value} makes no adapter call; nothing to fail')
        if kind not in FAILURE_KINDS:
            raise ValueError(f'unknown failure kind {kind!r} in {spec!r}')
        if kind == 'miss' and stage.value != 'GRASP':
            raise ValueError("'miss' only makes sense for the grasp stage")
        if kind == 'timeout':
            timeouts[stage] = _TIMEOUT_BUDGET_S
        # Stage FOO is served by the adapter method foo().
        script.setdefault(stage.value.lower(), []).append(FAILURE_KINDS[kind]())
    return script, timeouts


def print_event(event):
    """Print one workflow event as one line."""
    if event['event'] == 'run_start':
        print(f'run {event["run_id"]}: {event["utterance"]!r}')
    elif event['event'] == 'stage_end':
        line = (f'  {event["stage"]:<8} #{event["attempt"]} {event["outcome"]:<7} '
                f'{event["duration_s"]:.3f} s')
        if event['outcome'] != 'ok':
            line += f'  {event["reason"]}: {event["detail"].get("error", "")}'
        print(line)
    elif event['event'] == 'run_end':
        print(f'=> {event["outcome"]} ({event["reason"]}) in {event["duration_s"]:.3f} s')


def main(argv=None):
    """Run the dry run; return 0 only when the run succeeded."""
    parser = argparse.ArgumentParser(
        prog='dry_run',
        description='Run the pick workflow against mock adapters (no robot).')
    parser.add_argument('utterance', help='e.g. "pick up the red block"')
    parser.add_argument(
        '--log-dir', metavar='DIR',
        help='write the JSONL run log to DIR/<run_id>.jsonl')
    parser.add_argument(
        '--fail', metavar='STAGE:KIND', action='append', default=[],
        help='make the next call of a stage fail; repeat to fail later calls '
             'too. KIND is one of: ' + ', '.join(FAILURE_KINDS))
    args = parser.parse_args(argv)

    try:
        script, timeouts = build_script(args.fail)
    except ValueError as error:
        parser.error(str(error))

    orchestrator = Orchestrator(
        MockAdapters(script), timeouts=timeouts, log_dir=args.log_dir,
        on_event=print_event)
    try:
        record = orchestrator.run(args.utterance)
    finally:
        orchestrator.close()
    print(record.to_json(indent=2))
    return 0 if record.outcome == 'succeeded' else 1


if __name__ == '__main__':
    sys.exit(main())
