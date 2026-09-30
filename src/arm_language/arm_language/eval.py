"""
Run the acceptance corpus against a backend and report what happened.

This runner reports **two different numbers**, and conflating them is the main
way a language component gets shipped broken:

``contract``
    Do the safety invariants hold for every utterance — no command on a
    refusal, no action outside the whitelist, no target that is not a noun
    phrase? This is a property of *our* code. It is backend-independent, it is
    deterministic, and it is the gate CI enforces on every PR.

``accuracy``
    Did the parser reach the *right* answer for each utterance? This is a
    property of the **backend's language understanding**, and measuring it
    against the offline keyword backend proves nothing — that backend and this
    corpus were written by the same person on the same afternoon. Only an
    accuracy run against a real LLM backend satisfies task 3.1's acceptance
    criterion.

The distinction is why ``--fail-under`` defaults to gating the contract and not
the accuracy: a green CI badge should never be able to mean "our stub agreed
with our fixtures".
"""
import argparse
import json
import os
import sys
from typing import Dict, List, Optional

from . import guardrails, schema, validator
from .backends import BackendError, create
from .parser import IntentParser
from .result import Outcome, ParseResult

CORPUS_PATH = os.path.join(os.path.dirname(__file__), 'corpus',
                           'utterances.json')

#: Backends with no language model behind them. Corpus entries marked
#: ``llm_only`` are skipped for these, and an accuracy score from one of them
#: is not an acceptance result.
OFFLINE_BACKENDS = frozenset({'keyword', 'replay'})


def load_corpus(path: str = CORPUS_PATH) -> List[dict]:
    """Load the corpus entries."""
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)['utterances']


def check_contract(result: ParseResult,
                   policy: guardrails.GuardrailPolicy) -> List[str]:
    """
    Return the safety invariants this result violates (empty list is good).

    These are the properties the node promises no matter what a backend says.
    Every one of them is checkable without knowing what the right answer was,
    which is what makes them usable as a gate.
    """
    problems = []
    command = result.command

    if result.outcome is Outcome.REFUSED and command is not None:
        problems.append('refused but still produced a command')

    if result.may_move and result.outcome is not Outcome.ACCEPTED:
        problems.append('may_move is True on a non-accepted outcome')

    if command is None:
        return problems

    if command.action not in policy.allowed_actions:
        problems.append(
            f'action "{command.action}" is outside the whitelist '
            f'{policy.allowed_actions}'
        )

    if command.action in schema.JOG_ACTIONS:
        # A jog names no object; its contract is the bounds instead.
        if command.target_query:
            problems.append(f'jog carries target_query "{command.target_query}"')
        if command.distance_cm is not None and not (
                0.0 < command.distance_cm <= schema.MAX_MOVE_CM):
            problems.append(f'distance_cm out of bounds: {command.distance_cm}')
        if command.angle_deg is not None and not (
                0.0 < command.angle_deg <= schema.MAX_ROTATE_DEG):
            problems.append(f'angle_deg out of bounds: {command.angle_deg}')
        if command.speed_level is not None and not (
                0 < abs(command.speed_level) <= schema.MAX_SPEED_LEVEL):
            problems.append(f'speed_level out of bounds: {command.speed_level}')
        return problems

    try:
        validator.check_noun_phrase(command.target_query, 'target_query')
    except validator.ValidationError as exc:
        problems.append(f'target_query is not a noun phrase: {exc.message}')

    if command.action == schema.ACTION_PICK_AND_PLACE:
        if not command.place_target:
            problems.append('pick_and_place with no place_target')
        else:
            try:
                validator.check_noun_phrase(command.place_target,
                                            'place_target')
            except validator.ValidationError as exc:
                problems.append(f'place_target is not a noun phrase: '
                                f'{exc.message}')
    elif command.place_target is not None:
        problems.append(f'plain pick carries place_target '
                        f'"{command.place_target}"')

    unknown = sorted(set(command.modifiers) - set(schema.MODIFIER_KEYS))
    if unknown:
        problems.append(f'unknown modifier key(s): {", ".join(unknown)}')

    if not 0.0 <= command.confidence <= 1.0:
        problems.append(f'confidence out of range: {command.confidence}')

    return problems


def check_expectation(entry: dict, result: ParseResult) -> List[str]:
    """Return the ways this result differs from what the corpus expected."""
    expect = entry['expect']
    problems = []

    if 'outcome' in expect and result.outcome.value != expect['outcome']:
        problems.append(f'outcome {result.outcome.value} != '
                        f'{expect["outcome"]}')

    if 'outcome_in' in expect and result.outcome.value not in expect['outcome_in']:
        problems.append(f'outcome {result.outcome.value} not in '
                        f'{expect["outcome_in"]}')

    if ('reason_code_in' in expect
            and result.reason_code.value not in expect['reason_code_in']):
        problems.append(f'reason_code {result.reason_code.value} not in '
                        f'{expect["reason_code_in"]}')

    command = result.command
    if command is None:
        # Field-level expectations only apply when a command exists; the
        # outcome checks above already caught "we expected one and got none".
        return problems

    for field_name in ('action', 'target_query', 'place_target',
                       'direction', 'distance_cm', 'speed_level',
                       'angle_deg', 'pose_name'):
        if field_name in expect and getattr(command, field_name) != expect[field_name]:
            problems.append(f'{field_name} "{getattr(command, field_name)}" != '
                            f'"{expect[field_name]}"')

    if 'modifiers' in expect and command.modifiers != expect['modifiers']:
        problems.append(f'modifiers {command.modifiers} != '
                        f'{expect["modifiers"]}')

    for banned in expect.get('target_query_excludes', []):
        if banned in command.target_query:
            problems.append(f'target_query contains forbidden "{banned}"')

    return problems


def corpus_policy(**kwargs) -> guardrails.GuardrailPolicy:
    """
    Return the policy the corpus is scored under: every motion action enabled.

    The corpus measures whether the parser *understood* a sentence, so it must
    not be narrowed by a deployment's whitelist — a jog entry scored under the
    pick-only default would fail as ``action_not_allowed`` and say nothing
    about the parse. Deployments still narrow; see ``GuardrailPolicy``.
    """
    kwargs.setdefault('allowed_actions', schema.ALL_MOTION_ACTIONS)
    return guardrails.GuardrailPolicy(**kwargs)


def run(parser: IntentParser, entries: List[dict],
        include_llm_only: bool) -> dict:
    """Run every applicable entry and collect a structured report."""
    rows = []
    for entry in entries:
        if entry.get('llm_only') and not include_llm_only:
            rows.append({
                'id': entry['id'], 'text': entry['text'],
                'category': entry.get('category', ''),
                'status': 'skipped',
                'reason': 'llm_only entry on a non-LLM backend',
            })
            continue

        result = parser.parse(entry['text'])
        contract_problems = check_contract(result, parser.policy)
        expectation_problems = check_expectation(entry, result)

        if contract_problems:
            status = 'contract_violation'
        elif expectation_problems:
            status = 'wrong_answer'
        else:
            status = 'pass'

        rows.append({
            'id': entry['id'],
            'text': entry['text'],
            'category': entry.get('category', ''),
            'status': status,
            'outcome': result.outcome.value,
            'reason_code': result.reason_code.value,
            'message': result.message,
            'command': None if result.command is None else {
                'action': result.command.action,
                'target_query': result.command.target_query,
                'place_target': result.command.place_target,
                'modifiers': result.command.modifiers,
                'confidence': round(result.command.confidence, 3),
            },
            'contract_problems': contract_problems,
            'expectation_problems': expectation_problems,
            'latency_ms': result.detail.get('latency_ms'),
        })

    scored = [row for row in rows if row['status'] != 'skipped']
    return {
        'backend': parser.backend_name,
        'policy': {
            'allowed_actions': list(parser.policy.allowed_actions),
            'accept_threshold': parser.policy.accept_threshold,
            'confirm_threshold': parser.policy.confirm_threshold,
            'require_confirmation': parser.policy.require_confirmation,
        },
        'counts': {
            'total': len(rows),
            'scored': len(scored),
            'skipped': len(rows) - len(scored),
            'passed': sum(1 for row in scored if row['status'] == 'pass'),
            'wrong_answer': sum(1 for row in scored
                                if row['status'] == 'wrong_answer'),
            'contract_violations': sum(1 for row in scored
                                       if row['status'] == 'contract_violation'),
        },
        'rows': rows,
    }


def _format_report(report: dict) -> str:
    """Render a report as a terminal table."""
    marks = {'pass': 'PASS', 'wrong_answer': 'WRONG', 'skipped': 'skip',
             'contract_violation': 'UNSAFE'}
    lines = [
        f'Backend: {report["backend"]}    '
        f'policy: accept>={report["policy"]["accept_threshold"]} '
        f'confirm>={report["policy"]["confirm_threshold"]} '
        f'actions={",".join(report["policy"]["allowed_actions"])}',
        '',
        f'{"":6} {"id":<12} {"outcome":<19} {"reason":<26} text',
        '-' * 100,
    ]
    for row in report['rows']:
        lines.append(
            f'{marks[row["status"]]:<6} {row["id"]:<12} '
            f'{row.get("outcome", "-"):<19} {row.get("reason_code", "-"):<26} '
            f'{row["text"][:40]}'
        )
        for problem in row.get('contract_problems', []):
            lines.append(f'       !! CONTRACT: {problem}')
        for problem in row.get('expectation_problems', []):
            lines.append(f'       -- expected: {problem}')

    counts = report['counts']
    accuracy = (100.0 * counts['passed'] / counts['scored']
                if counts['scored'] else 0.0)
    lines += [
        '-' * 100,
        f'contract violations : {counts["contract_violations"]}  '
        f'(must be 0 — these are safety failures, not accuracy ones)',
        f'accuracy            : {counts["passed"]}/{counts["scored"]} '
        f'= {accuracy:.1f}%   ({counts["skipped"]} skipped)',
    ]
    if report['backend'] in OFFLINE_BACKENDS:
        lines.append(
            'NOTE: this backend has no language model. The accuracy number '
            'above is NOT an acceptance result for task 3.1.'
        )
    return '\n'.join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    """Command-line entry point. Returns a process exit code."""
    cli = argparse.ArgumentParser(
        prog='python3 -m arm_language.eval',
        description='Run the intent-parser acceptance corpus.',
    )
    cli.add_argument('--backend', default='keyword',
                     help='backend name (default: keyword, which runs offline)')
    cli.add_argument('--model', default=None,
                     help='model id, for backends that take one')
    cli.add_argument('--corpus', default=CORPUS_PATH)
    cli.add_argument('--report', default=None,
                     help='write the full JSON report here')
    cli.add_argument('--accept-threshold', type=float, default=None)
    cli.add_argument('--confirm-threshold', type=float, default=None)
    cli.add_argument('--fail-under', type=float, default=None,
                     help='exit non-zero if accuracy is below this percentage. '
                          'Contract violations always fail regardless.')
    cli.add_argument('--include-llm-only', action='store_true', default=None,
                     help='force-run llm_only entries (default: automatic)')
    args = cli.parse_args(argv)

    policy_kwargs: Dict[str, object] = {}
    if args.accept_threshold is not None:
        policy_kwargs['accept_threshold'] = args.accept_threshold
    if args.confirm_threshold is not None:
        policy_kwargs['confirm_threshold'] = args.confirm_threshold
    policy = corpus_policy(**policy_kwargs)

    backend_kwargs = {'model': args.model} if args.model else {}
    try:
        backend = create(args.backend, **backend_kwargs)
    except BackendError as exc:
        print(f'Could not start backend "{args.backend}": {exc}',
              file=sys.stderr)
        return 2

    include_llm_only = (args.include_llm_only
                        if args.include_llm_only is not None
                        else args.backend not in OFFLINE_BACKENDS)

    report = run(IntentParser(backend, policy), load_corpus(args.corpus),
                 include_llm_only)
    print(_format_report(report))

    if args.report:
        with open(args.report, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=2)
        print(f'\nJSON report written to {args.report}')

    counts = report['counts']
    if counts['contract_violations']:
        return 1
    if args.fail_under is not None and counts['scored']:
        accuracy = 100.0 * counts['passed'] / counts['scored']
        if accuracy < args.fail_under:
            print(f'\nAccuracy {accuracy:.1f}% is below --fail-under '
                  f'{args.fail_under}%.', file=sys.stderr)
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
