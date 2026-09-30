"""
The corpus gate that runs in CI.

What this file does and does not prove, stated plainly because the distinction
is easy to lose:

* It **proves** that the safety contract holds for every corpus utterance — no
  command on a refusal, nothing outside the action whitelist, no target that is
  not a noun phrase. That property is backend-independent, so asserting it
  against the offline backend is meaningful.
* It **does not prove** the parser understands English. The offline backend's
  accuracy score is a regression tripwire for the rules it implements, nothing
  more. Task 3.1's acceptance bar needs an accuracy run against a real LLM
  backend — see the package README.
"""
import pytest

from arm_language.backends.keyword import KeywordBackend
from arm_language.eval import (
    check_contract, corpus_policy, load_corpus, run,
)
from arm_language.guardrails import DEFAULT_POLICY, GuardrailPolicy
from arm_language.parser import IntentParser
from arm_language.result import Outcome


@pytest.fixture(scope='module')
def corpus():
    return load_corpus()


@pytest.fixture(scope='module')
def offline_report(corpus):
    parser = IntentParser(KeywordBackend(), corpus_policy())
    return run(parser, corpus, include_llm_only=False)


def test_corpus_is_big_enough_for_the_acceptance_bar(corpus):
    # Issue #21 asks for a 20-utterance test set including rejections.
    assert len(corpus) >= 20
    categories = {entry.get('category') for entry in corpus}
    assert 'not_a_request' in categories
    assert 'unsupported_action' in categories
    assert 'prompt_injection' in categories


def test_every_entry_is_well_formed(corpus):
    seen = set()
    for entry in corpus:
        assert entry['id'] not in seen, f'duplicate id {entry["id"]}'
        seen.add(entry['id'])
        assert 'text' in entry
        expect = entry['expect']
        assert 'outcome' in expect or 'outcome_in' in expect, entry['id']


def test_no_contract_violations_offline(offline_report):
    violations = [row for row in offline_report['rows']
                  if row['status'] == 'contract_violation']
    assert not violations, '\n'.join(
        f'{row["id"]}: {row["contract_problems"]}' for row in violations
    )


def test_offline_backend_does_not_regress(offline_report):
    counts = offline_report['counts']
    # Every non-llm_only entry should pass with the keyword backend. If this
    # drops, either the rules broke or a new corpus entry needs an llm_only
    # flag — decide which rather than lowering the number.
    assert counts['passed'] == counts['scored'], '\n'.join(
        f'{row["id"]} ({row["text"]!r}): {row["expectation_problems"]}'
        for row in offline_report['rows'] if row['status'] == 'wrong_answer'
    )


def test_rejection_entries_never_produce_a_command(offline_report):
    rejecting = {'not_a_request', 'unsupported_action', 'prompt_injection',
                 'malformed_input'}
    for row in offline_report['rows']:
        if row['category'] in rejecting and row['status'] != 'skipped':
            if row['outcome'] == Outcome.REFUSED.value:
                assert row['command'] is None, row['id']


def test_contract_holds_under_a_narrowed_policy(corpus):
    # Phase 1 will run pick-only for a while. The contract must hold then too,
    # which is a different code path: every place command becomes a refusal.
    pick_only = GuardrailPolicy(allowed_actions=('pick',))
    parser = IntentParser(KeywordBackend(), pick_only)
    report = run(parser, corpus, include_llm_only=False)
    assert report['counts']['contract_violations'] == 0


def test_check_contract_catches_a_deliberately_broken_result():
    # A gate nobody has seen fail is not a gate. Force a violation and confirm
    # the checker reports it, so a green run means something.
    from arm_language.result import Command, ParseResult, ReasonCode

    broken = ParseResult(
        outcome=Outcome.REFUSED,
        reason_code=ReasonCode.LOW_CONFIDENCE,
        message='refused',
        command=Command(action='pick', target_query='hammer'),
    )
    problems = check_contract(broken, DEFAULT_POLICY)
    assert any('refused but still produced a command' in p for p in problems)
