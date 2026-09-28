"""
Tests for the policy layer (issue #23).

The question here is never "did we understand the request" — every intent below
is already valid. It is "given that we understood it, do we move?".
"""
import pytest

from arm_language import schema
from arm_language.guardrails import DEFAULT_POLICY, GuardrailPolicy, apply
from arm_language.result import Outcome, ReasonCode
from arm_language.validator import ValidatedIntent


def intent(**overrides):
    """Return a valid intent, with overrides applied."""
    base = {
        'action': 'pick',
        'target_query': 'hammer',
        'place_target': None,
        'modifiers': {},
        'confidence': 0.9,
        'reason': None,
    }
    base.update(overrides)
    return ValidatedIntent(**base)


# --- the confidence bands ---------------------------------------------------

def test_confident_request_is_accepted():
    result = apply(intent(confidence=0.9))
    assert result.outcome is Outcome.ACCEPTED
    assert result.reason_code is ReasonCode.OK
    assert result.may_move


def test_middling_confidence_asks_first():
    result = apply(intent(confidence=0.6))
    assert result.outcome is Outcome.NEEDS_CONFIRMATION
    assert result.command is not None
    # The command is present so the caller can show it — but may_move is
    # False, because the human has not answered yet.
    assert not result.may_move
    assert 'did you mean' in result.message.lower()


def test_low_confidence_is_refused_outright():
    result = apply(intent(confidence=0.1))
    assert result.outcome is Outcome.REFUSED
    assert result.reason_code is ReasonCode.LOW_CONFIDENCE
    assert result.command is None
    assert not result.may_move
    # A refusal still records what it would have done, so the log explains
    # itself without a re-run.
    assert result.detail['would_have_done']


@pytest.mark.parametrize('confidence,expected', [
    (0.39, Outcome.REFUSED),
    (0.40, Outcome.NEEDS_CONFIRMATION),   # boundary is inclusive-below
    (0.74, Outcome.NEEDS_CONFIRMATION),
    (0.75, Outcome.ACCEPTED),
])
def test_band_boundaries(confidence, expected):
    assert apply(intent(confidence=confidence)).outcome is expected


# --- the action whitelist ---------------------------------------------------

def test_action_outside_the_whitelist_is_refused():
    # Phase 1 can legitimately run pick-only: the parser understands place
    # commands long before the arm can execute them.
    pick_only = GuardrailPolicy(allowed_actions=(schema.ACTION_PICK,))
    result = apply(intent(action='pick_and_place', place_target='bin'),
                   pick_only)
    assert result.outcome is Outcome.REFUSED
    assert result.reason_code is ReasonCode.ACTION_NOT_ALLOWED
    assert result.command is None
    assert 'not enabled' in result.message


def test_backend_rejection_is_passed_through():
    result = apply(intent(action=schema.ACTION_REJECT, target_query='',
                          reason='The arm cannot weld.'))
    assert result.outcome is Outcome.REFUSED
    assert result.reason_code is ReasonCode.LLM_REJECTED
    assert result.message == 'The arm cannot weld.'


def test_backend_rejection_without_a_reason_still_says_something():
    result = apply(intent(action=schema.ACTION_REJECT, target_query='',
                          reason=None))
    assert result.message


# --- always-confirm mode ----------------------------------------------------

def test_require_confirmation_holds_even_a_certain_command():
    policy = GuardrailPolicy(require_confirmation=True)
    result = apply(intent(confidence=1.0), policy)
    assert result.outcome is Outcome.NEEDS_CONFIRMATION
    assert result.reason_code is ReasonCode.CONFIRMATION_REQUIRED
    assert not result.may_move


# --- the echo ---------------------------------------------------------------

def test_echo_is_built_from_the_parse_not_the_utterance():
    result = apply(intent(action='pick_and_place', target_query='red block',
                          place_target='bin', modifiers={'color': 'red'},
                          confidence=0.5))
    # What the human confirms must be what the arm will do. The modifier is
    # not duplicated just because it also appears in the target.
    assert result.message == 'Did you mean: pick up the red block and place it in the bin?'


# --- policy validation ------------------------------------------------------

@pytest.mark.parametrize('kwargs', [
    {'accept_threshold': 0.3, 'confirm_threshold': 0.8},   # inverted
    {'accept_threshold': 1.4},
    {'confirm_threshold': -0.2},
    {'allowed_actions': ('pick', 'reject')},               # reject is not motion
    {'allowed_actions': ('teleport',)},
])
def test_impossible_policies_are_rejected_at_construction(kwargs):
    # A policy that cannot behave as written must fail where an operator sees
    # it — at startup — not by quietly behaving differently at runtime.
    with pytest.raises(ValueError):
        GuardrailPolicy(**kwargs)


def test_default_policy_is_conservative():
    assert DEFAULT_POLICY.accept_threshold >= 0.7
    assert DEFAULT_POLICY.confirm_threshold > 0.0
    assert set(DEFAULT_POLICY.allowed_actions) <= set(schema.MOTION_ACTIONS)
