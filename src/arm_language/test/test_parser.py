"""
End-to-end pipeline tests, including the ways a backend can fail us.

The recurring assertion is that ``parse`` never raises and never returns a
command it should not. A parser that crashes is a stuck orchestrator; a parser
that returns a bad command is a moving arm.
"""
import json

import pytest

from arm_language import schema
from arm_language.backends.base import BackendError
from arm_language.backends.replay import ReplayBackend
from arm_language.parser import IntentParser
from arm_language.result import Outcome, ReasonCode


def response(**overrides):
    """Return a valid backend response as JSON text."""
    body = {
        'action': 'pick',
        'target_query': 'hammer',
        'place_target': None,
        'modifiers': {key: None for key in schema.MODIFIER_KEYS},
        'confidence': 0.9,
        'reason': None,
    }
    body.update(overrides)
    return json.dumps(body)


class ExplodingBackend:
    """A backend that raises whatever it was told to."""

    name = 'exploding'

    def __init__(self, error):
        self._error = error

    def complete(self, utterance):
        raise self._error


def test_happy_path():
    parser = IntentParser(ReplayBackend({'pick up the hammer': response()}))
    result = parser.parse('pick up the hammer')
    assert result.may_move
    assert result.command.target_query == 'hammer'


def test_diagnostics_are_attached_to_every_result():
    parser = IntentParser(ReplayBackend({'pick up the hammer': response()}))
    result = parser.parse('pick up the hammer')
    assert result.detail['backend'] == 'replay'
    assert result.detail['utterance'] == 'pick up the hammer'
    assert isinstance(result.detail['latency_ms'], float)


def test_early_refusals_also_carry_diagnostics():
    # The failures you most want to explain are the ones that never reached a
    # backend, so these must not be a second-class path.
    parser = IntentParser(ReplayBackend({}))
    result = parser.parse('')
    assert result.reason_code is ReasonCode.EMPTY_UTTERANCE
    assert 'latency_ms' in result.detail


@pytest.mark.parametrize('utterance', ['', '   ', '\n\t '])
def test_blank_utterance_never_reaches_the_backend(utterance):
    # ReplayBackend with no entries raises on any call, so reaching it would
    # surface as BACKEND_ERROR rather than EMPTY_UTTERANCE.
    parser = IntentParser(ReplayBackend({}))
    assert parser.parse(utterance).reason_code is ReasonCode.EMPTY_UTTERANCE


def test_overlong_utterance_never_reaches_the_backend():
    parser = IntentParser(ReplayBackend({}))
    result = parser.parse('pick up the hammer ' * 40)
    assert result.reason_code is ReasonCode.UTTERANCE_TOO_LONG
    assert result.command is None


def test_backend_error_becomes_a_refusal_not_an_exception():
    parser = IntentParser(ExplodingBackend(BackendError('no credentials')))
    result = parser.parse('pick up the hammer')
    assert result.outcome is Outcome.REFUSED
    assert result.reason_code is ReasonCode.BACKEND_ERROR
    assert 'no credentials' in result.message


@pytest.mark.parametrize('error', [
    RuntimeError('socket closed'),
    ValueError('unexpected payload'),
    KeyError('model'),
    TimeoutError('read timed out'),
])
def test_undeclared_backend_exceptions_are_contained(error):
    # A backend is third-party code. Whatever it throws, the node stays up and
    # the workflow gets a state it already knows how to handle.
    parser = IntentParser(ExplodingBackend(error))
    result = parser.parse('pick up the hammer')
    assert result.outcome is Outcome.REFUSED
    assert result.reason_code is ReasonCode.BACKEND_ERROR
    assert result.detail['error_type'] == type(error).__name__


def test_raw_response_is_kept_on_a_validation_failure():
    # "schema_violation" with no sample of what arrived is an unactionable log.
    parser = IntentParser(ReplayBackend({'x': '{"action": "teleport"}'}))
    result = parser.parse('x')
    assert result.outcome is Outcome.REFUSED
    assert 'teleport' in result.detail['raw_response']


def test_a_refused_parse_never_carries_a_command():
    responses = {
        'a': 'not json',
        'b': response(action='shutdown'),
        'c': response(target_query='pick up the hammer'),
        'd': response(confidence=0.05),
        'e': response(action='reject', target_query='', reason='nope'),
        'f': response(action='pick', place_target='bin'),
    }
    parser = IntentParser(ReplayBackend(responses))
    for utterance in responses:
        result = parser.parse(utterance)
        assert result.outcome is Outcome.REFUSED, utterance
        assert result.command is None, utterance
        assert not result.may_move, utterance


def test_parse_is_total_over_hostile_input():
    # Nothing here should produce an exception, whatever else it produces.
    hostile = [
        '\x00\x01\x02',
        'ç' * 250,
        '{"action": "pick"}',
        'pick up the ' + 'hammer ' * 30,
        'DROP TABLE objects; --',
    ]
    parser = IntentParser(ReplayBackend({}, default=response()))
    for utterance in hostile:
        result = parser.parse(utterance)
        assert result.outcome in tuple(Outcome)


def test_replay_backend_fails_loudly_on_an_unknown_utterance():
    # A stale fixture file must not silently turn into a skipped test case.
    with pytest.raises(BackendError):
        ReplayBackend({}).complete('anything')
