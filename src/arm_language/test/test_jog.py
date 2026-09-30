"""
Tests for the jog vocabulary (task 3.2): move / rotate / go_to.

Two things are under test. The validator's *bounds* — the property that no
sentence can move the tool further than ``schema.MAX_MOVE_CM`` — and the
offline keyword grammar for the sentences the lab demo is built around.
"""
import json

import pytest

from arm_language import schema
from arm_language.backends.keyword import KeywordBackend
from arm_language.guardrails import GuardrailPolicy, apply
from arm_language.parser import IntentParser
from arm_language.result import Outcome, ReasonCode
from arm_language.validator import ValidationError, validate


def payload(action, **motion):
    """Return a schema-shaped backend response for a jog action."""
    body = {
        'action': action,
        'target_query': '',
        'place_target': None,
        'modifiers': {key: None for key in schema.MODIFIER_KEYS},
        'motion': dict({key: None for key in schema.MOTION_KEYS}, **motion),
        'confidence': 0.9,
        'reason': None,
    }
    return json.dumps(body)


def reason_of(raw):
    with pytest.raises(ValidationError) as caught:
        validate(raw)
    return caught.value.reason_code


JOG_POLICY = GuardrailPolicy(allowed_actions=schema.JOG_ACTIONS)


# --- validator: defaults ------------------------------------------------------

def test_move_without_amount_gets_the_default():
    intent = validate(payload('move', direction='up'))
    assert intent.direction == 'up'
    assert intent.distance_cm == schema.DEFAULT_MOVE_CM


def test_rotate_without_parameters_gets_defaults():
    intent = validate(payload('rotate'))
    assert intent.speed_level == schema.DEFAULT_SPEED_LEVEL
    assert intent.angle_deg == schema.DEFAULT_ROTATE_DEG


def test_go_to_offset_distance_defaults_when_direction_given():
    intent = validate(payload('go_to', pose_name='Home', direction='up'))
    assert intent.pose_name == 'home'
    assert intent.distance_cm == schema.DEFAULT_MOVE_CM


def test_missing_motion_object_is_tolerated_for_pick():
    body = json.loads(payload('pick'))
    del body['motion']
    body['target_query'] = 'hammer'
    assert validate(json.dumps(body)).action == 'pick'


# --- validator: bounds (the safety property) ---------------------------------

@pytest.mark.parametrize('distance', [0, -2, schema.MAX_MOVE_CM + 0.1, 100, 1000])
def test_move_distance_outside_bounds_is_refused(distance):
    assert (reason_of(payload('move', direction='up', distance_cm=distance))
            is ReasonCode.MOTION_OUT_OF_BOUNDS)


def test_move_at_the_bound_is_allowed():
    intent = validate(payload('move', direction='down', distance_cm=schema.MAX_MOVE_CM))
    assert intent.distance_cm == schema.MAX_MOVE_CM


@pytest.mark.parametrize('level', [0, 4, -4, 10])
def test_rotate_speed_outside_bounds_is_refused(level):
    assert (reason_of(payload('rotate', speed_level=level))
            is ReasonCode.MOTION_OUT_OF_BOUNDS)


@pytest.mark.parametrize('angle', [0, -30, schema.MAX_ROTATE_DEG + 1, 720])
def test_rotate_angle_outside_bounds_is_refused(angle):
    assert (reason_of(payload('rotate', angle_deg=angle))
            is ReasonCode.MOTION_OUT_OF_BOUNDS)


def test_go_to_offset_is_bounded_too():
    assert (reason_of(payload('go_to', pose_name='home', direction='up',
                              distance_cm=50))
            is ReasonCode.MOTION_OUT_OF_BOUNDS)


# --- validator: shape and cross-field rules ----------------------------------

def test_move_needs_a_direction():
    assert reason_of(payload('move', distance_cm=2)) is ReasonCode.MOTION_FIELD_MISSING


def test_unknown_direction_is_a_schema_violation():
    assert reason_of(payload('move', direction='sideways')) is ReasonCode.SCHEMA_VIOLATION


def test_go_to_needs_a_pose_name():
    assert reason_of(payload('go_to')) is ReasonCode.MOTION_FIELD_MISSING


def test_go_to_distance_without_direction_is_refused():
    assert (reason_of(payload('go_to', pose_name='home', distance_cm=3))
            is ReasonCode.MOTION_FIELD_MISSING)


def test_pose_name_must_be_a_noun_phrase():
    assert (reason_of(payload('go_to', pose_name='go to the home pose now'))
            is ReasonCode.TARGET_NOT_NOUN_PHRASE)


def test_rotate_with_a_direction_is_refused():
    assert (reason_of(payload('rotate', direction='up'))
            is ReasonCode.MOTION_FIELD_UNEXPECTED)


def test_pick_with_motion_parameters_is_refused():
    body = json.loads(payload('pick', direction='up'))
    body['target_query'] = 'hammer'
    assert reason_of(json.dumps(body)) is ReasonCode.MOTION_FIELD_UNEXPECTED


def test_jog_with_an_object_is_refused():
    body = json.loads(payload('move', direction='up'))
    body['target_query'] = 'hammer'
    assert reason_of(json.dumps(body)) is ReasonCode.SCHEMA_VIOLATION


@pytest.mark.parametrize('value', ['2', True, [2]])
def test_distance_must_be_a_number(value):
    assert (reason_of(payload('move', direction='up', distance_cm=value))
            is ReasonCode.SCHEMA_VIOLATION)


def test_speed_level_must_be_an_integer():
    assert reason_of(payload('rotate', speed_level=1.5)) is ReasonCode.SCHEMA_VIOLATION


# --- guardrails ---------------------------------------------------------------

def test_jog_actions_are_off_by_default():
    intent = validate(payload('move', direction='up'))
    result = apply(intent)
    assert result.outcome is Outcome.REFUSED
    assert result.reason_code is ReasonCode.ACTION_NOT_ALLOWED


def test_jog_policy_accepts_and_echoes_every_number():
    intent = validate(payload('move', direction='down', distance_cm=2))
    result = apply(intent, JOG_POLICY)
    assert result.outcome is Outcome.ACCEPTED
    assert result.command.direction == 'down'
    assert result.command.distance_cm == 2.0
    assert '2 cm' in result.message and 'down' in result.message


def test_jog_policy_still_refuses_picks():
    body = json.loads(payload('pick'))
    body['target_query'] = 'hammer'
    result = apply(validate(json.dumps(body)), JOG_POLICY)
    assert result.reason_code is ReasonCode.ACTION_NOT_ALLOWED


def test_rotate_echo_names_spin_direction_and_speed():
    result = apply(validate(payload('rotate', speed_level=-1)), JOG_POLICY)
    assert 'clockwise' in result.message
    assert 'counter' not in result.message
    assert 'speed -1' in result.message


# --- keyword backend grammar ---------------------------------------------------

@pytest.fixture(scope='module')
def parser():
    return IntentParser(KeywordBackend(), JOG_POLICY)


@pytest.mark.parametrize('text, direction, distance', [
    ('could you go up a bit?', 'up', schema.SMALL_MOVE_CM),
    ('could you go down by 2?', 'down', 2.0),
    ('go up', 'up', schema.DEFAULT_MOVE_CM),
    ('go down 2', 'down', 2.0),
    ('go right', 'right', schema.DEFAULT_MOVE_CM),
    ('go left', 'left', schema.DEFAULT_MOVE_CM),
    ('please move forward 10cm', 'forward', 10.0),
    ('move back 3 centimeters', 'backward', 3.0),
    ('raise it a little', 'up', schema.SMALL_MOVE_CM),
    ('lower 4 cm', 'down', 4.0),
    ('up', 'up', schema.DEFAULT_MOVE_CM),
])
def test_keyword_moves(parser, text, direction, distance):
    result = parser.parse(text)
    assert result.outcome is Outcome.ACCEPTED, result.message
    assert result.command.action == 'move'
    assert result.command.direction == direction
    assert result.command.distance_cm == distance


@pytest.mark.parametrize('text, level, angle', [
    ('can you spin slowly?', 1, schema.DEFAULT_ROTATE_DEG),
    ('spin at speed -1', -1, schema.DEFAULT_ROTATE_DEG),
    ('rotate clockwise fast by 45 degrees', -3, 45.0),
    ('turn the wrist', schema.DEFAULT_SPEED_LEVEL, schema.DEFAULT_ROTATE_DEG),
    ('spin counter-clockwise at speed 2', 2, schema.DEFAULT_ROTATE_DEG),
    ('rotate 90 degrees', schema.DEFAULT_SPEED_LEVEL, 90.0),
])
def test_keyword_rotations(parser, text, level, angle):
    result = parser.parse(text)
    assert result.outcome is Outcome.ACCEPTED, result.message
    assert result.command.action == 'rotate'
    assert result.command.speed_level == level
    assert result.command.angle_deg == angle


@pytest.mark.parametrize('text, pose, direction, distance', [
    ('go home', 'home', None, None),
    ('return to home', 'home', None, None),
    ('go to the start pose', 'start', None, None),
    ('go to the start pose but 3 cm up', 'start', 'up', 3.0),
    ('go home and then up 3', 'home', 'up', 3.0),
    ('go to observe position 2 cm higher', 'observe', 'up', 2.0),
])
def test_keyword_go_to(parser, text, pose, direction, distance):
    result = parser.parse(text)
    assert result.outcome is Outcome.ACCEPTED, result.message
    assert result.command.action == 'go_to'
    assert result.command.pose_name == pose
    assert result.command.direction == direction
    assert result.command.distance_cm == distance


@pytest.mark.parametrize('text', [
    'let me drive it myself',
    'can I control the arm?',
    'give me manual control',
    'switch to keyboard control',
    'I want to control it myself',
    'take over',
    'teleop',
])
def test_keyword_teleop(parser, text):
    result = parser.parse(text)
    assert result.outcome is Outcome.ACCEPTED, result.message
    assert result.command.action == 'teleop'
    assert 'controls' in result.message


@pytest.mark.parametrize('text', [
    'let me grab the hammer',
    'control the temperature',
])
def test_keyword_teleop_does_not_swallow_other_requests(parser, text):
    result = parser.parse(text)
    assert result.command is None or result.command.action != 'teleop'


def test_teleop_takes_no_motion_fields():
    assert (reason_of(payload('teleop', direction='up'))
            is ReasonCode.MOTION_FIELD_UNEXPECTED)
    assert validate(payload('teleop')).action == 'teleop'


def test_keyword_refuses_out_of_bounds_move(parser):
    result = parser.parse('go up 100')
    assert result.outcome is Outcome.REFUSED
    assert result.reason_code is ReasonCode.MOTION_OUT_OF_BOUNDS


def test_keyword_still_parses_picks_as_picks(parser):
    # The jog grammar must not swallow object requests.
    result = parser.parse('move the blue block onto the tray')
    assert result.reason_code is ReasonCode.ACTION_NOT_ALLOWED
    assert result.detail['requested_action'] == 'pick_and_place'


def test_keyword_refuses_blended_request(parser):
    result = parser.parse('go up and grab the hammer')
    assert result.outcome is Outcome.REFUSED
