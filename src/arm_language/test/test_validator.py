"""
Adversarial tests for the trust boundary.

Every case here is something a backend might hand us that we must not act on.
They are written as raw strings rather than built from the schema on purpose:
constructing a fixture with the same code that validates it would test that a
function agrees with itself.
"""
import json

import pytest

from arm_language import schema
from arm_language.result import ReasonCode
from arm_language.validator import (
    ValidationError, check_noun_phrase, normalize_phrase, validate,
)


def payload(**overrides):
    """Return a valid backend response as JSON text, with overrides applied."""
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


def reason_of(raw):
    """Validate ``raw`` and return the reason code it was rejected with."""
    with pytest.raises(ValidationError) as caught:
        validate(raw)
    return caught.value.reason_code


# --- the happy path, so the negative tests below mean something -------------

def test_valid_pick_survives():
    intent = validate(payload())
    assert intent.action == 'pick'
    assert intent.target_query == 'hammer'
    assert intent.place_target is None
    assert intent.confidence == pytest.approx(0.9)


def test_valid_pick_and_place_survives():
    intent = validate(payload(action='pick_and_place', place_target='the Bin'))
    assert intent.action == 'pick_and_place'
    assert intent.place_target == 'bin'


def test_modifiers_are_kept_and_nulls_dropped():
    intent = validate(payload(
        target_query='red screwdriver',
        modifiers=dict({key: None for key in schema.MODIFIER_KEYS},
                       color='Red'),
    ))
    assert intent.modifiers == {'color': 'red'}


# --- malformed transport ----------------------------------------------------

@pytest.mark.parametrize('raw', [
    '',
    'not json at all',
    '{"action": "pick"',          # truncated mid-object
    '```json\n{"action": "pick"}\n```',   # fenced, as chatty models emit
])
def test_non_json_is_refused(raw):
    assert reason_of(raw) is ReasonCode.NOT_JSON


@pytest.mark.parametrize('raw', ['[]', '"pick up the hammer"', '42', 'null'])
def test_json_that_is_not_an_object_is_refused(raw):
    assert reason_of(raw) is ReasonCode.SCHEMA_VIOLATION


# --- schema violations ------------------------------------------------------

def test_missing_field_is_refused():
    body = json.loads(payload())
    del body['confidence']
    assert reason_of(json.dumps(body)) is ReasonCode.SCHEMA_VIOLATION


def test_extra_field_is_refused():
    body = json.loads(payload())
    body['urgency'] = 'high'
    assert reason_of(json.dumps(body)) is ReasonCode.SCHEMA_VIOLATION


@pytest.mark.parametrize('action', ['shutdown', 'delete_everything', 'PICK UP',
                                    'pick_and_throw', ''])
def test_invented_action_is_refused(action):
    assert reason_of(payload(action=action)) is ReasonCode.SCHEMA_VIOLATION


def test_unknown_modifier_key_is_refused():
    modifiers = dict({key: None for key in schema.MODIFIER_KEYS},
                     urgency='immediate')
    assert (reason_of(payload(modifiers=modifiers))
            is ReasonCode.MODIFIER_KEY_UNKNOWN)


@pytest.mark.parametrize('confidence', [1.5, -0.1, 42, 'high', None, True])
def test_confidence_outside_the_contract_is_refused(confidence):
    # `True` is in this list because bool subclasses int in Python: without an
    # explicit check, `confidence: true` would validate as 1.0 — maximum
    # confidence from a backend that did not report a number at all.
    assert reason_of(payload(confidence=confidence)) is ReasonCode.SCHEMA_VIOLATION


def test_wrong_type_for_target_is_refused():
    assert reason_of(payload(target_query=['hammer'])) is ReasonCode.SCHEMA_VIOLATION


# --- the noun-phrase rule, which is the point of the whole node -------------

@pytest.mark.parametrize('target', [
    'pick up the hammer',
    'grab the red screwdriver from the bench',
    'the hammer, please.',
    'I think you want the hammer',
    'move the thing over there',
])
def test_sentence_in_target_is_refused(target):
    assert reason_of(payload(target_query=target)) is ReasonCode.TARGET_NOT_NOUN_PHRASE


@pytest.mark.parametrize('target', [
    '<script>alert(1)</script>',
    'hammer; rm -rf /',
    'hammer & wrench',
    'hammer, wrench',
    'hammer/wrench',
])
def test_non_word_characters_in_target_are_refused(target):
    assert reason_of(payload(target_query=target)) is ReasonCode.TARGET_NOT_NOUN_PHRASE


def test_embedded_newlines_are_normalised_rather_than_refused():
    # Normalisation runs before the checks, so a newline collapses to a space
    # and "hammer\nwrench" is judged as the two-word phrase it became — not
    # refused for containing a control character it no longer contains.
    assert validate(payload(target_query='hammer\nwrench')).target_query == \
        'hammer wrench'


def test_empty_target_is_refused():
    assert reason_of(payload(target_query='')) is ReasonCode.TARGET_EMPTY


@pytest.mark.parametrize('target', [
    'guitar pick',      # 'pick' is a fine noun when it is not leading
    'set square',
    '10 mm wrench',
    'allen key',
    'long-nose pliers',
])
def test_legitimate_objects_that_look_like_verbs_survive(target):
    # The verb check looks only at the first word precisely so a makerspace
    # can own guitar picks.
    assert validate(payload(target_query=target)).target_query == target


@pytest.mark.parametrize('target', ['drop cloth', 'pick', 'lift ring'])
def test_known_false_positives_of_the_verb_rule(target):
    # Documented, not accidental: these are real objects the first-word verb
    # rule refuses. The trade is argued in check_noun_phrase. This test exists
    # so the cost stays visible and someone can revisit it deliberately.
    assert reason_of(payload(target_query=target)) is ReasonCode.TARGET_NOT_NOUN_PHRASE


# --- cross-field rules ------------------------------------------------------

def test_pick_and_place_without_destination_is_refused():
    assert (reason_of(payload(action='pick_and_place', place_target=None))
            is ReasonCode.PLACE_TARGET_MISSING)


def test_plain_pick_with_a_destination_is_refused():
    # Contradictory output means the model was not sure which command it was
    # producing. Acting on half of it is how an object ends up somewhere
    # nobody asked for.
    assert (reason_of(payload(action='pick', place_target='bin'))
            is ReasonCode.PLACE_TARGET_UNEXPECTED)


def test_rejection_needs_no_target():
    intent = validate(payload(action='reject', target_query='',
                              reason='Not a manipulation request.'))
    assert intent.action == schema.ACTION_REJECT
    assert intent.reason == 'Not a manipulation request.'


# --- normalisation ----------------------------------------------------------

@pytest.mark.parametrize('raw,expected', [
    ('the Hammer', 'hammer'),
    ('  a  RED   screwdriver ', 'red screwdriver'),
    ('"hammer"', 'hammer'),
    ('an allen key', 'allen key'),
    ('the', 'the'),          # a lone article is not an article to strip
])
def test_normalize_phrase(raw, expected):
    assert normalize_phrase(raw) == expected


def test_check_noun_phrase_names_the_field_it_rejected():
    with pytest.raises(ValidationError) as caught:
        check_noun_phrase('put it over there somewhere', 'place_target')
    assert 'place_target' in caught.value.message
