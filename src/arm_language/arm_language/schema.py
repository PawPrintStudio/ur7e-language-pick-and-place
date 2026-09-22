"""
The command schema: one definition, used twice.

``COMMAND_SCHEMA`` is sent to the LLM as a JSON Schema so the model's decoding
is *constrained* to our shape, and it is the reference the validator checks
against afterwards. Keeping one constant for both jobs is the point: a schema
that lives in two places drifts, and the half that drifts is always the half
nobody tested.

Constrained decoding is not validation
--------------------------------------
It is tempting to think structured outputs make :mod:`arm_language.validator`
redundant. They do not, for three reasons:

1. **Shape is not meaning.** The schema can force ``target_query`` to be a
   string. It cannot force it to be a *noun phrase* — and a sentence here
   silently degrades NanoOWL (architecture D3), which is the one thing this
   node exists to prevent.
2. **Not every backend is constrained.** The offline keyword backend and any
   future local model produce text with no schema enforcement at all. The
   safety properties must hold for every backend, so they cannot live inside
   one of them.
3. **Cross-field rules are outside JSON Schema's comfort zone.** "``place_target``
   is required if and only if the action is ``pick_and_place``" is expressible
   with ``oneOf`` gymnastics that strict mode rejects; it is one clear line of
   Python.

The rule of thumb: the schema is an *optimisation* that makes the model's job
easier, never the thing standing between a typo and the robot moving.
"""

# --- action vocabulary -----------------------------------------------------
# Actions that cause the arm to move. This tuple IS the whitelist of issue #23;
# arm_interfaces/msg/Command.msg carries the same values as message constants.
ACTION_PICK = 'pick'
ACTION_PICK_AND_PLACE = 'pick_and_place'
MOTION_ACTIONS = (ACTION_PICK, ACTION_PICK_AND_PLACE)

# Not a motion action: the backend's way of saying "this was not a request to
# move an object". Letting the model say so explicitly is much safer than
# forcing it to choose a motion action for "what time is it" and hoping the
# confidence score saves us.
ACTION_REJECT = 'reject'

BACKEND_ACTIONS = MOTION_ACTIONS + (ACTION_REJECT,)

# --- modifier vocabulary ---------------------------------------------------
# Descriptive attributes we let the speaker attach to a target. Closed on
# purpose: an LLM inventing a key is a signal that it is improvising, and
# phase-5 modifier-aware re-ranking needs a vocabulary it can plan around.
# Widening this set is a one-line change here plus a corpus entry.
MODIFIER_KEYS = ('color', 'size', 'material', 'position')

# --- bounds ----------------------------------------------------------------
# Longest utterance we will forward to a backend. Bounds cost, latency, and the
# surface area of anything pasted into the console.
MAX_UTTERANCE_CHARS = 300

# A noun phrase, not a sentence. "small red screwdriver" is 3 words; anything
# past 4 is a description or an instruction that leaked through.
MAX_TARGET_WORDS = 4
MAX_TARGET_CHARS = 64

# Characters that mean "this is a sentence, not a noun phrase".
SENTENCE_PUNCTUATION = '.?!;:\n\r'

# After normalisation, a target may only contain these. Object names are words,
# digits ("10 mm wrench"), spaces, hyphens and apostrophes — nothing else.
# This is the check that turns markup, shell fragments, and anything else pasted
# into the console into a refusal rather than a detector query that quietly
# matches nothing. Whitelisting the characters is safer than blacklisting the
# dangerous ones, because the blacklist is never finished.
TARGET_CHAR_CLASS = r"a-z0-9 '\-"

# Imperative verbs that must never survive into `target_query`. If the model
# hands us "pick up the hammer" we reject rather than silently repair it: a
# quiet repair hides a misparse, and the next misparse might not be repairable.
IMPERATIVE_VERBS = (
    'pick', 'grab', 'get', 'take', 'bring', 'move', 'put', 'place',
    'fetch', 'hand', 'give', 'lift', 'drop', 'find', 'locate', 'go',
)

# Stripped from the front of a target. NanoOWL is unbothered by "a hammer", but
# normalising means "the hammer" and "hammer" produce identical queries, which
# makes detections comparable run over run.
ARTICLES = ('a', 'an', 'the')


def _nullable_string() -> dict:
    """
    Return a schema for a string field the model may leave unset.

    Strict structured outputs require every property to appear in ``required``,
    so "optional" is expressed as a nullable type rather than by omission.
    """
    return {'type': ['string', 'null']}


COMMAND_SCHEMA = {
    'type': 'object',
    'properties': {
        'action': {
            'type': 'string',
            'enum': list(BACKEND_ACTIONS),
            'description': (
                'pick = move one object. pick_and_place = move one object to a '
                'named destination. reject = the text is not a request to move '
                'a physical object.'
            ),
        },
        'target_query': {
            'type': 'string',
            'description': (
                'The object to grasp, as a bare noun phrase with no verb and no '
                'article: "hammer", "red screwdriver", "blue block". Empty '
                'string when action is reject.'
            ),
        },
        'place_target': dict(
            _nullable_string(),
            description=(
                'Destination as a bare noun phrase ("bin", "wooden tray"). '
                'Null unless action is pick_and_place.'
            ),
        ),
        'modifiers': {
            'type': 'object',
            'properties': {key: _nullable_string() for key in MODIFIER_KEYS},
            'required': list(MODIFIER_KEYS),
            'additionalProperties': False,
            'description': (
                'Attributes the speaker used to distinguish the object. Null '
                'for any attribute they did not mention.'
            ),
        },
        'confidence': {
            'type': 'number',
            'description': (
                'How confident you are that this structure captures what the '
                'speaker wants, 0.0 to 1.0. Use a low value when the object is '
                'unnamed or the request is ambiguous.'
            ),
        },
        'reason': dict(
            _nullable_string(),
            description=(
                'When action is reject, one short sentence a human can read '
                'explaining why. Null otherwise.'
            ),
        ),
    },
    'required': ['action', 'target_query', 'place_target', 'modifiers',
                 'confidence', 'reason'],
    'additionalProperties': False,
}
