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

# Jog actions (task 3.2, the command console): motion that needs NO camera.
# The speaker moves the tool itself, not an object. These are the vocabulary
# of the lab demo "go up a bit / go down 2 / spin slowly / go home".
#   move   - translate the tool a bounded distance along one base-frame axis
#   rotate - spin the wrist a bounded angle at a signed speed level
#   go_to  - a named, previously taught pose, optionally plus a move offset
ACTION_MOVE = 'move'
ACTION_ROTATE = 'rotate'
ACTION_GO_TO = 'go_to'
# teleop - the speaker wants the controls themselves ("let me drive it").
# The console hands the terminal to the keyboard jog tool and takes it back
# when that exits. No numbers travel with it; it is a mode, not a motion.
ACTION_TELEOP = 'teleop'
JOG_ACTIONS = (ACTION_MOVE, ACTION_ROTATE, ACTION_GO_TO, ACTION_TELEOP)

# Everything that can make the arm move. A GuardrailPolicy picks a subset of
# this; MOTION_ACTIONS (pick family) stays the default so existing deployments
# do not silently gain jog commands.
ALL_MOTION_ACTIONS = MOTION_ACTIONS + JOG_ACTIONS

# Not a motion action: the backend's way of saying "this was not a request to
# move an object". Letting the model say so explicitly is much safer than
# forcing it to choose a motion action for "what time is it" and hoping the
# confidence score saves us.
ACTION_REJECT = 'reject'

BACKEND_ACTIONS = ALL_MOTION_ACTIONS + (ACTION_REJECT,)

# --- jog vocabulary and bounds ---------------------------------------------
# Directions are in the robot's base frame (base_link): up/down = +/-Z,
# forward/backward = +/-X, left/right = +/-Y. "Left" is the robot's left, not
# the audience's — the console has a flag to mirror it for a demo.
DIRECTIONS = ('up', 'down', 'left', 'right', 'forward', 'backward')

# Distances in centimetres because that is how people say them ("go down 2").
# The bounds are the safety story: no sentence can move the tool further than
# MAX_MOVE_CM in one command, whatever the model wrote.
DEFAULT_MOVE_CM = 5.0     # "go up" with no amount
SMALL_MOVE_CM = 2.0       # "go up a bit"
MAX_MOVE_CM = 20.0

# Speed levels are small signed integers: magnitude 1 (slow) to 3 (fast), sign
# is the direction of spin (+ = counter-clockwise looking at the tool flange,
# i.e. wrist_3 increasing; - = clockwise). The console maps them to joint
# rates well under the driver's velocity ceiling.
MAX_SPEED_LEVEL = 3
DEFAULT_SPEED_LEVEL = 1

# One rotate command turns the wrist a bounded angle, never "forever".
DEFAULT_ROTATE_DEG = 30.0
MAX_ROTATE_DEG = 90.0

# Fields of the `motion` object, in one place so backends, validator and the
# ROS message agree on the names.
MOTION_KEYS = ('direction', 'distance_cm', 'speed_level', 'angle_deg',
               'pose_name')

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
                'named destination. move = translate the robot tool itself a '
                'short distance in a direction (no object involved). rotate = '
                'spin the robot wrist. go_to = drive to a named pose such as '
                '"home". teleop = the speaker asks to control or drive the arm '
                'themselves, manually, by hand or by keyboard. reject = the '
                'text is not a request the arm can act on.'
            ),
        },
        'target_query': {
            'type': 'string',
            'description': (
                'The object to grasp, as a bare noun phrase with no verb and no '
                'article: "hammer", "red screwdriver", "blue block". Empty '
                'string when action is reject, move, rotate or go_to.'
            ),
        },
        'motion': {
            'type': 'object',
            'properties': {
                'direction': {
                    # The API rejects `enum` next to a `["string", "null"]`
                    # type list ("Enum value 'up' does not match declared
                    # type", 2026-09-30), so nullable-enum is spelled anyOf.
                    'anyOf': [
                        {'type': 'string', 'enum': list(DIRECTIONS)},
                        {'type': 'null'},
                    ],
                    'description': (
                        'For move: which way the tool travels. For go_to: an '
                        'optional offset direction from the named pose. Null '
                        'otherwise.'
                    ),
                },
                'distance_cm': {
                    'type': ['number', 'null'],
                    'description': (
                        'How far, in centimetres. "a bit"/"a little" = '
                        f'{SMALL_MOVE_CM}. Null when the speaker gave no '
                        'amount (a default applies); "go down 2" means 2 cm.'
                    ),
                },
                'speed_level': {
                    'type': ['integer', 'null'],
                    'description': (
                        'For rotate: signed integer, magnitude 1 (slow) to '
                        f'{MAX_SPEED_LEVEL} (fast); negative spins the other '
                        'way. "slowly" = 1, "fast" = 3, "at speed -1" = -1. '
                        'Null when unspecified or for other actions.'
                    ),
                },
                'angle_deg': {
                    'type': ['number', 'null'],
                    'description': (
                        'For rotate: how many degrees to turn, if the speaker '
                        'said. Null when unspecified or for other actions.'
                    ),
                },
                'pose_name': dict(
                    _nullable_string(),
                    description=(
                        'For go_to: the named pose as a bare noun phrase '
                        '("home", "start", "observe"). Null otherwise.'
                    ),
                ),
            },
            'required': list(MOTION_KEYS),
            'additionalProperties': False,
            'description': (
                'Parameters of a move, rotate or go_to action. Every key is '
                'null for pick, pick_and_place and reject.'
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
                 'motion', 'confidence', 'reason'],
    'additionalProperties': False,
}
