# A validated manipulation command — the output of `intent_parser` and the
# input the orchestrator's PARSE stage hands to the rest of the pipeline.
#
# This message only ever carries commands that ALREADY passed validation and
# guardrails. An unparseable or refused utterance never becomes a Command; it
# comes back as a refusal on ParseIntent.srv instead. That is the structural
# form of "no motion on parse failure" (issue #23): there is no such thing as
# an invalid Command on the wire.

# The actions the arm is allowed to perform. The whitelist lives here, in the
# interface, so every consumer sees the same closed vocabulary — an LLM cannot
# widen it by inventing a verb.
string ACTION_PICK=pick
string ACTION_PICK_AND_PLACE=pick_and_place
# Jog actions (task 3.2 command console): the tool moves, no object involved.
string ACTION_MOVE=move
string ACTION_ROTATE=rotate
string ACTION_GO_TO=go_to

string action

# The object to look for, as a bare noun phrase ("hammer", "red screwdriver").
# NanoOWL consumes this string directly (architecture D3), which is the whole
# reason this pipeline exists: it takes noun phrases, not sentences.
string target_query

# Where to put the object, as a noun phrase. Empty for ACTION_PICK; required
# and non-empty for ACTION_PICK_AND_PLACE.
string place_target

# Descriptive attributes the speaker supplied ("color", "size", "material",
# "position"), as a JSON object string — e.g. {"color": "red"}.
#
# Why JSON-in-a-string rather than a KeyValue[] array: modifiers are advisory
# metadata for phase-5 re-ranking, not control inputs. Nothing in the motion
# path reads them. A JSON blob keeps the schema in ONE place (schema.py) rather
# than duplicating the key vocabulary into an .msg that must be rebuilt on
# every change. Revisit if modifiers ever start driving behaviour.
string modifiers_json

# The backend's self-reported confidence, 0.0–1.0. Coarse ambiguity signal
# only — see the confidence discussion in the arm_language README.
float32 confidence

# --- jog parameters (only meaningful for ACTION_MOVE / ROTATE / GO_TO) ---
# Already validated and bounded by arm_language (schema.MAX_MOVE_CM etc.) and
# defaults already applied: a consumer never has to guess a missing number.
# Empty string / 0 when the action does not use the field.
string direction        # up, down, left, right, forward, backward (base frame)
float32 distance_cm     # move length, or go_to offset length
int8 speed_level        # rotate: -3..3, sign = spin direction, never 0
float32 angle_deg       # rotate: bounded turn angle
string pose_name        # go_to: taught pose name, e.g. "home"
