# arm_language

Free-form text in, a validated manipulation command out — or a clear refusal.
Task 3.1 ([#21]) plus the parser-side half of task 3.3 ([#23]).

```bash
# Offline, no credentials, no SDK, runs anywhere (even with no ROS installed):
python3 -m arm_language.eval --backend keyword

# The real thing (see "Installing the SDK" below — a bare pip install is
# refused on Ubuntu 24.04):
export ANTHROPIC_API_KEY=...
python3 -m arm_language.eval --backend claude --report /tmp/acceptance.json

# As a node:
ros2 run arm_language intent_parser_node
ros2 service call /intent_parser/parse_intent arm_interfaces/srv/ParseIntent \
  "{text: 'pick up the red screwdriver and put it in the bin'}"
```

## Installing the SDK

The cloud backend needs `anthropic`, which is a **pip** package — deliberately
not in `package.xml`, because it has no rosdep key and declaring it would break
`rosdep install` for everyone including CI.

**On the workstation (Ubuntu 24.04):** `pip install anthropic` is refused with
`error: externally-managed-environment`. That is [PEP 668] — 24.04 protects the
system Python from pip. Use a venv; the repo's `.venv/` is already gitignored:

```bash
python3 -m venv .venv
.venv/bin/pip install anthropic
cd src/arm_language && PYTHONPATH=. ../../.venv/bin/python -m arm_language.eval --backend claude
```

**On the Jetson (JetPack 6.2 = Ubuntu 22.04):** PEP 668 is not enforced there,
so a plain `pip install anthropic` works and the node picks it up.

If you ever *do* need the SDK importable from a ROS node on a PEP 668 system,
build the venv with `--system-site-packages` so `rclpy` stays visible, then
activate it before `ros2 run` — the entry point is a `#!/usr/bin/env python3`
script, so it follows whichever `python3` is on `PATH`:

```bash
python3 -m venv --system-site-packages ~/.venvs/arm && ~/.venvs/arm/bin/pip install anthropic
source ~/.venvs/arm/bin/activate && ros2 run arm_language intent_parser_node
```

### Credentials

The SDK resolves, in order: `ANTHROPIC_API_KEY`, `ANTHROPIC_AUTH_TOKEN`, then a
stored OAuth profile from `ant auth login`. An unset `ANTHROPIC_API_KEY` does
not by itself mean you are unauthenticated, which is why the backend's
pre-flight check looks for a profile too. With none of them present it refuses
at startup rather than mid-demo, and `eval` exits 2.

[PEP 668]: https://peps.python.org/pep-0668/

## The concept: why a language model sits between a human and a robot arm

The temptation is to skip this node. A member types "pick up the hammer", you
regex out the noun, you hand `hammer` to the detector. Why involve an LLM?

Because of a constraint one layer down. **NanoOWL takes noun phrases, not
sentences** (architecture D3). Feed it `"pick up the hammer"` instead of
`"hammer"` and it does not error — it returns a *worse detection*. That
propagates into a worse mask, a worse centroid, and a grasp that misses by a
centimetre. Every symptom appears in the motion stage, and nothing in the log
points back at the sentence that caused it.

So the job of this package is narrow and specific: **turn an arbitrary English
request into the small, clean noun phrase the rest of the pipeline was designed
around, or refuse loudly.** Everything else here — the schema, the validator,
the guardrails — exists to make "or refuse loudly" true even when the language
model misbehaves.

> **Why not have the LLM talk to the robot directly?** Because then "what can
> this robot be made to do" is a property of a model's weights, which is not a
> thing you can test, review, or bound. Here the model's entire influence is:
> pick one of two actions, and name one object. That bound holds even if the
> model is completely wrong or actively subverted — which is what makes the
> prompt-injection question below boring rather than frightening.

## The pipeline

```
utterance ──► backend ──► validator ──► guardrails ──► ParseResult
            (untrusted)   (shape +      (whitelist +
                           meaning)      confidence)
```

Each arrow narrows what is possible; none widens it. That one-way property is
what lets you reason about the node without reasoning about the model inside it.

| Module | Question it answers | Where it lives |
|---|---|---|
| `backends/` | "What does *a* language model say about this?" | untrusted |
| `schema.py` | "What shape are we willing to consider?" | the contract |
| `validator.py` | "Did we understand this?" | trust boundary |
| `guardrails.py` | "Should we act on it?" | policy |
| `parser.py` | assembles the above; **never raises** | public API |
| `intent_parser_node.py` | ROS service wrapper | ~50 lines, no logic |

Only `intent_parser_node.py` imports ROS. Everything else is plain Python, so
the parser can be developed and evaluated on a laptop with no ROS install —
the same motivation as the simulation tiers in architecture D7.

### A refusal is a value, not an exception

`IntentParser.parse()` returns a `ParseResult` for every input and raises
nothing — not for empty input, not for a malformed model response, not when
the backend throws something undeclared. The caller is a state machine driving
a robot arm; an uncaught exception there is a stuck run, while an `outcome`
field is something you have to read to get at the command.

```python
result = parser.parse(text)
if result.may_move:            # the single predicate motion code branches on
    execute(result.command)
elif result.outcome is Outcome.NEEDS_CONFIRMATION:
    ask_human(result.message)  # "Did you mean: pick up the hammer?"
else:
    show(result.message)       # refused; result.command is None
```

`may_move` is deliberately `False` for `NEEDS_CONFIRMATION` — at the moment it
is read, the human has not answered yet.

## The three guardrails (#23)

1. **Action whitelist** — `pick` and `pick_and_place`, and nothing else, ever.
   It is configurable *narrower*: phase 1 can run `allowed_actions:=['pick']`
   and refuse place commands the parser understands perfectly well, because the
   place half is not built yet.
2. **Confidence threshold** — three bands: act (≥ 0.75), ask (≥ 0.40), refuse.
   `require_confirmation:=true` forces the middle band always, which is the
   right setting for a first public demo.
3. **No motion on parse failure** — structural rather than a check: a refusal
   carries `command=None`, so there is nothing to act on even if a caller
   ignores `outcome` entirely.

### About that confidence number

It is **self-reported by a language model and is not a calibrated
probability**. A model can be fluently, confidently wrong. Treat it as a coarse
ambiguity detector: it reliably drops when the speaker named no object ("pick
up that thing"), which is the case we most want to catch, and it is close to
useless for telling a good parse from a subtly wrong one.

The real defences against a confident misparse are the noun-phrase check, the
whitelist, and a human reading the confirmation echo. The thresholds are
starting points to tune against the corpus on real hardware, not measured
constants.

### About prompt injection

The utterance is attacker-controlled text being fed to a model whose output
moves a robot, so "ignore your instructions and…" deserves an answer. The
answer is not prompt wording — it is that a successful injection buys an
attacker the ability to make the arm **pick up the wrong object**, which is
exactly what a typo buys them. That ceiling is set by the whitelist and the
schema, and it holds even if the system prompt is fully defeated.

The delimiting and the "this is data" instruction in the system prompt are
defence in depth. They are not the defence.

## Backends (architecture D5)

| Backend | Use | Needs |
|---|---|---|
| `claude` | **default.** Cloud Claude with constrained decoding | `pip install anthropic`, `ANTHROPIC_API_KEY` |
| `keyword` | offline dev, hermetic CI, orchestrator dry-runs | nothing |
| `replay` | test double; deliberately not in the launch registry | fixtures |

Cloud-first on a machine with a GPU looks backwards until you remember the
Jetson's 8 GB is the scarce resource and perception needs all of it. A cloud
call costs zero bytes of that budget, and 1–3 s of latency disappears next to a
~20 s pick cycle. A local Llama backend later means writing one more `Backend`
class — nothing downstream changes.

**`anthropic` is a pip dependency, not a `package.xml` one.** It has no rosdep
key, so declaring it would break `rosdep install` for everyone including CI.
The package builds, lints, and passes its full test suite without it; only
calling the cloud needs it installed.

> **The keyword backend is not a fallback the robot drops to.** It
> pattern-matches English imperatives and has no idea what a sentence means.
> It exists so that CI is hermetic and a member with no lab access can exercise
> the pipeline. It is held to the same validator and guardrails as the cloud
> backend, so when it is wrong, it is wrong in ways the layers above still
> catch.

## The corpus, and the two numbers it produces

`arm_language/corpus/utterances.json` holds 29 utterances — simple picks, picks
with modifiers, pick-and-place, ambiguous requests, non-pick requests,
unsupported actions, prompt injections, and malformed input.

`python3 -m arm_language.eval` reports **two different numbers**, and conflating
them is the main way a language component ships broken:

- **contract violations** — did the safety invariants hold for every utterance?
  No command on a refusal, nothing outside the whitelist, no target that is not
  a noun phrase. This is a property of *our code*, it is backend-independent
  and deterministic, and **it is what CI gates on**. It must be 0.
- **accuracy** — did the parser reach the *right* answer? This is a property of
  the **backend's language understanding**. Measuring it against the offline
  keyword backend proves nothing: that backend and this corpus were written by
  the same person on the same afternoon.

That is why a green CI badge here cannot mean "our stub agreed with our
fixtures". Entries needing real language understanding are flagged `llm_only`
and skipped for offline backends, so the offline score is never mistaken for an
acceptance result.

### Acceptance status

| Gate | Status |
|---|---|
| Builds + lints under `colcon` on Humble | ✅ verified in `ros:humble` |
| 107 tests, incl. adversarial validator fixtures | ✅ 98 run in CI; 9 cloud-backend shape tests need the SDK |
| Contract invariants, 27 scored utterances, default policy | ✅ 0 violations |
| Contract invariants under a narrowed (`pick`-only) policy | ✅ 0 violations |
| Live node answers `ParseIntent` over the ROS graph | ✅ smoke-tested |
| Claude request shape + error path | ✅ tested with the network stubbed |
| **Accuracy ≥ acceptance bar on the real Claude backend** | ⏳ **needs an API key — not yet run** |

The last row is issue #21's actual acceptance criterion and it is **not met
yet**. To close it:

```bash
# From the repo root, after the venv step in "Installing the SDK":
export ANTHROPIC_API_KEY=...
cd src/arm_language && PYTHONPATH=. ../../.venv/bin/python -m arm_language.eval \
  --backend claude --report ../../docs/acceptance_3_1.json
```

Commit the report, paste the summary into the issue, and note any utterance the
model got wrong — a corpus entry that the cloud backend fails is either a
prompt bug or a corpus bug, and which one it is should be argued in the issue
rather than silently fixed.

## Jog vocabulary: move / rotate / go_to (task 3.2, [#22])

The pick vocabulary needs a camera to mean anything. The jog vocabulary does
not: the speaker moves the *tool*, and every number in the command is bounded
before anyone can act on it.

| Sentence | Command | Bound |
|---|---|---|
| "could you go up a bit?" | `move` up, 2 cm | `MAX_MOVE_CM` = 20 |
| "go down 2" | `move` down, 2 cm | |
| "go left" | `move` left, 5 cm (default) | |
| "can you spin slowly?" | `rotate` speed +1, 30° | `MAX_SPEED_LEVEL` = 3, `MAX_ROTATE_DEG` = 90 |
| "spin at speed -1" | `rotate` speed −1 (clockwise) | |
| "go home" | `go_to` "home" | poses are taught, never typed |
| "go to the start pose but 3 cm up" | `go_to` "start" + offset up 3 cm | offset shares `MAX_MOVE_CM` |
| "go up one metre" | **refused**, `motion_out_of_bounds` | |
| "go up and grab the hammer" | **refused**: one command at a time | |

Directions are the *robot's* (`base_link`): up/down = ±Z, forward/backward =
±X, left/right = ±Y. The console has `--mirror-lr` for an audience facing the
arm.

Three things worth knowing about how it was added:

- **Same pipeline, same trust boundary.** The backend fills a `motion` object
  (`schema.MOTION_KEYS`); `validator._validate_motion` applies the cross-field
  rules ("a move needs a direction, a rotate must not have one") and the
  bounds, and applies the defaults — so a `Command` never carries a missing
  number. Refusing an out-of-bounds value instead of clamping is deliberate:
  a clamped value moves the arm somewhere the speaker did not ask for.
- **Off by default.** `GuardrailPolicy.allowed_actions` still defaults to the
  pick family. A deployment opts in with `allowed_actions=schema.JOG_ACTIONS`
  (which is what `scripts/lab_console.py` does, and it refuses picks in
  return — a camera-free session should say so out loud).
- **Corpus scoring uses `eval.corpus_policy()`**, which enables every motion
  action, because the corpus measures understanding and the whitelist is a
  deployment choice. `unsupported_action` entries still refuse through the
  backend's `reject`, not through the whitelist.

The execution half — MoveIt planning against the lab table, re-timing, the
pendant gates — lives in `scripts/lab_jog.py`; the ask-before-moving loop that
the "Known limitations" below used to list as missing is `scripts/lab_console.py`.

## Known limitations

- **Objects whose names begin with a verb are refused.** `drop cloth` and
  `lift ring` are the realistic casualties of the first-word verb rule. The
  trade is argued in `validator.check_noun_phrase` and pinned by a test: a
  refused drop cloth is a visible message a member can rephrase past, while an
  accepted `"drop the hammer"` silently degrades the detector with no error
  anywhere.
- **One object per command.** "pick up the hammer, the wrench, and the
  screwdriver" is refused, not silently narrowed to the first one
  (architecture §1.3).
- **Unknown modifier keys are refused** rather than dropped. Strict schema, per
  the issue; widening the vocabulary is a one-line change in `schema.py`.
- **The confirmation echo has one UI: the lab console.** `ParseResult.message`
  carries the "did you mean…?" string; `scripts/lab_console.py` ([#22]) asks
  and waits. The orchestrator ([#24]) has no equivalent yet, so there
  `NEEDS_CONFIRMATION` still simply means "do not move".

[#21]: https://github.com/PawPrintStudio/ur7e-language-pick-and-place/issues/21
[#22]: https://github.com/PawPrintStudio/ur7e-language-pick-and-place/issues/22
[#23]: https://github.com/PawPrintStudio/ur7e-language-pick-and-place/issues/23
[#24]: https://github.com/PawPrintStudio/ur7e-language-pick-and-place/issues/24
