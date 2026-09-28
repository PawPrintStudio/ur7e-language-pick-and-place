# Task 3: LLM intent parser, integrated

Implemented on `codex/task2-camera-free-perception`, 2026-09-28, by merging
branch `experiment/3.1-intent-parser` (built 2026-09-22, never previously
merged) and wiring its output into Task 2's Gazebo pick pipeline. No camera,
Jetson, robot, or lab access is required for anything below; one step uses a
live Claude API call (opt-in, needs `ANTHROPIC_API_KEY`).

## What each task delivers

| Task | Software delivered | Remaining physical acceptance |
|---|---|---|
| 3.1 Intent parser | `arm_language`: backend -> validator -> guardrails -> `ParseResult` pipeline; `arm_interfaces/srv/ParseIntent`; Claude backend (constrained JSON-schema decoding) plus an offline `keyword` backend for CI/lab-less development | None — task 3.1's acceptance bar (a test set parsing correctly, incl. rejections) is met in software; see Recorded validation |
| 3.3 Guardrails (parser side) | Action whitelist, two-threshold confidence bands (accept / confirm / refuse), "did you mean" echo built from the parsed structure (not the raw utterance), `require_confirmation` policy knob | The motion-side half of #23 (echoing to a physical console/speaker before a real pick) is out of scope here |
| 3.2 Command console | Not built | Open — no CLI/console node watching workflow stage progress yet |
| Language -> pick integration | `scripts/pick_place_language_demo.py`: free text -> `IntentParser.parse()` -> branches on `outcome` -> `PerceptionPick.run_query()` (Task 2's Gazebo pipeline, reused unchanged) | Real-object/real-robot runs; `pick_and_place` is understood by the parser but refused by this demo's guardrail policy, since there is no place stage downstream of `run_query`'s LIFT |

## Recorded validation

| Check | Result |
|---|---|
| Merge of `experiment/3.1-intent-parser` | Clean except `.github/workflows/ci.yml` (both branches had touched it); resolved by hand, keeping both branches' additions |
| `colcon build` — full 15-package workspace incl. `arm_interfaces`, `arm_language` | Green |
| `colcon test --packages-select arm_interfaces arm_language` | 111 tests, 0 errors, 0 failures, 9 skipped (the 9 are `test_claude_backend.py` cases that skip without live credentials — expected) |
| Offline corpus eval, `--backend keyword` (no credentials) | 27/27 scored = 100%, 0 safety-contract violations (2 corpus entries need real language understanding and are marked `llm_only`, correctly skipped by this backend) |
| Live corpus eval, `--backend claude` | 29/29 scored = 100%, 0 safety-contract violations — exceeds task 3.1's 20-utterance acceptance bar; includes a politely-phrased request ("Can you pick up the tape measure?") the offline backend cannot handle. Full report: [`language-evidence/claude-corpus-acceptance.json`](language-evidence/claude-corpus-acceptance.json) |
| Gazebo, scenario REFUSED | `"what's the weather today"` -> `refused/llm_rejected`, no ROS connection ever opened, no motion. [`language-evidence/1-refused.log`](language-evidence/1-refused.log) |
| Gazebo, scenario NEEDS_CONFIRMATION | `"pick up the red block"` under a `require_confirmation` policy -> echoed `"Did you mean: pick up the red block?"`, declined at the prompt -> no motion, exit 1. [`language-evidence/2-declined.log`](language-evidence/2-declined.log) |
| Gazebo, scenario ACCEPTED (live Claude) | `"could you grab the red block for me"` -> Claude parses `target_query="red block"` -> full `PARSE->OBSERVE->DETECT->LOCATE->PLAN->DESCEND->GRASP->LIFT` sequence executes against real Gazebo physics, `motion_actions_succeeded: true`, exit 0. [`language-evidence/3-accepted.log`](language-evidence/3-accepted.log) |

The synthetic/simulated nature of these results carries the same caveats as
[Task 2's](TASK2_SOFTWARE.md): the Gazebo grasp latch stands in for contact,
not proof of grasp quality, and `red block` is the fixture/OWLv2 vocabulary
already validated there. What is new here is specifically the language layer:
a sentence that the old `pick <noun phrase>` grammar could never parse
("could you grab the red block for me" has no `pick ` prefix) now reaches the
same proven pipeline through a real LLM call, with all three guardrail
outcomes (accept / confirm / refuse) exercised against the running sim.

## Start without a camera or the lab

```bash
# Offline, no credentials, no SDK, runs anywhere:
cd src/arm_language && PYTHONPATH=. python3 -m arm_language.eval --backend keyword

# The real thing (needs `anthropic`; see src/arm_language/README.md for the
# PEP 668 venv workaround on Ubuntu 24.04 workstations):
export ANTHROPIC_API_KEY=...
cd src/arm_language && PYTHONPATH=. python3 -m arm_language.eval --backend claude
```

Full Gazebo run (needs the devcontainer / a built workspace, per Task 2's own
instructions):

```bash
ros2 launch ur7e_perception gazebo_demo.launch.py backend:=fixture
# Second terminal, same environment:
python3 scripts/pick_place_language_demo.py --execute-simulation \
  --backend keyword "what's the weather today"                       # refused
echo n | python3 scripts/pick_place_language_demo.py --execute-simulation \
  --backend keyword --require-confirmation "pick up the red block"   # declined
python3 scripts/pick_place_language_demo.py --execute-simulation \
  --backend claude "could you grab the red block for me"             # accepted, real pick
```

`pick_place_language_demo.py` is a new, separate script from
`perception_pick_demo.py` (Task 2's `pick <noun phrase>` grammar driver) — the
two are kept apart on purpose; see the new script's module docstring.

## An environment defect found and fixed (not a code defect)

The `ur7e-dev` devcontainer image's system Python (3.10, Ubuntu 22.04) has an
apt-installed `Brotli==1.0.9` that is incompatible with the `httpx2` version
the `anthropic` SDK pulls in — `_decoders.py` calls the decompressor's
`process()` with a keyword argument the old binding rejects, which `httpx`2
wraps into a generic `anthropic.APIConnectionError: Connection error.` with no
indication of the real cause. Fix: `pip install --ignore-installed --upgrade
Brotli` inside that environment before using the `claude` backend there (the
repo's own host-side `.venv`, used by `src/arm_language/README.md`'s
instructions, has no system Brotli and is unaffected). Worth an upstream apt
package pin or a documented note if this environment is used for the `claude`
backend again — this report is that note.

## Primary references

* [arm_language/README.md](../src/arm_language/README.md) — SDK install, PEP 668 workaround, running as a node
* [ARCHITECTURE.md D5](ARCHITECTURE.md) — the language layer's design decision
* [TASK2_SOFTWARE.md](TASK2_SOFTWARE.md) — the perception pipeline this integrates with
