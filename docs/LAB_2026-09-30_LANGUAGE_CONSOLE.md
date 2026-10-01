# Language console on the real arm — lab session 2026-09-30

Goal set at the start of the day: a live-demonstrable demo with no camera in
the loop — plain-English requests ("could you go up a bit?", "go down 2",
"spin at speed -1", "go to this pose plus an offset") turned into motion on
the UR7e. This is the summary and acceptance record. The chronological log,
with every number, is the 2026-09-30 entry in [RUNBOOK.md](RUNBOOK.md); the
show script is [DEMO_CONSOLE.md](DEMO_CONSOLE.md).

Setup: laptop in place of the Jetson (the 2026-09-28 pattern — `ur-link` on
`enp7s0`, container `ur7e-lab-20260930` from `ur7e-dev:latest`, host
networking, domain 42), arm-only driver, empty RG2 attached, Nikola at the
pendant throughout.

## What was achieved

| Capability | Evidence | Status |
|---|---|---|
| Sentence → bounded command (`move` / `rotate` / `go_to` / `teleop`) through backend → validator → guardrails | 178 tests incl. lint; corpus 47 entries, keyword 100 % of its scope | Done |
| Claude backend on the new vocabulary | 46/47, 0 contract violations, `docs/language-evidence/claude-corpus-acceptance-2026-09-30-jog.json` | Done |
| Command → collision-checked plan → trajectory on the arm | `scripts/lab_jog.py` + `scripts/lab_console.py`; 11-sentence script 11/11 from `ready`, displacements matched to 0.1 mm | **Verified on hardware** at 10, 25, 50, 70 % slider; individual commands at 100 % |
| Planned folded-pose → open-pose move (`go to ready`) | 1.68 rad wrist_1 swing, table-checked, tool landed on the seeded pose | **Verified on hardware** |
| Execution gates (safety mode, program running, slider ≤ approved ceiling) | Gate refused five commands when the slider was raised past the ceiling mid-run | **Exercised on hardware** |
| Typed LLM console used by the operator | Nikola drove the arm interactively with the Claude backend, including a typo-ridden line parsed correctly at 0.97 confidence | **Verified on hardware** |
| "Let me drive it" → keyboard teleop hand-over | Parse, gate and plan-only path verified; the first live attempt was blocked by a gate bug (fixed, fix verified against live telemetry) | Built; **hand-over itself not recorded on hardware** |
| Voice input (push-to-talk and hands-free) | Mic capture, Whisper transcription (~1 s per utterance), file bridge into the container, console parse/plan of an inbox line | Built; **no spoken command was recorded reaching the arm** |
| Devcontainer reproducibility | SDK, brotli, pytest baked into the Dockerfile; pip line verified on a clean `ros:humble`; API key forwarded | Done (image not yet rebuilt) |
| Recovery from a link loss / robot reboot | Link back, stale processes cleared, driver + MoveIt relaunched, joint states live | **Verified** (Play and first command after recovery not recorded) |

Issue #22 (3.2 command console) closed against the hardware evidence.

## Findings, most consequential first

1. **pick_ik cannot do millimetre Cartesian stepping reliably.** With the
   committed tuning, a 5 cm lateral Cartesian path reached 2 % from every
   pose; with a lower displacement weight it flipped between 4 % and 100 %
   on encoder noise in the start sample; with a tighter accept threshold it
   reached 0 %. KDL planned the same paths 18/18. The console uses KDL
   (`lab_arm_moveit.launch.py ik:=kdl`). **Open consequence:** `ur7e_motion`'s
   Cartesian primitives (approach, descend, lift, retreat) run on pick_ik and
   were only ever exercised on mock hardware and Gazebo. They need the same
   test on the real description before the first physical pick.
2. **`compute_cartesian_path` needs `max_step` = 1 mm.** 2 mm reached 9 %,
   5 mm 0 %, of the same 2 cm lift.
3. **The cold folded rest pose sits on the shoulder singularity.** "Go right"
   has no solution there, and "go left" costs 0.63 rad of shoulder_pan for
   5 cm. The seeded `ready` pose (`scripts/lab_poses.json`) reaches all six
   directions for about 0.2 rad per 5 cm. `go to ready` replaces the
   "freedrive to an open posture first" step every earlier session carried.
4. **No velocity veto at any slider setting.** Nominal joint rates of
   0.03–0.05 rad/s ran clean at 10 / 25 / 50 / 70 / 100 %. The session-2
   "0.018 rad/s" mystery did not reproduce; nothing faster was tried.
5. **Wall time is nominal × 100 / slider, and a dead client is not a stopped
   arm.** A 120 s client timeout killed the console during a 200 s move at
   10 %; the robot finished the goal regardless.
6. **A gate must spin before it judges.** The teleop hand-over read the speed
   and program-state fields before the node had spun and refused a healthy
   robot with "no live telemetry". `gate()` now listens first.
7. **A restarted console must not re-teach `home`.** It did at first, and
   "go home" went, correctly, to the wrong place.
8. **Structured-output schema:** the API rejects `enum` next to a
   `["string", "null"]` type list; nullable enums are spelled `anyOf`.
9. **SDK in the container:** apt's `python3-brotli` 1.0.9 breaks the SDK's
   response decompression (reported as a connection error while `curl`
   returns 200); the SDK's anyio also needs pytest ≥ 7. Both are in the
   Dockerfile now.
10. **Whisper invents commands from room noise** when its prompt is biased
    toward the command vocabulary: 3 s of ambient audio came back as
    "Go up." The client drops near-silence, offers `--wake WORD`, and the
    console's typed confirmation remains the actual guard. The hands-free
    listener's first live utterance was a bystander's sentence, transcribed
    accurately — a public demo should use the wake word.
11. **Link loss at 16:59.** `enp7s0: Link is Down`, NO-CARRIER, robot found
    rebooted and in a different pose. Carrier is electrical; nothing on the
    laptop removes it. Second such event in two sessions (2026-09-28 was a
    facility outage). Worth checking the controller's power feed before a
    public demo.

## Not done, stated plainly

- No spoken command was recorded moving the arm, and the teleop hand-over
  was not recorded running on hardware after its gate fix. Both are built,
  both passed every check short of that, and both are the first five minutes
  of the next session.
- No gripper, no camera, no object. `pick` and `pick_and_place` parse and are
  refused by policy in this console.
- The protective-stop recovery drill (#13) was not performed. No protective
  stop occurred.
- `ur7e-dev:latest` was not rebuilt from the updated Dockerfile; the SDK
  lived in the session container, which has since exited.
- `mod-003` in the corpus ("wooden mallet": model says `wood`, corpus says
  `wooden`) is a pre-existing miss, not addressed.

## End state

The laptop left the lab: its link to the robot is down and the session
container has exited. The robot's own state at departure (program, slider,
power) was not recorded from the laptop side. Everything is on `main`;
CI green. Taught poses in `scripts/.console_poses.json` are from before the
reboot — start the next console with `--forget-poses`.

## Next session, in order

1. Bring up as in [DEMO_CONSOLE.md](DEMO_CONSOLE.md); `go to ready` if folded.
2. Say one command through `voice_input.py --auto --wake robot` and confirm
   it; ask for teleop and jog one joint. Record both. (Closes the two "built,
   not recorded" rows above.)
3. Run `ur7e_motion`'s Cartesian primitives against the real description
   with pick_ik and with KDL; pick the solver on evidence (finding 1).
4. Then the existing queue: gripper bridge into the combined bringup (#8,
   #14), TCP and payload validation (#10), protective-stop drill (#13).
