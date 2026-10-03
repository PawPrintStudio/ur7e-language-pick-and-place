# Webcam-only language-directed pick on the real arm — lab session 2026-10-02

Goal set at the start of the day: close the loop that every earlier session had left open — a
sentence ("pick up the blue hat") → camera → grasp pose → planned motion → real gripper → object
in the air — on the real UR7e, with whatever camera was at hand and no Jetson. This is the
summary and acceptance record. The chronological log, with every number and every recovery, is
the 2026-10-02 entry in [RUNBOOK.md](RUNBOOK.md); the one-page bring-up is
[COLD_START.md](COLD_START.md); the camera path is explained in
[SINGLE_CAMERA_PERCEPTION.md](SINGLE_CAMERA_PERCEPTION.md).

Setup: laptop in place of the Jetson (the 2026-09-28 pattern — `nmcli con up ur-link`,
192.168.56.1 ↔ .101, PolyScope 5.23), all ROS in container `ur7e-lab-20261002` from
`ur7e-task2:visual` (torch + OWLv2 weights, `--network host --init`, video devices passed
through). The laptop's own webcam gave only black frames (privacy shutter), so the **ZED 2i was
plugged into the laptop and used as a plain USB camera** — no SDK, no GPU: `/dev/video2`,
3840×1080 YUYV side by side at ~19 fps, left half = left lens, factory calibration downloaded
into [calibration/zed2i_SN35717973.conf](calibration/zed2i_SN35717973.conf) (FHD fx 1065.5 px,
8-coefficient rational distortion, baseline 119.891 mm). Camera on a mount in front and left of
the stand, finally about 0.72 m from the base at roughly (0.72, 0.43) m in `base_link`, looking
down about 29°. RG2 on the tool connector, driven over the OnRobot URCap's XML-RPC server.
Nikola at the pendant throughout.

## What was achieved

| Capability | Evidence | Status |
|---|---|---|
| RG2 commanded over XML-RPC **while the External Control program runs** | 60 → 80 → 60 mm at 15:5x, program stayed running | **Verified on hardware** — the pendant hand-off program ([URCAP_HANDOFF.md](URCAP_HANDOFF.md)) is not needed; kept as an alternative |
| Print-free camera calibration (`scripts/lab_camera_calibration.py`: the gripper waves an object, `solvePnPRansac` on the factory intrinsics, table by force) | attempt 3: **3.2 px RMS, 15 of 17 poses**; `map` overlay: (0,0) on the base foot, clamps on their grid cells | **Verified on hardware** (three times — the camera was bumped twice) |
| Table height by force (closed fingertips, wrist F/T) | tool0 z = **0.1485 m**, contact step 0.82 → 39 N between z 0.150 and 0.147, 3 mm steps | **Verified on hardware** |
| `webcam_perception_node` in fixed-camera mode, OWLv2 + colour fallback, near-edge height-free localisation | camera vs robot-placed hat **0.8 cm** ((0.2356, 0.057) vs (0.24, 0.05)); hat footprint 8.0 × 7.4 cm | **Verified on hardware, one object, one placement** |
| Sentence → orchestrator → adapters → motion + gripper (`scripts/lab_pick.py`) | "pick up the blue hat": PARSE / OBSERVE / DETECT (8 s) / LOCATE / PLAN / APPROACH (27 s) / GRASP (71.4 mm) / LIFT / RETREAT (set down) / HOME, **87 s** | **First complete language-directed pick on the real robot** — webcam, no Jetson |
| Protective stop → clean abort (`abort()` → dashboard stop) | exercised 3× | **Exercised on hardware** |
| Stage timeout → abort | exercised 1× (benchmark run 5, 120 s) | **Exercised on hardware** |
| Motion goals that time out are cancelled instead of hanging | benchmark launches 1–3 | **Exercised on hardware** |
| Repeatability benchmark (`scripts/lab_benchmark.py pick`) | 4 software-reported successes (99–102 s each), run 5 aborted | **Ran; does not demonstrate ≥ 80 %** — see the operator verdict |
| `ur7e_orchestrator` package (state machine, adapters, retry policy) | 39 tests; README | Done in software; two retry paths not yet on hardware |
| IK solver comparison for the pick pipeline's Cartesian segments (#38) | [evidence/ik-solver-comparison-2026-10-02.md](evidence/ik-solver-comparison-2026-10-02.md), mock hardware on the calibrated description | Done (not the physical arm) |
| One-page cold start (#28) | [COLD_START.md](COLD_START.md) | Written; a teammate has not yet run it |

Test suites at the end of the day: `test_monocular.py` 16 pass, `test_stereo.py` 15 pass (the
stereo height module from the ZED's second lens exists but is not wired into the node),
`scripts/test_lab_pick.py` 14 pass, orchestrator 39 pass.

## Operator verdict (read this before the benchmark numbers)

The benchmark's "4 of 5" is the **software's** view: a run counted as a success when the jaws
closed on something (width above the empty-jaw threshold) and every stage returned. Nikola,
watching the arm: **only the first pick of the day was a clean grasp. In the later runs the
gripper grazed the hat, gripped it badly, or missed it, while the software still reported
success because its criterion was only "jaws closed on something".** The benchmark therefore
does NOT demonstrate the ≥ 80 % acceptance of task 4.4; its JSON and Markdown under
[evidence/](evidence/pick-benchmark-2026-10-02-blue-hat.md) are the software's record and are
annotated as such.

Two root causes, named by Nikola, are the next work items:

1. **Nothing re-checks the object's position before or during a run.** After a release the
   object can roll (a round hat does). The camera should confirm the object left the table on
   LIFT and re-locate before APPROACH, instead of trusting the pose from the previous DETECT.
2. **Every object is grasped the same way** — top-down, fixed heights and widths. Irregular
   shapes need a grasp and approach estimate per object (where to close, how wide, how high,
   which yaw), ideally from the camera.

## Findings, most consequential first

1. **The table is not one surface.** The robot sits on a raised platform; the pick plate in
   front is a *second* piece about 11 cm below the robot's mounting plane (Nikola: "I added a
   piece of table… offset the end by 10 cm up"), and the clamp area behind it is lower still.
   Earlier force probes at (0.30, 0.12) and (0.33, 0.10) found nothing because those points are
   *beyond the plate's front edge* (x ≈ 0.30); the white plug released there fell on the floor.
   The plate's usable area in `base_link` is about x 0.14–0.30, y −0.07–0.22 — now the default
   `workspace` in `scripts/lab_pick.py` — and `scripts/lab_obstacles.yaml` gives the planner the
   platform (x ≤ 0.14, z −0.12..0), both toggle clamps and a keep-out box around the camera.
   This cost more time than anything else.
2. **`--table-z` is the one number that must be fixed empirically.** The wave heights live in the
   frame "tool0 height when the held object's centre is at the plate", so the unknown
   flange-to-object distance cancels — but a touchdown of the *held* object by force did not
   work (a 10 N grip lets the object slide at 0.2 N). It was fixed by setting the object down at
   a known point and comparing with `scripts/lab_detect.py`: **0.2025** for the hat held by its
   brim. The first guesses (0.1785 / 0.177) put objects 4–6 cm too far from the camera, and one
   of them put the jaws onto the platform edge (protective stop).
3. **Round objects break the "shadow erosion" height model.** It needs the object height and
   over-eroded the hat. The near-edge estimator (`localize_on_table(..., object_height=None)`:
   nearest silhouette edge plus the width across the view, no height needed, about 1 cm on
   rendered scenes) is now the default and gave the hat's footprint as 8.0 × 7.4 cm.
4. **The RG2's fingers slope inward above the pads.** An attempt with 100 mm jaws and 8 mm tip
   clearance stopped (protective stop) because the 85 mm dome met the fingers. All successful
   picks used jaws at 108 mm, 15 N, grasp 25 mm up the dome, 70 % slider.
5. **Gripper telemetry needs reading with care.** `busy` stays on while the jaws squeeze an
   object that settles, and `width` shrinks (hat: 74 → 51 mm) although the object is still held
   — the first drop detector mis-read that and aborted a run with the hat in the air; now a
   drop means width < miss width + 2 mm. Never command empty jaws below ~5 mm: the stall at the
   mechanical stop registers as a grip and the URCap's "grip lost" guard then stops the program
   when the jaws open (happened once). The fingertip safety latch (`s1_triggered`,
   `safety_failed`) tripped once while an object was pushed between the pads and could only be
   reset from the pendant's OnRobot toolbar (tool power cycle also works); afterwards the width
   read −17 mm until the jaws moved once. `depth` is the fingertip retraction at the current
   width (7 mm at 59 mm, 14.8 mm at 80 mm). Flange (tool0) to closed fingertips ≈ 0.258 m,
   derived from the touchdown and the base-foot pixel check.
6. **CycloneDDS runs out of participant indices at the 11th node.** Discovery on this laptop
   goes over loopback by unicast with the default range (~10 per host): "Failed to find a free
   participant index for domain 42". `scripts/lab_env.sh` raises `MaxAutoParticipantIndex` to
   120 and must be sourced in **every** process; the driver had to be restarted for it.
7. **After a driver restart with the program stopped, Play alone leaves the controller
   inactive** ("Can't accept new action goals. Controller is not running"); a
   `switch_controller` activation fixes it. The controller_stopper logged "Could not activate
   requested controllers" once.
8. **OWLv2 on the laptop CPU is usable but slow and prompt-sensitive**: ~8 s per query; "blue
   helmet" scored 0.13–0.20, "blue hat" fell below the 0.10 threshold. The colour fallback for
   `blue` needed a saturation floor of 150: the black anodised breadboard reads as dark navy
   and was once segmented whole as "blue".
9. **A skewed silhouette at the plate's corner produced a 75° wrist turn.** Benchmark run 5: the
   hat had rolled to the plate's left-front corner, the silhouette there looked elongated (axis
   ratio 7.8), the planner turned the wrist to align, the lift stalled near the wrist_3 limit,
   and the orchestrator stopped the program after 120 s (`stage_timeout`). Root cause 1 above.
10. **Benchmark plumbing found by running it**: launch 1 failed at run 1 (`gripper_busy` — fixed
    by waiting out `busy`); launch 2 at run 2 (`motion_blocked`, "arm still moving" right after
    a release — `fresh()` now waits up to 4 s); launch 3 is the one recorded.
11. **Solver choice, on evidence (#38).** On mock hardware with the calibrated description, KDL
    and pick_ik both plan 30/30 at every step size with the retuned yaml; pick_ik's accuracy is
    about 1 mm / 3 mrad against KDL's 0.01 mm; weight 1.0 reproduces the lab's 2 %. The
    recommendation is KDL for the Cartesian segments and keeping the retuned values.
    `revolute_jump_threshold` is ignored by this MoveIt build — only `lab_jog.audit` guards
    joint jumps.

## Not done, stated plainly

- The benchmark does not show ≥ 80 % success (operator verdict above). One clean pick, one
  object, one day.
- Not-found and grasp-miss retries exist in the orchestrator but were not exercised on hardware
  (retries used: 0 and 0).
- The `safety_monitor` stop/recovery drill (#13) was not performed as such; the three protective
  stops were recovered by hand (Unlock → driver restart → Play) and the orchestrator's abort path
  was what got exercised.
- Voice and teleop were not recorded on hardware (#39), unchanged from 2026-09-30.
- Perception validation over ~10 objects (#20), the three-object demo (#26), the 1.7 ten-run
  acceptance (#14) and the 2.5 five-touch verification (#19) were not attempted.
- TCP vs pendant (#10) was not compared; the 0.258 m flange-to-fingertip figure is from today's
  touch, not from the pendant.
- Stereo height from the second lens is tested on rendered pairs but not wired into the node.

## Issues

| Issue | Task | Today | Why |
|---|---|---|---|
| #8 | 1.1 raw RS-485 gripper route | **closed** | XML-RPC alongside External Control makes the raw-RS485 route moot; documented as deferred |
| #11 | 1.4 combined model, table, real `home`/`observe` | **closed** | table geometry measured by force, obstacles in the planner, poses used for a real pick |
| #18 | 2.4 locator: mask → grasp pose | **closed** | 0.8 cm against a robot-placed object; near-edge estimator |
| #23 | 3.3 guardrails | **closed** | validator → policy → `[y/N]` carried through to a real pick |
| #24 | 4.1 orchestrator state machine | **closed** | `src/ur7e_orchestrator`, 39 tests, drove the real pick |
| #30 | 1.8 Gazebo grasp latch | **closed** | latch verified per [TASK2_SOFTWARE.md](TASK2_SOFTWARE.md) ("close, latch and 10 cm lift succeeded") |
| #17 | 2.3 detector runtime | **closed** | OWLv2 on the laptop is the deployed detector; NanoOWL on a Jetson stays an adapter for later |
| #38 | Cartesian planning: pick_ik vs KDL | **closed** | [evidence/ik-solver-comparison-2026-10-02.md](evidence/ik-solver-comparison-2026-10-02.md) |
| #25 | 4.2 failure / retry policy | open | protective stop and stage timeout exercised; not-found and grasp-miss retries not on hardware |
| #26 | 4.3 three-object demo | open | one object, no video |
| #27 | 4.4 repeatability ≥ 80 % | open | not ≥ 80 %; operator verdict above |
| #28 | 4.5 cold-start runbook | open | [COLD_START.md](COLD_START.md) written; a teammate run would close it |
| #13 | 1.6 stop/recovery drill | open | not performed as a drill (see above) |
| #14 | 1.7 ten-run scripted pick | open | not attempted |
| #19 | 2.5 five-touch verification | open | not attempted |
| #20 | 2.6 ~10-object validation | open | not attempted |
| #39 | voice + teleop recording | open | unchanged |
| #10 | 1.3 TCP / payload vs pendant | open | not compared; 0.258 m is from today's touch |
| #15 / #16 | 2.1 / 2.2 Jetson + ZED SDK | open | today's USB-ZED path is a documented alternative; decision pending |

## End state

Robot: program stopped after benchmark run 5's abort; the hat was at the plate's left-front
corner. Calibration file `scripts/lab_table.json` is for the camera's *final* position — any
bump means §3 of [COLD_START.md](COLD_START.md) again (the tips touchdown can be skipped).
Container `ur7e-lab-20261002` holds the build; the code is being packaged for a PR alongside
this record.

## Next session, in order

1. Bring up per [COLD_START.md](COLD_START.md); have a teammate do it from the page alone (#28).
2. Root cause 1: re-locate before APPROACH and confirm departure on LIFT from the camera.
3. Root cause 2: a per-object grasp estimate (close point, width, height, yaw) from the camera,
   at least for the hat and one box-like object.
4. Then re-run `lab_benchmark.py pick` with a human judging each grasp, not the jaw width.
5. Then the queue: not-found / grasp-miss retries on hardware (#25), the stop drill (#13), voice
   and teleop recording (#39), ~10-object validation (#20).
