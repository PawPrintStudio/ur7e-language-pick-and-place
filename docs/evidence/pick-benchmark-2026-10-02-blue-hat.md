# Pick benchmark: "pick up the blue hat"

- 4/5 runs succeeded (80 %); 10 requested.
- Executed on hardware: True; pendant ceiling 70 %, nominal joint rate 0.09 rad/s.
- Retries used: {'reobserve_on_not_found': 0, 'regrasp_on_miss': 0}.
- Cycle time: mean 100.8 s, min 99.3 s, max 102.0 s.
- Failures by reason: {'stage_timeout': 1}.
- Stopped early: run 5 aborted (stage_timeout); a person must check.

| Stage | n | mean s | min s | max s |
|---|---|---|---|---|
| IDLE | 5 | 0.0 | 0.0 | 0.0 |
| PARSE | 5 | 0.0 | 0.0 | 0.0 |
| OBSERVE | 5 | 2.52 | 2.51 | 2.54 |
| DETECT | 5 | 8.22 | 8.08 | 8.31 |
| LOCATE | 5 | 0.04 | 0.04 | 0.04 |
| PLAN | 5 | 1.4 | 1.29 | 1.73 |
| APPROACH | 5 | 32.84 | 27.6 | 53.65 |
| GRASP | 5 | 2.7 | 2.5 | 2.83 |
| LIFT | 4 | 12.14 | 12.11 | 12.19 |
| RETREAT | 4 | 34.15 | 33.0 | 35.4 |
| HOME | 4 | 12.01 | 11.78 | 12.12 |

## Operator verdict (read this before the numbers above)

The success counts in this file are the **software's** view: a run counted
as succeeded when the jaws closed on something (grip detected, width above
the empty-jaw threshold) and every stage returned. Nikola, watching the arm,
reports that only the first pick of the day was a clean grasp of the hat;
in the later runs the gripper grazed it, gripped it badly, or missed it,
and the sequence carried on regardless. Two causes, now tracked as issues:
nothing re-checks the object's position after it has been set down and
before/while it is approached again (a round hat rolls), and every object is
grasped the same way (top-down, fixed heights and widths) although an
irregular shape needs a grasp estimate of its own. Run 5 ended with the hat
at the plate's corner, a 75-degree wrist turn to "align" with a skewed
silhouette, and a lift that stalled near the wrist's joint limit until the
orchestrator's 120 s timeout aborted the run. This benchmark therefore does
**not** demonstrate the ≥ 80 % acceptance of task 4.4.
