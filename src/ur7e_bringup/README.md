# ur7e_bringup

One command that brings up our UR7e — against the real robot, mock hardware,
or URSim — with the settings that must never be forgotten baked in.

```bash
# Real robot (or URSim, which deliberately shares the same IP):
ros2 launch ur7e_bringup ur7e_bringup.launch.py

# Sim tier 1 — no robot at all (this is what CI runs):
ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true
```

## The concept: what a "bringup" package is

Every ROS 2 robot deployment has a pile of launch-time configuration: which
driver, which robot model, which calibration file, which controllers, which
IP. A *bringup package* is the project's answer to "how do I start this thing"
— a single launch entrypoint that encodes those choices as defaults, so the
correct invocation is also the shortest one.

The alternative — everyone typing `ros2 launch ur_robot_driver
ur_control.launch.py` with five hand-remembered arguments — fails in a
specific, sneaky way, and we have already lived it:

> **Case study (lab sessions 1–2):** launched without our
> `kinematics_params_file`, the driver happily runs — but computes forward
> kinematics from the *generic* UR7e model instead of our robot's factory
> calibration, and every TCP pose is silently wrong by millimetres. The only
> symptom is a checksum warning scrolling past in the log
> (`calib_12788...` = generic file, `calib_12445...` = our robot). With the
> calibration wired in, the warning becomes "Calibration checked
> successfully." This package exists so nobody has to remember that flag.

What this package deliberately does **not** do: reimplement the driver
launch. It *includes* upstream `ur_control.launch.py` and only pins/forwards
arguments — upstream keeps maintaining the hard parts (controller spawning,
dashboard client, URScript assets), and we can't drift out of sync with them.

## What is pinned, what is a knob

| Setting | Value | Why |
|---|---|---|
| `ur_type` | `ur7e` (pinned, not an argument) | Launching this repo against another arm is always a mistake. |
| `kinematics_params_file` | our factory calibration ([config/ur7e_calibration.yaml](config/ur7e_calibration.yaml)) | The case study above. |
| `robot_ip` | `192.168.56.101` | The lab robot — and the URSim container is given the same IP (see [sim/ursim](../../sim/ursim/docker-compose.yml)), so sim and lab commands are identical. |
| `use_mock_hardware` | `false` | Set `true` for sim tier 1: full controller chain, no robot. (Maps to the Humble driver's older `use_fake_hardware` name internally.) |
| `headless_mode` | `false` | See the evaluation below. |
| `launch_rviz` | `false` | The Jetson runs headless over SSH; set `true` on a workstation. |

## headless_mode vs External Control URCap (task 0.5 evaluation)

Two ways exist for the driver to get its control program running on the robot:

| | External Control URCap + **Play** (our default) | `headless_mode:=true` |
|---|---|---|
| How | A person at the pendant presses Play on a program containing the External Control node | The driver injects the URScript program over port 30003 and starts it itself |
| Arming | **A deliberate human act at the machine** | Remote — the arm can be armed with nobody in the room |
| Pendant mode | Works in Manual (our current setup) | Requires Remote Control mode |
| Recovery after protective stop | Re-press Play | Call `resend_robot_program` service |
| Validated on our robot | **Yes** (lab session 2: "Robot connected to reverse interface") | Not yet |

**Recommendation:** keep the URCap + Play ritual as the default. Its "cost" —
a human must press Play — is exactly the property we want while people share
a room with a moving arm (compare RUNBOOK 4: speed slider low, hand on
e-stop, *then* Play). `headless_mode` stays available as a launch argument
for a future where supervised remote arming is genuinely needed; exercising
it is a lab task, not a default to flip in a repo.

Final sign-off of this choice happens at lab session 3 (issue #5's
acceptance: "one command brings the robot to *ready*" — needs the robot).

## Files

- [launch/ur7e_bringup.launch.py](launch/ur7e_bringup.launch.py) — the single entrypoint; heavily commented
- [config/ur7e_calibration.yaml](config/ur7e_calibration.yaml) — factory kinematics extracted from our robot (lab session 1, `ur_calibration`); do not edit by hand
