# UR7e motion demos

A graduated ladder of joint-space motion scripts, simplest first, sharing one
safety module ([`ur_motion.py`](ur_motion.py)). Built for the learning-platform
goal: read any one in isolation and understand *how* the arm is commanded.

> **These move a real 7 kg arm.** Keep the pendant speed slider low (10–25 %)
> and a hand on the e-stop for every run. There is **no collision checking**
> here — that arrives with MoveIt in phase 1. Motions are small and *relative*
> to the current pose on purpose. Ramp amplitudes up slowly, watching the arm.

## The one hardware gotcha baked into the design

`/joint_states` does **not** publish joints in anatomical order (observed order:
`shoulder_pan, wrist_2, wrist_3, wrist_1, elbow, shoulder_lift`). Every script
reads `name → position` into a dict and rebuilds trajectories **by joint name**.
Never zip positions by index — that commands the wrong joints and swings the arm.

## Safety envelope (enforced in code, before anything reaches the robot)

`MotionClient.run()` rejects a trajectory that breaks either bound:

| Guard | Default | Why |
|---|---|---|
| `MAX_JOINT_VEL` | 0.10 rad/s nominal | Below the ~0.018 rad/s *velocity veto* seen in lab session 2 once the pendant slider is low; a hard ceiling against gross mistakes. |
| `MAX_JOINT_STEP` | 0.50 rad/waypoint | Sanity net against a single wild sample. |

Nominal = before the pendant slider, which scales the **actual** speed down further.

## The ladder

| Script | Motion | Status |
|---|---|---|
| [`demo_01_nudge.py`](demo_01_nudge.py) | wrist_3 +0.05 rad and back | ✅ ran on real robot 2026-09-21 — run first every session |
| [`demo_02_wave.py`](demo_02_wave.py) | wrist_3 slow sine, 3 cycles | ✅ ran on real robot 2026-09-21; also the safe way to probe the veto |
| [`demo_03_fluid.py`](demo_03_fluid.py) | 6-joint phased sine | ✅ ran on real robot 2026-09-21 — freedrive to an open pose first |

**Roadmap (not yet written — gated on verification):**

- **Fast spin / large-amplitude dance** waits on **Gate A**: deliberately
  characterizing the velocity veto with `demo_02` (raise `AMP` / lower `PERIOD`
  until the pendant vetoes — that speed is the ceiling). Until then everything
  stays slow by design.
- **Big-envelope choreography** (large base/shoulder sweeps) waits on **Gate B**:
  MoveIt collision checking (phase 1), so the arm plans around the table and
  itself instead of trusting hand-tuned amplitudes.

Fluidity today comes from *smoothness and phase*, not speed — that is why the
demos already look expressive while staying under the caps.

## Run

Driver up (`ros2 launch ur7e_bringup ur7e_bringup.launch.py`) and **Play**
pressed on the pendant, then in a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ur7e_ws/install/setup.bash
python3 ~/ur7e_ws/scripts/motion/demo_01_nudge.py   # then 02, then 03 — in order
```

## Pull-and-run loop (internet ↔ robot toggle)

`eno1` is either on the internet **or** on the robot — one cable, one port. To
pull new scripts, stop the driver and flip the profile, then flip back:

```bash
# Ctrl+C the driver first
sudo nmcli con up "Wired connection 1"   # <- your makerspace wired profile (nmcli con show)
cd ~/ur7e_ws && git pull
sudo nmcli con up ur-link                # back to the robot (static 192.168.56.1)
# relaunch the driver, press Play, run
```
