# Lab runbook 2026-10-08: table-board calibration + real end-to-end picks

> **Corrections from running it (see [LAB_2026-10-08_REAL_E2E.md](LAB_2026-10-08_REAL_E2E.md)):**
> - Objects go on the **plate beside the board**, never on the board; the
>   default workspace now covers that plate. `--verify-marker` needs
>   `--workspace 0.29 0.13 0.55 0.32` (the board).
> - This board's marker IDs are **0–16**; there is no 42.
> - Teach `observe` above the board edge nearest the robot before the first pick.
> - Do **not** add `--tip-clearance 0.025` for the tape: it lifts the
>   fingertips above an 18 mm roll. Defaults (`--surface-drop 0.015`) worked.
> - Pass `--device 0` (or whichever `/dev/video*` is the ZED) and the
>   measured `--square-mm` to every `lab_table_calibration.py` command.
> - Console: add `--joint-rate 0.09 --max-excursion 2.5` to match `lab_pick`.

You run every command. Each step says **why**, **what you should see**, and
**what to do if you don't**. Stop at any ☐ checkpoint that fails and find the
cause before you go on; don't retry the same move blind.

## The order, and why it isn't "calibrate first"

| # | Step | Moves the arm? | Why it's here |
|---|------|----------------|---------------|
| A | Bring-up (power, link, container, driver, MoveIt) | no | everything below needs it |
| B | **Verify `lab_tooling.yaml`** against the pendant | no | hard gate: `--execute` is refused while `verified: false`, and the board touches compute the fingertip *from this file* |
| C | Table-board calibration (camera, aim, 4 touches, fit, check) | you jog it on the pendant | gives perception the board → `base_link` transform |
| D | Perception up + detect-only checks | no | proves the camera finds each object before the arm moves |
| E | `verify-marker`: fingertips over a board marker | yes, slow | checks camera + board + tool model together, before any grasp |
| F | Picks: plan-only → 25 % → 75 % | yes | the RG2-down pick (wrist tilted ~60° for the Dual Quick Changer) has **never run on the real arm**, only in Gazebo |

About **B before C**: `lab_table_calibration.py touch` reads `tool0` from TF and
finds the fingertip through `lab_tooling.tool0_to_tips()`. If `changer_yaw_deg`
is wrong, every touch lands in the wrong place (old touches were about 0.2 m
off) and the fit either gets refused or, worse, looks fine but is shifted.

About **75 %**: you'll get there, but not on the first run.
[[arm-motion-preview-first]] says ≤ 25 % for a new motion. The rule came from
the 2026-10-06 stops, which happened at 75 %. Gazebo checked the logic, but it
can't check that the real changer angle matches the model.

---

## A. Bring-up (≈10 min)

1. Pendant: **Power on → Brake release** (RUNNING). Load `ros2_external_control.urp`.
   **Set the slider to 25 %.**
2. Laptop:
   ```bash
   nmcli con up ur-link
   ```
   ```bash
   ping -c 3 192.168.56.101
   ```
   ☐ Replies come back. If not, check the cable and the pendant network settings before going further.
3. **Dev container.** Since 2026-10-08 `.devcontainer` is hardware-ready:
   - The ZED is passed in: the video cgroup rule, the `video` group and a `/dev` bind.
   - The image has CPU `torch` and `transformers<5`, which OWLv2 needs.
   - The OWLv2 weights live in the `ur7e-hf-cache` volume, so they survive rebuilds.

   After pulling these changes, run **Dev Containers: Rebuild Container** once.
   It needs about 3 GB free on `/`. Then, inside the container:
   ```bash
   ls /dev/video*
   ```
   ☐ The ZED's nodes are listed (it was `/dev/video2` for the 3840×1080 side-by-side stream). If nothing shows, replug the ZED into a USB-3 port; there's no need to rebuild.
   ```bash
   python3 -c "import cv2, cv_bridge, torch, transformers; print(cv2.__version__, torch.__version__, transformers.__version__)"
   ```
   ☐ It prints three versions with no error. A numpy error here means pip upgraded numpy past 1.x.

   Also rebuild the workspace once if `install/` has no `ur7e_perception`:
   `colcon build --symlink-install`. The first perception start downloads
   about 600 MB of weights; later starts don't.

   Then open **four** VS Code terminals (all inside the dev container), and in each:
   ```bash
   source install/setup.bash && source scripts/lab_env.sh
   ```
   `lab_env.sh` sets `ROS_DOMAIN_ID=42`. The Gazebo rehearsal used 84, so a
   leftover Gazebo session won't cross-talk, but close it anyway: it takes CPU
   that OWLv2 needs.
   Why `lab_env.sh` every time: it also raises CycloneDDS's participant limit.
   Without it, about the 11th node dies with "no free participant index".
4. **T1, driver:**
   ```bash
   ros2 launch ur7e_bringup ur7e_bringup.launch.py launch_rviz:=false
   ```
   Wait for `Configured and activated scaled_joint_trajectory_controller`, then press **Play** on the pendant.
   ☐ Before relaunching, make sure no old driver is running. A closed tab can leave `ros2 launch` alive, which gives you two controller managers. Check with `ps aux | grep ros2`.
5. **T2, MoveIt** (KDL, because pick_ik is unreliable for 1 mm Cartesian steps):
   ```bash
   ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl
   ```
6. **T3**, gripper sanity check. Note which **index** it reports. It was 2 on 10-02 and 1 on 10-06, and you'll need it in step C.
   ```bash
   python3 scripts/lab_rg2_native.py
   ```

## B. Verify the tool model (≈15 min, no motion)

Full procedure: `docs/vendor/cad/README.md` → "Checking `lab_tooling.yaml` on the arm".

1. Pendant: **Installation → TCP**. Write down the OnRobot dynamic TCP (X Y Z RX RY RZ).
2. See what the model thinks:
   ```bash
   python3 scripts/lab_tooling.py
   ```
3. Compare the two.
   - **Direction in the flange plane** → `changer_yaw_deg`.
   - **Length** → `rg2_tcp_m`.
   - **Tilt**: about 60°. Anything else means the CAD reading is wrong; **stop**.
4. Look at the arm at `ready`. Which way do the jaws close relative to the changer's V? That gives `rg2_roll_deg`.
5. Close the jaws empty to about 10 mm (**never below 5 mm**: an empty close to 0 counts as a "grip" and stops the program). Measure how far the tips reach past the TCP → `tcp_to_tips_m`.
6. Edit `scripts/lab_tooling.yaml` and set `verified: true`. Write the pendant numbers in a LAB note.
7. **Restart T2 (MoveIt)** so the `lab_rg2` / `rg2_tcp` links pick up the new values.

☐ After restarting T2, `python3 scripts/lab_tooling.py` agrees with the pendant TCP to within a few mm.

## C. Table-board calibration (≈20 min)

The board is 5 × 7 squares of 35 mm, so 175 × 245 mm. First, **measure five
squares with a ruler**. If you don't get 175 mm, the printer scaled it; pass
`--square-mm <measured/5>`. Global flags go **before** the subcommand.

1. Lens model from the ZED factory file:
   ```bash
   python3 scripts/lab_table_calibration.py --square-mm 35 zed --conf docs/calibration/zed2i_SN35717973.conf
   ```
2. Place the camera. Open http://localhost:8089 in a browser.
   ```bash
   python3 scripts/lab_table_calibration.py aim
   ```
   ☐ All board corners are marked and the view angle is **≥ 30°** down. Lock the camera; any bump after this means redoing C.
3. **Four corner touches.** Close the jaws to about 10 mm (again, not below 5). On the pendant Move tab, jog the **fingertips** onto corner N, the number printed on the board. Point the RG2 **straight down**: the script warns above 5° of tilt. Then run:
   ```bash
   python3 scripts/lab_table_calibration.py touch --corner 1 --tool-index <index from A6>
   ```
   Repeat for corners 2, 3 and 4. The script only listens; it sends no motion.
   Why `--tool-index`: the default is 2. If that's wrong, it can't read the jaw depth, so it assumes closed jaws. That's harmless only if the jaws really are closed.
   ☐ Each touch prints `tilt_deg` < 5.
4. Fit:
   ```bash
   python3 scripts/lab_table_calibration.py fit
   ```
   ☐ Residual **< 3 mm** is good. Over 10 mm gets refused, and it prints the side lengths so you can see which corner disagrees. Re-touch that one.
   If every side is off by a similar amount, suspect **B** (the tool model), not your jogging.
5. Check:
   ```bash
   python3 scripts/lab_table_calibration.py check
   ```
   Open `docs/calibration/check.jpg`.
   ☐ The drawn `base_link` axes sit at the robot's base, and the tinted reach region covers the board.

## D. Perception and detect-only (no motion)

**T4, perception node:**
```bash
OMP_NUM_THREADS=4 ros2 run ur7e_perception webcam_perception_node --ros-args \
  -p calibration:=$PWD/scripts/lab_table.json -p objects_file:=$PWD/scripts/lab_objects.yaml \
  -p snapshot_dir:=$PWD/log/perception -p backend:=auto -p reach_min:=0.08 -p reach_max:=0.5 \
  -p default_height:=-1.0
```

Put **one object at a time** on the board area, then run a plan-only pick. It
observes, detects and plans, but sends nothing to the arm:
```bash
python3 scripts/lab_pick.py --say "pick up the red tape"
```
Look at the newest image in `log/perception/`.
☐ The box is on the right object, and the printed `xy` is where it actually sits. Check with a ruler from a board corner.

Things to watch, because they're how this pipeline has failed before:
- **Blue hat:** OWLv2 found "blue helmet" but not "blue hat" on 10-02. The robot's joint caps are also blue.
- **Red tape:** a tape roll is a ring. If it's wider than about 100 mm, the RG2 (110 mm max) can't straddle it. Stand it on edge, or expect a grasp on the rim.
- **Headphones:** irregular shape, and the grasp is undefined (issue #41). It may detect fine and still miss. Treat it as the stretch goal.
- **"collect"** is **not** in the keyword backend's verb list (`pick|grab|get|fetch|take|lift…`). `--say "collect the red tape"` should be **refused** with `--backend keyword`, which is a good check that the guardrail works. Use `--backend claude` to see it understood.

## E. One slow touch-free pointing check (motion, 25 %)

Clear the board of objects first: things on the plate are **not** in the planning scene.

```bash
python3 scripts/lab_pick.py --verify-marker 42 --execute --max-speed-percent 25
```

The arm hovers its closed fingertips 10 mm above the marker the camera found.
**Keep a hand on the E-stop**, especially during the first wrist tilt from `ready`.
☐ Offset < 1 cm. Enter it when asked. Do it at 2–3 different markers.
If it's off by a constant shift, the fault is in B or C. If the offset grows
across the board, the fault is in the camera view (C2).

## F. The picks

`--execute` asks for a typed confirmation before moving; read what it says
it will do. Run each object as **plan-only → 25 % → 75 %**, and only move up
once the run below it was clean: no stop, and the object really left the table.

```bash
# plan only
python3 scripts/lab_pick.py --say "pick up the red tape"
```
```bash
# first real run of this object, slider 25 %
python3 scripts/lab_pick.py --execute --max-speed-percent 25 --say "pick up the red tape"
```
```bash
# slider to 75 %, then
python3 scripts/lab_pick.py --execute --max-speed-percent 75 --say "pick up the red tape"
```

Per object, add the flags that fit it:
- **Wide or round** (hat, tape roll): `--open-mm 108 --tip-clearance 0.025 --force 15`. The fingers slope inward above the pads, so a wide object only clears them near the pads.
- **Put it somewhere else** instead of back in place: `--drop-xy X Y`, in `base_link` metres, inside the board area.
- **Natural language:** `--backend claude --say "collect the blue hat"`. The dev container forwards `ANTHROPIC_API_KEY` from the shell that **started VS Code**. Run `set -a; source .env; set +a` on the host, then launch `code .` from that shell. `echo ${ANTHROPIC_API_KEY:+set}` inside the container should print `set`.

The console does several commands in one session:
```bash
python3 scripts/lab_console.py --pick --execute --max-speed-percent 75 --backend claude \
  --open-mm 108 --tip-clearance 0.025
```
The console refuses to move if the pendant slider is above `--max-speed-percent`.
Restart it with the matching cap rather than treating the refusal as a fault.

## When something goes wrong

| You see | Do |
|---|---|
| Protective stop | **First: what touched what?** Look before you guess. Pendant Unlock → kill and relaunch T1 → Play → re-activate the controller if goals get REJECTED |
| C153 "position deviates from path" | the arm was pushed off its plan (collision). Run `scripts/lab_path_audit.py` on the plan before retrying |
| Planner code 99999 / "invalid bounds" | wrist 3 is past ±6.133 rad. Jog it back toward 0 |
| Goal REJECTED "Controller is not running" | `ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [scaled_joint_trajectory_controller], strictness: 1, activate_asap: true}"` |
| "grip lost" popup | dismiss, Play. Never close empty jaws < 5 mm |
| RG2 `safety_failed` | pendant OnRobot toolbar → reset, then open/close once |
| Camera bumped | redo C2–C5 |

Write down per run: object, speed, result (clean / graze / miss / stop), and
the log folder. Those notes become this session's LAB record.
