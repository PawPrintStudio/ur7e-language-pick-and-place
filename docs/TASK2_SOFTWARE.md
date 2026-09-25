# Task 2: camera-free perception workflow

Implemented on `codex/task2-camera-free-perception`, 2026-09-25. Physical
hardware work is deferred. The software can now receive a noun phrase, capture
registered RGB-D, detect an object, compute its grasp pose, and execute a
collision-checked pick in Gazebo. No camera, Jetson, robot, or API credential
is required for the default regression workflow.

## What each task delivers

| Task | Software delivered | Remaining physical acceptance |
|---|---|---|
| 2.1 ZED platform | Camera-independent source and capture contract; Jetson preparation steps below | SDK/JetPack installation on the actual Jetson, diagnostic, rigid mount |
| 2.2 Camera bringup | 15 Hz RGB-D source, low-memory ZED override, synchronized subscriptions, NPZ snapshots, rosbag capture/replay with clock and QoS | Real camera FPS, GPU budget and camera recordings |
| 2.3 Detection | `DetectObject` service; selectable fixture, real OWLv2, and NanoOWL+NanoSAM engine adapter | Execute TensorRT adapter on Jetson, live hammer mask, memory and latency measurements |
| 2.4 Localization | Mask projection, robust depth centroid, PCA yaw, top-down pose, workspace/height rejection | Known physical marker error below 1 cm |
| 2.5 Calibration | Printable ChArUco/marker patterns, synchronized observation extraction, eye-to-hand solver, held-out verification, per-capture marker drift gate | Physical board acquisition/easy_handeye2 session, measured static TF, five robot touch checks |
| 2.6 Evaluation | Seeded regression suite and recorded RGB-D manifest evaluator with mask IoU, position/yaw error, stage latency and overlays | Labeled recordings of about ten actual makerspace objects; 8/10 real-object acceptance |
| 2.7 Simulation | Rendered Fortress RGB-D, optical TF, sensor-data QoS, pluggable detection, complete motion test | Software evidence only; grasp latch does not validate physical contact quality |

The synthetic score is **not** a claim of recognizing ten real tools. The
fixture recognizes only red/green/blue blocks. OWLv2 runs a pretrained model;
its CPU development mask uses depth foreground separation within the detected
box. That assumes separated tabletop objects and rejects missing or ambiguous
foreground. Jetson's NanoSAM adapter supplies an actual instance mask instead.

## Recorded validation

| Check | Result |
|---|---|
| Full development container build | All 13 workspace packages built |
| Standalone `ur7e-perception:cpu` image | Built from the delivered Dockerfile; fresh container passed 55 perception tests, 30/30 scenes, live services and camera-stopped bag replay; exit 0 |
| Focused colcon tests: interfaces, motion, perception | 67 tests, 0 failures, 0 errors, 0 skipped; includes actual ROS and rendered fiducials |
| Seeded RGB-D regression | 30/30 accepted; worst position error 2.68 mm; worst yaw error 1.68 degrees |
| Actual OWLv2, synthetic color-block scenes | 3/3 accepted; worst position error 0.56 mm; inference 9.41–10.04 seconds on CPU |
| Rendered Gazebo capture, fixture localization | `(0.4517, -0.1494, 0.0800)` m for a block at `(0.45, -0.15, 0.08)` m: about 1.8 mm error |
| Actual OWLv2 on saved Gazebo RGB-D | Red block detected at confidence 0.290; depth-refined mask 100 pixels |
| Synthetic source stopped, bag replay started | Same live ROS detection/localization contract passed using replay `/clock` |
| Gazebo bag capture | 217 messages over 3.29 simulated seconds, including 50 RGB/depth/CameraInfo triplets |
| Full fresh-start Gazebo sequence | Observe, detect, locate, complete approach/descent, close, latch and 10 cm lift succeeded; endpoint checks enabled |
| Full fresh-start Gazebo sequence with real OWLv2 | Same complete sequence succeeded; detected target `(0.4334, -0.1461, 0.0800)` m; CPU inference about 11.6 seconds in the loaded workstation |

The source, noise, labels and ground truth are deterministic; timings depend
on CPU contention. There is no physical-camera accuracy or Jetson-performance
claim in these results. The simple `pick <noun phrase>` grammar is a Phase 2
integration driver, not completion of the Phase 3 LLM parser.

Raw simulator execution evidence is in `perception-evidence/gazebo/` and
`perception-evidence/gazebo_owlv2/`. `perception-evidence/gazebo_capture.npz`
and its PNG are an actual rendered camera snapshot, not a generated illustration.

## Start without a camera

From the repository root, build the portable CPU image and run its complete
software check:

```bash
docker build -f docker/perception.Dockerfile -t ur7e-perception:cpu docker
docker run --rm --network bridge -v "$PWD:/ws" ur7e-perception:cpu
```

That command builds interfaces/perception into `/tmp`, runs the tests and
30-scene evaluation, starts the 15 Hz synthetic RGB-D source, calls both
services, records a bag, stops the source, then repeats the calls from replay.
It exits nonzero on failure. Default domain 83 and localhost-only discovery
keep this container separate from a lab graph. No host networking is needed.

In an existing Humble development container:

```bash
bash scripts/perception_software_check.sh
source /opt/ros/humble/setup.bash
source /tmp/ur7e-perception-check/install/setup.bash
ros2 launch ur7e_perception perception.launch.py
```

In a second terminal with the same environment:

```bash
ros2 service call /perception/detect_object ur7e_interfaces/srv/DetectObject "{query: 'red block'}"
ros2 service call /perception/locate_object ur7e_interfaces/srv/LocateObject "{capture_id: 'COPY_RETURNED_ID'}"
```

Detection returns a mask, bounding box, confidence, latency and capture ID.
Localization returns a `PoseStamped` in `base_link`, valid depth count, and
axis ratio. The pose is the observed top surface with tool Z pointing down;
finger insertion and approach clearance remain motion-policy decisions.

The ID preserves the **same** RGB, depth and intrinsics used for detection.
A later camera frame cannot change an existing localization. The last eight
captures are retained; unknown, evicted, stale, or future-clock IDs fail with
`success=false`. Callers must check `success` before consuming a pose. Frames
must be younger than two seconds when detection starts; the default inference
plus localization budget is 15 seconds.

## Run the complete simulated robot

Use the repository's full development container (the small CPU image above
does not include Gazebo, robot assets or MoveIt). Install `xvfb` if running
without a display. Import vendor dependencies as described in the main README.

```bash
source /opt/ros/humble/setup.bash
bash scripts/vendor_import.sh
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
bash scripts/perception_gazebo_check.sh
```

The script starts one simulator plus MoveIt, motion and perception, waits for
startup, exercises the services, runs the pick, and terminates its process
groups. It writes logs under `/tmp/ur7e-gazebo-perception`. For an alternate
install prefix use `PERCEPTION_INSTALL=/absolute/install`; for log output use
`PERCEPTION_OUTPUT=/absolute/output`.

To leave the workstation running for experiments:

```bash
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
ros2 launch ur7e_perception gazebo_demo.launch.py
# Second terminal, same environment:
python3 scripts/perception_pick_demo.py --execute-simulation 'pick red block'
```

On a headless host prefix the launch with `LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a`.
The startup latch detaches at eight seconds; wait at least twelve seconds
before requesting a pick. Recreate the world between complete pick runs: the
demo intentionally ends holding the object, with matching Gazebo and MoveIt
attachment state. A failed motion stops the sequence; it does not disable
collision checks or silently accept a partial path.

For real OWLv2 inference in this same workstation, install the pinned model
dependencies from `docker/perception.Dockerfile`, then select
`backend:=owlv2` on the launch, or set `PERCEPTION_BACKEND=owlv2` on the check
script. The first model load downloads approximately 600 MB of public model
weights. `OMP_NUM_THREADS=4` is a useful CPU starting point. Cache weights in
the container user's `~/.cache/huggingface` for subsequent offline use.

## Save and replay scenes

```bash
python3 scripts/perception_snapshot.py /tmp/capture.npz
ros2 launch ur7e_perception perception.launch.py replay_file:=/tmp/capture.npz
bash scripts/perception_capture.sh record /tmp/new-bag
# When recording Gazebo, use its clock for bag timestamps:
bash scripts/perception_capture.sh record /tmp/new-sim-bag --sim-time
# Stop the source before playing; start perception with the replay clock:
ros2 launch ur7e_perception perception.launch.py external_camera:=true use_sim_time:=true
bash scripts/perception_capture.sh play /tmp/new-bag
```

NPZ stores RGB, metric depth, intrinsics, original stamp and optical frame
without pickle. The NPZ publisher deliberately restamps all three streams
together for a new replay session. Rosbag playback preserves recorded stamps
and publishes `/clock`. A Gazebo capture needs `config/gazebo.yaml`, since its
camera axes differ from the simple synthetic source. Never label a replay's
static scene as a new live observation of the physical workspace.

## Evaluate future recordings

```bash
ros2 run ur7e_perception perception_eval --backend fixture --count 30 --output /tmp/evaluation
ros2 run ur7e_perception perception_eval --backend owlv2 --manifest /data/objects.json --output /tmp/objects-evaluation
```

A recorded manifest contains `source: "recorded"`, a 4x4 `camera_to_base`
matrix, the XY `workspace` polygon, and `cases`. Each case supplies `frame`
(NPZ), `mask` (independently annotated PNG), `query`, `position_base_m`
(three numbers), and `yaw_rad`. Paths resolve beside the manifest. A case
passes only with mask IoU at least 0.5 and position error below 1 cm; the report
passes when at least 80% of cases pass. Do not generate reference masks from
the detector being evaluated. Supply real metrology for reference positions.

## Calibration and eventual ZED integration

`ur7e_perception.calibration.eye_to_hand` accepts at least six paired 4x4
`base_from_gripper` and `camera_from_board` matrices. The board must be rigid
on the gripper. Collect varied positions and rotations about at least two
axes using ChArUco/easy_handeye2. The solver inverts robot poses to adapt
OpenCV's hand-eye equation to the fixed-camera case, checks constant board
attachment, then requires at least five **held-out** 3D marker correspondences
within 1 cm. Synthetic frame-chain recovery and degenerate/inconsistent input
rejection are tested.

```bash
python3 -m ur7e_perception.fiducials print /tmp/calibration-patterns
python3 -m ur7e_perception.fiducials observe capture.npz robot_pose.json observation.json
python3 -m ur7e_perception.calibration observations.json calibration.json
```

For extraction, `robot_pose.json` supplies `stamp` and `base_from_gripper` at
the image timestamp (within 40 ms). Use TF recorded with the capture or a
synchronized robot-pose sample. The observation extractor detects actual
ChArUco corners, estimates the board pose and refuses missing patterns or
unsynchronized robot data. Collect its outputs into the solver's pose arrays.
Print the board at 200 x 280 mm and marker 42 with a 40 mm black square; measure
the print instead of relying on a printer's automatic scaling.

The observations JSON contains `source` (`synthetic` or `physical`), the two
pose arrays above, and `verification.camera_points` / `verification.base_points`.
The output includes the fitted transform, residuals and verification data;
it refuses to overwrite an existing calibration. Separately publish the
measured camera TF and compare it against the configured transform before
using the physical system.

Physical node configuration must set `simulation: false`, a measured
`camera_to_base`, matching `camera_frame`, measured workspace/height limits,
and `calibration_verification` pointing to a successful **physical** report.
Also configure the measured `base_from_marker` (4x4), `calibration_marker_id`
and `calibration_marker_size`. Physical mode checks the permanent marker in
every requested capture, before detection, and rejects missing/ambiguous
markers, displacement of 1 cm or more, or orientation error of 0.05 radians
or more. This complements startup verification and the physical touch checks.
The synthetic fixture backend is refused in physical mode. Actual printed
pattern accuracy, pose uncertainty and the complete physical calibration
session still require lab validation.

The ZED override is `config/zed2i_low_memory.yaml`. With the current upstream
launch interface:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i \
  ros_params_override_path:=/absolute/zed2i_low_memory.yaml \
  publish_tf:=false publish_map_tf:=false
ros2 launch ur7e_perception perception.launch.py external_camera:=true \
  backend:=nanoowl config:=/absolute/measured-perception.yaml \
  rgb_topic:=/zed/zed_node/rgb/image_rect_color \
  depth_topic:=/zed/zed_node/depth/depth_registered \
  info_topic:=/zed/zed_node/rgb/camera_info
```

Verify topic names, optical frame, SDK/wrapper compatibility and actual
parameters on the installed release. Build NanoOWL's image engine and
NanoSAM's encoder/decoder on the target Jetson; configure `owl_engine`,
`sam_encoder`, `sam_decoder`. The adapter does not silently fall back to the
fixture if engines are missing. Record the canonical remapped topics or
apply equivalent explicit bag remappings for ZED-native names. Point clouds
are subscriber-driven: do not subscribe or record them. In current upstream,
`point_cloud_freq: 0` means **unlimited**, not disabled.

## Integration defects found and fixed

* The old Cartesian helper accepted a trajectory fraction of zero by default.
  It could report success after moving only 2.4% of a requested lift. Vertical
  primitives now require 100% and verify the final TCP position/orientation.
* An arbitrary hover IK solution could reach the hover but collide with the
  table during descent. Approach planning now finds a collision-free grasp
  configuration, derives a compatible hover, and checks the complete descent
  before moving. Global IK selects a new configuration; local IK follows
  vertical segments. Concurrent motion primitives are rejected.
* Pose assignments shared mutable objects, changing the caller's grasp Z when
  constructing a hover. Copies now preserve the original detected pose.
* The gripper link was lost during URDF fixed-joint lumping. The Gazebo tag
  now references the joint and preserves the link needed by the grasp latch.
* Rendered gripper meshes needed an explicit Gazebo resource search path.
* Closing the fingers made MoveIt correctly detect a collision with the
  still-unattached object. The demo now attaches the collision object with
  explicit gripper touch links. Only object/table departure contact is allowed
  during lift, then disabled; robot/table collisions remain checked.

## Live desktop demonstration

The display-capable `ur7e-task2-visual` container uses the host X11 socket,
the same UID (1000) as the desktop, and `xhost +SI:localuser:nikola`.
X access control stays enabled. Its terminal is shared through host tmux
session `task2-visual`: `demo` contains the commands, `stack` the launch logs,
and `camera` the image viewer. Reattach with `tmux attach -t task2-visual`.

Inside this container, explicitly select the simulation domain **after**
opening an interactive shell: the inherited lab `.bashrc` sets domain 42.

```bash
source /opt/ros/humble/setup.bash
source /tmp/task2-install/setup.bash
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=4
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
ros2 launch ur7e_perception gazebo_demo.launch.py \
  backend:=owlv2 gazebo_gui:=true launch_rviz:=true
```

In other terminals with the same environment:

```bash
python3 scripts/perception_viewer.py
python3 scripts/perception_pick_demo.py --execute-simulation \
  --stage-pause 3 'pick red block'
```

The viewer shows live RGB and depth, the frozen inference capture with a
green segmentation overlay, and the located grasp coordinates. Diagnostic
topics `/perception/annotated` and `/perception/grasp_pose` retain the last
successful result. They are historical captures, not continuous tracking.

On a slower visual startup, the initial latch can displace the block after
the launch's timed reset. If the block is outside the camera, **before a new
pick** and after the initial detach, restore the simulated object:

```bash
ign service -s /world/pick_place_table/set_pose \
  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
  --timeout 2000 --req 'name: "pick_object", position: {x: 0.45, y: -0.15, z: 0.04}, orientation: {w: 1.0}'
```

The live run on September 25 completed OWLv2 detection, depth localization,
approach, descent, simulated grasp and 10 cm lift. For a repeat after lifting,
restart the simulation first so the physics latch and MoveIt attached-object
state are both clean. Stop all owned demo processes/windows with
`docker stop ur7e-task2-visual` from the host. The narrow xhost entry existed
before this demo and was preserved.

## Primary references

* [OWLv2 API](https://huggingface.co/docs/transformers/model_doc/owlv2)
* [NanoOWL](https://github.com/NVIDIA-AI-IOT/nanoowl) and [NanoSAM](https://github.com/NVIDIA-AI-IOT/nanosam)
* [ZED upstream configuration](https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/config/common_stereo.yaml)
* [pick_ik modes and cost weights](https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html)

Context7 returned no matching OWLv2 documentation, so the implementation was
checked against the official Hugging Face and NVIDIA sources directly.
