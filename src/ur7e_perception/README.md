# ur7e_perception

The robot's eyes. Given a short noun phrase ("red block", "hammer"), this package answers one
question:

> **Where is the thing called X, in the robot's own coordinates, and which way is it lying?**

The answer is a *grasp pose*: a point on top of the object and a top-down tool orientation,
expressed in `base_link` (the coordinate frame fixed to the robot's base, in metres). Everything
downstream (the orchestrator, MoveIt, the gripper) works from that one pose.

The package never moves the robot and never decides how to grasp. It only looks, measures, and
either answers or refuses with a reason.

> Status, 2026-10-02: everything here is verified in software only (seeded synthetic scenes,
> Gazebo renders, and rendered single-camera scenes). **No accuracy has been measured with a real
> camera on the real arm yet.** Numbers quoted below come from rendered images and say so.

## The concept: an image is not a place

A camera gives you pixels. A robot needs metres. Getting from one to the other takes three
separate pieces of knowledge, and it helps to keep them apart in your head because each one fails
in its own way:

1. **Which pixels are the object?** A *detector* turns the words "red block" into a *mask*: an
   image the same size as the photo where the object's pixels are 1 and everything else is 0.
2. **How far away is each of those pixels?** A photo is flat. One pixel tells you the *direction*
   light came from, not the distance. Something else must supply the distance (see
   [Three ways to get 3-D out of a camera](#three-ways-to-get-3-d-out-of-a-camera)).
3. **Where is the camera relative to the robot?** A point "40 cm in front of the lens" is useless
   to the arm until you know where the lens is. That relationship is the *calibration*.

Read the code in this order: `core.py` (the geometry, no ROS), `backends.py` (detectors for the
depth path), `nodes.py` (the ROS services), then `monocular.py` and `webcam_node.py` (the same
services from one ordinary camera), and `stereo.py` (height from the ZED's second lens).

## Two services, one capture ID

Both ROS nodes in this package offer exactly the same two services
(`ur7e_interfaces/srv/DetectObject` and `LocateObject`):

```text
/perception/detect_object      query            ->  success, reason, capture_id,
                                                    mask, bbox[4], confidence, latency_ms
/perception/locate_object      capture_id       ->  success, reason, pose (PoseStamped),
                                                    valid_points, axis_ratio
```

`detect_object` takes the noun phrase, grabs **one** image, finds the object in it, and stores
that image together with the mask under a random `capture_id`. `locate_object` takes the ID and
turns the stored mask into a grasp pose.

Why two steps instead of one "find it" call?

- **They fail differently.** "I cannot see a hammer" (detection) and "I see it but cannot measure
  it" (localization) are different problems with different fixes. The orchestrator has a separate
  DETECT and LOCATE stage for exactly this reason: a missing object is retried by looking again,
  a geometry failure is not.
- **They cost differently.** Detection may run a large neural network for seconds. Localization
  is a few milliseconds of arithmetic.

Why the capture ID?

- **The mask and the geometry must come from the same instant.** If `locate` used "the latest
  frame", a hand moving the object, or the arm swinging into view between the two calls, would
  pair a mask from one picture with distances from another. The result would be a confident,
  wrong pose. The ID freezes everything detection used (image, depth or camera pose, intrinsics)
  so a later frame can never change an earlier answer. The service definition says it in one
  line: *"Resolve only an immutable capture returned by DetectObject, never latest depth."*
- **Old answers expire.** Each node keeps the last 8 captures. An unknown, evicted, or stale ID
  fails with `success=false`. The age limit is the `max_capture_age` parameter (15 s by default in
  the depth node, 60 s in the single-camera node).

Two rules for callers:

- **Always check `success`.** A service must answer, so every failure comes back as
  `success=false` plus a human-readable `reason`; the pose fields are then meaningless.
- **A refusal is a result, not a crash.** The package raises `PerceptionError` whenever a capture
  or target is "unsafe or insufficient for localization" and reports it. It does not guess.

## Three ways to get 3-D out of a camera

A camera with a known lens (known *intrinsics*: focal length and image centre, in pixels) turns
each pixel into a *ray*: a straight line from the lens out into the world. The object is
somewhere on that ray. Three different extra facts can pin down where, and this repository uses
all three.

| Extra fact | What it gives | Where it is used |
|---|---|---|
| **A depth image**: the camera itself reports a distance for every pixel | A 3-D point per pixel, directly | The designed path: `core.localize`, `nodes.py` |
| **A pattern of known size and shape**: a printed board or marker | The full pose (position and orientation) of the pattern relative to the camera | Calibration and drift checks: `fiducials.py`; the table board in `monocular.py` |
| **A known surface**: the object sits on a plane whose position is known | One 3-D point per pixel, where its ray meets the plane | The single-camera path: `monocular.rays_to_plane` |

### Depth image (the designed sensor)

A depth camera such as the ZED 2i with its SDK delivers, alongside the colour image, a second
image whose pixel values are distances in metres, already lined up ("registered") with the colour
pixels. Multiply a pixel's ray by its depth and you have a 3-D point in the camera's frame. One
fixed 4x4 matrix, `camera_to_base`, moves it into `base_link`. That matrix comes from a
calibration session and stays valid only while the camera does not move, which is why the depth
node checks a permanent marker in every physical capture (`fiducials.check_workspace_marker`).
See [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md) decisions D3 and D4 for why this is the
design, and [`docs/TASK2_SOFTWARE.md`](../../docs/TASK2_SOFTWARE.md) for the full workflow.

### A pattern of known size

Print a chessboard-like pattern at a known scale and the camera can work out where the *pattern*
is in 3-D from a single ordinary photo: only one position and orientation makes squares of that
real size land on those pixels. OpenCV's `solvePnP` does this ("Perspective-n-Point": the pose
from n known points). It locates the pattern, not arbitrary objects, so on its own it is a
calibration tool.

### One camera, no depth

This is the path built for sessions without the depth SDK. A plain camera gives only rays. What
turns a ray back into a point is a known surface: everything we pick sits on the table, and a
straight line meets a flat plane exactly once.

So a single camera **needs the table**, in two senses:

- it needs to know *where the table plane is* (a printed board taped to the table tells it, in
  every single image, using the known-size-pattern trick above), and
- it can only locate things that are *on* that plane. It finds where an object touches the
  table. It cannot see how tall the object is, so the height has to be told to it
  ([`scripts/lab_objects.yaml`](../../scripts/lab_objects.yaml)). A wrong height does not just
  give a wrong Z: it shifts the X/Y answer too, because a tall object seen from the side hides a
  strip of table behind it.

The full explanation, the calibration procedure, and the meaning of every failure message are in
[`docs/SINGLE_CAMERA_PERCEPTION.md`](../../docs/SINGLE_CAMERA_PERCEPTION.md).

### Two lenses: where a depth image comes from

A depth camera is not magic; the ZED makes its depth image from two ordinary lenses 120 mm
apart. A point seen by the left lens at image column `x` appears in the right lens slightly
further left, at `x - d`. The shift `d` (the *disparity*, in pixels) is large for near points
and zero at infinity:

```text
depth = focal_px * baseline_m / d
```

The Stereolabs SDK does this on a GPU. `stereo.py` does the same geometry with OpenCV on the
CPU, for one purpose: to measure an object's **height** so the single-camera path no longer has
to be told it. From its docstrings:

- **Rectification.** Searching the right image for each left pixel is only practical along one
  image row. Real lenses distort and are not perfectly parallel, so the raw images are first
  warped into two virtual, exactly parallel pinhole cameras (`cv2.stereoRectify`, from the
  factory calibration of both lenses). After that every scene point sits on the same row in
  both images, and semi-global block matching (`cv2.StereoSGBM`) does the row search.
- **`StereoRig.points_for_pixels`** returns a 3-D point for each requested pixel of the left
  image, or NaN where no trustworthy match exists.
- **`object_height`** takes those points for an object's mask, measures each one's distance
  above the table plane, and returns the 90th percentile: high enough to sit in the top surface
  rather than the sides, without trusting the single highest (possibly wrong) point.
- **Its stated limits.** One pixel of disparity is worth `depth^2 / (focal * baseline)` metres,
  which grows with the square of the distance, so the camera should be close. Matching needs
  texture: a plain surface yields few or no points. And the factory calibration can drift;
  `StereoRig.rectification_error` checks how well it still holds on any image pair.

As of this writing `stereo.py` is a library with its own tests. `webcam_node.py` does not call
it yet, so the node still takes heights from the objects file.

## From mask to grasp pose

Both paths end the same way: they have a set of points belonging to the object, in `base_link`,
and reduce it to four numbers the arm can use.

**Centre.** The depth path takes the median of the mask's 3-D points (the median ignores a few
wild depth readings where a mean would not). The single-camera path takes the mean of the
object's footprint cells on the table and lifts it by the object's height. Either way the
returned position is **on the object's top surface**. How deep the fingers go and how high to
hover are motion decisions and are deliberately not made here.

**Yaw from the principal axis.** To know which way a long object lies, the code computes the
covariance of the points' X/Y coordinates and takes its eigenvectors (this is PCA, *principal
component analysis*: find the direction along which the points are most spread out). The
direction of largest spread is the object's long axis; its angle in the table plane is the yaw.
Two details:

- An axis has no front or back, so yaw is wrapped into the range -90 to +90 degrees.
- The ratio of largest to smallest spread is returned as `axis_ratio`. For a rectangle it is the
  *square* of the side ratio. If the object is nearly round or square the "long axis" is just
  noise, so below a threshold the yaw is set to exactly 0: a deterministic answer beats a random
  one. The threshold is 1.2 in the depth path (`core.localize`) and 2.0 in the single-camera path
  (`monocular.localize_on_table`, where 2.0 corresponds to a side ratio of about 1.4).

**Top-down convention.** The orientation is always

```text
R = Rz(yaw) * Rx(pi)        quaternion (x, y, z, w) = (cos(yaw/2), sin(yaw/2), 0, 0)
```

`Rx(pi)` is a half-turn about X: it flips the tool so its Z axis points straight down at the
table. `Rz(yaw)` then spins it about the vertical so the tool's X axis lies along the object's
long axis. The resulting tool axes in `base_link` are X = (cos yaw, sin yaw, 0),
Y = (sin yaw, -cos yaw, 0), Z = (0, 0, -1). Perception reports the object's axis; which way the
jaws should straddle it is the motion side's choice (`scripts/lab_pick.py` turns a further 90
degrees because the RG2's jaws close along tool X).

**Refusals.** A pose is only returned if it passes these checks. Each failure has its own
`reason` string.

| Check | Depth node (`nodes.py`) | Single-camera node (`webcam_node.py`) |
|---|---|---|
| Inside the workspace | Every point of the object inside the `workspace` polygon | Horizontal distance from the base origin between `reach_min` (0.20 m) and `reach_max` (0.80 m) |
| Plausible height | Centre between `table_z` and `max_z` (0.35 m) | Height from the objects file within 0 to 0.3 m; camera at least 5 cm above the object's top |
| Enough data | At least 20 valid depth pixels, and at least half of the mask | Mask of at least 50 pixels; at least 50 footprint cells (1 mm each) left after removing the height shadow |
| Calibration still true | Workspace marker within 1 cm and 0.05 rad of where it was calibrated (physical mode) | Table board visible in this capture (at least 8 corners) with reprojection error at most 2.5 px |
| Not the calibration target | not applicable | Target not on, or within 10 mm of, the board |

The single-camera geometry function also accepts a workspace polygon, but the node does not pass
one today; its workspace test is the reach ring above.

## The detectors, and when each is the right tool

A detector's job is words in, mask out. Which ones are available depends on the node; the node
parameter is called `backend` in both.

**Depth node** (`backends.py`):

| `backend` | What it is | Use it for |
|---|---|---|
| `fixture` | Thresholds pure red/green/blue. Accepts only the exact queries `red block`, `green block`, `blue block`. Refused outside simulation. | Regression tests of geometry and ROS transport. It proves nothing about recognising real objects. |
| `owlv2` | OWLv2 (`google/owlv2-base-patch16-ensemble`), an *open-vocabulary* detector: it takes free text and returns boxes. The box becomes a mask by keeping pixels that stand above the table in the depth image. | Development machines without a Jetson. Assumes separated objects; refuses when a second distinct box is found. |
| `nanoowl` | NanoOWL + NanoSAM TensorRT engines. | The Jetson. Not yet run on the target hardware (see TASK2_SOFTWARE.md). |

**Single-camera node** (`webcam_node.py`):

| `backend` | What it is | Use it for |
|---|---|---|
| `color` | Largest saturated blob of a named colour (`red`, `orange`, `yellow`, `green`, `blue`, `purple`) on the reachable table. The query must contain exactly one of those words. No model; a few milliseconds. | Coloured blocks, rehearsals, anything that must be fast, repeatable and offline. Refuses if a second blob is more than half the size of the first. |
| `owlv2` | The same OWLv2 model, asked for "a photo of a *query*". Boxes larger than a quarter of the image, off the reachable table, or on the board are discarded. With no depth to separate object from table, the mask comes from colour instead (GrabCut: pixels just outside the box are assumed to be table). | Arbitrary objects named in free text. Slow on a CPU, and the first run downloads about 600 MB. |
| `auto` (default) | `owlv2` first; if it refuses, `color`; if both refuse, the OWLv2 reason is reported. | General use. |

In the single-camera node the special query `marker NN` (for example `marker 42`) bypasses the
backend and finds a printed ArUco marker. A marker is flat and its centre is known exactly, which
makes it the right target for checking calibration: any error left is the system's, not the
detector's.

All detectors share one habit: **two plausible answers is a refusal**, not a coin toss
("multiple plausible targets; clarify query", "more than one red object; be more specific").

## Running it

All commands assume the dev container with the workspace built and sourced, run from the
repository root. Both nodes are named `perception_node` and serve the same two service names, so
run one or the other, not both.

### Synthetic (depth path, no camera)

A built-in source publishes a rendered RGB-D scene at 15 Hz; the `fixture` backend finds the
block.

```bash
ros2 launch ur7e_perception perception.launch.py
```

In a second terminal:

```bash
ros2 service call /perception/detect_object ur7e_interfaces/srv/DetectObject "{query: 'red block'}"
ros2 service call /perception/locate_object ur7e_interfaces/srv/LocateObject "{capture_id: 'COPY_RETURNED_ID'}"
```

The same two calls work unchanged against every mode below.

### Replay (depth path, a saved capture)

```bash
python3 scripts/perception_snapshot.py /tmp/capture.npz
ros2 launch ur7e_perception perception.launch.py replay_file:=/tmp/capture.npz
```

Rosbag record/replay, Gazebo, real OWLv2 (`backend:=owlv2`) and the physical ZED launch are
covered in [`docs/TASK2_SOFTWARE.md`](../../docs/TASK2_SOFTWARE.md).

### Single camera, rehearsal (a rendered image instead of a camera)

`synthetic_table` renders what a camera would see of the board and two 40 mm blocks, and writes a
matching calibration file marked `synthetic`. The node accepts a synthetic calibration only
together with `replay_image`: a live camera with a made-up calibration would send the arm to
made-up places.

```bash
python3 -m ur7e_perception.synthetic_table /tmp/scene.png /tmp/table.json
ros2 run ur7e_perception webcam_perception_node --ros-args \
  -p calibration:=/tmp/table.json -p replay_image:=/tmp/scene.png \
  -p backend:=color -p objects_file:="$PWD/scripts/lab_objects.yaml"
```

`bash scripts/lab_pick_rehearsal.sh "pick up the red block"` wraps this with mock robot hardware,
MoveIt and the orchestrator to rehearse the whole pick without the lab.

### Single camera, live

Calibrate first (see the
[operator checklist](../../docs/SINGLE_CAMERA_PERCEPTION.md#calibration-checklist)), then:

```bash
ros2 run ur7e_perception webcam_perception_node --ros-args \
  -p calibration:="$PWD/scripts/lab_table.json" \
  -p objects_file:="$PWD/scripts/lab_objects.yaml" \
  -p snapshot_dir:="$PWD/log/perception"
```

Without `objects_file` every object is assumed to be 30 mm tall (`default_height`). With
`snapshot_dir` set, each successful `locate` saves the frozen image with the mask, box and grasp
point drawn on it (`<capture_id>.jpg` and `latest.jpg`). The parameters, topics and failure
messages are listed in [`docs/SINGLE_CAMERA_PERCEPTION.md`](../../docs/SINGLE_CAMERA_PERCEPTION.md).

## How it is tested

The geometry modules import no ROS, so most tests need only Python, NumPy and OpenCV. The pattern
is the same throughout: **render a scene whose true geometry is known, run the real code on it,
compare.**

```bash
# Build, all tests, 30-scene evaluation, live services, bag replay:
bash scripts/perception_software_check.sh
# Tests only, after the workspace is built and sourced:
python3 -m pytest -q src/ur7e_perception/test
```

| Test file | What it checks |
|---|---|
| `test_core.py` | 30 seeded RGB-D scenes located within 5 mm and 0.15 rad; a list of deliberate faults (NaN depth, wrong units, outside workspace, stale capture, ...) each refused |
| `test_ros_contract.py` | Real ROS messages through the depth node: a capture ID keeps its original depth, mismatched frames are rejected, old IDs are evicted (skipped where `rclpy` is absent) |
| `test_calibration.py`, `test_fiducials.py` | Eye-to-hand solver recovers a known transform and rejects bad data; printed marker and ChArUco poses, drift detection |
| `test_monocular.py` | The single-camera path on scenes rendered by `synthetic_table.py`: board detection, focal length, touch fit, height shadow, yaw, colour detector, refusals |
| `test_stereo.py` | Stereo height on rendered, textured left/right pairs: 40 and 80 mm boxes measured within 5 mm from about 0.8 m, a blank table yields no points instead of wrong ones, refusals. Tests that need the ZED's factory file or a stored real frame are skipped when those files are absent |

What the single-camera tests assert, all on **rendered images** (a perfect pinhole camera, no
lens distortion, flat-coloured boxes, exact masks):

- a 40 mm cube is located within 6 mm when its height is given, and its footprint sides come out
  within 8 mm of 40 mm;
- ignoring the height (telling the code the cube is flat) moves the answer by more than 12 mm;
- an overstated height (120 mm for the 40 mm cube) is refused rather than guessed;
- a 140 x 30 x 20 mm bar at 40 degrees: yaw within 6 degrees, centre within 8 mm;
- focal length from a single board view: within 15 %; from 16 views: within 3 %;
- a missing board, a second object of the same colour, and a target outside the workspace
  polygon are each refused.

These numbers bound the *arithmetic*. They say nothing about a real lens, a real print, real
lighting, or the robot. Recorded results for the depth path (also software-only) are in
[`docs/TASK2_SOFTWARE.md`](../../docs/TASK2_SOFTWARE.md).

## Limits

- **Nothing here has been validated on hardware yet.** No real-camera, real-robot accuracy has
  been measured for either path. `TODO(verify in lab)`: record the touch-verification results
  (`scripts/lab_pick.py --verify-marker`) and link them here.
- **Top-down grasps on a table only.** One pose per object, tool straight down. No stacked
  objects, no grasping from the side, no 6-DoF grasp planning. This is architecture decision Q5.
- **One object per query.** Two candidates is a refusal. Touching objects are not separated by
  the development masks (depth foreground, GrabCut).
- **The single-camera node does not measure height.** It is told the height, and it assumes the
  object has vertical sides (a box or cylinder standing on the table). A wrong height or an
  overhanging shape (a hammer head on a thin handle) biases the X/Y answer. The depth camera
  remains the designed sensor for this reason; `stereo.py` is a step toward measuring the height
  without the SDK but is not wired into the node yet.
- **The single-camera path needs the board in every capture.** If the arm or an object covers it,
  detection is refused. Objects cannot be picked off the board itself.
- **Colour-based masks need contrast.** GrabCut and the colour detector separate object from
  table by appearance. A grey tool on a grey table will fail or produce a poor mask.
- **OWLv2 is a pretrained general model.** It has not been evaluated on the makerspace's actual
  tools. The synthetic scores in this repository are not a claim that it recognises them.
- **OpenCV version.** The ArUco code uses the older API (`cv2.aruco.CharucoBoard_create`,
  `drawMarker`, `board.draw`) as shipped by the Humble containers' `python3-opencv`. Newer OpenCV
  releases renamed these calls, so run the tools inside the container, not a host virtualenv.

## Further reading

- [`docs/SINGLE_CAMERA_PERCEPTION.md`](../../docs/SINGLE_CAMERA_PERCEPTION.md): the single-camera
  geometry step by step, the calibration checklist, every failure message.
- [`docs/TASK2_SOFTWARE.md`](../../docs/TASK2_SOFTWARE.md): the depth path: validation record,
  Gazebo, rosbags, evaluation, eye-to-hand calibration, ZED integration.
- [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md): D3 (open-vocabulary, on-demand
  detection) and D4 (ZED 2i, fixed overhead mount).
- [`src/ur7e_orchestrator/README.md`](../ur7e_orchestrator/README.md): the DETECT and LOCATE
  stages that call these services.
