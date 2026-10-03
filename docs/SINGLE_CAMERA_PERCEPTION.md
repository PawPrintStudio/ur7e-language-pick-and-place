# Single-camera perception: locating objects on a table without a depth sensor

This is the explainer and operator guide for the single-camera path in `ur7e_perception`
(`monocular.py`, `webcam_node.py`, `camera.py`) and its calibration tool
(`scripts/lab_table_calibration.py`). For what the package does as a whole, and how this path
relates to the depth-camera path, start with the
[package README](../src/ur7e_perception/README.md).

> Status before the 2026-10-02 lab session: the geometry is unit-tested on **rendered images**
> only. Every number in the explanation below is either a default read from the code or a bound
> asserted by a test on rendered scenes, and is labelled. **Update, 2026-10-02 evening:** the
> path ran on the real arm with a real camera, in a variant without a printed board — one
> object, one placement, 0.8 cm. See [What ran in the lab on 2026-10-02](#what-ran-in-the-lab-on-2026-10-02);
> the board-based explanation is unchanged and still describes what the code does.

## The idea in one paragraph

The pipeline was designed around a depth camera, which reports a distance for every pixel. An
ordinary camera does not: one pixel is a *ray* (a line out of the lens), and the object could be
anywhere along it. What turns the ray back into a point is a known surface. Everything we pick
sits on the table, and a line meets a plane exactly once. So the camera needs to know where the
table is, in the robot's coordinates. A printed board taped to the table tells it. The camera
finds the board in every picture and works out its own position relative to the board; once, at
calibration time, the robot touches the board's four corners, which fixes where the board is
relative to the robot. Chain the two and every pixel's ray can be followed down to a point on the
table in `base_link`. The one thing this cannot recover is the object's height, which the node
takes from a list (a ZED's second lens can measure it instead; see
[the last section](#measuring-the-height-with-the-second-lens)).

## The frames and the chain

A *frame* is a coordinate system attached to something. Three matter here:

- **camera**: origin at the lens, Z pointing out along the view, X right, Y down in the image.
- **board**: origin at corner 1 of the printed board, X and Y along its edges, Z up out of the
  paper. The table surface is the plane Z = 0 of this frame.
- **`base_link`**: the robot's base. This is the frame the arm is commanded in.

A 4x4 *rigid transform* converts a point's coordinates from one frame to another (a rotation plus
a shift, no stretching). The code names them `target_from_source`.

```text
               from THIS image                          from four touches, ONCE
            (solvePnP on the board)                   (rigid fit to the robot's TF)

  camera  <---------------------------  board  --------------------------->  base_link
              camera_from_board                      base_from_board

  cam_to_base  =  base_from_board  x  inverse(camera_from_board)
```

The left arrow is re-measured in every capture. The right arrow is measured once and stored in
the calibration file. That split is the whole design:

- Bump the **camera** (it is a laptop, or a tripod in a makerspace) and nothing is lost: the next
  picture re-solves where the camera is.
- Move the **board** and the stored half is wrong: the four touches must be redone.
- Hide the board and there is no left arrow at all, so the node refuses instead of guessing.

## How it works, step by step

### 1. A pixel is a ray

The *pinhole model* describes an ideal camera with three numbers: the focal length `f` in pixels
and the image centre `(cx, cy)`. Together they form the *intrinsic matrix* `K`. A pixel `(u, v)`
corresponds to the direction `((u - cx) / f, (v - cy) / f, 1)` in the camera frame. The code
assumes square pixels and the image centre in the middle of the image (`monocular.intrinsics`).

A real lens bends straight lines slightly (*distortion*). If the calibration file carries
distortion coefficients, every frame is first re-mapped into the image an ideal pinhole camera
would have taken (`TableCalibration.undistort`), so all later arithmetic can stay simple.

### 2. Camera to board, from the image

The board is a **ChArUco board**: a chessboard whose white squares each contain a small ArUco
marker (a square barcode with an ID). The markers say *which* chessboard corner is which even
when part of the board is hidden; the chessboard corners themselves can be found to a fraction of
a pixel. Ours is 5 x 7 squares of 35 mm with 26 mm markers, 175 x 245 mm overall, which gives 24
inner corners (`monocular.table_board`).

For each capture the code:

1. finds the corners in the most recent frames (up to `average_frames`, default 5). A corner is
   kept only if it shows up in at least half of those frames, and its position is the median
   over them, which suppresses single-frame noise. At least 8 corners must survive.
2. calls OpenCV's `solvePnP`. "PnP" is *Perspective-n-Point*: given n points whose real
   positions are known (the corners, in board millimetres) and where each appears in the image,
   find the one camera position and orientation that explains the picture. Because the real size
   of the board is known, the answer is in metres.
3. checks the answer. A flat target always has two mathematically valid poses (the camera could
   be on either side of the paper, mirrored). Both are computed, and the one with the camera on
   the printed side that fits best is kept (`monocular.board_pose`). Then the corners are
   projected back through the solved pose and compared with where they were detected. The RMS
   distance is the *reprojection error*; above 2.5 px the capture is refused.

### 3. Board to robot, from four touches

**Why touches are needed at all.** The camera can measure where the board is relative to
*itself*. It has never seen the robot's base and has no idea where that is. Only the robot can
say where something is relative to the robot. So the robot has to measure the board too, and the
simplest measuring instrument a robot has is its own fingertip: put it on a known point of the
board and read the arm's joint sensors.

**What is recorded.** For each of the board's four outer corners (numbered 1 to 4 on the print,
counter-clockwise seen from above, starting at the board's origin), the operator jogs the closed
fingertips onto the corner with the tool pointing straight down. The script reads the robot's own
`base_link -> tool0` transform from TF (`tool0` is the mounting flange) and computes the
fingertip point as "flange position, pushed along the tool's Z axis by the gripper length". The
script only listens; it commands no motion.

**The fit.** The four corners are known in board coordinates: (0, 0), (175, 0), (175, 245),
(0, 245) mm. The four fingertip points are known in `base_link`. `monocular.fit_board_anchor`
finds the rotation and shift that best lays the first set onto the second (a least-squares rigid
fit using the SVD; the same routine as `core.fit_calibration`). The result is `base_from_board`.

**Why four, when three would do.** Three points define a rigid placement exactly, with nothing
left over to check it. The fourth makes the problem over-determined, so each point has a
*residual*: how far it lands from where the fit says it should be. The fit allows no stretching,
so the residual is an honest measure of two things at once: how carefully the corners were
touched, and whether the print is really the size the software believes. The camera cannot check
the print size (a board printed 2 % small just looks like a board slightly further away). The
touches can, because they are in real robot metres. Under 3 mm is good; 10 mm or more is refused.

**Why the gripper length cancels out.** The script needs a gripper length (`--tool-length`,
default 0.20 m) to turn a flange position into a fingertip position, and that number is not
known exactly. With the tool vertical it does not need to be:

```text
at touch time:    table_z            =  flange_z_at_touch - L
at pick time:     flange_z_commanded =  table_z + L + clearance
                                     =  flange_z_at_touch + clearance
```

`L` is stored in the calibration file and `scripts/lab_pick.py` reuses the same value, so it
drops out. "Fingertips 8 mm above the table" means 8 mm above where they stood when they touched
it. An error in `L` shifts every stored height by the same amount, including the absolute table
height written in the file, but X and Y are untouched and every commanded height is still right
relative to the real table.

This only holds for a **vertical** tool. If the tool is tilted, a wrong `L` pushes the computed
fingertip sideways as well as down, and that error does not cancel. The script prints the tilt
and warns above 5 degrees. The calculation also assumes the fingertip centre lies on the tool's
Z axis. `TODO(verify in lab)`: confirm both the 0.20 m default and the on-axis assumption for the
RG2 as mounted.

**Why jaws closed.** The RG2's fingers swing on arms, so the fingertips are lowest (the gripper
is longest) when the jaws are closed and rise as they open; this is the reasoning recorded in
`scripts/lab_pick.py`. Every height in the system is therefore expressed for the closed,
longest state. During a pick the jaws are open and the real fingertips are higher than assumed,
never lower, so every clearance the software computes is a minimum. Calibrating against open
jaws and then descending with them closed would err in the dangerous direction: the fingertips
would end up lower than intended.

The `touch` command guards against that. It asks the gripper (through the OnRobot URCap's
XML-RPC server on the robot, `--robot-ip`, `--tool-index`) how much shorter it currently is than
when closed, a value the RG2 reports as `depth` in millimetres, and subtracts it, so a touch
made with the jaws partly open is converted to the closed-jaw length. If the gripper cannot be
read, the script prints `NOTE: could not read the gripper (...); assuming closed jaws` and
applies no correction. In that case the jaws really must be closed. `TODO(verify in lab)`:
confirm that `depth` reads 0 with the jaws closed and grows as they open.

### 4. Ray meets table

With `cam_to_base` known, a pixel's ray can be written in `base_link`: it starts at the camera
position `C` and runs in direction `D` (the pixel's camera-frame direction, rotated). The table
is the plane through the board origin `P0` with normal `n` (the board's Z axis). The ray
`C + t * D` meets the plane at

```text
t = ((P0 - C) . n) / (D . n)
```

(`monocular.rays_to_plane`). If the ray is parallel to the table, or the intersection is behind
the camera, there is no answer and the pixel is dropped.

This is exact for anything flat on the table, which is why a paper marker is the calibration
check target. It is wrong for anything with height.

### 5. The height problem

A camera looking across the table sees the *top* of an object in front of table that is further
away. Follow the ray through a point on the object's top edge down to the table and it lands
behind the object:

```text
        C  camera
        |   `-.
        |        `-.
      H |              `-.
        |                   `-.
        |                   +------+
        |                 h | obj  |   `-.
        |                   |      |         `-.
--------+-------------------+------+---------------*------  table
        |<------- d ------->|<---->|<----- s ----->|
                           footprint    shadow
```

By similar triangles, a point at height `h`, at horizontal distance `d` from the spot under a
camera that is `H` above the table, lands on the table pushed away from the camera by

```text
s = d * h / (H - h)
```

So the mask of a standing object, laid onto the table, covers the object's true *footprint*
plus a "shadow" stretching away from the camera. Taken at face value the object looks longer
than it is, its centre slides away from the camera, and its apparent long axis is pulled toward
the line of sight.

**A worked example, from the test scene.** `test/test_monocular.py` renders a 40 mm cube at
(-0.30, 0.12) m, seen by a camera at (-0.55, -0.85, 0.55) m above a table at z = 0.012 m. That
puts the camera H = 0.538 m above the table and d = 1.00 m away horizontally: a line of sight
about 62 degrees from vertical.

- Arithmetic from the formula above (worked by hand from the scene geometry, not asserted by a
  test): s = 1.00 x 0.04 / (0.538 - 0.04) = 0.080 m. The top of a 40 mm cube lands 80 mm behind
  its base. The silhouette on the table covers about 58 cm2 instead of the footprint's 16 cm2,
  and its centre of area sits about 42 mm beyond the cube's true centre.
- What the tests assert, on rendered images: treating the cube as flat (`object_height=0.0`)
  puts the answer **more than 12 mm** from the truth
  (`test_ignoring_height_smears_away_from_camera`; a loose lower bound, and that test also uses
  the rough single-view focal length). Giving the true height (0.04) and the true focal length
  puts it **within 6 mm**, with both footprint sides within 8 mm of 40 mm and a yaw of exactly 0
  because the footprint is square again (`test_block_localized_within_6mm_when_height_is_known`).

**Removing the shadow.** If the height is known, so are the shadow's length and direction, and
it can be subtracted (`monocular.table_footprint`):

1. Project the mask's outline onto the table and draw it as a filled shape on a grid of 1 mm
   cells in the board frame. Cells are equal areas of table, so nothing is biased toward the
   camera the way image pixels are.
2. Compute the shadow vector `s` for this object (direction: away from the camera; length: the
   formula above).
3. For an object with vertical sides, the silhouette is the footprint *swept* along that one
   short segment. Sweeping can be undone: keep a cell only if the whole segment starting at that
   cell and pointing away from the camera stays inside the silhouette. Footprint cells pass
   (their segment runs through the object and its shadow). Shadow cells fail (their segment
   runs off the far end). In image-processing terms this is an *erosion*.

What remains is the footprint. Its mean is the centre, its principal axis is the yaw, and the
grasp point is the centre lifted by `h`.

**What a wrong height does.** The erosion removes exactly as much as it is told to.

- Height too small: some shadow is left; the centre stays shifted away from the camera.
- Height too large: part of the real footprint is eaten from the far side; the centre shifts
  toward the camera.
- Height far too large: nothing survives, and the node refuses ("nothing is left of the object
  after removing a ... mm height shadow"). In the test scene, claiming 120 mm for the 40 mm cube
  is refused (`test_overstated_height_is_refused_not_guessed`).

The size of the shift is about half the leftover shadow: `(height error / 2) x tan(angle)`,
where the angle is that of the line of sight measured from vertical (so tan = d / H). For the
test camera, tan is about 1.9, so each 10 mm of height error moves the centre by roughly 9 mm
(hand arithmetic, not a test result). Two practical consequences: measure heights with a ruler,
and mount the camera high and looking down steeply, because a low, shallow view multiplies every
height error.

**Where the height comes from.** [`scripts/lab_objects.yaml`](../scripts/lab_objects.yaml) maps
names to heights in metres. The node picks the longest listed name contained in the query;
anything unlisted uses `default_height_m` (0.03). The file currently lists `block` and `cube` at
0.04 and `marker` at 0.0. The `marker NN` query always uses height 0.

**What is still approximate even with the right height.** The method assumes vertical sides and
uses one shadow vector for the whole object. The top face is also slightly magnified, by
H / (H - h), which the erosion does not undo. These are why the rendered-scene bound is 6 mm and
not zero.

## The focal length: why one view is not enough

Everything above needs the focal length `f`. Nobody calibrated a laptop webcam, so the code can
estimate `f` from the board itself (`monocular.estimate_focal`): a flat pattern seen at an angle
shows perspective (the far squares look smaller), and how strong that effect is depends on `f`.
The code tries focal lengths between 0.4 and 3 times the image width and keeps the one whose
best pose reprojects the corners most exactly.

The trouble is that the board is small and flat. From about a metre away, its far edge is only a
little further from the lens than its near edge, so the perspective effect is weak. A longer
lens further away and a shorter lens closer in produce nearly the same picture of the board, and
the reprojection error barely changes between them. The estimate is therefore loose even from a
perfect image: `test_single_view_focal_is_only_approximate` accepts anything within 15 % of the
true value on a rendered image, and the code comments put the typical error around 9 to 10 %.

A wrong `f` matters little *at* the board (the pose is solved so that the board fits) and more
the further an object is from it, because the error acts like a wrong camera distance.

Two remedies:

1. **Multi-view calibration** (`monocular.calibrate_intrinsics`, the `intrinsics` command). Move
   the camera around the board for about 20 seconds. Each view on its own is ambiguous, but one
   focal length has to explain all of them, and views from different angles and distances
   disagree about every wrong value. The code needs at least 6 views with at least 10 corners
   each, solves for `f` and one radial distortion term (`k1`), and refuses a result whose RMS
   error exceeds 2 px. On 16 rendered views the test recovers `f` within 3 %
   (`test_multi_view_calibration_recovers_focal`).
2. **Factory calibration, for the ZED** (`camera.zed_factory_calibration`, the `zed` command). A
   ZED 2i without its SDK is still an ordinary USB camera: it delivers both lenses side by side
   in one wide frame, and the left half is a usable colour image. Stereolabs publishes each
   unit's factory-measured lens parameters by serial number at
   `https://calib.stereolabs.com/?SN=<serial>`. The script reads the left lens's focal length,
   image centre and distortion from that file, so no measuring is needed.

## Calibration checklist

Run from the repository root, in the lab container. The `touch` step needs the robot driver
running; the camera steps need the camera free (stop the perception node first). `--device N`
and `--square-mm X` are *global* options and go before the command name.

You need: a printer, a ruler and tape.

1. **Print the board.**

   ```bash
   python3 scripts/lab_table_calibration.py print
   ```

   This writes [`docs/calibration/table_board_letter.pdf`](calibration/table_board_letter.pdf):
   page 1 is the board with its corners marked by crosses and numbered 1 to 4; page 2 has
   verification markers 40 to 43 (50 mm). Print at 100 % ("Actual size"), not "fit to page".

2. **Measure it.** Five squares must measure 175 mm. If they do not, divide your measurement by
   5 and pass it as `--square-mm` to **every** later command (for example
   `python3 scripts/lab_table_calibration.py --square-mm 34.8 fit`). The value is not remembered
   between commands.

3. **Tape it flat** on the table where the camera sees all of it, the arm can reach all four
   corners, and it will not be covered by the arm's observe pose or by objects. Objects on or
   within 10 mm of the board are never picked, so keep it out of the working area. A curled
   corner breaks the "flat" assumption; tape all edges.

4. **Tell the software about the lens.** One of:

   - ZED used as a USB camera. Download the unit's file from
     `https://calib.stereolabs.com/?SN=<serial>` into `docs/calibration/`, then:

     ```bash
     python3 scripts/lab_table_calibration.py zed --conf docs/calibration/SN<serial>.conf
     ```

     Defaults: `--mode FHD` (1920 x 1080 per lens; also `2K`, `HD`, `VGA`) and camera index 2
     unless `--device` is given.

   - Any other webcam:

     ```bash
     python3 scripts/lab_table_calibration.py intrinsics
     ```

     Move the camera slowly around the board for the 20 seconds it captures (`--seconds`), so
     the board is seen from clearly different angles and distances. Then put the camera where
     it will stay.

   - Skip this step and the focal length is estimated from each image (looser; `fit` prints a
     note).

   The result is saved in `scripts/.lab_camera.json`.

5. **Place the camera and look.** Confirms the camera, the index and the view before any
   touching.

   ```bash
   python3 scripts/lab_table_calibration.py aim     # live view while you position the camera
   python3 scripts/lab_table_calibration.py look    # one saved frame and a printed report
   ```

   `aim` serves a live picture at `http://localhost:8089` (`--port`; it stops after `--seconds`,
   default 600, or on Ctrl-C). Open it in a browser on the same machine. The overlay shows
   `board corners: N/24` and, once at least 8 are found, how far the camera is from the board,
   how high above the table, and how steeply it looks down. The text is green when the camera
   looks down by 30 degrees or more and orange when the view is shallower; a steep view keeps
   the height shadow short (see [the height problem](#5-the-height-problem)).

   `look` saves `docs/calibration/look.jpg` and prints how many board corners it saw (24 is all
   of them), the image size, the reprojection error, and the camera's height above and distance
   from the board.

6. **Touch the four corners.** Start the robot driver
   (`ros2 launch ur7e_bringup ur7e_bringup.launch.py`). Close the gripper. For each corner
   N = 1, 2, 3, 4: jog with the pendant's Move tab until the closed fingertips rest on the cross
   at corner N with the tool pointing straight down, let the arm settle, then:

   ```bash
   python3 scripts/lab_table_calibration.py touch --corner N
   ```

   It prints the fingertip point, `tilt_deg`, and the jaw width and `depth` it read from the
   gripper. Keep the tilt under 5 degrees. If it prints
   `NOTE: could not read the gripper ...; assuming closed jaws`, check that the jaws really are
   closed (pass `--robot-ip` if the robot is not at the default address). Touching a corner
   again overwrites it. The touches are kept in `scripts/.lab_touches.json`, which persists
   between sessions: if the board has moved, redo all four, not just one.

7. **Fit.**

   ```bash
   python3 scripts/lab_table_calibration.py fit
   ```

   This writes `scripts/lab_table.json`, the file the node loads, and prints `residual_mm` (one
   per corner), `table_z_m`, `table_tilt_deg` and `focal_px`. Under 3 mm per corner is good. At
   10 mm or more it refuses and prints the four measured side lengths against the expected
   175, 245, 175, 245 mm: re-touch the corner whose sides disagree. `fit` copies the lens data
   from step 4 into the file, so if you redo step 4 later, run `fit` again (no need to re-touch).

8. **Check.**

   ```bash
   python3 scripts/lab_table_calibration.py check
   ```

   It saves `docs/calibration/check.jpg`: the camera's view with the reachable part of the
   table tinted, the four board corners circled and numbered, and the `base_link` axes drawn at
   the table point under the robot's base (X red, Y green, Z blue, 10 cm long). The circles
   always land on the board, because they come from the same picture. The informative part is
   the axes and the tint: if the axes sit where the robot's base is and point along its X and Y,
   the touches and the camera agree. It also prints the camera's position in `base_link`, which
   can be compared with a tape measure.

9. **Verify with the arm.** With the driver running and the pendant program playing, start
   MoveIt (`ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl`) and the perception node
   ([below](#running-the-node)). Cut out marker 42 with a white border, lay it flat within
   reach and off the board, and run:

   ```bash
   python3 scripts/lab_pick.py --verify-marker 42              # plan only
   python3 scripts/lab_pick.py --verify-marker 42 --execute    # the arm moves
   ```

   The arm hovers its closed fingertips 10 mm above where perception says the marker's centre
   is. Measure the offset and type it in; it is appended to
   `docs/perception-evidence/touch-verification.jsonl`. The acceptance in `lab_pick.py` is five
   placements within 1 cm. This is the only step that tests the whole chain (lens model, board
   anchor, robot kinematics) at once.

When to redo what:

| What changed | Redo |
|---|---|
| Camera moved or bumped | Nothing. The pose is re-solved in every capture. |
| Board moved, re-taped or reprinted | Steps 6 to 9 (all four touches). |
| Different camera, lens, or resolution | Steps 4, 7, 8, 9. Restart the node. |
| Gripper or fingertips changed | Steps 6 to 9. |

## Running the node

```bash
ros2 run ur7e_perception webcam_perception_node --ros-args \
  -p calibration:="$PWD/scripts/lab_table.json" \
  -p objects_file:="$PWD/scripts/lab_objects.yaml" \
  -p snapshot_dir:="$PWD/log/perception"
```

The node loads the calibration once at start-up, so restart it after a new `fit`. The camera
settings (index, resolution, pixel format, left-half crop) travel inside the calibration file,
because the lens data is only valid for the resolution it was made with.

For each `detect_object` request the node:

1. takes the most recent frames (younger than `max_frame_age`) and undistorts them;
2. finds the board and solves the camera pose *now*;
3. runs the detector on the newest frame, keeping only candidates that sit on the reachable
   table and not on the board;
4. stores image, camera pose, mask and assumed height under a new capture ID.

`locate_object` then turns that stored capture into a pose, checks the reach limits again on the
final point, and publishes the result.

| Parameter | Default | Meaning |
|---|---|---|
| `calibration` | (required) | Path to the file written by `fit` |
| `objects_file` | empty | Height table; without it every object is `default_height` tall |
| `default_height` | 0.03 | Height in metres for unlisted objects (the file's `default_height_m` overrides it) |
| `backend` | `auto` | `auto`, `owlv2`, or `color` (see the README) |
| `threshold` | 0.10 | OWLv2 score threshold |
| `ambiguity_ratio` | 0.85 | A second, distinct OWLv2 box scoring at least this fraction of the best is "ambiguous" |
| `device` | -1 | Camera index; -1 uses the one saved in the calibration |
| `average_frames` | 5 | Frames used for the board solve |
| `max_frame_age` | 1.0 | Seconds; older frames are not used |
| `max_capture_age` | 60.0 | Seconds a capture ID stays valid |
| `reach_min`, `reach_max` | 0.20, 0.80 | Allowed horizontal distance of a target from the base origin, metres |
| `base_frame` | `base_link` | Frame name stamped on the pose |
| `snapshot_dir` | empty | If set, each successful `locate` saves `<capture_id>.jpg` and `latest.jpg` there |
| `publish_rate` | 3.0 | Hz of the live image topic |
| `replay_image` | empty | A still image used instead of a camera (rehearsal; the only mode that accepts a synthetic calibration) |

| Topic | Content |
|---|---|
| `/camera/rgb` | The live camera image as it arrives (not undistorted) |
| `/perception/annotated` | The frozen capture of the last successful `locate`, with mask, box and grasp point drawn (latched) |
| `/perception/grasp_pose` | The last grasp pose (latched) |
| `/perception/last_result` | JSON: position, yaw, assumed height, footprint size, table height, board reprojection error, confidence (latched) |

To rehearse without a camera or robot, see "Single camera, rehearsal" in the
[package README](../src/ur7e_perception/README.md) and `scripts/lab_pick_rehearsal.sh`.

## Failure messages and what to do

Every refusal comes back as `success=false` with one of these strings in `reason` (or, for
start-up problems, as the error the node exits with). `{...}` marks a value filled in at run
time.

### At start-up

| Message | Meaning | What to do |
|---|---|---|
| `set the calibration parameter (scripts/lab_table_calibration.py writes the file)` | No `calibration` parameter was given | Pass `-p calibration:=...` |
| `the webcam node needs a physical table calibration` | The file is marked synthetic and a live camera was requested | Use a file from `fit`, or add `replay_image` for a rehearsal |
| `replay_image could not be read` | The rehearsal image path is wrong or not an image | Check the path |
| `cannot open camera {N} (already in use?)` | The camera index is wrong, or another program holds the camera | Stop the other program (often the calibration script or a second node); try `-p device:=N` |
| `invalid camera-to-base rigid transform` | The calibration file's board transform is damaged | Run `fit` again |

### During `detect_object`: camera and board

| Message | Meaning | What to do |
|---|---|---|
| `query must be a short noun phrase` | Empty, multi-line, or longer than 120 characters | Send only the object's name |
| `no fresh camera frame` | No frame arrived within `max_frame_age` | Check the cable; restart the node |
| `camera image is blank (privacy shutter closed?)` | The image has no texture at all | Open the shutter, remove the lens cap |
| `table board not visible` | No board marker was found in any recent frame | Uncover the board, turn the camera toward it, add light |
| `only {N} board corners visible; need 8 (is the arm or an object covering the board?)` | Part of the board is hidden or out of frame | Move the arm to its observe pose; clear the board |
| `board pose could not be estimated` | No valid camera pose on the printed side of the board | Usually a very poor or partial view; reposition the camera |
| `board reprojection error {X} px is too high (blurred image, bent paper, or wrong focal length)` | The solved pose does not explain the corners to within 2.5 px | Hold the camera still, flatten and re-tape the paper, redo the lens step |

### During `detect_object`: finding the target

| Message | Meaning | What to do |
|---|---|---|
| `target not detected` followed by an optional reason in brackets | OWLv2 returned no acceptable box. The bracket gives why the best candidate was discarded: `box covers a quarter of the image`, `not on the table`, `out of reach ({D} m from the base)`, or `on the calibration board` | Move the object into reach and off the board; try a plainer name |
| `multiple plausible targets; clarify query` | Two distinct boxes scored similarly | Remove the lookalike or add a distinguishing word |
| `the colour detector needs exactly one colour word (red, orange, yellow, green, blue, purple)` | `backend:=color` and the query has none, or several | Include exactly one of those words, as its own word |
| `no {colour} object on the table` | No blob of that colour (at least 150 px) on the reachable table | Check lighting and that the object is within reach and off the board |
| `more than one {colour} object; be more specific` | A second blob is more than half the size of the largest | Remove one |
| `marker {N} missing or seen more than once` | The `marker N` query did not find exactly one such marker | Lay the marker flat and fully in view; use only one copy |
| `empty detection box` | The detector's box is degenerate (under 4 px) | Retry; if it persists the object is too small in the image |
| `no foreground separable from the table by colour` | GrabCut found nothing inside the box that differs from its surroundings | The object blends into the table; change the background or use a coloured object |
| `foreground is too small inside the detection box` | What GrabCut kept is under 5 % of the box | Same as above |

### During `locate_object`

| Message | Meaning | What to do |
|---|---|---|
| `unknown or evicted capture ID` | The ID was never issued, or more than 8 newer captures exist | Detect again |
| `capture is stale; detect again` | The capture is older than `max_capture_age` | Detect again |
| `object height must be 0..0.3 m` | The height from the objects file is outside the accepted range | Fix the entry in `lab_objects.yaml` |
| `mask too small to localize` | The mask has fewer than 50 pixels | Move the object or camera closer |
| `mask does not project onto the table` | Part of the outline's rays miss the table (the mask reaches above the table's horizon in the image) | Tilt the camera down; check the mask in the snapshot |
| `camera is not above the object` | The camera is less than 5 cm above the object's assumed top | Raise the camera; check the height entry |
| `object silhouette is implausibly large` | The outline spans more than 1.5 m of table | The mask is wrong (often a near-horizontal view); check the snapshot |
| `nothing is left of the object after removing a {H} mm height shadow: its height in the objects file is too large` | The assumed height removes the whole silhouette | Measure the object and correct `lab_objects.yaml` |
| `object is {D} m from the base: out of reach` | The final grasp point is outside `reach_min`..`reach_max` | Move the object |

Three more messages exist in `monocular.py` but are not produced by the node today:
`the camera sees above the table horizon; tilt it down` (from `visible_table`),
`object is outside the workspace` (only when a workspace polygon is passed), and
`no colour rule for "{colour}"`.

`scripts/lab_pick.py` reads these strings: a reason containing `not detected`,
`object on the table`, `missing or seen`, `multiple plausible` or `more than one` is treated as
"not found" (the orchestrator looks again once); anything else in DETECT is a perception error.
Changing the wording of a message therefore changes the retry behaviour.

### From the calibration script

| Message | Meaning | What to do |
|---|---|---|
| `only {N} usable board views; need at least 6 from clearly different angles` | `intrinsics` saw the board well in too few frames | Move more slowly, keep the whole board in view, vary the angle |
| `intrinsic calibration did not converge (RMS {X} px)` | The views could not be explained by one lens model to within 2 px | Repeat with a flat board, good light, and no motion blur |
| `camera image is blank: open the privacy shutter` | As above | Open the shutter |
| `cannot open camera ... (is the perception node using it?)` | The camera is busy | Stop the node first |
| `no base_link -> tool0 transform: is the driver running?` | `touch` got no robot pose from TF | Start the driver |
| `the arm is moving; let it settle and run this again` | The flange moved more than 0.5 mm while sampling | Wait and repeat |
| `NOTE: could not read the gripper (...); assuming closed jaws` | `touch` could not ask the RG2 for its jaw state, so no open-jaw correction was applied | Make sure the jaws are closed, or fix `--robot-ip` / `--tool-index` and touch again |
| `WARNING: the tool is tilted ... deg from straight down ...` | The fingertip point now depends on `--tool-length` being exact | Straighten the tool and touch again |
| `corners not touched yet: ...` | `fit` needs all four touches | Touch the listed corners |
| `calibration residual exceeds 1 cm. Measured side lengths ...` | The four touches do not form the expected rectangle | Re-touch the corner whose sides are wrong; check `--square-mm` |
| `unknown ZED mode ...`, `cannot read ZED calibration file ...`, `... has no [...] section` | The `zed` step could not use the file | Check the path, the mode, and that the file is the unit's own |

## What changes when the depth camera returns

The services, the capture-ID contract and the pose convention are identical in both nodes, so
the orchestrator and `lab_pick.py` call them the same way. What changes:

- **Node.** Run `perception_node` (`nodes.py`) fed by the ZED wrapper's rectified colour and
  registered depth topics, instead of `webcam_perception_node`. See
  [`TASK2_SOFTWARE.md`](TASK2_SOFTWARE.md).
- **Where 3-D comes from.** Each mask pixel gets its own measured distance. The table is no
  longer needed to make sense of a ray, the height is measured instead of looked up, and
  `lab_objects.yaml` is not used. Objects that are not simple upright shapes stop being a
  special problem.
- **Calibration.** A fixed `camera_to_base` from the eye-to-hand procedure (board on the gripper,
  many arm poses, held-out verification points) replaces the per-capture board solve. The price
  is that the camera must then be rigidly mounted: a bump silently invalidates it, which is why
  that node checks a permanent workspace marker in every capture.
- **Workspace test.** A polygon and a height band (`workspace`, `table_z`, `max_z`) replace the
  reach ring.
- **Masks and detectors.** Depth separation (or NanoSAM on the Jetson) replaces colour-based
  GrabCut; `nanoowl` becomes available.
- **What stays.** `scripts/lab_pick.py` takes its table plane and gripper length from the
  table calibration file, so the four touches remain useful for motion heights unless that
  adapter is changed. The single-camera node remains the way to rehearse and a fallback for
  sessions without the SDK.

Some defaults differ between the two nodes and should be reconciled deliberately rather than by
accident: the yaw threshold (`axis_ratio` 1.2 vs 2.0), `max_capture_age` (15 s vs 60 s), the
OWLv2 threshold (0.15 vs 0.10) and prompt wording, and the size of printed marker 42 (40 mm in
the depth path's drift check, 50 mm on this path's printable sheet).

## Measuring the height with the second lens

The honest limit of everything above is the height list. A ZED has a second lens 120 mm from
the first, and two views of the same point do fix its distance. `ur7e_perception/stereo.py`
uses that, without the Stereolabs SDK, to measure an object's height directly. This summary
follows its docstrings.

A point seen by the left lens at column `x` appears in the right lens at `x - d`. The shift `d`
is the *disparity*, and

```text
depth = focal_px * baseline_m / d
```

To find `d` the code has to search the right image for the patch around each left pixel. That
search is only practical along a single image row, so both raw images are first *rectified*:
warped, using the factory calibration of both lenses and of the small rotation and offset
between them, into two virtual pinhole cameras with exactly parallel axes
(`cv2.stereoRectify`). Semi-global block matching (`cv2.StereoSGBM`) then does the row search,
and three filters (uniqueness, left-right consistency, speckle removal) discard matches that
cannot be trusted. `StereoRig.from_zed_conf` builds all of this from the same `SN<serial>.conf`
file the `zed` calibration step uses, and `Camera.read_pair` delivers both halves of one
exposure.

`object_height` then takes the 3-D points found inside an object's mask, measures each one's
distance above the table plane, and returns the 90th percentile. The reasoning: a mask seen from
the side covers the top of the object, the sides facing the camera, and a rim of table, so the
mean lands somewhere in the middle; the single highest point is whatever mismatch happened to
stick out; an upper percentile sits inside the top surface as long as the top supplies more than
a tenth of the points. It refuses if fewer than 30 points lie between the table and 0.3 m above
it.

What limits it, as the module states:

- **Resolution.** One pixel of disparity is worth `depth^2 / (focal * baseline)` metres. The
  docstring works this out for this camera at 1080p as about 2 mm at 0.5 m, 5 mm at 0.8 m and
  11 mm at 1.2 m (arithmetic from the formula, not a measured accuracy). It grows with the
  square of the distance: keep the camera close.
- **Texture.** Matching needs something to match. A plain table top or a featureless lid gives
  few or no points. If only the textured sides of an object match, the height reads low.
- **Calibration.** The factory file describes the camera as it left the factory.
  `StereoRig.rectification_error` measures, on any image pair, how well the two images still
  line up row for row.

What its tests assert (`test/test_stereo.py`), on **rendered** left/right pairs in which every
surface carries a random texture: boxes 40 mm and 80 mm tall, seen from about 0.8 m, are
measured within 5 mm, with exactly parallel lenses and with lenses slightly tilted and offset
as on a real ZED; table points land within 2 mm of the table plane; and a blank table gives
almost no points and is refused rather than measured wrongly. On made-up point sets (no images)
the tests also show that when stereo and the board disagree by a common 8 mm, measuring the
object against stereo's own points on the table next to it brings the error back under 2 mm.
Two tests use the lab ZED's factory file, and one checks row alignment on a stored real frame;
they are skipped when those files are absent, and the real-frame test is a calibration sanity
check, not an accuracy measurement.

As of this writing `stereo.py` is a library only: `webcam_node.py` does not call it, so the
node's heights still come from `lab_objects.yaml`. Once it is wired in, the height list becomes
a fallback and the rest of this path (board, touches, ray-plane intersection, shadow removal)
stays the same.

## What ran in the lab on 2026-10-02

The explanation above is built around a printed board. In the lab no board could be printed, so
the same geometry was fed differently — and this is what actually located an object for the
first real pick. Chronology and every number: [RUNBOOK.md](RUNBOOK.md) (2026-10-02 entry);
acceptance record: [LAB_2026-10-02_WEBCAM_PICK.md](LAB_2026-10-02_WEBCAM_PICK.md); the commands
in order: [COLD_START.md](COLD_START.md) §3.

- **The camera.** A ZED 2i on the laptop's USB port, used as a plain webcam (`/dev/video2`,
  3840×1080 side by side, left half only, ~19 fps). No SDK, no GPU. Its factory intrinsics for
  FHD (`fx` 1065.5 px, 8-coefficient rational distortion) came from
  `calib.stereolabs.com/?SN=35717973` into [calibration/zed2i_SN35717973.conf](calibration/zed2i_SN35717973.conf)
  and were loaded with `scripts/lab_table_calibration.py zed --conf …`. So the
  [focal length section](#the-focal-length-why-one-view-is-not-enough) was not needed: the
  factory file is better than anything a board sweep would estimate.
- **Camera-to-robot without a board** (`scripts/lab_camera_calibration.py`). The gripper holds a
  coloured object; `wave` moves it through a box grid of 27 poses at 3 heights (4–6 min at
  50–70 % slider); in each frame the camera finds the object by colour (a strict and a loose
  threshold, "lowest moving blob"); `solve` runs `solvePnPRansac` on the (robot pose, pixel)
  pairs with the factory intrinsics. The robot's own kinematics replace the printed grid: the
  arm *is* the board. Results over the day — the camera was bumped twice and recalibrated each
  time (about 6 min): 0.68 px RMS over 11 poses (white plug, first position); 5.0 px over 24 of
  27 (plug, second position); **3.2 px RMS over 15 of 17 poses** (blue toy hard hat, final
  position; the three lowest poses nearest the base were skipped — wrist flip or platform
  collision). `map` draws the `base_link` grid on a frame as a visual check: (0,0) landed on the
  base foot and the clamps on their grid cells.
- **The table plane by force, not by touching corners.** `touchdown --kind tips` lowers the
  closed fingertips in 3 mm steps and watches the wrist force/torque sensor (baseline noise
  ~0.1 N): the contact step went 0.82 → 39 N between tool0 z 0.150 and 0.147, so the plate is at
  **tool0 z = 0.1485 m**. This number is independent of the camera and survives camera bumps.
- **`--table-z`, the one number fixed by hand.** The wave records heights as "tool0 height when
  the held object's centre is at the plate", so the unknown distance from flange to object
  cancels out of the pose fit — but it reappears as the table height in that frame. A force
  touchdown of the *held* object failed (a 10 N grip lets the object slide at 0.2 N), so the
  value was fixed by setting the object down at a known point and comparing the camera's answer
  (`scripts/lab_detect.py`): **0.2025** for the hat held by its brim. The first guesses
  (0.1785 / 0.177) put objects 4–6 cm too far from the camera and one of them drove the jaws
  onto the platform edge. Get this right before any pick.
- **Fixed-camera mode.** `finish` writes `scripts/lab_table.json` with the camera pose stored in
  it (`source: physical`); `webcam_node.py` then trusts that pose instead of re-solving a board
  in every capture (`TableCalibration.fixed_view`). The trade is the one described in
  [the frames section](#the-frames-and-the-chain): bump the camera and the stored pose is wrong,
  so recalibrate. There is no board to hide, so the "board not visible" refusals do not apply.
- **The near-edge estimator replaces the height list for compact objects.** The
  [shadow-removal method](#5-the-height-problem) needs the object's height and, on the round hat,
  eroded too much. `localize_on_table(..., object_height=None)` instead takes the silhouette's
  edge nearest the camera (where the object meets the table, no shadow) and its width across the
  viewing direction (not smeared at all), and puts the centre half that width behind the near
  edge. No height needed; about 1 cm on rendered scenes. It is now the default
  (`default_height_m: -1` in `scripts/lab_objects.yaml`); a listed height switches an object
  back to shadow removal, which remains better for flat, box-like things of known height. The
  hat's footprint came out as 8.0 × 7.4 cm.
- **Detectors.** `backend:=auto` runs OWLv2 on the laptop CPU (~8 s per query; "blue helmet"
  scored 0.13–0.20, "blue hat" fell below the 0.10 threshold) and falls back to the colour
  detector. `blue` needed a saturation floor of 150: the black anodised breadboard reads as dark
  navy and was once segmented whole as "blue".
- **Accuracy observed:** camera (0.2356, 0.057) m vs the robot-placed hat at (0.24, 0.05) m —
  **0.8 cm**. One object, one placement, one day. It is a data point, not a characterisation;
  the ~10-object validation (task 2.6) is still to do.
- **Caveats.** The benchmark that followed showed the weak spot is not this measurement but what
  happens between measurements: the object is located once, then approached, grasped and set
  down without the camera checking again, and a round object rolls after release. At the plate's
  corner the hat's silhouette read as elongated (axis ratio 7.8) and the grasp yaw followed it.
  Re-locating before APPROACH and confirming departure on LIFT are the next items. The stereo
  height module below is still not wired into the node.

## Limits

- **Real-world accuracy has been measured once** — 0.8 cm, one object, one placement, with the
  print-free variant above. That is a data point, not a characterisation.
- **Height is assumed, not measured,** and the X/Y answer depends on it.
- **Upright, convex-ish shapes only.** The shadow removal assumes vertical sides. Overhangs,
  leaning objects and objects lying on other objects are outside the model.
- **The table must be flat and the board must lie in its plane.** The table plane is
  extrapolated from a 175 x 245 mm patch, so a small tilt error in the board grows with distance
  from it.
- **The board must be visible, and unobstructed, in every capture.**
- **The lens model is simple.** `intrinsics` estimates one focal length and one distortion term
  with the image centre fixed at the middle of the image.
- **Segmentation is by colour.** Low contrast between object and table gives poor masks, and
  the mask is the measurement.
- **The image size is recorded in the calibration file but not compared with the frames that
  actually arrive.** If the camera opens at a different resolution than it was calibrated at,
  the focal length is silently wrong. Check `size` in the output of `look`.
