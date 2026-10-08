# Manufacturer CAD for the arm and its tooling

Copied into the repo on 2026-10-08 from files collected by Nikola. These are
the reference geometry for the robot model: collision envelopes, TCP and
reach checks. Do not edit them; derive meshes or boxes from them instead.

| File | What it is | Source | SHA-256 (first 12) |
|---|---|---|---|
| `UR7e_1006923.step` | UR7e arm, UR part 1006923 (`1006923_Solid_B`) | Universal Robots download centre | `f27a7b314170` |
| `UR_terms_graphical_documentation.txt` | UR's Terms and Conditions for Use of Graphical Documentation, v1.01 | ships with the UR CAD | `5393cdef698b` |
| `OnRobot_Dual_Quick_Changer_v3.step` | **OnRobot Dual Quick Changer v3** (item 109878; STEP name `Dual QC_2-0`). Sits between the UR flange and the two grippers | OnRobot download centre | `86c9cb239deb` |
| `OnRobot_Quick_Changer_Tool_Side_v2_ST-RB-028-0023.step` | OnRobot Quick Changer **tool side** v2 (the plate on each gripper) | OnRobot download centre | `f8a1fdb2eedb` |
| `SoftGripper_PR-SG-004-1015.step` / `.pdf` | 4-finger soft gripper (parts `SG.IN.P.F4…`, `SG_FI.60`); drawing shows 66 × 66 mm body, 90 mm tall, 30° finger spread, 2×M5 mount. **Mounted on the second Dual Quick Changer face**, opposite the RG2 (see photos) | supplier download | `690c0aeefc7a` / `0bcd08edf1e9` |

The RG2 itself is modelled from the community `onrobot_description` meshes
(`ur7e.repos`); its datasheet values are in
[`../OnRobot_RG2_QC_UR_manual_v1.17.0_EN.pdf`](../OnRobot_RG2_QC_UR_manual_v1.17.0_EN.pdf).
The UR7e tech sheet is [`../UR7e_techsheet_en.pdf`](../UR7e_techsheet_en.pdf).

## Licensing

- **Universal Robots:** the UR CAD may be used for simulation, path planning
  and collision avoidance. Section 1.2 allows non-commercial academic use.
  Public sharing is allowed only together with UR's T&Cs (section 1.4), so
  `UR_terms_graphical_documentation.txt` must stay next to `UR7e_1006923.step`.
  Derived models belong to UR (section 2.2). Copyright notice for any rendering:
  "© 2023 Universal Robots A/S. Use hereof is subject to Universal Robots A/S
  Terms and Conditions for Use of Graphical Documentation."
- **OnRobot and the soft-gripper supplier:** public downloads with no licence
  text included. Kept here for the same simulation and collision-checking purpose.

## Dual Quick Changer geometry (measured from the STEP, 2026-10-08)

Measured with OpenCascade (`cadquery-ocp`) on `OnRobot_Dual_Quick_Changer_v3.step`.
STEP frame: **+Y is the robot flange axis** (the robot side is at the top, +Y);
units are mm.

| Quantity | Value | Check |
|---|---|---|
| Bounding box | 128 × 97 × 71 mm (X × Y × Z) | OnRobot lists 127 × 96 × 71 mm |
| Weight | 0.41 kg | OnRobot manual §8.1.1 |
| Robot-side interface | Ø63 boss on the +Y axis, top at Y = +49; largest +Y face at Y = +44.5 | UR flange (ISO 9409-1-50-4-M6) |
| Tool interfaces | two flat mounting faces, centres at **X = ±34.6, Y = −10.7, Z = 0** | — |
| Tool axis directions | outward normals **(±0.866, −0.5, 0)** | — |
| Tilt of each tool from the flange axis | **60°** (each); **120°** between the two tools | — |
| Tool face offset from the flange face | about 55–60 mm along the flange axis, ±34.6 mm sideways | depends on which Y face is the contact face |

So each tool points 60° away from the flange axis, in the plane of the two
tools. A gripper on either side does **not** point along `tool0`'s Z axis.

### What that means for the RG2 on it (estimate)

OnRobot gives the RG2 TCP as Z = 200 mm from its own mount, at 0° (manual
§8.3.1). The tool-side plate adds about 28 mm (bounding box of the tool-side
STEP). A closed-finger tip therefore sits roughly 230 mm out along the tilted
axis from the changer's tool face, which puts it about **0.23 m sideways and
0.17 m below** the flange. That is outside the 0.20 × 0.20 × 0.28 m
single-gripper envelope in `scripts/lab_arm_moveit.launch.py`.

**Confirmed by photo (2026-10-08, `photos/`):** the Dual Quick Changer is
on the UR flange, the RG2 on one face and the 4-finger soft gripper on the
other, splayed in a V. Neither tool points along `tool0`.

The lab code assumes the opposite: it plans `tool0` straight down
(`top_down_quaternion` in `scripts/lab_pick.py`), and its single envelope box
is centred on the `tool0` axis (`scripts/lab_arm_moveit.launch.py`). So during
a "top-down" pick the RG2 actually points about 60° off vertical, and the soft
gripper sticks out on the other side, unmodelled. This fits the 2026-10-02
benchmark (only the first grasp was clean, later ones grazed or missed) and the
2026-10-06 finger-on-forearm protective stops. The exact angle and fingertip
position should still be measured on the arm before they go into the model.

## Photos

| File | Shows |
|---|---|
| `photos/2026-10-08_dual_qc_side.jpg` | Dual Quick Changer on the UR flange (OnRobot label), soft gripper on the upper face |
| `photos/2026-10-08_dual_qc_from_below.jpg` | Both tools from below: soft gripper (left) and RG2 with ArUco markers (right), splayed in a V |

## Reproduce the measurements

```bash
python3 -m venv /tmp/cadenv && /tmp/cadenv/bin/pip install cadquery-ocp
```

Then read the STEP with `OCP.STEPControl.STEPControl_Reader`, take the bounding
box (`Bnd_Box.CornerMin/CornerMax`) and list planar faces with their area,
outward normal and centroid (`BRepAdaptor_Surface`, `BRepGProp.SurfaceProperties_s`).
The two largest planar faces with normals (±0.866, −0.5, 0) are the tool mounts.
