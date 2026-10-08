#!/usr/bin/env python3
"""Replay a planned move offline and report where the WHOLE arm goes (no motion).

Written after 2026-10-06, when four calibration approach moves ended in
protective stops (C153: the arm was pushed off its path) and the operator
saw the arm heading into itself. The planner's collision check and the
joint-excursion numbers had both said "fine"; neither says where the links
actually travel. This does: it plans the straight tool line exactly as the
lab executors do, then runs forward kinematics for every link at every
waypoint and reports

* how far the tool tilts away from straight down along the "straight" line,
* the closest approach of the gripper (flange to fingertips, plus what hangs
  below) to the arm's own upper arm and forearm, axis to axis,
* each joint's total travel,

and draws side and top views of the swept arm.

    python3 scripts/lab_path_audit.py --from-pose front --to 0.30 -0.02 0.36
"""
import argparse
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

CHAIN = ('base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link',
         'wrist_2_link', 'wrist_3_link', 'tool0')
# Below the flange: closed fingertips at 0.258 m, a held hat ~0.10 m lower.
HANG_M = 0.36


def segment_distance(p1, q1, p2, q2):
    """Shortest distance between segments p1-q1 and p2-q2."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    denom = a * e - b * b
    s = np.clip((b * f - c * e) / denom, 0, 1) if denom > 1e-12 else 0.0
    t = np.clip((b * s + f) / e, 0, 1) if e > 1e-12 else 0.0
    s = np.clip((b * t - c) / a, 0, 1) if a > 1e-12 else 0.0
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


# The gate the waves use: the gripper axis (flange to whatever hangs below)
# must stay this far from the arm's own upper-arm and forearm axes along the
# whole planned path. Gripper body 0.08 m half-width + forearm 0.05 m radius
# + margin. The 2026-10-02/06 `front` pose starts at 0.101 m: it fails.
MIN_GAP_M = 0.17  # 2026-10-06: gripper envelope widened to 0.10 m half-width
# Fraction of the elbow-to-wrist-1 line treated as forearm body (see gaps()).
FOREARM_REACH = 0.7


def sweep(executor, state, points, every=4):
    """FK of every chain link at every ``every``-th planned waypoint (and the last)."""
    from moveit_msgs.srv import GetPositionFK
    from scipy.spatial.transform import Rotation

    from lab_stereo_calibration import with_joints
    frames = []
    for joints in points[::every] + [points[-1]]:
        req = GetPositionFK.Request(robot_state=with_joints(state, joints),
                                    fk_link_names=list(CHAIN))
        req.header.frame_id = 'base_link'
        res = executor.call(GetPositionFK, '/compute_fk', req)
        frames.append({name: (np.array([p.pose.position.x, p.pose.position.y,
                                        p.pose.position.z]),
                              Rotation.from_quat([p.pose.orientation.x, p.pose.orientation.y,
                                                  p.pose.orientation.z, p.pose.orientation.w]))
                       for name, p in zip(CHAIN, res.pose_stamped)})
    return frames


def gaps(frames):
    """Return (tilts deg, gripper-to-arm axis gaps m) per frame."""
    tilts, clearances = [], []
    for f in frames:
        position, rotation = f['tool0']
        axis = rotation.apply([0, 0, 1])
        tilts.append(math.degrees(math.acos(np.clip(-axis[2], -1, 1))))
        hang = (position, position + HANG_M * axis)
        # The forearm stops FOREARM_REACH of the way to wrist 1: the wrist
        # itself carries the gripper ~0.14 m off its axis in every top-down
        # pose, so measuring to it makes the gate unpassable (2026-10-06 scan).
        elbow, wrist = f['forearm_link'][0], f['wrist_1_link'][0]
        arm = [(f['upper_arm_link'][0], elbow),
               (elbow, elbow + FOREARM_REACH * (wrist - elbow)),
               (f['base_link'][0], f['shoulder_link'][0])]
        clearances.append(min(segment_distance(*hang, *seg) for seg in arm))
    return tilts, clearances


def check_gap(executor, state, points):
    """Raise ValueError if the planned sweep brings the gripper within MIN_GAP_M of the arm."""
    _, clearances = gaps(sweep(executor, state, points))
    if min(clearances) < MIN_GAP_M:
        raise ValueError(f'refused: gripper passes {min(clearances):.3f} m from the arm\'s own '
                         f'links (axis to axis; need {MIN_GAP_M}) -- start from a more open pose')
    return min(clearances)


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--from-pose', default='front')
    cli.add_argument('--to', type=float, nargs=3, action='append', required=True,
                     metavar=('X', 'Y', 'Z'), help='tool0 target(s), base_link m')
    cli.add_argument('--png', default=os.path.join(_HERE, '..', 'log', 'path_audit.png'))
    args = cli.parse_args()

    import rclpy

    import lab_pick
    from lab_stereo_calibration import with_joints

    rclpy.init()
    executor = lab_pick.PickExecutor(None, max_speed_percent=100.0, joint_rate=0.09,
                                     max_excursion=3.6, plan_only=True,
                                     table_surface_z=0.1485 - lab_pick.APPROX_GRIPPER_LENGTH)
    executor.load_poses(lab_pick.SEED_POSES_FILE)
    state = with_joints(executor.fresh(), executor.poses[args.from_pose])
    start = executor.fk(state)
    reports, sweeps = [], []
    try:
        for target in args.to:
            pose = lab_pick.Pose()
            pose.position.x, pose.position.y, pose.position.z = target
            pose.orientation = start.orientation
            points = executor.plan_pose(state, pose)
            frames = sweep(executor, state, points)
            tilts, clearances = gaps(frames)
            travel = {j: round(max(p[j] for p in points) - min(p[j] for p in points), 3)
                      for j in points[0]}
            worst = int(np.argmin(clearances))
            report = {'to': target, 'waypoints': len(points),
                      'max_tilt_deg': round(max(tilts), 1),
                      'min_gripper_to_arm_axis_m': round(min(clearances), 3),
                      'at_fraction': round(worst / max(1, len(frames) - 1), 2),
                      'joint_travel_rad': travel}
            reports.append(report)
            sweeps.append(frames)
            print(json.dumps(report), flush=True)
    finally:
        executor.destroy_node()
        executor.motion.destroy_node()
        rclpy.shutdown()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(sweeps), 2, figsize=(12, 5 * len(sweeps)), squeeze=False)
    for row, (frames, report) in enumerate(zip(sweeps, reports)):
        for col, (i, j, label) in enumerate(((0, 2, 'side: x vs z'), (0, 1, 'top: x vs y'))):
            ax = axes[row][col]
            for k, f in enumerate(frames):
                chain = np.array([f[name][0] for name in CHAIN])
                tip = f['tool0'][0] + HANG_M * f['tool0'][1].apply([0, 0, 1])
                colour = plt.cm.viridis(k / max(1, len(frames) - 1))
                ax.plot(chain[:, i], chain[:, j], '-o', color=colour, alpha=0.5, ms=3)
                ax.plot([chain[-1, i], tip[i]], [chain[-1, j], tip[j]], '-', color='red',
                        alpha=0.5, lw=3)
            ax.set_title(f'to {report["to"]} -- {label}; min gripper-arm axis gap '
                         f'{report["min_gripper_to_arm_axis_m"]} m')
            ax.set_aspect('equal')
            ax.grid(True)
    fig.tight_layout()
    fig.savefig(args.png, dpi=80)
    print(json.dumps({'png': os.path.normpath(args.png)}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
