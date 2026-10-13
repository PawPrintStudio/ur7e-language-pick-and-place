#!/usr/bin/env python3
"""Bring the arm back to a taught pose (default ``observe``) one joint at a time.

Why this exists (2026-10-09): after pendant jogging or a protective stop the
arm can end up with its wrists wound far from ``observe`` (wrist_2 2 rad,
wrist_3 3 rad away). A single joint-space plan then has to unwind both at
once, and with the RG2 sticking out 60 deg on the Dual Quick Changer every
such path swept it through ``shoulder_link`` -- MoveIt refused, correctly.

Here each stage turns ONE joint along a straight line, with the other five
held still, so the motion is a single predictable rotation. Every stage is
collision-checked (``/check_state_validity``, the RG2 envelope and the lab
table at its *calibrated* height, built exactly as ``lab_console --pick``
does) every 0.02 rad, about 5 mm at the gripper, before anything moves, and
reports the lowest the RG2 TCP gets. Execution goes through
``JogExecutor.execute`` with the console's gate: NORMAL safety mode, External
Control running, pendant slider at or under ``--max-speed-percent``.

  python3 scripts/lab_unwind.py                       # preview only (default)
  python3 scripts/lab_unwind.py --execute             # run, confirming each stage
  python3 scripts/lab_unwind.py --order wrist_2 wrist_3 shoulder_lift wrist_1 elbow shoulder_pan

Default order: wrist_2 first (it folds the tool toward the forearm; checked
in the model on 2026-10-09, wrist_3 before it collides), lift, wrist_3, rest.
"""
import argparse
from copy import deepcopy
import json
import math
import os
import sys

import rclpy

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from lab_jog import JOINTS, WRIST_3_LIMIT, JogError  # noqa: E402

DEFAULT_POSES_FILE = os.path.join(_HERE, '.console_poses.json')
DEFAULT_CALIBRATION = os.path.join(_HERE, 'lab_table.json')
DEFAULT_ORDER = ['wrist_2', 'shoulder_lift', 'wrist_3', 'wrist_1', 'elbow', 'shoulder_pan']
STEP_RAD = 0.02     # interpolation and collision-check spacing
SKIP_RAD = 0.01     # a joint this close to its target is left alone
FK_EVERY = 10       # TCP height sampled every this many steps (and at the end)


def say(tag, payload):
    print(f'{tag:<8} {json.dumps(payload) if not isinstance(payload, str) else payload}',
          flush=True)


def with_joints(state, joints):
    """A copy of ``state`` with the named joint positions replaced."""
    moved = deepcopy(state)
    names = moved.joint_state.name
    moved.joint_state.position = [joints.get(n, p) for n, p in
                                  zip(names, moved.joint_state.position)]
    return moved


def stage_points(start, joint, target):
    """Straight-line waypoints for ``joint`` from start to target, start excluded."""
    delta = target - start[joint]
    n = max(1, math.ceil(abs(delta) / STEP_RAD))
    return [dict(start, **{joint: start[joint] + delta * k / n}) for k in range(1, n + 1)]


def check_stage(ex, state, points, joint):
    """Raise JogError at the first colliding waypoint; return the lowest TCP z (m)."""
    lowest = math.inf
    for i, pose in enumerate(points):
        probe = with_joints(state, pose)
        if not ex.valid(probe):
            raise JogError(f'{joint} would collide at {pose[joint]:.2f} rad '
                           f'({math.degrees(pose[joint]):.0f} deg); try another --order')
        if i % FK_EVERY == 0 or i == len(points) - 1:
            lowest = min(lowest, ex.fk(probe).position.z)
    return lowest


def wrist_3_target(current, goal, wrap):
    """The goal's wrist_3, or (with --wrap-wrist3) its nearest 2*pi equivalent in range."""
    if not wrap:
        return goal
    options = [goal + k * 2 * math.pi for k in (-2, -1, 0, 1, 2)]
    options = [o for o in options if abs(o) <= WRIST_3_LIMIT]
    return min(options, key=lambda o: abs(o - current))


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--pose', default='observe', help='taught pose to return to')
    cli.add_argument('--poses-file', default=DEFAULT_POSES_FILE)
    cli.add_argument('--calibration', default=DEFAULT_CALIBRATION)
    cli.add_argument('--execute', action='store_true', help='really move (default: preview)')
    cli.add_argument('--max-speed-percent', type=float, default=25.0,
                     help='refuse to move if the pendant slider is above this')
    cli.add_argument('--joint-rate', type=float, default=0.09,
                     help='nominal rad/s before the pendant slider')
    cli.add_argument('--order', nargs='+', default=DEFAULT_ORDER, metavar='JOINT',
                     help='joint order (names without _joint); unlisted joints go last')
    cli.add_argument('--wrap-wrist3', action='store_true',
                     help='let wrist_3 use the nearest full-turn equivalent of the goal '
                          '(only if no cable wraps around the wrist)')
    cli.add_argument('--yes', action='store_true', help='do not ask before each stage')
    args = cli.parse_args()

    order = [j if j.endswith('_joint') else j + '_joint' for j in args.order]
    unknown = [j for j in order if j not in JOINTS]
    if unknown:
        cli.error(f'unknown joint(s): {", ".join(unknown)}')
    order += [j for j in JOINTS if j not in order]

    import lab_pick
    from ur7e_perception.monocular import TableCalibration

    rclpy.init()
    # The console's executor class, so the planning scene's 'lab_table' goes
    # where the console puts it: at the calibrated board plane. (JogExecutor's
    # default box sits at the base plane, 1 cm too high here, and replacing the
    # shared scene object with it also skews the console's later plans.)
    # max_excursion only bounds the audit: one stage may turn a wrist up to a
    # full turn, and each stage is checked waypoint by waypoint.
    ex = lab_pick.PickExecutor(
        TableCalibration.load(args.calibration),
        max_speed_percent=args.max_speed_percent, joint_rate=args.joint_rate,
        max_excursion=2 * math.pi, plan_only=not args.execute,
        poses_file=args.poses_file, tool_link=lab_pick.PICK_LINK)
    try:
        ex.load_poses()
        if args.pose not in ex.poses:
            raise JogError(f'no pose "{args.pose}" in {args.poses_file} '
                           f'(known: {", ".join(sorted(ex.poses)) or "none"})')
        ex.ensure_table()
        state = ex.fresh()
        now = ex.joints_of(state)
        if not ex.valid(state):
            raise JogError('the arm\'s CURRENT pose already counts as a collision in the '
                           'planning model (gripper into the table or the robot), so no '
                           'move can start from it. Jog it clear on the pendant first; '
                           'RViz or /display_contacts shows which links touch')
        goal = dict(ex.poses[args.pose])
        goal['wrist_3_joint'] = wrist_3_target(now['wrist_3_joint'], goal['wrist_3_joint'],
                                               args.wrap_wrist3)

        # Preview every stage from the current pose before moving anything.
        stages = []
        virtual, at = state, dict(now)
        for joint in order:
            if abs(goal[joint] - at[joint]) < SKIP_RAD:
                continue
            points = stage_points(at, joint, goal[joint])
            lowest = check_stage(ex, virtual, points, joint)
            stages.append(joint)
            say('STAGE', {'n': len(stages), 'joint': joint.replace('_joint', ''),
                          'from_deg': round(math.degrees(at[joint]), 1),
                          'to_deg': round(math.degrees(goal[joint]), 1),
                          'turn_deg': round(math.degrees(goal[joint] - at[joint]), 1),
                          'nominal_s': round(abs(goal[joint] - at[joint]) / ex.joint_rate, 1),
                          'lowest_tcp_z_m': round(lowest, 3)})
            at = points[-1]
            virtual = with_joints(virtual, at)
        if not stages:
            say('DONE', f'already at "{args.pose}"')
            return 0
        say('PREVIEW', f'{len(stages)} stage(s), all collision-free. Wall time = nominal '
                       f'x 100 / pendant slider.')
        if not args.execute:
            say('PLAN', 'preview only; nothing was sent. Rerun with --execute to move.')
            return 0

        for n, joint in enumerate(stages, 1):
            # Re-plan from where the arm really is, and re-check before sending.
            state = ex.fresh()
            at = ex.joints_of(state)
            if abs(goal[joint] - at[joint]) < SKIP_RAD:
                continue
            points = stage_points(at, joint, goal[joint])
            check_stage(ex, state, points, joint)
            turn = math.degrees(goal[joint] - at[joint])
            if not args.yes:
                answer = input(f'Stage {n}/{len(stages)}: turn {joint.replace("_joint", "")} '
                               f'{turn:+.0f} deg. Hand on the E-stop. Run? [y/N] ')
                if answer.strip().lower() not in ('y', 'yes'):
                    say('STOPPED', f'operator declined stage {n}; the arm is where it is')
                    return 1
            report = ex.execute(state, points, ex.joint_rate)
            report['stage'] = n
            report['joint'] = joint.replace('_joint', '')
            say('EXEC', report)

        tcp, joints = ex.where()
        say('WHERE', {'tcp_xyz_m': [round(v, 4) for v in tcp],
                      'joints': {j: round(v, 3) for j, v in joints.items()}})
        say('DONE', f'at "{args.pose}". In the console: /teach home  (so HOME matches it)')
        return 0
    except JogError as err:
        say('BLOCKED', str(err))
        return 1
    finally:
        ex.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
