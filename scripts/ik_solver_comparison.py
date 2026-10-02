#!/usr/bin/env python3
"""Measure how reliably a MoveIt IK plugin plans the pick pipeline's Cartesian moves.

Issue #38. On the real UR7e (2026-09-30) ``/compute_cartesian_path`` with pick_ik planned
the same 5 cm move to 100 % or 4 % depending on encoder noise in the start sample, while
KDL planned it every time. This script turns that anecdote into numbers: it asks the
RUNNING move_group for the same Cartesian request many times, each time from a start state
that is the nominal pose plus a little uniform joint noise, and records the achieved
fraction and the planning time of every single call.

What it does NOT do: it never moves anything. ``/compute_cartesian_path`` takes an explicit
``start_state``, so the (mock) arm can stay wherever it is. The request has the same shape
as ``JogExecutor.plan_move`` in ``scripts/lab_jog.py`` -- FK of ``tool0`` from the start
state, shift that pose in ``base_link``, one waypoint, absolute revolute jump limit,
collision checking on -- because that is the request that ran on the real arm.

The solver is whatever the running move_group loaded; the script reads it back from
``/move_group``'s parameters and stores it with the results, so a label can never disagree
with what was measured. One invocation measures one move_group; run it once per solver and
the results accumulate in the same JSON file (a run with the same label replaces the old).

Recipe (inside a container, NEVER in the lab's ROS domain 42; the script refuses that)::

    export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
    source install/setup.bash
    ros2 launch ur7e_bringup ur7e_bringup.launch.py use_mock_hardware:=true \
        launch_rviz:=false &                      # calibrated description, no robot
    ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl &
    python3 scripts/ik_solver_comparison.py --output results.json     # KDL first, see below
    # stop that move_group, check with `ps` that it is gone, then:
    ros2 launch scripts/lab_arm_moveit.launch.py ik:=pick_ik &
    python3 scripts/ik_solver_comparison.py --output results.json
    python3 scripts/ik_solver_comparison.py --report results.json     # markdown table

Run KDL first: the two lift requests start from the joint state at the bottom of the
corresponding descent, which is taken from one full planned descent and then stored in the
JSON so that every later run (any solver) starts its lifts from exactly the same joints.

Two move_group processes in one domain answer the same service and give contradictory
results; the script counts the ``move_group`` nodes it can see and refuses unless it is one.

Two things to know when reading the numbers (both measured 2026-10-02, MoveIt 2.5.10):

* ``revolute_jump_threshold`` is sent because the console sends it, but this move_group
  ignores the field (0.02 and 0.00001 give identical paths; its log prints only the
  relative ``jump_threshold``, which is 0 = off). Nothing in the request rejects a joint
  jump, so the script records how far the busiest joint travels in every returned path.
* The IK solve is boxed in wall-clock time (``kinematics_solver_timeout``). A stalled
  machine therefore changes the result; planning times here are from an idle 2-CPU
  container and are not what a busy lab laptop will show.
"""
import argparse
from copy import deepcopy
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import random
import statistics
import sys
import time

GROUP = 'ur_manipulator'      # same three names as scripts/lab_jog.py
TOOL_LINK = 'tool0'
BASE_FRAME = 'base_link'
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Each request: (name, start state, tool displacement in base_link in metres).
# `ready` comes from scripts/lab_poses.json; the `lowered_*` states are the ends of the
# 5 cm and 15 cm descents (see derive_lowered_states).
REQUESTS = [
    ('descend_5cm', 'ready', (0.0, 0.0, -0.05)),
    ('descend_15cm', 'ready', (0.0, 0.0, -0.15)),
    ('descend_30cm', 'ready', (0.0, 0.0, -0.30)),
    ('lift_5cm', 'lowered_5cm', (0.0, 0.0, 0.05)),
    ('lift_15cm', 'lowered_15cm', (0.0, 0.0, 0.15)),
    ('x_plus_5cm', 'ready', (0.05, 0.0, 0.0)),
    ('x_minus_5cm', 'ready', (-0.05, 0.0, 0.0)),
    ('x_plus_20cm', 'ready', (0.20, 0.0, 0.0)),
    ('x_minus_20cm', 'ready', (-0.20, 0.0, 0.0)),
    ('y_plus_5cm', 'ready', (0.0, 0.05, 0.0)),
    ('y_minus_5cm', 'ready', (0.0, -0.05, 0.0)),
    ('y_plus_20cm', 'ready', (0.0, 0.20, 0.0)),
    ('y_minus_20cm', 'ready', (0.0, -0.20, 0.0)),
    # Away from the base in X and Y and down: towards the table, as a reach-and-lower
    # would be. (+X from `ready` heads for the base axis instead, see x_plus_20cm.)
    ('diagonal_-x+y-z_20cm', 'ready', (-0.20, 0.20, -0.20)),
]
LOWERED = {'lowered_5cm': 0.05, 'lowered_15cm': 0.15}

# Kinematics parameters worth keeping next to the numbers (unset ones are skipped).
KINEMATICS_PARAMETERS = [
    'kinematics_solver', 'kinematics_solver_timeout', 'kinematics_solver_attempts',
    'kinematics_solver_search_resolution', 'mode', 'position_scale', 'rotation_scale',
    'position_threshold', 'orientation_threshold', 'cost_threshold',
    'minimal_displacement_weight', 'gd_step_size',
]
SUCCESS_FRACTION = 0.999      # the console's own "100 %" test (lab_jog.plan_move)


# --- talking to move_group ---------------------------------------------------------------

class Probe:
    """Thin wrapper around the move_group services this measurement needs."""

    def __init__(self):
        # ROS imports live here so that --report works on a machine without ROS.
        from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
        from moveit_msgs.srv import GetPositionFK
        from rcl_interfaces.srv import GetParameters
        import rclpy
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('ik_solver_comparison')
        self.cartesian = self.node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.fk_client = self.node.create_client(GetPositionFK, '/compute_fk')
        self.scene = self.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.scene_reader = self.node.create_client(GetPlanningScene, '/get_planning_scene')
        self.parameters = self.node.create_client(GetParameters, '/move_group/get_parameters')

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()

    def call(self, client, request, timeout=300.0):
        if not client.wait_for_service(timeout_sec=15.0):
            sys.exit(f'{client.srv_name} unavailable -- is lab_arm_moveit.launch.py running?')
        future = client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            sys.exit(f'{client.srv_name} did not answer within {timeout:g} s')
        return future.result()

    def count_move_groups(self, listen_s=3.0):
        """Discovery takes a moment; spin, then count nodes called exactly move_group."""
        until = time.monotonic() + listen_s
        while time.monotonic() < until:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return sum(1 for name, _ in self.node.get_node_names_and_namespaces()
                   if name == 'move_group')

    def get_parameters(self, names):
        """Return {name: value} for the move_group parameters that are set."""
        from rcl_interfaces.srv import GetParameters
        response = self.call(self.parameters, GetParameters.Request(names=names))
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
        return {name: getattr(value, fields[value.type])
                for name, value in zip(names, response.values) if value.type in fields}

    def robot_state(self, joints):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        return RobotState(joint_state=JointState(
            name=list(JOINTS), position=[float(joints[name]) for name in JOINTS],
            velocity=[0.0] * len(JOINTS)))

    def fk(self, state):
        """Pose of tool0 in base_link, exactly as lab_jog.JogExecutor.fk asks for it."""
        from moveit_msgs.srv import GetPositionFK
        request = GetPositionFK.Request(robot_state=state, fk_link_names=[TOOL_LINK])
        request.header.frame_id = BASE_FRAME
        response = self.call(self.fk_client, request)
        if response.error_code.val != 1:
            sys.exit(f'FK failed with MoveIt error {response.error_code.val}')
        return response.pose_stamped[0].pose

    def set_lab_table(self, present):
        """Put the console's table box in the planning scene, or make sure it is absent.

        Same object as lab_jog.ensure_table. The scene outlives this script, so the state
        is set explicitly both ways instead of assuming a fresh move_group.
        """
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        from shape_msgs.msg import SolidPrimitive
        table = CollisionObject(id='lab_table')
        table.header.frame_id = BASE_FRAME
        if not present:
            # Removing an object that is not there is reported as a failure, so look first.
            wanted = PlanningSceneComponents(
                components=PlanningSceneComponents.WORLD_OBJECT_NAMES)
            world = self.call(self.scene_reader,
                              GetPlanningScene.Request(components=wanted)).scene.world
            if all(known.id != table.id for known in world.collision_objects):
                return
        if present:
            table.operation = CollisionObject.ADD
            pose = Pose()
            pose.position.z = -0.102
            pose.orientation.w = 1.0
            table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                               dimensions=[4.0, 4.0, 0.2])]
            table.primitive_poses = [pose]
        else:
            table.operation = CollisionObject.REMOVE
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [table]
        if not self.call(self.scene, request).success:
            sys.exit('could not update the planning scene')

    def cartesian_path(self, joints, delta, max_step, revolute_jump_threshold):
        """One /compute_cartesian_path call; returns (response, wall seconds).

        Mirrors lab_jog.JogExecutor.plan_move: the target is the FK pose of the start
        state shifted by ``delta`` metres in base_link, sent as a single waypoint.
        """
        from moveit_msgs.srv import GetCartesianPath
        state = self.robot_state(joints)
        target = deepcopy(self.fk(state))
        target.position.x += delta[0]
        target.position.y += delta[1]
        target.position.z += delta[2]
        request = GetCartesianPath.Request(
            start_state=state, group_name=GROUP, link_name=TOOL_LINK,
            waypoints=[target], max_step=max_step,
            jump_threshold=0.0, revolute_jump_threshold=revolute_jump_threshold,
            avoid_collisions=True,
            max_velocity_scaling_factor=0.01, max_acceleration_scaling_factor=0.01)
        request.header.frame_id = BASE_FRAME
        started = time.perf_counter()
        response = self.call(self.cartesian, request)
        seconds = time.perf_counter() - started
        return response, seconds, self.path_accuracy(response, request.waypoints[0], delta)

    def path_accuracy(self, response, target, delta, samples=12):
        """How well the returned joint path follows the requested straight tool line.

        ``fraction`` only says every IK step was ACCEPTED, and a solver accepts within its
        own tolerance (pick_ik: position_threshold and orientation_threshold). So the
        returned path is run through FK at ``samples`` evenly spaced points and compared
        with the line: sideways distance from it, tool rotation away from the start
        orientation, and how far the last point is from the target.
        """
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        trajectory = response.solution.joint_trajectory
        count = len(trajectory.points)
        if count < 2:
            return {}
        length = math.sqrt(sum(value * value for value in delta))
        along = [value / length for value in delta]
        start = [target.position.x - delta[0], target.position.y - delta[1],
                 target.position.z - delta[2]]
        wanted = target.orientation      # the request keeps the start orientation
        sideways = rotation = 0.0
        for index in sorted({round(k * (count - 1) / (samples - 1)) for k in range(samples)}):
            pose = self.fk(RobotState(joint_state=JointState(
                name=list(trajectory.joint_names),
                position=list(trajectory.points[index].positions))))
            offset = [pose.position.x - start[0], pose.position.y - start[1],
                      pose.position.z - start[2]]
            progress = sum(a * b for a, b in zip(offset, along))
            sideways = max(sideways, math.sqrt(max(
                0.0, sum(value * value for value in offset) - progress * progress)))
            dot = abs(pose.orientation.x * wanted.x + pose.orientation.y * wanted.y
                      + pose.orientation.z * wanted.z + pose.orientation.w * wanted.w)
            rotation = max(rotation, 2.0 * math.acos(min(1.0, dot)))
        # `pose` is now the FK of the last trajectory point.
        end = math.sqrt((pose.position.x - target.position.x) ** 2
                        + (pose.position.y - target.position.y) ** 2
                        + (pose.position.z - target.position.z) ** 2)
        return {'line_deviation_m': round(sideways, 6), 'tool_rotation_rad': round(rotation, 6),
                'end_error_m': round(end, 6)}


# --- the measurement ---------------------------------------------------------------------

def noisy(joints, noise):
    return {name: joints[name] + offset for name, offset in zip(JOINTS, noise)}


def derive_lowered_states(probe, ready):
    """Joint states at the bottom of the 5 cm and 15 cm descents from nominal `ready`.

    Taken from the last point of a fully planned 1 mm-step descent with the solver that is
    running now. Stored in the JSON and reused, so all solvers lift from the same joints.
    """
    lowered = {}
    for name, depth in LOWERED.items():
        for _ in range(3):      # the console also allows itself three tries
            response = probe.cartesian_path(ready, (0.0, 0.0, -depth), 0.001, 0.02)[0]
            if response.error_code.val == 1 and response.fraction >= SUCCESS_FRACTION:
                break
        else:
            sys.exit(f'could not plan the {depth * 100:g} cm descent that defines {name} '
                     f'(best fraction {response.fraction:.3f}); run with ik:=kdl first')
        trajectory = response.solution.joint_trajectory
        last = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        lowered[name] = {joint: float(last[joint]) for joint in JOINTS}
    return lowered


def measure(probe, start_states, request, max_step, args):
    """Run ``args.trials`` noisy-start calls of one request; return the per-trial records."""
    name, start_name, delta = request
    nominal = start_states[start_name]
    # Seeded per request, NOT per solver or step size: every solver and every max_step
    # sees the same sequence of start states, so their results are directly comparable.
    rng = random.Random(f'{args.seed}/{name}')
    trials = []
    for index in range(args.trials):
        noise = [rng.uniform(-args.noise, args.noise) for _ in JOINTS]
        response, seconds, accuracy = probe.cartesian_path(
            noisy(nominal, noise), delta, max_step, args.revolute_jump_threshold)
        points = [point.positions for point in response.solution.joint_trajectory.points]
        # How far the busiest joint travels along the returned path (sum of |steps|, so
        # going there and back counts twice). A straight 5 cm move costs ~0.1-0.2 rad; a
        # wrist flip or an extra turn costs radians and shows up here, not in `fraction`.
        # (The service re-times and resamples the path, so the size of one step in the
        # response says nothing about the IK steps; the accumulated travel does.)
        travel = [sum(abs(q[joint] - p[joint]) for p, q in zip(points, points[1:]))
                  for joint in range(len(points[0]))] if points else []
        trials.append({
            'trial': index,
            'noise_rad': [round(value, 7) for value in noise],
            'fraction': response.fraction,
            'error_code': response.error_code.val,
            'seconds': round(seconds, 5),
            'points': len(points),
            'max_joint_travel_rad': round(max(travel), 5) if travel else None,
            **accuracy,     # line_deviation_m, tool_rotation_rad, end_error_m
        })
    return trials


def run(args):
    if os.environ.get('ROS_DOMAIN_ID') == '42':
        sys.exit('refusing to run in the lab domain (42): that is the real robot')
    ready = json.loads(Path(args.poses).read_text())['ready']
    output = Path(args.output)
    data = json.loads(output.read_text()) if output.exists() else {
        'schema': 1, 'start_states': {'ready': ready}, 'runs': []}
    if data['start_states']['ready'] != ready:
        sys.exit(f'{output} was recorded from a different `ready` pose; use a new file')

    probe = Probe()
    try:
        seen = probe.count_move_groups()
        if seen != 1:
            sys.exit(f'{seen} move_group nodes visible; exactly one must be running')
        prefix = f'robot_description_kinematics.{GROUP}.'
        kinematics = {name[len(prefix):]: value for name, value in probe.get_parameters(
            [prefix + name for name in KINEMATICS_PARAMETERS]).items()}
        solver = kinematics.get('kinematics_solver', 'unknown')
        label = args.label or ('kdl' if 'KDL' in solver else
                               'pick_ik' if 'PickIk' in solver else solver)
        description = probe.get_parameters(['robot_description'])['robot_description']

        probe.set_lab_table(args.lab_table)
        missing = [name for name in LOWERED if name not in data['start_states']]
        if missing:
            data['start_states'].update(derive_lowered_states(probe, ready))
            data['start_states']['_lowered_derived_with'] = label
        tool = probe.fk(probe.robot_state(ready)).position

        wanted = [request for request in REQUESTS
                  if not args.requests or request[0] in args.requests]
        record = {
            'label': label,
            'recorded_utc': datetime.datetime.utcnow().isoformat(timespec='seconds') + 'Z',
            'note': args.note,
            'seed': args.seed,
            'noise_rad': args.noise,
            'trials_per_cell': args.trials,
            'request_settings': {
                'group_name': GROUP, 'link_name': TOOL_LINK, 'frame_id': BASE_FRAME,
                'jump_threshold': 0.0,
                'revolute_jump_threshold': args.revolute_jump_threshold,
                'avoid_collisions': True, 'lab_table_in_scene': args.lab_table,
            },
            'kinematics_parameters': kinematics,
            'robot_description_sha256': hashlib.sha256(description.encode()).hexdigest(),
            'ready_tool0_position_m': [round(tool.x, 5), round(tool.y, 5), round(tool.z, 5)],
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'results': [],
        }
        for max_step in args.max_steps:
            for request in wanted:
                trials = measure(probe, data['start_states'], request, max_step, args)
                record['results'].append({
                    'request': request[0], 'start_state': request[1],
                    'delta_m': list(request[2]), 'max_step': max_step, 'trials': trials})
                good = sum(trial['fraction'] >= SUCCESS_FRACTION for trial in trials)
                print(f'{label:>20} {request[0]:<22} max_step={max_step:<7g} '
                      f'{good:>3}/{len(trials)}', flush=True)
    finally:
        probe.close()

    data['runs'] = [old for old in data['runs'] if old['label'] != label] + [record]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, indent=1) + '\n')
    print(f'wrote {output}')


# --- the table ---------------------------------------------------------------------------

def summarize(trials):
    fractions = [trial['fraction'] for trial in trials]
    good = sum(fraction >= SUCCESS_FRACTION for fraction in fractions)
    # Path quality only over complete paths: those are the ones a pipeline would execute.
    complete = [trial for trial in trials if trial['fraction'] >= SUCCESS_FRACTION]

    def worst(key):
        values = [trial[key] for trial in complete if trial.get(key) is not None]
        return max(values) if values else None

    return {
        'n': len(trials), 'successes': good,
        'median_fraction': statistics.median(fractions),
        'min_fraction': min(fractions),
        'median_seconds': statistics.median(trial['seconds'] for trial in trials),
        'max_travel_rad': worst('max_joint_travel_rad'),
        'max_deviation_m': worst('line_deviation_m'),
        'max_end_error_m': worst('end_error_m'),
        'max_rotation_rad': worst('tool_rotation_rad'),
    }


def report(path):
    """Print one markdown table: a row per (run, request), a column group per max_step."""
    data = json.loads(Path(path).read_text())
    for record in data['runs']:
        steps = sorted({result['max_step'] for result in record['results']})
        settings = record['request_settings']
        print(f"\n### {record['label']}  (revolute_jump_threshold="
              f"{settings['revolute_jump_threshold']:g}, lab table "
              f"{'in' if settings['lab_table_in_scene'] else 'not in'} scene, "
              f"N={record['trials_per_cell']}, seed {record['seed']})\n")
        print('Cell: successes/N at fraction >= 0.999 · median fraction · minimum '
              'fraction · median planning time.\n')
        print('| request | ' + ' | '.join(f'max_step {step * 1000:g} mm' for step in steps)
              + ' |')
        print('|---|' + '---|' * len(steps))
        names = []
        for result in record['results']:
            if result['request'] not in names:
                names.append(result['request'])
        cells = {(result['request'], result['max_step']): summarize(result['trials'])
                 for result in record['results']}
        for name in names:
            row = []
            for step in steps:
                cell = cells.get((name, step))
                row.append('' if cell is None else (
                    f"{cell['successes']}/{cell['n']} · {cell['median_fraction']:.3f} · "
                    f"{cell['min_fraction']:.3f} · {cell['median_seconds'] * 1000:.0f} ms"))
            print(f'| {name} | ' + ' | '.join(row) + ' |')
        print('\nWorst case over the complete paths (- = none, or not recorded): joint travel '
              'rad · distance from the straight line mm · end-point error mm · tool '
              'rotation mrad.\n')
        print('| request | ' + ' | '.join(f'max_step {step * 1000:g} mm' for step in steps)
              + ' |')
        print('|---|' + '---|' * len(steps))

        def shown(value, scale, digits):
            return '-' if value is None else f'{value * scale:.{digits}f}'

        for name in names:
            row = []
            for step in steps:
                cell = cells.get((name, step), {})
                row.append(' · '.join([shown(cell.get('max_travel_rad'), 1, 2),
                                       shown(cell.get('max_deviation_m'), 1000, 2),
                                       shown(cell.get('max_end_error_m'), 1000, 2),
                                       shown(cell.get('max_rotation_rad'), 1000, 1)]))
            print(f'| {name} | ' + ' | '.join(row) + ' |')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        epilog='See the module docstring for the full recipe.')
    parser.add_argument('--output', default='docs/evidence/ik-solver-comparison.json',
                        help='JSON file to create or add this run to')
    parser.add_argument('--report', metavar='JSON',
                        help='print the markdown summary of an existing file and exit')
    parser.add_argument('--label', help='name of this run in the JSON (default: derived '
                        'from the solver plugin move_group reports: kdl or pick_ik)')
    parser.add_argument('--note', default='', help='free text stored with the run')
    parser.add_argument('--trials', type=int, default=30, help='calls per cell (default 30)')
    parser.add_argument('--seed', type=int, default=20261002, help='RNG seed for the noise')
    parser.add_argument('--noise', type=float, default=0.0005,
                        help='uniform start-state noise per joint, +/- rad (default 0.0005, '
                        'what the real encoders show at rest)')
    # 1 mm is what the console sends; 2.5 mm is pymoveit2's default, i.e. what ur7e_motion's
    # primitives send today; 5 and 10 mm are what a coarser pipeline would like to use.
    parser.add_argument('--max-steps', type=float, nargs='+',
                        default=[0.001, 0.0025, 0.005, 0.01],
                        help='max_step values in metres (default 0.001 0.0025 0.005 0.01)')
    parser.add_argument('--revolute-jump-threshold', type=float, default=0.02,
                        help='value sent in the request field of that name (default 0.02, '
                        'as the console sends)')
    parser.add_argument('--requests', nargs='+', choices=[r[0] for r in REQUESTS],
                        help='measure only these requests (default: all)')
    parser.add_argument('--lab-table', action='store_true',
                        help="add the console's table collision box to the planning scene")
    parser.add_argument('--poses', help='pose file holding `ready` (default: lab_poses.json '
                        'next to this script)',
                        default=str(Path(__file__).resolve().parent / 'lab_poses.json'))
    args = parser.parse_args()
    if args.report:
        report(args.report)
    else:
        if args.trials < 1 or args.noise < 0 or min(args.max_steps) <= 0:
            parser.error('trials must be >= 1, noise >= 0 and every max_step > 0')
        run(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
