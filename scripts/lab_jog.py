#!/usr/bin/env python3
"""Execute validated jog commands (move / rotate / go_to) on the arm-only UR7e.

This is the motion half of the command console (task 3.2). It takes an
``arm_language.result.Command`` whose action is one of ``schema.JOG_ACTIONS``
-- already validated and bounded -- and turns it into a collision-checked
trajectory on the real arm. It never reads free text; that is the parser's job.

How a command becomes motion
----------------------------
* **move**  -- FK of ``tool0`` from a fresh joint state, shift the pose by the
  requested vector in ``base_link``, ask MoveIt for a Cartesian path
  (``/compute_cartesian_path``, collision-checked against the lab table and
  the RG2 envelope from ``lab_arm_moveit.launch.py``), require 100 % of the
  path, then execute.
* **rotate** -- a joint-space plan (``/plan_kinematic_path``) that changes
  only ``wrist_3_joint`` by the bounded angle; the speed level picks the
  nominal joint rate.
* **go_to** -- a joint-space plan to a pose taught in this session (``home`` is
  taught automatically at startup), refused if it is further than
  ``max_excursion`` from where the arm is; then the optional offset as a move.

Every plan is re-timed here at a nominal joint rate we choose (MoveIt's own
timing is discarded), then sent through ``MotionClient.run()`` from
``scripts/motion/ur_motion.py`` -- the same proven path as the demo ladder,
with its safety envelope (velocity ceiling, per-waypoint step cap, explicit
t=0 anchor). The pendant speed slider scales everything down further.

Gates checked before every execution: safety mode NORMAL, External Control
program running, live speed-scaling at or under the operator-approved ceiling,
fresh joint state, arm stationary. Any failure raises ``JogError`` and nothing
is sent. With ``plan_only=True`` everything short of sending runs, which is
how the console is rehearsed before Play is pressed.
"""
from copy import deepcopy
import json
import math
import os
import sys
import time

from builtin_interfaces.msg import Duration  # noqa: F401  (kept for clarity of retiming)
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan
from moveit_msgs.srv import GetPositionFK, GetStateValidity
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float64
from ur_dashboard_msgs.msg import SafetyMode

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motion'))
from ur_motion import JOINTS, MAX_JOINT_VEL, MotionClient  # noqa: E402

from arm_language import schema  # noqa: E402

GROUP = 'ur_manipulator'
TOOL_LINK = 'tool0'
BASE_FRAME = 'base_link'

# Direction word -> unit vector in base_link. "Left" is the robot's +Y; the
# console's --mirror-lr flips left/right for an audience facing the robot.
DIRECTION_VECTORS = {
    'up': (0.0, 0.0, 1.0), 'down': (0.0, 0.0, -1.0),
    'left': (0.0, 1.0, 0.0), 'right': (0.0, -1.0, 0.0),
    'forward': (1.0, 0.0, 0.0), 'backward': (-1.0, 0.0, 0.0),
}

# Nominal joint rate per |speed_level| (rad/s, BEFORE the pendant slider).
# All under ur_motion.MAX_JOINT_VEL so the envelope check can never be the
# thing that fails at demo time; the slider is the operator's extra brake.
SPEED_LEVEL_RATE = {1: 0.03, 2: 0.06, 3: 0.09}

# Wrist_3 hard range is +/-2pi; keep a margin so a rotate never plans into
# the limit and stops short with an error the audience cannot read.
WRIST_3_LIMIT = 2.0 * math.pi - 0.1

# Re-timing: dense MoveIt paths get at least this much time per waypoint so
# the controller never sees two points closer than its 2 ms resolution.
MIN_WAYPOINT_DT = 0.05


class JogError(RuntimeError):
    """A command could not be planned or the execution gate is closed."""


class JogExecutor(Node):
    """Plans with MoveIt, executes through ``MotionClient``."""

    def __init__(self, max_speed_percent=50.0, joint_rate=0.05, max_excursion=0.6,
                 mirror_lr=False, plan_only=False, poses_file=None):
        super().__init__('lab_jog')
        self.max_speed_percent = float(max_speed_percent)
        self.joint_rate = min(float(joint_rate), MAX_JOINT_VEL)
        self.max_excursion = float(max_excursion)
        self.mirror_lr = mirror_lr
        self.plan_only = plan_only
        self.poses_file = poses_file
        self.poses = {}
        self.plan_attempts = 3
        self.last_plan_attempts = 0

        self.latest = None
        self.received = 0.0
        self.speed = None
        self.speed_time = 0.0
        self.running = False
        self.safety = None
        self.create_subscription(JointState, '/joint_states', self._joints, 10)
        self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                                 self._scaling, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                 lambda msg: setattr(self, 'running', msg.data), latched)
        self.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode',
                                 lambda msg: setattr(self, 'safety', msg.mode), latched)

        self.motion = MotionClient()
        self._table_added = False

    # --- telemetry -----------------------------------------------------------

    def _joints(self, msg):
        self.latest = msg
        self.received = time.monotonic()

    def _scaling(self, msg):
        self.speed = msg.data
        self.speed_time = time.monotonic()

    def fresh(self, settle_s=1.0):
        """Spin briefly, then return a fresh, stationary RobotState."""
        until = time.monotonic() + settle_s
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest is None or time.monotonic() - self.received > 0.5:
            raise JogError('No fresh joint state -- is the driver running?')
        if self.latest.velocity and max(abs(v) for v in self.latest.velocity) > 0.0005:
            raise JogError('The arm is still moving; wait for it to settle.')
        return RobotState(joint_state=deepcopy(self.latest))

    @staticmethod
    def joints_of(state):
        """name -> position dict. NEVER index by position (see ur_motion)."""
        return dict(zip(state.joint_state.name, state.joint_state.position))

    def gate(self):
        """Raise unless the robot is in a state we are willing to move."""
        problems = []
        if self.safety != SafetyMode.NORMAL:
            problems.append(f'safety mode is {self.safety}, not NORMAL')
        if not self.running:
            problems.append('External Control program is not running (press Play)')
        if self.speed is None or time.monotonic() - self.speed_time > 0.5:
            problems.append('no live speed-scaling telemetry')
        elif not 0.0 < self.speed <= self.max_speed_percent + 0.1:
            problems.append(f'pendant speed {self.speed:.1f}% is outside the approved '
                            f'0-{self.max_speed_percent:g}% window')
        if problems:
            raise JogError('execution gate closed: ' + '; '.join(problems))

    # --- MoveIt services -------------------------------------------------------

    def call(self, kind, endpoint, request, timeout=15.0):
        client = self.create_client(kind, endpoint)
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise JogError(f'{endpoint} unavailable -- is lab_arm_moveit.launch.py running?')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise JogError(f'{endpoint} timed out')
            return future.result()
        finally:
            self.destroy_client(client)

    def ensure_table(self):
        """Add the conservative lab-table box once per session (idempotent)."""
        if self._table_added:
            return
        obj = CollisionObject(id='lab_table')
        obj.header.frame_id = BASE_FRAME
        obj.operation = CollisionObject.ADD
        pose = Pose()
        # Top face just under the base mounting plane (same model as
        # lab_arm_check.py): anything below the base plate is "table".
        pose.position.z = -0.102
        pose.orientation.w = 1.0
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4.0, 4.0, 0.2])]
        obj.primitive_poses = [pose]
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.world.collision_objects = [obj]
        if not self.call(ApplyPlanningScene, '/apply_planning_scene', req).success:
            raise JogError('could not add the lab table to the planning scene')
        self._table_added = True

    def valid(self, state):
        response = self.call(GetStateValidity, '/check_state_validity',
                             GetStateValidity.Request(robot_state=state, group_name=GROUP))
        return response.valid

    def fk(self, state):
        req = GetPositionFK.Request(robot_state=state, fk_link_names=[TOOL_LINK])
        req.header.frame_id = BASE_FRAME
        response = self.call(GetPositionFK, '/compute_fk', req)
        if response.error_code.val != 1:
            raise JogError(f'FK failed with MoveIt error {response.error_code.val}')
        return response.pose_stamped[0].pose

    def where(self):
        """Return the tool position (m) and the joint dict, for the console."""
        state = self.fresh(settle_s=0.3)
        pose = self.fk(state)
        return (pose.position.x, pose.position.y, pose.position.z), self.joints_of(state)

    # --- planners: each returns a list of joint dicts, start pose excluded -----

    def plan_move(self, state, direction, distance_cm):
        self.ensure_table()
        if self.mirror_lr and direction in ('left', 'right'):
            direction = 'right' if direction == 'left' else 'left'
        vector = DIRECTION_VECTORS[direction]
        start = self.fk(state)
        target = deepcopy(start)
        d = distance_cm / 100.0
        target.position.x += vector[0] * d
        target.position.y += vector[1] * d
        target.position.z += vector[2] * d
        req = GetCartesianPath.Request(
            start_state=state, group_name=GROUP, link_name=TOOL_LINK,
            # 1 mm steps, measured on the real stack 2026-09-30: with pick_ik
            # in local mode the same 2 cm lift reaches 100% at 1 mm, 9% at
            # 2 mm and 0% at 5 mm -- each step is an IK solve seeded by the
            # previous one, and the local solver only converges for tiny
            # moves. Bigger steps are not faster, they are unreachable.
            waypoints=[target], max_step=0.001,
            # Absolute joint-jump limit instead of the relative-to-average
            # heuristic, which is unstable on short paths (see lab_arm_check).
            jump_threshold=0.0, revolute_jump_threshold=0.02,
            avoid_collisions=True,
            max_velocity_scaling_factor=0.01, max_acceleration_scaling_factor=0.01)
        req.header.frame_id = BASE_FRAME
        # The per-step IK solve is time-boxed, so a path can stop short on a
        # busy machine and succeed a moment later. A few retries cost
        # milliseconds when the first one works and save the demo when it
        # does not; the *reachability* verdict below is only final after all.
        best = None
        for attempt in range(1, self.plan_attempts + 1):
            response = self.call(GetCartesianPath, '/compute_cartesian_path', req)
            if response.error_code.val == 1 and response.fraction >= 0.999:
                self.last_plan_attempts = attempt
                return self._points(response.solution.joint_trajectory), (start, target)
            if best is None or response.fraction > best.fraction:
                best = response
        raise JogError(f'only {best.fraction * 100:.0f}% of a {distance_cm:g} cm '
                       f'{direction} move is reachable without collision '
                       f'(MoveIt code {best.error_code.val}, {self.plan_attempts} tries)')

    def plan_joint_goal(self, state, goal, tolerance=0.0005, path_slack=None):
        """Plan to ``goal`` (joint dict) in joint space, collision-checked."""
        self.ensure_table()
        req = GetMotionPlan.Request()
        plan = req.motion_plan_request
        plan.group_name = GROUP
        plan.start_state = state
        plan.allowed_planning_time = 5.0
        plan.num_planning_attempts = 3
        plan.max_velocity_scaling_factor = 0.01
        plan.max_acceleration_scaling_factor = 0.01
        target = Constraints()
        current = self.joints_of(state)
        for name in JOINTS:
            target.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(goal[name]),
                tolerance_above=tolerance, tolerance_below=tolerance, weight=1.0))
            if path_slack is not None:
                # Corridor around the straight joint-space line: for a wrist
                # spin, the other five joints may not wander.
                slack = path_slack if name != 'wrist_3_joint' else 2 * math.pi
                plan.path_constraints.joint_constraints.append(JointConstraint(
                    joint_name=name, position=float(current[name]),
                    tolerance_above=slack, tolerance_below=slack, weight=1.0))
        plan.goal_constraints = [target]
        response = self.call(GetMotionPlan, '/plan_kinematic_path', req,
                             timeout=20.0).motion_plan_response
        if response.error_code.val != 1:
            raise JogError(f'joint-space plan failed (MoveIt code {response.error_code.val})')
        return self._points(response.trajectory.joint_trajectory)

    def plan_rotate(self, state, speed_level, angle_deg):
        current = self.joints_of(state)
        goal = dict(current)
        delta = math.radians(angle_deg) * (1 if speed_level > 0 else -1)
        goal['wrist_3_joint'] = current['wrist_3_joint'] + delta
        if abs(goal['wrist_3_joint']) > WRIST_3_LIMIT:
            raise JogError(f'wrist_3 would reach {goal["wrist_3_joint"]:.2f} rad, past its '
                           f'limit; spin the other way first')
        return self.plan_joint_goal(state, goal, path_slack=0.02)

    def plan_go_to(self, state, pose_name):
        if pose_name not in self.poses:
            known = ', '.join(sorted(self.poses)) or 'none'
            raise JogError(f'no pose called "{pose_name}" has been taught this session '
                           f'(known: {known}). Use /teach {pose_name} at the console.')
        goal = self.poses[pose_name]
        current = self.joints_of(state)
        excursion = max(abs(goal[j] - current[j]) for j in JOINTS)
        if excursion > self.max_excursion:
            raise JogError(f'"{pose_name}" is {excursion:.2f} rad away on some joint; the '
                           f'per-command limit is {self.max_excursion:.2f} rad '
                           f'(--max-excursion). Move closer first.')
        if excursion < 1e-3:
            return []
        return self.plan_joint_goal(state, goal)

    @staticmethod
    def _points(joint_trajectory):
        names = joint_trajectory.joint_names
        points = []
        for point in joint_trajectory.points:
            if any(not math.isfinite(v) for v in point.positions):
                raise JogError('planner returned a non-finite joint value')
            points.append(dict(zip(names, point.positions)))
        if not points:
            raise JogError('planner returned an empty trajectory')
        return points

    # --- execution -------------------------------------------------------------

    def retime(self, start, points, rate):
        """Turn joint dicts into (pose, t) pairs at a nominal joint rate.

        The first planner point is the start state itself; ``MotionClient``
        sends the anchor at t=0, so that duplicate is dropped. Each following
        waypoint gets the time its largest joint step needs at ``rate``.
        """
        rate = min(rate, MAX_JOINT_VEL * 0.95)
        pts = []
        previous = start
        t = 0.0
        for pose in points:
            step = max(abs(pose[j] - previous[j]) for j in JOINTS)
            if step < 1e-6 and not pts:
                continue  # the planner's copy of the start state
            t += max(MIN_WAYPOINT_DT, step / rate)
            pts.append((pose, t))
            previous = pose
        return pts

    def audit(self, start, points):
        """Bounds the console reports before asking to execute."""
        excursion = 0.0
        jump = 0.0
        previous = start
        for pose in points:
            excursion = max(excursion, max(abs(pose[j] - start[j]) for j in JOINTS))
            jump = max(jump, max(abs(pose[j] - previous[j]) for j in JOINTS))
            previous = pose
        if excursion > max(self.max_excursion, 1.0) + 1e-6:
            raise JogError(f'plan swings a joint {excursion:.2f} rad, over the limit')
        return {'max_joint_excursion_rad': round(excursion, 4),
                'max_waypoint_step_rad': round(jump, 4), 'waypoints': len(points)}

    def execute(self, state, points, rate):
        """Send a re-timed trajectory, or just report it in plan_only mode."""
        start = self.joints_of(state)
        report = self.audit(start, points)
        report['plan_attempts'] = self.last_plan_attempts
        self.last_plan_attempts = 0
        pts = self.retime(start, points, rate)
        if not pts:
            report['nominal_seconds'] = 0.0
            report['executed'] = False
            return report
        report['nominal_seconds'] = round(pts[-1][1], 1)
        report['nominal_joint_rate'] = rate
        if self.plan_only:
            report['executed'] = False
            return report
        self.gate()
        # Re-read right before sending: the anchor must be where the arm IS.
        anchor = self.joints_of(self.fresh(settle_s=0.3))
        if max(abs(anchor[j] - start[j]) for j in JOINTS) > 0.002:
            raise JogError('the arm moved between planning and execution; re-plan')
        report['speed_percent'] = self.speed
        ok = self.motion.run(pts, anchor=anchor)
        report['executed'] = bool(ok)
        if not ok:
            raise JogError('trajectory did not succeed (see the driver log / pendant)')
        return report

    # --- the one entry point the console uses -----------------------------------

    def run_command(self, command, plan_only=None):
        """Plan and execute one validated jog Command. Returns a report dict.

        ``plan_only=True`` previews: every stage is planned and audited from
        the current pose, nothing is sent. The console shows that preview
        before it asks for confirmation, so the human confirms a plan with
        numbers in it, not just a sentence.
        """
        if command.action not in schema.JOG_ACTIONS:
            raise JogError(f'"{command.action}" is not a jog action')
        saved = self.plan_only
        if plan_only is not None:
            self.plan_only = plan_only
        try:
            return self._run_command(command)
        finally:
            self.plan_only = saved

    def _run_command(self, command):
        state = self.fresh()
        before = self.fk(state).position
        stages = []

        if command.action == schema.ACTION_MOVE:
            points, _ = self.plan_move(state, command.direction, command.distance_cm)
            stages.append(('move', points, self.joint_rate))

        elif command.action == schema.ACTION_ROTATE:
            points = self.plan_rotate(state, command.speed_level, command.angle_deg)
            stages.append(('rotate', points, SPEED_LEVEL_RATE[abs(command.speed_level)]))

        else:  # go_to, optionally followed by an offset move
            points = self.plan_go_to(state, command.pose_name)
            stages.append(('go_to', points, self.joint_rate))
            if command.direction:
                if self.plan_only:
                    # Plan the offset from where the pose WILL be.
                    predicted = deepcopy(state)
                    end = points[-1] if points else self.joints_of(state)
                    predicted.joint_state.name = list(JOINTS)
                    predicted.joint_state.position = [end[j] for j in JOINTS]
                    predicted.joint_state.velocity = [0.0] * len(JOINTS)
                    offset, _ = self.plan_move(predicted, command.direction,
                                               command.distance_cm)
                    stages.append(('offset', offset, self.joint_rate))
                else:
                    stages.append(('offset', None, self.joint_rate))

        reports = []
        for name, points, rate in stages:
            if points is None:
                # Second stage of a go_to with offset: plan from the real pose
                # the first stage ended at, not a prediction.
                state = self.fresh()
                points, _ = self.plan_move(state, command.direction, command.distance_cm)
            report = self.execute(state, points, rate)
            report['stage'] = name
            reports.append(report)
            if not self.plan_only and points:
                state = self.fresh()

        after = self.fk(self.fresh(settle_s=0.3)).position if not self.plan_only else before
        return {
            'stages': reports,
            'measured_delta_mm': [round((after.x - before.x) * 1000, 1),
                                  round((after.y - before.y) * 1000, 1),
                                  round((after.z - before.z) * 1000, 1)],
        }

    # --- taught poses ------------------------------------------------------------

    def teach(self, name):
        joints = self.joints_of(self.fresh(settle_s=0.3))
        self.poses[name] = {j: float(joints[j]) for j in JOINTS}
        self.save_poses()
        return self.poses[name]

    def save_poses(self):
        if self.poses_file:
            with open(self.poses_file, 'w') as handle:
                json.dump(self.poses, handle, indent=2, sort_keys=True)

    def load_poses(self, path=None):
        """Load poses from ``path`` (default: the session file); later wins."""
        path = path or self.poses_file
        if path and os.path.exists(path):
            with open(path) as handle:
                stored = json.load(handle)
            for name, joints in stored.items():
                if isinstance(joints, dict) and set(joints) == set(JOINTS):
                    self.poses[name] = {j: float(joints[j]) for j in JOINTS}
        return self.poses
