#!/usr/bin/env python3
"""Language-directed pick on the real UR7e + RG2, seen through one webcam.

This file is the *lab adapter* for ``ur7e_orchestrator``: the orchestrator
owns the fixed stage sequence, timeouts, retries and logging; everything that
touches hardware is here, built from parts already proven on the arm:

* motion -- ``lab_jog.JogExecutor`` (MoveIt Cartesian paths with KDL, re-timed
  and sent through ``MotionClient`` with its safety envelope and gates);
* gripper -- the OnRobot URCap's own XML-RPC server (``lab_rg2_native``);
* eyes -- ``/perception/detect_object`` + ``/perception/locate_object`` from
  ``ur7e_perception.webcam_node`` (table-board anchored, see that package);
* language -- ``arm_language`` (backend -> validator -> guardrails).

Where the numbers come from
---------------------------
All heights are relative to the table plane that the four calibration touches
defined, and the gripper length used at touch time is stored in the
calibration file and reused here, so its exact value cancels out: "fingertips
8 mm above the table" means 8 mm above where they stood when they touched it.
The touches are made with the jaws closed, which is the RG2's longest state
(the fingers swing up as they open), so every clearance below is a minimum.

Prerequisites (each in its own terminal, in the lab container)::

    ros2 launch ur7e_bringup ur7e_bringup.launch.py           # then press Play
    ros2 launch scripts/lab_arm_moveit.launch.py ik:=kdl
    ros2 run ur7e_perception webcam_perception_node --ros-args \
        -p calibration:=$PWD/scripts/lab_table.json \
        -p objects_file:=$PWD/scripts/lab_objects.yaml \
        -p snapshot_dir:=$PWD/log/perception

Then::

    python3 scripts/lab_pick.py --say "pick up the red block"          # plan only
    python3 scripts/lab_pick.py --say "pick up the red block" --execute
    python3 scripts/lab_pick.py --verify-marker 42 --execute           # task 2.5 check
    python3 scripts/lab_pick.py --gripper-test --execute               # open/close once
"""
import argparse
from copy import deepcopy
import json
import math
import os
import socket
import sys
import time
import xmlrpc.client

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
import numpy as np
import rclpy
from shape_msgs.msg import SolidPrimitive
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
for package in ('ur7e_orchestrator', 'ur7e_perception'):
    sys.path.insert(0, os.path.join(_HERE, '..', 'src', package))

from lab_jog import BASE_FRAME, GROUP, JogError, JogExecutor, TOOL_LINK  # noqa: E402
from lab_rg2_native import identify, TimeoutTransport, validate_state  # noqa: E402
from ur_motion import JOINTS  # noqa: E402  (lab_jog put scripts/motion on the path)

from arm_language import schema  # noqa: E402
from arm_language.backends import available, BackendError, create  # noqa: E402
from arm_language.guardrails import GuardrailPolicy  # noqa: E402
from arm_language.parser import IntentParser  # noqa: E402
from arm_language.result import Outcome  # noqa: E402
from ur7e_interfaces.srv import DetectObject, LocateObject  # noqa: E402
from ur7e_orchestrator.adapters import (  # noqa: E402
    Adapters, Detection, GraspResult, Intent, NotFound, Refused, SafetyAbort, StageError,
    Target)
from ur7e_orchestrator.workflow import Orchestrator  # noqa: E402
from ur7e_perception.monocular import TableCalibration  # noqa: E402

DEFAULT_CALIBRATION = os.path.join(_HERE, 'lab_table.json')
OBSTACLES_FILE = os.path.join(_HERE, 'lab_obstacles.yaml')
APPROX_GRIPPER_LENGTH = 0.258  # tool0 flange to closed RG2 fingertips, m (measured 2026-10-02)
DEFAULT_LOG_DIR = os.path.join(_HERE, '..', 'log', 'runs')
SEED_POSES_FILE = os.path.join(_HERE, 'lab_poses.json')
SESSION_POSES_FILE = os.path.join(_HERE, '.console_poses.json')


def top_down_quaternion(yaw_x):
    """Quaternion (x, y, z, w) for tool Z straight down, tool X at ``yaw_x``.

    The rotation matrix has columns X=(c, s, 0), Y=(s, -c, 0), Z=(0, 0, -1):
    a half-turn about the axis halfway between base X and the tool X.
    """
    half = yaw_x / 2.0
    return (math.cos(half), math.sin(half), 0.0, 0.0)


def tool_yaw(pose):
    """Angle of the tool X axis in the base XY plane, and the tilt from vertical."""
    q = pose.orientation
    x_axis = (1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.z * q.w))
    z_down = -(1 - 2 * (q.x * q.x + q.y * q.y))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, z_down))))
    return math.atan2(x_axis[1], x_axis[0]), tilt


def nearest_equivalent(angle, reference, period=math.pi):
    """Return ``angle`` shifted by whole periods to be closest to ``reference``.

    A two-finger gripper is the same turned by 180 degrees, so of the two
    ways to line it up with an object we take the one needing less wrist.
    """
    return angle + round((reference - angle) / period) * period


class Rg2:
    """The RG2 through the OnRobot URCap's XML-RPC server.

    The URCap keeps ownership of the tool connector; we only call the same
    functions its pendant nodes call. Widths in mm, force in N.
    """

    def __init__(self, host, tool=2, serial='1000042561', max_force=25.0):
        self.host, self.tool, self.max_force = host, tool, max_force
        self.rpc = xmlrpc.client.ServerProxy(f'http://{host}:41414/',
                                             transport=TimeoutTransport())
        self.device = identify(self.rpc, tool, serial)

    def state(self):
        """Return the validated native state (width, busy, status bits)."""
        return validate_state(self.rpc.rg_get_all_variables(self.tool))

    def move(self, width_mm, force_n, timeout=8.0):
        """Command a width and wait until the jaws stop. Return the final state.

        Stopping short of the target is not an error here: that is exactly
        what gripping an object looks like. The caller judges the width.
        """
        if not 0 <= width_mm <= 110 or not 0 < force_n <= self.max_force:
            raise StageError('gripper_bounds', f'width {width_mm} mm / force {force_n} N '
                                               'outside the lab bounds')
        # "busy" also stays on while the jaws keep squeezing an object that
        # settles in the grip (the lab hat), so a busy gripper is given a
        # moment and then simply commanded: a new rg_grip supersedes the old.
        settle = time.monotonic() + 1.5
        while self.state()['busy'] and time.monotonic() < settle:
            time.sleep(0.1)
        result = self.rpc.rg_grip(self.tool, float(width_mm), float(force_n))
        if result != 0:
            raise StageError('gripper_rejected', f'rg_grip returned {result}')
        deadline = time.monotonic() + timeout
        started = time.monotonic()
        last, stable, seen_moving = None, 0, False
        time.sleep(0.15)
        while time.monotonic() < deadline:
            state = self.state()
            seen_moving |= bool(state['busy'])
            settled = last is not None and abs(state['width'] - last) < 0.3
            # "Settled" only counts once the jaws have been seen moving (or
            # were already at the target): the first polls after rg_grip can
            # catch the gripper before it has started, which looked like a
            # finished move at 9.7 mm instead of 75 mm (2026-10-02).
            ready = seen_moving or abs(state['width'] - width_mm) <= 1.0 \
                or time.monotonic() - started > 1.5
            stable = stable + 1 if settled and not state['busy'] and ready else 0
            if stable >= 3:
                state['grip_detected'] = bool(state['status'] & 2)
                return state
            last = state['width']
            time.sleep(0.1)
        try:
            self.rpc.rg_stop(self.tool)
        finally:
            raise StageError('gripper_timeout', f'jaws still moving after {timeout:g} s')


class FakeRg2:
    """Stand-in for plan-only runs: never talks to the robot."""

    def __init__(self, width=80.0):
        self.width = width

    def state(self):
        """Return a plausible idle state."""
        return {'width': self.width, 'busy': False, 'status': 0}

    def move(self, width_mm, force_n, timeout=8.0):
        """Pretend to move; a 'close' stops at 35 mm as if it held a block."""
        self.width = 35.0 if width_mm < 20 else float(width_mm)
        return {'width': self.width, 'busy': False, 'status': 2, 'grip_detected': True}


class PickExecutor(JogExecutor):
    """``JogExecutor`` plus absolute, top-down Cartesian moves over a known table.

    Without a calibration only the table-independent moves work (absolute
    poses, taught poses); that is what the camera calibration itself uses.
    """

    def __init__(self, calibration=None, hover=0.12, table_surface_z=None, **kwargs):
        super().__init__(**kwargs)
        self.calibration = calibration
        self.hover = hover
        self.anchor = self.tool_length = None
        # Without a calibration the planner's table is the base plane unless
        # told otherwise (the lab's pick plate is 11 cm below it).
        self.table_surface_z = table_surface_z
        if calibration is None:
            return
        self.anchor = np.asarray(calibration.base_from_board, dtype=float)
        # tool0-to-reference-point length used when the table was measured;
        # reusing the same number here is what makes it cancel out.
        lengths = {touch['tool_length'] for touch in
                   calibration.notes.get('touches', {}).values()}
        if 'tool_length' in calibration.notes:
            lengths = {calibration.notes['tool_length']}
        if len(lengths) != 1:
            raise JogError('calibration file has no single tool length; re-run the '
                           'calibration (lab_table_calibration.py / lab_camera_calibration.py)')
        self.tool_length = lengths.pop()

    # --- table geometry -----------------------------------------------------------

    def table_z(self, x, y):
        """Height of the table plane under (x, y), base_link metres."""
        origin, normal = self.anchor[:3, 3], self.anchor[:3, 2]
        return origin[2] - (normal[0] * (x - origin[0]) + normal[1] * (y - origin[1])) / normal[2]

    def tool_z(self, x, y, tip_above_table):
        """tool0 height that puts the closed fingertips this far above the table."""
        if tip_above_table < 0.004:
            raise JogError(f'refusing a fingertip height of {tip_above_table * 1000:.0f} mm: '
                           'the floor is 4 mm above the table')
        return self.table_z(x, y) + self.tool_length + tip_above_table

    def ensure_table(self):
        """Put the collision table at the *measured* surface, not the base plane."""
        if self.anchor is None and self.table_surface_z is None:
            return super().ensure_table()
        if self._table_added:
            return
        obj = CollisionObject(id='lab_table')
        obj.header.frame_id = BASE_FRAME
        obj.operation = CollisionObject.ADD
        pose = Pose()
        # anchor z + tool_length is tool0's height when the fingertips rest on
        # the table; the real surface is one gripper length below that. The
        # length only needs to be roughly right here (it places a collision
        # slab, 5 mm low on purpose, 20 cm thick); grasp heights never use it.
        if self.anchor is None:
            surface = float(self.table_surface_z)
        else:
            contact = float(self.anchor[2, 3] + self.tool_length)
            surface = contact - APPROX_GRIPPER_LENGTH
        pose.position.z = surface - 0.005 - 0.1
        pose.orientation.w = 1.0
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4.0, 4.0, 0.2])]
        obj.primitive_poses = [pose]
        objects = [obj] + self.lab_obstacles(surface)
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.world.collision_objects = objects
        if not self.call(ApplyPlanningScene, '/apply_planning_scene', req).success:
            raise JogError('could not add the lab table to the planning scene')
        self._table_added = True

    def lab_obstacles(self, surface):
        """Collision boxes from ``lab_obstacles.yaml`` plus a keep-out around the camera."""
        boxes = {}
        if os.path.exists(OBSTACLES_FILE):
            with open(OBSTACLES_FILE) as stream:
                boxes.update((yaml.safe_load(stream) or {}).get('boxes') or {})
        if self.calibration is not None and self.calibration.camera_to_base is not None:
            # The camera stands inside the arm's reach; its true height is
            # its calibration-frame height minus the table's.
            cam = np.array(self.calibration.camera_to_base)[:3, 3]
            top = surface + (cam[2] - float(self.anchor[2, 3])) + 0.12
            boxes['camera'] = [cam[0] - 0.12, cam[0] + 0.12, cam[1] - 0.12, cam[1] + 0.12,
                               surface - 0.3, top]
        objects = []
        for name, (x0, x1, y0, y1, z0, z1) in boxes.items():
            obj = CollisionObject(id=f'lab_{name}')
            obj.header.frame_id = BASE_FRAME
            obj.operation = CollisionObject.ADD
            pose = Pose()
            pose.position.x, pose.position.y = (x0 + x1) / 2, (y0 + y1) / 2
            pose.position.z = (z0 + z1) / 2
            pose.orientation.w = 1.0
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                             dimensions=[x1 - x0, y1 - y0, z1 - z0])]
            obj.primitive_poses = [pose]
            objects.append(obj)
        return objects

    # --- planning -----------------------------------------------------------------

    def plan_pose(self, state, pose, avoid_collisions=True):
        """Cartesian straight line from ``state`` to an absolute tool0 pose."""
        self.ensure_table()
        req = GetCartesianPath.Request(
            start_state=state, group_name=GROUP, link_name=TOOL_LINK,
            waypoints=[pose], max_step=0.001, jump_threshold=0.0,
            revolute_jump_threshold=0.02, avoid_collisions=avoid_collisions,
            max_velocity_scaling_factor=0.01, max_acceleration_scaling_factor=0.01)
        req.header.frame_id = BASE_FRAME
        best = None
        for attempt in range(1, self.plan_attempts + 1):
            response = self.call(GetCartesianPath, '/compute_cartesian_path', req)
            if response.error_code.val == 1 and response.fraction >= 0.999:
                self.last_plan_attempts = attempt
                return self._points(response.solution.joint_trajectory)
            if best is None or response.fraction > best.fraction:
                best = response
        p = pose.position
        raise JogError(f'only {best.fraction * 100:.0f}% of the straight line to '
                       f'({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) is reachable'
                       f'{" without collision" if avoid_collisions else ""} '
                       f'(MoveIt code {best.error_code.val})')

    def plan_joint_line(self, state, goal, step=0.03):
        """Straight line in joint space, every step checked for collisions.

        Deterministic, unlike a sampling planner: the same two poses always
        give the same motion, which is what a repeatability benchmark needs.
        Falls back to the collision-aware planner if the line is blocked.
        """
        self.ensure_table()
        start = self.joints_of(state)
        span = max(abs(goal[j] - start[j]) for j in JOINTS)
        if span < 1e-3:
            return []
        count = max(2, int(math.ceil(span / step)))
        points = [{j: start[j] + (goal[j] - start[j]) * i / count for j in JOINTS}
                  for i in range(1, count + 1)]
        for point in points[::2] + [points[-1]]:
            probe = deepcopy(state)
            probe.joint_state.name = list(JOINTS)
            probe.joint_state.position = [point[j] for j in JOINTS]
            probe.joint_state.velocity = [0.0] * len(JOINTS)
            probe.joint_state.effort = []
            if not self.valid(probe):
                return self.plan_joint_goal(state, goal)
        return points

    def predicted(self, state, points):
        """Return the RobotState the arm will be in after ``points``."""
        if not points:
            return state
        after = deepcopy(state)
        after.joint_state.name = list(JOINTS)
        after.joint_state.position = [points[-1][j] for j in JOINTS]
        after.joint_state.velocity = [0.0] * len(JOINTS)
        after.joint_state.effort = []
        return after

    def pose_at(self, x, y, tip_above_table, yaw_x):
        """Top-down tool0 pose with the fingertips over (x, y)."""
        pose = Pose()
        pose.position.x, pose.position.y = float(x), float(y)
        pose.position.z = float(self.tool_z(x, y, tip_above_table))
        q = pose.orientation
        q.x, q.y, q.z, q.w = top_down_quaternion(yaw_x)
        return pose

    # --- execution ------------------------------------------------------------------

    def move_pose(self, label, pose, avoid_collisions=True, rate=None):
        """Plan from where the arm is to ``pose`` and execute. Return the report."""
        state = self.fresh()
        points = self.plan_pose(state, pose, avoid_collisions)
        report = self.execute(state, points, rate or self.joint_rate)
        report['stage'] = label
        return report

    def move_named(self, name, rate=None):
        """Joint-space move to a taught pose. Return the report."""
        if name not in self.poses:
            raise JogError(f'no pose called "{name}" (teach it at the console: /teach {name})')
        state = self.fresh()
        points = self.plan_joint_line(state, self.poses[name])
        if not points:
            return {'stage': name, 'executed': False, 'waypoints': 0, 'nominal_seconds': 0.0}
        report = self.execute(state, points, rate or self.joint_rate)
        report['stage'] = name
        return report


class LabAdapters(Adapters):
    """The orchestrator's hands, eyes and ears in the lab."""

    def __init__(self, executor, gripper, parser, robot_host, config, confirm=None,
                 notify=None):
        self.executor, self.gripper, self.parser = executor, gripper, parser
        self.robot_host, self.config = robot_host, config
        self.confirm, self.notify = confirm, notify or (lambda tag, payload: None)
        self.parsed = {}
        self.intent = self.target = self.place = None
        self.grasp_yaw = self.tip_height = self.release_height = None
        self.held_width = None

    # --- language ---------------------------------------------------------------------

    def remember(self, utterance, result, confirmed=False):
        """Reuse a parse the console already did (one LLM call per sentence).

        ``confirmed`` means the console already showed the echo and got a
        yes, so PARSE must not ask a second time.
        """
        self.parsed[utterance] = (result, confirmed)

    def parse(self, utterance):
        """PARSE: text -> validated Intent, or Refused."""
        result, confirmed = self.parsed.pop(utterance, (None, False))
        if result is None:
            result = self.parser.parse(utterance)
        if result.outcome is Outcome.REFUSED:
            raise Refused(result.reason_code.value, result.message)
        command = result.command
        if command.action not in schema.MOTION_ACTIONS:
            raise Refused('not_a_pick', f'"{command.echo()}" is a jog, not a pick')
        if result.outcome is Outcome.NEEDS_CONFIRMATION and not confirmed:
            if self.confirm is None or not self.confirm(result.message):
                raise Refused('not_confirmed', 'operator did not confirm')
        extra = [command.modifiers[key] for key in schema.MODIFIER_KEYS
                 if command.modifiers.get(key) and command.modifiers[key]
                 not in command.target_query]
        query = ' '.join(extra + [command.target_query])
        place = command.place_target if command.action == schema.ACTION_PICK_AND_PLACE else ''
        self.intent = Intent(command.action, query, place or '', command.echo())
        return self.intent

    # --- safety -------------------------------------------------------------------------

    def safety_ok(self):
        """Gate checked before every stage that moves hardware."""
        if self.executor.plan_only:
            return True, ''
        try:
            self.executor.gate()
            return True, ''
        except JogError as error:
            # The orchestrator uses this string as the run's stable reason
            # code; the human-readable detail goes to the console here.
            self.notify('GATE', str(error))
            text = str(error)
            if 'safety mode' in text:
                return False, 'safety_mode_not_normal'
            if 'not running' in text:
                return False, 'program_not_running'
            if 'pendant speed' in text:
                return False, 'speed_above_approved'
            return False, 'no_robot_telemetry'

    def abort(self):
        """Stop the robot program from the dashboard: works from any thread."""
        if self.executor.plan_only:
            return
        try:
            with socket.create_connection((self.robot_host, 29999), timeout=2) as sock:
                sock.makefile('rb').readline()
                sock.sendall(b'stop\n')
        except OSError as error:
            self.notify('ABORT', f'dashboard stop failed: {error}; use the pendant')

    def _move(self, action):
        """Run one motion; translate executor errors into stage errors."""
        try:
            report = action()
        except JogError as error:
            text = str(error)
            if 'execution gate closed' in text or 'did not succeed' in text:
                raise SafetyAbort('motion_stopped', text)
            raise StageError('motion_blocked', text)
        self.notify('MOVE', report)
        return report

    # --- perception ---------------------------------------------------------------------

    def observe(self):
        """OBSERVE: move the arm out of the camera's view of the table."""
        # Same route as home(): never start a joint-space move from table height.
        self.home() if self.config['observe_pose'] == self.config['home_pose'] else \
            self._move(lambda: self.executor.move_named(self.config['observe_pose']))

    def detect(self, query):
        """DETECT: ask the perception node for the object's mask."""
        response = self.executor.call(DetectObject, '/perception/detect_object',
                                      DetectObject.Request(query=query), timeout=90.0)
        if not response.success:
            reason = response.reason
            missing = ('not detected', 'object on the table', 'missing or seen',
                       'multiple plausible', 'more than one')
            if any(text in reason for text in missing):
                raise NotFound('not_found', f'"{query}": {reason}')
            raise StageError('perception_error', reason)
        self.notify('DETECT', {'query': query, 'confidence': round(response.confidence, 3),
                               'latency_ms': round(response.latency_ms)})
        return Detection(response.capture_id, float(response.confidence))

    def locate(self, detection):
        """LOCATE: mask -> grasp point on the table, base_link."""
        response = self.executor.call(LocateObject, '/perception/locate_object',
                                      LocateObject.Request(capture_id=detection.capture_id))
        if not response.success:
            raise StageError('locate_failed', response.reason)
        p, q = response.pose.pose.position, response.pose.pose.orientation
        x0, y0, x1, y1 = self.config['workspace']
        if not (x0 <= p.x <= x1 and y0 <= p.y <= y1):
            raise NotFound('outside_workspace',
                           f'object at ({p.x:.2f}, {p.y:.2f}) is outside the pick area '
                           f'x {x0}..{x1}, y {y0}..{y1}; move it toward the middle')
        yaw = 2 * math.atan2(q.y, q.x) if abs(q.x) + abs(q.y) > 1e-9 else 0.0
        height = p.z - self.executor.table_z(p.x, p.y)
        detail = {'height_m': round(height, 4), 'axis_ratio': round(response.axis_ratio, 2),
                  'points': int(response.valid_points)}
        return Target((p.x, p.y, p.z), yaw, detail)

    # --- planning -----------------------------------------------------------------------

    def plan(self, intent, target, place):
        """PLAN: choose heights and wrist angle; prove every segment plans."""
        cfg, ex = self.config, self.executor
        self.target, self.place = target, place
        state = ex.fresh()
        current_yaw, tilt = tool_yaw(ex.fk(state))
        if tilt > 3.0:
            raise StageError('tool_not_vertical',
                             f'tool is tilted {tilt:.1f} deg; start from a top-down pose')
        self.grasp_yaw = current_yaw
        if target.detail['axis_ratio'] >= cfg['align_ratio']:
            # Jaws close along tool X, so X goes ACROSS the object's long axis.
            aligned = nearest_equivalent(target.yaw + math.pi / 2, current_yaw)
            # A half-turn-symmetric gripper never needs more than 90 deg; cap
            # lower still so wrist_3 cannot be wound toward its +-2pi stop by
            # one skewed silhouette (a hat at the plate edge did exactly that,
            # 2026-10-02, and the lift then stalled at the joint limit).
            turn = max(-cfg['max_align_turn'], min(cfg['max_align_turn'], aligned - current_yaw))
            self.grasp_yaw = current_yaw + turn
        height = max(0.0, target.detail['height_m'])
        self.tip_height = max(cfg['min_tip_clearance'], height - cfg['finger_reach'])
        x, y = target.xyz[0], target.xyz[1]
        if place is None:
            px, py = cfg['drop_xy'] or (x, y)
            place_surface = 0.0
        else:
            px, py = place.xyz[0], place.xyz[1]
            place_surface = max(0.0, place.detail['height_m'])
        self.release_xy = (px, py)
        self.release_height = place_surface + self.tip_height + cfg['release_clearance']
        hover = ex.hover + max(height, place_surface)
        self.hover_height = hover

        # Dry-run every segment from predicted states before anything moves.
        summary = {'grasp_yaw_deg': round(math.degrees(self.grasp_yaw), 1),
                   'tip_above_table_mm': round(self.tip_height * 1000, 1),
                   'hover_mm': round(hover * 1000), 'pick_xy': [round(x, 3), round(y, 3)],
                   'release_xy': [round(px, 3), round(py, 3)],
                   'release_tip_mm': round(self.release_height * 1000, 1)}
        try:
            segments = [
                ('approach', ex.pose_at(x, y, hover, self.grasp_yaw), True),
                ('descend', ex.pose_at(x, y, self.tip_height, self.grasp_yaw), False),
                ('lift', ex.pose_at(x, y, hover, self.grasp_yaw), False),
                ('carry', ex.pose_at(px, py, hover, self.grasp_yaw), True),
                ('lower', ex.pose_at(px, py, self.release_height, self.grasp_yaw), False),
                ('back_up', ex.pose_at(px, py, hover, self.grasp_yaw), False),
            ]
            waypoints = 0
            for name, pose, avoid in segments:
                points = ex.plan_pose(state, pose, avoid)
                waypoints += len(points)
                state = ex.predicted(state, points)
            ex.plan_joint_line(state, ex.poses[cfg['home_pose']])
        except JogError as error:
            raise StageError('unreachable', str(error))
        summary['waypoints'] = waypoints
        return summary

    # --- motion -------------------------------------------------------------------------

    def approach(self):
        """APPROACH: open the jaws, hover above the object, descend."""
        cfg, ex = self.config, self.executor
        x, y = self.target.xyz[0], self.target.xyz[1]
        self._grip(cfg['open_mm'], cfg['force_n'])
        self._move(lambda: ex.move_pose(
            'approach', ex.pose_at(x, y, self.hover_height, self.grasp_yaw)))
        # The last centimetres go below the conservative gripper envelope, so
        # collision checking is off for this vertical segment only; the
        # fingertip floor in tool_z() is the guard instead.
        self._move(lambda: ex.move_pose(
            'descend', ex.pose_at(x, y, self.tip_height, self.grasp_yaw),
            avoid_collisions=False))

    def _grip(self, width, force):
        if self.executor.plan_only and not isinstance(self.gripper, FakeRg2):
            return {'width': width, 'grip_detected': False}
        state = self.gripper.move(width, force)
        self.notify('GRIP', {'target_mm': width, 'width_mm': round(state['width'], 1),
                             'grip_detected': state.get('grip_detected')})
        return state

    def grasp(self):
        """GRASP: close until the jaws stall; a stall near zero is a miss."""
        cfg = self.config
        state = self._grip(cfg['close_mm'], cfg['force_n'])
        holding = state['width'] > cfg['miss_width_mm']
        self.held_width = state['width'] if holding else None
        return GraspResult(holding, float(state['width']),
                           {'grip_detected': state.get('grip_detected')})

    def lift(self):
        """LIFT: straight up to hover; then check the object came with us."""
        ex = self.executor
        x, y = self.target.xyz[0], self.target.xyz[1]
        self._move(lambda: ex.move_pose(
            'lift', ex.pose_at(x, y, self.hover_height, self.grasp_yaw),
            avoid_collisions=False))
        if self.held_width is not None and not ex.plan_only:
            # A lost object lets the jaws close all the way; a width that
            # merely shrank means the object settled or slid in the grip (the
            # lab's toy hat slid from its dome to its brim, 74 -> 51 mm, and
            # was still firmly held, 2026-10-02).
            width = self.gripper.state()['width']
            if width < self.config['miss_width_mm'] + 2.0:
                raise StageError('object_dropped',
                                 f'jaw width went {self.held_width:.1f} -> {width:.1f} mm '
                                 'during the lift: the jaws are closed on nothing')
            self.held_width = width

    def retreat(self):
        """RETREAT: carry to the release point, lower, let go, back up."""
        cfg, ex = self.config, self.executor
        px, py = self.release_xy
        self._move(lambda: ex.move_pose(
            'carry', ex.pose_at(px, py, self.hover_height, self.grasp_yaw)))
        self._move(lambda: ex.move_pose(
            'lower', ex.pose_at(px, py, self.release_height, self.grasp_yaw),
            avoid_collisions=False))
        self._grip(cfg['open_mm'], cfg['force_n'])
        self.held_width = None
        self._move(lambda: ex.move_pose(
            'back_up', ex.pose_at(px, py, self.hover_height, self.grasp_yaw),
            avoid_collisions=False))

    def release(self):
        """Open the jaws where the arm is (after a missed grasp)."""
        self._grip(self.config['open_mm'], self.config['force_n'])
        self.held_width = None

    def home(self):
        """HOME: back to the pose every run starts from.

        After a failed approach the fingertips may still be low over the
        table, where a joint-space move would sweep the gripper sideways
        through whatever is there. So: straight up first, then home.
        """
        ex = self.executor
        pose = ex.fk(ex.fresh())
        x, y = pose.position.x, pose.position.y
        clear = ex.tool_z(x, y, ex.hover)
        if pose.position.z < clear - 0.01:
            target = deepcopy(pose)
            target.position.z = clear
            self._move(lambda: ex.move_pose('clear', target, avoid_collisions=False))
        self._move(lambda: ex.move_named(self.config['home_pose']))


DEFAULT_CONFIG = {
    'observe_pose': 'observe', 'home_pose': 'observe',
    'open_mm': 80.0, 'close_mm': 0.0, 'force_n': 20.0, 'miss_width_mm': 6.0,
    'min_tip_clearance': 0.008, 'finger_reach': 0.028, 'release_clearance': 0.004,
    # Same threshold as monocular.localize_on_table: below it perception
    # reports yaw 0 ("no meaningful long axis") and the wrist stays put.
    'align_ratio': 2.0, 'max_align_turn': math.radians(60), 'drop_xy': None,
    # Targets outside this base_link rectangle (m) are refused at LOCATE:
    # the lab's raised front piece minus a margin from its edges, where a
    # round object can be half over the drop and a grasp pulls it off.
    'workspace': (0.14, -0.07, 0.30, 0.22),
}


def say(tag, payload):
    """One line per event, machine-readable (same format as the console)."""
    text = payload if isinstance(payload, str) else json.dumps(payload, default=str)
    print(f'{tag:8} {text}', flush=True)


def ask(prompt, auto_yes):
    """Typed confirmation; ``--yes`` answers for scripted runs."""
    if auto_yes:
        say('CONFIRM', 'auto-yes')
        return True
    try:
        return input(f'{prompt} [y/N] ').strip().lower() in ('y', 'yes')
    except EOFError:
        return False


def on_event(event):
    """Print orchestrator events as they happen."""
    kind = event.get('event', '')
    if kind == 'stage_end':
        fields = {'outcome': event['outcome'], 'seconds': event['duration_s']}
        if event['attempt'] > 1:
            fields['attempt'] = event['attempt']
        if event['outcome'] != 'ok':
            fields['reason'] = event['reason']
        if event.get('detail'):
            fields['detail'] = event['detail']
        say(event['stage'].upper(), fields)
    elif kind == 'run_start':
        say('RUN', {'run_id': event['run_id'], 'utterance': event['utterance']})


def build(args, executor=None):
    """Construct executor, gripper, parser, adapters and orchestrator."""
    calibration = TableCalibration.load(args.calibration)
    if executor is None:
        executor = PickExecutor(
            calibration, hover=args.hover, max_speed_percent=args.max_speed_percent,
            joint_rate=args.joint_rate, max_excursion=args.max_excursion,
            plan_only=not args.execute, poses_file=args.poses_file)
        executor.load_poses(SEED_POSES_FILE)
        executor.load_poses()
    config = dict(DEFAULT_CONFIG)
    if 'observe' not in executor.poses:
        # No dedicated pose taught: observe/home from `ready` (top-down hub).
        config['observe_pose'] = config['home_pose'] = 'ready'
    if args.drop_xy:
        config['drop_xy'] = tuple(args.drop_xy)
    if args.workspace:
        config['workspace'] = tuple(args.workspace)
    config.update(force_n=args.force, open_mm=args.open_mm,
                  min_tip_clearance=args.tip_clearance)
    fake = args.fake_gripper or not args.execute
    gripper = FakeRg2() if fake else Rg2(args.robot_ip)
    backend = create(args.backend, **({'model': args.model} if args.model else {}))
    parser = IntentParser(backend, GuardrailPolicy(
        allowed_actions=schema.MOTION_ACTIONS, require_confirmation=not args.no_confirm))
    adapters = LabAdapters(executor, gripper, parser, args.robot_ip, config,
                           confirm=lambda message: ask(message, args.yes), notify=say)
    orchestrator = Orchestrator(adapters, log_dir=args.log_dir, on_event=on_event)
    return executor, adapters, orchestrator


def verify_marker(args, executor, adapters):
    """Task 2.5 gate: point the fingertips at a marker the camera located.

    The arm hovers its closed fingertips 10 mm above where perception says
    the marker centre is. You measure how far off it is; five within 1 cm is
    the acceptance. This checks the whole chain at once -- camera model,
    board anchor, robot kinematics -- which no single-part test can.
    """
    query = f'marker {args.verify_marker}'
    adapters.observe()
    detection = adapters.detect(query)
    target = adapters.locate(detection)
    x, y = target.xyz[0], target.xyz[1]
    yaw, _ = tool_yaw(executor.fk(executor.fresh()))
    say('TARGET', {'query': query, 'xy': [round(x, 4), round(y, 4)],
                   'table_z': round(executor.table_z(x, y), 4)})
    if not executor.plan_only:
        adapters.gripper.move(0.0, 10.0)
    adapters._move(lambda: executor.move_pose('hover', executor.pose_at(x, y, 0.10, yaw)))
    adapters._move(lambda: executor.move_pose(
        'point', executor.pose_at(x, y, 0.010, yaw), avoid_collisions=False))
    record = {'marker': args.verify_marker, 'commanded_xy': [x, y],
              'stamp': time.strftime('%Y-%m-%dT%H:%M:%S'), 'executed': not executor.plan_only}
    if not executor.plan_only and not args.yes:
        answer = input('Offset of the fingertip centre from the marker centre, mm '
                       '(e.g. 4, or "x y"): ').strip()
        record['measured_offset_mm'] = answer
    adapters._move(lambda: executor.move_pose(
        'back_up', executor.pose_at(x, y, 0.10, yaw), avoid_collisions=False))
    adapters.observe()
    os.makedirs(os.path.dirname(args.evidence), exist_ok=True)
    with open(args.evidence, 'a') as stream:
        stream.write(json.dumps(record) + '\n')
    say('VERIFY', record)
    return 0


def gripper_test(adapters):
    """Open and close the empty jaws once with the arm program running."""
    for width in (60.0, 80.0, 60.0):
        state = adapters.gripper.move(width, 10.0)
        say('GRIP', {'target_mm': width, 'width_mm': round(state['width'], 1)})
    return 0


def add_session_arguments(cli):
    """Add the arguments every lab pick tool shares (also used by lab_benchmark)."""
    cli.add_argument('--execute', action='store_true', help='really move (default: plan only)')
    cli.add_argument('--yes', action='store_true', help='auto-confirm (scripted runs)')
    cli.add_argument('--no-confirm', action='store_true',
                     help='do not ask before confident commands')
    cli.add_argument('--backend', default='keyword', choices=available())
    cli.add_argument('--model', default='')
    cli.add_argument('--robot-ip', default='192.168.56.101')
    cli.add_argument('--calibration', default=DEFAULT_CALIBRATION)
    cli.add_argument('--max-speed-percent', type=float, default=50.0)
    cli.add_argument('--joint-rate', type=float, default=0.09)
    cli.add_argument('--max-excursion', type=float, default=2.5)
    cli.add_argument('--hover', type=float, default=0.12,
                     help='fingertip height above the tallest object while carrying, m')
    cli.add_argument('--force', type=float, default=20.0, help='grip force, N')
    cli.add_argument('--open-mm', type=float, default=80.0)
    cli.add_argument('--tip-clearance', type=float, default=0.008,
                     help='lowest fingertip height above the table when grasping, m. '
                          'Raise it for wide, rounded objects: the RG2 fingers slope '
                          'inward above the pads, so a wide object is only clear of them '
                          'near the pads (the lab hat stopped the arm at 8 mm, 2026-10-02)')
    cli.add_argument('--drop-xy', type=float, nargs=2, metavar=('X', 'Y'),
                     help='where a plain "pick" puts the object down (default: back in place)')
    cli.add_argument('--workspace', type=float, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'),
                     help='base_link rectangle objects must lie in (default: the lab plate)')
    cli.add_argument('--log-dir', default=DEFAULT_LOG_DIR)
    cli.add_argument('--fake-gripper', action='store_true',
                     help='rehearsal only: do not talk to the RG2')
    cli.add_argument('--poses-file', default=SESSION_POSES_FILE,
                     help='taught poses (shared with lab_console.py)')


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--say', action='append', default=[], metavar='TEXT',
                     help='run this sentence through the full workflow (repeatable)')
    add_session_arguments(cli)
    cli.add_argument('--verify-marker', type=int, metavar='ID')
    cli.add_argument('--evidence', default=os.path.join(
        _HERE, '..', 'docs', 'perception-evidence', 'touch-verification.jsonl'))
    cli.add_argument('--gripper-test', action='store_true')
    args = cli.parse_args()

    rclpy.init()
    try:
        try:
            executor, adapters, orchestrator = build(args)
        except (BackendError, JogError, OSError, RuntimeError) as error:
            say('BLOCKED', str(error))
            return 2
        try:
            if args.gripper_test:
                if not args.execute:
                    say('BLOCKED', '--gripper-test needs --execute')
                    return 2
                return gripper_test(adapters)
            if args.verify_marker is not None:
                return verify_marker(args, executor, adapters)
            failures = 0
            for text in args.say:
                say('SAY', text)
                record = orchestrator.run(text)
                say('RESULT', {'run_id': record.run_id, 'outcome': record.outcome,
                               'reason': record.reason, 'message': record.message,
                               'duration_s': round(record.duration_s, 1),
                               'retries': record.retries})
                failures += record.outcome != 'succeeded'
            return 1 if failures else 0
        except (StageError, JogError) as error:
            say('BLOCKED', str(error))
            return 1
        finally:
            executor.destroy_node()
            executor.motion.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
