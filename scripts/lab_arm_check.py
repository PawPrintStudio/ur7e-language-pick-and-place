#!/usr/bin/env python3
"""Bounded arm-only MoveIt checks; planning-only unless --execute is supplied."""

import argparse
from copy import deepcopy
import json
import math
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan
from moveit_msgs.srv import GetPositionFK, GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float64
from ur_dashboard_msgs.msg import SafetyMode


class Check(Node):
    def __init__(self):
        super().__init__('lab_arm_check')
        self.latest = None
        self.received = 0.0
        self.speed = None
        self.speed_time = 0.0
        self.running = False
        self.safety = None
        self.max_excursion = 0.05
        self.max_speed_percent = 10.0
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                                 self.scaling, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                 lambda msg: setattr(self, 'running', msg.data), qos)
        self.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode',
                                 lambda msg: setattr(self, 'safety', msg.mode), qos)

    def joints(self, msg):
        self.latest = msg
        self.received = time.monotonic()

    def scaling(self, msg):
        self.speed = msg.data
        self.speed_time = time.monotonic()

    def fresh(self):
        until = time.monotonic() + 1
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest is None or time.monotonic() - self.received > 0.5:
            raise RuntimeError('No fresh physical joint state')
        if max(abs(v) for v in self.latest.velocity) > 0.0002:
            raise RuntimeError('Robot is moving; wait for the active test to finish')
        return RobotState(joint_state=deepcopy(self.latest))

    def call(self, kind, endpoint, request):
        client = self.create_client(kind, endpoint)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError(f'{endpoint} unavailable')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15)
            if not future.done() or future.result() is None:
                raise RuntimeError(f'{endpoint} timed out')
            return future.result()
        finally:
            self.destroy_client(client)

    def box(self, name, xyz, dimensions, remove=False):
        obj = CollisionObject(id=name)
        obj.header.frame_id = 'base_link'
        obj.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
        if not remove:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = xyz
            pose.orientation.w = 1.0
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dimensions)]
            obj.primitive_poses = [pose]
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.world.collision_objects = [obj]
        if not self.call(ApplyPlanningScene, '/apply_planning_scene', req).success:
            raise RuntimeError('Planning scene update failed')

    def valid(self, state):
        response = self.call(GetStateValidity, '/check_state_validity',
                             GetStateValidity.Request(robot_state=state,
                                                      group_name='ur_manipulator'))
        print(json.dumps({'state_valid': response.valid,
                          'contacts': [[c.contact_body_1, c.contact_body_2]
                                       for c in response.contacts]}), flush=True)
        return response.valid

    def fk(self, state):
        req = GetPositionFK.Request(robot_state=state, fk_link_names=['tool0'])
        req.header.frame_id = 'base_link'
        response = self.call(GetPositionFK, '/compute_fk', req)
        if response.error_code.val != 1:
            raise RuntimeError(f'FK failed: {response.error_code.val}')
        return response.pose_stamped[0]

    def wrist_plan(self, state, delta=0.02):
        req = GetMotionPlan.Request()
        plan = req.motion_plan_request
        plan.group_name = 'ur_manipulator'
        plan.start_state = state
        plan.allowed_planning_time = 3.0
        plan.num_planning_attempts = 1
        plan.max_velocity_scaling_factor = 0.01
        plan.max_acceleration_scaling_factor = 0.01
        target = Constraints()
        for name, value in zip(state.joint_state.name, state.joint_state.position):
            target.joint_constraints.append(JointConstraint(
                joint_name=name, position=value + (delta if name == 'wrist_3_joint' else 0),
                tolerance_above=0.0002, tolerance_below=0.0002, weight=1.0))
            plan.path_constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=0.05,
                tolerance_below=0.05, weight=1.0))
        plan.goal_constraints = [target]
        return self.call(GetMotionPlan, '/plan_kinematic_path', req).motion_plan_response

    def cartesian(self, state, delta, target=None):
        if target is None:
            target = deepcopy(self.fk(state).pose)
            target.position.z += delta
        req = GetCartesianPath.Request(start_state=state, group_name='ur_manipulator',
                                       link_name='tool0', waypoints=[target], max_step=0.001,
                                       # Tiny paths make a relative-to-average jump
                                       # heuristic unstable. Use absolute limits instead.
                                       jump_threshold=0.0, revolute_jump_threshold=0.02,
                                       avoid_collisions=True,
                                       max_velocity_scaling_factor=0.01,
                                       max_acceleration_scaling_factor=0.01)
        req.header.frame_id = 'base_link'
        return self.call(GetCartesianPath, '/compute_cartesian_path', req)

    def audit(self, trajectory, state):
        start = dict(zip(state.joint_state.name, state.joint_state.position))
        names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        if len(points) < 2 or set(names) != set(start):
            raise RuntimeError('Incomplete trajectory')
        previous = [start[j] for j in names]
        excursion = 0.0
        jump = 0.0
        for point in points:
            if len(point.positions) != len(names) or any(
                    not math.isfinite(v) for v in point.positions):
                raise RuntimeError('Invalid trajectory positions')
            excursion = max(excursion, max(abs(point.positions[i]-start[j])
                                          for i, j in enumerate(names)))
            jump = max(jump, max(abs(a-b) for a, b in zip(point.positions, previous)))
            previous = point.positions
        if excursion > self.max_excursion or jump > 0.01:
            raise RuntimeError(f'Trajectory bounds failed: excursion={excursion}, jump={jump}')
        print(json.dumps({'audited_max_excursion_rad': excursion,
                          'audited_max_waypoint_step_rad': jump}), flush=True)

    def execute(self, trajectory):
        state = self.fresh()
        self.audit(trajectory, state)
        if (self.safety != SafetyMode.NORMAL or not self.running or self.speed is None
                or not 0 < self.speed <= self.max_speed_percent + 0.1
                or time.monotonic() - self.speed_time > 0.5):
            raise RuntimeError(f'Execution gate: safety={self.safety}, '
                               f'running={self.running}, speed={self.speed}')
        start = dict(zip(state.joint_state.name, state.joint_state.position))
        traj = deepcopy(trajectory)
        points = traj.joint_trajectory.points
        names = traj.joint_trajectory.joint_names
        if len(points) < 2 or set(names) != set(start):
            raise RuntimeError('Incomplete trajectory')
        if max(abs(points[0].positions[i] - start[j]) for i, j in enumerate(names)) > 0.002:
            raise RuntimeError('Trajectory start differs from fresh feedback')
        elapsed = 0.0
        previous = points[0].positions
        for index, point in enumerate(points):
            if any(not math.isfinite(v) for v in point.positions):
                raise RuntimeError('Nonfinite trajectory')
            if max(abs(point.positions[i] - start[j]) for i, j in enumerate(names)) > self.max_excursion:
                raise RuntimeError('Plan exceeds joint excursion bound')
            if index:
                step = max(abs(a-b) for a, b in zip(point.positions, previous))
                if step > 0.01:
                    raise RuntimeError('Plan exceeds 0.01-rad per-waypoint jump bound')
                elapsed += max(0.05, step / 0.02)
            point.time_from_start = Duration(sec=int(elapsed),
                                            nanosec=int((elapsed % 1) * 1e9))
            # Positions-only interpolation avoids the original plan's faster derivatives.
            point.velocities = []
            point.accelerations = []
            previous = point.positions
        print(json.dumps({'executing_points': len(points), 'nominal_seconds': elapsed,
                          'speed_percent': self.speed}), flush=True)
        client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        try:
            if not client.wait_for_server(timeout_sec=5):
                raise RuntimeError('MoveIt execution server unavailable')
            sent = client.send_goal_async(ExecuteTrajectory.Goal(trajectory=traj))
            rclpy.spin_until_future_complete(self, sent, timeout_sec=5)
            if not sent.done() or not sent.result().accepted:
                raise RuntimeError('Execution not accepted')
            goal = sent.result()
            future = goal.get_result_async()
            deadline = time.monotonic() + elapsed * 15 + 20
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if (not self.running or self.safety != SafetyMode.NORMAL
                        or self.speed is None or not math.isfinite(self.speed)
                        or self.speed > self.max_speed_percent + 0.1
                        or time.monotonic() - self.received > 0.5
                        or time.monotonic() - self.speed_time > 0.5):
                    snapshot = {'running': self.running, 'safety': self.safety,
                                'speed_percent': self.speed,
                                'joint_age_s': time.monotonic() - self.received,
                                'speed_age_s': time.monotonic() - self.speed_time}
                    print(json.dumps({'execution_gate_failure': snapshot}), flush=True)
                    cancel = goal.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel, timeout_sec=5)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=5)
                    print(json.dumps({'cancel_response_code':
                                      cancel.result().return_code if cancel.done() else None,
                                      'terminal_status':
                                      future.result().status if future.done() else None}), flush=True)
                    raise RuntimeError('Live telemetry execution gate changed; cancellation requested')
            if not future.done():
                goal.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future, timeout_sec=5)
                raise RuntimeError('Execution deadline exceeded; cancellation requested')
            result = future.result()
            print(json.dumps({'action_status': result.status,
                              'moveit_error_code': result.result.error_code.val}), flush=True)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != 1:
                raise RuntimeError('Trajectory did not succeed')
            current = self.fresh().joint_state
            actual = dict(zip(current.name, current.position))
            error = max(abs(actual[j] - points[-1].positions[i]) for i, j in enumerate(names))
            print(json.dumps({'max_endpoint_error_rad': error}), flush=True)
            if error > 0.003:
                raise RuntimeError('Joint endpoint verification failed')
        finally:
            client.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--max-speed-percent', type=float, choices=[10.0, 50.0],
                        default=10.0, help='Operator-approved pendant speed ceiling')
    parser.add_argument('--vertical', choices=['up', 'down'],
                        help='Only check a single 5 mm vertical segment')
    parser.add_argument('--visible', action='store_true',
                        help='Plan a 5 cm lift and return, with a 0.25-rad excursion cap')
    args = parser.parse_args()
    rclpy.init()
    node = Check()
    node.max_speed_percent = args.max_speed_percent
    if args.visible:
        node.max_excursion = 0.25
    try:
        state = node.fresh()
        # Top just below base mounting plane. Conservative for a raised base.
        node.box('lab_table', [0.0, 0.0, -0.102], [4.0, 4.0, 0.2])
        if not node.valid(state):
            raise RuntimeError('Starting state collides in the conservative lab model')
        fk = node.fk(state)
        print(json.dumps({'tool0': [fk.pose.position.x, fk.pose.position.y,
                                   fk.pose.position.z]}), flush=True)
        if args.visible:
            up = node.cartesian(state, 0.05)
            if up.error_code.val != 1 or up.fraction < 0.999:
                raise RuntimeError(f'Complete 5 cm lift unavailable: {up.fraction}')
            node.audit(up.solution, state)
            predicted = deepcopy(state)
            predicted.joint_state.name = up.solution.joint_trajectory.joint_names
            predicted.joint_state.position = up.solution.joint_trajectory.points[-1].positions
            predicted.joint_state.velocity = [0.0] * len(predicted.joint_state.name)
            back = node.cartesian(predicted, 0.0, target=fk.pose)
            if back.error_code.val != 1 or back.fraction < 0.999:
                raise RuntimeError(f'Complete return unavailable: {back.fraction}')
            node.audit(back.solution, predicted)
            print(json.dumps({'visible_lift_and_return_planned': True}), flush=True)
            if args.execute:
                node.execute(up.solution)
                raised_state = node.fresh()
                raised = node.fk(raised_state).pose.position
                print(json.dumps({'measured_lift_xyz_m': [raised.x-fk.pose.position.x,
                                  raised.y-fk.pose.position.y, raised.z-fk.pose.position.z]}),
                      flush=True)
                back = node.cartesian(raised_state, 0.0, target=fk.pose)
                if back.error_code.val != 1 or back.fraction < 0.999:
                    raise RuntimeError('Return replanning failed; holding raised position')
                node.execute(back.solution)
                final = node.fk(node.fresh()).pose.position
                error = math.sqrt(sum((a-b)**2 for a, b in zip(
                    [final.x, final.y, final.z],
                    [fk.pose.position.x, fk.pose.position.y, fk.pose.position.z])))
                print(json.dumps({'return_position_error_m': error}), flush=True)
                if error > 0.002:
                    raise RuntimeError('Cartesian return endpoint outside 2 mm tolerance')
            print(json.dumps({'visible_checks_passed': True,
                              'physical_execution': args.execute}), flush=True)
            return
        if args.vertical:
            delta = 0.005 if args.vertical == 'up' else -0.005
            cart = node.cartesian(state, delta)
            print(json.dumps({'cartesian_code': cart.error_code.val,
                              'fraction': cart.fraction}), flush=True)
            if cart.error_code.val != 1 or cart.fraction < 0.999:
                raise RuntimeError('Complete Cartesian segment unavailable')
            if args.execute:
                node.execute(cart.solution)
                final = node.fk(node.fresh()).pose.position
                print(json.dumps({'measured_xyz_delta_m': [final.x-fk.pose.position.x,
                                  final.y-fk.pose.position.y, final.z-fk.pose.position.z]}),
                      flush=True)
            return
        plan = node.wrist_plan(state)
        print(json.dumps({'wrist_plan_code': plan.error_code.val,
                          'points': len(plan.trajectory.joint_trajectory.points)}), flush=True)
        if plan.error_code.val != 1:
            raise RuntimeError('Bounded wrist plan failed')
        cart = node.cartesian(state, 0.005)
        print(json.dumps({'cartesian_code': cart.error_code.val, 'fraction': cart.fraction}),
              flush=True)
        # Insert a virtual obstacle around tool0 and require rejection. Never execute it.
        p = fk.pose.position
        node.box('lab_rejection_probe', [p.x, p.y, p.z], [0.12, 0.12, 0.12])
        try:
            blocked = node.wrist_plan(state)
            assert not node.valid(state)
            assert blocked.error_code.val != 1, 'Planner accepted a colliding start'
            print(json.dumps({'collision_rejected_code': blocked.error_code.val}), flush=True)
        finally:
            node.box('lab_rejection_probe', [], [], remove=True)
        if not node.valid(node.fresh()):
            raise RuntimeError('Scene did not recover after virtual-obstacle removal')
        if args.execute:
            node.execute(plan.trajectory)
            back = node.wrist_plan(node.fresh(), -0.02)
            if back.error_code.val != 1:
                raise RuntimeError('Wrist return plan failed')
            node.execute(back.trajectory)
            for delta in [0.005, -0.005]:
                cart = node.cartesian(node.fresh(), delta)
                if cart.error_code.val != 1 or cart.fraction < 0.999:
                    raise RuntimeError('Complete Cartesian segment unavailable')
                node.execute(cart.solution)
        print(json.dumps({'checks_passed': True, 'physical_execution': args.execute}), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
