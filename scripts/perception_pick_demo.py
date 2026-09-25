#!/usr/bin/env python3
"""Gazebo-only integration: query -> observe -> detect -> locate -> plan -> grasp.

The fixed command grammar is a Phase 2 test driver, not the Phase 3 LLM parser.
The simulator's grasp latch stands in for contact; it is not grasp-quality proof.
"""
import argparse
import json
import time

import rclpy
from std_msgs.msg import Empty
from rosgraph_msgs.msg import Clock
from ur7e_interfaces.srv import DetectObject, LocateObject
from ur7e_interfaces.action import ExecutePrimitive
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from pick_place_demo import PickPlaceDemo, HOVER_HEIGHT, GRIPPER_OPEN, GRIPPER_CLOSED


class PerceptionPick(PickPlaceDemo):
    def service(self, service_type, name, request):
        client = self.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError(f'{name} unavailable')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=40)
            if not future.done() or future.result() is None:
                raise RuntimeError(f'{name} timed out')
            response = future.result()
            if not response.success:
                raise RuntimeError(response.reason)
            return response
        finally:
            self.destroy_client(client)

    def table_contact(self, allowed):
        """Allow only the held object's departure contact with the table."""
        client = self.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('planning scene unavailable')
        request = GetPlanningScene.Request()
        request.components.components = 128
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        self.destroy_client(client)
        if not future.done() or future.result() is None:
            raise RuntimeError('planning scene read timed out')
        matrix = future.result().scene.allowed_collision_matrix
        for name in ['pick_object', 'table']:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        a, b = [matrix.entry_names.index(name) for name in ['pick_object', 'table']]
        matrix.entry_values[a].enabled[b] = allowed
        matrix.entry_values[b].enabled[a] = allowed
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.allowed_collision_matrix = matrix
        self.service(ApplyPlanningScene, '/apply_planning_scene', request)

    def attach_scene(self):
        """Keep the planning world consistent with Gazebo's held block."""
        attached = AttachedCollisionObject()
        attached.link_name = 'gripper_tcp'
        attached.touch_links = ['gripper_tcp', 'onrobot_base_link', 'left_inner_finger',
                                'right_inner_finger', 'left_outer_knuckle', 'right_outer_knuckle',
                                'left_inner_knuckle', 'right_inner_knuckle']
        attached.object.header.frame_id = 'gripper_tcp'
        attached.object.id = 'pick_object'
        attached.object.operation = CollisionObject.ADD
        attached.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.03, .03, .08])]
        center = Pose()
        center.position.z, center.orientation.w = .04, 1.0
        attached.object.primitive_poses = [center]
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [attached]
        self.service(ApplyPlanningScene, '/apply_planning_scene', request)

    def run_query(self, query, stage_pause=0.0):
        def stage(message):
            self.get_logger().info(message)
            time.sleep(stage_pause)

        self.clock_seen = False
        self.create_subscription(Clock, '/clock', lambda _: setattr(self, 'clock_seen', True), 10)
        deadline = time.monotonic()+10
        while not self.clock_seen and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.2)
        if not self.clock_seen:
            raise RuntimeError('simulation clock required')
        latch = self.create_publisher(Empty, '/pick_object/attach', 10)
        stage('1 / OBSERVE: move to the camera observation pose')
        self._run_primitive(primitive=ExecutePrimitive.Goal.PRIMITIVE_GOTO_NAMED, named_pose='observe')
        self._run_gripper(GRIPPER_OPEN)
        # Allow a new capture after motion completes.
        time.sleep(.5)
        stage(f'2 / DETECT: asking OWLv2/perception for "{query}"')
        result = self.service(DetectObject, '/perception/detect_object', DetectObject.Request(query=query))
        self.get_logger().info('DETECT '+result.capture_id)
        location = self.service(LocateObject, '/perception/locate_object',
                                LocateObject.Request(capture_id=result.capture_id))
        target = location.pose
        p = target.pose.position
        self.get_logger().info(f'LOCATE {p.x:.4f} {p.y:.4f} {p.z:.4f}')
        stage('3 / PLAN: validate a complete collision-free hover and descent')
        self._run_primitive(primitive=ExecutePrimitive.Goal.PRIMITIVE_APPROACH_ABOVE,
                            target_pose=target, z_offset=HOVER_HEIGHT)
        stage('4 / DESCEND: follow the validated straight-line grasp path')
        self._run_primitive(primitive=ExecutePrimitive.Goal.PRIMITIVE_CARTESIAN_DESCEND,
                            z_offset=HOVER_HEIGHT)
        stage('5 / GRASP: close gripper and engage the simulation grasp latch')
        self.attach_scene()
        self.table_contact(True)
        self._run_gripper(GRIPPER_CLOSED)
        if latch.get_subscription_count() < 1:
            raise RuntimeError('Gazebo latch bridge missing')
        latch.publish(Empty())
        time.sleep(.3)
        stage('6 / LIFT: raise the held block by 10 cm')
        self._run_primitive(primitive=ExecutePrimitive.Goal.PRIMITIVE_CARTESIAN_LIFT,
                            z_offset=.10)
        self.table_contact(False)
        print(json.dumps({'software_pipeline': 'PARSE->OBSERVE->DETECT->LOCATE->PLAN->GRASP->LIFT',
                          'capture_id': result.capture_id, 'target_m': [p.x, p.y, p.z],
                          'simulation_only': True, 'motion_actions_succeeded': True}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', nargs='?', default='pick red block')
    parser.add_argument('--execute-simulation', action='store_true', required=True)
    parser.add_argument('--stage-pause', type=float, default=0.0,
                        help='Narrated pause before each stage, in seconds (0-5)')
    args = parser.parse_args()
    if not 0 <= args.stage_pause <= 5:
        parser.error('--stage-pause must be between 0 and 5 seconds')
    if not args.command.startswith('pick ') or len(args.command[5:].strip()) == 0:
        parser.error('Phase 2 test grammar is: pick <noun phrase>')
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = PerceptionPick()
    try:
        node.run_query(args.command[5:].strip(), args.stage_pause)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
