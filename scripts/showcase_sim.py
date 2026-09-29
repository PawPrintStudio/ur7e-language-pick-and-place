#!/usr/bin/env python3
"""Full RViz mock showcase with explicit kinematic object attachment.

Only runs in the network-disabled showcase container on ROS domain 84.
This demonstrates planning and action sequencing, not physical grasp dynamics.
MoveIt attachment API reference:
https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
"""
import copy
import os
from pathlib import Path
import sys
import time

import rclpy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from ur7e_pick_place_bringup.planning_scene import _box

from pick_place_demo import PickPlaceDemo


def simulation_guard():
    if (not Path("/.dockerenv").exists()
            or os.environ.get("ROS_DOMAIN_ID") != "84"
            or {p.name for p in Path("/sys/class/net").iterdir()} != {"lo"}):
        raise RuntimeError("Showcase requires a network-disabled Docker container and ROS domain 84")


class Showcase(PickPlaceDemo):
    def __init__(self):
        super().__init__()
        self.apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.attached = False

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError("Planning scene service unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError("Planning scene service timeout")
        return future.result()

    def scene_diff(self, objects=(), attached=()):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = list(objects)
        scene.robot_state.attached_collision_objects = list(attached)
        if not self.call(self.apply, ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError("Planning scene update failed")

    @staticmethod
    def remove(name):
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        return obj

    def prepare(self):
        # The destination box is a marker for an empty placement location,
        # not a second physical object to collide with the carried object.
        detach = AttachedCollisionObject()
        detach.link_name = "gripper_tcp"
        detach.object = self.remove("pick_object")
        obj = _box("base_link", "pick_object", (0.03, 0.03, 0.08), (0.45, -0.15, 0.04))
        request = GetPlanningScene.Request()
        request.components.components = 16 | 4
        current = self.call(self.get_scene, request).scene
        objects = [obj]
        if any(o.id == "place_target" for o in current.world.collision_objects):
            objects.insert(0, self.remove("place_target"))
        attached = [detach] if any(a.object.id == "pick_object" for a in
                                  current.robot_state.attached_collision_objects) else []
        self.scene_diff(objects, attached)
        self.original = copy.deepcopy(obj)
        super()._run_gripper(0.10)

    def _run_gripper(self, position):
        # A 30 mm object prevents a physical full close; represent that width.
        super()._run_gripper(0.03 if position == 0 else position)
        if position == 0:
            attachment = AttachedCollisionObject()
            attachment.link_name = "gripper_tcp"
            attachment.object = copy.deepcopy(self.original)
            attachment.touch_links = ["left_inner_finger", "right_inner_finger"]
            # Humble processes attached bodies before world diffs and removes
            # the matching world object automatically. An extra REMOVE would
            # report failure because the object is already gone.
            self.scene_diff(attached=[attachment])
            self.attached = True
            self.get_logger().info("SIMULATION: object attached to gripper for transport")
        elif self.attached:
            detach = AttachedCollisionObject()
            detach.link_name = "gripper_tcp"
            detach.object = self.remove("pick_object")
            # MoveIt puts the detached object into the world at its current
            # transformed pose; avoid teleporting it to a prescribed target.
            self.scene_diff(attached=[detach])
            self.attached = False
            self.get_logger().info("SIMULATION: object released at the place location")
        time.sleep(0.5)

    def _run_primitive(self, **fields):
        super()._run_primitive(**fields)
        time.sleep(0.5)

    def verify_place(self):
        request = GetPlanningScene.Request()
        request.components.components = 16 | 4  # world geometry + attached objects
        scene = self.call(self.get_scene, request).scene
        if any(a.object.id == "pick_object" for a in scene.robot_state.attached_collision_objects):
            raise RuntimeError("Object still attached after release")
        objects = [o for o in scene.world.collision_objects if o.id == "pick_object"]
        if len(objects) != 1:
            raise RuntimeError("Placed object missing")
        obj = objects[0]
        # Humble represents shape placement as object pose plus primitive pose.
        p = obj.primitive_poses[0].position
        base = obj.pose.position
        actual = (base.x + p.x, base.y + p.y, base.z + p.z)
        self.get_logger().info(f"Placed object frame={obj.header.frame_id}, position={actual}")
        if obj.header.frame_id not in ("base_link", "world"):
            raise RuntimeError("Unexpected placed-object frame")
        if sum((a-b)**2 for a,b in zip(actual, (0.45, 0.20, 0.04))) ** 0.5 > 0.01:
            raise RuntimeError("Object is not within 10 mm of the place target")


def main():
    simulation_guard()
    rclpy.init()
    node = Showcase()
    try:
        node.prepare()
        node.run()
        node.verify_place()
        return 0
    except Exception as error:
        node.get_logger().error(f"Showcase failed: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
