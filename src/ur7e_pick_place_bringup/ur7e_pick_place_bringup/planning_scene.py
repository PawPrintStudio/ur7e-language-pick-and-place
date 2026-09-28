#!/usr/bin/env python3
"""
Publish the fixed workspace collision geometry for Phase 1 as MoveIt CollisionObjects.

Covers task 1.3's table collision and task 1.7's hardcoded pick/place poses.

Deliberately NOT baked into the URDF: this is workspace furniture, not part
of the robot, and 1.7's whole premise ("taped, hardcoded pose") means these
numbers get re-measured against the real table during a lab session. Keeping
them here means that re-measurement is a one-file edit, no xacro/URDF rebuild
and no re-running xacro:include chains across three packages.

All poses are placeholders pending the lab measurement — see the module-level
comments below for exactly what to re-check.
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def _box(frame_id, object_id, size_xyz, position_xyz):
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size_xyz)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position_xyz
    pose.orientation.w = 1.0
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


class PlanningSceneSeeder(Node):
    def __init__(self):
        super().__init__("planning_scene_seeder")
        self._pub = self.create_publisher(PlanningScene, "/planning_scene", 1)
        # /planning_scene is only latched by convention in RViz; move_group's
        # PlanningSceneMonitor needs a moment to be listening, and QoS on this
        # topic is volatile — send a few times rather than once.
        self._sent = 0
        self._timer_handle = self.create_timer(1.0, self._publish_once)

    def _publish_once(self):
        if self._sent >= 3:
            self.destroy_timer(self._timer_handle)
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = self._objects()
        self._pub.publish(scene)
        self._sent += 1
        self.get_logger().info(
            f"published {len(scene.world.collision_objects)} collision "
            f"object(s) ({self._sent}/3 announcements)"
        )

    def _objects(self):
        # PLACEHOLDER — re-measure against the real makerspace table + taped
        # marks during the next lab session (task 1.7's own acceptance
        # criterion is "taped, hardcoded pose"; these are that pose, until
        # measured). base_link is the robot's mounting point; if the arm
        # isn't mounted flush with the tabletop, adjust table_top_z first —
        # everything else is relative to it.
        table_top_z = 0.0
        table_thickness = 0.05
        table = _box(
            "base_link", "table",
            size_xyz=(1.0, 0.8, table_thickness),
            position_xyz=(0.4, 0.0, table_top_z - table_thickness / 2),
        )

        # The object task 1.7 picks: small enough for the RG2's 110 mm max
        # stroke (D6), tall enough for a two-finger side grasp with 10-20mm
        # of finger past its center.
        pick_object = _box(
            "base_link", "pick_object",
            size_xyz=(0.03, 0.03, 0.08),
            position_xyz=(0.45, -0.15, table_top_z + 0.04),
        )

        # Where 1.7 places it — far enough from the pick pose to prove the
        # arm actually transports the object, not just re-grasps in place.
        place_object = _box(
            "base_link", "place_target",
            size_xyz=(0.03, 0.03, 0.08),
            position_xyz=(0.45, 0.20, table_top_z + 0.04),
        )

        return [table, pick_object, place_object]


def main():
    rclpy.init()
    node = PlanningSceneSeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
