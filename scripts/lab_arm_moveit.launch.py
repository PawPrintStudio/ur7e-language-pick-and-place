"""Arm-only lab planning from the running driver's calibrated description.

No hardware controllers or TF publishers are started here. The attached RG2
is represented by a conservative fixed collision envelope, not fake joint
feedback. Named lab poses are captured from live joints; production home and
observe poses are deliberately not reused from a possibly folded start.
"""

from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
import xacro
import yaml


def setup(context):
    rclpy.init()
    node = rclpy.create_node('lab_description_reader')
    try:
        client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Start the calibrated arm driver first')
        future = client.call_async(GetParameters.Request(names=['robot_description']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Could not read the live robot description')
        description = future.result().values[0].string_value
        samples = []
        node.create_subscription(JointState, '/joint_states', samples.append, 10)
        deadline = time.monotonic() + 5
        while not samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not samples:
            raise RuntimeError('No live joint state; refusing to construct lab poses')
        joints = dict(zip(samples[-1].name, samples[-1].position))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    robot = ET.fromstring(description)
    if robot.find(".//joint[@name='finger_width']") is not None:
        raise RuntimeError('This launch is only for the arm-only driver')
    # RG2 collision-mesh bounds sampled across 0..110 mm opening (1 mm
    # increments), expressed in tool0: x +/-72.62 mm, y -39.43..35.89 mm,
    # z 0..233.59 mm. Add mounting allowance along Z and at least 5 mm
    # padding. This envelope covers every opening, without invented feedback.
    link = ET.SubElement(robot, 'link', name='lab_gripper_envelope')
    collision = ET.SubElement(link, 'collision')
    ET.SubElement(collision, 'origin', xyz='0 0 0.135', rpy='0 0 0')
    geometry = ET.SubElement(collision, 'geometry')
    ET.SubElement(geometry, 'box', size='0.16 0.09 0.28')
    joint = ET.SubElement(robot, 'joint', name='lab_gripper_envelope_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='tool0')
    ET.SubElement(joint, 'child', link='lab_gripper_envelope')

    share = Path(get_package_share_directory('ur_moveit_config'))
    srdf = ET.fromstring(xacro.process_file(
        str(share / 'srdf/ur.srdf.xacro'), mappings={'name': 'ur', 'prefix': ''}
    ).toxml())
    srdf.set('name', robot.attrib['name'])
    for state in list(srdf.findall('group_state')):
        srdf.remove(state)
    for name, delta in [('lab_start', 0.0), ('lab_wrist_offset', 0.02)]:
        state = ET.SubElement(srdf, 'group_state', name=name, group='ur_manipulator')
        for name in ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']:
            ET.SubElement(state, 'joint', name=name,
                          value=str(joints[name] + (delta if name == 'wrist_3_joint' else 0)))
    for name in ['tool0', 'flange', 'wrist_3_link']:
        ET.SubElement(srdf, 'disable_collisions', link1='lab_gripper_envelope',
                      link2=name, reason='Adjacent')

    repo = Path(__file__).resolve().parents[1]
    config = repo / 'src/ur7e_pick_place_bringup/config'
    ik = yaml.safe_load((config / 'ur7e_kinematics.yaml').read_text())
    if LaunchConfiguration('ik').perform(context) == 'kdl':
        # The console's Cartesian jogs are 1 mm IK steps seeded from the
        # previous one. Measured on the real arm 2026-09-30: pick_ik's local
        # gradient descent planned the same 5 cm lateral move to 100% or 4%
        # depending on encoder noise in the start sample, and to 0% once its
        # accept threshold was tightened below the step. KDL's Newton-Raphson
        # from the seed is the right tool for tiny steps; the path's
        # revolute_jump_threshold still guards the wrist singularities that
        # motivated pick_ik for the pick pipeline (task 1.4).
        ik = {'ur_onrobot_manipulator': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005,
            'kinematics_solver_timeout': 0.05,
            'kinematics_solver_attempts': 3,
        }}
    controllers = yaml.safe_load((config / 'ur7e_moveit_controllers.yaml').read_text())
    ompl = yaml.safe_load((share / 'config/ompl_planning.yaml').read_text())
    ompl.update({
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                            'default_planner_request_adapters/FixWorkspaceBounds '
                            'default_planner_request_adapters/FixStartStateBounds '
                            'default_planner_request_adapters/FixStartStateCollision '
                            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.01,
    })
    params = {
        'robot_description': ET.tostring(robot, encoding='unicode'),
        'robot_description_semantic': ET.tostring(srdf, encoding='unicode'),
        'robot_description_kinematics': {'ur_manipulator': ik['ur_onrobot_manipulator']},
        'robot_description_planning': yaml.safe_load((share / 'config/joint_limits.yaml').read_text()),
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 15.0,
        'trajectory_execution.allowed_goal_duration_margin': 5.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'use_sim_time': False,
    }
    params.update(controllers)
    return [Node(package='moveit_ros_move_group', executable='move_group',
                 output='screen', parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ik', default_value='pick_ik', choices=['pick_ik', 'kdl'],
            description='IK plugin for the arm group. kdl for the lab console '
                        '(millimetre Cartesian jogs); pick_ik matches the pick '
                        'pipeline configuration.'),
        OpaqueFunction(function=setup),
    ])
