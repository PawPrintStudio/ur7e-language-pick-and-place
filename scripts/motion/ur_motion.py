#!/usr/bin/env python3
"""Shared motion helpers + safety scaffolding for the UR7e demo ladder.

Every demo builds a joint-space trajectory and hands it to ``MotionClient.run()``,
which enforces a safety envelope BEFORE anything reaches the robot:

* ``MAX_JOINT_VEL``  - hard ceiling on commanded joint speed (rad/s, NOMINAL;
  the pendant speed slider scales the ACTUAL speed further down). ``run()``
  refuses any trajectory that implies a faster move between consecutive
  waypoints. Set well under session-2's observed ~0.018 rad/s "velocity veto"
  so that a low slider keeps the actual speed comfortably safe.
* ``MAX_JOINT_STEP`` - loose sanity cap: no single waypoint may jump a joint
  more than this from the previous one (catches a wild sample, not slow moves).
* relative + home-return - the sampling helpers build motion RELATIVE to the
  pose the arm is in when the script starts, and return to it. Absolute targets
  are never typed, so a fat-fingered number cannot fling the arm across the room.

CRITICAL, learned on real hardware (see the README): ``/joint_states`` does NOT
publish joints in anatomical order. We read ``name -> position`` into a dict and
rebuild every trajectory by explicit joint name. Never zip positions by index.

None of this replaces the human. Keep the pendant slider low (10-25 %) and a
hand on the e-stop. There is NO collision checking here - that arrives with
MoveIt in phase 1. Motions are small and relative on purpose; ramp amplitude up
slowly while watching the arm, never blind.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Canonical order we COMMAND in. /joint_states may report a different order;
# we always map by name (see module docstring).
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# --- safety envelope -------------------------------------------------------
MAX_JOINT_VEL = 0.10     # rad/s NOMINAL (before the pendant slider). Hard cap.
MAX_JOINT_STEP = 0.50    # rad, max change per waypoint per joint (sanity net).
ACTION = '/scaled_joint_trajectory_controller/follow_joint_trajectory'


def _dur(t):
    return Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def relative_sine(home, joint_amps, period, cycles, dt=0.1):
    """Smooth relative sine per joint, densely sampled.

    ``joint_amps``: ``{joint_name: (amplitude_rad, phase_rad)}``.
    Offset = ``amp * (sin(2*pi*t/period + phase) - sin(phase))`` so every joint
    starts AND ends at ``home`` (offset 0 at t=0 and t=cycles*period), whatever
    its phase. Dense sampling (dt=0.1 s) keeps linear interpolation smooth
    without needing explicit waypoint velocities.
    """
    pts = []
    n = int(round(cycles * period / dt))
    for k in range(1, n + 1):
        t = k * dt
        pose = dict(home)
        for j, (amp, phase) in joint_amps.items():
            pose[j] = home[j] + amp * (math.sin(2 * math.pi * t / period + phase)
                                       - math.sin(phase))
        pts.append((pose, t))
    return pts


class MotionClient(Node):
    def __init__(self):
        super().__init__('ur_motion')
        self._start = None
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self._ac = ActionClient(self, FollowJointTrajectory, ACTION)

    def _js(self, msg):
        if self._start is None:
            self._start = dict(zip(msg.name, msg.position))

    def home(self):
        """Block until the current pose is read; return it as a name->rad dict."""
        while rclpy.ok() and self._start is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(
            'start pose: ' + ', '.join('%s=%.3f' % (j, self._start[j]) for j in JOINTS))
        return dict(self._start)

    def _check(self, pts, anchor):
        """Raise ValueError if the trajectory leaves the safety envelope."""
        seq = [(dict(anchor), 0.0)] + list(pts)
        prev_pos, prev_t = seq[0]
        for pos, t in seq[1:]:
            dt = t - prev_t
            for j in JOINTS:
                dp = abs(pos[j] - prev_pos[j])
                if dp > MAX_JOINT_STEP:
                    raise ValueError('waypoint step %.3f rad on %s exceeds '
                                     'MAX_JOINT_STEP %.2f' % (dp, j, MAX_JOINT_STEP))
                if dt > 0 and dp / dt > MAX_JOINT_VEL:
                    raise ValueError('implied vel %.4f rad/s on %s exceeds '
                                     'MAX_JOINT_VEL %.2f (nominal). Slow the '
                                     'motion or reduce amplitude.'
                                     % (dp / dt, j, MAX_JOINT_VEL))
            prev_pos, prev_t = pos, t

    def run(self, pts, anchor=None):
        """Send a trajectory. ``pts``: list of (pose_dict, time_from_start_s).

        ``anchor``: the pose the first waypoint is safety-checked against
        (default: the pose read at startup). Teleop passes the previous target
        so each small jog is validated as a delta, not against the original home.

        Returns True on SUCCESSFUL, False otherwise. Enforces the safety
        envelope first; a violation raises before anything reaches the robot.
        """
        if anchor is None:
            anchor = self._start
        self._check(pts, anchor)
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        for pos, t in pts:
            p = JointTrajectoryPoint()
            p.positions = [float(pos[j]) for j in JOINTS]
            p.time_from_start = _dur(t)
            traj.points.append(p)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('no action server - is the driver running?')
            return False
        self.get_logger().info('sending %d waypoints over %.1f s'
                               % (len(pts), pts[-1][1]))
        fut = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('goal REJECTED - External Control not running? '
                                    'Press Play on the pendant.')
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        ec = rf.result().result.error_code
        if ec == 0:
            self.get_logger().info('SUCCESSFUL (error_code=0)')
            return True
        self.get_logger().error(
            'ABORTED error_code=%d. Pendant popup said "speed limit"? -> the '
            'session-2 velocity veto; slow it. Protective stop? -> unlock '
            '(>=5 s), then RESTART the driver before retrying.' % ec)
        return False
