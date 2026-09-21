#!/usr/bin/env python3
"""Demo 2 - smooth wrist wave. wrist_3 follows a slow sine for a few cycles.

The point is SMOOTHNESS, not speed: many closely-spaced waypoints make one
continuous glide instead of a step. wrist_3 only rotates the tool flange, so
this is low-risk in any pose - a good first "expressive" motion, and a "spin"
in miniature.

It is also the safe way to CHARACTERIZE the velocity veto: raise AMP or lower
PERIOD a little at a time, watching the pendant. The moment a run aborts with a
"speed limit" popup, you have found this robot's actual ceiling (session-2's
open question). The safety cap in ur_motion.py refuses anything reckless first.

Peak speed = AMP * 2*pi / PERIOD (rad/s, nominal; slider scales it down).
"""
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_motion import MotionClient, relative_sine  # noqa: E402

AMP = 0.12       # rad (~6.9 deg)
PERIOD = 16.0    # s per cycle  -> peak vel = 0.047 rad/s nominal
CYCLES = 3


def main():
    rclpy.init()
    m = MotionClient()
    home = m.home()
    pts = relative_sine(home, {'wrist_3_joint': (AMP, 0.0)}, PERIOD, CYCLES)
    m.run(pts)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
