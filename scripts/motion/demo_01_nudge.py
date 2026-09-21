#!/usr/bin/env python3
"""Demo 1 - bounded single-joint nudge (the baseline liveness check).

Moves wrist_3 by DELTA and back. The motion is bounded by DELTA (a small
constant), never by a typed absolute target. Run this first every session: if
it ends error_code=0 the whole chain (script -> driver -> reverse interface ->
robot) is alive and it is safe to try the richer demos.

Speed: DELTA/LEG = 0.005 rad/s nominal, cut further by the pendant slider.
"""
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_motion import MotionClient  # noqa: E402

DELTA = 0.05   # rad (~2.9 deg)
LEG = 10       # seconds per leg (out, then back)


def main():
    rclpy.init()
    m = MotionClient()
    home = m.home()
    out = dict(home)
    out['wrist_3_joint'] += DELTA
    m.run([(out, LEG), (home, 2 * LEG)])
    rclpy.shutdown()


if __name__ == '__main__':
    main()
