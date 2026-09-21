#!/usr/bin/env python3
"""Demo 3 - fluid multi-joint motion (the "dance").

Several joints follow slow sines with different PHASES, so the arm moves in one
continuous, sinuous, alive-looking way. The expressiveness comes from smooth
coordination and phase offsets, NOT from speed - a slow phased wave reads as far
more "fluid" than a fast jerky one.

SAFETY, read before running:
* Amplitudes are small and RELATIVE to the current pose. Ramp them up slowly,
  watching the arm, hand on the e-stop.
* There is NO collision checking yet (that is MoveIt, phase 1). The big joints
  (pan/lift/elbow) sweep real volume, so their amplitudes are kept small.
* Freedrive the arm to an OPEN posture first. In a folded pose (elbow deeply
  bent, as observed on 2026-09-21) small moves can bring links near each other.

joint -> (amplitude_rad, phase_rad). Tune amplitudes to taste; the ur_motion.py
safety cap will refuse anything that implies too fast a move.
"""
import math
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_motion import MotionClient, relative_sine  # noqa: E402

CHOREO = {
    'shoulder_pan_joint':  (0.06, 0.0),          # small: sweeps the whole arm
    'shoulder_lift_joint': (0.05, math.pi / 2),  # small: sweeps the whole arm
    'elbow_joint':         (0.06, math.pi),      # small: sweeps the forearm
    'wrist_1_joint':       (0.10, math.pi / 2),
    'wrist_2_joint':       (0.10, 0.0),
    'wrist_3_joint':       (0.15, math.pi / 3),  # largest: flange, low risk
}
PERIOD = 16.0    # s per cycle
CYCLES = 3


def main():
    rclpy.init()
    m = MotionClient()
    home = m.home()
    pts = relative_sine(home, CHOREO, PERIOD, CYCLES)
    m.run(pts)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
