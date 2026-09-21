#!/usr/bin/env python3
"""Keyboard jog teleop for the UR7e (task 0.6 / issue #6).

Discrete-jog teleop: each keypress moves ONE selected joint by a fixed small
step, sent as a short trajectory through the shared MotionClient — so it inherits
the same safety envelope, joint-by-name mapping, and abort handling as the demos.

It is discrete (press key -> arm jogs a step -> completes) rather than continuous
velocity streaming, on purpose: our bringup drives the scaled_joint_trajectory_
controller (not a velocity controller), and the velocity veto is not yet
characterized. Discrete steps keep every command provably under the cap.

    keys
      1..6      select joint (1=base ... 6=wrist_3)
      .  or  >  jog selected joint by +STEP
      ,  or  <  jog selected joint by -STEP
      [ / ]     halve / double STEP (bounded)
      h         return all joints to the start pose
      q / Ctrl-C  quit

Requires a real interactive terminal (raw tty). Run on the Jetson console with
the driver up and Play pressed. Keep the pendant slider low and a hand on the
e-stop.

STATUS: written 2026-09-21, NOT YET run on hardware — verify before trusting.
"""
import os
import select
import sys
import termios
import tty

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_motion import JOINTS, MotionClient  # noqa: E402

STEP = 0.05        # rad per jog (~2.9 deg)
STEP_MIN = 0.005
STEP_MAX = 0.20
STEP_TIME = 2.0    # s per jog -> 0.05/2 = 0.025 rad/s nominal, under the cap
HOME_VEL = 0.03    # rad/s used to time the 'h' return move


def get_key(timeout=None):
    """Read one keystroke in raw mode (blocking if timeout is None)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def status(sel, target, home0):
    j = JOINTS[sel]
    sys.stdout.write('\r[joint %d/%s = %+.3f rad, STEP=%.3f]  '
                     % (sel + 1, j, target[j], STEP) + ' ' * 8)
    sys.stdout.flush()


def main():
    global STEP
    rclpy.init()
    m = MotionClient()
    home0 = m.home()          # the pose to return to on 'h'
    target = dict(home0)      # running commanded target
    sel = 5                   # start on wrist_3 (lowest-risk joint)

    print(__doc__.split('\n\n')[2])   # print the key map
    print('ready — driver up and Play pressed? jog away.\n')
    status(sel, target, home0)

    try:
        while rclpy.ok():
            k = get_key()
            if k in ('q', '\x03'):        # q or Ctrl-C
                break
            elif k in '123456':
                sel = int(k) - 1
            elif k in ('.', '>', ',', '<'):
                sign = 1.0 if k in ('.', '>') else -1.0
                nxt = dict(target)
                nxt[JOINTS[sel]] += sign * STEP
                if m.run([(nxt, STEP_TIME)], anchor=target):
                    target = nxt
            elif k == '[':
                STEP = max(STEP_MIN, STEP / 2)
            elif k == ']':
                STEP = min(STEP_MAX, STEP * 2)
            elif k == 'h':
                dmax = max(abs(target[j] - home0[j]) for j in JOINTS)
                if dmax > 1e-4:
                    t = max(3.0, dmax / HOME_VEL)
                    if m.run([(dict(home0), t)], anchor=target):
                        target = dict(home0)
            status(sel, target, home0)
    finally:
        print('\nteleop done.')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
