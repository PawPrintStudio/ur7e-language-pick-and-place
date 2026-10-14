"""Timing for a planned joint path: thin dense waypoints, then time them at a rate.

Pure Python, no ROS: ``lab_jog.JogExecutor.retime`` calls this, and the tests
run on a laptop.

Why thin at all (2026-10-14): MoveIt's Cartesian paths come about 0.003 rad
apart. With a 0.05 s floor per waypoint (the controller's resolution, with a
margin), a 156-point hover took 7.8 s whatever the joint rate: 0.003 / 0.09
was already under the floor, so the rate was never the binding limit. Keeping
only the waypoints that are about ``rate * min_dt`` apart makes the nominal
rate the real one. The audit still sees every planned point; the safety
envelope checks the kept ones.
"""


def retime(start, points, rate, min_dt=0.05, max_vel=0.6, thin=True):
    """Turn joint dicts into (pose, seconds-from-start) pairs at ``rate`` rad/s.

    ``start`` is the arm's present joints; the planner's own copy of it (a
    first point with no motion) is dropped, since the motion client sends the
    anchor at t=0 itself. Each kept waypoint gets the time its largest joint
    step needs at ``rate``, never less than ``min_dt``. With ``thin``, a
    waypoint closer than 0.9 x ``rate * min_dt`` to the last kept one is
    skipped; the last point is always kept, so the path ends where it was
    planned to.
    """
    rate = min(float(rate), max_vel * 0.95)
    stride = rate * min_dt
    pts, previous, t = [], start, 0.0
    for index, pose in enumerate(points):
        step = max(abs(pose[j] - previous[j]) for j in previous)
        last = index == len(points) - 1
        if step < 1e-6 and not pts:
            continue                    # the planner's copy of the start state
        if thin and not last and step < 0.9 * stride:
            continue                    # keep going: the next point is still close
        t += max(min_dt, step / rate)
        pts.append((pose, t))
        previous = pose
    return pts
