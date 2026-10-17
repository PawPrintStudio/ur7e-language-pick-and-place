"""retime(): thinning and timing of a planned joint path. No ROS; run with pytest."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from retime import retime  # noqa: E402

J = ['a', 'b', 'c']


def path(steps, per_step):
    """A straight joint path: ``steps`` points, joint a moving ``per_step`` rad each."""
    return [{'a': per_step * i, 'b': 0.0, 'c': 0.0} for i in range(steps + 1)]


def velocities(start, pts):
    previous, t0, out = start, 0.0, []
    for pose, t in pts:
        out.append(max(abs(pose[j] - previous[j]) for j in J) / (t - t0))
        previous, t0 = pose, t
    return out


def test_dense_cartesian_path_is_thinned_to_the_rate():
    start = path(0, 0)[0]
    dense = path(156, 0.003)                       # 0.468 rad, like the e4 hover
    pts = retime(start, dense, rate=0.4, ramp=0)
    assert 20 <= len(pts) <= 30                    # about every 7th point kept
    assert pts[-1][0] == dense[-1]                 # ends where it was planned to
    assert pts[-1][1] == pytest.approx(0.468 / 0.4, rel=0.15)
    assert max(velocities(start, pts)) <= 0.4 + 1e-9


def test_segments_start_and_stop_gently():
    start = path(0, 0)[0]
    pts = retime(start, path(156, 0.003), rate=0.4)
    v = velocities(start, pts)
    assert v[0] < 0.5 * max(v) and v[-1] < 0.5 * max(v)      # slow at both ends
    assert v[0] < v[2] < v[5] <= max(v) + 1e-9                 # ramping up
    assert max(v) <= 0.4 + 1e-9
    assert pts[-1][1] < 2.0 * 0.468 / 0.4                      # the ramp costs well under 2x


def test_without_thinning_the_floor_dominates_as_before():
    start = path(0, 0)[0]
    dense = path(156, 0.003)
    pts = retime(start, dense, rate=0.09, thin=False, ramp=0)
    assert len(pts) == 156 and pts[-1][1] == pytest.approx(156 * 0.05)


def test_the_old_rate_also_gains_from_thinning():
    start = path(0, 0)[0]
    pts = retime(start, path(156, 0.003), rate=0.09, ramp=0)
    assert pts[-1][1] < 156 * 0.05
    assert max(velocities(start, pts)) <= 0.09 + 1e-9


def test_the_rate_is_capped_by_the_envelope():
    start = path(0, 0)[0]
    pts = retime(start, path(10, 0.05), rate=5.0, max_vel=0.6, ramp=0)
    assert max(velocities(start, pts)) <= 0.6 * 0.95 + 1e-9


def test_the_planners_copy_of_the_start_is_dropped_and_sparse_paths_are_untouched():
    start = {'a': 0.0, 'b': 0.0, 'c': 0.0}
    sparse = [dict(start)] + path(9, 0.03)[1:]   # joint-line plan: 0.03 rad steps
    pts = retime(start, sparse, rate=0.4, ramp=0)
    assert len(pts) == 9                           # the start copy is gone, nothing thinned
    assert pts[0][1] == pytest.approx(0.03 / 0.4)


def test_a_path_that_ends_where_it_starts_still_keeps_its_last_point():
    start = {'a': 0.0, 'b': 0.0, 'c': 0.0}
    there_and_back = path(5, 0.01) + list(reversed(path(5, 0.01)[:-1]))
    pts = retime(start, there_and_back, rate=0.4)
    assert pts[-1][0] == start and len(pts) >= 2
