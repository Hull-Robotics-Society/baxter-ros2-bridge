"""Tests for the trajectory arithmetic: no robot, bridge or ROS needed."""

import pytest

from baxter_interface._core.pid import PID
from baxter_interface._core.trajectory import Trajectory, largest_error, make_trajectory, waypoint_times


def _path():
    return Trajectory(
        joint_names=['a', 'b'],
        times=[0.0, 2.0, 3.0],
        positions=[[0.0, 1.0], [1.0, 1.0], [1.5, 0.0]]
    )


def test_path_passes_through_the_waypoints():
    path = _path()

    assert path.at(0.0) == {'a': 0.0, 'b': 1.0}
    assert path.at(2.0) == pytest.approx({'a': 1.0, 'b': 1.0})
    assert path.at(3.0) == {'a': 1.5, 'b': 0.0}
    assert path.duration == 3.0


def test_path_stays_at_the_ends_outside_its_time():
    path = _path()

    assert path.at(-1.0) == path.at(0.0)
    assert path.at(99.0) == path.at(3.0)


def test_path_starts_and_ends_at_rest():
    path = _path()

    # Right after the start and right before the end, the joints have hardly moved.
    assert abs(path.at(0.01)['a'] - 0.0) < 1e-3
    assert abs(path.at(2.99)['b'] - 0.0) < 1e-3


def test_path_has_no_jumps():
    path = _path()
    step = 0.01
    previous = path.at(0.0)

    for i in range(1, 301):
        current = path.at(i * step)
        for joint in ('a', 'b'):
            assert abs(current[joint] - previous[joint]) < 0.02  # under 2 rad/s
        previous = current


def test_joint_that_keeps_going_does_not_stop_at_a_waypoint():
    path = _path()  # joint 'a' rises through the middle waypoint

    assert path.velocities[1][0] > 0.0   # 'a' passes through
    assert path.velocities[1][1] == 0.0  # 'b' is level before and falls after: it rests


def test_given_velocities_are_used():
    path = Trajectory(
        joint_names=['a'],
        times=[0.0, 1.0],
        positions=[[0.0], [1.0]],
        velocities=[[1.0], [1.0]]
    )

    assert path.at(0.5)['a'] == pytest.approx(0.5)  # a straight line at constant speed


def test_single_waypoint_is_allowed():
    path = Trajectory(['a'], [1.0], [[0.3]])

    assert path.at(0.0) == {'a': 0.3}
    assert path.duration == 0.0


@pytest.mark.parametrize('times, positions, message', [
    ([0.0, 1.0], [[0.0]], 'one time for each waypoint'),
    ([], [], 'at least one waypoint'),
    ([0.0, 1.0], [[0.0], [1.0, 2.0]], 'one value per joint'),
    ([0.0, 1.0, 1.0], [[0.0], [1.0], [2.0]], 'must increase'),
])
def test_lists_that_do_not_fit_are_reported(times, positions, message):
    with pytest.raises(ValueError, match=message):
        Trajectory(['a'], times, positions)


def test_largest_error_names_the_worst_joint():
    wanted = {'a': 1.0, 'b': 2.0, 'c': 3.0}
    actual = {'a': 1.1, 'b': 1.5}  # 'c' is not reported

    joint, error = largest_error(wanted, actual)

    assert joint == 'b'
    assert error == pytest.approx(0.5)
    assert largest_error({}, {}) == (None, 0.0)


def test_sample_gives_speeds_that_fit_the_positions():
    path = _path()
    step = 1e-4

    for moment in (0.3, 1.0, 1.9, 2.5):
        positions, velocities, accelerations = path.sample(moment)
        before, velocities_before, _ = path.sample(moment - step)
        after, velocities_after, _ = path.sample(moment + step)
        for j in range(2):
            assert velocities[j] == pytest.approx((after[j] - before[j]) / (2 * step), abs=1e-4)
            slope = (velocities_after[j] - velocities_before[j]) / (2 * step)
            assert accelerations[j] == pytest.approx(slope, abs=1e-3)


def test_sample_is_at_rest_outside_the_path():
    path = _path()

    assert path.sample(-1.0) == ([0.0, 1.0], [0.0, 0.0], [0.0, 0.0])
    assert path.sample(9.0) == ([1.5, 0.0], [0.0, 0.0], [0.0, 0.0])


def test_given_accelerations_are_met_at_the_waypoints():
    path = Trajectory(
        joint_names=['a'],
        times=[0.0, 2.0, 3.0],
        positions=[[0.0], [1.0], [1.2]],
        velocities=[[0.0], [0.5], [0.0]],
        accelerations=[[0.3], [-0.2], [0.0]]
    )
    tiny = 1e-6

    positions, velocities, accelerations = path.sample(tiny)
    assert positions[0] == pytest.approx(0.0, abs=1e-5)
    assert accelerations[0] == pytest.approx(0.3, abs=1e-4)
    for moment in (2.0 - tiny, 2.0 + tiny):   # the same from both sides: no jump
        positions, velocities, accelerations = path.sample(moment)
        assert positions[0] == pytest.approx(1.0, abs=1e-5)
        assert velocities[0] == pytest.approx(0.5, abs=1e-4)
        assert accelerations[0] == pytest.approx(-0.2, abs=1e-4)


def test_accelerations_need_velocities():
    with pytest.raises(ValueError, match='together with velocities'):
        Trajectory(['a'], [0.0, 1.0], [[0.0], [1.0]], accelerations=[[0.0], [0.0]])


def test_waypoint_times_follow_the_joint_that_travels_furthest():
    times = waypoint_times(
        positions=[[0.0, 0.0], [1.0, 0.2], [1.0, 0.3], [0.0, 0.3]],
        speed=0.5
    )

    # 1.0 rad at 0.5 rad/s = 2 s; a short hop still gets the shortest time; then 2 s again.
    assert times == pytest.approx([0.0, 2.0, 2.5, 4.5])
    with pytest.raises(ValueError, match='greater than 0'):
        waypoint_times([[0.0], [1.0]], speed=0.0)


def test_pid_proportional_part():
    pid = PID(kp=2.0)

    assert pid.compute_output(0.1) == pytest.approx(0.2)
    assert pid.compute_output(-0.3) == pytest.approx(-0.6)


def test_pid_integral_part_grows_while_the_error_lasts(monkeypatch):
    import time
    clock = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    pid = PID(ki=1.0)

    clock[0] += 0.5
    assert pid.compute_output(0.2) == pytest.approx(0.1)    # 0.2 for 0.5 s
    clock[0] += 0.5
    assert pid.compute_output(0.2) == pytest.approx(0.2)    # 0.2 for 1 s
    pid.initialize()
    clock[0] += 0.5
    assert pid.compute_output(0.2) == pytest.approx(0.1)    # forgotten


def test_pid_derivative_part_reacts_to_change(monkeypatch):
    import time
    clock = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    pid = PID(kd=1.0)

    clock[0] += 0.1
    assert pid.compute_output(0.05) == pytest.approx(0.5)   # error rose by 0.05 in 0.1 s
    clock[0] += 0.1
    assert pid.compute_output(0.05) == pytest.approx(0.0)   # no change


# --- The SDK's curve (bezier.py) ------------------------------------------------------------

def _natural_spline(values, position):
    """A natural cubic spline through equally spaced values, worked out the textbook way
    (second derivatives from a system of equations), to compare the SDK's curve against."""
    np = pytest.importorskip('numpy')
    last = len(values) - 1
    matrix = np.zeros((last - 1, last - 1))
    right = np.zeros(last - 1)
    for row in range(last - 1):
        matrix[row, row] = 4.0
        if row > 0:
            matrix[row, row - 1] = 1.0
        if row < last - 2:
            matrix[row, row + 1] = 1.0
        right[row] = 6.0 * (values[row] - 2.0 * values[row + 1] + values[row + 2])
    bends = [0.0] + list(np.linalg.solve(matrix, right)) + [0.0]
    i = min(int(position), last - 1)
    s = position - i
    return (bends[i] * (1 - s) ** 3 / 6 + bends[i + 1] * s ** 3 / 6
            + (values[i] - bends[i] / 6) * (1 - s) + (values[i + 1] - bends[i + 1] / 6) * s)


def _sdk_path():
    pytest.importorskip('numpy')
    return make_trajectory(
        'sdk',
        joint_names=['a', 'b'],
        times=[0.0, 1.0, 2.0, 3.0, 4.0],
        positions=[[0.0, 1.0], [1.0, 1.0], [0.5, 0.0], [1.5, 0.5], [1.0, 0.0]]
    )


def test_sdk_curve_passes_through_the_waypoints():
    path = _sdk_path()

    for moment, expected in zip(path.times, path.positions):
        assert path.sample(moment + 1e-9)[0] == pytest.approx(expected, abs=1e-6)
    assert path.at(4.0) == {'a': 1.0, 'b': 0.0}
    assert path.duration == 4.0


def test_sdk_curve_is_the_natural_spline_through_the_waypoints():
    path = _sdk_path()

    for moment in (0.25, 0.9, 1.5, 2.2, 3.7):
        positions = path.sample(moment)[0]
        assert positions[0] == pytest.approx(_natural_spline([0.0, 1.0, 0.5, 1.5, 1.0], moment), abs=1e-9)
        assert positions[1] == pytest.approx(_natural_spline([1.0, 1.0, 0.0, 0.5, 0.0], moment), abs=1e-9)


def test_sdk_curve_keeps_speed_and_acceleration_steady_at_evenly_timed_waypoints():
    path = _sdk_path()
    tiny = 1e-7

    for moment in (1.0, 2.0, 3.0):
        _, speed_before, bend_before = path.sample(moment - tiny)
        _, speed_after, bend_after = path.sample(moment + tiny)
        assert speed_after == pytest.approx(speed_before, abs=1e-5)
        assert bend_after == pytest.approx(bend_before, abs=1e-5)


def test_sdk_curve_starts_at_speed_without_acceleration():
    path = _sdk_path()

    _, speeds, bends = path.sample(1e-7)
    assert abs(speeds[0]) > 0.5                        # not at rest, unlike the smooth curve
    assert bends == pytest.approx([0.0, 0.0], abs=1e-5)  # "natural" end
    assert make_trajectory('smooth', ['a'], [0.0, 1.0, 2.0], [[0.0], [1.0], [0.5]]).sample(1e-7)[1] \
        == pytest.approx([0.0], abs=1e-5)


def test_sdk_curve_splines_given_speeds_on_their_own():
    pytest.importorskip('numpy')
    path = make_trajectory(
        'sdk',
        joint_names=['a'],
        times=[0.0, 1.0, 2.0, 3.0],
        positions=[[0.0], [1.0], [2.0], [3.0]],
        velocities=[[0.0], [5.0], [-5.0], [0.0]],
        accelerations=[[1.0], [2.0], [3.0], [0.0]]
    )

    positions, velocities, accelerations = path.sample(1.0 + 1e-9)
    assert positions == pytest.approx([1.0], abs=1e-6)       # the path ignores the given speeds:
    assert path.sample(1.5)[0] == pytest.approx([1.5], abs=1e-9)   # a straight line stays one
    assert velocities == pytest.approx([5.0], abs=1e-6)      # which are passed on as given
    assert accelerations == pytest.approx([2.0], abs=1e-6)


def test_sdk_curve_handles_short_paths():
    pytest.importorskip('numpy')

    two = make_trajectory('sdk', ['a'], [0.0, 2.0], [[0.0], [1.0]])
    assert two.sample(1e-9)[0] == pytest.approx([0.0], abs=1e-6)
    assert two.sample(2.0 - 1e-9)[0] == pytest.approx([1.0], abs=1e-6)
    assert 0.0 < two.at(1.0)['a'] < 1.0
    three = make_trajectory('sdk', ['a'], [0.0, 1.0, 2.0], [[0.0], [1.0], [0.0]])
    assert three.sample(1.0 + 1e-9)[0] == pytest.approx([1.0], abs=1e-6)
    one = make_trajectory('sdk', ['a'], [1.0], [[0.3]])
    assert one.at(5.0) == {'a': 0.3}


def test_unknown_curve_is_refused():
    with pytest.raises(ValueError, match="curve must be one of"):
        make_trajectory('wobbly', ['a'], [0.0, 1.0], [[0.0], [1.0]])
