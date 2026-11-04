"""A smooth path through several arm postures: the arithmetic, without any ROS.

A trajectory is a list of waypoints, each with a time: "be at these joint angles 2 s after
the start, at those after 3.5 s". Between the waypoints the path is filled in with a
smooth curve (one polynomial piece per stretch, joined so that the speed never jumps), so
that the arm can be sent a new target 100 times per second and moves without jerks.

Used by Limb.move_through() and by the joint trajectory action server
(tools/joint_trajectory_action_server.py).

    path = Trajectory(
        joint_names=['left_s0', 'left_s1'],
        times=[0.0, 2.0, 3.5],
        positions=[[0.0, 0.0], [0.5, -0.2], [0.5, 0.3]]
    )
    path.at(1.0)        # {'left_s0': ..., 'left_s1': ...} one second after the start

The Baxter SDK's action server used another curve (a cubic B-spline through the
waypoints). This one is new: its pieces are fixed by position and speed (and
acceleration, if given) at their two ends, which makes a path without given speeds start
and end at rest. The SDK's curve is available as well (bezier.py); make_trajectory()
builds either.
"""

from __future__ import annotations

from typing import Mapping, Sequence

# The curves a path can follow: this library's own, and the Baxter SDK's.
CURVES = ('smooth', 'sdk')

# The curve of one stretch is a weighted sum of the values at its two ends. Each weight is
# a polynomial in s, which runs from 0 at the stretch's start to 1 at its end; the tuples
# hold the polynomials' coefficients for s^0, s^1, s^2, ...
#
# Cubic ("Hermite") curve: fixed by position and speed at both ends.
_CUBIC = {
    'start': (1.0, 0.0, -3.0, 2.0),
    'start_speed': (0.0, 1.0, -2.0, 1.0),
    'end': (0.0, 0.0, 3.0, -2.0),
    'end_speed': (0.0, 0.0, -1.0, 1.0),
}
# Quintic curve: fixed by position, speed and acceleration at both ends.
_QUINTIC = {
    'start': (1.0, 0.0, 0.0, -10.0, 15.0, -6.0),
    'start_speed': (0.0, 1.0, 0.0, -6.0, 8.0, -3.0),
    'start_acceleration': (0.0, 0.0, 0.5, -1.5, 1.5, -0.5),
    'end': (0.0, 0.0, 0.0, 10.0, -15.0, 6.0),
    'end_speed': (0.0, 0.0, 0.0, -4.0, 7.0, -3.0),
    'end_acceleration': (0.0, 0.0, 0.0, 0.5, -1.0, 0.5),
}


def _polynomial(coefficients: Sequence[float], s: float, derivative: int) -> float:
    """Value of a polynomial's derivative (0 = the polynomial itself) at s."""
    total = 0.0
    for power, coefficient in enumerate(coefficients):
        if power < derivative:
            continue
        factor = 1.0
        for k in range(derivative):
            factor *= power - k
        total += coefficient * factor * s ** (power - derivative)
    return total


class Trajectory:
    """A smooth path through waypoints, one value per joint."""

    def __init__(
        self,
        joint_names: Sequence[str],
        times: Sequence[float],
        positions: Sequence[Sequence[float]],
        velocities: Sequence[Sequence[float]] | None = None,
        accelerations: Sequence[Sequence[float]] | None = None
    ) -> None:
        """
        Parameters:
            joint_names:   the joints, in the order of the values in each waypoint
            times:         seconds from the start at which each waypoint is reached;
                           must increase from one waypoint to the next
            positions:     one list of joint angles (rad) per waypoint
            velocities:    optional: one list of joint speeds (rad/s) per waypoint. Left
                           out, the speeds are chosen so that the arm starts and ends at
                           rest and passes the waypoints in between without stopping.
            accelerations: optional, only together with velocities: one list of joint
                           accelerations (rad/s²) per waypoint

        Raises:
            ValueError if the lists don't fit together.
        """
        self.joint_names = list(joint_names)
        self.times = [float(t) for t in times]
        self.positions = [[float(p) for p in point] for point in positions]

        if len(self.times) != len(self.positions) or len(self.times) == 0:
            raise ValueError('A trajectory needs one time for each waypoint, and at least one waypoint')
        if any(len(point) != len(self.joint_names) for point in self.positions):
            raise ValueError('Each waypoint needs one value per joint')
        if any(later <= earlier for earlier, later in zip(self.times, self.times[1:])):
            raise ValueError('The times of the waypoints must increase')

        if velocities is None:
            self.velocities = self._smooth_velocities()
        else:
            self.velocities = self._checked(velocities, 'Velocities')
        self.accelerations: list[list[float]] | None
        if accelerations is None:
            self.accelerations = None
        elif velocities is None:
            raise ValueError('Accelerations can only be given together with velocities')
        else:
            self.accelerations = self._checked(accelerations, 'Accelerations')

    def _checked(self, values: Sequence[Sequence[float]], what: str) -> list[list[float]]:
        checked = [[float(v) for v in point] for point in values]
        if (len(checked) != len(self.positions)
                or any(len(point) != len(self.joint_names) for point in checked)):
            raise ValueError(f'{what} need one value per joint and waypoint')
        return checked

    @property
    def duration(self) -> float:
        """Seconds from the first waypoint to the last."""
        return self.times[-1] - self.times[0]

    def _smooth_velocities(self) -> list[list[float]]:
        """Choose the speed at each waypoint: at rest at both ends, flowing in between."""
        count = len(self.times)
        velocities = [[0.0] * len(self.joint_names) for _ in range(count)]
        for i in range(1, count - 1):
            before = self.times[i] - self.times[i - 1]
            after = self.times[i + 1] - self.times[i]
            for j in range(len(self.joint_names)):
                slope_in = (self.positions[i][j] - self.positions[i - 1][j]) / before
                slope_out = (self.positions[i + 1][j] - self.positions[i][j]) / after
                # Where the joint turns round at this waypoint, it stops there. Otherwise
                # it passes through at the average of the speed before and after.
                if slope_in * slope_out > 0:
                    velocities[i][j] = (slope_in + slope_out) / 2.0
        return velocities

    def at(self, time: float) -> dict[str, float]:
        """Return the joint angles on the path at a time.

        Parameters:
            time: seconds from the start. Before the first waypoint the path stays at
                  the first waypoint, after the last one at the last.

        Returns:
            dict {joint name: angle}
        """
        return dict(zip(self.joint_names, self.sample(time)[0]))

    def sample(self, time: float) -> tuple[list[float], list[float], list[float]]:
        """Return angles, speeds and accelerations on the path at a time.

        Parameters:
            time: seconds from the start (see at())

        Returns:
            (positions, velocities, accelerations): three lists in the order of
            joint_names, in rad, rad/s and rad/s². Outside the path's time the arm is
            at rest at the first or last waypoint.
        """
        count = len(self.joint_names)
        if time <= self.times[0]:
            return list(self.positions[0]), [0.0] * count, [0.0] * count
        if time >= self.times[-1]:
            return list(self.positions[-1]), [0.0] * count, [0.0] * count

        # The stretch between two waypoints that this time falls into.
        i = 0
        while self.times[i + 1] < time:
            i += 1
        length = self.times[i + 1] - self.times[i]
        s = (time - self.times[i]) / length  # 0 at the stretch's start, 1 at its end

        result: list[list[float]] = []
        for derivative in (0, 1, 2):
            # The speeds and accelerations at the ends are per second, the curve runs over
            # s: one factor of the stretch's length converts each. The same factor comes
            # back once per derivative.
            if self.accelerations is None:
                weights = [
                    (_CUBIC['start'], self.positions[i]),
                    (_CUBIC['start_speed'], [v * length for v in self.velocities[i]]),
                    (_CUBIC['end'], self.positions[i + 1]),
                    (_CUBIC['end_speed'], [v * length for v in self.velocities[i + 1]]),
                ]
            else:
                weights = [
                    (_QUINTIC['start'], self.positions[i]),
                    (_QUINTIC['start_speed'], [v * length for v in self.velocities[i]]),
                    (_QUINTIC['start_acceleration'], [a * length ** 2 for a in self.accelerations[i]]),
                    (_QUINTIC['end'], self.positions[i + 1]),
                    (_QUINTIC['end_speed'], [v * length for v in self.velocities[i + 1]]),
                    (_QUINTIC['end_acceleration'], [a * length ** 2 for a in self.accelerations[i + 1]]),
                ]
            values = [0.0] * count
            for coefficients, ends in weights:
                weight = _polynomial(coefficients, s, derivative)
                for j in range(count):
                    values[j] += weight * ends[j]
            result.append([value / length ** derivative for value in values])
        return result[0], result[1], result[2]


def make_trajectory(
    curve: str,
    joint_names: Sequence[str],
    times: Sequence[float],
    positions: Sequence[Sequence[float]],
    velocities: Sequence[Sequence[float]] | None = None,
    accelerations: Sequence[Sequence[float]] | None = None
) -> Trajectory:
    """Build a path along one of the two curves.

    Parameters:
        curve: 'smooth' (this library's curve, the Trajectory class above) or 'sdk' (the
               Baxter SDK's spline, bezier.SdkTrajectory)
        the others: as for Trajectory

    Returns:
        an object with joint_names, positions, duration, at(time) and sample(time)

    Raises:
        ValueError for an unknown curve, or lists that don't fit together.
    """
    kind: type[Trajectory]
    if curve == 'smooth':
        kind = Trajectory
    elif curve == 'sdk':
        from .bezier import SdkTrajectory  # needs NumPy, so only loaded when asked for
        kind = SdkTrajectory
    else:
        raise ValueError(f"curve must be one of {CURVES}, not {curve!r}")
    return kind(
        joint_names,
        times,
        positions,
        velocities,
        accelerations
    )


def waypoint_times(positions: Sequence[Sequence[float]], speed: float, shortest: float = 0.5) -> list[float]:
    """Choose a time for each waypoint from how far the joints have to travel.

    Parameters:
        positions: one list of joint angles (rad) per waypoint, the first being the start
        speed:     average speed (rad/s) of the joint that has furthest to go on a stretch
        shortest:  no stretch gets less than this many seconds (default 0.5)

    Returns:
        list of seconds from the start, one per waypoint, the first being 0.0
    """
    if speed <= 0:
        raise ValueError('speed must be greater than 0')
    times = [0.0]
    for before, after in zip(positions, positions[1:]):
        furthest = max(abs(b - a) for a, b in zip(before, after))
        times.append(times[-1] + max(furthest / speed, shortest))
    return times


def largest_error(wanted: Mapping[str, float], actual: Mapping[str, float]) -> tuple[str | None, float]:
    """Return the joint that is furthest from where it should be.

    Parameters:
        wanted: dict {joint name: angle} the path asks for
        actual: dict {joint name: angle} the arm reports

    Returns:
        (joint name, error in rad); joints missing from `actual` are left out.
        (None, 0.0) if there is nothing to compare.
    """
    worst: str | None = None
    error = 0.0
    for joint, angle in wanted.items():
        if joint in actual and abs(angle - actual[joint]) >= error:
            worst, error = joint, abs(angle - actual[joint])
    return worst, error
