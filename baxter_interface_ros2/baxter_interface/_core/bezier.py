"""The Baxter SDK's curve through waypoints: a cubic spline made of Bezier pieces.

Ported from joint_trajectory_action/bezier.py (baxter_interface v1.2.0, Copyright (c)
2013-2015, Rethink Robotics, BSD license; see LICENSE), which says about itself: "The
Bezier library was implemented as a class project in CIS515, Fundamentals of Linear
Algebra, taught by Professor Jean Gallier in the summer of 2011 at the University of
Pennsylvania" (http://www.cis.upenn.edu/~cis515/proj1-12.pdf). The arithmetic is
unchanged, including what it does for paths of two or three waypoints; only the parts the
action server used are ported (natural end conditions), and SdkTrajectory is new: it
wraps the functions in the interface of trajectory.Trajectory.

This is the curve the SDK's joint trajectory action server moved the arms along. Select
it with `--curve sdk` on the server or `curve='sdk'` in Limb.move_through(). How it
differs from this library's own curve ("smooth", trajectory.py):

  - One spline for the whole path: every waypoint influences the curve everywhere. With
    equally spaced waypoints, speed and acceleration are continuous at the waypoints.
  - The arithmetic treats all stretches as equally long. If the waypoints' times are
    unevenly spaced, the speed steps at each waypoint.
  - "Natural" ends: the acceleration is zero at the first and last waypoint, the speed
    is not. The arm is asked to be at speed from the first moment and arrives at speed.
  - Speeds and accelerations given with the waypoints are not used for the path. Each
    gets a spline of its own, which is only passed on as feed-forward (position_w_id).

Needs NumPy.
"""

from __future__ import annotations

import bisect
from typing import Any, Sequence

import numpy as np

from .trajectory import Trajectory


def de_boor_control_pts(points_array: np.ndarray) -> np.ndarray:
    """Compute the de Boor control points of the spline through the given points.

    Parameters:
        points_array: numpy array, N+1 rows (the points) by k columns (the dimensions)

    Returns:
        numpy array of N+3 by k control points
    """
    # N+3 auxiliary points required to compute d_pts
    # dpts_(-1) = x_(0)
    # dpts_(N+1) = x_(N)
    # so it is only necessary to find N+1 pts, dpts_(0) to to dpts_(N)
    (rows, k) = np.shape(points_array)
    N = rows - 1  # minus 1 because list includes x_(0)
    # Compute A matrix (a number instead, for paths of up to three points)
    A: Any
    if N > 2:
        A = np.zeros((N - 1, N - 1))
        A[np.ix_([0], [0, 1])] = [4, 1]
        A[np.ix_([N - 2], [N - 3, N - 2])] = [1, 4]
    else:
        A = 4.0
    for i in range(1, N - 2):
        A[np.ix_([i], [i - 1, i, i + 1])] = [1, 4, 1]
    # Construct de Boor Control Points from A matrix
    d_pts = np.zeros((N + 3, k))
    for col in range(0, k):
        x = np.zeros((max(N - 1, 1), 1))
        if N > 2:
            # Compute start / end conditions
            x[N - 2, 0] = 6 * points_array[-2, col] - points_array[-1, col]
            x[0, 0] = 6 * points_array[1, col] - points_array[0, col]
            x[list(range(1, N - 3 + 1)), 0] = 6 * points_array[list(range(2, N - 2 + 1)), col]
            # Solve bezier interpolation
            d_pts[2:N + 1, col] = np.linalg.solve(A, x).T
        else:
            # Compute start / end conditions
            x[0, 0] = 6 * points_array[1, col] - points_array[0, col]
            # Solve bezier interpolation
            d_pts[2, col] = x[0, 0] / A
    # Store off start and end positions
    d_pts[0, :] = points_array[0, :]
    d_pts[-1, :] = points_array[-1, :]
    # Compute the second to last de Boor point based on end conditions
    one_third = (1.0 / 3.0)
    two_thirds = (2.0 / 3.0)
    d_pts[1, :] = two_thirds * points_array[0, :] + one_third * d_pts[2, :]
    d_pts[N + 1, :] = one_third * d_pts[-3, :] + two_thirds * points_array[-1, :]
    return d_pts


def bezier_coefficients(points_array: np.ndarray, d_pts: np.ndarray) -> np.ndarray:
    """Compute the four Bezier coefficients of every stretch between two points.

    A stretch's curve is, with t running from 0 to 1 along it:
        C(t) = (1 - t)^3*b0 + 3(1 - t)^2*t*b1 + 3(1 - t)*t^2*b2 + t^3*b3

    Parameters:
        points_array: numpy array, N+1 points by k dimensions
        d_pts:        the de Boor control points from de_boor_control_pts()

    Returns:
        numpy array of k by N by 4
    """
    (rows, k) = np.shape(points_array)
    N = rows - 1  # N minus 1 because points array includes x_0
    b_coeffs = np.zeros(shape=(k, N, 4))
    for i in range(0, N):
        points_array_i = i + 1
        d_pts_i = i + 2
        for axis_pos in range(0, k):
            b_coeffs[axis_pos, i, 0] = points_array[points_array_i - 1, axis_pos]
            b_coeffs[axis_pos, i, 3] = points_array[points_array_i, axis_pos]
            if i == 0:
                b_coeffs[axis_pos, i, 1] = d_pts[d_pts_i - 1, axis_pos]
                b_coeffs[axis_pos, i, 2] = (0.5 * d_pts[d_pts_i - 1, axis_pos]
                                            + 0.5 * d_pts[d_pts_i, axis_pos])
            elif i == N - 1:
                b_coeffs[axis_pos, i, 1] = (0.5 * d_pts[d_pts_i - 1, axis_pos]
                                            + 0.5 * d_pts[d_pts_i, axis_pos])
                b_coeffs[axis_pos, i, 2] = d_pts[d_pts_i, axis_pos]
            else:
                b_coeffs[axis_pos, i, 1] = (2.0 / 3.0 * d_pts[d_pts_i - 1, axis_pos]
                                            + 1.0 / 3.0 * d_pts[d_pts_i, axis_pos])
                b_coeffs[axis_pos, i, 2] = (1.0 / 3.0 * d_pts[d_pts_i - 1, axis_pos]
                                            + 2.0 / 3.0 * d_pts[d_pts_i, axis_pos])
    return b_coeffs


def bezier_point(b_coeffs: np.ndarray, b_index: int, t: float) -> np.ndarray:
    """Return the point on the curve in stretch `b_index` (counted from 1) at t (0 to 1).

    Parameters:
        b_coeffs: k by N by 4 array from bezier_coefficients()
        b_index:  which stretch; 0 or less gives the first point, more than N the last
        t:        how far along the stretch, 0 to 1

    Returns:
        numpy array of k values
    """
    if b_index <= 0:
        return b_coeffs[:, 0, 0]
    if b_index > b_coeffs.shape[1]:
        return b_coeffs[:, -1, -1]
    t = min(max(t, 0.0), 1.0)
    b = b_coeffs[:, b_index - 1, :]
    return ((1 - t) ** 3 * b[:, 0]
            + 3 * (1 - t) ** 2 * t * b[:, 1]
            + 3 * (1 - t) * t ** 2 * b[:, 2]
            + t ** 3 * b[:, 3])


class SdkTrajectory(Trajectory):
    """A path through waypoints along the SDK's curve; used like trajectory.Trajectory.

    As in the SDK's action server, the path is the spline through the waypoints'
    positions alone. Given velocities and accelerations each get a spline of their own,
    which sample() returns (the server passes them on as feed-forward). Where they are
    not given, sample() returns what the path itself does: its speed and acceleration.
    """

    def __init__(
        self,
        joint_names: Sequence[str],
        times: Sequence[float],
        positions: Sequence[Sequence[float]],
        velocities: Sequence[Sequence[float]] | None = None,
        accelerations: Sequence[Sequence[float]] | None = None
    ) -> None:
        """Parameters and errors as for trajectory.Trajectory."""
        super().__init__(
            joint_names,
            times,
            positions,
            velocities,
            accelerations
        )
        given: list[list[list[float]]] = [self.positions]
        if velocities is not None:
            given.append(self.velocities)
        if self.accelerations is not None:
            given.append(self.accelerations)
        self._velocity_column = 1 if velocities is not None else None
        self._acceleration_column = len(given) - 1 if accelerations is not None else None
        # One array of k by stretches by 4 coefficients per joint, where k counts what was
        # given: positions, and velocities and accelerations if any.
        self._coefficients: list[np.ndarray] = []
        if len(self.times) > 1:
            for j in range(len(self.joint_names)):
                points = np.array([[values[i][j] for values in given] for i in range(len(self.times))])
                self._coefficients.append(bezier_coefficients(points, de_boor_control_pts(points)))

    def sample(self, time: float) -> tuple[list[float], list[float], list[float]]:
        """Return angles, speeds and accelerations at a time; see Trajectory.sample()."""
        count = len(self.joint_names)
        if time <= self.times[0]:
            return list(self.positions[0]), [0.0] * count, [0.0] * count
        if time >= self.times[-1]:
            return list(self.positions[-1]), [0.0] * count, [0.0] * count

        # As the SDK's server: which stretch, and how far along it (0 to 1).
        index = bisect.bisect(self.times, time)
        length = self.times[index] - self.times[index - 1]
        t = (time - self.times[index - 1]) / length

        positions: list[float] = []
        velocities: list[float] = []
        accelerations: list[float] = []
        for coefficients in self._coefficients:
            point = bezier_point(coefficients, index, t)
            b = coefficients[0, index - 1, :]  # the stretch of the position spline
            positions.append(float(point[0]))
            if self._velocity_column is not None:
                velocities.append(float(point[self._velocity_column]))
            else:
                slope = 3 * ((1 - t) ** 2 * (b[1] - b[0]) + 2 * (1 - t) * t * (b[2] - b[1]) + t ** 2 * (b[3] - b[2]))
                velocities.append(float(slope / length))
            if self._acceleration_column is not None:
                accelerations.append(float(point[self._acceleration_column]))
            else:
                bend = 6 * ((1 - t) * (b[2] - 2 * b[1] + b[0]) + t * (b[3] - 2 * b[2] + b[1]))
                accelerations.append(float(bend / length ** 2))
        return positions, velocities, accelerations
