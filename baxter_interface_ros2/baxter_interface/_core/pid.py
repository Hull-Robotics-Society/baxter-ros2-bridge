"""A PID controller: turns "how far off is the joint" into "how fast should it move".

Ported from baxter_control.PID (baxter_interface v1.2.0, Copyright (c) 2013-2015, Rethink
Robotics, BSD license; see LICENSE). Change: wall-clock time instead of ROS time.

Used by the joint trajectory action server in its velocity mode.
"""

from __future__ import annotations

import time


class PID:
    """Proportional, integral, derivative controller for one value.

        pid = PID(kp=2.0)
        speed = pid.compute_output(wanted_angle - actual_angle)   # call at a steady rate
    """

    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0) -> None:
        """
        Parameters:
            kp: proportional gain: output per unit of error
            ki: integral gain: output per unit of error that has lasted a second
            kd: derivative gain: output per unit of error change per second
        """
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self.initialize()

    def initialize(self) -> None:
        """Forget the past: call before each new run."""
        self._prev_time = time.monotonic()
        self._prev_err = 0.0
        self._ci = 0.0

    def set_kp(self, value: float) -> None:
        self._kp = value

    def set_ki(self, value: float) -> None:
        self._ki = value

    def set_kd(self, value: float) -> None:
        self._kd = value

    def compute_output(self, error: float) -> float:
        """Return the control value for the current error.

        Parameters:
            error: wanted value minus actual value
        """
        now = time.monotonic()
        dt = now - self._prev_time
        de = error - self._prev_err

        self._ci += error * dt
        cd = de / dt if dt > 0 else 0.0

        self._prev_time = now
        self._prev_err = error
        return self._kp * error + self._ki * self._ci + self._kd * cd
