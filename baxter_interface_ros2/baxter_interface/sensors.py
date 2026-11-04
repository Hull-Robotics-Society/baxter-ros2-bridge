"""Baxter's distance and motion sensors: sonar ring, hand range sensors, accelerometers.

New in the ROS 2 library; the Baxter SDK had no classes for these, only the raw topics.

    Sonar()               twelve ultrasound sensors in a ring around the head: how far away
                          are things (people) around the robot?
    RangeSensor('left')   the infrared sensor in a hand: how far is the thing right in
                          front of the gripper (4 to 40 cm)?
    Accelerometer('left') the accelerometer in a hand: which way is down, is the hand
                          being shaken or bumped?

    from baxter_interface import RangeSensor

    sensor = RangeSensor('left')
    print(sensor.distance())      # e.g. 0.12 (metres), or None if nothing is in range

Not tried on the robot yet (written 6 Oct 2026 from the robot inventory, which recorded
these topics and what their messages look like).
"""

from __future__ import annotations

import threading
from typing import NamedTuple

from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud, Range

from . import settings
from ._core import connecting, dataflow
from ._core.node import get_node
from ._core.signals import Signal


class Sonar:
    """The ring of twelve sonar sensors around Baxter's head.

    Each sensor that hears an echo reports how far away the thing is. Sensors that hear
    nothing report nothing, so the answers hold only the sensors that see something.

    Available methods:
        distances() - {sensor number: metres} for every sensor that sees something
        closest()   - (sensor number, metres) of the nearest thing, or None
        points()    - where the echoes are, as (x, y, z) in metres in the robot's frame
    Signals (see _core/signals.py):
        changed - called with distances() every time the sonar reports (about 40 times
                  per second); a slow function gets the newest report each time
    """

    class Point(NamedTuple):
        """A place in metres: x forward, y to the robot's left, z up."""

        x: float
        y: float
        z: float

    def __init__(self, node: Node | None = None) -> None:
        """
        Parameters:
            node: optional ROS 2 node to use (default: the shared background node)
        """
        self._node = get_node(node)
        self._lock = threading.Lock()  # guards the stored values
        self._distances: dict[int, float] | None = None
        self._points: dict[int, Sonar.Point] = {}
        self._fresh = dataflow.Freshness('sonar', dataflow.full_name(self._node, 'robot/sonar/head_sonar/state'))
        self.changed: Signal[dict[int, float]] = Signal(keep='latest')

        self._sub = self._node.create_subscription(
            PointCloud,
            'robot/sonar/head_sonar/state',
            self._on_state,
            1  # only the newest report is of interest
        )
        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        dataflow.wait_for(
            lambda: self._distances is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg='Failed to get the sonar state from /robot/sonar/head_sonar/state. '
                        + dataflow.BRIDGE_HINT,
        )

    def _on_state(self, msg: PointCloud) -> None:
        # The message lists only the sensors that hear an echo: one point per sensor, and
        # two value lists of the same length, "SensorId" and "Distance".
        channels = {channel.name: channel.values for channel in msg.channels}
        ids = [int(value) for value in channels.get('SensorId', [])]
        distances = dict(zip(ids, (float(d) for d in channels.get('Distance', []))))
        points = {
            sensor: self.Point(point.x, point.y, point.z)
            for sensor, point in zip(ids, msg.points)
        }
        self._fresh.mark()
        with self._lock:
            self._distances = distances
            self._points = points
        self.changed(dict(distances))

    def distances(self) -> dict[int, float]:
        """Return how far away things are, for every sensor that sees something.

        Returns:
            dict {sensor number (0 to 11): distance in metres}; empty if no sensor
            hears an echo

        Raises:
            ConnectionError if the sonar has not reported for a second.
        """
        self._fresh.require()
        with self._lock:
            return dict(connecting.connected(self._distances, 'the sonar'))

    def closest(self) -> tuple[int, float] | None:
        """Return the nearest thing any sensor sees.

        Returns:
            (sensor number, distance in metres), or None if no sensor sees anything
        """
        distances = self.distances()
        if not distances:
            return None
        sensor = min(distances, key=lambda number: distances[number])
        return sensor, distances[sensor]

    def points(self) -> dict[int, Sonar.Point]:
        """Return where the echoes are.

        Returns:
            dict {sensor number: Sonar.Point(x, y, z)} in metres in the robot's frame
            ("base": x forward, y to the robot's left, z up)
        """
        self._fresh.require()
        with self._lock:
            return dict(self._points)


class RangeSensor:
    """The infrared distance sensor in one of Baxter's hands.

    It looks along the gripper and measures 4 to 40 cm.

    Available methods:
        distance() - metres to the thing in front of the hand, or None if nothing is in range
        min_range, max_range - the limits of the sensor in metres
    Signals (see _core/signals.py):
        changed - called with distance() every time the sensor reports; a slow function
                  gets the newest value each time
    """

    def __init__(self, limb: str, node: Node | None = None) -> None:
        """
        Parameters:
            limb: 'left' or 'right'
            node: optional ROS 2 node to use (default: the shared background node)
        """
        if limb not in ('left', 'right'):
            raise ValueError(f"limb must be 'left' or 'right', not {limb!r}")
        self.name = limb
        self._node = get_node(node)
        self._topic = dataflow.full_name(self._node, f'robot/range/{limb}_hand_range/state')
        self._msg: Range | None = None
        self._fresh = dataflow.Freshness(f'{limb} hand range sensor', self._topic)
        self.changed: Signal[float | None] = Signal(keep='latest')

        self._sub = self._node.create_subscription(
            Range,
            self._topic,
            self._on_state,
            1
        )
        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        dataflow.wait_for(
            lambda: self._msg is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Failed to get the range sensor state from {self._topic}. '
                        + dataflow.BRIDGE_HINT,
        )

    def _on_state(self, msg: Range) -> None:
        self._fresh.mark()
        self._msg = msg
        self.changed(self._in_range(msg))

    @staticmethod
    def _in_range(msg: Range) -> float | None:
        # With nothing in range the sensor reports a value far above its maximum (65.5).
        if msg.min_range <= msg.range <= msg.max_range:
            return float(msg.range)
        return None

    def distance(self) -> float | None:
        """Return the distance to the thing in front of the hand.

        Returns:
            float: metres, or None if nothing is within the sensor's range

        Raises:
            ConnectionError if the sensor has not reported for a second.
        """
        self._fresh.require()
        return self._in_range(self._newest())

    def _newest(self) -> Range:
        # The newest message; an error instead of None if none has arrived yet.
        return connecting.connected(self._msg, f'the {self.name} hand range sensor')

    @property
    def min_range(self) -> float:
        """The shortest distance the sensor can measure, in metres (0.004)."""
        return float(self._newest().min_range)

    @property
    def max_range(self) -> float:
        """The longest distance the sensor can measure, in metres (0.4)."""
        return float(self._newest().max_range)


class Accelerometer:
    """The accelerometer in one of Baxter's hands.

    A hand at rest measures gravity: about 9.8 m/s² pointing "up" as the hand sees it.
    Moving, shaking or bumping the hand adds to that.

    Available methods:
        acceleration() - (x, y, z) in m/s², in the hand's own directions
        strength()     - the size of that acceleration in m/s² (about 9.8 at rest)
    Signals (see _core/signals.py):
        changed - called with acceleration() every time the sensor reports; a slow
                  function gets the newest value each time
    """

    class Vector(NamedTuple):
        """An acceleration in m/s², along the hand's own three directions."""

        x: float
        y: float
        z: float

    def __init__(self, limb: str, node: Node | None = None) -> None:
        """
        Parameters:
            limb: 'left' or 'right'
            node: optional ROS 2 node to use (default: the shared background node)
        """
        if limb not in ('left', 'right'):
            raise ValueError(f"limb must be 'left' or 'right', not {limb!r}")
        self.name = limb
        self._node = get_node(node)
        self._topic = dataflow.full_name(self._node, f'robot/accelerometer/{limb}_accelerometer/state')
        self._value: Accelerometer.Vector | None = None
        self._fresh = dataflow.Freshness(f'{limb} accelerometer', self._topic)
        self.changed: Signal[Accelerometer.Vector] = Signal(keep='latest')

        self._sub = self._node.create_subscription(
            Imu,
            self._topic,
            self._on_state,
            1
        )
        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        dataflow.wait_for(
            lambda: self._value is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Failed to get the accelerometer state from {self._topic}. '
                        + dataflow.BRIDGE_HINT,
        )

    def _on_state(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        self._fresh.mark()
        self._value = self.Vector(float(a.x), float(a.y), float(a.z))
        self.changed(self._value)

    def acceleration(self) -> Accelerometer.Vector:
        """Return the acceleration the hand feels.

        Returns:
            Accelerometer.Vector(x, y, z) in m/s², in the hand's own directions

        Raises:
            ConnectionError if the sensor has not reported for a second.
        """
        self._fresh.require()
        return connecting.connected(self._value, f'the {self.name} accelerometer')

    def strength(self) -> float:
        """Return the size of the acceleration in m/s² (about 9.8 when the hand is at rest)."""
        a = self.acceleration()
        return (a.x ** 2 + a.y ** 2 + a.z ** 2) ** 0.5
