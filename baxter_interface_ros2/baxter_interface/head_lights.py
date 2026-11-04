"""The lights on Baxter's head: the red and green halo, and the twelve sonar lights.

New in the ROS 2 library; the Baxter SDK had no class for these, only the raw topics.

    from baxter_interface import HeadLights

    lights = HeadLights()
    lights.set_halo(red=100, green=0)         # the halo at the top of the head: red
    lights.set_sonar_lights([0, 3, 7])        # three of the yellow lights on, the rest off
    lights.release_sonar_lights()             # hand them back to the robot

The halo is two lights, one red and one green, each with a brightness from 0 to 100;
mixed, they give colours in between (both at 100: yellow-orange).

The sonar lights are twelve yellow lights, one around each sonar sensor on the ring
around the head, numbered 0 to 11 like the sensors. By default the robot switches each
one on while its sensor sees something near. The robot only lets a program hold them
for as long as it keeps saying so 100 times per second; this class does that in the
background from set_sonar_lights() until release_sonar_lights() or the program's end.

How the lights are commanded is taken from Rethink's API reference
(https://github.com/RethinkRobotics/sdk-docs/wiki/API-Reference, "Lights"), the topic
names from the robot inventory. Not tried on the robot yet (written 6 Oct 2026).
"""

from __future__ import annotations

import threading
import time
from typing import Iterable, NamedTuple

from rclpy.node import Node
from ._core.node import ros_is_running
from std_msgs.msg import Float32, UInt16

from . import settings
from ._core import connecting, dataflow
from ._core.node import get_node

LIGHTS = 12                 # sonar lights, numbered 0 to 11
_ALL = (1 << LIGHTS) - 1    # 0x0fff: one bit per light
_OVERRIDE = 0x8000          # "the program decides", added to the bits of the lights
_RATE = 100.0               # how often the robot wants to hear it, per second

_NS = 'robot/sonar/head_sonar/lights/'


def lights_to_number(lights: Iterable[int] | str) -> int:
    """Turn light numbers into the number the robot takes: one bit per light.

    Parameters:
        lights: the lights that should be on: numbers from 0 to 11, or 'all'

    Returns:
        int, without the override flag

    Raises:
        ValueError for anything that is not a light number.
    """
    if isinstance(lights, str):
        if lights != 'all':
            raise ValueError(f"lights must be numbers from 0 to {LIGHTS - 1} or 'all', not {lights!r}")
        return _ALL
    number = 0
    for light in lights:
        if isinstance(light, bool) or not isinstance(light, int) or not 0 <= light < LIGHTS:
            raise ValueError(f'{light!r} is not a sonar light: they are numbered 0 to {LIGHTS - 1}')
        number |= 1 << light
    return number


def number_to_lights(number: int) -> list[int]:
    """Turn the robot's number back into the list of lights that are on."""
    return [light for light in range(LIGHTS) if number & (1 << light)]


class HeadLights:
    """Interface class for the halo and the sonar lights on Baxter's head.

    Available methods:
        halo()                   - (red, green) brightness of the halo, 0 to 100
        set_halo(red, green)     - set one or both
        sonar_lights()           - numbers of the sonar lights that are on right now
        set_sonar_lights(lights) - take the sonar lights over: these on, the others off
        release_sonar_lights()   - hand the sonar lights back to the robot
        holding_sonar_lights()   - True while this object holds them
    """

    class Halo(NamedTuple):
        """Brightness of the halo's two lights, each 0.0 (off) to 100.0 (full)."""

        red: float
        green: float

    def __init__(self, node: Node | None = None) -> None:
        """
        Parameters:
            node: optional ROS 2 node to use (default: the shared background node)
        """
        self._node = get_node(node)
        self._lock = threading.Lock()  # guards what the background thread sends
        self._red: float | None = None
        self._green: float | None = None
        self._state: int | None = None
        self._held: int | None = None  # the number being sent while the lights are held
        self._sender: threading.Thread | None = None
        self._fresh = dataflow.Freshness('head lights', dataflow.full_name(self._node, _NS + 'state'))

        self._pub_red = self._node.create_publisher(
            Float32,
            _NS + 'set_red_level',
            10
        )
        self._pub_green = self._node.create_publisher(
            Float32,
            _NS + 'set_green_level',
            10
        )
        self._pub_lights = self._node.create_publisher(
            UInt16,
            _NS + 'set_lights',
            1  # only the newest command matters
        )
        self._subs = [
            self._node.create_subscription(
                Float32,
                _NS + 'red_level',
                self._on_red,
                1
            ),
            self._node.create_subscription(
                Float32,
                _NS + 'green_level',
                self._on_green,
                1
            ),
            self._node.create_subscription(
                UInt16,
                _NS + 'state',
                self._on_state,
                1
            ),
        ]
        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        dataflow.wait_for(
            lambda: None not in (self._red, self._green, self._state),
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Failed to get the state of the head lights from /{_NS}state, '
                        'red_level and green_level. ' + dataflow.BRIDGE_HINT,
        )

    def _on_red(self, msg: Float32) -> None:
        self._red = float(msg.data)

    def _on_green(self, msg: Float32) -> None:
        self._green = float(msg.data)

    def _on_state(self, msg: UInt16) -> None:
        self._fresh.mark()
        self._state = int(msg.data)

    # --- Halo ------------------------------------------------------------------------------

    def halo(self) -> HeadLights.Halo:
        """Return the brightness of the halo's two lights.

        Returns:
            HeadLights.Halo(red, green), each from 0.0 (off) to 100.0 (full)
        """
        self._fresh.require()
        return self.Halo(
            connecting.connected(self._red, 'the halo'),
            connecting.connected(self._green, 'the halo')
        )

    def set_halo(self, red: float | None = None, green: float | None = None) -> None:
        """Set the brightness of the halo's red and green light.

            lights.set_halo(red=100, green=0)     # red
            lights.set_halo(green=100)            # green up, red as it is
            lights.set_halo(0, 0)                 # off

        Parameters:
            red, green: 0 (off) to 100 (full). One that is left out stays as it is.

        Raises:
            ValueError for a value outside 0 to 100; ConnectionError if the command
            could not be delivered.
        """
        for name, level, publisher in (('red', red, self._pub_red), ('green', green, self._pub_green)):
            if level is None:
                continue
            if not 0 <= level <= 100:
                raise ValueError(f'{name} must be between 0 and 100, not {level!r}')
            dataflow.publish_once(
                publisher,
                Float32(data=float(level)),
                what=f'{name} level of the halo'
            )

    # --- Sonar lights ----------------------------------------------------------------------

    def sonar_lights(self) -> list[int]:
        """Return which sonar lights are on right now, whoever switched them.

        Returns:
            list of light numbers (0 to 11)
        """
        self._fresh.require()
        return number_to_lights(connecting.connected(self._state, 'the sonar lights'))

    def holding_sonar_lights(self) -> bool:
        """Return True while this object holds the sonar lights (see set_sonar_lights)."""
        return self._held is not None

    def set_sonar_lights(self, lights: Iterable[int] | str) -> None:
        """Take the sonar lights over: switch these on and all others off.

            lights.set_sonar_lights([0, 1, 2])    # three next to each other
            lights.set_sonar_lights('all')
            lights.set_sonar_lights([])           # all off (and still held)

        The lights stay as set until the next call, release_sonar_lights(), or the end
        of the program; then the robot switches them by its sonar again.

        Parameters:
            lights: numbers of the lights to switch on (0 to 11), or 'all'

        Raises:
            ValueError for a wrong light number; ConnectionError if the command cannot
            be delivered.
        """
        number = lights_to_number(lights) | _OVERRIDE
        if not dataflow.wait_for_listener(self._pub_lights):
            raise ConnectionError(
                f'The sonar lights cannot be set: nothing listens on {self._pub_lights.topic_name}. '
                + dataflow.BRIDGE_HINT)
        with self._lock:
            self._held = number
            if self._sender is None:
                self._sender = threading.Thread(
                    target=self._keep_sending,
                    daemon=True  # ends with the program; the lights then go back to the robot
                )
                self._sender.start()

    def release_sonar_lights(self) -> None:
        """Hand the sonar lights back to the robot (it switches them by its sonar again)."""
        with self._lock:
            was_held = self._held is not None
            self._held = None
        if was_held:
            # Say so once, instead of leaving the robot to notice that the commands stopped.
            self._pub_lights.publish(UInt16(data=0))

    def _keep_sending(self) -> None:
        # The robot hands the lights back to its sonar when it has not heard from the
        # program for a moment, so the command is repeated at a steady beat.
        period = 1.0 / _RATE
        next_beat = time.monotonic()
        while ros_is_running():
            with self._lock:
                number = self._held
                if number is None:
                    # Decided under the lock, so that a set_sonar_lights() right after
                    # this starts a new thread instead of relying on this one.
                    self._sender = None
                    return
            try:
                self._pub_lights.publish(UInt16(data=number))
            except Exception:  # the program is shutting down
                return
            next_beat += period
            pause = next_beat - time.monotonic()
            if pause > 0:
                time.sleep(pause)
            else:
                next_beat = time.monotonic()
