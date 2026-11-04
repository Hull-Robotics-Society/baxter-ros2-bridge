"""Read a navigator (scroll wheel and buttons on the arms and torso) and set its lights.

Ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink Robotics, BSD license;
see LICENSE). Same methods and behaviour; ROS 1 calls replaced by ROS 2 ones.
ITBState.innerLight / outerLight are called inner_light / outer_light in ROS 2 (the
bridge maps them).

Needs these topics in bridge_config/topics.yaml, for each navigator used:
    /robot/navigators/<location>_navigator/state
    /robot/digital_io/<location>_inner_light/state
    /robot/digital_io/<location>_outer_light/state
    /robot/digital_io/command
These are the names of robot software 1.2.0. One difference from the 1.2.0 SDK:
wheel_changed passes the new wheel position (the 1.2.0 SDK passed the change).
"""

from __future__ import annotations

from rclpy.node import Node

from baxter_core_msgs.msg import NavigatorState

from .. import settings
from .._core import connecting, dataflow
from .._core.node import get_node
from .digital_io import DigitalIO
from .._core.signals import Signal


class Navigator:
    """Interface class for a navigator on the Baxter robot.

    Inputs:
        button0      - press wheel
        button1      - above wheel
        button2      - below wheel
        wheel        - scroll wheel, 0-255
    Outputs:
        inner_led
        outer_led
    Signals (see _core/signals.py):
        button0_changed - True/False
        button1_changed - True/False
        button2_changed - True/False
        wheel_changed   - new wheel value
    Valid locations:
        left, right, torso_left, torso_right
    """

    _LOCATIONS = ('left', 'right', 'torso_left', 'torso_right')

    def __init__(self, location: str, node: Node | None = None) -> None:
        """
        Parameters:
            location: 'left', 'right', 'torso_left' or 'torso_right'
            node:     optional ROS 2 node to use (default: the shared background node)
        """
        if location not in self._LOCATIONS:
            raise ValueError(f"Invalid Navigator name '{location}'")
        self._id = location
        self._node = get_node(node)
        self._state: NavigatorState | None = None

        self.button0_changed: Signal[bool] = Signal()
        self.button1_changed: Signal[bool] = Signal()
        self.button2_changed: Signal[bool] = Signal()
        self.wheel_changed: Signal[int] = Signal()

        nav_state_topic = f'robot/navigators/{self._id}_navigator/state'
        self._state_sub = self._node.create_subscription(
            NavigatorState,
            nav_state_topic,
            self._on_state,
            10
        )

        self._inner_led = DigitalIO(f'{self._id}_inner_light', node=node)
        self._outer_led = DigitalIO(f'{self._id}_outer_light', node=node)

        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        nav_state_topic = f'robot/navigators/{self._id}_navigator/state'
        dataflow.wait_for(
            lambda: self._state is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Navigator init failed to get current state from {nav_state_topic}. '
                        + dataflow.BRIDGE_HINT,
        )

    @property
    def _current(self) -> NavigatorState:
        # The newest state; an error instead of None if none has arrived yet.
        return connecting.connected(self._state, f'the {self._id} navigator')

    @property
    def wheel(self) -> int:
        """Current position of the scroll wheel (0-255)."""
        return self._current.wheel

    @property
    def button0(self) -> bool:
        """Current state of button 0 (press wheel)."""
        return self._button(0)

    @property
    def button1(self) -> bool:
        """Current state of button 1 (above wheel)."""
        return self._button(1)

    @property
    def button2(self) -> bool:
        """Current state of button 2 (below wheel)."""
        return self._button(2)

    @property
    def inner_led(self) -> bool:
        """Current state of the inner LED."""
        return self._light('inner')

    @inner_led.setter
    def inner_led(self, enable: bool) -> None:
        """Turn the inner LED on or off."""
        self._inner_led.set_output(enable)

    @property
    def outer_led(self) -> bool:
        """Current state of the outer LED."""
        return self._light('outer')

    @outer_led.setter
    def outer_led(self, enable: bool) -> None:
        """Turn the outer LED on or off."""
        self._outer_led.set_output(enable)

    def _button(self, index: int) -> bool:
        return bool(list(self._current.buttons)[index])

    def _light(self, which: str) -> bool:
        """Read a light's state ('inner' or 'outer') from the newest state message."""
        names = list(self._current.light_names)
        index = names.index(which) if which in names else ('inner', 'outer').index(which)
        return bool(list(self._current.lights)[index])

    def _on_state(self, msg: NavigatorState) -> None:
        if self._state is None:
            self._state = msg
        if self._state == msg:
            return

        buttons = [
            self.button0_changed,
            self.button1_changed,
            self.button2_changed,
        ]
        before = list(self._current.buttons)
        now = list(msg.buttons)
        for i, signal in enumerate(buttons):
            if before[i] != now[i]:
                signal(bool(now[i]))

        if self._current.wheel != msg.wheel:
            self.wheel_changed(int(msg.wheel))

        self._state = msg
