"""Read and set a digital input/output (buttons, lights, suction valves, ...).

Ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink Robotics, BSD license;
see LICENSE). Same methods and behaviour; ROS 1 calls replaced by ROS 2 ones.
DigitalIOState.isInputOnly is called is_input_only in ROS 2 (the bridge maps it).
Not writable raises PermissionError (was IOError(EACCES)).

Each component has its own state topic, /robot/digital_io/<component_id>/state, which
must be listed in bridge_config/topics.yaml. The component ids are in the robot inventory
(topics.txt).
"""

from __future__ import annotations

from rclpy.node import Node

from baxter_core_msgs.msg import DigitalIOState, DigitalOutputCommand

from .. import settings
from .._core import connecting, dataflow
from .._core.node import get_node
from .._core.signals import Signal


class DigitalIO:
    """Interface class for a simple digital input and/or output on the Baxter robot.

    Available methods:
        state         - current state (True = on / pressed)
        is_output     - True if the component can be set
        set_output()  - turn the output on or off
        state_changed - Signal fired with the new state when it changes
    """

    def __init__(self, component_id: str, node: Node | None = None) -> None:
        """
        Parameters:
            component_id: name of the component, e.g. 'left_itb_light_inner'
            node:         optional ROS 2 node to use (default: the shared background node)
        """
        self._id = component_id
        self._component_type = 'digital_io'
        self._node = get_node(node)
        self._is_output = False
        self._state: bool | None = None
        self.state_changed: Signal[bool] = Signal()

        type_ns = 'robot/' + self._component_type
        topic_base = type_ns + '/' + self._id

        self._sub_state = self._node.create_subscription(
            DigitalIOState,
            topic_base + '/state',
            self._on_io_state,
            10
        )

        # Created for every component, also those that turn out to be inputs only (the
        # first state message tells): the bridge can then connect the command topic while
        # the component waits for that state, instead of at the first set_output().
        self._pub_output = self._node.create_publisher(
            DigitalOutputCommand,
            type_ns + '/command',
            10
        )

        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        type_ns = 'robot/' + self._component_type
        topic_base = type_ns + '/' + self._id
        dataflow.wait_for(
            lambda: self._state is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Failed to get current digital_io state from {topic_base}. '
                        + dataflow.BRIDGE_HINT,
        )

    def _on_io_state(self, msg: DigitalIOState) -> None:
        new_state = msg.state == DigitalIOState.PRESSED
        if self._state is None:
            self._state = new_state
            self._is_output = not msg.is_input_only
        # trigger the signal if changed
        if self._state != new_state:
            self.state_changed(new_state)
        self._state = new_state

    @property
    def is_output(self) -> bool:
        """True if the component can be set (is an output)."""
        return self._is_output

    @property
    def state(self) -> bool:
        """Current state of the digital input/output (True = on / pressed)."""
        return connecting.connected(self._state, self._id)

    @state.setter
    def state(self, value: bool) -> None:
        """Set the digital output (is_output must be True)."""
        self.set_output(value)

    def set_output(self, value: bool, timeout: float = 2.0) -> None:
        """Set the state of the digital output.

        Parameters:
            value:   True for on, False for off
            timeout: seconds to wait until the state has changed;
                     0 sends the command once and returns (default 2)
        """
        if not self._is_output:
            raise PermissionError(
                f'Component is not an output [{self._component_type}: {self._id}]')

        cmd = DigitalOutputCommand(
            name=self._id,
            value=bool(value)
        )
        dataflow.publish_once(
            self._pub_output,
            cmd,
            what=f'command for {self._id}'
        )

        if timeout != 0:
            dataflow.wait_for(
                test=lambda: self.state == value,
                timeout=timeout,
                rate=100,
                timeout_msg=f'Failed to command digital io to: {value!r}',
                body=lambda: self._pub_output.publish(cmd),
            )
