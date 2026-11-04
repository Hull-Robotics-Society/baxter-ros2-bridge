"""Read and set an analog input/output (e.g. the torso fan).

Ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink Robotics, BSD license;
see LICENSE). Same methods and behaviour; ROS 1 calls replaced by ROS 2 ones.
AnalogIOState.isInputOnly is called is_input_only in ROS 2 (the bridge maps it).
Not writable raises PermissionError (was IOError(EACCES)).

Each component has its own state topic, /robot/analog_io/<component_id>/state, which
must be listed in bridge_config/topics.yaml. The component ids are in the robot inventory
(topics.txt).
"""

from __future__ import annotations

from rclpy.node import Node

from baxter_core_msgs.msg import AnalogIOState, AnalogOutputCommand

from .. import settings
from .._core import connecting, dataflow
from .._core.node import get_node


class AnalogIO:
    """Interface class for a simple analog input and/or output on the Baxter robot.

    Available methods:
        state()      - latest value
        is_output()  - True if the component can be set
        set_output() - set a new output value
    """

    def __init__(self, component_id: str, node: Node | None = None) -> None:
        """
        Parameters:
            component_id: name of the component, e.g. 'torso_fan'
            node:         optional ROS 2 node to use (default: the shared background node)
        """
        self._id = component_id
        self._component_type = 'analog_io'
        self._node = get_node(node)
        self._is_output = False
        self._state: dict[str, float] = {}

        type_ns = 'robot/' + self._component_type
        topic_base = type_ns + '/' + self._id

        self._sub_state = self._node.create_subscription(
            AnalogIOState,
            topic_base + '/state',
            self._on_io_state,
            10
        )

        # Created for every component, also those that turn out to be inputs only (the
        # first state message tells): the bridge can then connect the command topic while
        # the component waits for that state, instead of at the first set_output().
        self._pub_output = self._node.create_publisher(
            AnalogOutputCommand,
            type_ns + '/command',
            10
        )

        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        type_ns = 'robot/' + self._component_type
        topic_base = type_ns + '/' + self._id
        dataflow.wait_for(
            lambda: len(self._state) != 0,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Failed to get current analog_io state from {topic_base}. '
                        + dataflow.BRIDGE_HINT,
        )

    def _on_io_state(self, msg: AnalogIOState) -> None:
        self._is_output = not msg.is_input_only
        self._state['value'] = msg.value

    def state(self) -> float:
        """Return the latest value of the analog input/output."""
        return connecting.connected(self._state.get('value'), self._id)

    def is_output(self) -> bool:
        """Return True if the component can be set (is an output)."""
        return self._is_output

    def set_output(self, value: float, timeout: float = 2.0) -> None:
        """Set the value of the analog output.

        Parameters:
            value:   new output value (0 to 65535)
            timeout: seconds to wait until the value has changed;
                     0 sends the command once and returns (default 2)
        """
        if not self._is_output:
            raise PermissionError(
                f'Component is not an output [{self._component_type}: {self._id}]')

        cmd = AnalogOutputCommand(
            name=self._id,
            value=int(value)
        )
        dataflow.publish_once(
            self._pub_output,
            cmd,
            what=f'command for {self._id}'
        )

        if timeout != 0:
            dataflow.wait_for(
                test=lambda: self.state() == value,
                timeout=timeout,
                rate=100,
                timeout_msg=f'Failed to command analog io to: {value}',
                body=lambda: self._pub_output.publish(cmd),
            )
