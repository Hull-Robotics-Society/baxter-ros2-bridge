"""Enable, disable, reset and stop the robot.

Ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink Robotics, BSD license;
see LICENSE). Same methods and behaviour; ROS 1 calls replaced by ROS 2 ones.
"""

from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import Bool, Empty

from baxter_core_msgs.msg import AssemblyState

from . import settings
from ._core import connecting, dataflow
from ._core.node import get_node


class RobotEnable:
    """Control and status wrapper around the robot state.

    Available methods:
        enable()  - enable all joints
        disable() - disable all joints
        reset()   - reset all joints, reset all jrcp faults, disable the robot
        stop()    - stop the robot, similar to hitting the e-stop button
    """

    def __init__(self, node: Node | None = None) -> None:
        self._node = get_node(node)
        self._state: AssemblyState | None = None
        
        self._state_sub = self._node.create_subscription(
            AssemblyState, 
            'robot/state', 
            self._state_callback, 
            10
        )
        
        self._pub_enable = self._node.create_publisher(Bool, 'robot/set_super_enable', 10)
        self._pub_reset = self._node.create_publisher(Empty, 'robot/set_super_reset', 10)
        self._pub_stop = self._node.create_publisher(Empty, 'robot/set_super_stop', 10)
        
        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        dataflow.wait_for(
            lambda: self._state is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg='Failed to get robot state on /robot/state. ' + dataflow.BRIDGE_HINT
                        + " Is ROS_DOMAIN_ID the same as the bridge's?",
        )

    def _state_callback(self, msg: AssemblyState) -> None:
        self._state = msg

    def _toggle_enabled(self, status: bool) -> None:
        dataflow.wait_for(
            test=lambda: self.state().enabled == status,
            timeout=2.0 if status else 5.0,
            timeout_msg=f"Failed to {'en' if status else 'dis'}able robot",
            body=lambda: self._pub_enable.publish(Bool(data=status)),
        )
        self._node.get_logger().info(f"Robot {'Enabled' if status else 'Disabled'}")

    def state(self) -> AssemblyState:
        """Return the last known robot state (baxter_core_msgs/AssemblyState).

        Its fields: ready, enabled, stopped, error (True or False), estop_button and
        estop_source (numbers; the AssemblyState.ESTOP_... constants name them).

        Raises:
            ConnectionError if no state has arrived yet (only possible inside a setup()
            block).
        """
        return connecting.connected(self._state, 'the robot (/robot/state)')

    def enable(self) -> None:
        """Enable all joints."""
        if self.state().stopped:
            self._node.get_logger().info('Robot Stopped: Attempting Reset...')
            self.reset()
        self._toggle_enabled(True)

    def disable(self) -> None:
        """Disable all joints."""
        self._toggle_enabled(False)

    def reset(self) -> bool:
        """Reset all joints. Trigger JRCP hardware to reset all faults. Disable the robot.

        Returns:
            True if the robot is reset; False if the stop was cleared but the robot still
            reports a (non-fatal) error.
        """
        def is_reset() -> bool:
            s = self.state()
            return (not s.enabled and not s.stopped and not s.error
                    and s.estop_button == 0 and s.estop_source == 0)

        if (self.state().stopped and self.state().estop_button == AssemblyState.ESTOP_BUTTON_PRESSED):
            msg = 'E-Stop is ASSERTED. Disengage E-Stop and then reset the robot.'
            self._node.get_logger().fatal(msg)
            raise RuntimeError('Failed to Reset: E-Stop Engaged')

        self._node.get_logger().info('Resetting robot...')
        try:
            dataflow.wait_for(
                test=is_reset,
                timeout=3.0,
                timeout_msg='Failed to reset robot. Check that the bridge is running and '
                            'that the robot can reach it (ROS_IP on the bridge machine).',
                body=lambda: self._pub_reset.publish(Empty()),
            )
        except TimeoutError:
            if self.state().error and not self.state().stopped:
                self._node.get_logger().warn(
                    'Non-fatal Robot Error on reset. Robot reset cleared stopped state and '
                    'robot can be enabled, but a non-fatal error persists. Check '
                    'diagnostics or rethink.log for more info.')
                return False
            raise
        return True

    def stop(self) -> None:
        """Simulate an e-stop button being pressed. The robot must be reset afterwards."""
        dataflow.wait_for(
            test=lambda: self.state().stopped,
            timeout=3.0,
            timeout_msg='Failed to stop the robot',
            body=lambda: self._pub_stop.publish(Empty()),
        )
