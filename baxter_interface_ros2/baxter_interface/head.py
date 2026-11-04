"""Pan the head and make it nod.

Ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink Robotics, BSD license;
see LICENSE). Same methods and behaviour; ROS 1 calls replaced by ROS 2 ones.
The messages are those of robot software 1.2.0: HeadState's fields are called
is_turning / is_nodding in ROS 2 (isTurning / isNodding in ROS 1; the bridge maps them),
and the pan command takes a speed ratio from 0 to 1. set_pan() keeps the SDK v0.7.0 speed
range of 0 to 100 and converts it.
"""

from __future__ import annotations

import threading

from rclpy.node import Node
from std_msgs.msg import Bool

from baxter_core_msgs.msg import HeadPanCommand, HeadState

from . import settings
from ._core import at_once, connecting, dataflow
from ._core.locks import locked, part_lock
from ._core.node import get_node

# Added to the messages of head commands that ran out of time: the most common reason.
_ENABLED_HINT = ('Is the robot enabled? A disabled robot does not move its head '
                 '(RobotEnable().enable(), or: ros2 run baxter_interface enable_robot -e).')


class Head:
    """Interface class for the head on the Baxter robot.

    Available methods:
        pan()         - current pan angle (rad)
        panning()     - True while the head is panning
        nodding()     - True while the head is nodding
        set_pan()     - pan to an angle
        command_nod() - nod once
    """

    def __init__(self, node: Node | None = None) -> None:
        self._node = get_node(node)
        self._lock = threading.Lock()  # guards the stored values
        # Held by set_pan() and command_nod(): one head command at a time.
        self._part_lock = part_lock('head', self._node)
        self._state: dict[str, float | bool] = {}
        self._fresh = dataflow.Freshness('head', dataflow.full_name(self._node, 'robot/head/head_state'))

        self._pub_pan = self._node.create_publisher(
            HeadPanCommand,
            'robot/head/command_head_pan',
            10
        )
        self._pub_nod = self._node.create_publisher(
            Bool,
            'robot/head/command_head_nod',
            10
        )

        self._sub_state = self._node.create_subscription(
            HeadState,
            'robot/head/head_state',
            self._on_head_state,
            10
        )

        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        dataflow.wait_for(
            lambda: len(self._state) != 0,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg='Failed to get current head state from /robot/head/head_state. '
                        + dataflow.BRIDGE_HINT,
        )

    def _on_head_state(self, msg: HeadState) -> None:
        self._fresh.mark()
        with self._lock:
            self._state['pan'] = msg.pan
            self._state['panning'] = msg.is_turning
            self._state['nodding'] = msg.is_nodding

    def pan(self) -> float:
        """Get the current pan angle of the head.

        Returns:
            float: current angle in radians
        """
        with self._lock:
            return float(connecting.connected(self._state.get('pan'), 'the head'))

    def nodding(self) -> bool:
        """Check if the head is currently nodding.

        Returns:
            bool: True if the head is currently nodding
        """
        with self._lock:
            return bool(connecting.connected(self._state.get('nodding'), 'the head'))

    def panning(self) -> bool:
        """Check if the head iis currently panning.

        Returns:
            bool: True if the head is currently pannng
        """
        with self._lock:
            return bool(connecting.connected(self._state.get('panning'), 'the head'))

    @at_once.command
    @locked
    def set_pan(self, angle: float, speed: float = 100, timeout: float = 10.0) -> None:
        """Pan at the given speed to the desired angle.

        Parameters:
            angle:   desired pan angle in radians
            speed:   speed to pan at, 0 to 100 (default 100)
            timeout: seconds to wait for the head to reach the angle;
                     0 sends the command once and returns (default 10)
        """
        msg = HeadPanCommand(
            target=float(angle),
            speed_ratio=min(max(float(speed) / 100.0, 0.0), 1.0),
            enable_pan_request=HeadPanCommand.REQUEST_PAN_ENABLE
        )
        dataflow.publish_once(
            self._pub_pan,
            msg,
            what='head pan command'
        )

        def reached():
            self._fresh.require()
            return abs(self.pan() - angle) <= settings.HEAD_PAN_ANGLE_TOLERANCE

        if timeout != 0:
            dataflow.wait_for(
                reached,
                timeout=timeout,
                rate=100,
                timeout_msg=f'Failed to move head to pan command {angle:f}. ' + _ENABLED_HINT,
                body=lambda: self._pub_pan.publish(msg),
            )

    @at_once.command
    @locked
    def command_nod(self, timeout: float = 5.0) -> None:
        """Command the head to nod once.

        Parameters:
            timeout: seconds to wait for the nod to start and to finish;
                     0 sends the command once and returns (default 5)
        """
        nod = Bool(data=True)
        dataflow.publish_once(
            self._pub_nod,
            nod,
            what='head nod command'
        )

        def nodding():
            self._fresh.require()
            return self.nodding()

        if timeout != 0:
            # Wait for the nod to start ...
            dataflow.wait_for(
                test=nodding,
                timeout=timeout,
                rate=100,
                timeout_msg='Failed to initiate head nod command. ' + _ENABLED_HINT,
                body=lambda: self._pub_nod.publish(nod),
            )
            # ... and to finish
            dataflow.wait_for(
                test=lambda: not nodding(),
                timeout=timeout,
                rate=100,
                timeout_msg='Failed to complete head nod command. ' + _ENABLED_HINT,
                # While waiting for the nod to finish, the SDK sends "stop nodding".
                body=lambda: self._pub_nod.publish(Bool(data=False)),
            )
