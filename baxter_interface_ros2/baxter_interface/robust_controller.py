"""Run one of the robot's built-in maintenance routines ("robust controllers").

Ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink Robotics, BSD license;
see LICENSE). Same state machine and error codes; ROS 1 calls replaced by ROS 2 ones.
RobustControllerStatus.isEnabled is called is_enabled in ROS 2 (the bridge maps it).

A robust controller (e.g. CalibrateArm, Tare) runs on the robot. It keeps running only
while it receives its enable message at least once per second, and reports progress on
its status topic. This class sends the enable message at 2 Hz until the routine reports
success or failure, or until the timeout.

Needs both topics of the routine in bridge_config/topics.yaml:
    <namespace>/enable   (the routine's own enable message type)
    <namespace>/status   (baxter_core_msgs/msg/RobustControllerStatus)
"""

from __future__ import annotations

import errno
import threading
import time
from typing import Any

from rclpy.node import Node

from baxter_core_msgs.msg import RobustControllerStatus

from ._core import dataflow
from ._core.locks import locked, part_lock
from ._core.node import get_node

_ERROR_MESSAGES = {
    errno.EIO: 'Robust controller failed',
    errno.ENOMSG: 'Robust controller failed to enable',
    errno.ETIMEDOUT: 'Robust controller timed out',
    errno.ECONNABORTED: 'Robust controller interrupted by user',
}


class RobustController:
    """Wrapper around controlling a robust controller.

    Available methods:
        run() - enable the routine and wait until it finishes; raises OSError on failure
    """

    (STATE_IDLE,
     STATE_STARTING,
     STATE_RUNNING,
     STATE_STOPPING) = range(4)

    def __init__(
        self,
        namespace: str,
        enable_msg: Any,
        disable_msg: Any,
        timeout: float = 60,
        node: Node | None = None
    ) -> None:
        """
        Parameters:
            namespace:   namespace containing the enable and status topics,
                         e.g. 'robustcontroller/left/CalibrateArm'
            enable_msg:  message to send to enable the routine
            disable_msg: message to send to disable the routine
            timeout:     seconds to wait for the routine to finish (default 60)
            node:        optional ROS 2 node to use (default: the shared background node)
        """
        self._node = get_node(node)
        # One run of the same routine at a time, also across threads.
        self._part_lock = part_lock('robust_controller' + namespace, self._node)
        # The state is changed by the receiving thread (_callback) and by run();
        # this lock keeps each change in one piece.
        self._state_lock = threading.Lock()

        self._command_pub = self._node.create_publisher(
            type(enable_msg),
            namespace + '/enable',
            10
        )
        self._status_sub = self._node.create_subscription(
            RobustControllerStatus,
            namespace + '/status',
            self._callback,
            10
        )

        self._enable_msg = enable_msg
        self._disable_msg = disable_msg
        self._timeout = timeout
        self._state = self.STATE_IDLE
        self._return = 0

    def _callback(self, msg: RobustControllerStatus) -> None:
        with self._state_lock:
            if self._state == self.STATE_RUNNING:
                if msg.complete == RobustControllerStatus.COMPLETE_W_SUCCESS:
                    self._state = self.STATE_STOPPING
                    self._return = 0
                elif msg.complete == RobustControllerStatus.COMPLETE_W_FAILURE:
                    self._state = self.STATE_STOPPING
                    self._return = errno.EIO
                elif not msg.is_enabled:
                    self._state = self.STATE_IDLE
                    self._return = errno.ENOMSG
            elif self._state == self.STATE_STOPPING and not msg.is_enabled:
                # Would be nice to use msg.state here, but it does not
                # consistently reflect reality.
                self._state = self.STATE_IDLE
            elif self._state == self.STATE_STARTING and msg.is_enabled:
                self._state = self.STATE_RUNNING

    def _run_loop(self) -> None:
        # Robust controllers need messages at more than 1 Hz to continue their operation.
        period = 0.5
        start = time.monotonic()
        while True:
            with self._state_lock:
                if (self._state == self.STATE_RUNNING
                        and time.monotonic() - start > self._timeout):
                    self._state = self.STATE_STOPPING
                    self._return = errno.ETIMEDOUT
                state = self._state
            if state in (self.STATE_STARTING, self.STATE_RUNNING):
                self._command_pub.publish(self._enable_msg)
            elif state == self.STATE_STOPPING:
                self._command_pub.publish(self._disable_msg)
            else:  # STATE_IDLE
                break
            time.sleep(period)

    def _abort(self, give_up_after: float = 10.0) -> None:
        """Tell the routine to stop (Ctrl+C), and wait until it reports idle."""
        end = time.monotonic() + give_up_after
        while self._state != self.STATE_IDLE and time.monotonic() < end:
            self._command_pub.publish(self._disable_msg)
            time.sleep(0.5)
        with self._state_lock:
            self._return = errno.ECONNABORTED

    @locked
    def run(self) -> None:
        """Enable the robust controller and run until completion or error.

        Raises:
            OSError with errno EIO (failed), ENOMSG (failed to enable), ETIMEDOUT
            (timed out) or ECONNABORTED (interrupted with Ctrl+C).
        """
        with self._state_lock:
            self._state = self.STATE_STARTING
            self._return = 0
        dataflow.publish_once(
            self._command_pub,
            self._enable_msg,
            what='start command of the maintenance routine'
        )
        try:
            self._run_loop()
        except KeyboardInterrupt:
            self._abort()

        if self._return != 0:
            message = _ERROR_MESSAGES.get(self._return)
            if message:
                raise OSError(self._return, message)
            raise OSError(self._return)
