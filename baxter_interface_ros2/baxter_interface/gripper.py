"""Control a gripper (electric or suction) on one of Baxter's arms.

Ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink Robotics, BSD license;
see LICENSE). Same methods and behaviour; ROS 1 calls replaced by ROS 2 ones.

Note: EndEffectorState's flags (enabled, calibrated, ready, ...) are uint8 with three
values (STATE_FALSE, STATE_TRUE, STATE_UNKNOWN), not bool. The original compared them
with True/False; this port compares with STATE_TRUE / STATE_FALSE explicitly.
"""

from __future__ import annotations

import json
import time
from typing import Any, Callable, Mapping

from rclpy.node import Node

from baxter_core_msgs.msg import EndEffectorCommand, EndEffectorProperties, EndEffectorState

from . import settings
from ._core import at_once, connecting, dataflow
from ._core.locks import locked, part_lock
from ._core.node import get_node

_TRUE = EndEffectorState.STATE_TRUE
_FALSE = EndEffectorState.STATE_FALSE


class Gripper:
    """Interface class for a gripper on the Baxter robot.

    Available methods (the most used ones):
        open(), close()       - fully open / close (electric), stop / start suction (suction)
        command_position()    - move to a position, 0 (closed) to 100 (open)
        calibrate()           - calibrate an electric gripper (needed before moving it)
        reset(), reboot()     - clear errors
        position(), force()   - current position and force, 0 to 100
        type()                - 'electric', 'suction' or 'custom'
    """

    def __init__(self, gripper: str, node: Node | None = None) -> None:
        """
        Parameters:
            gripper: arm the gripper is mounted on, 'left' or 'right'
            node:    optional ROS 2 node to use (default: the shared background node)
        """
        self.name: str = gripper + '_gripper'
        self._node = get_node(node)
        # Held by every command: one command at a time per gripper. A blocking command
        # waits for the gripper to acknowledge its own sequence number, so two threads
        # commanding the same gripper must not overlap.
        self._part_lock = part_lock(f'gripper/{gripper}', self._node)
        self._cmd_sender = self._node.get_fully_qualified_name() + '_{}'
        self._cmd_sequence = 0
        self._state: EndEffectorState | None = None
        self._fresh = dataflow.Freshness(
            f'{self.name} gripper',
            f'robot/end_effector/{self.name}/state'
        )
        self._prop = EndEffectorProperties()
        self._parameters: dict[str, float] = {}

        ns = f'robot/end_effector/{self.name}/'
        self._cmd_pub = self._node.create_publisher(
            EndEffectorCommand,
            ns + 'command',
            10
        )

        self._state_sub = self._node.create_subscription(
            EndEffectorState,
            ns + 'state',
            self._on_gripper_state,
            10
        )
        self._prop_sub = self._node.create_subscription(
            EndEffectorProperties,
            ns + 'properties',
            self._on_gripper_prop,
            10
        )

        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        ns = f'robot/end_effector/{self.name}/'
        # Wait for the gripper state message to be populated
        dataflow.wait_for(
            lambda: self._state is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Failed to get state from {ns}state. ' + dataflow.BRIDGE_HINT,
        )
        # Wait for the gripper type to be populated
        dataflow.wait_for(
            lambda: self.type() is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Failed to get properties from {ns}properties. ' + dataflow.BRIDGE_HINT,
        )

        self.set_parameters(defaults=True)

    # Messages arrive as new objects, so storing the reference is enough (no copy needed).
    def _on_gripper_state(self, state: EndEffectorState) -> None:
        self._fresh.mark()
        self._state = state

    def _on_gripper_prop(self, properties: EndEffectorProperties) -> None:
        self._prop = properties

    @property
    def _current(self) -> EndEffectorState:
        # The newest state; an error instead of None if none has arrived yet.
        return connecting.connected(self._state, f'the {self.name}')

    def _inc_cmd_sequence(self) -> int:
        # manage roll over with safe value (maxint)
        self._cmd_sequence = (self._cmd_sequence % 0x7FFFFFFF) + 1
        return self._cmd_sequence

    def _clip(self, val: float) -> float:
        return max(min(val, 100.0), 0.0)

    def _capability_warning(self, cmd: str) -> None:
        self._node.get_logger().warn(f"{self.name} {self.type()} - not capable of '{cmd}' command")

    def _state_json(self, key: str) -> Any:
        return json.loads(self._current.state)[key]

    # --- Raw command -----------------------------------------------------------------------

    @locked
    def command(
        self,
        cmd: str,
        block: bool = False,
        test: Callable[[], bool] = lambda: True,
        timeout: float = 0.0,
        args: Mapping[str, Any] | None = None
    ) -> bool:
        """Raw command call to directly control the gripper.

        Parameters:
            cmd:     one of the EndEffectorCommand.CMD_* strings
            block:   wait for the command to finish (default False)
            test:    function that returns True when the command has succeeded
            timeout: seconds to wait when blocking
            args:    dict of command arguments {parameter: value}

        Returns:
            bool: True if the command succeeded (always True when not blocking)
        """
        ee_cmd = EndEffectorCommand()
        ee_cmd.id = self.hardware_id()
        ee_cmd.command = cmd
        ee_cmd.sender = self._cmd_sender.format(cmd)
        ee_cmd.sequence = self._inc_cmd_sequence()
        ee_cmd.args = json.dumps(args) if args is not None else ''

        def acknowledged():
            self._fresh.require()
            # The gripper echoes sender and sequence once it has handled the command.
            return (self._current.command_sender == ee_cmd.sender
                    and self._current.command_sequence in (ee_cmd.sequence, 0))

        dataflow.publish_once(
            self._cmd_pub,
            ee_cmd,
            what=f"command '{cmd}' for the {self.name} gripper"
        )
        if not block:
            return True

        finish_time = time.monotonic() + timeout
        ok = dataflow.wait_for(
            test=acknowledged,
            timeout=timeout,
            raise_on_error=False,
            body=lambda: self._cmd_pub.publish(ee_cmd),
        )
        if not ok:
            self._node.get_logger().debug(
                f'Timed out on gripper command acknowledgement for {self.name}:{ee_cmd.command}')

        time_remain = max(0.5, finish_time - time.monotonic())
        return dataflow.wait_for(
            test=test,
            timeout=time_remain,
            raise_on_error=False,
            body=lambda: self._cmd_pub.publish(ee_cmd),
        )

    # --- Parameters ------------------------------------------------------------------------

    def valid_parameters_text(self) -> str:
        """Text describing the valid gripper parameters."""
        if self.type() == 'electric':
            return (
                'Valid gripper parameters for the electric gripper are\n'
                'PARAMETERS:\n'
                '  velocity - Velocity at which a position move will execute\n'
                '  moving_force - Force threshold at which a move will stop\n'
                '  holding_force - Force at which a grasp will continue holding\n'
                '  dead_zone - Position deadband within move considered successful\n'
                'ALL PARAMETERS (0-100)'
            )
        if self.type() == 'suction':
            return (
                'Valid gripper parameters for the suction gripper are\n'
                'PARAMETERS:\n'
                '  vacuum_sensor_threshold - Measured suction threshold denoting grasp\n'
                '  blow_off_seconds - Time in seconds to blow air on release\n'
                'ALL PARAMETERS (0-100)'
            )
        return f'No valid parameters for {self.type()} {self.name}.'

    def valid_parameters(self) -> dict[str, float]:
        """Return the available gripper parameters with their default values.

        Returns:
            dict: {parameter: default value}
        """
        if self.type() == 'electric':
            return {
                'velocity': 50.0,
                'moving_force': 40.0,
                'holding_force': 30.0,
                'dead_zone': 5.0,
            }
        if self.type() == 'suction':
            return {
                'vacuum_sensor_threshold': 18.0,
                'blow_off_seconds': 0.4,
            }
        return {}

    @locked
    def set_parameters(self, parameters: Mapping[str, float] | None = None, defaults: bool = False) -> None:
        """Set the parameters that describe how position commands are executed.

        Parameters:
            parameters: dict {parameter: value}, each a percentage of maximum (0-100)
            defaults:   start from the default parameters (default False)
        """
        valid_parameters = self.valid_parameters()
        if defaults:
            self._parameters = valid_parameters
        for key, value in (parameters or {}).items():
            if key in valid_parameters:
                self._parameters[key] = value
            else:
                self._node.get_logger().warn(
                    f'Invalid parameter: {key} provided. {self.valid_parameters_text()}')
        self.command(EndEffectorCommand.CMD_CONFIGURE, args=self._parameters)

    def parameters(self) -> dict[str, float]:
        """Return the parameters describing the gripper command execution.

        Returns:
            dict: {parameter: value}
        """
        return dict(self._parameters)

    # --- Commands --------------------------------------------------------------------------

    @locked
    def reset(self, block: bool = True, timeout: float = 2.0) -> bool | None:
        """Reset the gripper state, removing any errors (electric only).

        Parameters:
            block:   wait for the reset to finish (default True)
            timeout: seconds to wait for the reset (default 2)
        """
        if self.type() != 'electric':
            return self._capability_warning('reset')
        return self.command(
            EndEffectorCommand.CMD_RESET,
            block,
            test=lambda: self._current.error == _FALSE and self._current.ready == _TRUE,
            timeout=timeout,
        )

    @locked
    def _cmd_reboot(self, block: bool = True, timeout: float = 5.0) -> bool | None:
        """Power cycle the gripper, removing calibration information.

        Basic reboot: waits for the gripper to be ready again but does not clear errors
        that could occur during boot. Use reboot() instead.
        """
        if self.type() != 'electric':
            return self._capability_warning('reboot')
        success = self.command(
            EndEffectorCommand.CMD_REBOOT,
            block,
            test=lambda: self._current.enabled == _TRUE and self._current.ready == _TRUE,
            timeout=timeout,
        )
        time.sleep(0.5)  # Allow extra time for reboot to complete
        self.set_parameters(defaults=True)
        return success

    @locked
    def reboot(self, timeout: float = 5.0, delay_check: float = 0.1) -> bool | None:
        """Clean reboot of the gripper: removes calibration and errors (electric only).

        Power cycles the gripper, then checks for errors and resets it if needed.

        Parameters:
            timeout:     seconds to wait for the reboot and the reset (default 5)
            delay_check: seconds after the reboot before checking for errors (default 0.1)

        Returns:
            bool: True on success
        """
        if self.type() != 'electric':
            return self._capability_warning('reboot')
        self._cmd_reboot(block=True, timeout=timeout)
        time.sleep(delay_check)
        if self.error():
            if not self.reset(block=True, timeout=timeout):
                self._node.get_logger().error('Failed to reset gripper error after reboot.')
                return False
            self.set_parameters(defaults=True)
        return True

    @locked
    def clear_calibration(self, block: bool = True, timeout: float = 2.0) -> bool | None:
        """Clear the calibration, allowing (and requiring) a new calibration (electric only).

        Parameters:
            block:   wait for it to finish (default True)
            timeout: seconds to wait (default 2)
        """
        if self.type() != 'electric':
            return self._capability_warning('clear_calibration')
        return self.command(
            EndEffectorCommand.CMD_CLEAR_CALIBRATION,
            block,
            test=lambda: self._current.calibrated == _FALSE and self._current.ready == _TRUE,
            timeout=timeout,
        )

    @at_once.command(wait_keyword='block')
    @locked
    def calibrate(self, block: bool = True, timeout: float = 5.0) -> bool | None:
        """Calibrate the gripper, setting its minimum and maximum travel (electric only).

        Parameters:
            block:   wait for the calibration to finish (default True)
            timeout: seconds to wait (default 5)

        Returns:
            bool: True on success
        """
        if self.type() != 'electric':
            return self._capability_warning('calibrate')

        # clear any previous calibration and any current errors
        if self.calibrated():
            self.clear_calibration()
        if self.error():
            self.reset(block=block)

        success = self.command(
            EndEffectorCommand.CMD_CALIBRATE,
            block,
            test=lambda: self._current.calibrated == _TRUE and self._current.ready == _TRUE,
            timeout=timeout,
        )
        self.set_parameters(defaults=True)
        return success

    @locked
    def stop(self, block: bool = True, timeout: float = 5.0) -> bool | None:
        """Stop the gripper at the current position and apply the holding force.

        For a suction gripper: stop suction and blow off.

        Parameters:
            block:   wait for it to finish (default True)
            timeout: seconds to wait (default 5)
        """
        if self.type() == 'custom':
            return self._capability_warning('stop')
        if self.type() == 'electric':
            cmd = EndEffectorCommand.CMD_STOP
            stop_test = lambda: self._current.moving == _FALSE  # noqa: E731
        else:  # suction
            timeout = max(self._parameters['blow_off_seconds'], timeout)
            cmd = EndEffectorCommand.CMD_RELEASE
            stop_test = lambda: not self.sucking() and not self.blowing()  # noqa: E731
        return self.command(
            cmd,
            block,
            test=stop_test,
            timeout=timeout,
        )

    @at_once.command(wait_keyword='block')
    @locked
    def command_position(self, position: float, block: bool = False, timeout: float = 5.0) -> bool | None:
        """Move the gripper to a position.

        Parameters:
            position: 0 (closed) to 100 (open)
            block:    wait for the move to finish (default False)
            timeout:  seconds to wait when blocking (default 5)

        Returns:
            bool: True on success
        """
        if self.type() == 'custom':
            return self._capability_warning('command_position')
        if self._current.calibrated != _TRUE:
            self._node.get_logger().warn(f'Unable to command {self.name} position until calibrated')
            return False

        arguments = {'position': self._clip(position)}
        if self.type() == 'electric':
            def reached():
                return (abs(self._current.position - position) < self._parameters['dead_zone']
                        or self._current.gripping == _TRUE)

            return self.command(
                EndEffectorCommand.CMD_GO,
                block,
                test=reached,
                timeout=timeout,
                args=arguments,
            )
        if arguments['position'] < 100.0:
            return self.close(block=block, timeout=timeout)
        return self.open(block=block, timeout=timeout)

    @locked
    def command_suction(self, block: bool = False, timeout: float = 5.0) -> bool | None:
        """Start suction (suction gripper only).

        Parameters:
            block:   wait until a grasp is detected (vacuum threshold exceeded)
            timeout: seconds to apply suction while trying to detect a grasp (default 5)
        """
        if self.type() != 'suction':
            return self._capability_warning('command_suction')
        return self.command(
            EndEffectorCommand.CMD_GO,
            block,
            test=lambda: bool(self.vacuum()),
            timeout=timeout,
            args={'grip_attempt_seconds': timeout},
        )

    @at_once.command(wait_keyword='block')
    @locked
    def open(self, block: bool = False, timeout: float = 5.0) -> bool | None:
        """Open fully (electric), or stop suction and blow off (suction)."""
        if self.type() == 'custom':
            return self._capability_warning('open')
        if self.type() == 'electric':
            return self.command_position(position=100.0, block=block, timeout=timeout)
        return self.stop(block=block, timeout=timeout)

    @at_once.command(wait_keyword='block')
    @locked
    def close(self, block: bool = False, timeout: float = 5.0) -> bool | None:
        """Close fully (electric), or start suction (suction)."""
        if self.type() == 'custom':
            return self._capability_warning('close')
        if self.type() == 'electric':
            return self.command_position(position=0.0, block=block, timeout=timeout)
        return self.command_suction(block=block, timeout=timeout)

    # --- Parameter shortcuts ---------------------------------------------------------------

    def set_velocity(self, velocity: float) -> None:
        """Set the velocity of position moves, 0 (stop) to 100 (max), default 50 (electric only)."""
        if self.type() != 'electric':
            return self._capability_warning('set_velocity')
        self.set_parameters(parameters={'velocity': self._clip(velocity)})

    def set_moving_force(self, force: float) -> None:
        """Set the force at which a position move stops, 0 to 100, default 40 (electric only)."""
        if self.type() != 'electric':
            return self._capability_warning('set_moving_force')
        self.set_parameters(parameters={'moving_force': self._clip(force)})

    def set_holding_force(self, force: float) -> None:
        """Set the force applied after a move has finished, 0 to 100, default 30 (electric only)."""
        if self.type() != 'electric':
            return self._capability_warning('set_holding_force')
        self.set_parameters(parameters={'holding_force': self._clip(force)})

    def set_dead_band(self, dead_band: float) -> None:
        """Set the position error at which a move counts as done, in %, default 5 (electric only)."""
        if self.type() != 'electric':
            return self._capability_warning('set_dead_band')
        self.set_parameters(parameters={'dead_zone': self._clip(dead_band)})

    def set_vacuum_threshold(self, threshold: float) -> None:
        """Set the vacuum that counts as a successful grasp, in %, default 18 (suction only)."""
        if self.type() != 'suction':
            return self._capability_warning('set_vacuum_threshold')
        self.set_parameters(parameters={'vacuum_sensor_threshold': self._clip(threshold)})

    def set_blow_off(self, blow_off: float) -> None:
        """Set how long air is blown on release, in seconds, default 0.4 (suction only).

        Only used after the previous suction command detected a grasp.
        """
        if self.type() != 'suction':
            return self._capability_warning('set_blow_off')
        self.set_parameters(parameters={'blow_off_seconds': blow_off})

    # --- State -----------------------------------------------------------------------------

    def calibrated(self) -> bool:
        """Return True if the gripper is calibrated."""
        return self._current.calibrated == _TRUE

    def ready(self) -> bool:
        """Return True if the gripper is ready: calibrated, not busy and not moving."""
        return self._current.ready == _TRUE

    def moving(self) -> bool:
        """Return True if the gripper is moving."""
        return self._current.moving == _TRUE

    def gripping(self) -> bool:
        """Return True if a position move stopped because the moving force was exceeded (a grasp)."""
        return self._current.gripping == _TRUE

    def missed(self) -> bool:
        """Return True if a position move finished without exceeding the moving force."""
        return self._current.missed == _TRUE

    def error(self) -> bool:
        """Return True if the gripper is in an error state. Clear it with reset()."""
        return self._current.error == _TRUE

    def position(self) -> float:
        """Return the current position, 0 (closed) to 100 (open)."""
        return self._current.position

    def force(self) -> float:
        """Return the current force, 0 to 100 of the maximum."""
        return self._current.force

    def vacuum_sensor(self) -> float | None:
        """Return the vacuum sensor reading, 0 to 100 of its range (suction only)."""
        if self.type() != 'suction':
            return self._capability_warning('vacuum_sensor')
        return (self._state_json('vacuum sensor') / 255.0) * 100.0

    def vacuum(self) -> bool | None:
        """Return True if the vacuum threshold was exceeded during suction (suction only)."""
        if self.type() != 'suction':
            return self._capability_warning('vacuum')
        return self._state_json('vacuum')

    def blowing(self) -> bool | None:
        """Return True if the gripper is blowing (suction only)."""
        if self.type() != 'suction':
            return self._capability_warning('blowing')
        return self._state_json('blowing')

    def sucking(self) -> bool | None:
        """Return True if the gripper is sucking (suction only)."""
        if self.type() != 'suction':
            return self._capability_warning('sucking')
        return self._state_json('sucking')

    # --- Properties ------------------------------------------------------------------------

    def has_force(self) -> bool:
        """Return True if the gripper is capable of force control."""
        return self._prop.controls_force

    def has_position(self) -> bool:
        """Return True if the gripper is capable of position control."""
        return self._prop.controls_position

    def type(self) -> str | None:
        """Return the gripper type: 'suction', 'electric' or 'custom'.

        An unknown or missing gripper is reported as 'custom'. Returns None until the
        properties message has arrived.
        """
        return {
            EndEffectorProperties.SUCTION_CUP_GRIPPER: 'suction',
            EndEffectorProperties.ELECTRIC_GRIPPER: 'electric',
            EndEffectorProperties.PASSIVE_GRIPPER: 'custom',
        }.get(self._prop.ui_type, None)

    def hardware_id(self) -> int:
        """Return the gripper's unique hardware id (needed to send it commands)."""
        return self._current.id

    def hardware_version(self) -> str:
        """Return the gripper hardware (product name)."""
        return self._prop.product

    def firmware_version(self) -> str:
        """Return the gripper firmware revision."""
        return self._prop.firmware_rev
