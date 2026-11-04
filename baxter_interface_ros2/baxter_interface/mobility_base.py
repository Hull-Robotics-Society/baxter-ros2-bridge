"""The Dataspeed Mobility Base a Baxter can stand on: its state, and driving it.

New in the ROS 2 library; the Baxter SDK has nothing for the base, Dataspeed's own SDK
offers the raw topics only (docs/mobility-base.md lists them).

    from baxter_interface import MobilityBase

    base = MobilityBase()
    print(base.mode(), base.battery().percent)
    base.drive(forward=0.1, seconds=2.0)        # 0.1 m/s for two seconds, then stop

The base is a ROS 1 system of its own, on the computer inside the base, and needs a
bridge of its own next to the robot's (docs/mobility-base.md). With several robots, the
base's bridge takes the name of the robot that stands on it, and the base is that
robot's: Robot('two').MobilityBase().

How the base is driven, and what keeps that safe:

  - The base follows a speed command only for a moment: it stops by itself when no new
    command has arrived for a fifth of a second. A program therefore never "switches the
    base on": it has to keep saying how fast, and a program that crashes, hangs or loses
    its connection leaves a standing base. drive() repeats the command for you for the
    time you give; set_velocity() sends it once, for a loop of your own.
  - The base only follows a program while its wireless joystick allows it: joystick on,
    enabled (left stick up) and its mode switch on "Computer". The joystick stays in
    charge: left stick down stops the base whatever a program sends. drive() checks that
    the base really follows and says why if it does not.
  - A program can tell the base to ignore the joystick (ignore_joystick=True). The base
    then follows the program whatever the joystick's switches say, also with the
    joystick switched off. That takes the joystick's stop away: Dataspeed's documentation
    does not say that it still works then, and it has not been tried. The e-stop button
    is then the way to stop the base by hand. Off unless asked for.
  - Speeds are limited to settings.BASE_MAX_SPEED and settings.BASE_MAX_TURN, well below
    what the base can do.

Directions are the base's own, seen from the robot: forward is where Baxter looks, left
is Baxter's left, and turning is counter-clockwise seen from above for positive values.
"""

from __future__ import annotations

import threading
import time
from typing import NamedTuple

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.client import Client
from rclpy.node import Node
from std_msgs.msg import Empty

from mobility_base_core_msgs.msg import BatteryState, BumperState, Mode
from mobility_base_core_msgs.srv import GetMaxSpeed, SetMaxSpeed

from . import settings
from ._core import at_once, connecting, dataflow
from ._core.locks import locked, part_lock
from ._core.node import get_node
from ._core.services import call_service
from ._core.signals import Signal

_NS = 'mobility_base/'

# What the base reports as its mode: a short name, and what it means for a program.
MODES: dict[int, tuple[str, str]] = {
    Mode.MODE_ESTOP: ('estop', 'the e-stop is pressed'),
    Mode.MODE_DISABLED: ('disabled', 'the base is disabled: the joystick is off, out of '
                                     'range, or its left stick is down'),
    Mode.MODE_BATTERY_LIMP_HOME: ('battery_low', 'the battery is low; the base only moves slowly'),
    Mode.MODE_BATTERY_CRITICAL: ('battery_critical', 'the battery is nearly empty; the base does not move'),
    Mode.MODE_MOTOR_FAULT: ('motor_fault', 'a motor reports a fault'),
    Mode.MODE_WIRELESS: ('joystick', 'the joystick drives the base (its mode switch is on "Manual")'),
    Mode.MODE_TIMEOUT: ('waiting', 'the base waits for commands from a program'),
    Mode.MODE_VELOCITY: ('driving', 'the base follows speed commands from a program'),
    Mode.MODE_VELOCITY_RAW: ('driving_raw', 'the base follows raw speed commands from a program'),
}

_BATTERY_STATES: dict[int, str] = {
    BatteryState.BATTERY_CHARGED: 'charged',
    BatteryState.BATTERY_CHARGING: 'charging',
    BatteryState.BATTERY_DISCHARGING: 'discharging',
    BatteryState.BATTERY_WARN: 'low',
    BatteryState.BATTERY_LIMP_HOME: 'very low',
    BatteryState.BATTERY_CRITICAL: 'critical',
}

_SIDES: tuple[tuple[int, str], ...] = (
    (BumperState.BUMPER_FRONT, 'front'),
    (BumperState.BUMPER_LEFT, 'left'),
    (BumperState.BUMPER_REAR, 'rear'),
    (BumperState.BUMPER_RIGHT, 'right'),
)
_CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')


class Battery(NamedTuple):
    """The base's battery: voltage (V), current (A, negative while it is being used),
    charge and capacity (Ah), percent (0 to 100) and state ('charged', 'charging',
    'discharging', 'low', 'very low', 'critical')."""
    voltage: float
    current: float
    charge: float
    capacity: float
    percent: int
    state: str


class Speed(NamedTuple):
    """A speed of the base: forward and left in m/s, turn in rad/s."""
    forward: float
    left: float
    turn: float


def limited(forward: float, left: float, turn: float) -> Speed:
    """Return the speed cut down to the library's limits (settings.BASE_MAX_...).

    The two driving directions are cut together, so that the direction stays the same.
    """
    forward, left, turn = float(forward), float(left), float(turn)
    fastest = max(abs(forward), abs(left))
    if fastest > settings.BASE_MAX_SPEED:
        forward *= settings.BASE_MAX_SPEED / fastest
        left *= settings.BASE_MAX_SPEED / fastest
    turn = max(-settings.BASE_MAX_TURN, min(settings.BASE_MAX_TURN, turn))
    return Speed(forward, left, turn)


def pressed_sides(value: int) -> list[str]:
    """Return which sides of one bumper are pressed, from the number the base reports."""
    return [name for bit, name in _SIDES if value & bit]


class MobilityBase:
    """Interface class for the Dataspeed Mobility Base.

    Available methods:
        mode()            - what the base is doing, as a short name ('waiting', 'driving',
                            'joystick', 'disabled', 'estop', ...)
        mode_meaning()    - the same in a sentence
        battery()         - voltage, percent, state, ...
        bumpers()         - which bumpers are pressed, and on which side
        bumper_pressed()  - True if any bumper is pressed
        speed()           - the speed the base measures
        drive()           - drive at a speed for a time, then stop
        set_velocity()    - one speed command, for a loop of your own
                            (both take ignore_joystick=True)
        stop()            - stop
        max_speed(), set_max_speed()  - the speed limit inside the base itself
    Signals:
        mode_changed      - the new mode name, whenever the mode changes
        bumpers_changed   - the bumpers, whenever one is pressed or released
    """

    def __init__(self, node: Node | None = None) -> None:
        """
        Parameters:
            node: optional ROS 2 node to use (default: the shared background node)
        """
        self._node = get_node(node)
        self._lock = threading.Lock()  # guards the stored values
        # Held by drive(): one driving command at a time.
        self._part_lock = part_lock('mobility_base', self._node)
        self._mode: int | None = None
        self._battery: Battery | None = None
        self._bumpers: dict[str, list[str]] | None = None
        self._speed: Speed | None = None
        self._get_max: Client | None = None  # created on first use
        self._set_max: Client | None = None
        # The measured speed arrives 250 times per second: the sign of a live connection.
        self._fresh = dataflow.Freshness('mobility base', dataflow.full_name(self._node, _NS + 'twist'))
        self.mode_changed: Signal[str] = Signal(keep='latest')
        self.bumpers_changed: Signal[dict[str, list[str]]] = Signal(keep='latest')

        self._pub_velocity = self._node.create_publisher(Twist, _NS + 'cmd_vel', 10)
        self._pub_ignore = self._node.create_publisher(Empty, _NS + 'suppress_wireless', 10)
        self._subscriptions = [
            self._node.create_subscription(Mode, _NS + 'mode', self._on_mode, 10),
            self._node.create_subscription(BatteryState, _NS + 'battery', self._on_battery, 10),
            self._node.create_subscription(BumperState, _NS + 'bumper_states', self._on_bumpers, 10),
            self._node.create_subscription(TwistStamped, _NS + 'twist', self._on_twist, 10),
        ]

        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        # The mode comes twice per second, the speed 250 times: both show that the base's
        # bridge and the base's own programs are running. The battery reports only every
        # two seconds and the bumpers when they change, so they are not waited for.
        dataflow.wait_for(
            lambda: self._mode is not None and self._speed is not None,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'Failed to get the state of the mobility base from '
                        f"{dataflow.full_name(self._node, _NS + 'mode')} and "
                        f"{dataflow.full_name(self._node, _NS + 'twist')}. Is the bridge for "
                        'the BASE running (it is a second '
                        'bridge, see docs/mobility-base.md), and are the programs on the '
                        "base's own computer started?",
        )

    # --- What arrives from the base ---------------------------------------------------------

    def _on_mode(self, msg: Mode) -> None:
        with self._lock:
            changed = self._mode is not None and msg.mode != self._mode
            self._mode = msg.mode
        if changed:
            self.mode_changed(_mode_name(msg.mode))

    def _on_battery(self, msg: BatteryState) -> None:
        battery = Battery(
            voltage=msg.voltage,
            current=msg.rate,
            charge=msg.charge,
            capacity=msg.capacity,
            percent=int(msg.percent),
            state=_BATTERY_STATES.get(msg.state, f'unknown ({msg.state})')
        )
        with self._lock:
            self._battery = battery

    def _on_bumpers(self, msg: BumperState) -> None:
        bumpers = {corner: pressed_sides(getattr(msg, corner)) for corner in _CORNERS}
        with self._lock:
            changed = self._bumpers is not None and bumpers != self._bumpers
            self._bumpers = bumpers
        if changed:
            self.bumpers_changed(bumpers)

    def _on_twist(self, msg: TwistStamped) -> None:
        self._fresh.mark()
        speed = Speed(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        with self._lock:
            self._speed = speed

    # --- Reading ----------------------------------------------------------------------------

    def mode(self) -> str:
        """Return what the base is doing, as a short name.

        Returns:
            'estop', 'disabled', 'joystick', 'waiting' (for a program's commands),
            'driving' (following a program), 'battery_low', 'battery_critical',
            'motor_fault' or 'driving_raw'
        """
        with self._lock:
            return _mode_name(connecting.connected(self._mode, 'the mobility base'))

    def mode_meaning(self) -> str:
        """Return the base's mode as a sentence, e.g. 'the e-stop is pressed'."""
        with self._lock:
            mode = connecting.connected(self._mode, 'the mobility base')
        return MODES.get(mode, ('', f'a mode this library does not know ({mode})'))[1]

    def battery(self, timeout: float = 5.0) -> Battery:
        """Return the state of the base's battery.

        The base reports it every two seconds, so the first call may wait a moment.

        Parameters:
            timeout: seconds to wait for the first report (default 5)

        Returns:
            Battery(voltage, current, charge, capacity, percent, state)
        """
        dataflow.wait_for(
            lambda: self._battery is not None,
            timeout=timeout,
            timeout_msg=f"No battery report from the mobility base on {dataflow.full_name(self._node, _NS + 'battery')} within "
                        f'{timeout:g} s. ' + dataflow.BRIDGE_HINT,
        )
        with self._lock:
            return connecting.connected(self._battery, 'the mobility base')

    def bumpers(self) -> dict[str, list[str]]:
        """Return which bumpers are pressed.

        Returns:
            dict with the four bumpers 'front_left', 'front_right', 'rear_left' and
            'rear_right'; each value lists the sides on which that bumper is pressed
            ('front', 'left', 'rear', 'right'), empty if it is not.
            Before the base has reported its bumpers once, all are taken as not pressed.
        """
        with self._lock:
            if self._bumpers is None:
                return {corner: [] for corner in _CORNERS}
            return {corner: list(sides) for corner, sides in self._bumpers.items()}

    def bumper_pressed(self) -> bool:
        """Return True if any bumper is pressed."""
        return any(self.bumpers().values())

    def speed(self) -> Speed:
        """Return the speed the base measures at its wheels.

        Returns:
            Speed(forward, left, turn): m/s, m/s, rad/s
        """
        with self._lock:
            return connecting.connected(self._speed, 'the mobility base')

    # --- Driving ----------------------------------------------------------------------------

    def set_velocity(
        self,
        forward: float = 0.0,
        left: float = 0.0,
        turn: float = 0.0,
        ignore_joystick: bool = False
    ) -> None:
        """Send one speed command. For a loop of your own; otherwise use drive().

        The base follows it for a fifth of a second and then stops, so call this at
        least ten times per second for as long as the base should move. The speed is
        cut down to the library's limits (settings.BASE_MAX_SPEED, BASE_MAX_TURN).

        Parameters:
            forward: m/s, negative for backward
            left:    m/s sideways to the left, negative for right
            turn:    rad/s, positive turns counter-clockwise seen from above
            ignore_joystick: True: also tell the base to ignore its joystick for the
                     next fifth of a second (see the text at the top of this file: the
                     e-stop button is then the way to stop the base by hand)
        """
        speed = limited(forward, left, turn)
        if ignore_joystick:
            self._pub_ignore.publish(Empty())
        msg = Twist()
        msg.linear.x = speed.forward
        msg.linear.y = speed.left
        msg.angular.z = speed.turn
        self._pub_velocity.publish(msg)

    def stop(self) -> None:
        """Stop the base: send speed zero a few times.

        (Sending nothing stops it as well, a fifth of a second later.)
        """
        for _ in range(3):
            self.set_velocity(0.0, 0.0, 0.0)
            time.sleep(1.0 / settings.BASE_COMMAND_RATE)

    @at_once.command
    @locked
    def drive(
        self,
        forward: float = 0.0,
        left: float = 0.0,
        turn: float = 0.0,
        seconds: float = 1.0,
        ignore_joystick: bool = False
    ) -> None:
        """Drive at a speed for a time, then stop. Returns when the base has stopped.

        Before: joystick on, left stick up, mode switch on "Computer" (or
        ignore_joystick=True); someone at the base's e-stop; free space around the base
        and the robot's arms tucked or clear.

        Parameters:
            forward: m/s, negative for backward
            left:    m/s sideways to the left, negative for right
            turn:    rad/s, positive turns counter-clockwise seen from above
            seconds: how long to drive (default 1)
            ignore_joystick: True: the base follows this drive whatever the joystick's
                     switches say, also with the joystick off. The joystick cannot be
                     relied on to stop the base meanwhile; the e-stop button can.

        Raises:
            ConnectionError if nothing listens to the command or the connection to the
            base is lost on the way; RuntimeError if the base does not follow the program
            (the message says in which mode it is) or a bumper is pressed. In every case
            the base is told to stop first.
        """
        if seconds <= 0:
            raise ValueError(f'seconds must be more than 0, not {seconds!r}')
        speed = limited(forward, left, turn)
        if speed == Speed(0.0, 0.0, 0.0):
            self.stop()
            return
        if ignore_joystick and not dataflow.wait_for_listener(self._pub_ignore):
            raise ConnectionError(
                f'The joystick cannot be ignored: nothing listens on {self._pub_ignore.topic_name}. '
                "Is it in the base's bridge list (bridge_config/mobility_base.yaml)?")
        if not dataflow.wait_for_listener(self._pub_velocity):
            raise ConnectionError(
                f"The base cannot be driven: nothing listens on {self._pub_velocity.topic_name}. "
                "Is the bridge for the BASE running (docs/mobility-base.md)?")
        if self.bumper_pressed():
            raise RuntimeError(f'The base is not driven: a bumper is pressed ({self._pressed_text()}).')

        start = time.monotonic()
        follow_by = start + settings.BASE_FOLLOW_TIME

        def done() -> bool:
            self._fresh.require()
            now = time.monotonic()
            if self.bumper_pressed():
                raise RuntimeError(f'Stopped: a bumper is pressed ({self._pressed_text()}).')
            # "Follows" is judged by the mode the base reports or, in case a base reports
            # its modes differently than documented, by its wheels turning.
            if now > follow_by and self.mode() != 'driving' and not self._moving():
                raise RuntimeError(
                    f"The base does not follow the program: {self.mode_meaning()} (mode "
                    f"'{self.mode()}'). A program can drive it when the joystick is on, its "
                    'left stick is up and its mode switch is on "Computer", or with '
                    'ignore_joystick=True.')
            return now - start >= seconds

        try:
            dataflow.wait_for(
                done,
                timeout=seconds + 5.0,
                rate=settings.BASE_COMMAND_RATE,
                body=lambda: self.set_velocity(*speed, ignore_joystick=ignore_joystick),
            )
        finally:
            self.stop()

    def _moving(self) -> bool:
        speed = self.speed()
        return max(abs(speed.forward), abs(speed.left), abs(speed.turn)) > 0.005

    def _pressed_text(self) -> str:
        return ', '.join(f"{corner}: {' and '.join(sides)}"
                         for corner, sides in self.bumpers().items() if sides)

    # --- The speed limit inside the base ----------------------------------------------------

    def max_speed(self) -> tuple[float, float]:
        """Return the speed limit set inside the base itself.

        Returns:
            (m/s, rad/s)
        """
        if self._get_max is None:
            self._get_max = self._node.create_client(GetMaxSpeed, _NS + 'get_max_speed')
        response = call_service(self._get_max, GetMaxSpeed.Request())
        return float(response.linear), float(response.angular)

    def set_max_speed(self, linear: float, angular: float) -> None:
        """Set the speed limit inside the base itself.

        It applies to every program and to the joystick, until the base is restarted.
        The library's own limits (settings.BASE_MAX_...) apply in addition.

        Parameters:
            linear:  m/s
            angular: rad/s
        """
        if self._set_max is None:
            self._set_max = self._node.create_client(SetMaxSpeed, _NS + 'set_max_speed')
        call_service(self._set_max, SetMaxSpeed.Request(linear=float(linear), angular=float(angular)))


def _mode_name(mode: int) -> str:
    return MODES.get(mode, (f'unknown ({mode})', ''))[0]
