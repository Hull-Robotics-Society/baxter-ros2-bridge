"""Tests for the Mobility Base class that need no base and no ROS 2 traffic: the speed
limits, reading the bumpers, and that drive() repeats its command, stops at the end and
gives up with an explanation when the base does not follow."""

import threading

import pytest

from baxter_interface import mobility_base as module
from baxter_interface import settings
from baxter_interface._core import dataflow
from baxter_interface.mobility_base import MobilityBase, Speed, limited, pressed_sides
from mobility_base_core_msgs.msg import Mode


def test_a_speed_within_the_limits_is_kept():
    assert limited(0.1, -0.05, 0.2) == Speed(0.1, -0.05, 0.2)


def test_too_fast_is_cut_down_and_keeps_its_direction():
    speed = limited(3.0, 1.5, 0.0)     # twice as fast forward as to the left

    assert speed.forward == pytest.approx(settings.BASE_MAX_SPEED)
    assert speed.left == pytest.approx(settings.BASE_MAX_SPEED / 2)


def test_turning_is_cut_down_on_both_sides():
    assert limited(0.0, 0.0, 9.0).turn == settings.BASE_MAX_TURN
    assert limited(0.0, 0.0, -9.0).turn == -settings.BASE_MAX_TURN


def test_a_bumper_number_names_the_pressed_sides():
    assert pressed_sides(0) == []
    assert pressed_sides(1) == ['front']
    assert pressed_sides(1 | 2) == ['front', 'left']
    assert pressed_sides(4 | 8) == ['rear', 'right']


class Publisher:
    """Stands in for the command publisher: remembers what was sent."""
    topic_name = '/mobility_base/cmd_vel'

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append((msg.linear.x, msg.linear.y, msg.angular.z))

    def get_subscription_count(self):
        return 1


class Counter:
    """Stands in for the "ignore the joystick" publisher: counts what was sent."""
    topic_name = '/mobility_base/suppress_wireless'

    def __init__(self):
        self.count = 0

    def publish(self, msg):
        self.count += 1

    def get_subscription_count(self):
        return 1


def a_base(mode=Mode.MODE_TIMEOUT, follows=True):
    """A MobilityBase without ROS 2: its state is set here instead of arriving.

    Parameters:
        mode:    the mode the base reports before the first command
        follows: whether it changes to "following a program" once commands arrive
    """
    base = MobilityBase.__new__(MobilityBase)
    base._lock = threading.Lock()
    base._part_lock = threading.RLock()
    base._mode = mode
    base._battery = None
    base._bumpers = None
    base._speed = Speed(0.0, 0.0, 0.0)
    base._fresh = dataflow.Freshness('mobility base', '/mobility_base/twist')
    base._fresh.require = lambda: None          # the connection counts as alive
    base._pub_velocity = Publisher()
    base._pub_ignore = Counter()
    if follows:
        publish = base._pub_velocity.publish

        def publish_and_follow(msg):
            publish(msg)
            if (msg.linear.x, msg.linear.y, msg.angular.z) != (0.0, 0.0, 0.0):
                base._mode = Mode.MODE_VELOCITY
        base._pub_velocity.publish = publish_and_follow
    return base


@pytest.fixture(autouse=True)
def quick(monkeypatch):
    monkeypatch.setattr(settings, 'BASE_FOLLOW_TIME', 0.15)
    # No ROS 2 runs in these tests; the waiting loop would otherwise give up at once.
    monkeypatch.setattr(dataflow, 'ros_is_running', lambda: True)


def test_drive_repeats_the_command_and_ends_with_stop():
    base = a_base()

    base.drive(forward=0.1, seconds=0.5)

    sent = base._pub_velocity.sent
    moving = [s for s in sent if s != (0.0, 0.0, 0.0)]
    # 20 commands per second for half a second; the base would stop below 5 per second.
    assert 8 <= len(moving) <= 12
    assert set(moving) == {(0.1, 0.0, 0.0)}
    assert sent[-3:] == [(0.0, 0.0, 0.0)] * 3


def test_the_joystick_is_only_ignored_when_asked_for():
    base = a_base()

    base.drive(forward=0.1, seconds=0.3)

    assert base._pub_ignore.count == 0


def test_ignoring_the_joystick_is_repeated_with_every_command():
    base = a_base()

    base.drive(forward=0.1, seconds=0.5, ignore_joystick=True)

    moving = [s for s in base._pub_velocity.sent if s != (0.0, 0.0, 0.0)]
    # The base forgets it after a fifth of a second, so it goes out with each command.
    assert base._pub_ignore.count == len(moving)


def test_drive_never_asks_for_more_than_the_limits():
    base = a_base()

    base.drive(forward=5.0, turn=-5.0, seconds=0.2)

    assert all(abs(x) <= settings.BASE_MAX_SPEED and abs(turn) <= settings.BASE_MAX_TURN
               for x, _, turn in base._pub_velocity.sent)


def test_a_base_that_does_not_follow_is_explained_and_stopped():
    base = a_base(mode=Mode.MODE_WIRELESS, follows=False)

    with pytest.raises(RuntimeError, match='does not follow the program: the joystick drives the base'):
        base.drive(forward=0.1, seconds=2.0)

    assert base._pub_velocity.sent[-1] == (0.0, 0.0, 0.0)


def test_a_pressed_bumper_keeps_the_base_from_starting():
    base = a_base()
    base._bumpers = {'front_left': ['front'], 'front_right': [], 'rear_left': [], 'rear_right': []}

    with pytest.raises(RuntimeError, match='a bumper is pressed \\(front_left: front\\)'):
        base.drive(forward=0.1, seconds=0.5)

    assert base._pub_velocity.sent == []


def test_a_bumper_pressed_on_the_way_stops_the_base():
    base = a_base()
    publish = base._pub_velocity.publish

    def publish_then_hit(msg):
        publish(msg)
        if len(base._pub_velocity.sent) == 4:
            base._bumpers = {'front_left': [], 'front_right': ['front', 'right'], 'rear_left': [], 'rear_right': []}
    base._pub_velocity.publish = publish_then_hit

    with pytest.raises(RuntimeError, match='Stopped: a bumper is pressed'):
        base.drive(forward=0.1, seconds=2.0)

    assert base._pub_velocity.sent[-1] == (0.0, 0.0, 0.0)
    assert len(base._pub_velocity.sent) < 12


def test_speed_zero_is_just_a_stop():
    base = a_base()

    base.drive(seconds=1.0)

    assert set(base._pub_velocity.sent) == {(0.0, 0.0, 0.0)}


def test_every_mode_the_base_knows_has_a_name_and_a_meaning():
    numbers = [value for name, value in vars(Mode).items() if name.startswith('MODE_')]

    assert numbers and all(len(module.MODES[number]) == 2 for number in numbers)
