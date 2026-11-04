"""Waiting for a condition, with a time limit.

Ported from baxter_dataflow.wait_for (baxter_interface v0.7.0, Copyright (c) 2013,
Rethink Robotics, BSD license; see LICENSE). Changes: uses wall-clock time instead of ROS
time, and raises TimeoutError instead of OSError(ETIMEDOUT).
"""

from __future__ import annotations

import time
from typing import Any, Callable, TypeVar

from rclpy.node import Node
from rclpy.publisher import Publisher
from .node import ros_is_running

from .. import settings

Message = TypeVar('Message')


def wait_for(
    test: Callable[[], Any],
    timeout: float = 1.0,
    raise_on_error: bool = True,
    rate: float = 100,
    timeout_msg: str = 'timeout expired',
    body: Callable[[], object] | None = None
) -> bool:
    """Call `test()` until it returns True.
    
    Parameters:
        test:        function without arguments, returns True when done
        timeout:     seconds before giving up; negative or inf waits forever
        raise_on_error: raise TimeoutError on timeout (else return False)
        rate:        how often to check, in Hz
        timeout_msg: message for the TimeoutError
        body:        optional function called on every check (e.g. to publish a command)

    Returns:
        True when `test()` succeeded, False on timeout if raise_on_error is False.
    """
    end = time.monotonic() + timeout
    forever = timeout < 0 or timeout == float('inf')
    period = 1.0 / rate
    # The checks are timed against a fixed beat, not "work, then sleep one period": with
    # the latter, the time test() and body() take adds to every round, and a loop meant
    # to run 100 times per second ran 77 times on the real robot, sending arm commands
    # unevenly.
    next_check = time.monotonic()
    while not test():
        if not ros_is_running():
            if raise_on_error:
                raise RuntimeError('ROS 2 was shut down')
            return False
        if not forever and time.monotonic() >= end:
            if raise_on_error:
                raise TimeoutError(timeout_msg)
            return False
        if body is not None:
            body()
        next_check += period
        pause = next_check - time.monotonic()
        if pause > 0:
            time.sleep(pause)
        else:
            next_check = time.monotonic()  # fell behind: carry on from now, don't rush to catch up
    return True


# Added to error messages about a missing connection.
BRIDGE_HINT = ('Is the bridge running (docker compose up -d bridge), and the robot switched '
               'on and reachable? With BRIDGE_MODE=fixed_list the topic must also be listed '
               'in bridge_config/topics.yaml.')


def wait_for_listener(publisher: Publisher, timeout: float | None = None) -> bool:
    """Wait until someone listens to a publisher, normally the bridge.

    Why: a message published before the bridge has connected to this program is lost.
    The bridge connects a topic only once it has noticed that a program publishes on it,
    which takes a second or two.

    Parameters:
        publisher: publisher created with node.create_publisher(...)
        timeout:   seconds to wait (default: settings.CONNECT_TIMEOUT)

    Returns:
        True if someone listens, False if nobody appeared in time.
    """
    return wait_for(
        lambda: publisher.get_subscription_count() > 0,
        timeout=settings.CONNECT_TIMEOUT if timeout is None else timeout,
        raise_on_error=False
    )


def publish_once(publisher: Publisher, msg: Any, what: str = 'command', confirm: float = 2.0) -> None:
    """Send a message that is sent only once, and make sure it reached the bridge.

    For commands that are not repeated: a setting, a picture, the first message of a
    command. (Commands that are sent 100 times per second while the library waits for the
    robot don't need this: a lost one is followed by the next.)

    Two things are checked, and a ConnectionError says so if one fails, instead of the
    program carrying on as if the robot had received the command:
      1. someone listens (the bridge has connected the topic) before the message is sent;
      2. the listener confirmed that it received the message.

    The first message on a publisher also waits settings.BRIDGE_SETTLE_TIME after the
    bridge started listening: the robot still has to connect to the bridge then, and the
    bridge drops what arrives before that (seen on the robot, 7 Oct 2026: a halo colour
    that was set in one run and not in the next).

    Parameters:
        publisher: publisher created with node.create_publisher(...)
        msg:       the message
        what:      what the message is, for the error text, e.g. 'speed setting'
        confirm:   seconds to wait for the confirmation (default 2)

    Raises:
        ConnectionError if nobody listens or the message was not confirmed.
    """
    topic = getattr(publisher, 'topic_name', 'its topic')
    if not wait_for_listener(publisher):
        raise ConnectionError(
            f'The {what} was not sent: nothing listens on {topic}. ' + BRIDGE_HINT)
    _let_the_robot_connect(publisher)
    publisher.publish(msg)
    if not _confirmed(publisher, confirm):
        raise ConnectionError(
            f'The {what} was sent on {topic}, but the bridge did not confirm receiving it '
            f'within {confirm:g} s. It may not have reached the robot. ' + BRIDGE_HINT)


def _let_the_robot_connect(publisher: Publisher) -> None:
    """Before a publisher's first one-off message: give the robot time to reach the bridge.

    The time counts from the first moment this function saw the publisher with a
    listener, so only the first message on a publisher waits.
    """
    now = time.monotonic()
    since = getattr(publisher, '_baxter_listener_seen', None)
    if since is None:
        since = now
        try:
            setattr(publisher, '_baxter_listener_seen', now)
        except AttributeError:
            pass  # a publisher that takes no notes: waits every time, which is safe
    pause = since + settings.BRIDGE_SETTLE_TIME - now
    if pause > 0:
        time.sleep(pause)


def _confirmed(publisher: Publisher, seconds: float) -> bool:
    """True once every listener has acknowledged all messages of this publisher."""
    try:
        from rclpy.duration import Duration
        return bool(publisher.wait_for_all_acked(Duration(seconds=seconds)))
    except (AttributeError, ImportError):
        return True  # an rclpy without this check: nothing to ask


def full_name(node: Node, name: str) -> str:
    """Return a topic name as the node uses it: complete, with the robot's name if it has one.

    For messages to the user. 'robot/joint_states' is '/robot/joint_states' for a robot
    without a name and '/one/robot/joint_states' for the robot "one".
    """
    try:
        return str(node.resolve_topic_name(name))
    except Exception:
        return name if name.startswith('/') else '/' + name


class Freshness:
    """Remembers when a robot part last sent its state, to notice a lost connection.

    The classes keep the newest state they received. If the connection is lost (bridge
    stopped, cable out, robot off) that state silently stays as it was. A class calls
    mark() for every state message, and require() where it waits for the robot: after
    settings.STATE_TIMEOUT seconds of silence that raises a ConnectionError instead of
    waiting on with outdated values.
    """

    def __init__(self, what: str, topic: str) -> None:
        """
        Parameters:
            what:  whose state this is, for the error text, e.g. 'left arm'
            topic: the topic the state arrives on
        """
        self._what = what
        self._topic = topic
        self._last: float | None = None

    def mark(self) -> None:
        """Call for every state message that arrives."""
        self._last = time.monotonic()

    def age(self) -> float:
        """Seconds since the last state message (infinite if none arrived yet)."""
        return float('inf') if self._last is None else time.monotonic() - self._last

    def require(self) -> None:
        """Raise ConnectionError if the state is older than settings.STATE_TIMEOUT."""
        age = self.age()
        if age > settings.STATE_TIMEOUT:
            raise ConnectionError(
                f'No state from the {self._what} for {age:.1f} s ({self._topic}): the '
                'connection to the robot is lost. ' + BRIDGE_HINT)


def wait_for_message(node: Node, msg_type: type[Message], topic: str, timeout: float = 5.0) -> Message:
    """Wait for one message on a topic and return it (like rospy.wait_for_message).

    Parameters:
        node:     the ROS 2 node to subscribe with (it must be spinning, as the shared
                  background node is)
        msg_type: message class, e.g. sensor_msgs.msg.JointState
        topic:    topic name
        timeout:  seconds to wait (default 5)

    Returns:
        the message

    Raises:
        TimeoutError if no message arrives in time.
    """
    received: list[Message] = []
    subscription = node.create_subscription(
        msg_type,
        topic,
        received.append,
        10
    )
    try:
        wait_for(
            lambda: len(received) > 0,
            timeout=timeout,
            timeout_msg=f'No message on {full_name(node, topic)} within {timeout:g} s. ' + BRIDGE_HINT,
        )
    finally:
        node.destroy_subscription(subscription)
    return received[0]
