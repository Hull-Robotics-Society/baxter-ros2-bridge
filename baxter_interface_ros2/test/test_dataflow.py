"""Tests that need neither the robot nor the bridge: colcon test --packages-select baxter_interface"""

import time

import pytest
import rclpy

from baxter_interface._core import dataflow


@pytest.fixture(autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_returns_true_when_condition_met():
    assert dataflow.wait_for(lambda: True)


def test_raises_timeout_error():
    start = time.monotonic()
    with pytest.raises(TimeoutError, match='nope'):
        dataflow.wait_for(lambda: False, timeout=0.2, timeout_msg='nope')
    assert time.monotonic() - start < 1.0


def test_returns_false_without_raising():
    assert dataflow.wait_for(lambda: False, timeout=0.1, raise_on_error=False) is False


def test_body_keeps_its_rate_even_if_it_takes_time():
    calls = []

    def body():
        calls.append(time.monotonic())
        time.sleep(0.005)  # half of the 10 ms period

    dataflow.wait_for(
        lambda: len(calls) >= 30,
        timeout=2.0,
        rate=100,
        body=body
    )
    took = calls[-1] - calls[0]

    # 29 periods of 10 ms. "Work, then sleep a whole period" would take 15 ms per round.
    assert took < 29 * 0.0125


def test_body_runs_until_condition_met():
    calls = []
    dataflow.wait_for(lambda: len(calls) >= 3, timeout=1.0, body=lambda: calls.append(1))
    assert len(calls) == 3


# --- commands that are sent once, and lost connections -------------------------------------

class FakePublisher:
    """Stands in for a ROS 2 publisher: counts what is published."""

    topic_name = '/test/topic'

    def __init__(self, listeners=1, confirms=True):
        self.listeners = listeners
        self.confirms = confirms
        self.published = []

    def get_subscription_count(self):
        return self.listeners

    def publish(self, msg):
        self.published.append(msg)

    def wait_for_all_acked(self, _timeout):
        return self.confirms


def test_publish_once_sends_when_someone_listens_and_confirms():
    publisher = FakePublisher()

    dataflow.publish_once(publisher, 'message')

    assert publisher.published == ['message']


def test_first_message_on_a_publisher_waits_for_the_robot_to_connect(monkeypatch):
    # The bridge listening is not enough: the robot still has to connect to the bridge.
    monkeypatch.setattr(dataflow.settings, 'BRIDGE_SETTLE_TIME', 0.3)
    publisher = FakePublisher()

    start = time.monotonic()
    dataflow.publish_once(publisher, 'first')
    first = time.monotonic() - start
    dataflow.publish_once(publisher, 'second')
    both = time.monotonic() - start

    assert publisher.published == ['first', 'second']
    assert first >= 0.3            # waited before the first message ...
    assert both - first < 0.2      # ... and not again before the second


def test_publish_once_refuses_when_nobody_listens(monkeypatch):
    monkeypatch.setattr(dataflow.settings, 'CONNECT_TIMEOUT', 0.2)
    publisher = FakePublisher(listeners=0)

    with pytest.raises(ConnectionError, match='nothing listens on /test/topic'):
        dataflow.publish_once(publisher, 'message', what='speed setting')

    assert publisher.published == []  # nothing was sent into the void


def test_publish_once_reports_a_message_that_was_not_confirmed():
    publisher = FakePublisher(confirms=False)

    with pytest.raises(ConnectionError, match='did not confirm'):
        dataflow.publish_once(publisher, 'message', confirm=0.1)


def test_wait_for_listener_notices_a_listener_that_appears_later(monkeypatch):
    monkeypatch.setattr(dataflow.settings, 'CONNECT_TIMEOUT', 2.0)
    publisher = FakePublisher(listeners=0)
    start = time.monotonic()

    def listener_appears():  # asked by wait_for 100 times per second
        return 1 if time.monotonic() - start > 0.2 else 0

    publisher.get_subscription_count = listener_appears

    assert dataflow.wait_for_listener(publisher) is True
    assert time.monotonic() - start < 1.0


def test_freshness_reports_silence(monkeypatch):
    monkeypatch.setattr(dataflow.settings, 'STATE_TIMEOUT', 0.1)
    fresh = dataflow.Freshness('left arm', '/robot/joint_states')

    with pytest.raises(ConnectionError, match='left arm'):
        fresh.require()  # nothing has arrived yet

    fresh.mark()
    fresh.require()  # just arrived: fine
    assert fresh.age() < 0.1

    time.sleep(0.15)
    with pytest.raises(ConnectionError, match='connection to the robot is lost'):
        fresh.require()
