"""Tests for Signal and its per-callback threads: no robot, bridge or ROS needed.

Connected functions run in their own threads, so the tests wait for them with events and
short timeouts instead of checking results right after firing.
"""

import threading
import time

from baxter_interface._core.signals import Signal

WAIT = 2.0  # seconds to wait for a callback thread; far more than it ever needs


def test_calls_connected_function_with_arguments():
    received = []
    done = threading.Event()

    def handler(value):
        received.append(value)
        done.set()

    signal = Signal()
    signal.connect(handler)
    signal(42)

    assert done.wait(WAIT)
    assert received == [42]


def test_calls_connected_method():
    class Counter:
        def __init__(self):
            self.count = 0
            self.done = threading.Event()

        def bump(self, step):
            self.count += step
            if self.count == 5:
                self.done.set()

    counter = Counter()
    signal = Signal()
    signal.connect(counter.bump)
    signal(2)
    signal(3)

    assert counter.done.wait(WAIT)
    assert counter.count == 5


def test_lambda_stays_connected():
    # In the ROS 1 SDK a lambda passed directly to connect() was never called.
    done = threading.Event()
    signal = Signal()
    signal.connect(lambda: done.set())
    signal()

    assert done.wait(WAIT)


def test_calls_of_one_function_run_in_order():
    received = []
    done = threading.Event()

    def handler(value):
        time.sleep(0.01)  # slower than the signal fires
        received.append(value)
        if value == 9:
            done.set()

    signal = Signal()
    signal.connect(handler)
    for i in range(10):
        signal(i)

    assert done.wait(WAIT)
    assert received == list(range(10))


def test_firing_does_not_wait_for_a_slow_function():
    release = threading.Event()
    signal = Signal()
    signal.connect(lambda: release.wait(WAIT))

    start = time.monotonic()
    signal()
    elapsed = time.monotonic() - start
    release.set()

    assert elapsed < 0.1


def test_slow_function_does_not_delay_another():
    release = threading.Event()
    fast_done = threading.Event()
    signal = Signal()
    signal.connect(lambda: release.wait(WAIT))  # blocks until released
    signal.connect(fast_done.set)
    signal()

    try:
        assert fast_done.wait(WAIT)  # runs although the first one is still blocked
    finally:
        release.set()


def test_function_may_wait_for_what_the_firing_thread_does_next():
    # The case this design exists for: a callback that waits for an update which only the
    # thread that fired the signal can deliver (e.g. limb.move_to_neutral() in a button
    # callback, waiting for joint states).
    state = {'arrived': False}
    result = []
    done = threading.Event()

    def handler():
        deadline = time.monotonic() + WAIT
        while not state['arrived'] and time.monotonic() < deadline:
            time.sleep(0.005)
        result.append(state['arrived'])
        done.set()

    signal = Signal()
    signal.connect(handler)
    signal()                     # the "receiving thread" fires the signal ...
    time.sleep(0.05)
    state['arrived'] = True      # ... and afterwards delivers the update

    assert done.wait(WAIT)
    assert result == [True]


def test_disconnect_stops_calls():
    received = []
    other_done = threading.Event()

    def handler(value):
        received.append(value)

    signal = Signal()
    signal.connect(handler)
    signal.connect(lambda _value: other_done.set())
    signal.disconnect(handler)
    signal('ignored')

    assert other_done.wait(WAIT)  # the signal was delivered ...
    time.sleep(0.05)
    assert received == []         # ... but not to the disconnected function


def test_connecting_twice_calls_once():
    received = []
    done = threading.Event()

    def handler(value):
        received.append(value)
        done.set()

    signal = Signal()
    signal.connect(handler)
    signal.connect(handler)
    signal(1)

    assert done.wait(WAIT)
    time.sleep(0.05)
    assert received == [1]


def test_error_in_function_does_not_stop_later_calls(capsys):
    received = []
    done = threading.Event()

    def handler(value):
        if value == 'bad':
            raise ValueError('deliberate error in a test')
        received.append(value)
        done.set()

    signal = Signal()
    signal.connect(handler)
    signal('bad')
    signal('good')

    assert done.wait(WAIT)
    assert received == ['good']
    assert 'deliberate error in a test' in capsys.readouterr().err


def test_keep_latest_skips_firings_while_busy():
    received = []
    started = threading.Event()
    release = threading.Event()
    done = threading.Event()

    def handler(value):
        received.append(value)
        if value == 0:
            started.set()
            release.wait(WAIT)  # busy while frames 1..9 arrive
        else:
            done.set()

    signal = Signal(keep='latest')
    signal.connect(handler)
    signal(0)
    assert started.wait(WAIT)
    for i in range(1, 10):
        signal(i)
    release.set()

    assert done.wait(WAIT)
    time.sleep(0.05)
    assert received == [0, 9]  # the first, then only the newest of the rest
