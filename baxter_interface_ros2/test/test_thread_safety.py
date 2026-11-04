"""Tests for the thread-safety helpers: no robot, bridge or ROS needed."""

import threading
import time

from baxter_interface._core import callbacks
from baxter_interface._core.callbacks import CallbackWorker
from baxter_interface._core.locks import locked, part_lock

WAIT = 2.0


def test_part_lock_is_the_same_for_the_same_name():
    assert part_lock('test/arm') is part_lock('test/arm')
    assert part_lock('test/arm') is not part_lock('test/other-arm')


def test_part_lock_can_be_taken_again_by_the_same_thread():
    lock = part_lock('test/reentrant')
    with lock:
        with lock:  # e.g. calibrate() calling reset()
            pass


class Part:
    """Stands in for a robot part whose commands must not overlap."""

    def __init__(self, name):
        self._part_lock = part_lock(name)
        self.running = 0
        self.most_at_once = 0

    @locked
    def command(self):
        self.running += 1
        self.most_at_once = max(self.most_at_once, self.running)
        time.sleep(0.02)
        self.running -= 1

    @locked
    def command_twice(self):
        self.command()
        self.command()


def test_locked_methods_of_one_part_never_overlap():
    first = Part('test/shared')
    second = Part('test/shared')  # another object for the same part

    threads = [threading.Thread(target=part.command) for part in (first, second) * 4]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(WAIT)

    assert first.most_at_once == 1
    assert second.most_at_once == 1


def test_locked_methods_of_different_parts_do_overlap():
    left = Part('test/left')
    right = Part('test/right')
    both_inside = threading.Barrier(2, timeout=WAIT)

    def command(part):
        with part._part_lock:
            both_inside.wait()  # only passes if both threads hold their lock at once

    threads = [threading.Thread(target=command, args=(part,)) for part in (left, right)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(WAIT)

    assert not both_inside.broken


def test_locked_method_may_call_another_locked_method():
    part = Part('test/nested')
    part.command_twice()

    assert part.most_at_once == 1


def test_stuck_callback_drops_oldest_calls_past_the_limit(monkeypatch, capsys):
    monkeypatch.setattr(callbacks, 'MAX_WAITING_CALLS', 5)
    received = []
    started = threading.Event()
    release = threading.Event()
    done = threading.Event()

    def handler(value):
        received.append(value)
        if value == 0:
            started.set()
            release.wait(WAIT)  # stuck while 20 more calls arrive
        if value == 20:
            done.set()

    worker = CallbackWorker(handler)
    worker.submit(0)
    assert started.wait(WAIT)
    for i in range(1, 21):
        worker.submit(i)
    release.set()

    assert done.wait(WAIT)
    assert received == [0, 16, 17, 18, 19, 20]  # the 5 newest of the waiting calls
    assert 'not keeping up' in capsys.readouterr().err


# --- setup ------------------------------------------------------------------------

def test_waits_run_at_once_outside_a_setup_block():
    from baxter_interface._core import connecting
    done = []

    connecting.finish_or_defer(lambda: done.append('now'))

    assert done == ['now']


def test_waits_are_put_off_to_the_end_of_a_setup_block():
    from baxter_interface._core import connecting
    done = []

    with connecting.setup():
        connecting.finish_or_defer(lambda: done.append('first'))
        connecting.finish_or_defer(lambda: done.append('second'))
        assert done == []  # nothing has waited yet

    assert done == ['first', 'second']


def test_an_error_in_the_block_skips_the_waits_and_ends_the_block():
    from baxter_interface._core import connecting
    done = []

    try:
        with connecting.setup():
            connecting.finish_or_defer(lambda: done.append('never'))
            raise ValueError('deliberate error in a test')
    except ValueError:
        pass
    connecting.finish_or_defer(lambda: done.append('after'))  # no block is open any more

    assert done == ['after']
