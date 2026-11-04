"""Tests for the at_once() block: no robot, bridge or ROS needed."""

import threading
import time

import pytest

from baxter_interface._core import at_once
from baxter_interface._core.at_once import at_once as block


class Part:
    """Stands in for a robot part with commands that take time."""

    def __init__(self, name, log):
        self.name = name
        self._part_lock = threading.RLock()
        self._log = log

    @at_once.command
    def move(self, label, seconds=0.2, fail=None):
        self._log.append(f'{self.name} {label} starts')
        time.sleep(seconds)
        if fail is not None:
            raise fail
        self._log.append(f'{self.name} {label} done')
        return label

    @at_once.command
    def twice(self, label):
        # A command that uses another command of the same part, as move_by() uses
        # move_to_pose(): the inner one must run directly, not be handed over again.
        return self.move(label + '-1', 0.05), self.move(label + '-2', 0.05)

    @at_once.command(wait_keyword='block')
    def grip(self, block=False):
        self._log.append(f'{self.name} grip block={block}')


def test_outside_a_block_commands_run_as_usual():
    log = []
    part = Part('left', log)

    assert part.move('a', 0.0) == 'a'
    assert log == ['left a starts', 'left a done']


def test_different_parts_run_at_the_same_time():
    log = []
    left, right, head = Part('left', log), Part('right', log), Part('head', log)

    start = time.monotonic()
    with block():
        left.move('a', 0.6)
        right.move('a', 0.3)
        head.move('a', 0.45)
        handed_over_after = time.monotonic() - start
    took = time.monotonic() - start

    # The limits are wide on purpose: a slow or briefly stalled machine must not fail this.
    assert handed_over_after < 0.3          # the lines in the block did not wait
    assert 0.55 < took < 1.2                # the block waited for the longest, not the sum (1.35)
    assert log[-1] == 'left a done'
    assert len(log) == 6


def test_commands_for_one_part_keep_their_order():
    log = []
    left, right = Part('left', log), Part('right', log)

    with block():
        left.move('first', 0.1)
        left.move('second', 0.1)
        right.move('only', 0.15)

    lefts = [line for line in log if line.startswith('left')]
    assert lefts == ['left first starts', 'left first done', 'left second starts', 'left second done']
    assert log.index('right only starts') < log.index('left first done')  # at the same time


def test_commands_return_nothing_inside_a_block():
    part = Part('left', [])

    with block():
        assert part.move('a', 0.0) is None


def test_command_inside_a_command_runs_directly():
    log = []
    part = Part('left', log)

    with block():
        part.twice('x')

    assert log == ['left x-1 starts', 'left x-1 done', 'left x-2 starts', 'left x-2 done']


def test_error_is_raised_at_the_end_after_the_others_finished():
    log = []
    left, right = Part('left', log), Part('right', log)

    with pytest.raises(TimeoutError, match='too slow'):
        with block():
            left.move('a', 0.05, fail=TimeoutError('too slow'))
            left.move('b', 0.05)        # same part, after the failure: skipped
            right.move('a', 0.3)        # another part: finishes

    assert 'right a done' in log
    assert 'left b starts' not in log


def test_error_in_the_block_itself_still_waits_for_what_was_started():
    log = []
    left = Part('left', log)

    with pytest.raises(KeyError):
        with block():
            left.move('a', 0.2)
            raise KeyError('typo')

    assert 'left a done' in log


def test_blocks_cannot_be_nested_and_end_cleanly():
    with block():
        with pytest.raises(RuntimeError, match='inside another'):
            with block():
                pass
    with block():   # the earlier blocks are over
        pass


def test_wait_keyword_is_switched_on_inside_a_block():
    log = []
    part = Part('left', log)

    part.grip()
    with block():
        part.grip()
    with block():
        part.grip(block=False)   # said explicitly: kept

    assert log == ['left grip block=False', 'left grip block=True', 'left grip block=False']


def test_own_functions_take_part():
    log = []
    left = Part('left', log)

    def blink(times):
        time.sleep(0.1)
        log.append(f'blinked {times}')

    with block() as group:
        left.move('a', 0.2)
        group.run(blink, 3)
        group.run(blink, 4)

    assert 'blinked 3' in log and 'blinked 4' in log and log[-1] == 'left a done'


def test_a_block_in_one_thread_does_not_affect_another():
    log = []
    part = Part('left', log)
    results = []

    with block():
        thread = threading.Thread(target=lambda: results.append(part.move('other', 0.0)))
        thread.start()
        thread.join()

    assert results == ['other']   # ran directly and returned its value
