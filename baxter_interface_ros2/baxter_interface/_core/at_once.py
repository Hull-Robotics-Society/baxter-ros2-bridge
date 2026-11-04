"""Commanding several parts of the robot at the same time: the at_once() block.

A command that waits for the robot, such as left.move_to_neutral(), returns when the arm
has arrived. Two of them in a row therefore run one after the other. Inside an at_once()
block they only start, and the block waits at its end until all of them are done:

    with baxter_interface.at_once():
        left.move_to_neutral()
        right.move_to_neutral()
        head.set_pan(0.5)
    # here both arms and the head have arrived, however long each of them took

How it works: each command in the block is handed to a helper thread and the program
moves on to the next line. There is one helper thread per robot part, so:

  - commands for different parts (left arm, right arm, head, a gripper) run at the same
    time;
  - several commands for the same part run in the order they were written, one after
    the other, while the other parts carry on.

If a command fails, the later commands for that part are skipped, the other parts finish
what they were given, and the error is raised at the end of the block.

Things to know:
  - Inside the block the commands return nothing (they have not happened yet).
  - Only commands that wait for the robot are handed over (moves of an arm, the head, a
    gripper, opening a camera). Everything else, such as reading a joint angle, happens
    at once, while the parts are moving.
  - Functions of your own can take part: see Group.run().
"""

from __future__ import annotations

import contextlib
import functools
import queue
import threading
from typing import Any, Callable, Iterator, ParamSpec, TypeVar, overload

P = ParamSpec('P')
R = TypeVar('R')

# The block that is open in each thread, if any. Kept per thread: a block opened in the
# main program does not affect functions running in other threads, including the helper
# threads themselves (inside them, commands run directly).
_local = threading.local()


class Group:
    """The commands of one at_once() block. Handed out by `with at_once() as group:`."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._queues: dict[object, queue.Queue] = {}     # part -> queue of (description, function)
        self._threads: list[threading.Thread] = []
        self._errors: list[tuple[str, BaseException]] = []  # in the order they happened
        self._skipped: list[str] = []    # descriptions of commands not started after a failure

    def run(self, function: Callable[..., object], *args: Any, **kwargs: Any) -> None:
        """Run a function of your own alongside the other commands of the block.

            def blink():
                for _ in range(5):
                    navigator.inner_led = not navigator.inner_led
                    time.sleep(0.3)

            with baxter_interface.at_once() as group:
                left.move_to_neutral()
                group.run(blink)

        The function gets a thread of its own; the block waits for it too.
        """
        name = getattr(function, '__name__', 'function')
        self._submit(object(), f'{name}()', lambda: function(*args, **kwargs))

    def _submit(self, part: object, description: str, function: Callable[[], object]) -> None:
        with self._lock:
            waiting = self._queues.get(part)
            if waiting is None:
                waiting = self._queues[part] = queue.Queue()
                thread = threading.Thread(
                    target=self._work,
                    args=(waiting,),
                    daemon=True
                )
                self._threads.append(thread)
                thread.start()
        waiting.put((description, function))

    def _work(self, waiting: queue.Queue) -> None:
        failed = False
        while True:
            item = waiting.get()
            if item is None:  # the block has ended and everything before this is done
                return
            description, function = item
            if failed:
                self._skipped.append(description)
                continue
            try:
                function()
            except BaseException as error:  # reported by _finish(), in the block's thread
                failed = True
                self._errors.append((description, error))

    def _finish(self) -> None:
        """Wait until every command is done."""
        for waiting in list(self._queues.values()):
            waiting.put(None)
        for thread in self._threads:
            # join() in short pieces, so that Ctrl+C reaches the main thread.
            while thread.is_alive():
                thread.join(0.2)

    def _raise_errors(self) -> None:
        if not self._errors:
            return
        description, error = self._errors[0]
        notes = [f'Raised by {description} inside an at_once() block.']
        for other, other_error in self._errors[1:]:
            notes.append(f'Also failed: {other}: {type(other_error).__name__}: {other_error}')
        if self._skipped:
            notes.append('Not started because of that: ' + ', '.join(self._skipped))
        if hasattr(error, 'add_note'):
            for note in notes:
                error.add_note(note)
        raise error


@contextlib.contextmanager
def at_once() -> Iterator[Group]:
    """Run the commands inside the block at the same time; wait for all at its end.

        with baxter_interface.at_once():
            left.move_to_neutral()
            right.move_to_neutral()

    Raises, at the end of the block, the first error one of the commands raised (a
    TimeoutError, ConnectionError, ...), after all other commands have finished.
    See the top of this file for the details.
    """
    if getattr(_local, 'group', None) is not None:
        raise RuntimeError('An at_once() block cannot be opened inside another one')
    group = Group()
    _local.group = group
    try:
        yield group
    finally:
        _local.group = None
        # Also after an error in the block itself: what was started cannot be taken back,
        # and the program should not carry on while the robot is still moving.
        group._finish()
    group._raise_errors()


@overload
def command(method: Callable[P, R]) -> Callable[P, R | None]: ...


@overload
def command(*, wait_keyword: str) -> Callable[[Callable[P, R]], Callable[P, R | None]]: ...


def command(method: Any = None, *, wait_keyword: str | None = None) -> Any:
    """Mark a method as a command that an at_once() block hands to a helper thread.

    Outside a block the method runs as usual. (For the type checker a marked method can
    return None in addition to what it normally returns: that is what it does inside a
    block, where it has not happened yet.)

    Parameters:
        wait_keyword: for methods that only wait for the robot when asked to (the
                      gripper's block=True): the name of that keyword. Inside a block
                      it is switched on, since the block is meant to wait for everything.
    """
    def mark(method: Any) -> Any:
        @functools.wraps(method)
        def wrapper(self: Any, *args: Any, **kwargs: Any) -> Any:
            group = getattr(_local, 'group', None)
            if group is None:
                return method(self, *args, **kwargs)
            if wait_keyword is not None:
                kwargs.setdefault(wait_keyword, True)
            # One helper thread per robot part: the part's lock stands for the part
            # (several Limb('left') objects share one lock, see locks.py).
            part = getattr(self, '_motion_lock', None) or getattr(self, '_part_lock', None) or self
            name = getattr(self, 'name', type(self).__name__)
            group._submit(
                id(part),
                f'{name}.{method.__name__}()',
                lambda: method(self, *args, **kwargs)
            )
            return None
        return wrapper
    return mark(method) if method is not None else mark
