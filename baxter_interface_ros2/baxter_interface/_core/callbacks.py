"""Running a user's callback in its own thread.

Why: the library receives all ROS 2 messages in one background thread (see node.py). If a
user's callback ran in that thread, a callback that takes long would hold up every other
update, and a callback that waits for the robot (limb.move_to_neutral(), a service call)
would wait for messages that the same, now busy, thread is supposed to deliver.

So every connected callback gets a thread of its own, as each subscription had in ROS 1:

    receiving thread  --call--> queue --> the callback's own thread --> your function

  - The receiving thread only puts the call into a queue, which never blocks it.
  - The calls of ONE callback run in order, one at a time.
  - DIFFERENT callbacks run independently, also at the same moment. If two callbacks
    change the same variable, protect it with a threading.Lock.
  - An error inside a callback is printed and does not stop later calls.
  - If a callback falls far behind (more than MAX_WAITING_CALLS calls waiting), the
    oldest waiting calls are dropped and a warning is printed once.
  - When the program ends, the callback threads end with it. A callback that is still
    running at that moment is abandoned, quietly: the errors it runs into because ROS 2
    is closing down are not printed.
"""

from __future__ import annotations

import queue
import sys
import threading
import traceback
import weakref
from typing import Any, Callable

_STOP = object()

# How many calls may wait for one callback before the oldest are dropped.
MAX_WAITING_CALLS = 1000

# Every worker that exists, so that stop_all() can reach them when the program ends.
_workers: weakref.WeakSet[CallbackWorker] = weakref.WeakSet()
_workers_guard = threading.Lock()
_closing = False


def stop_all() -> None:
    """Stop all callback threads. Called when the library shuts down (node.shutdown()).

    Waiting calls are dropped. A callback that is running right now cannot be interrupted;
    it is left to finish or to fail, and from here on its errors are no longer printed:
    they would only say that ROS 2 has been shut down underneath it.
    """
    global _closing
    _closing = True
    with _workers_guard:
        workers = list(_workers)
    for worker in workers:
        worker.stop()


def resume() -> None:
    """Print callback errors again. Called when the shared node is created."""
    global _closing
    _closing = False


class CallbackWorker:
    """One callback and the thread that runs it."""

    def __init__(
        self,
        callback: Callable[..., object],
        keep: str = 'all',
        name: str | None = None,
        prepare: Callable[..., tuple] | None = None
    ) -> None:
        """
        Parameters:
            callback: the function to run
            keep:     'all' runs the callback once for every call, in order (e.g. button
                      presses). 'latest' drops calls that arrive while the callback is
                      still busy, except the newest (e.g. camera frames: a slow callback
                      then skips frames instead of falling further and further behind).
            name:     name for the thread, shown in error messages
            prepare:  optional function that turns the submitted values into the callback's
                      arguments (it returns them as a tuple). It runs in the callback's
                      thread, so costly conversions (a camera message into a picture)
                      don't hold up the thread that submits.
        """
        if keep not in ('all', 'latest'):
            raise ValueError(f"keep must be 'all' or 'latest', not {keep!r}")
        self.callback = callback
        self._prepare = prepare
        self._latest = keep == 'latest'
        self._queue: queue.Queue = queue.Queue(maxsize=1 if self._latest else 0)
        self._name = name or getattr(callback, '__name__', 'callback')
        self._stopped = False
        self._warned = False
        self._thread = threading.Thread(
            target=self._run,
            name=f'baxter-callback-{self._name}',
            daemon=True  # never keeps the program alive on its own
        )
        self._thread.start()
        with _workers_guard:
            _workers.add(self)

    def submit(self, *args: Any, **kwargs: Any) -> None:
        """Queue one call of the callback. Returns at once; never waits for the callback."""
        if self._stopped:
            return
        if self._latest:
            self._replace_waiting((args, kwargs))
            return
        # A callback that never returns while its signal keeps firing would let the queue
        # grow without end. Past the limit, the oldest waiting calls are dropped.
        while self._queue.qsize() >= MAX_WAITING_CALLS:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
            if not self._warned:
                self._warned = True
                print(f"Callback '{self._name}' is not keeping up: more than {MAX_WAITING_CALLS} "
                      'calls are waiting, dropping the oldest. Is it stuck?', file=sys.stderr)
        self._queue.put((args, kwargs))

    def stop(self) -> None:
        """Stop the thread after the call it is currently running. Waiting calls are dropped."""
        self._stopped = True
        if self._latest:
            self._replace_waiting(_STOP)
            return
        while True:  # empty the queue so the stop request is seen next
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
        self._queue.put(_STOP)

    def _replace_waiting(self, item: object) -> None:
        """Put `item` into a one-place queue, replacing whatever is still waiting there."""
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass

    def _run(self) -> None:
        while True:
            call = self._queue.get()
            if call is _STOP:
                return
            args, kwargs = call
            try:
                if self._prepare is not None:
                    args, kwargs = self._prepare(*args, **kwargs), {}
                self.callback(*args, **kwargs)
            except Exception:
                if _closing:
                    continue  # the program is ending; see stop_all()
                print(f"Error in callback '{self._name}' (later calls still run):", file=sys.stderr)
                traceback.print_exc()
