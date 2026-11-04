"""Signals: have your own functions called when something happens.

Based on baxter_dataflow.signals (baxter_interface v0.7.0, Copyright (c) 2013,
Rethink Robotics, BSD license; see LICENSE). Same connect() / disconnect() interface.

Example:
    def on_button(pressed):
        if pressed:
            limb.move_to_neutral()

    nav = Navigator('left')
    nav.button0_changed.connect(on_button)

Every connected function runs in a thread of its own (see callbacks.py). That means:
  - it may take as long as it likes, move the robot or wait for it, as in the example;
  - its calls run in order, one at a time;
  - two different connected functions can run at the same moment. If both change the
    same variable, protect it with a threading.Lock.

Differences from the ROS 1 SDK: there, connected functions were only weakly referenced, so
a lambda passed directly to connect() was deleted at once and never called. Here a
connected function stays connected until you disconnect() it.
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Generic, TypeVar

from .callbacks import CallbackWorker

T = TypeVar('T')


class Signal(Generic[T]):
    """Something that can happen, and the functions to call when it does.

    Available methods:
        connect(function)    - call the function every time the signal fires
        disconnect(function) - stop calling it

    For the type checker a signal says what it hands to the connected functions:
    Signal[bool] calls them with True or False.
    """

    def __init__(self, keep: str = 'all', prepare: Callable[..., tuple] | None = None) -> None:
        """
        Parameters:
            keep:    'all' calls each function once for every time the signal fires, in
                     order. 'latest' skips firings that arrive while a function is still
                     busy, except the newest (used for camera frames).
            prepare: optional function that turns the fired values into the arguments of
                     the connected functions, in their threads (see CallbackWorker)
        """
        self._keep = keep
        self._prepare = prepare
        self._lock = threading.Lock()
        self._workers: list[CallbackWorker] = []

    def __call__(self, *args: Any, **kwargs: Any) -> None:
        """Fire the signal: queue one call of every connected function. Returns at once."""
        with self._lock:
            workers = list(self._workers)
        for worker in workers:
            worker.submit(*args, **kwargs)

    def connect(self, slot: Callable[[T], object]) -> None:
        """Call `slot` every time the signal fires, in a thread of its own.

        Parameters:
            slot: a function, method or lambda taking the signal's values as arguments
        """
        with self._lock:
            if any(worker.callback == slot for worker in self._workers):
                return  # already connected
            self._workers.append(CallbackWorker(
                slot,
                keep=self._keep,
                prepare=self._prepare
            ))

    def disconnect(self, slot: Callable[[T], object]) -> None:
        """Stop calling `slot`. A call that is running finishes; waiting calls are dropped.

        Parameters:
            slot: the function, method or lambda passed to connect()
        """
        with self._lock:
            matching = [worker for worker in self._workers if worker.callback == slot]
            self._workers = [worker for worker in self._workers if worker.callback != slot]
        for worker in matching:
            worker.stop()
