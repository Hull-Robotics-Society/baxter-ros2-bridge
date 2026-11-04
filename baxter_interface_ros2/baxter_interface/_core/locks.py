"""One lock per robot part, so commands from different threads don't get mixed up.

Every function connected to a signal runs in a thread of its own (callbacks.py), so two
of them can command the same arm, gripper, head or camera at the same moment. Without a
lock, the robot would receive two interleaved streams of commands.

part_lock('limb/left', node) always returns the same lock for the same name and robot,
also across several Limb('left') objects. A blocking command holds it for as long as it runs; a second
one for the same part, from another thread, waits until the first has finished.

The locks are re-entrant: a method holding the lock may call another method of the same
part that takes it again (calibrate() calling reset(), for example).
"""

from __future__ import annotations

import functools
import threading
from typing import Callable, ParamSpec, TypeVar

P = ParamSpec('P')
R = TypeVar('R')

_guard = threading.Lock()
_locks: dict[str, threading.RLock] = {}


def locked(method: Callable[P, R]) -> Callable[P, R]:
    """Mark a method as "one at a time per robot part".

    The method then holds the object's `_part_lock` while it runs. Use on methods that
    command the robot and wait for the result, or that send several commands in a row.

        class Head:
            def __init__(self):
                self._part_lock = part_lock('head')

            @locked
            def set_pan(self, angle):
                ...
    """
    @functools.wraps(method)
    def wrapper(*args: P.args, **kwargs: P.kwargs) -> R:
        # args[0] is the object the method belongs to (self).
        with getattr(args[0], '_part_lock'):
            return method(*args, **kwargs)
    return wrapper


def part_lock(name: str, node: object = None) -> threading.RLock:
    """Return the lock for one robot part.

    Parameters:
        name: a name identifying the part, e.g. 'limb/left', 'gripper/right', 'head',
              'camera/head_camera'
        node: the ROS 2 node the part's object uses. Its namespace tells which robot the
              part belongs to (see node.py): the left arms of two robots must not wait
              for each other.

    Returns:
        threading.RLock, the same object for every call with the same name and robot
    """
    key = f'{_robot_of(node)}:{name}'
    with _guard:
        if key not in _locks:
            _locks[key] = threading.RLock()
        return _locks[key]


def _robot_of(node: object) -> str:
    """The namespace of a node, which is its robot; '/' if it cannot be told (no node)."""
    try:
        return str(node.get_namespace())  # type: ignore[union-attr]
    except Exception:
        return '/'
