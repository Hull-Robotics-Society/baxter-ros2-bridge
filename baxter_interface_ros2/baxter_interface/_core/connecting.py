"""Creating several robot parts at once without waiting for each one in turn.

Every class (Limb, Head, Navigator, ...) does two things when it is created: it sets up
its topics, which takes no time, and it waits until the bridge has connected them and the
robot's first state has arrived, which takes a second or two, because the bridge looks
for new topics only about once per second.

Created one after the other, five parts wait five times. Inside a setup() block the parts
only set up their topics; the waiting happens once, at the end of the block, when the
bridge has usually connected all of them in the same look:

    import baxter_interface
    from baxter_interface import Limb, Head, Navigator

    with baxter_interface.setup():
        left = Limb('left')
        right = Limb('right')
        head = Head()
        navigator = Navigator('left')
    # here all four are connected and ready

Inside the block the parts are not ready yet: use them only after it. If one of them
cannot connect, the error is raised at the end of the block and names the part.
"""

from __future__ import annotations

import contextlib
import threading
from typing import Callable, Iterator, TypeVar

T = TypeVar('T')

# Per thread: the waits that parts created in a setup() block have put off.
_local = threading.local()


@contextlib.contextmanager
def setup() -> Iterator[None]:
    """Create several robot parts and wait for all of them at once.

        with baxter_interface.setup():
            left = Limb('left')
            head = Head()

    See the text at the top of this file.
    """
    if getattr(_local, 'waiting', None) is not None:
        yield  # already inside such a block: the outer one does the waiting
        return

    _local.waiting = []
    try:
        yield
        waiting = _local.waiting
    finally:
        _local.waiting = None
    # Reached only if the block ended without an error. By now the bridge has had time to
    # connect everything, so these waits mostly return at once.
    for finish in waiting:
        finish()


def finish_or_defer(finish: Callable[[], None]) -> None:
    """Run a part's waiting step now, or put it off to the end of a setup() block.

    Parameters:
        finish: function without arguments that waits until the part is connected
    """
    waiting = getattr(_local, 'waiting', None)
    if waiting is None:
        finish()
    else:
        waiting.append(finish)


def connected(value: T | None, what: str) -> T:
    """Return a part's stored state, or say clearly that the part is not connected yet.

    A part created inside a setup() block has no state until the block has ended. Without
    this check, using it too early would fail with an error about 'NoneType'.

    Parameters:
        value: the stored state, None until the first one has arrived
        what:  the part, for the error text, e.g. 'the left gripper'

    Raises:
        ConnectionError if nothing has arrived yet.
    """
    if value is None:
        raise ConnectionError(
            f'Nothing has arrived from {what} yet. Inside a setup() block the parts are '
            'not ready; use them after the block.')
    return value
