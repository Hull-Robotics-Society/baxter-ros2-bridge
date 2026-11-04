"""Motions built from the library's commands.

The split to keep: working out *where* to go is plain arithmetic and gets a test
(tests/test_motions.py); *going there* needs the robot and stays short.
"""

from baxter_interface import Limb

# A place for the hand: (x, y, z) in metres. x forward, y to the robot's left, z up.
Place = tuple[float, float, float]


def square_corners(start: Place, size: float) -> list[Place]:
    """Work out the corners of an upright square, beginning and ending at `start`.

    Parameters:
        start: the first corner (the lower one nearer to the robot's middle)
        size:  length of a side in metres

    Returns:
        five places: the four corners in order, then the start again
    """
    if size <= 0:
        raise ValueError('size must be greater than 0')
    x, y, z = start
    return [
        (x, y, z),
        (x, y + size, z),
        (x, y + size, z + size),
        (x, y, z + size),
        (x, y, z),
    ]


def draw_square(limb: Limb, size: float = 0.10, seconds: float = 12.0) -> None:
    """Move the hand along an upright square in front of the robot, starting where it is.

    Parameters:
        limb:    the arm to move (the robot must be enabled)
        size:    length of a side in metres
        seconds: time for the whole square
    """
    here = limb.hand_position()
    corners = square_corners(
        (here.x, here.y, here.z),
        size
    )
    # The first corner is where the hand already is.
    limb.move_through(
        corners[1:],
        seconds=seconds
    )
