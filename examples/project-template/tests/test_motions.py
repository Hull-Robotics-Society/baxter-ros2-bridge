"""Tests for the arithmetic in my_project.motions: no robot or bridge needed.

Run them with the Testing panel in VS Code, or:  python3 -m pytest
"""

import pytest

from my_project.motions import square_corners


def test_square_begins_and_ends_at_the_start():
    corners = square_corners((0.6, 0.2, 0.1), size=0.1)

    assert len(corners) == 5
    assert corners[0] == corners[-1] == (0.6, 0.2, 0.1)


def test_square_stays_at_the_same_distance_from_the_robot():
    corners = square_corners((0.6, 0.2, 0.1), size=0.1)

    assert all(x == 0.6 for x, _y, _z in corners)


def test_square_goes_left_then_up():
    corners = square_corners((0.6, 0.2, 0.1), size=0.1)

    assert corners[1] == pytest.approx((0.6, 0.3, 0.1))
    assert corners[2] == pytest.approx((0.6, 0.3, 0.2))
    assert corners[3] == pytest.approx((0.6, 0.2, 0.2))


def test_a_square_needs_a_size():
    with pytest.raises(ValueError):
        square_corners((0.6, 0.2, 0.1), size=0.0)
