"""Tests for what the type annotations brought with them at run time. No robot needed."""

import pathlib

import pytest

import baxter_interface
from baxter_interface import Accelerometer, HeadLights, Limb, Signal, Sonar
from baxter_interface._core.connecting import connected


def test_connected_hands_a_value_through():
    assert connected(3.5, 'the test part') == 3.5


def test_connected_accepts_values_that_count_as_false():
    # 0.0, False and an empty dict are states that have arrived, not missing ones.
    assert connected(0.0, 'the test part') == 0.0
    assert connected(False, 'the test part') is False
    assert connected({}, 'the test part') == {}


def test_connected_explains_a_missing_state():
    with pytest.raises(ConnectionError, match='the left gripper.*setup\\(\\) block'):
        connected(None, 'the left gripper')


def test_the_library_is_marked_as_typed():
    # Type checkers only read a library's annotations if this (empty) file is there.
    assert (pathlib.Path(baxter_interface.__file__).parent / 'py.typed').exists()


def test_named_values_still_behave_like_tuples():
    place = Limb.Point(0.6, 0.2, 0.1)
    turned = Limb.Quaternion(0.0, 1.0, 0.0, 0.0)

    assert (place.x, place.y, place.z) == (0.6, 0.2, 0.1)
    assert tuple(place) == (0.6, 0.2, 0.1)
    x, y, z = place
    assert z == 0.1
    assert turned.w == 0.0 and len(turned) == 4
    assert Sonar.Point(1.0, 2.0, 3.0).y == 2.0
    assert Accelerometer.Vector(0.0, 0.0, 9.8)[2] == 9.8
    assert HeadLights.Halo(red=100.0, green=0.0) == (100.0, 0.0)


def test_a_signal_can_say_what_it_hands_out():
    # Signal[bool] is only a note for the type checker; the signal works as before.
    received = []
    signal: Signal[bool] = Signal()
    signal.connect(received.append)
    signal(True)

    for _ in range(100):
        if received:
            break
        import time
        time.sleep(0.01)
    assert received == [True]
