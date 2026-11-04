"""Tests for the arithmetic of the head lights: no robot or bridge needed."""

import pytest

from baxter_interface.head_lights import lights_to_number, number_to_lights


def test_each_light_is_one_bit():
    assert lights_to_number([0]) == 1
    assert lights_to_number([3]) == 8
    assert lights_to_number([11]) == 2048
    assert lights_to_number([0, 3]) == 9


def test_all_and_none():
    assert lights_to_number('all') == 4095
    assert lights_to_number([]) == 0
    assert lights_to_number(range(12)) == 4095


def test_a_light_named_twice_counts_once():
    assert lights_to_number([5, 5, 5]) == 32


@pytest.mark.parametrize('wrong', [[12], [-1], [1.5], ['3'], [True], 'some'])
def test_wrong_light_numbers_are_refused(wrong):
    with pytest.raises(ValueError):
        lights_to_number(wrong)


def test_number_back_to_lights():
    assert number_to_lights(0) == []
    assert number_to_lights(9) == [0, 3]
    assert number_to_lights(4095) == list(range(12))


def test_the_override_flag_is_not_a_light():
    assert number_to_lights(0x8000 | 9) == [0, 3]
    assert number_to_lights(lights_to_number([2, 7, 11])) == [2, 7, 11]
