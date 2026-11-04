"""Tests for the helper that passes the robot's joint angles on to MoveIt: reading the
limits from the robot model, and putting a joint that is a hair past its limit onto it.

Run in the MoveIt container:  python3 -m pytest moveit_config/test
"""

import importlib.util
import pathlib

SCRIPT = pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'joint_states_for_moveit.py'
spec = importlib.util.spec_from_file_location('joint_states_for_moveit', SCRIPT)
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)

MODEL = """<robot name="test">
  <joint name="left_s1" type="revolute"><limit lower="-2.147" upper="1.047" velocity="1.5" effort="100"/></joint>
  <joint name="wheel" type="continuous"/>
  <joint name="torso_t0" type="fixed"/>
</robot>"""


def test_limits_are_read_for_moving_joints_only():
    assert helper.limits_from(MODEL) == {'left_s1': (-2.147, 1.047), 'wheel': (None, None)}


def test_an_angle_inside_its_limits_is_kept():
    assert helper.within_limits(0.5, -2.147, 1.047) == 0.5


def test_a_hair_past_a_limit_is_put_on_the_limit():
    # What a Baxter arm resting against its lower shoulder stop reports.
    assert helper.within_limits(1.0470001, -2.147, 1.047) == 1.047
    assert helper.within_limits(-2.16, -2.147, 1.047) == -2.147
    # Up to 0.1 rad, the original configuration's start_state_max_bounds_error.
    assert helper.within_limits(1.14, -2.147, 1.047) == 1.047


def test_really_out_of_range_is_passed_on_so_that_moveit_complains():
    assert helper.within_limits(1.2, -2.147, 1.047) == 1.2


def test_a_joint_without_limits_is_never_changed():
    assert helper.within_limits(25.0, None, None) == 25.0
