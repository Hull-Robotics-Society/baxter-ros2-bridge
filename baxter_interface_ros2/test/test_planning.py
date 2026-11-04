"""Tests for the arithmetic in the MoveIt interface that needs no MoveIt running: turning a
hand-tip pose as the robot reports it into the pose of the robot model's hand tip.

Skipped where MoveIt's messages are not installed (the client image; they are in the
MoveIt image). With a marker, not pytest.importorskip at module level: launch_testing's
pytest plugin (in every ROS 2 image) then aborts the collection of the whole folder, and
no test runs at all (found by the robot check, 21 Oct 2026)."""

import importlib.util
import math

import pytest

HAVE_MOVEIT = importlib.util.find_spec('moveit_msgs') is not None
pytestmark = pytest.mark.skipif(not HAVE_MOVEIT, reason="MoveIt's messages are not installed "
                                                        '(they are in the MoveIt image)')
if HAVE_MOVEIT:
    from baxter_interface.planning import model_tip_for, tip_offset

NOT_TURNED = (0.0, 0.0, 0.0, 1.0)
# Hand pointing down: a half turn about the x axis.
DOWN = (1.0, 0.0, 0.0, 0.0)
# A quarter turn about the z axis.
QUARTER = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))


def close(a, b):
    return all(abs(x - y) < 1e-9 for x, y in zip(a, b))


def test_without_a_gripper_both_tips_are_the_same():
    offset = tip_offset((0.6, 0.3, 0.1), DOWN, (0.6, 0.3, 0.1), DOWN)

    position, orientation = model_tip_for((0.7, 0.2, 0.3), DOWN, offset)

    assert close(position, (0.7, 0.2, 0.3)) and close(orientation, DOWN)


def test_a_gripper_pointing_down_puts_the_model_tip_above_the_target():
    # The hand points down; the robot reports its tip 13 cm below the model's.
    offset = tip_offset((0.6, 0.3, 0.20), DOWN, (0.6, 0.3, 0.07), DOWN)

    position, orientation = model_tip_for((0.7, 0.2, 0.30), DOWN, offset)

    assert close(position, (0.7, 0.2, 0.43)) and close(orientation, DOWN)


def test_the_offset_turns_with_the_hand():
    # Measured with the hand not turned: the gripper's tip is 13 cm along the hand's z.
    offset = tip_offset((0.0, 0.0, 0.0), NOT_TURNED, (0.0, 0.0, 0.13), NOT_TURNED)

    # Asked for with the hand pointing down: the model's tip must be 13 cm above.
    position, _ = model_tip_for((0.5, 0.0, 0.0), DOWN, offset)

    assert close(position, (0.5, 0.0, 0.13))


def test_measuring_in_one_posture_fits_every_other():
    # The same gripper, measured in a turned posture and used in another.
    measured = tip_offset((0.1, 0.2, 0.3), QUARTER, (0.1, 0.2, 0.43), QUARTER)
    straight = tip_offset((0.0, 0.0, 0.0), NOT_TURNED, (0.0, 0.0, 0.13), NOT_TURNED)

    assert close(measured[0], straight[0]) and close(measured[1], straight[1])
