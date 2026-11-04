#!/usr/bin/env python3
"""Example 13: both arms and the head at the same time.

    python3 13_at_once.py

BOTH ARMS AND THE HEAD MOVE. Nobody within reach of the arms, and someone at the e-stop.

A command such as left.move_to_neutral() returns when the arm has arrived, so two of
them run one after the other. Inside an at_once() block they only start, and the block
waits at its end until all are done.

Shows: at_once() with different parts, several commands for one part, a function of
your own alongside, and an error from inside a block.
"""

import time

import baxter_interface
from baxter_interface import Head, Limb, Navigator, RobotEnable

with baxter_interface.setup():
    robot = RobotEnable()
    left = Limb('left')
    right = Limb('right')
    head = Head()
    navigator = Navigator('torso_left')

robot.enable()
left.set_joint_position_speed(0.2)
right.set_joint_position_speed(0.2)

print('One after the other: left arm, then right arm ...')
start = time.monotonic()
left.move_to_neutral()
right.move_to_neutral()
print(f'  took {time.monotonic() - start:.1f} s')

left.move_to_joint_positions({'left_w2': 0.8})
right.move_to_joint_positions({'right_w2': -0.8})

print('At the same time: both arms and the head ...')
start = time.monotonic()
with baxter_interface.at_once():
    left.move_to_neutral()
    right.move_to_neutral()
    head.set_pan(0.5, speed=30)
print(f'  took {time.monotonic() - start:.1f} s')

# Commands for the same part run in the order they are written; the parts run side by
# side. Here the left hand goes up and down again while the right wrist turns and the
# head looks the other way.
print('Left hand up and down, right wrist turns, head turns ...')
with baxter_interface.at_once():
    left.move_by(z=0.08)
    left.move_by(z=-0.08)
    right.move_to_joint_positions({'right_w2': 0.8})
    head.set_pan(-0.5, speed=30)


# A function of your own can run alongside: here a light blinks while the arms move.
def blink():
    for _ in range(8):
        navigator.inner_led = not navigator.inner_led
        time.sleep(0.25)
    navigator.inner_led = False


print('Back to neutral, with a blinking light on the torso ...')
with baxter_interface.at_once() as group:
    group.run(blink)
    right.move_to_neutral()
    head.set_pan(0.0)

# An error inside a block is raised at the end of the block, after the other parts have
# finished: the head still nods, although the left hand cannot go 3 m forward.
try:
    with baxter_interface.at_once():
        left.move_to_pose(x=3.0)
        head.command_nod()
except ValueError as error:
    print('As expected, the left hand could not go there:')
    print(' ', error)

print('Done.')
