#!/usr/bin/env python3
"""Example 15: two robots in one program.

    python3 15_two_robots.py              # the robots named 'one' and 'two'
    python3 15_two_robots.py ada bob      # two other names

BOTH ROBOTS MOVE their left arm. Needs a settings file and a running bridge per robot
(docs/two-robots.md): the names are the ROBOT_NAME of each file.

Shows: baxter_interface.Robot, and that setup() and at_once() work across robots.
"""

import sys
import time

import baxter_interface

names = sys.argv[1:3] if len(sys.argv) > 2 else ['one', 'two']
print('Robots found:', baxter_interface.known_robots())

first = baxter_interface.Robot(names[0])
second = baxter_interface.Robot(names[1])

# Everything of a robot is made through its Robot object.
with baxter_interface.setup():
    arm_first = first.Limb('left')
    arm_second = second.Limb('left')

for robot in (first, second):
    print(f'{robot.name}: topics start with {robot.namespace}/, enabled: {robot.state().enabled}')
    robot.enable()

# One robot after the other ...
start = time.monotonic()
arm_first.move_to_neutral()
arm_second.move_to_neutral()
print(f'To the neutral pose, one after the other: {time.monotonic() - start:.1f} s')

# ... and both at the same time: the block ends when every command in it has finished.
wrist_up = {'left_w1': 0.9}
with baxter_interface.at_once():
    arm_first.move_to_joint_positions({**arm_first.joint_angles(), **wrist_up})
    arm_second.move_to_joint_positions({**arm_second.joint_angles(), **wrist_up})

start = time.monotonic()
with baxter_interface.at_once():
    arm_first.move_to_neutral()
    arm_second.move_to_neutral()
print(f'Back to the neutral pose, both at the same time: {time.monotonic() - start:.1f} s')

print('Done. Both robots stay enabled.')
