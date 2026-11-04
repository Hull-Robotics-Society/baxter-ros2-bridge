#!/usr/bin/env python3
"""Example 4: turn the head and nod.

    python3 04_head.py

THE HEAD MOVES (the arms don't). The robot must be enabled; the program does that.

Shows: Head.set_pan(), Head.command_nod(), Head.pan().
"""

import baxter_interface
from baxter_interface import Head, RobotEnable

with baxter_interface.setup():
    robot = RobotEnable()
    head = Head()

robot.enable()

# Angles in radians: 0 is straight ahead, positive is to the robot's left. The head
# turns about 1.3 rad (75 degrees) to each side. speed goes from 0 to 100.
print('Looking to its left ...')
head.set_pan(0.6, speed=30)
print(f'  the head is now at {head.pan():+.2f} rad')

print('Looking to its right ...')
head.set_pan(-0.6, speed=30)

print('Straight ahead, quickly ...')
head.set_pan(0.0, speed=80)

print('Nodding ...')
head.command_nod()

print('Done.')
