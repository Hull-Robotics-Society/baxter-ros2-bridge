#!/usr/bin/env python3
"""Example 8: open and close a gripper.

    python3 08_gripper.py            # the gripper on the left arm
    python3 08_gripper.py right

THE GRIPPER MOVES (the arms don't). Keep fingers out of it. Needs a gripper mounted on
that arm: an electric one (two fingers) or a suction one.

Shows: Gripper.type(), calibrate(), open(), close(), command_position(), and how to
tell whether something was gripped.

Not yet tried on the robot: no gripper was mounted on the test day (5 Oct 2026).
"""

import sys
import time

import baxter_interface
from baxter_interface import Gripper, RobotEnable

side = sys.argv[1] if len(sys.argv) > 1 else 'left'

with baxter_interface.setup():
    robot = RobotEnable()
    gripper = Gripper(side)

kind = gripper.type()
print(f'Gripper on the {side} arm: {kind}')
if kind not in ('electric', 'suction'):
    sys.exit('No electric or suction gripper is mounted on this arm.')

robot.enable()

if kind == 'electric':
    # An electric gripper has to find its end stops once after power-on, before it
    # accepts positions. It opens and closes fully while doing so.
    if not gripper.calibrated():
        print('Calibrating (the gripper opens and closes) ...')
        gripper.calibrate()

    print('Closing ...')
    gripper.close(block=True)           # block=True: wait until the gripper is done
    # gripping() is True if the fingers were stopped by something between them.
    print('  holding something:', gripper.gripping(), f'| position {gripper.position():.0f} of 100')

    print('Half open ...')
    gripper.command_position(50, block=True)     # 0 = closed, 100 = open

    print('Open ...')
    gripper.open(block=True)

else:
    print('Suction on for 3 seconds ...')
    gripper.close()                     # for a suction gripper, close() starts the suction
    time.sleep(3.0)
    print('  holding something:', gripper.gripping())
    print('Suction off ...')
    gripper.open()

print('Done.')
