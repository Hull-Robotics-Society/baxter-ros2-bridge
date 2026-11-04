#!/usr/bin/env python3
"""Example 10: a control loop of your own. The arm waves.

    python3 10_wave.py            # the left arm
    python3 10_wave.py right

THE ARM MOVES, higher and wider than in the other examples. Nobody within reach of the
arms, and someone at the e-stop.

The move_... commands go to a posture and stop. For a motion that never stops (waving,
following something) a program sends the arm a new target many times per second
itself. This is such a loop.

Shows: set_joint_positions() in a loop with a steady beat, joint_state_age() to notice
a lost connection, and leaving the loop cleanly.
"""

import math
import sys
import time

import baxter_interface
from baxter_interface import Limb, RobotEnable

side = sys.argv[1] if len(sys.argv) > 1 else 'left'
sign = 1.0 if side == 'left' else -1.0   # the right arm is a mirror image of the left

with baxter_interface.setup():
    robot = RobotEnable()
    arm = Limb(side)

robot.enable()
arm.set_joint_position_speed(0.3)

# The posture to wave from: the arm raised, with the forearm free to swing.
raised = {
    f'{side}_s0': sign * 0.6,
    f'{side}_s1': -0.4,
    f'{side}_e0': sign * -1.4,
    f'{side}_e1': 1.6,
    f'{side}_w0': 0.0,
    f'{side}_w1': 0.0,
    f'{side}_w2': 0.0,
}
print('Raising the arm ...')
arm.move_to_joint_positions(raised)

RATE = 100            # targets per second
SECONDS = 8.0
print(f'Waving for {SECONDS:g} seconds (Ctrl+C stops earlier) ...')

start = time.monotonic()
next_beat = start
try:
    while True:
        elapsed = time.monotonic() - start
        if elapsed > SECONDS:
            break

        # The values the arm reports arrive 100 times per second. If the newest one is
        # old, the connection to the robot is lost: stop instead of commanding blindly.
        if arm.joint_state_age() > 0.5:
            raise ConnectionError('The arm stopped reporting; is the bridge still running?')

        # The target for this moment: the forearm swings 0.35 rad to each side, once
        # every 1.5 seconds. It starts gently (the swing grows during the first second).
        swing = 0.35 * min(1.0, elapsed) * math.sin(2 * math.pi * elapsed / 1.5)
        target = dict(raised)
        target[f'{side}_e0'] = raised[f'{side}_e0'] + swing
        arm.set_joint_positions(target)

        # Keep a steady beat: sleep until the next 1/100 s, however long the work took.
        next_beat += 1.0 / RATE
        time.sleep(max(0.0, next_beat - time.monotonic()))
except KeyboardInterrupt:
    print('\nStopped.')

# When the targets stop, the arm holds its last one. Bring it down again.
print('Back to neutral ...')
arm.move_to_neutral()
print('Done.')
