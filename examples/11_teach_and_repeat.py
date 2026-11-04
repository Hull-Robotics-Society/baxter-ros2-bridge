#!/usr/bin/env python3
"""Example 11: show the arm some postures by hand, then let it repeat them.

    python3 11_teach_and_repeat.py            # the left arm
    python3 11_teach_and_repeat.py right

THE ARM MOVES when repeating. Nobody within reach of the arms then, and someone at the
e-stop.

How to use it:
  1. Hold the arm by its cuff (the ring just behind the hand). While the cuff is held,
     the robot lets the arm move freely.
  2. Put the arm into a posture and press the wheel of the navigator on that arm. The
     posture is stored, and the navigator's light blinks.
  3. Repeat for as many postures as you like, then let go, step back and press Enter.
     The arm goes through the stored postures in order: first stopping at each one,
     then once more in one smooth motion.

Shows: several parts of the robot working together: an event from a button, reading
joint angles, moving to postures one by one, and move_through() for a smooth motion
through all of them.
"""

import sys
import time

import baxter_interface
from baxter_interface import Limb, Navigator, RobotEnable

side = sys.argv[1] if len(sys.argv) > 1 else 'left'

with baxter_interface.setup():
    robot = RobotEnable()
    arm = Limb(side)
    navigator = Navigator(side)

robot.enable()   # the arm can only be moved by its cuff while the robot is enabled

postures = []


def on_wheel_button(pressed):
    if not pressed:
        return
    postures.append(arm.joint_angles())
    place = arm.hand_position()
    print(f'  posture {len(postures)} stored (hand at x={place.x:.2f}, y={place.y:.2f}, z={place.z:.2f} m)')
    navigator.inner_led = True
    time.sleep(0.3)
    navigator.inner_led = False


navigator.button0_changed.connect(on_wheel_button)

print(f'Hold the {side} arm by its cuff, move it, and press the wheel of the navigator')
print('on that arm to store a posture.')
input('When all postures are stored: let go, step back, and press Enter here.\n')
navigator.button0_changed.disconnect(on_wheel_button)

if not postures:
    sys.exit('No postures stored.')

arm.set_joint_position_speed(0.3)
print('Round 1: stopping at each posture')
for number, posture in enumerate(postures, start=1):
    print(f'  moving to posture {number} ...')
    arm.move_to_joint_positions(posture)

# move_through() takes the whole list and passes through the postures without stopping.
# speed: roughly the speed (rad/s) of the joint that has furthest to go.
print('Round 2: through all postures in one smooth motion')
arm.move_through(postures, speed=0.4)

print('Done.')
