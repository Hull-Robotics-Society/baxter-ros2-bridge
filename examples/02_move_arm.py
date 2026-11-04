#!/usr/bin/env python3
"""Example 2: move an arm by joint angles.

    python3 02_move_arm.py            # the left arm
    python3 02_move_arm.py right

THE ARM MOVES. Nobody within reach of the arms, and someone at the e-stop.

Shows: enabling the robot, the neutral pose, moving single joints, setting the speed,
and what happens when a move cannot be finished.
"""

import sys

import baxter_interface
from baxter_interface import Limb, RobotEnable

side = sys.argv[1] if len(sys.argv) > 1 else 'left'

with baxter_interface.setup():
    robot = RobotEnable()
    arm = Limb(side)

# The motors only work while the robot is enabled. It stays enabled after the program
# ends; disable it with:  ros2 run baxter_interface enable_robot -d
robot.enable()

# 0.0 (stopped) to 1.0 (full speed) for the move_... commands. The robot's own default
# is 0.3; slow is a good idea while trying things out.
arm.set_joint_position_speed(0.2)

print('Moving to the neutral pose ...')
arm.move_to_neutral()

# A move is a dictionary {joint name: angle in radians}. Joints that are not named stay
# where they are. The joints of the left arm are called left_s0, left_s1 (shoulder),
# left_e0, left_e1 (elbow), left_w0, left_w1, left_w2 (wrist).
print('Turning the wrist and lifting the elbow a little ...')
arm.move_to_joint_positions({
    f'{side}_w2': 0.8,
    f'{side}_e1': 1.0,
})

# Start from where the arm is now and change single values.
angles = arm.joint_angles()
angles[f'{side}_w2'] -= 1.6
print('Turning the wrist the other way ...')
arm.move_to_joint_positions(angles)

# A move waits until the arm has arrived. If it has not arrived after `timeout` seconds
# (something is in the way, the robot got disabled, the angle is out of the joint's
# range) it raises a TimeoutError; if the robot stops reporting, a ConnectionError.
try:
    arm.move_to_joint_positions({f'{side}_w2': 0.0}, timeout=10.0)
except TimeoutError as error:
    print('The arm did not arrive:', error)

print('Back to neutral ...')
arm.move_to_neutral()
print('Done.')
