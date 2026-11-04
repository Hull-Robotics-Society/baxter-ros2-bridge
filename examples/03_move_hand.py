#!/usr/bin/env python3
"""Example 3: move a hand to a place, instead of setting joint angles.

    python3 03_move_hand.py            # the left arm
    python3 03_move_hand.py right

THE ARM MOVES. Nobody within reach of the arms, and someone at the e-stop.

Places are given in metres, measured from the middle of the robot's torso and seen from
the robot: x = forward, y = to the robot's left, z = up. The robot works out the joint
angles for a place itself ("inverse kinematics").

Shows: hand_position(), move_by(), move_to_pose(), solve_pose(), and a place that
cannot be reached.
"""

import sys

import baxter_interface
from baxter_interface import Limb, RobotEnable

side = sys.argv[1] if len(sys.argv) > 1 else 'left'
# Towards the robot's middle: -y for the left arm, +y for the right. In the neutral pose
# the hand is far out to the side, and that is the direction with room to move.
inward = -1.0 if side == 'left' else 1.0

with baxter_interface.setup():
    robot = RobotEnable()
    arm = Limb(side)

robot.enable()
arm.set_joint_position_speed(0.2)
arm.move_to_neutral()

start = arm.hand_position()
print(f'The hand is at x={start.x:.3f}, y={start.y:.3f}, z={start.z:.3f} m')

# move_by: a distance from where the hand is now. How the hand is turned stays the same.
print('5 cm up ...')
arm.move_by(z=0.05)
print('8 cm towards the middle ...')
arm.move_by(y=inward * 0.08)

# move_to_pose: a place. Coordinates that are left out keep their current value.
print('Back down to the height it started at ...')
arm.move_to_pose(z=start.z)

print('Back to the place it started at ...')
arm.move_to_pose(
    x=start.x,
    y=start.y,
    z=start.z
)

# A square in the air, 10 cm wide, drawn sideways and up.
print('A square ...')
for sideways, up in ((0.1, 0.0), (0.0, 0.1), (-0.1, 0.0), (0.0, -0.1)):
    arm.move_by(
        y=inward * sideways,
        z=up
    )

# solve_pose: only ask for the joint angles, without moving. A place the arm cannot
# reach raises a ValueError (move_to_pose and move_by do the same, before anything moves).
try:
    arm.solve_pose(x=3.0)
except ValueError as error:
    print('As expected, 3 m forward is out of reach:')
    print(' ', error)

angles = arm.solve_pose(z=start.z + 0.1)
print('Joint angles for 10 cm above the start:', {j: round(a, 2) for j, a in angles.items()})

# Tip: to find the coordinates of a place, hold the arm by its cuff (the robot lets it
# move freely then), put the hand there and print arm.hand_position(), as example 1 does.
print('Done.')
