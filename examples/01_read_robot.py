#!/usr/bin/env python3
"""Example 1: read the robot. Nothing moves; safe to run at any time.

    python3 01_read_robot.py

Shows: creating the parts of the robot in one setup() block, and reading the robot's
state, the joint angles of an arm, the place of a hand and the direction of the head.
"""

import baxter_interface
from baxter_interface import Head, Limb, RobotEnable

# Each part waits until it is connected to the robot. Created inside a setup() block,
# they wait together (a second or two) instead of one after the other.
with baxter_interface.setup():
    robot = RobotEnable()
    left = Limb('left')
    right = Limb('right')
    head = Head()

print('Robot software:', baxter_interface.software_version())

state = robot.state()
print('Enabled:', state.enabled, '| e-stop pressed or other stop:', state.stopped)

print()
print('Left arm joint angles (radians; 3.14 rad = 180 degrees):')
angles = left.joint_angles()
for joint in left.joint_names():
    print(f'  {joint}: {angles[joint]:+.3f}')

# Where the hands are: metres from the middle of the robot's torso.
# x = forward, y = to the robot's left, z = up.
for limb in (left, right):
    place = limb.hand_position()
    print(f'{limb.name} hand: x={place.x:+.3f}  y={place.y:+.3f}  z={place.z:+.3f} m')

print(f'Head: turned {head.pan():+.2f} rad (0 = straight ahead, positive = to its left)')
