"""Check that the project reaches the robot. Nothing moves.

    python3 scripts/read_robot.py
"""

import baxter_interface
from baxter_interface import Limb, RobotEnable

with baxter_interface.setup():
    robot = RobotEnable()
    left = Limb('left')

print('Robot software:', baxter_interface.software_version())
print('Enabled:', robot.state().enabled)
place = left.hand_position()
print(f'Left hand: x={place.x:+.3f}  y={place.y:+.3f}  z={place.z:+.3f} m')
