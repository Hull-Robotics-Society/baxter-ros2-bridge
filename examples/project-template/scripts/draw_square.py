"""Move the left hand along a square in the air, using the project's own package.

    python3 scripts/draw_square.py

THE ARM MOVES. Nobody within reach of the arms, and someone at the e-stop.
"""

import baxter_interface
from baxter_interface import Limb, RobotEnable

from my_project.motions import draw_square

with baxter_interface.setup():
    robot = RobotEnable()
    left = Limb('left')

robot.enable()
left.set_joint_position_speed(0.3)
left.move_to_neutral()

print('Drawing a square of 10 cm ...')
draw_square(left, size=0.10)
print('Done.')
