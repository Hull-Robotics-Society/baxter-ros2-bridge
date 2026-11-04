#!/usr/bin/env python3
"""Example 17: motions planned by MoveIt, around a thing in the way.

    python3 17_moveit.py            # the left arm
    python3 17_moveit.py right

THE ARM MOVES. Needs MoveIt running (docs/moveit.md), in another terminal:
    ros2 launch baxter_moveit_config moveit.launch.py rviz:=false
In the container, MoveIt and this program both run in the MoveIt container:
    docker compose run --rm moveit ros2 launch baxter_moveit_config moveit.launch.py rviz:=false
    docker compose run --rm moveit python3 examples/17_moveit.py

Shows: baxter_interface.planning.MotionPlanner.
"""

import sys

from baxter_interface import Limb, RobotEnable
from baxter_interface.planning import MotionPlanner, PlanningError

side = sys.argv[1] if len(sys.argv) > 1 else 'left'
sign = 1.0 if side == 'left' else -1.0   # the right arm is a mirror image of the left

RobotEnable().enable()
arm = Limb(side)
planner = MotionPlanner()

# A planned motion to a known posture.
plan = planner.move_to_named(side, 'neutral')
print(f'To the neutral pose: a path of {plan.points} points, {plan.duration:.1f} s')

# Put a thin wall next to the hand, then ask for a place on its other side: MoveIt
# finds a way around it. Positions are in metres in the robot's base frame, and mean
# the same hand tip as arm.hand_position().
here = arm.hand_position()
planner.add_box(
    'wall',
    size=(0.3, 0.02, 0.4),
    position=(here.x, here.y - sign * 0.12, here.z)
)
other_side = (here.x, here.y - sign * 0.25, here.z)

try:
    plan = planner.move_to_pose(side, other_side, plan_only=True)
    print(f'Around the wall: a path of {plan.points} points, {plan.duration:.1f} s')
    planner.move_to_pose(side, other_side)
    print('Hand now at', arm.hand_position())
except PlanningError as error:
    # The planner searches at random; now and then it finds nothing in its time.
    print(error)

# A place inside the wall is refused, with the reason.
try:
    planner.move_to_pose(side, (here.x, here.y - sign * 0.12, here.z), plan_only=True)
except PlanningError as error:
    print('As expected:', error)

planner.clear_objects()
planner.move_to_named(side, 'neutral')
print('Done.')
