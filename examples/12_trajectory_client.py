#!/usr/bin/env python3
"""Example 12: send a whole path to the joint trajectory action (plain ROS 2).

Start the action server first, in a second terminal:

    ros2 run baxter_interface joint_trajectory_action_server

then:

    python3 12_trajectory_client.py            # the left arm
    python3 12_trajectory_client.py right

THE ARM MOVES. Nobody within reach of the arms, and someone at the e-stop. The robot
must be enabled (ros2 run baxter_interface enable_robot -e).

This is how other ROS 2 software (motion planners such as MoveIt) moves an arm: it
sends a list of waypoints with times, "be at these angles after 3 s, at those after
5 s", and the server takes the arm through them in one smooth motion, without stopping
at each. The example uses only standard ROS 2 (rclpy and the FollowJointTrajectory
action), not the baxter_interface classes, to show that any ROS 2 program can do this.

Shows: an action client: sending a goal, receiving feedback, reading the result.
"""

import os
import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action.client import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

side = sys.argv[1] if len(sys.argv) > 1 else 'left'
sign = 1.0 if side == 'left' else -1.0   # the right arm is a mirror image of the left

JOINTS = [f'{side}_{joint}' for joint in ('s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2')]

# (seconds after the start, the seven joint angles in the order of JOINTS)
WAYPOINTS = [
    (3.0, [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]),                    # the neutral pose
    (5.0, [sign * 0.2, -0.7, 0.0, 1.0, 0.0, 1.0, 0.0]),
    (7.0, [sign * 0.4, -0.55, 0.0, 0.75, 0.0, 1.26, sign * 0.8]),
    (9.0, [sign * 0.2, -0.4, 0.0, 0.6, 0.0, 1.4, 0.0]),
    (11.0, [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]),                   # and back
]

rclpy.init()
# Only with several robots (docs/two-robots.md): BAXTER_ROBOT names the robot this program
# is for, and the node is put into that robot's namespace, where its action server is.
node = Node(
    'trajectory_example',
    namespace=os.environ.get('BAXTER_ROBOT', '')
)
client = ActionClient(
    node,
    FollowJointTrajectory,
    # No slash in front: the name then follows the node's namespace.
    f'robot/limb/{side}/follow_joint_trajectory'
)
if not client.wait_for_server(timeout_sec=5.0):
    sys.exit('The action server does not answer. Start it first:\n'
             '  ros2 run baxter_interface joint_trajectory_action_server')

points = []
for seconds, angles in WAYPOINTS:
    point = JointTrajectoryPoint(positions=angles)
    point.time_from_start.sec = int(seconds)
    point.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
    points.append(point)
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = JOINTS
goal.trajectory.points = points


def on_feedback(message):
    # Sent by the server 20 times per second: where the arm should be and where it is.
    worst = max(abs(error) for error in message.feedback.error.positions)
    print(f'\r  furthest joint is {worst:.3f} rad from the path  ', end='', flush=True)


print(f'Sending a path of {len(WAYPOINTS)} waypoints, {WAYPOINTS[-1][0]:g} seconds long ...')
sending = client.send_goal_async(goal, feedback_callback=on_feedback)
rclpy.spin_until_future_complete(node, sending)
goal_handle = sending.result()
if goal_handle is None or not goal_handle.accepted:
    sys.exit('The server refused the path (see its terminal for the reason).')

try:
    finishing = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, finishing)
    finished = finishing.result()
    if finished is None:
        sys.exit('The server went away before the path was finished.')
    result = finished.result
    print()
    if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        sys.exit('The path was stopped: ' + result.error_string)
    print('The arm followed the path to its end.')
except KeyboardInterrupt:
    # Ctrl+C: ask the server to stop the arm where it is.
    print('\nCancelling ...')
    cancelling = goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancelling, timeout_sec=2.0)

node.destroy_node()
