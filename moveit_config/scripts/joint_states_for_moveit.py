#!/usr/bin/env python3
"""Pass the robot's joint angles on to MoveIt in a form MoveIt accepts.

Started by moveit.launch.py. Reads robot/joint_states (from the robot, through the
bridge) and publishes moveit/joint_states. Two things are changed on the way:

  1. The time stamp becomes this computer's time. The robot stamps its joint angles with
     its own clock (a simulated robot with the simulation's), and MoveIt throws away
     everything that does not look recent by this computer's clock: it then plans from a
     wrong posture or not at all.
  2. A joint that is reported a hair beyond its limit is put on the limit. Baxter reports
     joints resting against a stop slightly past the number in the robot model, and
     MoveIt refuses to plan from a posture "out of bounds". Only differences up to
     TOLERANCE are corrected; a joint that is really out of range is passed on as it is,
     so that MoveIt still complains. TOLERANCE is the original configuration's
     start_state_max_bounds_error (reference/moveit_robots_baxter,
     ompl_planning_pipeline.launch): MoveIt 1 made this correction itself, MoveIt 2 no
     longer does.

Joints the robot model does not know (gripper fingers, the nodding joint) are left out.
"""

import xml.etree.ElementTree as ElementTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOLERANCE = 0.1   # rad


def limits_from(model):
    """Return {joint name: (lower, upper)} of the moving joints of a robot model (URDF text).

    A joint without limits (it can turn without end) gets (None, None).
    """
    limits = {}
    for joint in ElementTree.fromstring(model).findall('joint'):
        if joint.get('type') in (None, 'fixed'):
            continue
        limit = joint.find('limit')
        if limit is None or joint.get('type') == 'continuous':
            limits[joint.get('name')] = (None, None)
        else:
            limits[joint.get('name')] = (float(limit.get('lower', 0.0)), float(limit.get('upper', 0.0)))
    return limits


def within_limits(angle, lower, upper, tolerance=TOLERANCE):
    """Return the angle, moved onto a limit if it is past it by no more than `tolerance`."""
    if lower is not None and lower - tolerance <= angle < lower:
        return lower
    if upper is not None and upper < angle <= upper + tolerance:
        return upper
    return angle


class JointStatesForMoveIt(Node):

    def __init__(self):
        super().__init__('joint_states_for_moveit')
        self.declare_parameter('robot_description', '')
        self._limits = limits_from(self.get_parameter('robot_description').value)
        self._publisher = self.create_publisher(JointState, 'moveit/joint_states', 10)
        self.create_subscription(JointState, 'robot/joint_states', self._pass_on, 10)

    def _pass_on(self, msg):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        for index, name in enumerate(msg.name):
            if name not in self._limits or index >= len(msg.position):
                continue
            out.name.append(name)
            out.position.append(within_limits(msg.position[index], *self._limits[name]))
            if len(msg.velocity) == len(msg.name):
                out.velocity.append(msg.velocity[index])
            if len(msg.effort) == len(msg.name):
                out.effort.append(msg.effort[index])
        if out.name:
            self._publisher.publish(out)


def main():
    rclpy.init()
    node = JointStatesForMoveIt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
