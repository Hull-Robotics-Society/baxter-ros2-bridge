#!/usr/bin/env python3
"""Tell ROS 2 where this robot stands in the room.

Run by run_bridge.sh. A robot does not know its place in the room; with several robots
in one picture (RViz, later motion planning) someone has to say it. This publishes one
transform that never changes: from the frame "world", a point in the room that all
robots share, to this robot's base. The value comes from ROBOT_POSITION in the robot's
settings file.

It goes to /baxter_model/tf_static, where the transforms of the robot models are (see
robot_model/launch/view_robots.launch.py), and is kept for programs that start later.

Usage: robot_place.py <robot name or ""> "<x> <y> <z> <turn>"
       x, y, z in metres; turn in degrees around the vertical axis, counter-clockwise
       seen from above. E.g. robot_place.py two "0 1.5 0 180"
"""

import math
import sys

TOPIC = '/baxter_model/tf_static'
ROOM = 'world'
BASE = 'base'   # the root of Baxter's robot model


def parse_position(text):
    """Return (x, y, z, turn in degrees) from "x y z turn"; ValueError if it is not that.

    Commas are accepted in place of spaces. Missing values at the end count as 0.
    """
    parts = text.replace(',', ' ').split()
    if len(parts) > 4:
        raise ValueError('more than four numbers')
    values = [float(part) for part in parts] + [0.0] * (4 - len(parts))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('not a finite number')
    return tuple(values)


def frame_of(name):
    """The name of this robot's base frame: 'one/base' for the robot 'one', 'base' for ''."""
    return f'{name}/{BASE}' if name else BASE


def main():
    name, text = sys.argv[1], sys.argv[2]
    try:
        x, y, z, turn = parse_position(text)
    except ValueError as error:
        print(f"robot_place: ROBOT_POSITION '{text}' is not understood ({error}). It is "
              '"x y z turn": metres and degrees, e.g. "0 1.5 0 180". Using 0 0 0 0.',
              flush=True)
        x = y = z = turn = 0.0

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    # No parameter services: in on_demand mode the bridge would offer them to the robot.
    node = rclpy.create_node(
        'baxter_robot_place',
        namespace='/' + name if name else '',
        start_parameter_services=False
    )
    # "Transient local": a program that starts later still gets the message.
    publisher = node.create_publisher(
        TFMessage,
        TOPIC,
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
    )
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = ROOM
    transform.child_frame_id = frame_of(name)
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    half = math.radians(turn) / 2.0
    transform.transform.rotation.z = math.sin(half)
    transform.transform.rotation.w = math.cos(half)
    publisher.publish(TFMessage(transforms=[transform]))
    print(f'robot_place: {frame_of(name)} is at x={x:g} y={y:g} z={z:g} m, turned {turn:g} '
          f'degrees, in the frame "{ROOM}" ({TOPIC})', flush=True)
    rclpy.spin(node)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
