#!/usr/bin/env python3
"""Pass the robot's own robot model on to ROS 2.

Run by run_bridge.sh. Every Baxter keeps its robot model (URDF) in the ROS 1 parameter
/robot_description. It is the robot's own: the arms' mounting points are measured for
each robot, so it is more exact than the general model of baxter_common, and the
simulator's has the grippers it simulates. ros1_bridge does not carry parameters, so
this program reads the parameter from the robot's ROS 1 master and publishes it in ROS 2
as the topic robot/robot_description (std_msgs/String; with a robot name
/<name>/robot/robot_description).

The topic is "transient local", as robot_state_publisher's robot_description: a program
that starts later still gets the model. The parameter is read again every 10 s and the
model published again only when it has changed.

It talks to the ROS 1 master over XML-RPC directly (the master's own interface), so it
needs no ROS 1 libraries and runs in the bridge container's ROS 2 environment.

MoveIt uses it (moveit_config/launch/moveit.launch.py, robot_model:=robot).

Usage: robot_model.py <robot name or "">
"""

import os
import sys
import time
import xmlrpc.client

PARAMETER = '/robot_description'
TOPIC = 'robot/robot_description'
REFRESH_SECONDS = 10.0


def read_model(master_uri):
    """Return the robot's /robot_description, or None if the master has none (or is away)."""
    try:
        code, _, value = xmlrpc.client.ServerProxy(master_uri).getParam('/baxter_bridge_robot_model', PARAMETER)
    except (OSError, xmlrpc.client.Error):
        return None
    if code != 1 or not isinstance(value, str) or not value.strip():
        return None
    return value


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else ''
    master_uri = os.environ.get('ROS_MASTER_URI', '')

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    # No parameter services: in on_demand mode the bridge would offer them to the robot.
    node = rclpy.create_node(
        'baxter_robot_model',
        namespace='/' + name if name else '',
        start_parameter_services=False
    )
    publisher = node.create_publisher(
        String,
        TOPIC,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)
    )
    sent = None
    said_missing = False
    while rclpy.ok():
        model = read_model(master_uri)
        if model is None:
            if not said_missing:
                print(f'robot_model: the robot has no {PARAMETER} (yet); trying again every '
                      f'{REFRESH_SECONDS:g} s', flush=True)
                said_missing = True
        elif model != sent:
            publisher.publish(String(data=model))
            print(f'robot_model: the robot\'s model ({len(model)} characters) is on '
                  f'{publisher.topic_name}{" (changed)" if sent else ""}', flush=True)
            sent = model
            said_missing = False
        # Let rclpy answer late joiners (transient local) while waiting.
        end = time.monotonic() + REFRESH_SECONDS
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.5)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:  # never take the bridge down with it
        print(f'robot_model: stopped: {error!r}', file=sys.stderr)
