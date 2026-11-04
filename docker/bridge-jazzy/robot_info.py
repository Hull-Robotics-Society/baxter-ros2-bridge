#!/usr/bin/env python3
"""Pass selected ROS 1 parameters of the robot on to ROS 2.

ros1_bridge carries topics and services, but not ROS 1 parameters. This small ROS 1 node
runs in the bridge container next to the bridge: it reads the parameters listed under
`robot_info_params` in bridge_config/topics.yaml from the robot's ROS 1 master and
publishes them as one JSON text message on /baxter_bridge/robot_info, which the bridge
carries to ROS 2 like any other topic:

    {"params": {"/rethink": {"software_version": "..."}, "/rosdistro": "groovy\\n", ...},
     "ros_master_uri": "http://...:11311"}

A parameter that does not exist on the robot is reported as null. A namespace (e.g.
"/rethink") is reported as a dictionary of everything below it.

It publishes once per second rather than once: a ROS 2 program that starts later would
never see a message that was only sent once. The parameters are re-read every 10 s.

It also offers the service <namespace>/ping (std_srvs/Empty), which answers at once.
bridge_guard.py calls it through the bridge to notice a bridge that has stopped
answering service calls.

In ROS 2: baxter_interface.robot_info()
"""

import json
import os
import sys
import time

REFRESH_SECONDS = 10.0
PUBLISH_HZ = 1.0


def collect(get_param, names):
    """Read the parameters `names` with `get_param(name)`; missing ones become None."""
    params = {}
    for name in names:
        try:
            params[name] = get_param(name)
        except KeyError:
            params[name] = None
    return params


def to_json(params, master_uri):
    # default=str: XML-RPC date and binary values have no JSON form; send their text.
    return json.dumps({'params': params, 'ros_master_uri': master_uri}, default=str)


def main():
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Empty, EmptyResponse

    namespace = sys.argv[1] if len(sys.argv) > 1 else '/baxter_bridge'
    rospy.init_node('baxter_bridge_robot_info')

    # A service that answers at once. bridge_guard.py calls it through the bridge to
    # find out whether the bridge still answers service calls at all.
    rospy.Service(namespace + '/ping', Empty, lambda request: EmptyResponse())

    names = rospy.get_param(namespace + '/robot_info_params', [])
    if not names:
        rospy.loginfo('robot_info: no robot_info_params configured, nothing to publish')
        rospy.spin()  # stay for the ping service
        return

    pub = rospy.Publisher(namespace + '/robot_info', String, queue_size=1)
    master_uri = os.environ.get('ROS_MASTER_URI', '')
    rospy.loginfo('robot_info: publishing %d parameters on %s/robot_info', len(names), namespace)

    # Wall-clock time on purpose (not rospy.Rate): a simulator runs ROS 1 on simulated
    # time, which stops when the simulation is paused and jumps back when it restarts.
    text, last_refresh = None, 0.0
    while not rospy.is_shutdown():
        now = time.monotonic()
        if text is None or now - last_refresh >= REFRESH_SECONDS:
            text = to_json(collect(rospy.get_param, names), master_uri)
            last_refresh = now
        pub.publish(String(data=text))
        time.sleep(1.0 / PUBLISH_HZ)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:  # never take the bridge down with it
        print('robot_info: stopped: %r' % (e,), file=sys.stderr)
