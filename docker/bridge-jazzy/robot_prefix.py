#!/usr/bin/env python3
"""Put a robot's name in front of every bridged topic and service, on the ROS 2 side only.

Run by run_bridge.sh when the robot has a name (ROBOT_NAME=one), after
existing_services.py and before the bridge starts.

Why: every Baxter uses the same names (/robot/joint_states, ...). Two of them in one
ROS 2 system are kept apart the way ROS 2 keeps any two robots apart, by a namespace: the
first robot's joint states are /one/robot/joint_states there. On the robot itself nothing
can be renamed.

How: ros1_bridge uses one name per topic for both sides and has no renaming of its own.
But it takes ROS 1 "remapping" arguments (new:=old), and only its ROS 1 half reads them.
So this script
  - writes the names with the prefix into the lists the bridge reads (that is the name
    the ROS 2 side gets), and
  - prints one argument per name, /one/robot/state:=/robot/state, which run_bridge.sh
    passes to the bridge: its ROS 1 half then uses the robot's real name again.

Usage: robot_prefix.py <parameter namespace> <prefix>     e.g. /baxter_bridge /one
Prints the arguments, one per line.
"""

import sys

LISTS = (
    ('topics', 'topic'),
    ('topics_to_robot', 'topic'),
    ('services_1_to_2', 'service'),
    ('services_2_to_1', 'service'),
)


def add_prefix(entries, key, prefix):
    """Return (the entries with the prefix in front of their names, the arguments).

    Parameters:
        entries: list of dicts as in bridge_config/topics.yaml, e.g. {'topic': name, ...}
        key:     'topic' or 'service'
        prefix:  e.g. '/one'
    """
    renamed, arguments = [], []
    for entry in entries:
        entry = dict(entry)
        name = entry[key]
        entry[key] = prefix + name
        renamed.append(entry)
        arguments.append('%s%s:=%s' % (prefix, name, name))
    return renamed, arguments


def main():
    import rosgraph

    namespace, prefix = sys.argv[1], sys.argv[2]
    master = rosgraph.Master('/baxter_bridge_setup')
    for param, key in LISTS:
        try:
            entries = master.getParam(namespace + '/' + param)
        except rosgraph.MasterError:
            continue  # this list does not exist (the topics were not split, for example)
        renamed, arguments = add_prefix(entries or [], key, prefix)
        master.setParam(namespace + '/' + param, renamed)
        for argument in arguments:
            print(argument)


if __name__ == '__main__':
    main()
