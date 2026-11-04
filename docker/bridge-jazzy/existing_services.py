#!/usr/bin/env python3
"""Trim the configured list to what can work with this robot, before the bridge starts.

Run by run_bridge.sh after the topic list is loaded and before the bridge starts. Two jobs:

1. Topics: split them into two lists: topics that carry commands to the robot (marked
   `direction: to_robot` in bridge_config/topics.yaml) and all others. Each list gets a
   bridge process of its own (see run_bridge.sh for why).

2. Services: keep only the services that the ROS 1 side actually provides.

Why: when a ROS 2 program calls a bridged service and the ROS 1 service does not answer
(for example because its program on the robot has stopped), ros1_bridge raises an
error it does not catch, and the whole bridge process stops. Bridging only services that
answer avoids that for the common case.

"Provides" means: the service is registered with the ROS 1 master AND its provider
answers a connection. Being registered is not enough: a ROS 1 master never removes the
registrations of programs that have died, so a crashed node leaves its services listed.

Consequence: a ROS 1 service that only appears after the bridge has started is not
bridged until the bridge is restarted.

Usage: existing_services.py <parameter namespace> [--clock]     e.g. /baxter_bridge
       --clock: also bridge /clock, the simulator's clock (run_bridge.sh: BRIDGE_CLOCK=1)
"""

import sys

def split_by_direction(topics):
    """Split the topics into (state from the robot, commands to the robot).

    A topic marked `direction: to_robot` in topics.yaml carries commands. The mark itself
    is removed: the bridge does not know it.
    """
    from_robot, to_robot = [], []
    for entry in topics:
        entry = dict(entry)
        direction = entry.pop('direction', None)
        (to_robot if direction == 'to_robot' else from_robot).append(entry)
    return from_robot, to_robot


def check_topics(master, namespace, clock=False):
    param = namespace + '/topics'
    wanted = master.getParam(param)
    if clock:
        # The simulator's clock (simulation time), for programs that follow it.
        wanted.append({'topic': '/clock', 'type': 'rosgraph_msgs/msg/Clock'})
    from_robot, to_robot = split_by_direction(wanted)
    # Two lists, for two bridge processes (see run_bridge.sh).
    master.setParam(param, from_robot)
    master.setParam(param + '_to_robot', to_robot)
    print('topics: bridging %d configured topics' % len(wanted))
    print('topics: %d carry commands to the robot and get their own bridge process'
          % len(to_robot))


def split(wanted, answers):
    """Split the configured services into (those that answer, [(name, reason), ...]).

    Parameters:
        wanted:  list of {'service': name, 'type': type} entries
        answers: function(name) returning None if the service answers, else the reason
                 it doesn't, as text
    """
    keep, missing = [], []
    for entry in wanted:
        reason = answers(entry['service'])
        if reason is None:
            keep.append(entry)
        else:
            missing.append((entry['service'], reason))
    return keep, missing


def main():
    import rosgraph
    import rosservice

    namespace = sys.argv[1] if len(sys.argv) > 1 else '/baxter_bridge'
    clock = '--clock' in sys.argv[2:]
    param = namespace + '/services_2_to_1'
    master = rosgraph.Master('/baxter_bridge_setup')

    try:
        check_topics(master, namespace, clock)
    except Exception as e:  # the bridge still starts, with the full topic list
        print('topics: could not check the message types: %r' % (e,), file=sys.stderr)

    try:
        wanted = master.getParam(param)
    except rosgraph.MasterError:
        wanted = []
    if not wanted:
        print('services: none configured')
        return

    def answers(name):
        try:
            uri = master.lookupService(name)
        except rosgraph.MasterError:
            return 'not registered on the ROS 1 side'
        try:
            # Connects to the provider and exchanges the service handshake.
            rosservice.get_service_headers(name, uri)
        except Exception as e:
            return 'registered, but its provider does not answer (%s)' % (str(e).strip() or type(e).__name__)
        return None

    keep, missing = split(wanted, answers)
    master.setParam(param, keep)

    print('services: bridging %d of %d configured ROS 1 services' % (len(keep), len(wanted)))
    for name, reason in missing:
        print('services: skipped %s: %s' % (name, reason))


if __name__ == '__main__':
    try:
        main()
    except Exception as e:  # the bridge still starts, with the full list
        print('services: could not check which services exist: %r' % (e,), file=sys.stderr)
        print('services: bridging all configured services; calling a missing one '
              'will stop the bridge', file=sys.stderr)
