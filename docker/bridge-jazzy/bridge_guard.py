#!/usr/bin/env python3
"""Restart the bridge process that carries service calls when it has stopped answering.

Why: when a ROS 2 program calls one of the robot's services and the robot never answers,
ros1_bridge waits for that answer for good. It handles one thing at a time, so from then
on it answers no other service call either, and in on_demand mode it also connects no
more topics: every program started afterwards gets no data. Seen on the robot on
7 Oct 2026, when the robot's camera program hung.

How: robot_info.py offers a small ROS 1 service in this container that answers at once
(<namespace>/ping). This guard, a ROS 2 program, calls it through the bridge every few
seconds. No answer for GIVE_UP_AFTER seconds means the bridge process is stuck; the guard
stops that process, and run_bridge.sh starts it again within two seconds.

A service call that really takes longer than GIVE_UP_AFTER seconds would also be cut
off. None of the robot's bridged services comes close (opening a camera: a few seconds).

Usage: bridge_guard.py <ping service> <bridge mode>
       e.g. bridge_guard.py /baxter_bridge/ping fixed_list
Switched off with BRIDGE_GUARD=0 (see run_bridge.sh).
"""

import subprocess
import sys
import time

CHECK_EVERY = 5.0      # seconds between two calls
GIVE_UP_AFTER = 30.0   # seconds without an answer after which the bridge counts as stuck
RESTART_TIME = 15.0    # seconds to leave the bridge alone after stopping it

# What to stop, as a pattern for the process's command line. It matches the bridge
# program and the "ros2 run" that started it, and nothing else in this container.
STUCK_PROCESS = {
    'on_demand': 'ros1_bridge.dynamic_bridge',
    'fixed_list': '__name:=ros_bridge_services',
}


def answered(node, client, request, seconds):
    """Call the service and return True if the answer comes within `seconds`.

    Also True if the service disappears while waiting: then the bridge process has
    stopped (and is being started again), which is not the same as being stuck. Seen
    with the simulator, 8 Oct 2026: a service of the ROS 1 side failed, the bridge
    stopped and restarted as it should, and the guard, still waiting for an answer from
    the old process, stopped the new one as well.
    """
    import rclpy

    future = client.call_async(request)
    end = time.monotonic() + seconds
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not client.service_is_ready():
            client.remove_pending_request(future)
            return True
    if future.done():
        return True
    client.remove_pending_request(future)
    return False


def main():
    import rclpy
    from std_srvs.srv import Empty

    service, mode = sys.argv[1], sys.argv[2]
    pattern = STUCK_PROCESS[mode]

    rclpy.init()
    # No parameter services: in on_demand mode the bridge would offer them to the robot.
    node = rclpy.create_node('baxter_bridge_guard', start_parameter_services=False)
    client = node.create_client(Empty, service)
    print(f'bridge_guard: watching {service} (stuck after {GIVE_UP_AFTER:g} s without an answer)',
          flush=True)

    seen = False
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=CHECK_EVERY)
        if not client.service_is_ready():
            continue  # the bridge is starting (or being restarted); nothing to judge yet
        if not seen:
            seen = True
            print('bridge_guard: the bridge answers service calls', flush=True)
        if answered(node, client, Empty.Request(), GIVE_UP_AFTER):
            time.sleep(CHECK_EVERY)
            continue
        print(f'bridge_guard: no answer through the bridge for {GIVE_UP_AFTER:g} s. It is '
              'waiting for a service of the robot that does not answer. Stopping it; it '
              'starts again in a few seconds.', flush=True)
        subprocess.run(['pkill', '-KILL', '-f', pattern], check=False)
        seen = False
        time.sleep(RESTART_TIME)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:  # never take the bridge down with it
        print('bridge_guard: stopped: %r' % (e,), file=sys.stderr)
