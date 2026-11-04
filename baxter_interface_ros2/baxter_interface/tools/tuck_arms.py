"""Tuck or untuck Baxter's arms to/from the shipping pose.

ROS 2 version of baxter_tools' tuck_arms.py (v0.7.0, Copyright (c) 2013,
Rethink Robotics, BSD license; see LICENSE):
    ros2 run baxter_interface tuck_arms -u     # untuck (arms to a neutral working pose)
    ros2 run baxter_interface tuck_arms -t     # tuck (arms folded for transport)

Same logic as the ROS 1 tool. Added: --timeout, since the original waits forever if a
joint can't reach its target.

Tucking passes very close to the head, so the tool suppresses the robot's collision
avoidance while it moves. Clear the area and have someone at the e-stop.

Needs these topics in bridge_config/topics.yaml, for left and right:
    /robot/limb/<side>/collision_avoidance_state
    /robot/limb/<side>/suppress_collision_avoidance
"""

import argparse
import sys
import time
from copy import deepcopy

from rclpy.utilities import remove_ros_args

from std_msgs.msg import Bool, Empty

from baxter_core_msgs.msg import CollisionAvoidanceState

import baxter_interface
from baxter_interface._core.node import get_node


class Tuck:

    def __init__(self, tuck_cmd, timeout=None):
        """
        Parameters:
            tuck_cmd: True to tuck, False to untuck
            timeout:  seconds before giving up on a move; None waits forever (as in ROS 1)
        """
        self._node = get_node()
        self._done = False
        self._limbs = ('left', 'right')
        self._arms = {
            'left': baxter_interface.Limb('left'),
            'right': baxter_interface.Limb('right'),
        }
        self._tuck = tuck_cmd
        self._timeout = timeout
        self._tuck_period = 1.0 / 20.0  # 20 Hz
        self._tuck_threshold = 0.2  # radians
        self._peak_angle = -1.6  # radians
        self._arm_state = {
            'tuck': {'left': 'none', 'right': 'none'},
            'collide': {'left': False, 'right': False},
            'flipped': {'left': False, 'right': False},
        }
        self._joint_moves = {
            'tuck': {
                'left': [-1.0, -2.07, 3.0, 2.55, 0.0, 0.01, 0.0],
                'right': [1.0, -2.07, -3.0, 2.55, -0.0, 0.01, 0.0],
            },
            'untuck': {
                'left': [-0.08, -1.0, -1.19, 1.94, 0.67, 1.03, -0.50],
                'right': [0.08, -1.0, 1.19, 1.94, -0.67, 1.03, 0.50],
            },
        }

        self._collide_lsub = self._node.create_subscription(
            CollisionAvoidanceState,
            'robot/limb/left/collision_avoidance_state',
            lambda msg: self._update_collision(msg, 'left'),
            10
        )
        self._collide_rsub = self._node.create_subscription(
            CollisionAvoidanceState,
            'robot/limb/right/collision_avoidance_state',
            lambda msg: self._update_collision(msg, 'right'),
            10
        )

        self._disable_pub = {
            'left': self._node.create_publisher(
                Empty,
                'robot/limb/left/suppress_collision_avoidance',
                10
            ),
            'right': self._node.create_publisher(
                Empty,
                'robot/limb/right/suppress_collision_avoidance',
                10
            ),
        }

        self._rs = baxter_interface.RobotEnable()
        self._enable_pub = self._node.create_publisher(Bool, 'robot/set_super_enable', 10)

    def _update_collision(self, data, limb):
        self._arm_state['collide'][limb] = len(data.collision_object) > 0
        self._check_arm_state()

    def _check_arm_state(self):
        """Check which pose each arm is in, and whether it is behind the collision field.

        If the s1 joint is over the peak, collision avoidance has to be suppressed to get
        the arm around the head-arm collision force field.
        """
        def close(a, b):
            return abs(a - b) <= self._tuck_threshold

        for limb in self._limbs:
            arm = self._arms[limb]
            angles = [arm.joint_angle(joint) for joint in arm.joint_names()]

            untuck_goal = [close(a, b) for a, b in zip(angles, self._joint_moves['untuck'][limb])]
            # only the two shoulder joints decide whether the arm counts as tucked
            tuck_goal = [close(a, b) for a, b in zip(angles[0:2], self._joint_moves['tuck'][limb][0:2])]

            if all(untuck_goal):
                self._arm_state['tuck'][limb] = 'untuck'
            elif all(tuck_goal):
                self._arm_state['tuck'][limb] = 'tuck'
            else:
                self._arm_state['tuck'][limb] = 'none'

            self._arm_state['flipped'][limb] = arm.joint_angle(limb + '_s1') <= self._peak_angle

    def _suppress_collision_avoidance(self):
        for pub in self._disable_pub.values():
            pub.publish(Empty())

    def _deadline(self):
        return None if self._timeout is None else time.monotonic() + self._timeout

    def _check_deadline(self, deadline, what):
        if deadline is not None and time.monotonic() > deadline:
            raise TimeoutError(f'Timed out after {self._timeout:g} s while {what}')

    def _prepare_to_tuck(self):
        head = baxter_interface.Head()
        start_disabled = not self._rs.state().enabled

        def at_goal():
            return abs(head.pan()) <= baxter_interface.settings.HEAD_PAN_ANGLE_TOLERANCE

        self._node.get_logger().info('Moving head to neutral position')
        deadline = self._deadline()
        while not at_goal():
            self._check_deadline(deadline, 'moving the head to neutral')
            if start_disabled:
                self._suppress_collision_avoidance()
            if not self._rs.state().enabled:
                self._enable_pub.publish(Bool(data=True))
            head.set_pan(0.0, 50, timeout=0)
            time.sleep(self._tuck_period)

        if start_disabled:
            deadline = self._deadline()
            while self._rs.state().enabled:
                self._check_deadline(deadline, 'disabling the robot again')
                self._suppress_collision_avoidance()
                self._enable_pub.publish(Bool(data=False))
                time.sleep(self._tuck_period)

    def _move_to(self, tuck, disabled):
        """Move arms until each is in its goal pose.

        Parameters:
            tuck:     {limb: 'tuck' or 'untuck'} for the arms to move
            disabled: {limb: bool}, True to suppress collision avoidance for that arm
        """
        if any(disabled.values()):
            self._suppress_collision_avoidance()

        deadline = self._deadline()
        while any(self._arm_state['tuck'][limb] != goal for limb, goal in tuck.items()):
            self._check_deadline(deadline, f'moving the arms to {tuck}')
            if not self._rs.state().enabled:
                self._enable_pub.publish(Bool(data=True))
            for limb in self._limbs:
                if disabled[limb]:
                    self._disable_pub[limb].publish(Empty())
                if limb in tuck:
                    arm = self._arms[limb]
                    arm.set_joint_positions(dict(zip(
                        arm.joint_names(),
                        self._joint_moves[tuck[limb]][limb]
                    )))
            self._check_arm_state()
            time.sleep(self._tuck_period)

        if any(self._arm_state['collide'].values()):
            self._rs.disable()

    def supervised_tuck(self):
        log = self._node.get_logger()
        self._prepare_to_tuck()
        self._check_arm_state()

        if self._tuck:
            if all(self._arm_state['tuck'][limb] == 'tuck' for limb in self._limbs):
                log.info("Tucking: Arms already in 'Tucked' position.")
                self._done = True
                return

            log.info('Tucking: One or more arms not Tucked.')
            if not all(self._arm_state['flipped'].values()):
                log.info('Moving to neutral start position with collision avoidance on.')

            # Arms that are not yet behind the head go to the untucked pose first,
            # with collision avoidance on.
            self._check_arm_state()
            actions = {}
            disabled = {'left': True, 'right': True}
            for limb in self._limbs:
                if not self._arm_state['flipped'][limb]:
                    actions[limb] = 'untuck'
                    disabled[limb] = False
            self._move_to(actions, disabled)

            log.info('Tucking: Tucking with collision avoidance off.')
            actions = {'left': 'tuck', 'right': 'tuck'}
            disabled = {'left': True, 'right': True}
            self._move_to(actions, disabled)
            self._done = True
            return

        if any(self._arm_state['flipped'].values()):
            log.info('Untucking: One or more arms Tucked; '
                     'Disabling Collision Avoidance and untucking.')
        else:
            log.info('Untucking: Arms already Untucked; Moving to neutral position.')
        self._check_arm_state()
        suppress = deepcopy(self._arm_state['flipped'])
        actions = {'left': 'untuck', 'right': 'untuck'}
        self._move_to(actions, suppress)
        self._done = True

    def clean_shutdown(self):
        """Leave the robot in a safe state if the tool was interrupted."""
        if not self._done:
            self._node.get_logger().warn('Aborting: Shutting down safely...')
        if any(self._arm_state['collide'].values()):
            deadline = time.monotonic() + 5.0
            while self._rs.state().enabled and time.monotonic() < deadline:
                self._suppress_collision_avoidance()
                self._enable_pub.publish(Bool(data=False))
                time.sleep(self._tuck_period)


def main():
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-t', '--tuck', action='store_true', help='tuck arms')
    group.add_argument('-u', '--untuck', action='store_true', help='untuck arms')
    parser.add_argument('--timeout', type=float, default=None,
                        help='seconds before giving up on a move (default: wait forever)')
    args = parser.parse_args(remove_ros_args()[1:])

    tucker = None
    try:
        print(f"{'T' if args.tuck else 'Unt'}ucking arms")
        tucker = Tuck(args.tuck, timeout=args.timeout)
        tucker.supervised_tuck()
        print('Finished tuck')
    except (TimeoutError, RuntimeError) as e:
        sys.exit(str(e))
    except KeyboardInterrupt:
        sys.exit('Interrupted')
    finally:
        if tucker is not None:
            tucker.clean_shutdown()


if __name__ == '__main__':
    main()
