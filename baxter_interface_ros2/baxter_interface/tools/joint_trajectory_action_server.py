"""Joint trajectory action server: plays smooth arm paths sent by other ROS 2 programs.

    ros2 run baxter_interface joint_trajectory_action_server                # both arms
    ros2 run baxter_interface joint_trajectory_action_server -l left        # one arm
    ros2 run baxter_interface joint_trajectory_action_server -m velocity    # another mode
    ros2 run baxter_interface joint_trajectory_action_server --curve sdk    # the SDK's curve

Motion planners such as MoveIt don't command an arm joint by joint. They send a whole
path ("be at these joint angles after 1 s, at those after 2.5 s, ...") to an action
called FollowJointTrajectory and wait for the result. This program offers that action
for Baxter's arms:

    /robot/limb/left/follow_joint_trajectory      (control_msgs/action/FollowJointTrajectory)
    /robot/limb/right/follow_joint_trajectory

It fills in a smooth curve between the waypoints and commands the arm 100 times per
second along it, in one of the three modes of the Baxter SDK's server (-m):

    position       position commands. The robot's own safety layer stays active (speed
                   limit, collision avoidance between the arms). The default here.
    position_w_id  "raw" position commands plus the path's speeds and accelerations for
                   the robot's inverse dynamics (feed-forward): the arm follows more
                   exactly. Raw commands BYPASS THE ROBOT'S COLLISION AVOIDANCE. The
                   SDK's default; needs all seven joints of an arm in every path.
    velocity       velocity commands from a PID controller per joint (gains: parameters).

The curve between the waypoints is this library's by default (--curve smooth: fixed by
position, speed and acceleration at the waypoints; without given speeds the arm starts
and ends at rest). --curve sdk selects the SDK's own spline instead, for motion exactly
as under the SDK (see _core/bezier.py for the differences).

After a path the server keeps holding the arm at the path's end until the next path
arrives, the arm is taken by its cuff, or the robot is disabled.

Tolerances come from the goal if it carries them, otherwise from this node's parameters
(the SDK used dynamic reconfigure for them); change them while it runs, for example:

    ros2 param set /rsdk_position_joint_trajectory_action_server left_s0_trajectory 0.3

    goal_time                    seconds the arm may be late at the end of a path
    stopped_velocity_tolerance   speed (rad/s) below which the arm counts as stopped at
                                 the end; negative: not checked
    <joint>_trajectory           how far (rad) the joint may be from the path on the way;
                                 negative: not checked
    <joint>_goal                 how far (rad) it may be from the last waypoint at the
                                 end; negative: not checked
    <joint>_kp, _ki, _kd         velocity mode only: the joint's PID gains

The robot must be enabled. Clear the area and have someone at the e-stop: the arms move
as soon as a program sends a path.

New ROS 2 program with the modes, defaults and behaviour of joint_trajectory_action in
baxter_interface v1.2.0 (Copyright (c) 2013-2015, Rethink Robotics, BSD license; see
LICENSE). Differences: the default mode is position (was position_w_id); the default
curve between the waypoints is this library's (_core/trajectory.py), with the SDK's
B-spline as an option; a path whose first waypoint lies in the future starts from where
the arm is; a
path for unknown joints is refused instead of aborted; a lost connection ends the path.
Not yet run on the robot (written 6 Oct 2026).
"""

import argparse
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import ok as ros_is_running
from rclpy.utilities import remove_ros_args
from std_msgs.msg import UInt16
from trajectory_msgs.msg import JointTrajectoryPoint

import baxter_interface
from baxter_interface import DigitalIO, Limb, RobotEnable
from baxter_interface._core import dataflow
from baxter_interface._core.node import get_node
from baxter_interface._core.pid import PID
from baxter_interface._core.trajectory import CURVES, make_trajectory

MODES = ('position', 'position_w_id', 'velocity')
FEEDBACK_EVERY = 5    # progress is reported every 5th command (20 times per second at 100 Hz)

# The defaults of the SDK's three dynamic reconfigure files, per mode. Negative: not checked.
DEFAULTS = {
    'position': {'goal_time': 0.1, 'stopped_velocity_tolerance': 0.25, 'goal': -1.0, 'trajectory': 0.2},
    'position_w_id': {'goal_time': 0.1, 'stopped_velocity_tolerance': 0.20, 'goal': -1.0, 'trajectory': 0.35},
    'velocity': {'goal_time': 0.0, 'stopped_velocity_tolerance': -1.0, 'goal': -1.0, 'trajectory': -1.0},
}
PID_DEFAULTS = {'kp': 2.0, 'ki': 0.0, 'kd': 0.0}

_CODES = FollowJointTrajectory.Result


def _seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def _duration(point, seconds):
    point.time_from_start.sec = int(seconds)
    point.time_from_start.nanosec = int((seconds % 1.0) * 1e9)


class ArmTrajectoryServer:
    """The FollowJointTrajectory action of one arm."""

    def __init__(self, node, limb, cuff, robot, mode, rate, curve='smooth'):
        """
        Parameters:
            node:  the ROS 2 node the action is offered on (it also holds the parameters)
            limb:  the Limb to move
            cuff:  DigitalIO of the arm's cuff sensor (True while the arm is held)
            robot: RobotEnable, to notice a disabled robot
            mode:  'position', 'position_w_id' or 'velocity'
            rate:  commands per second
            curve: 'smooth' or 'sdk': the curve between the waypoints
        """
        self._node = node
        self._limb = limb
        self._cuff = cuff
        self._robot = robot
        self._mode = mode
        self._rate = rate
        self._curve = curve
        self._alive = True
        self._pid = {joint: PID() for joint in limb.joint_names()}
        # A newer goal replaces a running one: each goal gets a number, and a running
        # goal (or the holding after it) stops when it is no longer the newest.
        self._newest_goal = 0
        self._guard = threading.Lock()
        # Held while a goal moves the arm or the arm is held afterwards, so that one has
        # stopped commanding before the next one starts.
        self._moving = threading.Lock()

        # Feed-forward for the robot's inverse dynamics (position_w_id mode).
        self._pub_feed_forward = node.create_publisher(
            JointTrajectoryPoint,
            f'robot/limb/{limb.name}/inverse_dynamics_command',
            1
        )
        self._server = ActionServer(
            node,
            FollowJointTrajectory,
            f'robot/limb/{limb.name}/follow_joint_trajectory',
            execute_callback=self._execute,
            goal_callback=self._accept,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup()  # goals and cancels at the same time
        )

    def feed_forward_connected(self):
        """True once the robot (through the bridge) listens to the feed-forward topic."""
        return dataflow.wait_for_listener(self._pub_feed_forward)

    def shutdown(self):
        """Stop commanding and hand the arm back to the robot's own position holding."""
        self._alive = False
        try:
            self._limb.exit_control_mode()
        except Exception:  # shutting down: nothing more to be done about it
            pass

    # --- Goals -----------------------------------------------------------------------------

    def _accept(self, goal):
        names = list(goal.trajectory.joint_names)
        unknown = [name for name in names if name not in self._limb.joint_names()]
        problem = None
        if unknown:
            problem = f'unknown joints {unknown}'
        elif self._mode == 'position_w_id' and set(names) != set(self._limb.joint_names()):
            problem = ('position_w_id mode needs all seven joints of the arm in a path '
                       f'(got {names}); use -m position for paths with fewer joints')
        if problem:
            self._node.get_logger().error(f'{self._limb.name} arm: goal refused: {problem}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        with self._guard:
            self._newest_goal += 1
            number = self._newest_goal
        with self._moving:
            code, text, hold = self._follow(goal_handle, number)

        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = text
        log = self._node.get_logger()
        if text == 'cancelled':
            goal_handle.canceled()
            log.info(f'{self._limb.name} arm: path cancelled')
        elif code == _CODES.SUCCESSFUL:
            goal_handle.succeed()
            log.info(f'{self._limb.name} arm: path finished')
        else:
            goal_handle.abort()
            log.warning(f'{self._limb.name} arm: path stopped: {text}')

        if hold is not None:
            # Keep the arm where the path left it, as the SDK's server did, without
            # holding up the result of the goal.
            threading.Thread(
                target=self._hold,
                args=(number, hold),
                daemon=True
            ).start()
        return result

    def _tolerances(self, goal, names):
        """Tolerances for one goal: from the goal where it carries them, else the parameters."""
        def parameter(name):
            return self._node.get_parameter(name).value

        goal_time = _seconds(goal.goal_time_tolerance) or parameter('goal_time')
        path = {name: parameter(f'{name}_trajectory') for name in names}
        end = {name: parameter(f'{name}_goal') for name in names}
        for tolerance in goal.path_tolerance:
            if tolerance.name in path and tolerance.position != 0.0:
                path[tolerance.name] = tolerance.position
        for tolerance in goal.goal_tolerance:
            if tolerance.name in end and tolerance.position != 0.0:
                end[tolerance.name] = tolerance.position
        if self._mode == 'velocity':
            for name in names:
                self._pid[name].set_kp(parameter(f'{name}_kp'))
                self._pid[name].set_ki(parameter(f'{name}_ki'))
                self._pid[name].set_kd(parameter(f'{name}_kd'))
                self._pid[name].initialize()
        return goal_time, parameter('stopped_velocity_tolerance'), path, end

    def _path(self, trajectory):
        """Turn the goal's waypoints into a smooth path that starts where the arm is now.

        Returns:
            (Trajectory, with_velocities, with_accelerations): the flags say what the
            goal's waypoints carried, which is what the feed-forward command passes on.
        """
        names = list(trajectory.joint_names)
        points = list(trajectory.points)
        times = [_seconds(point.time_from_start) for point in points]
        positions = [list(point.positions) for point in points]
        with_velocities = all(len(point.velocities) == len(names) for point in points)
        with_accelerations = with_velocities and all(
            len(point.accelerations) == len(names) for point in points)
        velocities = [list(point.velocities) for point in points] if with_velocities else None
        accelerations = [list(point.accelerations) for point in points] if with_accelerations else None

        # The arm is to be at rest at the end of a path, whatever its last waypoint says
        # (as in the SDK's server).
        if velocities is not None:
            velocities[-1] = [0.0] * len(names)
        if accelerations is not None:
            accelerations[-1] = [0.0] * len(names)

        if times[0] > 0.0:
            # The first waypoint lies in the future: start from the arm's current posture.
            now = self._limb.joint_angles()
            times.insert(0, 0.0)
            positions.insert(0, [now[name] for name in names])
            if velocities is not None:
                velocities.insert(0, [0.0] * len(names))
            if accelerations is not None:
                accelerations.insert(0, [0.0] * len(names))
        path = make_trajectory(
            self._curve,
            names,
            times,
            positions,
            velocities,
            accelerations
        )
        return path, with_velocities, with_accelerations

    # --- Commanding the arm ----------------------------------------------------------------

    def _command(self, names, positions, velocities, accelerations, elapsed, feed_forward):
        """Send one command for a point of the path, in the server's mode."""
        limb = self._limb
        if self._mode == 'velocity':
            actual = limb.joint_angles()
            limb.set_joint_velocities({
                name: self._pid[name].compute_output(position - actual[name])
                for name, position in zip(names, positions)
            })
            return
        raw = self._mode == 'position_w_id'
        limb.set_joint_positions(dict(zip(names, positions)), raw=raw)
        if raw:
            self._send_feed_forward(names, positions, velocities, accelerations, elapsed, feed_forward)

    def _send_feed_forward(self, names, positions, velocities, accelerations, elapsed, feed_forward):
        # The robot expects the values in the arm's own joint order.
        order = [names.index(joint) for joint in self._limb.joint_names()]
        with_velocities, with_accelerations = feed_forward
        point = JointTrajectoryPoint(positions=[float(positions[i]) for i in order])
        if with_velocities:
            point.velocities = [float(velocities[i]) for i in order]
        if with_accelerations:
            point.accelerations = [float(accelerations[i]) for i in order]
        _duration(point, elapsed)
        self._pub_feed_forward.publish(point)

    def _stopped(self, goal_handle, number):
        """Reasons to stop that have nothing to do with the path. Returns (code, text) or None."""
        if not ros_is_running() or not self._alive:
            return _CODES.INVALID_GOAL, 'the server is shutting down'
        if goal_handle.is_cancel_requested:
            return _CODES.SUCCESSFUL, 'cancelled'
        if number != self._newest_goal:
            return _CODES.INVALID_GOAL, 'replaced by a newer path for this arm'
        if not self._robot.state().enabled:
            return _CODES.INVALID_GOAL, 'the robot is not enabled'
        if self._limb.joint_state_age() > baxter_interface.settings.STATE_TIMEOUT:
            return _CODES.INVALID_GOAL, 'the arm stopped reporting its joints (connection lost)'
        return None

    def _follow(self, goal_handle, number):
        """Take the arm along the path.

        Returns:
            (error code, text, hold): hold is {joint: angle} to keep the arm at
            afterwards, or None for no holding.
        """
        goal = goal_handle.request
        trajectory = goal.trajectory
        if len(trajectory.points) == 0:
            return _CODES.INVALID_GOAL, 'empty path', None
        limb = self._limb
        names = list(trajectory.joint_names)
        try:
            path, with_velocities, with_accelerations = self._path(trajectory)
            goal_time, stopped_velocity, path_limit, goal_limit = self._tolerances(goal, names)
        except ValueError as error:
            return _CODES.INVALID_GOAL, str(error), None
        feed_forward = (with_velocities, with_accelerations)
        last = path.positions[-1]
        zeros = [0.0] * len(names)

        def here():
            angles = limb.joint_angles()
            return {name: angles[name] for name in names}

        try:
            # A path may ask to start at a later moment.
            start_at = trajectory.header.stamp.sec + trajectory.header.stamp.nanosec * 1e-9
            wait = start_at - self._node.get_clock().now().nanoseconds * 1e-9 if start_at > 0 else 0.0
            begin = self._now() + max(0.0, wait)
            while self._now() < begin:
                stop = self._stopped(goal_handle, number)
                if stop:
                    return stop[0], stop[1], None
                self._sleep(0.01)

            period = 1.0 / self._rate
            start = self._now()
            next_beat = start
            beat = 0
            while True:
                elapsed = self._now() - start
                on_path = elapsed < path.duration
                if not on_path and elapsed >= path.duration + goal_time:
                    break
                stop = self._stopped(goal_handle, number)
                if stop:
                    replaced = number != self._newest_goal
                    return stop[0], stop[1], None if replaced else here()

                # After the path's end the last waypoint is commanded for goal_time more
                # seconds, to give the arm time to arrive.
                if on_path:
                    positions, velocities, accelerations = path.sample(elapsed)
                else:
                    positions, velocities, accelerations = last, zeros, zeros
                actual = limb.joint_angles()
                for name, position in zip(names, positions):
                    error = position - actual[name]
                    if 0.0 <= path_limit[name] <= abs(error):
                        return (_CODES.PATH_TOLERANCE_VIOLATED,
                                f'{name} is {abs(error):.3f} rad away from the path '
                                f'(limit {path_limit[name]:g})', here())

                self._command(names, positions, velocities, accelerations, elapsed, feed_forward)
                if beat % FEEDBACK_EVERY == 0:
                    self._report(goal_handle, names, positions, actual, elapsed)
                beat += 1

                # A fixed beat, as in dataflow.wait_for(): the work's own time must not
                # stretch the period.
                next_beat += period
                pause = next_beat - self._now()
                if pause > 0:
                    self._sleep(pause)
                else:
                    next_beat = self._now()

            # The end: is the arm where it should be, and at rest?
            actual = limb.joint_angles()
            self._report(goal_handle, names, last, actual, self._now() - start)
            end = dict(zip(names, last))
            for name in names:
                error = abs(end[name] - actual[name])
                if 0.0 < goal_limit[name] < error:
                    return (_CODES.GOAL_TOLERANCE_VIOLATED,
                            f'{name} is {error:.3f} rad from the last waypoint at the end '
                            f'(limit {goal_limit[name]:g})', end)
            speeds = limb.joint_velocities()
            fastest = max(abs(speeds.get(name, 0.0)) for name in names)
            if 0.0 < stopped_velocity < fastest:
                return (_CODES.GOAL_TOLERANCE_VIOLATED,
                        f'the arm is still moving at the end ({fastest:.2f} rad/s, '
                        f'limit {stopped_velocity:g})', end)
            return _CODES.SUCCESSFUL, '', end
        except Exception as error:  # report it as the goal's result instead of dying
            return _CODES.INVALID_GOAL, f'{type(error).__name__}: {error}', None

    def _now(self):
        """Seconds by the clock paths are timed with: the computer's own steady clock."""
        return time.monotonic()

    def _sleep(self, seconds):
        """Wait `seconds` by the same clock as _now()."""
        if seconds <= 0:
            return
        time.sleep(seconds)

    def _hold(self, number, angles):
        """Keep the arm at `angles` until a new goal, a hand on the cuff, or a disabled robot."""
        names = list(angles)
        positions = [angles[name] for name in names]
        zeros = [0.0] * len(names)
        full = set(names) == set(self._limb.joint_names())
        period = 1.0 / self._rate
        with self._moving:
            start = time.monotonic()
            for joint in names:
                self._pid[joint].initialize()
            while (ros_is_running() and self._alive and number == self._newest_goal
                   and self._robot.state().enabled
                   and self._limb.joint_state_age() <= baxter_interface.settings.STATE_TIMEOUT):
                if self._cuff.state:
                    # Someone took the arm by its cuff: let go of it.
                    self._limb.exit_control_mode()
                    return
                if self._mode == 'velocity':
                    self._limb.set_joint_velocities(dict(zip(names, zeros)))
                else:
                    raw = self._mode == 'position_w_id' and full
                    self._limb.set_joint_positions(angles, raw=raw)
                    if raw:
                        self._send_feed_forward(
                            names,
                            positions,
                            zeros,
                            zeros,
                            time.monotonic() - start,
                            (True, True)
                        )
                time.sleep(period)

    @staticmethod
    def _report(goal_handle, names, wanted, actual, elapsed):
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(names)
        feedback.desired = JointTrajectoryPoint(positions=[float(p) for p in wanted])
        feedback.actual = JointTrajectoryPoint(positions=[actual[name] for name in names])
        feedback.error = JointTrajectoryPoint(
            positions=[float(p) - actual[name] for name, p in zip(names, wanted)])
        for point in (feedback.desired, feedback.actual, feedback.error):
            _duration(point, elapsed)
        goal_handle.publish_feedback(feedback)


def main():
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('-l', '--limb', choices=['both', 'left', 'right'], default='both',
                        help='which arm to offer the action for (default: both)')
    parser.add_argument('-r', '--rate', type=float, default=100.0,
                        help='commands per second (default 100)')
    parser.add_argument('-m', '--mode', choices=MODES, default='position',
                        help='how the arm is commanded (default: position; the SDK used '
                             'position_w_id, which bypasses collision avoidance)')
    parser.add_argument('--curve', choices=CURVES, default='smooth',
                        help="the curve between the waypoints: smooth (this library's, the "
                             "default) or sdk (the Baxter SDK's spline)")
    args = parser.parse_args(remove_ros_args()[1:])

    sides = ['left', 'right'] if args.limb == 'both' else [args.limb]
    try:
        with baxter_interface.setup():
            robot = RobotEnable()
            limbs = [Limb(side) for side in sides]
            cuffs = [DigitalIO(f'{side}_lower_cuff') for side in sides]
    except (TimeoutError, ConnectionError) as error:
        raise SystemExit(str(error))

    # A node of its own: actions need several things handled at the same time (a goal
    # being followed, a cancel request arriving), which the library's shared node, made
    # only for receiving, does not do. Named as in the SDK.
    suffix = '' if args.limb == 'both' else f'_{args.limb}'
    # In the namespace of the robot this server was started for, as the library's own node.
    namespace = get_node().get_namespace()
    node = rclpy.create_node(
        f'rsdk_{args.mode}_joint_trajectory_action_server{suffix}',
        namespace=namespace
    )
    defaults = DEFAULTS[args.mode]
    node.declare_parameter('goal_time', defaults['goal_time'])
    node.declare_parameter('stopped_velocity_tolerance', defaults['stopped_velocity_tolerance'])
    for limb in limbs:
        for joint in limb.joint_names():
            node.declare_parameter(f'{joint}_goal', defaults['goal'])
            node.declare_parameter(f'{joint}_trajectory', defaults['trajectory'])
            if args.mode == 'velocity':
                for gain, value in PID_DEFAULTS.items():
                    node.declare_parameter(f'{joint}_{gain}', value)

    servers = [
        ArmTrajectoryServer(
            node,
            limb,
            cuff,
            robot,
            args.mode,
            args.rate,
            args.curve
        )
        for limb, cuff in zip(limbs, cuffs)
    ]
    if args.mode == 'position_w_id':
        for server, side in zip(servers, sides):
            if not server.feed_forward_connected():
                raise SystemExit(
                    'Nothing listens on '
                    + dataflow.full_name(node, f'robot/limb/{side}/inverse_dynamics_command')
                    + ', which position_w_id mode needs. ' + dataflow.BRIDGE_HINT
                    + ' Or use -m position.')

    # The robot reports its joints at this rate (as the SDK's server asked for).
    rate_publisher = node.create_publisher(
        UInt16,
        'robot/joint_state_publish_rate',
        10
    )
    if dataflow.wait_for_listener(rate_publisher, timeout=3.0):
        rate_publisher.publish(UInt16(data=int(args.rate)))

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    print(f'Joint trajectory action server running in {args.mode} mode ({args.curve} curve) '
          'for: ' + ', '.join(sides) + '. Ctrl+C to stop.')
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\nStopping.')
    finally:
        for server in servers:
            server.shutdown()
        executor.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
