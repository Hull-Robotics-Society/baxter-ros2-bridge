"""Read and command one of Baxter's arms.

Ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink Robotics, BSD license;
see LICENSE). Same methods and behaviour; ROS 1 calls replaced by ROS 2 ones. Added a lock,
because messages arrive in a background thread while the program reads the values, and
a per-arm lock so that blocking moves from different threads run one after another.
"""

from __future__ import annotations

import threading
import time
from copy import deepcopy
from typing import Any, Mapping, NamedTuple, Sequence

from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Quaternion as QuaternionMsg
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header

from baxter_core_msgs.msg import EndpointState, JointCommand
from baxter_core_msgs.srv import SolvePositionIK

from . import settings
from ._core import at_once, connecting, dataflow
from ._core.locks import part_lock
from ._core.node import get_node
from ._core.services import call_service
from ._core.trajectory import CURVES, largest_error, make_trajectory, waypoint_times

# The ROS 1 SDK published these two "latched": the last value stays available to anyone who
# subscribes later. TRANSIENT_LOCAL is the ROS 2 equivalent.
_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# For the arm's state, which the robot sends 100 times per second: keep only the newest
# message and don't ask for lost ones to be sent again. A program that is busy for a
# moment then continues with the current state instead of working through a queue of
# old ones (which made it fall further behind, and loaded the bridge with repeats).
_NEWEST_ONLY = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class Limb:
    """Interface class for a limb ('left' or 'right') on the Baxter robot.

    Reading:
        joint_names(), joint_angles(), joint_velocities(), joint_efforts()
        hand_position(), hand_orientation()     where the hand is (metres, "base" frame)
        endpoint_pose(), endpoint_velocity(), endpoint_effort()     as the SDK had them
        joint_state_age()                       seconds since the arm last reported
    Moving, and waiting until done:
        move_to_neutral()
        move_to_joint_positions({joint: angle})
        move_to_pose(x, y, z, orientation)      the hand to a place
        move_by(x, y, z)                        the hand by a distance
        move_through([waypoint, ...])           through several postures or places in
                                                one smooth motion
        solve_pose(x, y, z, orientation)        joint angles for a place, without moving
    For a control loop of your own (send at least 5 times per second):
        set_joint_positions(), set_joint_velocities(), set_joint_torques(),
        exit_control_mode()
    Settings:
        set_joint_position_speed(0.0 to 1.0), set_command_timeout(seconds)
    """

    # Containers
    class Point(NamedTuple):
        """A place (metres) or a direction: x forward, y to the robot's left, z up."""

        x: float
        y: float
        z: float

    class Quaternion(NamedTuple):
        """How something is turned, as a quaternion."""

        x: float
        y: float
        z: float
        w: float

    _JOINT_NAMES = {
        'left': ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                 'left_w0', 'left_w1', 'left_w2'],
        'right': ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                  'right_w0', 'right_w1', 'right_w2'],
    }

    def __init__(self, limb: str, node: Node | None = None) -> None:
        if limb not in self._JOINT_NAMES:
            raise ValueError(f"limb must be 'left' or 'right', not {limb!r}")
        self.name = limb
        self._node = get_node(node)
        self._lock = threading.Lock()  # guards the stored values
        # Held by blocking moves, so two threads can't move this arm at the same time.
        self._motion_lock = part_lock(f'limb/{limb}', self._node)
        self._joint_angle: dict[str, float] = {}
        self._joint_velocity: dict[str, float] = {}
        self._joint_effort: dict[str, float] = {}
        self._cartesian_pose: dict[str, Any] = {}
        self._cartesian_velocity: dict[str, Any] = {}
        self._cartesian_effort: dict[str, Any] = {}
        self._ik_client: Client | None = None  # created on first use (solve_pose)
        self._joint_states = dataflow.Freshness(f'{limb} arm', dataflow.full_name(self._node, 'robot/joint_states'))

        ns = f'robot/limb/{limb}/'
        self._pub_speed_ratio = self._node.create_publisher(
            Float64, 
            ns + 'set_speed_ratio', 
            _LATCHED
        )
        self._pub_joint_cmd = self._node.create_publisher(
            JointCommand, 
            ns + 'joint_command', 
            10
        )
        self._pub_joint_cmd_timeout = self._node.create_publisher(
            Float64, 
            ns + 'joint_command_timeout', 
            _LATCHED
        )
        self._cartesian_state_sub = self._node.create_subscription(
            EndpointState, 
            ns + 'endpoint_state', 
            self._on_endpoint_states, 
            _NEWEST_ONLY
        )
        self._joint_state_sub = self._node.create_subscription(
            JointState, 
            'robot/joint_states', 
            self._on_joint_states, 
            _NEWEST_ONLY
        )

        # Wait until connected, now or at the end of a setup() block.
        connecting.finish_or_defer(self._wait_until_connected)

    def _wait_until_connected(self) -> None:
        limb = self.name
        ns = f'robot/limb/{limb}/'
        dataflow.wait_for(
            lambda: len(self._joint_angle) > 0,
            timeout=settings.CONNECT_TIMEOUT,
            timeout_msg=f'{limb.capitalize()} limb init failed to get current joint_states '
                        'from /robot/joint_states. ' + dataflow.BRIDGE_HINT
                        + ' Is ROS_DOMAIN_ID the same as the bridge\'s?',
        )
        # Commands must be able to reach the robot before the first one is sent: the
        # bridge connects the command topic a moment after this program created its
        # publisher. Without this, the first second of a control loop of your own
        # (set_joint_positions and the like) would be lost without notice.
        if not dataflow.wait_for_listener(self._pub_joint_cmd):
            raise ConnectionError(
                f'Commands for the {limb} arm cannot be delivered: nothing listens on '
                f'{ns}joint_command. ' + dataflow.BRIDGE_HINT)
        # Also give the hand's position (endpoint state) a moment to arrive, so that
        # endpoint_pose() right after creating the Limb is not empty. On the real robot it
        # came in a little later than the joint states (5 Oct 2026). No error if it stays
        # away: not every setup sends it, and endpoint_pose() then returns an empty dict.
        dataflow.wait_for(
            lambda: len(self._cartesian_pose) > 0,
            timeout=5.0,
            raise_on_error=False
        )

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_states.mark()
        with self._lock:
            for idx, name in enumerate(msg.name):
                if self.name in name:
                    self._joint_angle[name] = msg.position[idx]
                    if idx < len(msg.velocity):
                        self._joint_velocity[name] = msg.velocity[idx]
                    if idx < len(msg.effort):
                        self._joint_effort[name] = msg.effort[idx]

    def _on_endpoint_states(self, msg: EndpointState) -> None:
        p, t, w = msg.pose, msg.twist, msg.wrench
        with self._lock:
            # _pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}
            self._cartesian_pose = {
                'position': self.Point(
                        p.position.x, 
                        p.position.y, 
                        p.position.z
                    ),
                'orientation': self.Quaternion(
                        p.orientation.x, 
                        p.orientation.y,
                        p.orientation.z, 
                        p.orientation.w
                    ),
            }
            # _twist = {'linear': (x, y, z), 'angular': (x, y, z)}
            self._cartesian_velocity = {
                'linear': self.Point(
                        t.linear.x, 
                        t.linear.y, 
                        t.linear.z
                    ),
                'angular': self.Point(
                        t.angular.x, 
                        t.angular.y, 
                        t.angular.z
                    ),
            }
            # _wrench = {'force': (x, y, z), 'torque': (x, y, z)}
            self._cartesian_effort = {
                'force': self.Point(
                    w.force.x, 
                    w.force.y, 
                    w.force.z
                ),
                'torque': self.Point(
                    w.torque.x, 
                    w.torque.y, 
                    w.torque.z
                ),
            }

    # --- Reading ---------------------------------------------------------------------------

    def joint_names(self) -> list[str]:
        """Return the names of the joints for this limb."""
        return list(self._JOINT_NAMES[self.name])

    def joint_angle(self, joint: str) -> float:
        """Return the requested joint angle (rad)."""
        with self._lock:
            return self._joint_angle[joint]

    def joint_state_age(self) -> float:
        """Return the seconds since the arm last reported its joints (normally about 0.01).

        The values returned by joint_angles() and the other read methods are the newest
        ones received. If the connection to the robot is lost they stay as they were;
        this is how a program of your own can tell. Blocking moves check it themselves.
        """
        return self._joint_states.age()

    def joint_angles(self) -> dict[str, float]:
        """Return all joint angles as {joint_name: angle}."""
        with self._lock:
            return deepcopy(self._joint_angle)

    def joint_velocity(self, joint: str) -> float:
        """Return the requested joint velocity (rad/s)."""
        with self._lock:
            return self._joint_velocity[joint]

    def joint_velocities(self) -> dict[str, float]:
        """Return all joint velocities as {joint_name: velocity}."""
        with self._lock:
            return deepcopy(self._joint_velocity)

    def joint_effort(self, joint: str) -> float:
        """Return the requested joint effort (Nm)."""
        with self._lock:
            return self._joint_effort[joint]

    def joint_efforts(self) -> dict[str, float]:
        """Return all joint efforts as {joint_name: effort}."""
        with self._lock:
            return deepcopy(self._joint_effort)

    def endpoint_pose(self) -> dict[str, Any]:
        """Return the Cartesian endpoint pose {position, orientation}."""
        with self._lock:
            return deepcopy(self._cartesian_pose)

    def endpoint_velocity(self) -> dict[str, Any]:
        """Return the Cartesian endpoint twist {linear, angular}."""
        with self._lock:
            return deepcopy(self._cartesian_velocity)

    def endpoint_effort(self) -> dict[str, Any]:
        """Return the Cartesian endpoint wrench {force, torque}."""
        with self._lock:
            return deepcopy(self._cartesian_effort)

    # --- Commanding ------------------------------------------------------------------------

    def set_command_timeout(self, timeout: float) -> None:
        """Set the timeout in seconds for the joint controller."""
        dataflow.publish_once(
            self._pub_joint_cmd_timeout,
            Float64(data=float(timeout)),
            what=f'command time-out setting for the {self.name} arm'
        )

    def exit_control_mode(self, timeout: float = 0.2) -> None:
        """Clean exit from advanced control modes (joint torque or velocity).

        Resets control to joint position mode with the current positions.
        """
        self.set_command_timeout(timeout)
        self.set_joint_positions(self.joint_angles())

    def set_joint_position_speed(self, speed: float) -> None:
        """Set the ratio of max joint speed to use during joint position moves.

        speed: 0.0 to 1.0 (default on the robot: 0.3). Persists until set again.
        """
        dataflow.publish_once(
            self._pub_speed_ratio,
            Float64(data=float(speed)),
            what=f'speed setting for the {self.name} arm'
        )

    def _command(self, values: Mapping[str, float], mode: int) -> None:
        msg = JointCommand()
        msg.mode = mode
        msg.names = list(values.keys())
        msg.command = [float(v) for v in values.values()]
        self._pub_joint_cmd.publish(msg)

    def set_joint_positions(self, positions: Mapping[str, float], raw: bool = False) -> None:
        """Command the joints of this limb to positions: {joint_name: angle}.

        Parameters:
            positions: {joint name: angle in rad}
            raw:       advanced. True sends the angles to the joints unchanged ("raw"
                       position mode, as in the Baxter SDK from 1.0 on). The motion
                       follows the commands more exactly, but the robot's safety layer
                       is bypassed: no speed limit from set_joint_position_speed() and
                       NO COLLISION AVOIDANCE between the arms. Leave it off unless you
                       send closely spaced targets from a controller of your own.
        """
        mode = JointCommand.RAW_POSITION_MODE if raw else JointCommand.POSITION_MODE
        self._command(positions, mode)

    def set_joint_velocities(self, velocities: Mapping[str, float]) -> None:
        """Command the joints of this limb to velocities: {joint_name: velocity}.

        IMPORTANT: must be sent faster than the timeout set with set_command_timeout,
        otherwise the controller switches back to position mode for safety.
        """
        self._command(velocities, JointCommand.VELOCITY_MODE)

    def set_joint_torques(self, torques: Mapping[str, float]) -> None:
        """Command the joints of this limb to torques: {joint_name: torque}.

        IMPORTANT: must be sent faster than the timeout set with set_command_timeout,
        otherwise the controller switches back to position mode for safety.
        """
        self._command(torques, JointCommand.TORQUE_MODE)

    @at_once.command
    def move_to_neutral(self, timeout: float = 15.0) -> None:
        """Command the joints to the center of their joint ranges."""
        angles = dict(zip(self.joint_names(), [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
        self.move_to_joint_positions(angles, timeout)

    @at_once.command
    def move_to_joint_positions(self, positions: Mapping[str, float], timeout: float = 15.0) -> None:
        """Move to positions {joint_name: angle} and wait until they are reached.

        Raises TimeoutError if the joints are not within settings.JOINT_ANGLE_TOLERANCE
        after `timeout` seconds, and ConnectionError if the arm stops reporting its
        joints on the way (the commands then stop, and the arm holds its position).

        Only one such move per arm runs at a time: if another thread is already moving
        this arm, this call waits until that move has finished, then starts.
        """
        with self._motion_lock:
            cmd = self.joint_angles()

            def filtered_cmd():
                # First order filter, 0.2 Hz cutoff: approach the target smoothly.
                for joint in positions:
                    cmd[joint] = 0.012488 * positions[joint] + 0.98751 * cmd[joint]
                return cmd

            def reached():
                # Moving on without news from the arm would be moving blind.
                self._joint_states.require()
                angles = self.joint_angles()
                return all(abs(angle - angles[joint]) < settings.JOINT_ANGLE_TOLERANCE
                           for joint, angle in positions.items() if joint in angles)

            dataflow.wait_for(
                reached,
                timeout=timeout,
                timeout_msg=f'{self.name.capitalize()} limb failed to reach commanded '
                            'joint positions',
                rate=100,
                body=lambda: self.set_joint_positions(filtered_cmd()),
            )

    # --- Moving the hand to a place ----------------------------------------------------------
    #
    # New in the ROS 2 library. The arm is commanded in joint angles; to send the hand to
    # a place, the robot's inverse kinematics service works out the joint angles for it.
    #
    # Places are given in metres in the robot's own frame ("base"), seen from the robot:
    # x points forward, y to the robot's left, z up; the origin is in the robot's torso.
    # hand_position() tells where the hand is now, which is the easiest way to find
    # coordinates: move the arm by hand (hold the cuff) and print it.

    def hand_position(self) -> Limb.Point:
        """Return where the hand is, as (x, y, z) in metres.

        Returns:
            Limb.Point with x (forward), y (left) and z (up)
        """
        return self._hand_pose()['position']

    def hand_orientation(self) -> Limb.Quaternion:
        """Return how the hand is turned, as a quaternion (x, y, z, w)."""
        return self._hand_pose()['orientation']

    def _hand_pose(self) -> dict[str, Any]:
        pose = self.endpoint_pose()
        if not pose:
            raise ConnectionError(
                f'The position of the {self.name} hand is not known: nothing has arrived on '
                + dataflow.full_name(self._node, f'robot/limb/{self.name}/endpoint_state')
                + '. ' + dataflow.BRIDGE_HINT)
        return pose

    def solve_pose(
        self,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        orientation: Sequence[float] | None = None
    ) -> dict[str, float]:
        """Work out the joint angles that put the hand at a place, without moving.

        Parameters:
            x, y, z:     where the hand should be, in metres (see the note above). A
                         coordinate that is left out keeps its current value.
            orientation: how the hand should be turned, as a quaternion (x, y, z, w).
                         Left out: as it is turned now.

        Returns:
            dict {joint name: angle}, ready for move_to_joint_positions()

        Raises:
            ValueError if the arm cannot reach that place with that orientation.
        """
        now = self._hand_pose()
        position = now['position']
        target = (
            position.x if x is None else float(x),
            position.y if y is None else float(y),
            position.z if z is None else float(z),
        )
        turned = now['orientation'] if orientation is None else self.Quaternion(*orientation)

        return self._solve(target, turned)

    def _solve(
        self,
        position: Sequence[float],
        orientation: Limb.Quaternion,
        seed: Mapping[str, float] | None = None
    ) -> dict[str, float]:
        """Ask the robot for the joint angles that put the hand at a place.

        Parameters:
            position:    (x, y, z) in metres
            orientation: Limb.Quaternion
            seed:        optional {joint name: angle}: look for a solution near this
                         posture instead of near the arm's current one

        Returns:
            dict {joint name: angle}

        Raises:
            ValueError if the arm cannot reach the place.
        """
        if self._ik_client is None:
            self._ik_client = self._node.create_client(
                SolvePositionIK,
                f'ExternalTools/{self.name}/PositionKinematicsNode/IKService'
            )
        pose = PoseStamped(
            header=Header(
                stamp=self._node.get_clock().now().to_msg(),
                frame_id='base'
            ),
            pose=Pose(
                position=PointMsg(
                    x=float(position[0]),
                    y=float(position[1]),
                    z=float(position[2])
                ),
                orientation=QuaternionMsg(
                    x=float(orientation.x),
                    y=float(orientation.y),
                    z=float(orientation.z),
                    w=float(orientation.w)
                )
            )
        )

        # First look for a solution near the arm's current posture (or the given one), so
        # that a small step of the hand is a small move of the arm. Only if there is
        # none, let the robot try its other starting points (the arm may then swing into
        # another posture).
        if seed is None:
            attempts = [
                SolvePositionIK.Request(
                    pose_stamp=[pose],
                    seed_mode=SolvePositionIK.Request.SEED_CURRENT
                ),
            ]
        else:
            attempts = [
                SolvePositionIK.Request(
                    pose_stamp=[pose],
                    seed_angles=[JointState(
                        name=list(seed.keys()),
                        position=[float(angle) for angle in seed.values()]
                    )],
                    seed_mode=SolvePositionIK.Request.SEED_USER
                ),
            ]
        attempts.append(
            SolvePositionIK.Request(
                pose_stamp=[pose],
                seed_mode=SolvePositionIK.Request.SEED_AUTO
            )
        )
        for request in attempts:
            response = call_service(self._ik_client, request)
            if len(response.is_valid) > 0 and response.is_valid[0]:
                solution = response.joints[0]
                return dict(zip(solution.name, solution.position))

        raise ValueError(
            f'The {self.name} arm cannot reach x={position[0]:.3f}, y={position[1]:.3f}, '
            f'z={position[2]:.3f} m with this hand orientation. The place may be too far '
            'away or too close to the body, or the hand would have to turn in a way the '
            'wrist cannot.')

    @at_once.command
    def move_to_pose(
        self,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        orientation: Sequence[float] | None = None,
        timeout: float = 15.0
    ) -> None:
        """Move the hand to a place and wait until it is there.

            left.move_to_pose(x=0.7, y=0.3, z=0.1)     # a point in front of the robot
            left.move_to_pose(z=0.3)                    # straight up or down to z = 0.3

        Parameters:
            x, y, z:     where the hand should go, in metres (see the note above). A
                         coordinate that is left out keeps its current value.
            orientation: how the hand should be turned, as a quaternion (x, y, z, w).
                         Left out: as it is turned now.
            timeout:     seconds to wait for the arm to arrive (default 15)

        Raises:
            ValueError if the arm cannot reach the place (nothing moves then), and the
            errors of move_to_joint_positions().

        The hand does not travel in a straight line: each joint goes its own way to its
        new angle. For a straight path, go there in several small steps.
        """
        angles = self.solve_pose(x, y, z, orientation)
        self.move_to_joint_positions(angles, timeout)

    @at_once.command
    def move_by(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, timeout: float = 15.0) -> None:
        """Move the hand by a distance from where it is now, keeping how it is turned.

            left.move_by(z=-0.05)          # 5 cm down
            left.move_by(x=0.1, y=0.02)    # 10 cm forward and 2 cm to the left

        Parameters:
            x, y, z: distance in metres along each direction (forward, left, up)
            timeout: seconds to wait for the arm to arrive (default 15)

        Raises:
            ValueError if the arm cannot reach the place (nothing moves then), and the
            errors of move_to_joint_positions().
        """
        now = self.hand_position()
        self.move_to_pose(
            x=now.x + x,
            y=now.y + y,
            z=now.z + z,
            timeout=timeout
        )

    @at_once.command
    def move_through(
        self,
        waypoints: Sequence[Mapping[str, float] | Sequence[float]],
        speed: float = 0.5,
        seconds: float | None = None,
        timeout: float = 15.0,
        curve: str = 'smooth'
    ) -> None:
        """Move through several waypoints in one smooth motion, without stopping at each.

            left.move_through([posture_1, posture_2, posture_3])
            left.move_through([(0.7, 0.3, 0.1), (0.7, 0.1, 0.3)], seconds=6)

        Parameters:
            waypoints: list; each entry is either
                         - a posture: {joint name: angle}, as joint_angles() returns
                           (joints that are left out keep the angle they have before), or
                         - a place for the hand: (x, y, z) in metres, turned as the hand
                           is turned now. The joint angles for it are worked out before
                           anything moves, each near the waypoint before it.
            speed:     how fast to go: roughly the average speed, in rad/s, of the joint
                       that has furthest to travel between two waypoints (default 0.5)
            seconds:   instead of speed: the time the whole motion should take
            timeout:   seconds the arm gets after the end of the path to settle exactly
                       on the last waypoint (default 15)
            curve:     'smooth' (default): the arm starts and ends at rest.
                       'sdk': the curve of the Baxter SDK's trajectory server, which
                       starts and ends at speed and keeps the acceleration steady at
                       evenly timed waypoints (see _core/bezier.py).

        Raises:
            ValueError if a place cannot be reached or a joint name is wrong (nothing
            moves then); TimeoutError if the arm falls further behind the path than
            settings.PATH_ERROR_LIMIT (it is held back, or the path is faster than the
            arm may go: see set_joint_position_speed()), or does not settle at the end;
            ConnectionError if the arm stops reporting on the way.

        The path starts where the arm is. The arm starts and ends at rest, and between
        waypoints each joint follows a smooth curve; the hand does not travel in straight
        lines. For exact timing between other ROS 2 programs, see the joint trajectory
        action (docs/trajectory-action.md).
        """
        if curve not in CURVES:
            raise ValueError(f"curve must be one of {CURVES}, not {curve!r}")
        with self._motion_lock:
            names = self.joint_names()
            posture = self.joint_angles()
            orientation: Limb.Quaternion | None = None
            points = [[posture[name] for name in names]]
            for number, waypoint in enumerate(waypoints, start=1):
                if isinstance(waypoint, Mapping):
                    unknown = [joint for joint in waypoint if joint not in names]
                    if unknown:
                        raise ValueError(
                            f'Waypoint {number}: the {self.name} arm has no joints {unknown}')
                    posture = {**posture, **waypoint}
                else:
                    if len(waypoint) != 3:
                        raise ValueError(
                            f'Waypoint {number} must be a posture {{joint: angle}} or a place '
                            f'(x, y, z), not {waypoint!r}')
                    if orientation is None:
                        orientation = self.hand_orientation()
                    posture = {**posture, **self._solve(waypoint, orientation, seed=posture)}
                points.append([float(posture[name]) for name in names])
            if len(points) == 1:
                return

            times = waypoint_times(points, speed)
            if seconds is not None:
                if seconds <= 0:
                    raise ValueError('seconds must be greater than 0')
                times = [t * seconds / times[-1] for t in times]
            path = make_trajectory(
                curve,
                names,
                times,
                points
            )
            start = time.monotonic()

            def follow():
                # Moving on without news from the arm would be moving blind.
                self._joint_states.require()
                wanted = path.at(time.monotonic() - start)
                actual = self.joint_angles()
                joint, error = largest_error(wanted, actual)
                if error > settings.PATH_ERROR_LIMIT:
                    self.set_joint_positions(actual)  # hold where the arm is
                    raise TimeoutError(
                        f'{self.name.capitalize()} limb fell behind its path: {joint} is '
                        f'{error:.2f} rad from where it should be. Is the arm held back, or '
                        'the path faster than set_joint_position_speed() allows?')
                self.set_joint_positions(wanted)

            dataflow.wait_for(
                lambda: time.monotonic() - start >= path.duration,
                timeout=float('inf'),
                rate=100,
                body=follow,
            )
            # The path is over; let the arm settle exactly on the last waypoint.
            self.move_to_joint_positions(dict(zip(names, points[-1])), timeout)
