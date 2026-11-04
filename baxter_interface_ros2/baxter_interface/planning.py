"""Motions planned by MoveIt: around the robot's own body and around things in its way.

New in the ROS 2 library. Limb.move_to_joint_positions() and Limb.move_to_pose() move an
arm straight to its target and rely on the robot's own collision avoidance to stop it.
MoveIt instead looks for a path first: one on which the arm touches neither the rest of
the robot nor the objects you tell it about, and only then moves.

    from baxter_interface.planning import MotionPlanner

    planner = MotionPlanner()
    planner.add_box('table', size=(0.8, 1.6, 0.05), position=(0.8, 0.0, -0.2))
    planner.move_to_pose('left', position=(0.7, 0.3, 0.2))
    planner.move_to_named('both', 'neutral')

Needs MoveIt running (docs/moveit.md):  docker compose run --rm moveit
The robot must be enabled.

How it works: MoveIt's program "move_group" offers the action "move_action". A request
names a planning group (an arm, or both), a target and how fast to go; move_group plans,
turns the path into a timed motion and sends it to baxter_interface's joint trajectory
action server, which makes the arm follow it. This class writes the requests.
"""

from __future__ import annotations

import time
from typing import Any, NamedTuple, Sequence

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK
from rclpy.action.client import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from . import settings
from ._core.node import get_node
from ._core.services import call_service
from .limb import Limb

# The planning groups of moveit_config/config/baxter.srdf, by the names used here.
GROUPS: dict[str, str] = {'left': 'left_arm', 'right': 'right_arm', 'both': 'both_arms'}
JOINTS = ('s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2')
# The frame positions are given in: the robot's base, as everywhere in this library.
FRAME = 'base'

# MoveIt's result numbers, as far as they have something useful to say.
_REASONS: dict[int, str] = {
    MoveItErrorCodes.PLANNING_FAILED: 'no path was found',
    MoveItErrorCodes.FAILURE: 'no path was found; the target may be out of reach, or touch '
                              'the robot itself or an object in the scene',
    MoveItErrorCodes.INVALID_MOTION_PLAN: 'the path that was found is not valid any more '
                                          '(something moved while planning)',
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: 'the scene changed while the arm moved',
    MoveItErrorCodes.CONTROL_FAILED: 'the arm did not follow the planned motion (is the '
                                     'robot enabled, and the joint trajectory action server running?)',
    MoveItErrorCodes.TIMED_OUT: 'planning took longer than allowed',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'the arm is touching something where it is '
                                               'now, according to the scene',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'the target touches something, the robot itself or an object in the scene',
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'the arm ended too far from the target',
    MoveItErrorCodes.NO_IK_SOLUTION: 'the hand cannot reach that pose',
    MoveItErrorCodes.INVALID_GROUP_NAME: 'MoveIt does not know that planning group',
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'the planner cannot take this kind of target '
                                               '(CHOMP takes only joint angles: use '
                                               'move_to_joint_positions or move_to_named)',
    MoveItErrorCodes.PREEMPTED: 'the motion was cancelled',
}


Vector = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]   # x, y, z, w


def _multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """The turn b followed by the turn a, as one quaternion."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _inverse(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def _turn(q: Quaternion, v: Vector) -> Vector:
    """The vector v turned by the quaternion q."""
    x, y, z, _ = _multiply(_multiply(q, (v[0], v[1], v[2], 0.0)), _inverse(q))
    return (x, y, z)


def tip_offset(model_position: Vector, model_orientation: Quaternion,
               reported_position: Vector, reported_orientation: Quaternion) -> tuple[Vector, Quaternion]:
    """Return where the hand tip the robot reports sits, seen from the robot model's hand tip.

    The two differ when a gripper is mounted that MoveIt's robot model does not have: the
    robot then reports the gripper's tip, the model ends at the mounting plate. With the
    gripper in the model too, the two (nearly) agree. The result is the same in every
    posture.
    """
    apart = tuple(r - m for r, m in zip(reported_position, model_position))
    return (_turn(_inverse(model_orientation), apart),          # type: ignore[arg-type]
            _multiply(_inverse(model_orientation), reported_orientation))


def model_tip_for(position: Vector, orientation: Quaternion,
                  offset: tuple[Vector, Quaternion]) -> tuple[Vector, Quaternion]:
    """Return the pose of the robot model's hand tip that puts the reported tip at a pose."""
    apart, turned = offset
    model_orientation = _multiply(orientation, _inverse(turned))
    shift = _turn(model_orientation, apart)
    return (tuple(p - s for p, s in zip(position, shift)), model_orientation)   # type: ignore[return-value]


# How often a search for a path is tried, and after which results another try makes sense.
ATTEMPTS = 3
_WORTH_ANOTHER_TRY = (MoveItErrorCodes.FAILURE, MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.TIMED_OUT)


class PlanningError(RuntimeError):
    """MoveIt could not plan or run a motion. The message says why."""


class Plan(NamedTuple):
    """A planned motion: the joints it moves, how many points the path has, and how long
    it takes (seconds)."""
    joints: list[str]
    points: int
    duration: float


class MotionPlanner:
    """Plan and run arm motions with MoveIt.

    Available methods:
        move_to_pose()             - hand to a place (and orientation)
        move_to_joint_positions()  - arm to joint angles
        move_to_named()            - arm(s) to a named posture, e.g. 'neutral'
        add_box(), remove_object(), clear_objects()
                                   - things in the room the arms must not touch
    Every move_... takes plan_only=True: plan, show it in RViz, do not move.
    """

    def __init__(self, node: Node | None = None, connect_timeout: float = 15.0) -> None:
        """
        Parameters:
            node:            optional ROS 2 node to use (default: the shared background node)
            connect_timeout: seconds to wait for MoveIt (default 15)

        Raises:
            ConnectionError if MoveIt does not answer.
        """
        self._node = get_node(node)
        self._limbs: dict[str, Limb] = {}
        self._offsets: dict[str, tuple[Vector, Quaternion]] = {}
        self._fk = self._node.create_client(GetPositionFK, 'compute_fk')
        self._objects: set[str] = set()
        self._move = ActionClient(self._node, MoveGroup, 'move_action')
        self._scene = self._node.create_client(ApplyPlanningScene, 'apply_planning_scene')
        if not self._move.wait_for_server(timeout_sec=connect_timeout):
            raise ConnectionError(
                'MoveIt does not answer (its action "move_action" is not there). Start it '
                'first: docker compose run --rm moveit  (docs/moveit.md).')

    # --- Moving -----------------------------------------------------------------------------

    def move_to_pose(
        self,
        limb: str,
        position: Sequence[float],
        orientation: Sequence[float] | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
        plan_only: bool = False,
        planning_time: float = 5.0
    ) -> Plan:
        """Move a hand to a place along a path that touches nothing.

        The pose means the same hand tip as Limb.hand_position(): with a gripper mounted,
        the gripper's tip. (Where MoveIt's robot model has no gripper it ends at the
        mounting plate; the difference is measured once per arm, the first time that arm
        is used here, while the arm stands still.)

        Parameters:
            limb:        'left' or 'right'
            position:    (x, y, z) in metres in the robot's base frame, as Limb.hand_position()
            orientation: (x, y, z, w) quaternion; left out: as the hand is turned now
            speed:       0 to 1, share of the top speed; 1 is the robot's full speed
                         (default: settings.PLANNING_SPEED, 0.1)
            acceleration: 0 to 1, share of the top acceleration, which decides how
                         sharply a motion starts and stops (default:
                         settings.PLANNING_ACCELERATION, 0.1)
            plan_only:   True: plan and show the path in RViz, but do not move
            planning_time: seconds MoveIt may search for a path, per try (default 5;
                         a search that finds nothing is tried up to three times)

        Returns:
            Plan(joints, points, duration)

        Raises:
            PlanningError with the reason if no path is found or the motion fails.
        """
        if limb not in ('left', 'right'):
            raise ValueError(f"limb must be 'left' or 'right', not {limb!r}")
        if orientation is None:
            orientation = tuple(self._limb(limb).hand_orientation())
        position, orientation = model_tip_for(
            tuple(float(v) for v in position),         # type: ignore[arg-type]
            tuple(float(v) for v in orientation),      # type: ignore[arg-type]
            self._offset(limb)
        )
        link = f'{limb}_gripper'

        place = PositionConstraint()
        place.header.frame_id = FRAME
        place.link_name = link
        # The target as a tiny ball: the tip of the hand has to end inside it.
        place.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002]))
        centre = Pose()
        centre.position.x, centre.position.y, centre.position.z = (float(v) for v in position)
        centre.orientation.w = 1.0
        place.constraint_region.primitive_poses.append(centre)
        place.weight = 1.0

        turned = OrientationConstraint()
        turned.header.frame_id = FRAME
        turned.link_name = link
        (turned.orientation.x, turned.orientation.y,
         turned.orientation.z, turned.orientation.w) = (float(v) for v in orientation)
        turned.absolute_x_axis_tolerance = 0.01
        turned.absolute_y_axis_tolerance = 0.01
        turned.absolute_z_axis_tolerance = 0.01
        turned.weight = 1.0

        goal = Constraints(position_constraints=[place], orientation_constraints=[turned])
        return self._request(GROUPS[limb], goal, speed, acceleration, plan_only, planning_time)

    def move_to_joint_positions(
        self,
        limb: str,
        positions: dict[str, float],
        speed: float | None = None,
        acceleration: float | None = None,
        plan_only: bool = False,
        planning_time: float = 5.0
    ) -> Plan:
        """Move an arm (or both) to joint angles along a path that touches nothing.

        Parameters:
            limb:      'left', 'right' or 'both'
            positions: {joint name: angle in rad}, e.g. {'left_w1': 0.5}; joints of the
                       group that are left out are free to end anywhere
            speed, acceleration, plan_only, planning_time: as in move_to_pose()

        Returns:
            Plan(joints, points, duration)

        Raises:
            PlanningError with the reason if no path is found or the motion fails.
        """
        if limb not in GROUPS:
            raise ValueError(f"limb must be 'left', 'right' or 'both', not {limb!r}")
        goal = Constraints(joint_constraints=[
            JointConstraint(
                joint_name=name,
                position=float(angle),
                tolerance_above=0.001,
                tolerance_below=0.001,
                weight=1.0
            )
            for name, angle in positions.items()
        ])
        return self._request(GROUPS[limb], goal, speed, acceleration, plan_only, planning_time)

    def move_to_named(
        self,
        limb: str,
        name: str = 'neutral',
        speed: float | None = None,
        acceleration: float | None = None,
        plan_only: bool = False,
        planning_time: float = 5.0
    ) -> Plan:
        """Move an arm (or both) to a named posture along a path that touches nothing.

        Parameters:
            limb: 'left', 'right' or 'both'
            name: 'neutral' (the posture of Limb.move_to_neutral())
            speed, acceleration, plan_only, planning_time: as in move_to_pose()
        """
        if name != 'neutral':
            raise ValueError(f"Unknown posture {name!r}; this library knows 'neutral'.")
        sides = ('left', 'right') if limb == 'both' else (limb,)
        neutral = (0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0)
        positions = {f'{side}_{joint}': angle for side in sides for joint, angle in zip(JOINTS, neutral)}
        return self.move_to_joint_positions(limb, positions, speed, acceleration, plan_only, planning_time)

    def _limb(self, side: str) -> Limb:
        if side not in self._limbs:
            self._limbs[side] = Limb(side, node=self._node)
        return self._limbs[side]

    def _offset(self, side: str) -> tuple[Vector, Quaternion]:
        """Where the hand tip the robot reports sits, seen from the model's (measured once)."""
        if side not in self._offsets:
            request = GetPositionFK.Request()
            request.header.frame_id = FRAME
            request.fk_link_names = [f'{side}_gripper']
            request.robot_state.is_diff = True      # the posture the robot is in now
            answer = call_service(self._fk, request)
            if answer.error_code.val != MoveItErrorCodes.SUCCESS or not answer.pose_stamped:
                raise PlanningError(f"MoveIt could not say where the {side} hand is.")
            pose = answer.pose_stamped[0].pose
            limb = self._limb(side)
            self._offsets[side] = tip_offset(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                tuple(limb.hand_position()),           # type: ignore[arg-type]
                tuple(limb.hand_orientation())         # type: ignore[arg-type]
            )
        return self._offsets[side]

    def _request(
        self,
        group: str,
        goal: Constraints,
        speed: float | None,
        acceleration: float | None,
        plan_only: bool,
        planning_time: float
    ) -> Plan:
        speed = settings.PLANNING_SPEED if speed is None else speed
        acceleration = settings.PLANNING_ACCELERATION if acceleration is None else acceleration
        request = MoveGroup.Goal()
        request.request.group_name = group
        request.request.goal_constraints = [goal]
        request.request.num_planning_attempts = 5
        request.request.allowed_planning_time = float(planning_time)
        request.request.max_velocity_scaling_factor = min(max(float(speed), 0.01), 1.0)
        request.request.max_acceleration_scaling_factor = min(max(float(acceleration), 0.01), 1.0)
        request.request.start_state.is_diff = True          # start from where the robot is
        request.planning_options.plan_only = bool(plan_only)
        request.planning_options.planning_scene_diff.is_diff = True
        request.planning_options.planning_scene_diff.robot_state.is_diff = True

        # The planner searches at random, so a search that finds nothing is repeated a
        # few times before giving up. (A failure while the arm moves is not repeated.)
        for attempt in range(ATTEMPTS):
            accepted = _wait(self._move.send_goal_async(request), 10.0, 'MoveIt did not take the request')
            if not accepted.accepted:
                raise PlanningError('MoveIt refused the request.')
            # Planning time plus the motion itself; a slow simulated robot takes a while.
            result = _wait(accepted.get_result_async(), planning_time + 180.0,
                           'MoveIt did not finish the motion').result
            if result.error_code.val not in _WORTH_ANOTHER_TRY:
                break

        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            reason = _REASONS.get(code, f'MoveIt reports error {code}')
            what = 'planned' if plan_only else 'planned and run'
            raise PlanningError(f'The motion of {group} could not be {what}: {reason}.')
        path = result.planned_trajectory.joint_trajectory
        last = path.points[-1].time_from_start if path.points else None
        duration = (last.sec + last.nanosec * 1e-9) if last is not None else 0.0
        return Plan(list(path.joint_names), len(path.points), duration)

    # --- The room ---------------------------------------------------------------------------

    def add_box(
        self,
        name: str,
        size: Sequence[float],
        position: Sequence[float],
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0)
    ) -> None:
        """Tell MoveIt about a box-shaped thing the arms must not touch (a table, a wall).

        Parameters:
            name:        a name of your choice; a box with the same name is replaced
            size:        (x, y, z) edge lengths in metres
            position:    (x, y, z) of the box's centre in the robot's base frame
            orientation: (x, y, z, w) quaternion (default: not turned)
        """
        thing = CollisionObject()
        thing.header.frame_id = FRAME
        thing.id = name
        thing.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size]))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = (float(v) for v in orientation)
        thing.primitive_poses.append(pose)
        thing.operation = CollisionObject.ADD
        self._apply([thing])
        self._objects.add(name)

    def remove_object(self, name: str) -> None:
        """Take a thing out of MoveIt's picture of the room again."""
        thing = CollisionObject()
        thing.header.frame_id = FRAME
        thing.id = name
        thing.operation = CollisionObject.REMOVE
        self._apply([thing])
        self._objects.discard(name)

    def clear_objects(self) -> None:
        """Take out everything this MotionPlanner has added."""
        for name in list(self._objects):
            self.remove_object(name)

    def _apply(self, things: list[CollisionObject]) -> None:
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = things
        response = call_service(self._scene, ApplyPlanningScene.Request(scene=scene))
        if not response.success:
            raise PlanningError('MoveIt did not accept the change to its picture of the room.')


def _wait(future: Any, seconds: float, what: str) -> Any:
    """Wait for an answer that the receiving thread delivers; TimeoutError after `seconds`."""
    end = time.monotonic() + seconds
    while not future.done():
        if time.monotonic() > end:
            raise TimeoutError(f'{what} within {seconds:g} s.')
        time.sleep(0.01)
    return future.result()
