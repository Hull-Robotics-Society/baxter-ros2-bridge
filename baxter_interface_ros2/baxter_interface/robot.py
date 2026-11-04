"""Several robots in one program.

Each Baxter runs its own ROS 1 system, and both use the same topic names. They are kept
apart the way ROS 2 keeps any two robots apart: each robot's bridge puts the robot's name
in front of its topics and services (docs/two-robots.md).

    /one/robot/joint_states      the robot "one"
    /two/robot/joint_states      the robot "two"
    /robot/joint_states          a robot whose bridge gives it no name

A Robot object stands for one of them; every part of that robot is made through it:

    import baxter_interface

    one = baxter_interface.Robot('one')
    two = baxter_interface.Robot('two')

    one.enable()
    two.enable()
    with baxter_interface.at_once():          # both at the same time
        one.Limb('left').move_to_neutral()
        two.Limb('left').move_to_neutral()

The name is the ROBOT_NAME of the robot's settings file on the bridge machine.

A program for a single robot needs none of this: Limb('left') and the other classes used
directly mean the robot the program was started for. That is the robot without a name,
unless the program is started with a name: with BAXTER_ROBOT=one, or the ROS 2 argument
`--ros-args -r __ns:=/one`.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from baxter_core_msgs.msg import AssemblyState
from rclpy.node import Node

from . import camera as _camera
from . import info as _info
from ._core.node import get_node, robot_node
from .camera import Camera, CameraController
from .display import Display
from .gripper import Gripper
from .head import Head
from .head_lights import HeadLights
from .io import AnalogIO, DigitalIO, Navigator
from .limb import Limb
from .robot_enable import RobotEnable
from .robust_controller import RobustController
from .sensors import Accelerometer, RangeSensor, Sonar

if TYPE_CHECKING:
    from .mobility_base import MobilityBase

# Every bridge offers this topic for its robot (see info.py). It is how robots are found.
_SIGN_OF_A_ROBOT = '/baxter_bridge/robot_info'

# Seconds Robot('one') looks for the robot before it gives up. ROS 2 programs need a
# moment after their start to see what else is running.
LOOKUP_TIME: float = 5.0


def _robots_seen(node: Node) -> list[str]:
    """The names of the robots whose bridges this node sees right now ('' for no name)."""
    names = []
    for topic, _ in node.get_topic_names_and_types():
        if topic.endswith(_SIGN_OF_A_ROBOT) and node.count_publishers(topic) > 0:
            name = topic[:-len(_SIGN_OF_A_ROBOT)].lstrip('/')
            if '/' not in name:
                names.append(name)
    return sorted(names)


def known_robots(wait: float = 3.0) -> list[str]:
    """Return the names of the robots that can be reached right now.

    A robot can be reached while its bridge runs. Nothing is read from settings files:
    this looks at what is there.

    Parameters:
        wait: seconds to look. A program that has only just started sees nothing yet.

    Returns:
        list of names, sorted, e.g. ['one', 'two']; a robot without a name is ''
    """
    node = get_node()
    end = time.monotonic() + wait
    seen = _robots_seen(node)
    while time.monotonic() < end:
        time.sleep(0.1)
        seen = _robots_seen(node)
    return seen


def _describe(names: list[str]) -> str:
    """A list of robot names for a message."""
    if not names:
        return 'none'
    return ', '.join(f"'{name}'" if name else 'one without a name' for name in names)


class Robot:
    """One robot of several. Make its parts through it: robot.Limb('left').

    Available methods:
        Limb(), Gripper(), Head(), HeadLights(), Camera(), Display(), Navigator(),
        DigitalIO(), AnalogIO(), Sonar(), RangeSensor(), Accelerometer(), MobilityBase()
                    - the parts, with the same arguments as the classes of those names
        enable(), disable(), reset(), stop(), state()
                    - as RobotEnable's
        info(), software_version(), list_cameras()
    Attributes:
        name      - the robot's name ('' for a robot without a name)
        namespace - what is in front of its topics: '/one' for the robot 'one', '/' for ''
        node      - the ROS 2 node its parts use
    """

    def __init__(self, name: str | None = None) -> None:
        """
        Parameters:
            name: the robot's name, the ROBOT_NAME of its bridge, e.g. 'one'; '' for a
                  robot whose bridge gives it no name.
                  Left out: the robot this program was started for, the same one the
                  classes mean when used without a Robot.

        Raises:
            ValueError for a name that cannot be a robot name.
            ConnectionError if no robot with that name is found. The message lists the
            robots that are.
        """
        if name is None:
            self.node: Node = get_node()
        else:
            self.node = robot_node(name)
            self._look_for(name)
        self.namespace: str = self.node.get_namespace()
        self.name: str = self.namespace.lstrip('/')
        self._enable: RobotEnable | None = None

    def _look_for(self, name: str) -> None:
        """Wait until this robot's bridge is seen; ConnectionError if it does not appear."""
        end = time.monotonic() + LOOKUP_TIME
        while name not in _robots_seen(self.node):
            if time.monotonic() >= end:
                wanted = f"named '{name}'" if name else 'without a name'
                raise ConnectionError(
                    f'No robot {wanted} is found. A robot can be reached while its bridge '
                    'runs, and its name is the ROBOT_NAME in the settings file its bridge '
                    f'was started with. Robots found: {_describe(_robots_seen(self.node))}.')
            time.sleep(0.05)

    def __repr__(self) -> str:
        return f'Robot({self.name!r})'

    # --- The robot's parts ----------------------------------------------------------------

    def Limb(self, limb: str) -> Limb:
        """An arm of this robot: 'left' or 'right'."""
        return Limb(limb, node=self.node)

    def Gripper(self, gripper: str) -> Gripper:
        """A gripper of this robot: 'left' or 'right'."""
        return Gripper(gripper, node=self.node)

    def Head(self) -> Head:
        """This robot's head."""
        return Head(node=self.node)

    def HeadLights(self) -> HeadLights:
        """The halo and sonar lights on this robot's head."""
        return HeadLights(node=self.node)

    def Camera(self, name: str) -> Camera:
        """A camera of this robot: 'head', 'left_hand' or 'right_hand'."""
        return Camera(name, node=self.node)

    def CameraController(self, name: str) -> CameraController:
        """The settings of a camera of this robot (full name, e.g. 'head_camera')."""
        return CameraController(name, node=self.node)

    def Display(self) -> Display:
        """This robot's screen."""
        return Display(node=self.node)

    def Navigator(self, location: str) -> Navigator:
        """A navigator of this robot: 'left', 'right', 'torso_left' or 'torso_right'."""
        return Navigator(location, node=self.node)

    def DigitalIO(self, component_id: str) -> DigitalIO:
        """An on/off component of this robot, by its name (docs/io-reference.md)."""
        return DigitalIO(component_id, node=self.node)

    def AnalogIO(self, component_id: str) -> AnalogIO:
        """A number-valued component of this robot, by its name."""
        return AnalogIO(component_id, node=self.node)

    def Sonar(self) -> Sonar:
        """The sonar ring on this robot's head."""
        return Sonar(node=self.node)

    def RangeSensor(self, limb: str) -> RangeSensor:
        """The range sensor in a hand of this robot: 'left' or 'right'."""
        return RangeSensor(limb, node=self.node)

    def Accelerometer(self, limb: str) -> Accelerometer:
        """The accelerometer in a hand of this robot: 'left' or 'right'."""
        return Accelerometer(limb, node=self.node)

    def MobilityBase(self) -> 'MobilityBase':
        """The Mobility Base this robot stands on (its bridge has the robot's name)."""
        from .mobility_base import MobilityBase    # needs the base's messages; see __init__.py
        return MobilityBase(node=self.node)

    def RobustController(self, namespace: str, enable_msg: Any, disable_msg: Any, timeout: float = 60) -> RobustController:
        """One of this robot's maintenance routines (see robust_controller.py)."""
        return RobustController(namespace, enable_msg, disable_msg, timeout, node=self.node)

    # --- Enabling -------------------------------------------------------------------------

    def RobotEnable(self) -> RobotEnable:
        """This robot's enable interface (one object, made on first use)."""
        if self._enable is None:
            self._enable = RobotEnable(node=self.node)
        return self._enable

    def enable(self) -> None:
        """Enable all joints of this robot."""
        self.RobotEnable().enable()

    def disable(self) -> None:
        """Disable all joints of this robot."""
        self.RobotEnable().disable()

    def reset(self) -> bool:
        """Reset this robot after a stop or an error (see RobotEnable.reset())."""
        return self.RobotEnable().reset()

    def stop(self) -> None:
        """Stop this robot, like its e-stop button."""
        self.RobotEnable().stop()

    def state(self) -> AssemblyState:
        """Return this robot's state (see RobotEnable.state())."""
        return self.RobotEnable().state()

    # --- Information ----------------------------------------------------------------------

    def info(self, timeout: float = 5.0) -> dict[str, Any]:
        """Return this robot's ROS 1 parameters that the bridge passes on (see robot_info())."""
        return _info.robot_info(timeout, node=self.node)

    def software_version(self, timeout: float = 5.0) -> str | None:
        """Return this robot's software version, e.g. '1.2.0.57'."""
        return _info.software_version(timeout, node=self.node)

    def list_cameras(self) -> list[str]:
        """Return the names of the cameras this robot lists."""
        return _camera.list_cameras(self.node)
