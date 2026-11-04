"""Python interface to the Baxter robot for ROS 2.

Same classes and methods as the original Baxter SDK (baxter_interface v0.7.0), so
existing Baxter code ports with few changes, plus some simpler ones (moving a hand to a
place, sensors, pictures as OpenCV images). Guides: docs/ in the repository; examples:
examples/.

    from baxter_interface import Limb, RobotEnable

    RobotEnable().enable()
    left = Limb('left')
    print(left.joint_angles())
    left.move_to_neutral()

No ROS 2 setup is needed in the program: the classes share one background node (see
_core/node.py). Differences from the ROS 1 SDK:
  - no rospy.init_node() call
  - timeouts raise TimeoutError (was OSError)
  - classes accept an optional node=... to use an existing ROS 2 node

Layout:
    robot_enable.py, limb.py, gripper.py, head.py   the main classes
    camera.py, display.py  pictures from the cameras and on the screen (OpenCV images)
    sensors.py             Sonar, RangeSensor, Accelerometer
    head_lights.py         HeadLights: the halo and the sonar lights on the head
    robust_controller.py   runs the robot's maintenance routines (calibrate, tare)
    info.py                robot_info(), software_version(): the robot's ROS 1 parameters
    robot.py               Robot: one robot of several (each under its own name)
    io/        buttons, lights and simple sensors (DigitalIO, AnalogIO, Navigator)
    tools/     command-line tools (ros2 run baxter_interface ...)
    _core/     internal plumbing; not needed to use the library
Two blocks help with several parts: setup() connects them together at the start of a
program, at_once() runs commands for different parts at the same time.
Whatever the layout, import everything from the top level, as in the example above.
"""

from typing import TYPE_CHECKING

from . import settings
from ._core.at_once import at_once
from ._core.connecting import setup
from ._core.node import shutdown
from ._core.signals import Signal
from .camera import Camera, CameraController
from .display import Display
from .gripper import Gripper
from .head import Head
from .head_lights import HeadLights
from .info import robot_info, software_version
from .io import AnalogIO, DigitalIO, Navigator
from .limb import Limb
from .robot import Robot, known_robots
from .robot_enable import RobotEnable
from .robust_controller import RobustController
from .sensors import Accelerometer, RangeSensor, Sonar

if TYPE_CHECKING:
    from .mobility_base import MobilityBase


def __getattr__(name: str) -> object:
    # MobilityBase is only loaded when a program asks for it: it needs the base's message
    # package (mobility_base_core_msgs), which images built before 13 Oct 2026 do not
    # have. Loaded with the rest, it would stop every program on such an image.
    if name == 'MobilityBase':
        from .mobility_base import MobilityBase
        return MobilityBase
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')

__all__ = [
    'at_once',
    'Accelerometer',
    'AnalogIO',
    'Camera',
    'CameraController',
    'DigitalIO',
    'Display',
    'Gripper',
    'Head',
    'HeadLights',
    'Limb',
    'MobilityBase',
    'Navigator',
    'RangeSensor',
    'Robot',
    'RobotEnable',
    'RobustController',
    'Signal',
    'Sonar',
    'known_robots',
    'setup',
    'settings',
    'robot_info',
    'shutdown',
    'software_version',
]
