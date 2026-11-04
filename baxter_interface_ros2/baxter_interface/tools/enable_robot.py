"""Show the robot state, or enable, disable, reset or stop the robot.

ROS 2 version of baxter_tools' enable_robot.py:
    ros2 run baxter_interface enable_robot -s      # state
    ros2 run baxter_interface enable_robot -e      # enable
    ros2 run baxter_interface enable_robot -d      # disable
    ros2 run baxter_interface enable_robot -r      # reset
    ros2 run baxter_interface enable_robot -S      # stop (like the e-stop)
"""

import argparse
import sys

from rclpy.utilities import remove_ros_args

from baxter_interface import RobotEnable


def main():
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-s', '--state', action='store_true', help='print the robot state')
    group.add_argument('-e', '--enable', action='store_true', help='enable the robot')
    group.add_argument('-d', '--disable', action='store_true', help='disable the robot')
    group.add_argument('-r', '--reset', action='store_true', help='reset the robot')
    group.add_argument('-S', '--stop', action='store_true', help='stop the robot')
    args = parser.parse_args(remove_ros_args()[1:])

    try:
        robot = RobotEnable()
        if args.enable:
            robot.enable()
        elif args.disable:
            robot.disable()
        elif args.reset:
            robot.reset()
        elif args.stop:
            robot.stop()
        print(robot.state())
    except (TimeoutError, RuntimeError) as e:
        sys.exit(str(e))


if __name__ == '__main__':
    main()
