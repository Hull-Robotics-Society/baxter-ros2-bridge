"""Run the Baxter smoke tests: a self-test of the robot's main functions.

ROS 2 version of baxter_tools' smoke_test.py (v0.7.0, Copyright (c) 2013,
Rethink Robotics, BSD license; see LICENSE):
    ros2 run baxter_interface smoke_test                # all tests
    ros2 run baxter_interface smoke_test -t MoveArms    # one test
    ros2 run baxter_interface smoke_test -p             # continue after a failed test

Tests:
    Enable    - Verify ability to enable, check state and disable Baxter
    Messages  - Verify messages being published and ability to subscribe
    Services  - Verify services available and ability to make calls as client
    Head      - Move the head pan and nod, display image on the screen
    MoveArms  - Move both arms through a large part of their joint range
    Grippers  - Calibrate and move the grippers using position and velocity control
    BlinkLEDs - Blink the navigator LEDs
    Cameras   - Verify camera publishing and visualization

The robot MOVES during these tests (both arms, the head, the grippers). Clear the area
and have someone at the e-stop.

Differences from the ROS 1 tool: the robot's software version is shown and used in the
result file's name when the bridge reports it (see baxter_interface/info.py), but the
tests run either way; the ROS 1 tool refused to run on an unknown version. -o chooses a
different result file.

"""

import argparse
import sys
import time
import traceback

from rclpy.utilities import remove_ros_args

import baxter_interface
from baxter_interface.tools import _smoke_test_cases

TESTS = ['Enable', 'Messages', 'Services', 'Head', 'MoveArms', 'Grippers', 'BlinkLEDs', 'Cameras']


def run_test(name, filename, proceed):
    """Run one test and record its result.

    Parameters:
        name:     test name, one of TESTS
        filename: file to append the result to
        proceed:  continue with the next test after a failure

    Returns:
        bool: True if the remaining tests should run
    """
    try:
        test = getattr(_smoke_test_cases, name)(name)
    except Exception:
        print('Exiting: failed during initialization.')
        traceback.print_exc()
        return False

    test.start_test()
    test.finish_test(filename)

    if 'KeyboardInterrupt' in test.result[1] or (not proceed and not test.result[0]):
        print(f'Exiting: Failed Test {name}')
        return False
    return True


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description=(__doc__ or '').split('\n\nThe robot MOVES')[0],
    )
    parser.add_argument('-p', '--proceed', action='store_true',
                        help='continue testing after a failed test until all tests complete')
    parser.add_argument('-t', '--test', choices=TESTS, help='run only this test')
    parser.add_argument('-o', '--output', help='result file (default: named after the date)')
    args = parser.parse_args(remove_ros_args()[1:])

    try:
        version = baxter_interface.software_version(timeout=3.0)
    except TimeoutError:
        version = None  # older bridge image, or the topic is not bridged
    print(f"Robot software version: {version or 'not reported'}")

    now = time.localtime()
    filename = args.output or (
        f"baxter-{now.tm_mon}.{now.tm_mday}.{now.tm_year}-rsdk-{version or 'ros2'}.smoketest")

    print('The robot will MOVE: both arms, the head and the grippers.')
    try:
        input('Press <Enter> to Begin Smoke Test\n')
    except (EOFError, KeyboardInterrupt):
        print('\nExiting.')
        return 1

    tests = [args.test] if args.test else TESTS
    if not args.test:
        print('Performing All Tests')

    try:
        for name in tests:
            if not run_test(name, filename, args.proceed):
                return 1
    except KeyboardInterrupt:
        print('\nInterrupted. Disabling the robot...')
        try:
            baxter_interface.RobotEnable().disable()
        except Exception:
            print('Could not disable the robot: use the e-stop or enable_robot -d.')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
