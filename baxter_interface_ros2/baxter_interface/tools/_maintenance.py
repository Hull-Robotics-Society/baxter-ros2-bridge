"""Shared code for the maintenance tools calibrate_arm and tare.

Both run a routine on the robot that moves the arm by itself for several minutes and
changes the robot's stored calibration. They need the gripper removed and the robot
enabled, and they disable the robot again afterwards.
"""

import argparse
import sys

from rclpy.utilities import remove_ros_args

import baxter_interface


def gripper_removed(side):
    """Check that no gripper is attached to the arm (required for calibration and tare).

    Parameters:
        side: 'left' or 'right'

    Returns:
        bool: True if no gripper (type 'custom') is reported for that arm
    """
    return baxter_interface.Gripper(side).type() == 'custom'


def run_maintenance(name, description, make_controller):
    """Command-line entry point shared by calibrate_arm and tare.

    Parameters:
        name:            what to call the routine in messages, e.g. 'Calibrate arm'
        description:     first line of the tool's --help
        make_controller: function(limb) returning the RobustController to run

    Returns:
        int: process exit code, 0 on success
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('-l', '--limb', required=True, choices=['left', 'right'],
                        help='arm to run the routine on')
    parser.add_argument('-y', '--yes', action='store_true',
                        help="don't ask for confirmation")
    args = parser.parse_args(remove_ros_args()[1:])
    limb = args.limb

    print(f'Preparing: {name} on the {limb} arm.')
    print('\nIMPORTANT: Remove grippers and other attachments from the arm first.')
    print('The arm will move by itself for several minutes, and the robot\'s stored')
    print('calibration will change. Clear the area and have someone at the e-stop.\n')
    if not args.yes and input('Type "yes" to continue: ').strip().lower() != 'yes':
        print('Cancelled.')
        return 1

    try:
        if not gripper_removed(limb):
            print(f'Cannot run with a gripper attached. Remove the {limb} gripper first!',
                  file=sys.stderr)
            return 1

        robot = baxter_interface.RobotEnable()
        robot.enable()
        controller = make_controller(limb)
    except (TimeoutError, RuntimeError) as e:
        print(e, file=sys.stderr)
        return 1

    print(f'Running: {name} on the {limb} arm ...')
    error = None
    try:
        controller.run()
    except OSError as e:
        error = e.strerror or str(e)
    finally:
        try:
            robot.disable()
        except Exception:  # the result of the routine matters more than a failed disable
            pass

    if error is None:
        print(f'{name} finished')
        return 0
    print(f'{name} failed: {error}', file=sys.stderr)
    return 1
