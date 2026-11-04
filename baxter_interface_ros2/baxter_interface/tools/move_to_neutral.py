"""Move one or both arms to the neutral pose (center of the joint ranges).

    ros2 run baxter_interface move_to_neutral            # both arms
    ros2 run baxter_interface move_to_neutral -l left    # one arm

The robot must be enabled first (enable_robot -e). On the real robot: make sure the
arms have room to move, and have someone at the e-stop.
"""

import argparse
import sys

from rclpy.utilities import remove_ros_args

from baxter_interface import Limb


def main():
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('-l', '--limb', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('-t', '--timeout', type=float, default=15.0,
                        help='seconds to wait for each arm (default 15)')
    args = parser.parse_args(remove_ros_args()[1:])

    limbs = ['left', 'right'] if args.limb == 'both' else [args.limb]
    try:
        for name in limbs:
            print(f'Moving {name} arm to neutral ...')
            Limb(name).move_to_neutral(timeout=args.timeout)
        print('Done.')
    except (TimeoutError, RuntimeError) as e:
        sys.exit(str(e))


if __name__ == '__main__':
    main()
