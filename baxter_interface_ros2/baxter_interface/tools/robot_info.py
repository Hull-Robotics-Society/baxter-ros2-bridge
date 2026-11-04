"""Show information about the robot that the bridge passes on from its ROS 1 parameters.

    ros2 run baxter_interface robot_info

Which parameters are shown is configured under `robot_info_params` in
bridge_config/topics.yaml (see baxter_interface/info.py).
"""

import argparse
import json
import sys

from rclpy.utilities import remove_ros_args

import baxter_interface

# The robot software version whose messages this library and the bridge are built for.
BUILT_FOR = '1.2.0'


def main():
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--timeout', type=float, default=5.0,
                        help='seconds to wait for the information (default 5)')
    args = parser.parse_args(remove_ros_args()[1:])

    try:
        params = baxter_interface.robot_info(args.timeout)
        version = baxter_interface.software_version(args.timeout)
    except TimeoutError as e:
        sys.exit(str(e))

    print(f"Software version: {version if version is not None else 'not reported by the robot'}")
    if version is not None and not version.startswith(BUILT_FOR):
        print(f'WARNING: the robot runs {version}, but the bridge and this library are built '
              f'for robot software {BUILT_FOR}. Messages that differ between the two will '
              'not arrive (see msgs/README.md).')
    for name, value in params.items():
        if value is None:
            print(f'{name}: (not set on the robot)')
        elif isinstance(value, (dict, list)):
            print(f'{name}:')
            print('  ' + json.dumps(value, indent=2, sort_keys=True).replace('\n', '\n  '))
        else:
            print(f'{name}: {str(value).strip()}')


if __name__ == '__main__':
    main()
