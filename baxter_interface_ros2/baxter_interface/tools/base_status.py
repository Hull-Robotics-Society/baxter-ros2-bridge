#!/usr/bin/env python3
"""Show the state of the Dataspeed Mobility Base. Reads only; the base does not move.

    ros2 run baxter_interface base_status           # once
    ros2 run baxter_interface base_status -w        # keep watching, Ctrl+C to stop

Needs the bridge for the base (docs/mobility-base.md).
"""

import argparse
import sys
import time

from rclpy.utilities import remove_ros_args

import baxter_interface


def report(base) -> str:
    """One line about the base as it is now."""
    try:
        battery = base.battery(timeout=3.0)
        power = f'battery {battery.percent} % ({battery.voltage:.1f} V, {battery.state})'
    except TimeoutError:
        power = 'battery: no report yet'
    speed = base.speed()
    bumpers = ', '.join(f"{corner} ({' and '.join(sides)})"
                        for corner, sides in base.bumpers().items() if sides) or 'none pressed'
    return (f"mode '{base.mode()}': {base.mode_meaning()} | {power} | bumpers: {bumpers} | "
            f'speed: forward {speed.forward:+.2f}, left {speed.left:+.2f} m/s, '
            f'turn {speed.turn:+.2f} rad/s')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('-w', '--watch', action='store_true',
                        help='print the state once per second until Ctrl+C')
    args = parser.parse_args(remove_ros_args()[1:])

    try:
        base = baxter_interface.MobilityBase()
    except (TimeoutError, ConnectionError) as error:
        sys.exit(str(error))

    print(report(base))
    try:
        while args.watch:
            time.sleep(1.0)
            print(report(base))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
