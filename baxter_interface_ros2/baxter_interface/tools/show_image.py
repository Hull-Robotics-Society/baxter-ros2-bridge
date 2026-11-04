"""Show a picture on Baxter's screen.

    ros2 run baxter_interface show_image photo.png    # any image file; scaled to fit
    ros2 run baxter_interface show_image              # back to the standard SDK picture
    ros2 run baxter_interface show_image --black      # black screen

The picture stays on the screen until another one is shown.
"""

import argparse
import os
import sys

from rclpy.utilities import remove_ros_args

import baxter_interface
from baxter_interface import Display

_DEFAULT = os.path.join(os.path.dirname(baxter_interface.__file__), 'images', 'researchsdk.png')


def main():
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('file', nargs='?', default=_DEFAULT,
                        help='image file (PNG, JPEG, ...); default: the standard SDK picture')
    parser.add_argument('--black', action='store_true', help='show a black screen instead')
    args = parser.parse_args(remove_ros_args()[1:])

    try:
        display = Display()
        if args.black:
            display.clear()
            print('Screen set to black.')
        else:
            display.show(args.file)
            print(f'Shown: {args.file}')
    except (TimeoutError, ValueError, OSError) as e:
        sys.exit(str(e))


if __name__ == '__main__':
    main()
