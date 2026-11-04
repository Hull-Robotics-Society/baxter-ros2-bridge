"""List, open and close Baxter's cameras.

ROS 2 version of baxter_tools' camera_control.py (v0.7.0, Copyright (c) 2013,
Rethink Robotics, BSD license; see LICENSE):
    ros2 run baxter_interface camera_control -l                         # list cameras
    ros2 run baxter_interface camera_control -o left_hand_camera        # open (1280x800)
    ros2 run baxter_interface camera_control -o head_camera -r 640x400  # open, lower resolution
    ros2 run baxter_interface camera_control -c left_hand_camera        # close
    ros2 run baxter_interface camera_control -e                         # re-enumerate cameras

Difference from the ROS 1 tool: -l finds out which cameras are open by listening for
their camera_info messages for a few seconds (the ROS 1 tool asked the ROS 1 master, which
is not possible through the bridge).

Baxter powers only two of its three cameras at a time.
-l shows which have power. Closing one (-c) gives its power to the third, which can then
be opened (-o).
"""

import argparse
import sys

from rclpy.utilities import remove_ros_args

from baxter_interface.camera import (
    Camera, CameraController, camera_status, cameras_in_the_way, list_cameras, reset_cameras,
)

_STATUS_LABELS = {
    True: '  -  (open)',
    False: '',
}


def _parse_resolution(text):
    parts = text.lower().split('x')
    if len(parts) != 2 or not all(p.isdigit() for p in parts):
        return None
    res = (int(parts[0]), int(parts[1]))
    return res if res in CameraController.MODES else None


def main():
    modes = ', '.join(f'{w}x{h}' for w, h in CameraController.MODES)
    parser = argparse.ArgumentParser(
        description=(__doc__ or '').splitlines()[0],
        epilog=f'Valid resolutions: {modes}',
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-o', '--open', metavar='CAMERA', help='open the given camera')
    group.add_argument('-c', '--close', metavar='CAMERA', help='close the given camera')
    group.add_argument('-l', '--list', action='store_true', help='list available cameras')
    group.add_argument('-e', '--enumerate', action='store_true',
                       help='clear and re-enumerate connected cameras')
    parser.add_argument('-r', '--resolution', metavar='XxY', default='1280x800',
                        help='resolution for --open (default: 1280x800)')
    args = parser.parse_args(remove_ros_args()[1:])

    res = _parse_resolution(args.resolution)
    if args.open and res is None:
        parser.error(f'Invalid resolution {args.resolution!r}. Valid resolutions: {modes}')

    try:
        if args.enumerate:
            reset_cameras()

        if args.list or args.enumerate:
            cameras = list_cameras()
            if not cameras:
                print('No cameras found')
            for name, is_open in camera_status(cameras).items():
                print(f'{name}{_STATUS_LABELS[is_open]}')
            for name in Camera.NAMES:
                if name not in cameras:
                    print(f'{name}  -  (not offered: close one of the others to use it)')

        elif args.open:
            assert res is not None  # checked right after reading the arguments
            cam = CameraController(args.open)
            # Never a third camera: the robot's camera program can stop answering then.
            in_the_way = cameras_in_the_way(args.open)
            if in_the_way:
                sys.exit(f'{args.open} cannot be opened: Baxter runs two of its three '
                         f'cameras at a time, and two are in use ({", ".join(in_the_way)}). '
                         f'Close one first: camera_control -c {in_the_way[0]}')
            # No close first: that would give this camera's place away.
            cam.resolution = res
            cam.open()
            print(f'Opened {args.open} at {res[0]}x{res[1]}')

        elif args.close:
            CameraController(args.close).close()
            print(f'Closed {args.close}')

    except (TimeoutError, ValueError, OSError) as e:
        sys.exit(str(e))


if __name__ == '__main__':
    main()
