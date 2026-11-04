"""Baxter's cameras: get pictures from them, and open, close and configure them.

Two classes:

    Camera            the easy way to get pictures (new in the ROS 2 library):

                          from baxter_interface import Camera

                          camera = Camera('left_hand')
                          camera.open()
                          image = camera.frame()          # an OpenCV image

    CameraController  the original SDK class: open, close and all settings (exposure,
                      gain, ...), but no pictures. Camera uses one internally
                      (camera.controller).

CameraController is ported from baxter_interface v0.7.0 (Copyright (c) 2013, Rethink
Robotics, BSD license; see LICENSE). Same properties and behaviour; ROS 1 service proxies
replaced by ROS 2 service clients. Errors from the robot raise OSError as before; a robot
that doesn't answer raises TimeoutError.

Baxter runs only TWO of its three cameras at a time. After start-up these are the two
hand cameras. To use the third one, another has to be closed first:

    Camera('left_hand').close()
    Camera('head').open()

or in one step:   Camera('head').open(replace='left_hand')

Camera.open() checks this itself and refuses to open a third camera. It has to: the robot
does not always refuse. Asked to open the head camera while both hand cameras ran, the
robot's camera program stopped answering and delivered no more pictures from any camera
(7 Oct 2026).

list_cameras() returns the cameras the robot lists. That is not a reliable sign of which
ones can be opened: after a start the robot lists all three, later often only two.
cameras_in_the_way() answers that question.

With BRIDGE_MODE=fixed_list, needs in bridge_config/topics.yaml:
    services_2_to_1:  /cameras/list, /cameras/open, /cameras/close, /cameras/reset
    topics:           /cameras/<name>/image        for Camera (the pictures)
                      /cameras/<name>/camera_info  for camera_status()

These methods wait for the robot's answer. That is fine from your main program and from
functions connected to signals, which run in their own threads. Only a raw rclpy callback
on the shared node must not call them: it runs in the receiving thread, which is the one
that delivers the answer (see _core/node.py).
"""

from __future__ import annotations

import errno
import os
import threading
import time
from typing import Any, Sequence

from rclpy.client import Client
from rclpy.node import Node
from ._core.node import ros_is_running
from baxter_core_msgs.msg import CameraControl, CameraSettings
from baxter_core_msgs.srv import CloseCamera, ListCameras, OpenCamera
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Empty

from ._core import at_once, dataflow, images
from ._core.locks import locked, part_lock
from ._core.node import get_node
from ._core.services import call_service
from ._core.signals import Signal


def list_cameras(node: Node | None = None) -> list[str]:
    """Return the names of the cameras the robot lists.

    Sometimes all three, sometimes only two of them. Whether a camera can be opened
    right now is a different question: see cameras_in_the_way().

    Parameters:
        node: optional ROS 2 node to use (default: the shared background node)

    Returns:
        list of str, e.g. ['left_hand_camera', 'right_hand_camera']
    """
    client = get_node(node).create_client(ListCameras, 'cameras/list')
    return list(call_service(client, ListCameras.Request()).cameras)


def cameras_in_the_way(name: str, node: Node | None = None) -> list[str]:
    """Return the cameras that keep the camera `name` from being opened right now.

    Baxter runs two of its three cameras at a time. Two things can stand in the way:
    the robot does not list the camera (then the listed ones have to give way), or the
    two other cameras are both open. Closing one of the returned cameras clears the way.

    Takes a few seconds when the robot lists all three cameras: which ones are open can
    only be found out by listening for their messages (see camera_status()).

    Parameters:
        name: full camera name, e.g. 'head_camera'
        node: optional ROS 2 node to use (default: the shared background node)

    Returns:
        list of camera names; empty if the camera can be opened
    """
    listed = list_cameras(node)
    others = [other for other in listed if other != name]
    if name not in listed:
        return others
    if len(others) < 2:
        return []
    status = camera_status(others, node=node)
    running = [other for other in others if status[other]]
    return running if len(running) >= 2 else []


def reset_cameras(node: Node | None = None) -> None:
    """Clear and re-enumerate the cameras connected to the robot.

    Parameters:
        node: optional ROS 2 node to use (default: the shared background node)
    """
    client = get_node(node).create_client(Empty, 'cameras/reset')
    call_service(client, Empty.Request())


def camera_status(
    cameras: Sequence[str] | None = None,
    listen: float = 1.5,
    node: Node | None = None
) -> dict[str, bool]:
    """Find out which cameras are open, by listening for their messages.

    An open camera publishes a small camera_info message with every frame, so messages
    arriving on /cameras/<name>/camera_info mean the camera is open. (The ROS 1 SDK asked
    the ROS 1 master instead, which is not possible through the bridge.)

    Takes a few seconds: the bridge needs a moment to connect each topic, and a closed
    camera can only be told from silence.

    Parameters:
        cameras: camera names to check (default: all cameras from list_cameras())
        listen:  seconds to listen for messages (default 1.5)
        node:    optional ROS 2 node to use (default: the shared background node)

    Returns:
        dict {camera name: True (open) or False (closed; with the fixed-list bridge also
        if the camera's camera_info topic is not in bridge_config/topics.yaml)}
    """
    node = get_node(node)
    if cameras is None:
        cameras = list_cameras(node)

    counts = {name: 0 for name in cameras}

    def counter(name):
        def on_message(_msg):
            counts[name] += 1
        return on_message

    topics = {name: f'cameras/{name}/camera_info' for name in cameras}
    subscriptions = {
        name: node.create_subscription(
            CameraInfo,
            topic,
            counter(name),
            10
        )
        for name, topic in topics.items()
    }

    try:
        # Give the bridge time to connect the topics (it notices a new listener within a
        # second or two), then listen. For a closed camera nothing ever connects.
        end = time.monotonic() + 3.0
        while time.monotonic() < end and not all(
                node.count_publishers(topic) for topic in topics.values()):
            time.sleep(0.05)
        time.sleep(listen)
    finally:
        for subscription in subscriptions.values():
            node.destroy_subscription(subscription)

    return {name: counts[name] > 0 for name in cameras}


class CameraController:
    """Interface class for controlling camera settings on the Baxter robot.

    Available methods and properties:
        open(), close()
        resolution, fps, exposure, gain, white_balance_red/green/blue,
        window, flip, mirror, half_resolution
    Changing a setting while the camera is open reopens it with the new setting.
    """

    # Valid resolutions
    MODES = [
        (1280, 800),
        (960, 600),
        (640, 400),
        (480, 300),
        (384, 240),
        (320, 200),
    ]

    # Used to represent when the camera is using automatic controls.
    # Valid for exposure, gain and white balance.
    CONTROL_AUTO = -1

    def __init__(self, name: str, node: Node | None = None) -> None:
        """
        Parameters:
            name: camera identifier: 'left_hand_camera', 'right_hand_camera' or
                  'head_camera' (the robot's /cameras/list service lists them)
            node: optional ROS 2 node to use (default: the shared background node)
        """
        self._id = name
        self._node = get_node(node)
        # Held while opening, closing or reopening: one such step at a time per camera.
        self._part_lock = part_lock(f'camera/{name}', self._node)

        self._list_svc = self._node.create_client(ListCameras, 'cameras/list')
        self._open_svc = self._node.create_client(OpenCamera, 'cameras/open')
        self._close_svc = self._node.create_client(CloseCamera, 'cameras/close')

        cameras = self._call(self._list_svc, ListCameras.Request()).cameras
        if self._id not in cameras:
            if self._id in Camera.NAMES:
                raise ValueError(
                    f'{self._id} has no power right now. Baxter powers two of its three '
                    f'cameras at a time (now: {", ".join(cameras)}). Close one of those '
                    f'first; {self._id} then gets its power.')
            raise ValueError(f"Invalid camera name '{self._id}' (available: {list(cameras)})")

        self._settings = CameraSettings()
        self._settings.width = 320
        self._settings.height = 200
        self._settings.fps = 20.0
        self._open = False

    def _call(self, client: Client, request: Any) -> Any:
        return call_service(client, request)

    @locked
    def _reload(self) -> None:
        if self._open:
            # No close first (as the SDK v0.7.0 did): closing would hand this camera's
            # power to another one. Opening again is enough to apply new settings.
            self.open()

    def _get_value(self, control: int, default: int) -> int:
        lookup = [c.value for c in self._settings.controls if c.id == control]
        return lookup[0] if lookup else default

    def _set_control_value(self, control: int, value: int) -> None:
        lookup = [c for c in self._settings.controls if c.id == control]
        if lookup:
            lookup[0].value = int(value)
        else:
            # A new list is assigned (not appended to): a message's list field does
            # not promise to be a plain list.
            self._settings.controls = list(self._settings.controls) + [CameraControl(
                id=control,
                value=int(value)
            )]

    def _check_range(self, value: int, low: int, high: int, what: str) -> int:
        value = int(value)
        if (value < low or value > high) and value != self.CONTROL_AUTO:
            raise ValueError(f'Invalid {what} value')
        return value

    # --- Settings --------------------------------------------------------------------------

    @property
    def resolution(self) -> tuple[int, int]:
        """Camera resolution as (width, height); valid values are listed in MODES."""
        return (self._settings.width, self._settings.height)

    @resolution.setter
    def resolution(self, res: Sequence[int]) -> None:
        if len(res) != 2:
            raise ValueError('Invalid resolution specified')
        width, height = int(res[0]), int(res[1])
        if (width, height) not in self.MODES:
            raise ValueError(f'Invalid camera mode {width}x{height}')
        self._settings.width, self._settings.height = width, height
        self._reload()

    @property
    def fps(self) -> float:
        """Camera frames per second."""
        return self._settings.fps

    @fps.setter
    def fps(self, fps: float) -> None:
        self._settings.fps = float(fps)
        self._reload()

    @property
    def exposure(self) -> int:
        """Camera exposure, 0-100, or CONTROL_AUTO for automatic exposure."""
        return self._get_value(CameraControl.CAMERA_CONTROL_EXPOSURE, self.CONTROL_AUTO)

    @exposure.setter
    def exposure(self, exposure: int) -> None:
        exposure = self._check_range(exposure, 0, 100, 'exposure')
        self._set_control_value(CameraControl.CAMERA_CONTROL_EXPOSURE, exposure)
        self._reload()

    @property
    def gain(self) -> int:
        """Camera gain, 0-79, or CONTROL_AUTO for automatic gain."""
        return self._get_value(CameraControl.CAMERA_CONTROL_GAIN, self.CONTROL_AUTO)

    @gain.setter
    def gain(self, gain: int) -> None:
        gain = self._check_range(gain, 0, 79, 'gain')
        self._set_control_value(CameraControl.CAMERA_CONTROL_GAIN, gain)
        self._reload()

    @property
    def white_balance_red(self) -> int:
        """White balance red, 0-4095, or CONTROL_AUTO."""
        return self._get_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R, self.CONTROL_AUTO)

    @white_balance_red.setter
    def white_balance_red(self, value: int) -> None:
        value = self._check_range(value, 0, 4095, 'white balance')
        self._set_control_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R, value)
        self._reload()

    @property
    def white_balance_green(self) -> int:
        """White balance green, 0-4095, or CONTROL_AUTO."""
        return self._get_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_G, self.CONTROL_AUTO)

    @white_balance_green.setter
    def white_balance_green(self, value: int) -> None:
        value = self._check_range(value, 0, 4095, 'white balance')
        self._set_control_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_G, value)
        self._reload()

    @property
    def white_balance_blue(self) -> int:
        """White balance blue, 0-4095, or CONTROL_AUTO."""
        return self._get_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_B, self.CONTROL_AUTO)

    @white_balance_blue.setter
    def white_balance_blue(self, value: int) -> None:
        value = self._check_range(value, 0, 4095, 'white balance')
        self._set_control_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_B, value)
        self._reload()

    @property
    def window(self) -> tuple[int, int]:
        """Camera windowing as (x, y)."""
        x = self._get_value(CameraControl.CAMERA_CONTROL_WINDOW_X, self.CONTROL_AUTO)
        if x == self.CONTROL_AUTO:
            if self.half_resolution:
                width, height = self.resolution
                return (width // 2, height // 2)
            return self.resolution
        return (x, self._get_value(CameraControl.CAMERA_CONTROL_WINDOW_Y, self.CONTROL_AUTO))

    @window.setter
    def window(self, win: Sequence[int]) -> None:
        """Set the camera window. The maximum depends on the resolution and half_resolution."""
        x, y = tuple(win)
        cur_x, cur_y = self.resolution
        limit_x = 1280 - cur_x
        limit_y = 800 - cur_y
        if self.half_resolution:
            limit_x //= 2
            limit_y //= 2
        if x < 0 or x > limit_x:
            raise ValueError(f'Max X window is {limit_x}')
        if y < 0 or y > limit_y:
            raise ValueError(f'Max Y window is {limit_y}')
        self._set_control_value(CameraControl.CAMERA_CONTROL_WINDOW_X, x)
        self._set_control_value(CameraControl.CAMERA_CONTROL_WINDOW_Y, y)
        self._reload()

    @property
    def flip(self) -> bool:
        """True if the image is flipped."""
        return bool(self._get_value(CameraControl.CAMERA_CONTROL_FLIP, False))

    @flip.setter
    def flip(self, value: bool) -> None:
        self._set_control_value(CameraControl.CAMERA_CONTROL_FLIP, int(value != 0))
        self._reload()

    @property
    def mirror(self) -> bool:
        """True if the image is mirrored."""
        return bool(self._get_value(CameraControl.CAMERA_CONTROL_MIRROR, False))

    @mirror.setter
    def mirror(self, value: bool) -> None:
        self._set_control_value(CameraControl.CAMERA_CONTROL_MIRROR, int(value != 0))
        self._reload()

    @property
    def half_resolution(self) -> bool:
        """True if binning (half resolution) is enabled."""
        return bool(self._get_value(CameraControl.CAMERA_CONTROL_RESOLUTION_HALF, False))

    @half_resolution.setter
    def half_resolution(self, value: bool) -> None:
        self._set_control_value(CameraControl.CAMERA_CONTROL_RESOLUTION_HALF, int(value != 0))
        self._reload()

    # --- Open / close ----------------------------------------------------------------------

    @locked
    def open(self) -> None:
        """Open the camera with the current settings."""
        if self._id == 'head_camera':
            self._set_control_value(CameraControl.CAMERA_CONTROL_FLIP, True)
            self._set_control_value(CameraControl.CAMERA_CONTROL_MIRROR, True)
        request = OpenCamera.Request(
            name=self._id,
            settings=self._settings
        )
        ret = self._call(self._open_svc, request)
        for _ in range(3):
            if ret.err != errno.EAGAIN:
                break
            time.sleep(1.0)  # "try again": the camera is still starting up
            ret = self._call(self._open_svc, request)
        if ret.err != 0:
            raise OSError(ret.err, f'Failed to open camera ({os.strerror(ret.err)})')
        self._open = True

    @locked
    def close(self) -> None:
        """Close the camera (no error if it is already closed).

        This also takes the camera's power away and gives it to the camera that had none.
        """
        ret = self._call(self._close_svc, CloseCamera.Request(name=self._id))
        if ret.err != 0 and ret.err != errno.EINVAL:
            raise OSError(ret.err, 'Failed to close camera')
        self._open = False


class Camera:
    """One of Baxter's cameras, as a source of pictures.

    Available methods:
        open(resolution, fps) - switch the camera on
        close()               - switch it off
        frame()               - the newest picture, as an OpenCV image
        has_power()           - True if the camera can be opened right now
    Signals (see _core/signals.py):
        frame_received - called with every new picture
    Attributes:
        name       - the camera's full name, e.g. 'left_hand_camera'
        controller - its CameraController, for exposure, gain and the other settings

    Pictures are OpenCV images: NumPy arrays of shape (height, width, 3), blue-green-red.

        camera = Camera('left_hand')
        camera.open()
        cv2.imwrite('photo.png', camera.frame())

    To handle every picture as it arrives, connect a function:

        def on_frame(image):
            display.show(image)

        camera.frame_received.connect(on_frame)

    The function runs in its own thread. If it is slower than the camera, it gets the
    newest picture each time and the ones in between are skipped.

    Baxter powers only two of its three cameras at a time, normally the two hand cameras.
    To use the third one, give it the power of another:

        Camera('head').open(replace='left_hand')
    """

    NAMES = ('head_camera', 'left_hand_camera', 'right_hand_camera')

    # A stored picture older than this is not handed out by frame() any more: the camera
    # has probably been closed since.
    _MAX_AGE = 1.0

    def __init__(self, name: str, node: Node | None = None) -> None:
        """
        Parameters:
            name: 'head', 'left_hand' or 'right_hand' (the full names 'head_camera',
                  'left_hand_camera' and 'right_hand_camera' work as well)
            node: optional ROS 2 node to use (default: the shared background node)
        """
        self.name: str = self._full_name(name)
        self._node_argument = node
        self._node = get_node(node)
        self._controller: CameraController | None = None
        # Opening, closing and handing power over: one such step at a time per camera.
        self._part_lock = part_lock(f'camera/{self.name}', self._node)

        # Shared between the receiving thread (writes) and frame() (reads).
        self._newest = threading.Condition()
        self._msg: Image | None = None
        self._received = 0    # how many pictures have arrived
        self._handed_out = 0  # value of _received when frame() last returned one
        self._arrived = 0.0

        # keep='latest': a slow function skips pictures instead of falling behind.
        # prepare: the message is turned into an OpenCV image in the function's own
        # thread, not in the receiving thread.
        self.frame_received: Signal[images.Picture] = Signal(
            keep='latest',
            prepare=lambda msg: (images.from_image_msg(msg),)
        )

        self._topic = f'cameras/{self.name}/image'
        self._subscription = self._node.create_subscription(
            Image,
            self._topic,
            self._on_image,
            1  # only the newest picture is of interest
        )

    @classmethod
    def _full_name(cls, name: str) -> str:
        full = name if name.endswith('_camera') else f'{name}_camera'
        if full not in cls.NAMES:
            short = ', '.join(repr(n[:-len('_camera')]) for n in cls.NAMES)
            raise ValueError(f"Invalid camera name '{name}'. Baxter's cameras: {short}")
        return full

    @property
    def controller(self) -> CameraController:
        """The camera's CameraController (exposure, gain, ...). Needs the camera to have power."""
        if self._controller is None:
            self._controller = CameraController(self.name, node=self._node_argument)
        return self._controller

    def has_power(self) -> bool:
        """Return True if the camera can be opened right now.

        Baxter runs two of its three cameras at a time; see open(). False if the robot
        does not list this camera, or if the two other cameras are both open. May take a
        few seconds (see cameras_in_the_way()).
        """
        return not cameras_in_the_way(self.name, self._node_argument)

    def _on_image(self, msg: Image) -> None:
        with self._newest:
            self._msg = msg
            self._received += 1
            self._arrived = time.monotonic()
            self._newest.notify_all()
        self.frame_received(msg)

    @at_once.command
    @locked
    def open(
        self,
        resolution: tuple[int, int] = (640, 400),
        fps: float | None = None,
        replace: str | None = None
    ) -> None:
        """Switch the camera on. If it is already on, it restarts with the new settings.

        Baxter runs only two of its three cameras at a time. If two others are running
        (or the robot does not offer this one), another camera has to be closed: either
        close it yourself first, or name it with `replace`. Checking this takes a few
        seconds.

        Parameters:
            resolution: (width, height); one of CameraController.MODES: (1280, 800),
                        (960, 600), (640, 400), (480, 300), (384, 240), (320, 200).
                        Larger pictures arrive less smoothly; start with the default.
            fps:        pictures per second (default: leave as it is)
            replace:    camera to close if this one cannot be opened otherwise, e.g.
                        'left_hand'. Ignored if this camera can be opened as things are.

        Raises:
            ValueError for a resolution the camera doesn't have; RuntimeError if another
            camera has to be closed and `replace` is not given; OSError if the robot
            refuses.
        """
        resolution = (resolution[0], resolution[1])  # also accepts a list
        if resolution not in CameraController.MODES:
            modes = ', '.join(f'{w}x{h}' for w, h in CameraController.MODES)
            raise ValueError(f'Invalid camera resolution {resolution}. Valid: {modes}')

        self._get_power(replace)

        controller = self.controller
        # Set directly: the property setters would re-open the camera once per setting.
        controller._settings.width, controller._settings.height = resolution
        if fps is not None:
            controller._settings.fps = float(fps)
        try:
            controller.open()
        except OSError as error:
            raise OSError(error.errno, f'Failed to open {self.name}: {error.strerror}') from error

    def _get_power(self, replace: str | None) -> None:
        """Make sure this camera can be opened, closing `replace` first if necessary.

        Never lets a third camera be opened: the robot's camera program can stop
        answering for good when asked to (see the top of this file).
        """
        in_the_way = cameras_in_the_way(self.name, self._node_argument)
        if not in_the_way:
            return
        if replace is None:
            short = self.name[:-len('_camera')]
            others = ' or '.join(f"'{n[:-len('_camera')]}'" for n in in_the_way)
            raise RuntimeError(
                f'{self.name} cannot be opened: Baxter runs two of its three cameras at a '
                f'time (now: {", ".join(in_the_way)}). Say which one to switch off instead, '
                f"for example Camera('{short}').open(replace={others.split(' or ')[0]}), "
                f'or close one of them yourself first ({others}).')

        other = Camera._full_name(replace)
        if other == self.name:
            raise ValueError('replace must name another camera than this one')
        if other not in in_the_way:
            raise RuntimeError(
                f'Closing {other} would not help to open {self.name}: the cameras in the '
                f'way are {", ".join(in_the_way)}.')
        call_service(
            self._node.create_client(CloseCamera, 'cameras/close'),
            CloseCamera.Request(name=other)
        )
        dataflow.wait_for(
            self.has_power,
            timeout=20.0,
            rate=4,
            timeout_msg=f'{self.name} still cannot be opened after closing {other}',
        )

    @locked
    def close(self) -> None:
        """Switch the camera off (no error if it is off already).

        The camera also gives up its power, which goes to the camera that had none. To
        get it back: open(replace=...).
        """
        if self.name in list_cameras(self._node_argument):
            self.controller.close()
        with self._newest:
            self._msg = None  # frame() must not hand out a picture from before

    def frame(self, timeout: float = 5.0) -> images.Picture:
        """Return the newest picture.

        Each picture is handed out once: called in a loop, frame() waits for the next
        new picture every time, so the loop runs at the camera's speed.

        Parameters:
            timeout: seconds to wait for a picture (default 5)

        Returns:
            OpenCV image: NumPy array of shape (height, width, 3), blue-green-red

        Raises:
            TimeoutError if no picture arrives in time.
        """
        end = time.monotonic() + timeout
        with self._newest:
            while not self._has_new_frame():
                remaining = end - time.monotonic()
                if remaining <= 0 or not ros_is_running():
                    raise TimeoutError(
                        f'No picture from {self.name} within {timeout:g} s. Is the camera '
                        'open (camera.open())? ' + dataflow.BRIDGE_HINT)
                self._newest.wait(min(remaining, 0.1))
            msg = self._msg
            self._handed_out = self._received
        return images.from_image_msg(msg)

    def _has_new_frame(self) -> bool:
        return (self._msg is not None
                and self._received > self._handed_out
                and time.monotonic() - self._arrived < self._MAX_AGE)
