"""Show pictures on Baxter's screen (the "face").

New in the ROS 2 library; the ROS 1 SDK had no class for this, only an example script.

    from baxter_interface import Display

    display = Display()
    display.show('hello.png')        # an image file
    display.show(camera.frame())     # or an OpenCV image, e.g. from a camera
    display.clear()                  # black screen

The screen has 1024 x 600 pixels. Pictures of any other size are scaled to fit, keeping
their proportions, with black borders where needed. The picture stays until the next one
is shown, also after your program has ended.

With BRIDGE_MODE=fixed_list, /robot/xdisplay must be in bridge_config/topics.yaml.
"""

from __future__ import annotations

import os

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

from ._core import images
from ._core import dataflow
from ._core.locks import locked, part_lock
from ._core.node import get_node


class Display:
    """Interface class for Baxter's screen.

    Available methods:
        show(image) - show an image file or an OpenCV image
        clear()     - black screen
    """

    WIDTH = 1024
    HEIGHT = 600

    def __init__(self, node: Node | None = None) -> None:
        """
        Parameters:
            node: optional ROS 2 node to use (default: the shared background node)
        """
        self._node = get_node(node)
        # One picture at a time, also when several threads show pictures.
        self._part_lock = part_lock('display', self._node)
        # Queue of 1: if pictures are shown faster than they can be sent (a camera's
        # frames, for example), the older ones are dropped instead of piling up.
        self._pub = self._node.create_publisher(
            Image,
            'robot/xdisplay',
            1
        )

    @locked
    def show(self, image: str | os.PathLike[str] | np.ndarray, timeout: float = 5.0) -> None:
        """Show a picture on the screen.

        Parameters:
            image:   path of an image file (PNG, JPEG, ...) or an OpenCV image (NumPy
                     array, blue-green-red). Any size; it is scaled to fit the screen.
            timeout: seconds to wait for the bridge to confirm the picture (default 5)

        Raises:
            FileNotFoundError or ValueError if the picture can't be read, ConnectionError
            if the picture could not be delivered (bridge not running?).
        """
        picture = images.fit(
            images.load_image(image),
            self.WIDTH,
            self.HEIGHT
        )
        msg = images.to_image_msg(picture)

        # A picture is sent once: wait until the bridge listens, and until it has
        # confirmed the picture (a picture is large and travels in pieces).
        dataflow.publish_once(
            self._pub,
            msg,
            what='picture for the screen',
            confirm=timeout
        )

    def clear(self) -> None:
        """Make the screen black."""
        self.show(np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8))
