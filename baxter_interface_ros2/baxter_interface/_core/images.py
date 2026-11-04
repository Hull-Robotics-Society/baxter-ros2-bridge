"""Pictures: loading, resizing, and converting to and from ROS 2 image messages.

Built on OpenCV, the standard image library in robotics. Inside the library, and in your
own programs, a picture is always an OpenCV image: a NumPy array of shape
(height, width, 3) holding blue, green, red values from 0 to 255 for every pixel. Anything
OpenCV offers works on it directly (cv2.imwrite, cv2.rectangle, cv2.cvtColor, ...).

    load_image()      a file or an array  ->  OpenCV image
    fit()             scale a picture to a given size, keeping its proportions
    to_image_msg()    OpenCV image  ->  sensor_msgs/Image   (uses cv_bridge)
    from_image_msg()  sensor_msgs/Image  ->  OpenCV image   (uses cv_bridge)

load_image() and fit() need only OpenCV; the two message functions need ROS 2.
"""

from __future__ import annotations

import os
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

# A picture: an OpenCV image, a NumPy array of shape (height, width, 3) holding blue,
# green and red values from 0 to 255.
Picture = NDArray[np.uint8]

_bridge: Any = None


def _cv_bridge() -> Any:
    # Imported on first use, so load_image() and fit() also work without ROS 2.
    global _bridge
    if _bridge is None:
        from cv_bridge import CvBridge
        _bridge = CvBridge()
    return _bridge


def load_image(source: str | os.PathLike[str] | np.ndarray) -> Picture:
    """Turn a file name or an array into an OpenCV image.

    Parameters:
        source: path of an image file (PNG, JPEG, BMP, ... anything OpenCV reads), or an
                image as a NumPy array: colour (height, width, 3) in blue-green-red order,
                colour with transparency (height, width, 4), or grey (height, width).
                Values must be whole numbers from 0 to 255 (dtype uint8).

    Returns:
        NumPy array of shape (height, width, 3), dtype uint8, blue-green-red

    Raises:
        FileNotFoundError if the file does not exist, ValueError if it is not a picture
        or the array has the wrong shape or type.
    """
    if isinstance(source, (str, os.PathLike)):
        path = os.fspath(source)
        if not os.path.isfile(path):
            raise FileNotFoundError(f'No such image file: {path}')
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'{path} is not an image file that OpenCV can read')
        return image

    image = np.asarray(source)
    if image.dtype != np.uint8:
        raise ValueError(
            f'Image values must be whole numbers from 0 to 255 (dtype uint8), not {image.dtype}. '
            'Convert with image.astype(numpy.uint8).')
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.ndim == 3 and image.shape[2] == 3:
        return image
    if image.ndim == 3 and image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    raise ValueError(
        f'Not an image: array of shape {image.shape}. Expected (height, width, 3), '
        '(height, width, 4) or (height, width).')


def fit(image: np.ndarray, width: int, height: int) -> Picture:
    """Scale an image to fit into width x height, keeping its proportions.

    The picture is made as large as fits and centred; what remains is filled with black.

    Parameters:
        image:  OpenCV image (height, width, 3)
        width:  width of the result in pixels
        height: height of the result in pixels

    Returns:
        OpenCV image of exactly (height, width, 3). The input itself if it already fits.
    """
    image_height, image_width = image.shape[:2]
    if (image_width, image_height) == (width, height):
        return image

    scale = min(width / image_width, height / image_height)
    new_width = max(1, min(width, round(image_width * scale)))
    new_height = max(1, min(height, round(image_height * scale)))
    resized = cv2.resize(
        image,
        (new_width, new_height),
        # INTER_AREA gives the cleanest result when shrinking, INTER_LINEAR when enlarging.
        interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    )

    result = np.zeros((height, width, 3), dtype=np.uint8)
    left = (width - new_width) // 2
    top = (height - new_height) // 2
    result[top:top + new_height, left:left + new_width] = resized
    return result


def to_image_msg(image: np.ndarray) -> Any:
    """Convert an OpenCV image into a sensor_msgs/Image message ('bgr8' encoding).

    Parameters:
        image: OpenCV image (height, width, 3), dtype uint8, blue-green-red

    Returns:
        sensor_msgs.msg.Image
    """
    return _cv_bridge().cv2_to_imgmsg(image, encoding='bgr8')


def from_image_msg(msg: Any) -> Picture:
    """Convert a sensor_msgs/Image message into an OpenCV image.

    Whatever colour format the message uses (Baxter's cameras send 'bgra8'), the result
    is blue-green-red.

    Parameters:
        msg: sensor_msgs.msg.Image

    Returns:
        NumPy array of shape (height, width, 3), dtype uint8, that you may change freely
    """
    image = _cv_bridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
    if not image.flags.writeable:
        # Without a conversion the array still points into the message, read-only;
        # drawing on it (cv2.rectangle, ...) would fail.
        image = image.copy()
    return image
