"""Tests for the picture helpers (OpenCV): no robot or bridge needed.

The tests for converting to and from ROS 2 messages need cv_bridge and are skipped where
it is not installed; the others need only OpenCV.
"""

import os
import pathlib

import cv2
import numpy as np
import pytest

import baxter_interface
from baxter_interface._core.images import fit, from_image_msg, load_image, to_image_msg

IMAGE_DIR = os.path.join(os.path.dirname(baxter_interface.__file__), 'images')

BLUE = (255, 0, 0)  # OpenCV order: blue, green, red
RED = (0, 0, 255)


def _plain(width, height, colour):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = colour
    return image


# --- load_image -----------------------------------------------------------------------------

@pytest.mark.parametrize('name', ['researchsdk.png', 'baxterworking.png'])
def test_loads_the_screen_images(name):
    image = load_image(os.path.join(IMAGE_DIR, name))

    assert image.shape == (600, 1024, 3)  # the size of Baxter's screen
    assert image.dtype == np.uint8


def test_loads_a_file_with_its_colours(tmp_path):
    path = str(tmp_path / 'picture.png')
    original = _plain(4, 2, RED)
    original[0, 0] = BLUE
    cv2.imwrite(path, original)

    assert np.array_equal(load_image(path), original)


def test_accepts_a_pathlib_path(tmp_path):
    path = tmp_path / 'picture.png'
    cv2.imwrite(str(path), _plain(3, 3, BLUE))

    assert load_image(pathlib.Path(path)).shape == (3, 3, 3)


def test_missing_file_is_reported(tmp_path):
    with pytest.raises(FileNotFoundError, match='No such image file'):
        load_image(str(tmp_path / 'missing.png'))


def test_file_that_is_not_an_image_is_reported(tmp_path):
    path = tmp_path / 'not_an_image.png'
    path.write_bytes(b'this is not a png')

    with pytest.raises(ValueError, match='not an image file'):
        load_image(str(path))


def test_colour_array_is_passed_through():
    image = _plain(5, 4, RED)

    assert load_image(image) is image


def test_grey_array_becomes_colour():
    grey = np.full((4, 5), 128, dtype=np.uint8)

    image = load_image(grey)

    assert image.shape == (4, 5, 3)
    assert tuple(image[0, 0]) == (128, 128, 128)


def test_transparency_is_dropped():
    with_alpha = np.zeros((4, 5, 4), dtype=np.uint8)
    with_alpha[:] = (0, 0, 255, 77)

    image = load_image(with_alpha)

    assert image.shape == (4, 5, 3)
    assert tuple(image[0, 0]) == RED


def test_array_of_wrong_type_is_reported():
    with pytest.raises(ValueError, match='uint8'):
        load_image(np.zeros((4, 5, 3), dtype=np.float32))


def test_array_of_wrong_shape_is_reported():
    with pytest.raises(ValueError, match='Not an image'):
        load_image(np.zeros((4, 5, 2), dtype=np.uint8))


# --- fit ------------------------------------------------------------------------------------

def test_fit_returns_an_image_of_the_right_size_unchanged():
    image = _plain(1024, 600, RED)

    assert fit(image, 1024, 600) is image


def test_fit_wide_image_gets_black_bars_above_and_below():
    result = fit(_plain(200, 100, RED), 1024, 600)  # becomes 1024 x 512

    assert result.shape == (600, 1024, 3)
    assert tuple(result[300, 512]) == RED       # centre: the picture
    assert tuple(result[300, 0]) == RED         # reaches the left edge
    assert tuple(result[10, 512]) == (0, 0, 0)  # bar above (44 pixels high)
    assert tuple(result[590, 512]) == (0, 0, 0)


def test_fit_tall_image_gets_black_bars_left_and_right():
    result = fit(_plain(100, 200, BLUE), 1024, 600)  # becomes 300 x 600

    assert result.shape == (600, 1024, 3)
    assert tuple(result[300, 512]) == BLUE
    assert tuple(result[0, 512]) == BLUE         # reaches the top edge
    assert tuple(result[300, 100]) == (0, 0, 0)  # bar on the left (362 pixels wide)
    assert tuple(result[300, 900]) == (0, 0, 0)


def test_fit_shrinks_large_images():
    result = fit(_plain(1280, 800, RED), 1024, 600)  # a camera picture: becomes 960 x 600

    assert result.shape == (600, 1024, 3)
    assert tuple(result[300, 512]) == RED
    assert tuple(result[300, 10]) == (0, 0, 0)


@pytest.mark.parametrize('size', [(1, 1), (1, 1000), (1000, 1), (1023, 599), (3000, 7)])
def test_fit_copes_with_odd_sizes(size):
    width, height = size

    assert fit(_plain(width, height, RED), 1024, 600).shape == (600, 1024, 3)


# --- ROS 2 messages (need cv_bridge) ---------------------------------------------------------

def test_image_survives_the_way_into_a_message_and_back():
    pytest.importorskip('cv_bridge')
    original = _plain(6, 4, RED)
    original[1, 2] = BLUE

    msg = to_image_msg(original)

    assert (msg.width, msg.height, msg.encoding) == (6, 4, 'bgr8')
    assert np.array_equal(from_image_msg(msg), original)


def test_image_from_a_message_may_be_changed():
    pytest.importorskip('cv_bridge')
    image = from_image_msg(to_image_msg(_plain(6, 4, RED)))

    image[0, 0] = BLUE  # would raise if the array were read-only

    assert tuple(image[0, 0]) == BLUE


def test_camera_format_with_transparency_becomes_plain_colour():
    cv_bridge = pytest.importorskip('cv_bridge')
    bgra = np.zeros((4, 6, 4), dtype=np.uint8)  # Baxter's cameras send 'bgra8'
    bgra[:] = (0, 0, 255, 255)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(bgra, encoding='bgra8')

    image = from_image_msg(msg)

    assert image.shape == (4, 6, 3)
    assert tuple(image[0, 0]) == RED
