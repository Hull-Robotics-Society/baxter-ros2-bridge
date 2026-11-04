#!/usr/bin/env python3
"""Example 7: take a picture with a camera and show the live picture on the screen.

    python3 07_camera.py                  # the camera in the left hand
    python3 07_camera.py right_hand
    python3 07_camera.py head

Nothing moves. Baxter has three cameras ('left_hand', 'right_hand', 'head') but powers
only two at a time, normally the two in the hands. To use the head camera, one of the
hand cameras has to give up its power; this example hands it back at the end.

Shows: Camera.open(), Camera.frame(), saving a picture with OpenCV, and connecting the
camera to the screen.
"""

import sys

import cv2

import baxter_interface
from baxter_interface import Camera, Display

name = sys.argv[1] if len(sys.argv) > 1 else 'left_hand'

with baxter_interface.setup():
    camera = Camera(name)
    display = Display()

swapped = None
if camera.has_power():
    camera.open()                               # 640 x 400 pixels
else:
    # No power (normally the head camera): take it from the left hand's camera.
    swapped = 'left_hand'
    print(f'The {name} camera has no power; taking it from the {swapped} camera.')
    camera.open(replace=swapped)

# The newest picture, as an OpenCV image (a NumPy array: height x width x 3 colours).
picture = camera.frame()
height, width = picture.shape[:2]
cv2.imwrite('photo.png', picture)
print(f'Saved photo.png ({width} x {height} pixels)')

# Every new picture goes to the screen, with a frame drawn around it. A connected
# function that is slower than the camera gets the newest picture each time.
def show_with_frame(image):
    cv2.rectangle(image, (10, 10), (width - 10, height - 10), (0, 255, 0), 4)
    display.show(image)


camera.frame_received.connect(show_with_frame)
print("The camera's live picture is on Baxter's screen.")
input('Press Enter here to quit.\n')

camera.frame_received.disconnect(show_with_frame)
camera.close()
if swapped:
    Camera(swapped).open(replace=name)          # give the power back
display.clear()
