#!/usr/bin/env python3
"""Example 5: show pictures on Baxter's screen (its "face"). Nothing moves.

    python3 05_screen.py
    python3 05_screen.py my_picture.png     # an image file of your own

Shows: Display.show() with an image file and with a picture drawn with OpenCV.
The screen has 1024 x 600 pixels; other sizes are scaled to fit.
"""

import sys
import time

import cv2
import numpy as np

from baxter_interface import Display

display = Display()

if len(sys.argv) > 1:
    display.show(sys.argv[1])
    print('Showing', sys.argv[1])
    sys.exit()

# A picture is an OpenCV image: a NumPy array of shape (height, width, 3) holding blue,
# green and red values from 0 to 255 for every pixel.
WIDTH, HEIGHT = 1024, 600


def face(eyes_open, text):
    """Draw a simple face and a line of text."""
    picture = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    picture[:] = (60, 40, 20)  # dark blue background (blue, green, red)
    for x in (WIDTH // 2 - 180, WIDTH // 2 + 180):
        if eyes_open:
            cv2.circle(picture, (x, 220), 70, (255, 255, 255), -1)
            cv2.circle(picture, (x, 220), 30, (0, 0, 0), -1)
        else:
            cv2.line(picture, (x - 70, 220), (x + 70, 220), (255, 255, 255), 12)
    cv2.ellipse(picture, (WIDTH // 2, 380), (160, 80), 0, 20, 160, (255, 255, 255), 12)
    cv2.putText(
        picture,
        text,
        (40, HEIGHT - 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.5,
        (255, 255, 255),
        3
    )
    return picture


print('A face that blinks and counts down ...')
for seconds_left in range(5, 0, -1):
    display.show(face(True, f'Hello! {seconds_left}'))
    time.sleep(0.8)
    display.show(face(False, f'Hello! {seconds_left}'))
    time.sleep(0.2)

# A picture stays on the screen until the next one is shown.
display.show(face(True, 'Bye!'))
print('Done. (display.clear() makes the screen black.)')
