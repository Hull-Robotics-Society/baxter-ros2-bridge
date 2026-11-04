# Cameras and the screen

A picture is an **OpenCV image** everywhere in the library: a NumPy array of shape
`(height, width, 3)` with blue, green and red values from 0 to 255. Everything OpenCV
offers works on it (`cv2.cvtColor`, `cv2.rectangle`, `cv2.imwrite`, ...), and anything
you make with OpenCV can go to the screen.

## The screen

```python
from baxter_interface import Display

display = Display()
display.show('hello.png')        # an image file
display.show(picture)            # an OpenCV image
display.clear()                  # black
```

The screen has 1024 x 600 pixels. Other sizes are scaled to fit, with black borders. A
picture stays until the next one is shown. From the command line:
`ros2 run baxter_interface show_image hello.png`.

## Cameras

Baxter has three cameras: `'left_hand'`, `'right_hand'` and `'head'`.

```python
import cv2
from baxter_interface import Camera

camera = Camera('left_hand')
camera.open()                         # 640 x 400 pixels
picture = camera.frame()              # the newest picture
cv2.imwrite('photo.png', picture)
camera.close()
```

- `camera.open(resolution=(1280, 800))` for the full resolution. Valid sizes: 1280 x 800,
  960 x 600, 640 x 400, 480 x 300, 384 x 240, 320 x 200.
- `camera.frame()` hands out each picture once. In a loop it waits for the next new one,
  so the loop runs at the camera's speed.
- Exposure, gain and white balance: `camera.controller` (for example
  `camera.controller.exposure = 50`).

### Only two cameras at a time

Baxter runs **two of its three cameras at a time**, normally the two in the hands. To
use the third one, another has to be closed first:

```python
head_camera = Camera('head')
head_camera.has_power()                    # False while both hand cameras are open
head_camera.open(replace='left_hand')      # closes the left hand's camera first
...
Camera('left_hand').open(replace='head')   # and back
```

`open()` checks this itself and refuses to open a third camera (it says which ones are
in the way). That check takes a few seconds. It matters: asked for a third camera, the
robot's own camera program can stop answering, and then no camera delivers pictures
until the robot is restarted.

`ros2 run baxter_interface camera_control -l` shows the cameras and which are open.

At 640 x 400 a camera delivers 20 pictures per second, each 1 MB. Larger pictures
arrive less often.

### Every picture as it arrives

```python
def on_picture(picture):
    display.show(picture)

camera.frame_received.connect(on_picture)
input('Press Enter to quit')      # the function only runs while the program runs
```

A connected function that is slower than the camera always gets the newest picture; the
ones in between are skipped.

## Examples

[05_screen.py](../examples/05_screen.py) (drawing a face),
[07_camera.py](../examples/07_camera.py) (a photo, and the live picture on the screen).

## Limitations

- With both hand cameras at 640 x 400, each delivers about 20 pictures per second.
- Not tested yet: other resolutions, and exposure, gain and white balance.
