#!/usr/bin/env python3
"""Example 9: read the distance and motion sensors. Nothing moves.

    python3 09_sensors.py

Baxter has:
  - a ring of twelve sonar (ultrasound) sensors around its head, which notice people
    and things around the robot;
  - an infrared distance sensor in each hand, which measures 4 to 40 cm along the gripper;
  - an accelerometer in each hand, which feels gravity, shaking and bumps.

Shows: Sonar, RangeSensor and Accelerometer, read in a loop and through an event.

Not yet tried on the robot (written 6 Oct 2026).
"""

import time

import baxter_interface
from baxter_interface import Accelerometer, RangeSensor, Sonar

with baxter_interface.setup():
    sonar = Sonar()
    hand_range = RangeSensor('left')
    accelerometer = Accelerometer('left')


# An event: called every time the accelerometer reports. At rest it measures gravity,
# about 9.8 m/s²; much more than that means the hand was shaken or bumped.
def on_acceleration(acceleration):
    strength = (acceleration.x ** 2 + acceleration.y ** 2 + acceleration.z ** 2) ** 0.5
    if strength > 14.0:
        print(f'  >>> the left hand was bumped ({strength:.0f} m/s²)')


accelerometer.changed.connect(on_acceleration)

print('Hold something in front of the left hand, walk around the robot, tap the left hand.')
print('Ctrl+C to quit.')
try:
    while True:
        # None means: nothing within the sensor's range.
        distance = hand_range.distance()
        in_front = 'nothing in range' if distance is None else f'{distance * 100:.0f} cm'

        # The nearest thing any sonar sensor sees: (sensor number 0 to 11, metres), or None.
        nearest = sonar.closest()
        around = 'nothing' if nearest is None else f'{nearest[1]:.2f} m (sensor {nearest[0]})'

        print(f'in front of the left hand: {in_front:18} nearest thing around the robot: {around}')
        time.sleep(0.5)
except KeyboardInterrupt:
    print()
