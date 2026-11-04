#!/usr/bin/env python3
"""Example 14: the lights on the head. Nothing moves; the robot need not be enabled.

    python3 14_head_lights.py

Baxter's head has two kinds of light:
  - the halo at the top: a red and a green light that can be mixed;
  - twelve yellow lights on the ring, one around each sonar sensor. Normally the robot
    switches each one on while its sensor sees something near.

Shows: HeadLights: setting the halo's colour, taking the sonar lights over, patterns,
and handing them back to the robot.

Not yet tried on the robot (written 6 Oct 2026).
"""

import time

from baxter_interface import HeadLights

lights = HeadLights()

before = lights.halo()
print(f'The halo is at red {before.red:g}, green {before.green:g} (0 = off, 100 = full).')

print('Halo: red, then both, then green ...')
lights.set_halo(red=100, green=0)
time.sleep(1.0)
lights.set_halo(green=100)          # red stays as it is
time.sleep(1.0)
lights.set_halo(red=0)
time.sleep(1.0)

print('Halo: fading from green to red ...')
for step in range(0, 101, 5):
    lights.set_halo(red=step, green=100 - step)
    time.sleep(0.05)
lights.set_halo(red=before.red, green=before.green)     # as it was

# The sonar lights are numbered 0 to 11. set_sonar_lights() switches the named ones on
# and all others off, and keeps them like that until the next call.
print('Sonar lights: one light running round the ring ...')
for _round in range(3):
    for light in range(12):
        lights.set_sonar_lights([light])
        time.sleep(0.08)

print('Sonar lights: every second one, then the others ...')
for _blink in range(4):
    lights.set_sonar_lights([0, 2, 4, 6, 8, 10])
    time.sleep(0.4)
    lights.set_sonar_lights([1, 3, 5, 7, 9, 11])
    time.sleep(0.4)

print('Sonar lights: all on, all off ...')
lights.set_sonar_lights('all')
time.sleep(1.0)
lights.set_sonar_lights([])
time.sleep(1.0)

# Hand them back: the robot switches them by its sonar again. (This also happens by
# itself when the program ends.)
lights.release_sonar_lights()
time.sleep(1.0)
print('Back with the robot. Lights on where the sonar sees something:', lights.sonar_lights())
