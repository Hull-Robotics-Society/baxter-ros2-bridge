#!/usr/bin/env python3
"""Example 6: react to buttons and the scroll wheel, and switch lights. Nothing moves.

    python3 06_buttons_and_lights.py            # the navigator on the left arm
    python3 06_buttons_and_lights.py torso_left

A navigator is the group of a scroll wheel and three buttons. Baxter has four: one on
each arm ('left', 'right') and one on each side of the torso ('torso_left',
'torso_right'). Each has two lights, an inner and an outer one.

Shows: connecting your own functions to events (they are called when something
happens), reading buttons directly, and switching lights.
"""

import sys

from baxter_interface import Navigator

location = sys.argv[1] if len(sys.argv) > 1 else 'left'
navigator = Navigator(location)


# Functions that are called when something happens. Each one runs in a thread of its
# own, so it may take its time or even move the robot.
def on_ok_button(pressed):
    # Called with True when the button goes down and False when it comes up.
    print('wheel button', 'pressed' if pressed else 'released')
    navigator.inner_led = pressed       # the inner light is on while the button is held


def on_back_button(pressed):
    if pressed:
        navigator.outer_led = not navigator.outer_led   # each press switches the outer light
        print('outer light', 'on' if navigator.outer_led else 'off')


def on_wheel(position):
    print('wheel at', position)         # 0 to 255, wrapping round


navigator.button0_changed.connect(on_ok_button)     # button0: pressing the wheel
navigator.button1_changed.connect(on_back_button)   # button1: the button above the wheel
navigator.wheel_changed.connect(on_wheel)

# The same things can be read at any time, without events:
print(f'Right now: wheel at {navigator.wheel}, wheel button down: {navigator.button0}')

print(f'Try the {location} navigator: press the wheel, press the button above it, turn the wheel.')
# The connected functions only run while the program runs, so keep it alive here.
input('Press Enter here to quit.\n')

navigator.inner_led = False
navigator.outer_led = False
