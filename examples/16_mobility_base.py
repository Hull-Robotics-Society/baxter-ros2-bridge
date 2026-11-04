#!/usr/bin/env python3
"""Example 16: the Mobility Base.

    python3 16_mobility_base.py            # reads only: mode, battery, bumpers, speed
    python3 16_mobility_base.py drive      # THE BASE MOVES: 10 cm forward and back

Needs the bridge for the base next to the robot's (docs/mobility-base.md). Before
"drive": someone at the joystick and the e-stop, joystick on, left stick up, mode switch
on "Computer", arms tucked, free floor around the base.

Shows: baxter_interface.MobilityBase.
"""

import sys

from baxter_interface import MobilityBase

base = MobilityBase()

print(f"Mode: {base.mode()} ({base.mode_meaning()})")
battery = base.battery()
print(f'Battery: {battery.percent} %, {battery.voltage:.1f} V, {battery.state}')
pressed = {corner: sides for corner, sides in base.bumpers().items() if sides}
print('Bumpers pressed:', pressed or 'none')
print('Speed now:', base.speed())

if len(sys.argv) > 1 and sys.argv[1] == 'drive':
    # Tell when a bumper is touched (the function runs in a thread of its own).
    base.bumpers_changed.connect(lambda bumpers: print('Bumpers:', bumpers))

    answer = input('The base will drive 10 cm forward and 10 cm back. Type "yes" to go on: ')
    if answer.strip() != 'yes':
        sys.exit('Nothing moved.')

    # 0.05 m/s for two seconds is about 10 cm. drive() returns when the base has stopped,
    # and raises an error that says why if the base does not follow.
    base.drive(forward=0.05, seconds=2.0)
    base.drive(forward=-0.05, seconds=2.0)
    print('Done. Mode now:', base.mode())
