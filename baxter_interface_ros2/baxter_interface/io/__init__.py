"""Buttons, lights and simple sensors.

    digital_io.py  DigitalIO: on/off inputs and outputs (buttons, lights, valves)
    analog_io.py   AnalogIO: numeric inputs and outputs (e.g. the torso fan)
    navigator.py   Navigator: scroll wheel, buttons and lights on the arms and torso

Import the classes from the top level: from baxter_interface import Navigator
"""

from .analog_io import AnalogIO
from .digital_io import DigitalIO
from .navigator import Navigator

__all__ = ['AnalogIO', 'DigitalIO', 'Navigator']
