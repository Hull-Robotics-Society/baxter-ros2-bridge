# Head

```python
from baxter_interface import Head, RobotEnable

RobotEnable().enable()
head = Head()

head.set_pan(0.6)               # turn to the robot's left and wait until it is there
head.set_pan(-0.6, speed=30)    # to its right, slowly
head.set_pan(0.0)               # straight ahead
head.command_nod()              # nod once
print(head.pan())               # the current angle
```

- Angles are in radians: 0 is straight ahead, positive is to the robot's left. Rethink's
  documentation gives the range as a quarter turn (1.57 rad) to each side, and says the
  head stops within about 0.12 rad of the angle asked for.
- `speed` goes from 0 to 100 (default 100).
- `set_pan()` waits until the head has arrived, at most `timeout` seconds (default 10);
  `set_pan(angle, timeout=0)` returns at once.
- `head.panning()` and `head.nodding()` tell whether the head is moving right now.
- The head only moves while the robot is enabled.

The screen on the head is described in [cameras-and-screen.md](cameras-and-screen.md), the
sonar ring around it in [sensors.md](sensors.md).

## The lights on the head

```python
from baxter_interface import HeadLights

lights = HeadLights()

lights.set_halo(red=100, green=0)       # the halo: red
lights.set_halo(green=100)              # green up as well: yellow-orange
print(lights.halo())                    # Halo(red=100.0, green=100.0)

lights.set_sonar_lights([0, 3, 7])      # these three yellow lights on, all others off
lights.set_sonar_lights('all')
lights.release_sonar_lights()           # back to the robot
```

There are two kinds of light on the head. Neither moves anything, and the robot does not
have to be enabled.

**The halo** is the red and green light at the top of the head. It is two lights, one
red and one green, each with a brightness from 0 (off) to 100 (full); mixed, they give
colours in between. `set_halo()` changes one or both; what you leave out stays. The
robot uses the halo itself (it was green in the recording of 5 Oct), so it may set it
again by itself.

**The sonar lights** are twelve yellow lights, one around each sonar sensor on the ring,
numbered 0 to 11 like the sensors. By default the robot switches each one on while its
sensor sees something near.

- `set_sonar_lights([...])` takes them over: the named ones on, all others off.
  `'all'` and `[]` work too.
- They stay like that until the next call, `release_sonar_lights()`, or the end of your
  program; then the robot switches them by its sonar again. (The robot only lets a
  program hold them while it keeps saying so 100 times per second. The library does that
  in the background.)
- `sonar_lights()` returns the lights that are on right now, whoever switched them:
  with the lights released, it shows where the sonar sees something.

For a pattern, call `set_sonar_lights` in a loop:

```python
import time

for round_number in range(3):
    for light in range(12):
        lights.set_sonar_lights([light])
        time.sleep(0.08)
lights.release_sonar_lights()
```

These lights are not a `DigitalIO`; they have topics of their own
(`/robot/sonar/head_sonar/lights/...`). How they are commanded is taken from
[Rethink's API reference](https://github.com/RethinkRobotics/sdk-docs/wiki/API-Reference#lights).

Examples: [04_head.py](../examples/04_head.py), [14_head_lights.py](../examples/14_head_lights.py).

## Limitations

- Not known yet: which light number is where on the sonar ring, and whether the robot
  later overrides a halo colour set by a program.
