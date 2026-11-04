# Sensors: sonar, hand distance, accelerometers

These three classes are new in the ROS 2 library; the original SDK only had the raw
topics.

## Sonar ring

Twelve ultrasound sensors in a ring around the head notice people and things around the
robot.

```python
from baxter_interface import Sonar

sonar = Sonar()
sonar.distances()     # {3: 0.92, 4: 1.10}: sensor number -> metres
sonar.closest()       # (3, 0.92): the nearest thing, or None
sonar.points()        # {3: Point(x, y, z), ...}: where the echoes are, in metres
```

Only sensors that hear an echo are listed; an empty result means nothing is near.
A report can hold two echoes from the same sensor; `distances()` and `points()` then keep
one of them.
`points()` uses the same directions as the arms: x forward, y to the robot's left, z up.

The yellow lights around the sensors, and the halo above them, are described in
[head.md](head.md) ("The lights on the head").

## Distance sensor in the hand

An infrared sensor in each hand measures along the gripper, from 4 to 40 cm.

```python
from baxter_interface import RangeSensor

hand_range = RangeSensor('left')
hand_range.distance()     # 0.12 (metres), or None if nothing is within range
```

Useful to stop a hand just above a table or an object.

## Accelerometer in the hand

```python
from baxter_interface import Accelerometer

accelerometer = Accelerometer('left')
accelerometer.acceleration()    # Vector(x, y, z) in m/s², in the hand's own directions
accelerometer.strength()        # its size: about 9.8 at rest (gravity)
```

At rest it measures gravity, which tells which way the hand is tilted. A value far above
9.8 means the hand was shaken or bumped.

The directions, according to [Rethink's API reference](https://github.com/RethinkRobotics/sdk-docs/wiki/API-Reference#accelerometers): the
sensor sits inside the cuff. Its z points back up the arm (towards the wrist), x towards
the hand camera, y towards the cuff buttons. Gravity is not subtracted.

## Events

All three have a `changed` event that is called with every new measurement (see
[buttons-and-lights.md](buttons-and-lights.md) for how events work):

```python
hand_range.changed.connect(lambda distance: print(distance))
```

A function that is slower than the sensor gets the newest value each time.

## Lost connection

Each read raises a `ConnectionError` when the sensor has not reported for a second,
instead of returning an old value.

Example: [09_sensors.py](../examples/09_sensors.py).

## Limitations

- Not tested yet: a hand range sensor with something in front of it, and the `changed`
  events.
- Not known yet: how the twelve sonar sensors are numbered around the head.
