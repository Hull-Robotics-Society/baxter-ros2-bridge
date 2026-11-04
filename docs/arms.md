# Arms

```python
from baxter_interface import Limb, RobotEnable

RobotEnable().enable()
left = Limb('left')          # or 'right'
```

The arm only moves while the robot is enabled. Someone at the e-stop, nobody within reach.

Each arm has seven joints, named from the shoulder to the wrist: `left_s0`, `left_s1`
(shoulder), `left_e0`, `left_e1` (elbow), `left_w0`, `left_w1`, `left_w2` (wrist). Angles
are in radians (3.14 rad = 180 degrees).

## Reading

```python
left.joint_angles()         # {'left_s0': 0.12, 'left_s1': -0.55, ...}
left.joint_angle('left_w2') # one joint
left.hand_position()        # Point(x=0.58, y=0.19, z=0.10)   metres
left.hand_orientation()     # Quaternion(x, y, z, w): how the hand is turned
```

Also: `joint_names()`, `joint_velocities()`, `joint_efforts()` (torques), and the SDK's
`endpoint_pose()`, `endpoint_velocity()`, `endpoint_effort()`.

**Places** are measured in metres from the middle of the robot's torso, seen from the
robot: **x forward, y to the robot's left, z up**. To find the coordinates of a place,
hold the arm by its cuff (the ring behind the hand; the robot lets the arm move freely
while it is held), put the hand there and print `hand_position()`.

## Moving to joint angles

```python
left.set_joint_position_speed(0.2)       # 0.0 to 1.0; the robot's default is 0.3
left.move_to_neutral()                   # a safe start pose
left.move_to_joint_positions({'left_w2': 0.8, 'left_e1': 1.0})
```

A move names the joints to change; the others stay. It returns when the arm has arrived
(within 0.5 degrees), and raises `TimeoutError` if that takes longer than `timeout`
(default 15 s) or `ConnectionError` if the robot stops reporting on the way.

## Moving the hand to a place

```python
left.move_by(z=0.05)                     # 5 cm up from where the hand is
left.move_by(x=0.10, y=-0.02)            # 10 cm forward, 2 cm to the robot's right
left.move_to_pose(x=0.7, y=0.3, z=0.1)   # to a place
left.move_to_pose(z=0.3)                 # only change the height
```

The robot works out the joint angles for the place itself (inverse kinematics, a service
on the robot). What you leave out stays as it is: a missing coordinate keeps its value,
and without `orientation=(x, y, z, w)` the hand stays turned as it is.

If the arm cannot reach the place, a `ValueError` says so and nothing moves.
`solve_pose(...)` takes the same arguments and only returns the joint angles.

Two things to know:

- The hand does **not** travel in a straight line; each joint goes its own way to its new
  angle. For a straight path, go there in several small steps.
- The robot looks for a posture close to the current one first. If there is none it may
  choose a quite different posture, and the elbow swings round. Small steps avoid that.

## Through several waypoints in one motion

`move_to_...` stops at its target. To pass through several waypoints without stopping:

```python
left.move_through([posture_1, posture_2, posture_3])
left.move_through([(0.7, 0.3, 0.1), (0.7, 0.1, 0.3), (0.6, 0.3, 0.2)], seconds=8)
```

A waypoint is either a **posture**, a dictionary of joint angles as `joint_angles()`
returns it, or a **place** for the hand, `(x, y, z)` in metres (the hand stays turned as
it is). Both kinds can be mixed. The joint angles for places are worked out before
anything moves, so an unreachable place raises `ValueError` with the arm still at rest.

- `speed=0.5` sets how fast to go: roughly the speed, in rad/s, of the joint that has
  furthest to travel. Or give `seconds=` for the whole motion.
- The path starts where the arm is; the arm starts and ends at rest.
- If the arm falls more than 0.35 rad behind (it is held back, or the path is faster
  than `set_joint_position_speed()` lets the arm go), the move stops with a
  `TimeoutError` and the arm holds its position.
- Between waypoints each joint follows a smooth curve; the hand does not travel in
  straight lines.
- `curve='sdk'` uses the curve of the original SDK's trajectory server instead: the arm
  then starts and ends at speed. [trajectory-action.md](trajectory-action.md) compares
  the two.

## Both arms at the same time

Inside an `at_once()` block, commands for different parts of the robot run at the same
time, and the block waits until all are done:

```python
with baxter_interface.at_once():
    left.move_to_neutral()
    right.move_to_neutral()
```

See [at-once.md](at-once.md).

## A control loop of your own

For motion that never stops (following something, waving), send targets yourself, many
times per second:

```python
left.set_joint_positions({'left_w2': angle})     # returns at once
```

See [examples/10_wave.py](../examples/10_wave.py) for a complete loop with a steady beat.
The rules:

- Send at least 5 times per second; 100 is usual. If the commands stop, the arm holds its
  position (`set_command_timeout()` changes after how long).
- These commands are not confirmed, because waiting would break the loop's timing. Check
  `left.joint_state_age()` in the loop: it is normally about 0.01 s, and a large value
  means the connection is lost.
- `set_joint_velocities()` and `set_joint_torques()` work the same way, for speeds and
  torques. With torques the arm does only what you command: start with small values.
- `set_joint_positions(angles, raw=True)` sends the angles to the joints unchanged. The
  arm follows closely spaced targets more exactly, but the robot's speed limit and its
  **collision avoidance between the arms are bypassed**. Only for controllers of your own.

## Whole paths: the trajectory action

To send a list of waypoints with times and have the arm pass through them in one smooth
motion, or to connect a motion planner such as MoveIt, see
[trajectory-action.md](trajectory-action.md).

## Command-line tools

```bash
ros2 run baxter_interface move_to_neutral          # both arms (-l left for one)
ros2 run baxter_interface tuck_arms -u             # untuck; -t tucks the arms for transport
```

## Examples

[02_move_arm.py](../examples/02_move_arm.py) (joint angles),
[03_move_hand.py](../examples/03_move_hand.py) (places),
[10_wave.py](../examples/10_wave.py) (own loop),
[11_teach_and_repeat.py](../examples/11_teach_and_repeat.py) (show postures by hand,
`move_through`), [13_at_once.py](../examples/13_at_once.py) (both arms at once).

## Limitations

- Not tested yet on the robot: `move_to_pose` with an orientation, `solve_pose` on its
  own, `raw=True`, and `curve='sdk'`.
