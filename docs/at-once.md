# Several things at the same time: `at_once()`

A command that waits for the robot, such as `left.move_to_neutral()`, returns when the
arm has arrived. Two of them in a row run one after the other:

```python
left.move_to_neutral()      # about 4 seconds
right.move_to_neutral()     # then another 4
```

Inside an `at_once()` block the commands only *start*, and the block waits at its end
until all of them are done, however long each takes:

```python
import baxter_interface

with baxter_interface.at_once():
    left.move_to_neutral()
    right.move_to_neutral()
    head.set_pan(0.5)

# here both arms and the head have arrived
print('all done')
```

## The rules

- **Different parts run at the same time**: the left arm, the right arm, the head, each
  gripper and each camera are parts.
- **Several commands for the same part run in the order you wrote them**, one after the
  other, while the other parts carry on:

  ```python
  with baxter_interface.at_once():
      left.move_to_neutral()        # the left arm does these two in a row ...
      left.move_by(z=0.10)
      right.move_to_neutral()       # ... while the right arm does this
  ```

- **The block ends when everything is done.**
- **Inside the block, commands return nothing**, because they have not happened yet. If
  you need a value (`gripper.gripping()`, `left.hand_position()`), ask after the block.
- Everything that does not wait for the robot happens at once, as always: reading
  values, `print`, switching a light, your own calculations. A `time.sleep(2)` in the
  block pauses your program, not the robot.

Commands that take part: `Limb.move_to_neutral`, `move_to_joint_positions`,
`move_to_pose`, `move_by`, `move_through`; `Head.set_pan`, `command_nod`;
`Gripper.open`, `close`, `command_position`, `calibrate` (inside a block they wait for
the gripper, as with `block=True`); `Camera.open`.

## Errors

If a command fails (a `TimeoutError` because something was in the way, a `ValueError`
because a place is out of reach), the later commands *for that part* are not started.
The other parts finish what they were given. Then the error is raised at the end of the
block, so an ordinary `try` around the block catches it:

```python
try:
    with baxter_interface.at_once():
        left.move_to_pose(x=0.7, y=0.3, z=0.1)
        right.move_to_neutral()
except ValueError as error:
    print('The left hand cannot go there:', error)
```

## Your own functions

A function of yours can run alongside:

```python
def blink():
    for _ in range(6):
        navigator.inner_led = not navigator.inner_led
        time.sleep(0.3)

with baxter_interface.at_once() as group:
    left.move_to_neutral()
    group.run(blink)             # group.run(function, arguments...)
```

## How it works

Each command in the block is handed to a helper thread, one per robot part, and your
program moves on to the next line. At the end of the block the program waits for the
helper threads. Outside a block nothing changes.

`at_once()` is for commands that wait. Things that should happen *when something
happens* (a button press) are events: see [buttons-and-lights.md](buttons-and-lights.md).

Example: [13_at_once.py](../examples/13_at_once.py).

## Limitations

- Not tested yet on the robot: two `move_through` in one block, and grippers or cameras
  inside a block.
