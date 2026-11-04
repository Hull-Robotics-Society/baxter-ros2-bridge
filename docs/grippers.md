# Grippers

Baxter's hands take two kinds of gripper: an **electric** one with two fingers, and a
**suction** one that holds things with a vacuum cup. The library tells which is mounted.

```python
from baxter_interface import Gripper, RobotEnable

RobotEnable().enable()
gripper = Gripper('left')        # the gripper on the left arm; or 'right'

print(gripper.type())            # 'electric', 'suction' or 'custom' (none or unknown)
```

## Electric gripper

```python
if not gripper.calibrated():
    gripper.calibrate()          # once after power-on: it opens and closes fully

gripper.close(block=True)        # block=True waits until the gripper is done
if gripper.gripping():           # the fingers were stopped by something between them
    print('got it')
gripper.open(block=True)

gripper.command_position(50, block=True)   # 0 = closed, 100 = open
print(gripper.position())                  # where the fingers are, 0 to 100
```

Without `block=True` a command returns at once and the gripper moves while your program
continues. Useful settings: `set_velocity(0-100)`, `set_moving_force(0-100)`,
`set_holding_force(0-100)`.

## Suction gripper

```python
gripper.close()                  # suction on
if gripper.gripping():           # the vacuum holds: something is on the cup
    print('got it')
gripper.open()                   # suction off, and a short puff to let go
```

## If the gripper does not react

- `gripper.error()` is True after a fault: `gripper.reset()` clears it; `gripper.reboot()`
  restarts the gripper (it must be calibrated again afterwards).
- An electric gripper that is not calibrated ignores position commands.
- `gripper.type()` is `'custom'`: the robot does not see a gripper on that arm. Check the
  cable at the wrist.

Example: [08_gripper.py](../examples/08_gripper.py).

## Limitations

- Not tested yet: suction grippers, and the force and speed settings of the electric
  gripper.
