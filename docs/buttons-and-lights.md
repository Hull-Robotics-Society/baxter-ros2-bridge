# Buttons, wheels and lights

## Navigators

A navigator is a scroll wheel with three buttons. Baxter has four: one on each arm
(`'left'`, `'right'`) and one on each side of the torso (`'torso_left'`,
`'torso_right'`). Each has two lights.

```python
from baxter_interface import Navigator

navigator = Navigator('left')

navigator.button0        # True while the wheel is pressed
navigator.button1        # the button above the wheel
navigator.button2        # the button below the wheel
navigator.wheel          # position of the wheel, 0 to 255

navigator.inner_led = True      # lights
navigator.outer_led = False
```

## Events: functions that are called when something happens

Instead of asking in a loop, connect a function to an event:

```python
def on_press(pressed):
    if pressed:                       # True when pressed, False when released
        print('wheel pressed')

navigator.button0_changed.connect(on_press)
navigator.wheel_changed.connect(lambda position: print('wheel at', position))

input('Press Enter to quit')          # keep the program running
```

The events of a navigator are `button0_changed`, `button1_changed`, `button2_changed`
and `wheel_changed`. What to know about connected functions:

- Each connected function runs in **a thread of its own**. It may take as long as it
  likes, move the robot or wait for it; nothing else is held up.
- The calls of one function run in order, one at a time.
- Two *different* functions can run at the same moment. The library keeps the robot safe
  from that: blocking commands to the same arm, gripper or head run one after the other.
  Your own variables are yours to protect; if two functions change the same variable,
  use a `threading.Lock`.
- An error in a connected function is printed, and later calls still run.
- They only run while your program runs. A program that only reacts to events needs
  something at its end that keeps it alive, such as `input(...)`.
- `event.disconnect(my_function)` stops the calls.

Cameras (`frame_received`), sensors (`changed`) and digital inputs (`state_changed`) have
events of the same kind.

## Other buttons and lights

Every other button, light and switch of the robot is a `DigitalIO`, addressed by its
name:

```python
from baxter_interface import DigitalIO

cuff = DigitalIO('left_lower_cuff')        # True while the left cuff is held
print(cuff.state)
cuff.state_changed.connect(lambda held: print('cuff held:', held))

light = DigitalIO('left_outer_light')      # an output: the outer light of a navigator
light.set_output(True)
```

Some names (replace `left` by `right` for the other arm):

| Name | What |
| --- | --- |
| `left_lower_cuff` | the touch sensor in the cuff: the arm is being held |
| `left_lower_button` | the round button on the cuff |
| `left_upper_button` | the long, thin button on the cuff |
| `left_shoulder_button` | the button on the back of the torso, on that side |
| `head_red_light`, `head_yellow_light`, `head_green_light` | three lights on the head (not tried) |
| `left_inner_light`, `left_outer_light` | the navigator lights (also reachable through `Navigator`) |

**The full list of names, with what each one is, is in
[io-reference.md](io-reference.md).** Values that are numbers instead of on/off
(`torso_fan`, the wheels) are an `AnalogIO`, used the same way, except that its value is
read with `state()`.

The lights on the head (the red and green halo, the yellow sonar lights) are not a
`DigitalIO`; they have their own class, `HeadLights`, see [head.md](head.md).

With the bridge in its default `fixed_list` mode, each such name has to be listed in
`bridge_config/topics.yaml` first (how: [io-reference.md](io-reference.md), "Bridge
mode"); the navigator lights and the cuff sensors are listed already.

## Examples

[06_buttons_and_lights.py](../examples/06_buttons_and_lights.py),
[11_teach_and_repeat.py](../examples/11_teach_and_repeat.py).

## Limitations

- Of the names in the table, the navigators and their lights have been used on the robot;
  the others come from the robot's inventory and have not been tried one by one.
