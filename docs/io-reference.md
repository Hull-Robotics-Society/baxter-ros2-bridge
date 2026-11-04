# Reference: every button, light and switch

Everything the robot offers as a simple on/off or number value: its "digital" and
"analog" inputs and outputs. Each has a name, and the name is all a program needs:

```python
from baxter_interface import AnalogIO, DigitalIO

cuff = DigitalIO('left_lower_cuff')
print(cuff.state)                     # True while the left arm is held by its cuff

fan = AnalogIO('torso_fan')
print(fan.state())                    # a number
```

How these classes work (reading, switching, events) is in
[buttons-and-lights.md](buttons-and-lights.md). This page is the list of names.

**Where the list comes from:** the robot's own overview, recorded from the lab's Baxter
`011603p0009` (software 1.2.0.57) on 5 Oct 2026: 57 digital and 10 analog names. The
"In the recording" column is the value at that moment, with the robot standing idle. The
second robot may differ (it has the Mobility Base, and may have other grippers).

**How sure the descriptions are:** names marked *tried* were used on the robot. Names
marked *Rethink* are described in [Rethink's API reference](https://github.com/RethinkRobotics/sdk-docs/wiki/API-Reference#inputs-and-outputs),
which was written for an earlier software version but fits these names. The rest is
guessed from the name, and the table says so. Reading is always harmless. **Do not
switch an output whose purpose is unknown.**

## Digital inputs (29): read only

### On the arms

Replace `left` by `right` for the other arm.

| Name | What it is | In the recording |
| --- | --- | --- |
| `left_lower_cuff` | the touch sensor in the cuff: on while the cuff is squeezed (the robot then lets the arm move freely) (*Rethink*) | off |
| `left_lower_button` | the round button on the cuff, the "OK" button (*Rethink*) | off |
| `left_upper_button` | the long, thin button on the cuff, the "grasp" button (*Rethink*) | off |
| `left_shoulder_button` | the button on the back of the torso, on that side (*Rethink*) | **on**, with nobody pressing: it seems to report the other way round on this robot |
| `left_button_ok` | the navigator on the arm: pressing the wheel. Same as `Navigator('left').button0` (*tried* through `Navigator`) | off |
| `left_button_back` | the navigator's button above the wheel (`button1`) | off |
| `left_button_show` | the navigator's button below the wheel (`button2`) | off |

### On the torso

| Name | What it is | In the recording |
| --- | --- | --- |
| `torso_left_button_ok`, `torso_left_button_back`, `torso_left_button_show` | the navigator on the left side of the torso, as above (`Navigator('torso_left')`, *tried*) | off |
| `torso_right_button_ok`, `torso_right_button_back`, `torso_right_button_show` | the navigator on the right side of the torso | off |
| `torso_safety_stop` | unclear; by its name, part of the stop circuit | on |
| `torso_brake_sensor` | unclear | off |
| `torso_digital_input0` | unclear; by its name, a free input on the torso's connector | on |
| `torso_process_sense0`, `torso_process_sense1` | unclear | off |

### Inside the robot

| Name | What it is | In the recording |
| --- | --- | --- |
| `limit_switch_1`, `limit_switch_2`, `limit_switch_3` | unclear | off |
| `motor_fault_signal` | by its name, on when a motor reports a fault | off |

## Digital outputs (28): can be switched

### Lights

| Name | What it is | In the recording |
| --- | --- | --- |
| `left_inner_light`, `left_outer_light` | the two lights of the navigator on the left arm. Same as `Navigator('left').inner_led` / `.outer_led` (*tried*) | off |
| `right_inner_light`, `right_outer_light` | the same on the right arm (*tried*) | off |
| `torso_left_inner_light`, `torso_left_outer_light` | the navigator on the left of the torso (*tried*) | off |
| `torso_right_inner_light`, `torso_right_outer_light` | the navigator on the right of the torso (*tried*) | off |
| `head_red_light`, `head_yellow_light`, `head_green_light` | three lights by their name on the head; not tried, so where exactly they are is not known | red on, the others off |
| `right_blue_light` | unclear | off |

The lights on the head, the red and green halo and the twelve yellow sonar lights, are
**not** in this list. They have their own class, `HeadLights`: see "The lights on the head" in
[head.md](head.md).

### Grippers (suction)

Used by the `Gripper` class; there is normally no reason to switch them by hand.

| Name | What it is | In the recording |
| --- | --- | --- |
| `left_suck`, `right_suck` | sets the valve for holding; does not start the air by itself (*Rethink*) | off |
| `left_blow`, `right_blow` | sets the valve for letting go (*Rethink*) | off |
| `left_pneumatic`, `right_pneumatic` | switches the air pressure of that arm on and off (*Rethink*) | off |

### Leave alone

| Name | What it is | In the recording |
| --- | --- | --- |
| `left_hand_camera_power`, `right_hand_camera_power`, `torso_camera_power` | camera power (*Rethink*). The robot shares power between its cameras itself; use the `Camera` class ([cameras-and-screen.md](cameras-and-screen.md)) | on |
| `head_lcd_auto_config` | by its name, a setting of the screen | off |
| `torso_brake` | unclear | off |
| `torso_foot_pedal` | unclear | off |
| `torso_ui_output0` to `torso_ui_output3` | unclear; by their name, free outputs on the torso's connector | off |

## Analog inputs (8): numbers, read only

| Name | What it is | In the recording |
| --- | --- | --- |
| `left_hand_range`, `right_hand_range` | the distance sensor in the hand, as a raw number, roughly millimetres (*Rethink*). The `RangeSensor` class gives metres ([sensors.md](sensors.md)) | 65535: nothing in range |
| `left_wheel`, `right_wheel` | position of the navigator wheel on the arm. Same as `Navigator('left').wheel` | 0 |
| `torso_left_wheel`, `torso_right_wheel` | the wheels of the torso navigators | 0 |
| `left_vacuum_sensor_analog`, `right_vacuum_sensor_analog` | the vacuum sensor of a suction gripper (*Rethink*). Used by the `Gripper` class | 0 |

## Analog outputs (2): numbers that can be set

| Name | What it is | In the recording |
| --- | --- | --- |
| `torso_fan` | the fan in the torso (*Rethink*): 0 = automatic (the robot decides by temperature), 1 = off, 100 = full | 0: automatic |
| `torso_lighting` | the "workspace lights" (*Rethink*); what values it takes is not described | 0 |

## Bridge mode

With the bridge in its default `fixed_list` mode, only the navigator lights and the two
cuff sensors are listed; any other name needs its topic added to
`bridge_config/topics.yaml` first, followed by `docker compose restart bridge`:
`/robot/digital_io/<name>/state` (type `baxter_core_msgs/msg/DigitalIOState`) or
`/robot/analog_io/<name>/state` (`baxter_core_msgs/msg/AnalogIOState`). In `on_demand`
mode every name on this page works without a list, but that mode pauses all data once
per second on the real robot (see `docker/bridge-jazzy/README.md`).

## The raw data

This page was made from a recording of the topics `/robot/digital_io_states` and
`/robot/analog_io_states`. The robot inventory (`docker compose run --rm inventory`, see
[tools.md](tools.md)) makes such a recording of every topic. `topics.txt` in
`docs/robot-inventory/robot-1/` lists every topic of the robot.
