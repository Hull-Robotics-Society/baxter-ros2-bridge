# The Mobility Base

One of the lab's Baxters stands on a Dataspeed Mobility Base: a platform on four wheels
that can drive forward, sideways and turn on the spot. This page is about reaching the
base from ROS 2: its state (mode, battery, bumpers, speed) and driving it.

**The base is a machine that carries a 75 kg robot. Read "Before the base may move"
first.**

## How the base is built

```
wireless joystick  ~~~>  the base's controller  <-- USB -->  the base's computer "NUC"
                         (motors, bumpers)                   (Ubuntu, ROS 1 Kinetic)
                                                                      |
                                                                  network
                                                                      |
your program (ROS 2)  <->  the base's bridge  <-----------------------+
```

- **The joystick** is a radio-control transmitter of the kind used for model aircraft
  and drones, with Dataspeed's labels on it.
- **The controller** drives the motors and listens to the joystick. The motors only get
  power once the base's power button has been pressed, the same button that starts the
  base's computer. The joystick drives right away; nothing has to run on that computer.
  Then, with the e-stop released, the joystick drives the base.
- **The base's computer** runs Dataspeed's driver, which offers the base as ROS 1 topics.
  It is a ROS 1 system of its own, separate from the robot's.
- **The base's bridge** runs on the bridge machine next to the robot's bridge and
  carries the base's topics to ROS 2. Your programs do not have to run on the base.

## Before the base may move

1. Someone holds the joystick, knows it, and stays with it. Its left stick down stops
   the base at any time, whatever a program sends.
2. The e-stop is within reach. The robot and the base share one e-stop button: pressing
   it stops both. Check it before the first drive of the day: with it pressed the base
   must not move.
3. The robot's arms are tucked, or in a posture that stays inside the base's outline.
4. Free floor around the base: at least 60 cm on all sides, more in the driving
   direction. No cables on the floor that the wheels can catch, the network cable
   included.
5. A program can only drive the base while the joystick is on, its left stick is up
   ("enable") and its mode switch is on "Computer" (position 0). In "Manual" the joystick
   drives and programs are ignored.

The base stops by itself when no command has arrived for a fifth of a second. A program
that crashes, hangs or loses its connection therefore leaves a standing base.

### Driving without the joystick's permission

`drive(..., ignore_joystick=True)` (and the same for `set_velocity`) tells the base to
ignore its joystick while the program drives. The base then follows the program whatever
the joystick's switches say, also with the joystick switched off, so point 5 is not
needed. The price: **the joystick can no longer be relied on to stop the base.**
Dataspeed's documentation does not say whether its stop still works while it is ignored.
The e-stop button is then the way to stop the base by
hand, so someone has to be at the button, not at the joystick. The base goes back to
listening to the joystick a fifth of a second after the program's last command.

Try it first with the wheels off the ground or with a speed of a few centimetres per
second, and try the joystick's stop while it drives: then you know.

## Start

On the base's computer (monitor and keyboard, or SSH), start ROS itself and then the
base's programs. On the lab's base nothing starts by itself:

```bash
roscore
```

```bash
roslaunch mobility_base_bringup mobility_base.launch     # in a second terminal
```

On the bridge machine, once: the settings file for the base.

```bash
cp .env.base.example .env.base
nano .env.base            # the base computer's address and this machine's
```

Then the base's bridge, next to the robot's:

```bash
docker compose up -d bridge                          # the robot's, as always
docker compose --env-file .env.base up -d bridge     # the base's
docker compose --env-file .env.base logs -f bridge   # "Starting parameter_bridge", then Ctrl+C
```

The base's computer and the bridge machine must reach each other on the network. With
the robot, the base and the laptop on a small switch and no router, every machine needs
an address starting with `169.254.`; the base's computer may need one added by hand
(`sudo ip addr add 169.254.50.10/16 dev <its network card>`).

## Switching off

In this order; the circuit breaker cuts the base's battery off from everything, the
base's computer included, so that computer has to be off first:

1. On the bridge machine: `docker compose --env-file .env.base down` (and the robot's
   bridge, `docker compose down`).
2. The robot standing on the base: shut it down as usual and wait until it is off,
   above all if it takes its power from the base's battery.
3. The base's computer: Ctrl+C in the `roslaunch` terminal, then in the `roscore`
   terminal, then `sudo shutdown -h now` (over SSH the connection then drops). One press
   of the computer's power button does the same (Dataspeed sets it up that way). Wait
   until it is off.
4. Joystick off. Leave the e-stop released: a robot switched on with its e-stop pressed
   refuses to be enabled ("robot is still resetting").
5. Circuit breaker: press the red button (Dataspeed's "Shutdown"). The charging port
   works with the breaker off.

## Looking at the base

```bash
ros2 run baxter_interface base_status        # once
ros2 run baxter_interface base_status -w     # keep watching
```

(in a terminal set up with `source ~/baxter_ws/baxter.sh`; without ROS 2, put
`docker compose run --rm client` in front.)

```
mode 'waiting': the base waits for commands from a program | battery 61 % (24.1 V, discharging) | bumpers: none pressed | speed: forward +0.00, left +0.00 m/s, turn +0.00 rad/s
```

| Mode | Meaning |
| --- | --- |
| `estop` | the e-stop is pressed |
| `disabled` | the joystick is off, out of range, or its left stick is down |
| `joystick` | the joystick drives (mode switch on "Manual") |
| `waiting` | a program may drive; none is sending commands |
| `driving` | the base follows a program |
| `battery_low`, `battery_critical`, `motor_fault` | as they say |

## In a program

```python
from baxter_interface import MobilityBase

base = MobilityBase()

print(base.mode(), '-', base.mode_meaning())
print(base.battery().percent, '%')
print(base.bumpers())                 # which bumpers are pressed, and on which side
print(base.speed())                   # Speed(forward, left, turn), measured at the wheels

base.drive(forward=0.1, seconds=2.0)  # 0.1 m/s for two seconds: about 20 cm
base.drive(left=0.1, seconds=2.0)     # sideways
base.drive(turn=0.3, seconds=3.0)     # turn counter-clockwise, seen from above
```

Directions are the base's own, seen from the robot: forward is where Baxter looks, left
is Baxter's left.

With several robots ([two-robots.md](two-robots.md)): give the base's settings file the
same `ROBOT_NAME` as the robot standing on it. The base's topics then appear under that
name too (`/two/mobility_base/...`), and in a program the base is that robot's:

```python
two = baxter_interface.Robot('two')
base = two.MobilityBase()
```

- `drive()` repeats the command 20 times per second for the time given, then stops the
  base and returns. If the base does not follow within 1.5 s it stops trying and raises
  an error that names the base's mode, for example "the joystick drives the base". It
  also stops with an error when a bumper is pressed or the connection is lost.
- `set_velocity(forward, left, turn)` sends one command, for a loop of your own. Call it
  at least ten times per second for as long as the base should move.
- `stop()` stops the base.
- Speeds are cut down to `settings.BASE_MAX_SPEED` (0.3 m/s) and
  `settings.BASE_MAX_TURN` (0.5 rad/s), whatever a program asks for. The base itself can
  do 1.5 m/s. Raise the limits only when you know the space:
  `baxter_interface.settings.BASE_MAX_SPEED = 0.5`.
- `max_speed()` and `set_max_speed(linear, angular)` read and set the limit inside the
  base itself, which also applies to the joystick.
- `mode_changed` and `bumpers_changed` are signals, as the buttons have them
  ([buttons-and-lights.md](buttons-and-lights.md)): `base.bumpers_changed.connect(function)`.
- `drive()` works inside `at_once()` with arm and head commands.

The base has no command for "drive 30 cm": it knows speeds, not places. A distance comes
from speed times time and is only as exact as the floor allows.

## What the bridge carries

`bridge_config/mobility_base.yaml` lists it: mode, battery, bumpers, measured speed and
force, wheel positions, the joystick's requests, the inertial sensor, the speed command,
the topic that makes the base ignore its joystick (used only with
`ignore_joystick=True`), and the two speed-limit services. Left out on purpose: the
faster `cmd_vel_raw`, joystick pairing, and the floor camera (to save load; the file
says how to switch it on).

The base's message types (`mobility_base_core_msgs`) are in `msgs/`, from Dataspeed's
SDK 1.0.6. Their checksums equal those the lab's base sends.

## The lab's base

From its inventory (`docs/robot-inventory/`) and `scripts/base_check.sh`, which reads how
the base's computer is set up, over SSH, and changes nothing (how to run it is at the top
of the script):

| | |
| --- | --- |
| Computer | name `NUC`, Ubuntu 16.04.2 with ROS 1 Kinetic, factory user `mb` |
| Base software | Dataspeed's packages 1.0.3 (from 2017) |
| ROS system | its own (`ROS_MASTER_URI` is `localhost`) |
| At boot | nothing starts, as Dataspeed set the computer up; start it as under "Start" |
| Programs | driver, floor camera with picture processing, robot model |
| Rates | speed and wheels 250 per second, inertial sensor 100, joystick 50, mode 2 |

Dataspeed's documentation is offline; archived copies are in the Wayback Machine under
`http://mbsdk.dataspeedinc.com/`.

## Limitations

- The base's computer clock is behind (its clock battery is flat, and it has no time
  source without internet), so everything the base publishes carries a wrong time. To
  fix: replace the coin cell inside the computer (usually a CR2032), then set the time
  once.
- Not tested yet: `ignore_joystick`, the speed-limit services, moving the arms while the
  base drives.
- The meaning of the modes, and that a program needs the joystick's "Computer" position,
  come from Dataspeed's documentation.
