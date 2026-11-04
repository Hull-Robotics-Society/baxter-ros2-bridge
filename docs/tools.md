# Command-line tools

Small ready-made programs that come with the library. They do the everyday things, such
as switching the motors on, bringing the arms to a start pose or looking at a camera,
without writing any code.

## How to run a tool

Every tool is started with `ros2 run baxter_interface <tool> <options>`, with the bridge
running, in a terminal set up with `source ~/baxter_ws/baxter.sh`:

```bash
ros2 run baxter_interface enable_robot -s
```

Without ROS 2 installed, in the repository's folder, put `docker compose run --rm client`
in front, or open a shell in the container once with `docker compose run --rm client`
and type the short form there. Every tool explains its options with `--help`.

## Overview

| Tool | What it does | Moves the robot |
| --- | --- | --- |
| [`enable_robot`](#enable_robot) | shows the robot's state; switches the motors on and off; resets after a stop | no (but arms sink when disabled) |
| [`robot_info`](#robot_info) | shows the robot's software version and other facts | no |
| [`move_to_neutral`](#move_to_neutral) | brings one or both arms to the neutral pose | **arms** |
| [`tuck_arms`](#tuck_arms) | folds the arms for transport, or unfolds them to a working pose | **arms, widely** |
| [`camera_control`](#camera_control) | lists, opens and closes the cameras | no |
| [`show_image`](#show_image) | puts a picture on the robot's screen | no |
| [`joint_trajectory_action_server`](#joint_trajectory_action_server) | lets other ROS 2 software (MoveIt) move the arms along paths | **arms**, when a path is sent |
| [`smoke_test`](#smoke_test) | the SDK's self-test of the robot's main functions | **arms, head, grippers** |
| [`tare`](#tare) | re-zeroes an arm's force sensing | **an arm, by itself**; changes the robot's calibration |
| [`calibrate_arm`](#calibrate_arm) | runs the robot's arm calibration | **an arm, by itself**; changes the robot's calibration |

Whenever something moves: someone at the e-stop, nobody within reach of the arms.

## Everyday tools

### enable_robot

The motors only work while the robot is "enabled". This tool is the first thing to run
in a session and the last.

```bash
ros2 run baxter_interface enable_robot -s     # show the state
ros2 run baxter_interface enable_robot -e     # enable: motors on
ros2 run baxter_interface enable_robot -d     # disable: motors off (the arms sink down slowly)
ros2 run baxter_interface enable_robot -r     # reset: clears a stop or an error; the robot is disabled afterwards
ros2 run baxter_interface enable_robot -S     # stop: like pressing the e-stop
```

`-s` prints the robot's state; its values mean:

| Value | Meaning |
| --- | --- |
| `ready` | enabled and ready to work |
| `enabled` | the motors are on |
| `stopped` | the robot was stopped (e-stop pressed, or `-S`); nothing moves until it is released and reset |
| `error` | the robot reports an error |
| `estop_button` | state of the e-stop button: 0 = not pressed, 1 = pressed, 2 = unknown, 3 = released but not yet reset |
| `estop_source` | where a stop came from: 0 = none, 1 = the user (the red button), 2 = unknown, 3 = a joint fault, 4 = the robot's main computer stopped answering |

After an e-stop: release the button, then `-r`, then `-e`.

### robot_info

```bash
ros2 run baxter_interface robot_info
```

Prints the robot's software version, serial number and similar facts. They live on the
robot as ROS 1 parameters, which ROS 2 cannot read; the bridge passes on a chosen set
(listed under `robot_info_params` in `bridge_config/topics.yaml`). Useful to check which
robot and which software you are connected to. `--timeout N` waits up to N seconds.

### move_to_neutral

```bash
ros2 run baxter_interface move_to_neutral             # both arms, one after the other
ros2 run baxter_interface move_to_neutral -l left     # one arm
```

Brings the arms to the neutral pose: elbows bent, hands in front of the robot. A good
start for any program. The robot must be enabled. `-t N` gives each arm N seconds
(default 15).

### tuck_arms

```bash
ros2 run baxter_interface tuck_arms -u     # untuck: arms out to a working pose
ros2 run baxter_interface tuck_arms -t     # tuck: arms folded close to the body, for transport or storage
```

The robot is delivered "tucked". Untuck it at the start of a session, tuck it at the end
if it is to be moved. The tool enables the robot itself and, when tucking, disables it
again at the end.

Tucking passes very close to the head, so the tool switches off the robot's collision
avoidance while it moves. **The arms swing widely: clear the space around the robot.**
`--timeout N` gives up after N seconds instead of waiting forever for a joint that
cannot reach its place.

### camera_control

```bash
ros2 run baxter_interface camera_control -l                          # list the cameras and their state
ros2 run baxter_interface camera_control -o left_hand_camera         # open, 1280 x 800
ros2 run baxter_interface camera_control -o head_camera -r 640x400   # open at another resolution
ros2 run baxter_interface camera_control -c left_hand_camera         # close
ros2 run baxter_interface camera_control -e                          # make the robot look for its cameras again
```

The cameras are `left_hand_camera`, `right_hand_camera` and `head_camera`. Baxter runs
only two of the three at a time: `-l` shows which are open. To open the third, close one
of the others first (`-c`); `-o` refuses to open a third camera, because the robot's
camera program can stop answering when asked to. In a program, the `Camera` class does
this for you ([cameras-and-screen.md](cameras-and-screen.md)).

Use this tool when a camera gives no pictures: `-l` tells why.

### show_image

```bash
ros2 run baxter_interface show_image photo.png     # any image file; scaled to fit the screen
ros2 run baxter_interface show_image               # the standard SDK picture
ros2 run baxter_interface show_image --black       # a black screen
```

The picture stays until another one is shown. To use a file from your computer in the
`client` container, put it into the folder the container starts in (the repository, or
your `PROJECT_DIR`).

## For other ROS 2 software

### joint_trajectory_action_server

```bash
ros2 run baxter_interface joint_trajectory_action_server                # both arms
ros2 run baxter_interface joint_trajectory_action_server -l left -m position_w_id --curve sdk
```

Runs until Ctrl+C and offers the standard `FollowJointTrajectory` action for each arm,
which motion planners such as MoveIt use to send whole paths. While it runs it keeps
holding the arms after each path, so stop it before moving an arm from another program.
Everything about it, including its modes and settings, is in
[trajectory-action.md](trajectory-action.md).

## Checking and maintaining the robot

### smoke_test

```bash
ros2 run baxter_interface smoke_test                # all tests, one after the other
ros2 run baxter_interface smoke_test -t Head        # one test
ros2 run baxter_interface smoke_test -p             # carry on after a failed test
```

The Baxter SDK's own self-test. **The robot moves, widely, in several of the tests.**

| Test | What it does | Moves |
| --- | --- | --- |
| `Enable` | enables, checks the state, disables | no |
| `Messages` | checks that the robot's messages arrive | no |
| `Services` | calls the robot's services (camera list, inverse kinematics) | no |
| `Head` | turns the head, nods, shows a picture on the screen | the head |
| `MoveArms` | moves both arms through a large part of their range, at the same time | **both arms, widely** |
| `Grippers` | calibrates and moves the grippers | the grippers |
| `BlinkLEDs` | blinks the navigator lights | no |
| `Cameras` | opens each camera and shows its picture on the screen | no |

The results are written to a file named after the date (`-o FILE` chooses another name).
For checking this project's bridge and library against the robot, `scripts/robot_check.sh`
(below) is the better start: it asks before every stage that moves.

### tare

```bash
ros2 run baxter_interface tare -l left
```

Re-zeroes the force sensing of one arm. Rethink recommends it about weekly, or when an
arm drifts or pushes by itself after someone leaned on it. **The arm moves by itself for
up to 5 minutes, and the robot's stored calibration changes.** Remove the gripper first
and clear the space. The tool asks for confirmation before it starts (`-y` skips the
question).

### calibrate_arm

```bash
ros2 run baxter_interface calibrate_arm -l left
```

The robot's thorough arm calibration; Rethink recommends it about monthly. **The arm
moves by itself for up to 10 minutes, and the robot's stored calibration changes.**
Remove the gripper first, clear the space, and run `tare` afterwards. It asks for
confirmation (`-y` skips it).

`tare` and `calibrate_arm` are for whoever looks after the robot. If in doubt, ask
before running them.

## Scripts in the repository

These are not part of the library; they live in `scripts/` and are run with `bash` from
the repository's folder on the bridge machine.

| Script | What it does |
| --- | --- |
| `scripts/robot_check.sh` | checks the bridge and the library against the real robot, stage by stage. Starts with stages that only read and asks before every stage that makes the robot do something. `--list` shows the stages, `--only cameras` runs one, `--from arms` starts at one. The `inventory` stage, which compares the robot's message checksums with the bridge's, runs only with `--inventory` or `--only inventory`: for a new robot, or one with other software than 1.2.0. Results go to `robot_check_logs/`. Run it at the start of a robot session and after changes. |
| `docker compose run --rm inventory` | records everything a robot runs (topics, services, parameters, message definitions, a short recording) into `docs/robot-inventory/`. Reads only. For a new robot, or after the robot's software changed. |
| `scripts/port_msgs_to_ros2.py` | generates the ROS 2 versions of Baxter's message packages (`msgs/ros2/`) from the original ones. Only needed if the messages ever change; the bridge image must be rebuilt afterwards. |

## Bridge mode

All tools work with the bridge in its default `fixed_list` mode: their topics are in
`bridge_config/topics.yaml`. Run `tare` and `calibrate_arm` in that mode only. Their
routines stop by themselves when no "keep going" message arrives for a second, and the
`on_demand` mode pauses all data once per second on the real robot.

## Limitations

- Not tested yet on the robot: `show_image` by hand, the smoke test `Enable`, and
  `camera_control -o`, `-c` and `-e`.
