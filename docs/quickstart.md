# Quickstart: your first program for Baxter

This page takes you from a set-up computer to a program of your own that moves the robot.
It assumes the computer has been set up as the [README](../README.md#on-another-computer)
describes: `scripts/setup.sh` and `scripts/add_robot.sh` have run. On a lab machine,
follow [lab-machines.md](lab-machines.md) first.

The commands below are for a computer with ROS 2 installed. Without ROS 2, programs run in
the `client` container: put `docker compose run --rm client` in front of each command, in
the repository's folder (see "Without ROS 2" at the end).

## Before the robot moves

- Someone stands at the **e-stop** (the red button on the cable) whenever the robot is
  enabled, and nobody is within reach of the arms.
- The arms are strong and do not see you. Try new programs slowly first:
  `arm.set_joint_position_speed(0.2)`.
- Every example says at its top whether it moves the robot.

## 1. Start the bridge

In the repository's folder:

```bash
docker compose up -d bridge
```

The bridge keeps running in the background. Stop it with `docker compose stop bridge`.
If the robot was switched off and on again, restart the bridge:
`docker compose restart bridge`.

## 2. Set up the terminal

In every terminal you use for Baxter:

```bash
source ~/baxter_ws/baxter.sh
```

It loads ROS 2 and the workspace `setup.sh` built, and sets the ROS 2 domain and
middleware the bridge uses. It prints one line, for example
`Baxter: ROS 2 jazzy, domain 42, discovery LOCALHOST, Fast DDS`.

## 3. Run a program that only reads

```bash
python3 ~/baxter-ros2-bridge/examples/01_read_robot.py
```

It prints the robot's software version, the joint angles of the left arm and where both
hands are. If it prints an error instead, the message says what is missing;
[troubleshooting.md](troubleshooting.md) lists the usual causes.

## 4. Look around with the command-line tools

```bash
ros2 run baxter_interface enable_robot -s      # the robot's state
ros2 run baxter_interface enable_robot -e      # enable the motors   (-d disables)
ros2 run baxter_interface tuck_arms -u         # arms to the "untucked" start pose
ros2 run baxter_interface robot_info           # software version, serial number, ...
```

All tools are described in [tools.md](tools.md).

## 5. Write your own program

Create a file in any folder, for example `my_first_program.py`:

```python
import baxter_interface
from baxter_interface import Head, Limb, RobotEnable

# Create the parts of the robot you want to use. Inside a setup() block they connect
# together, which is faster than one after the other.
with baxter_interface.setup():
    robot = RobotEnable()
    left = Limb('left')
    head = Head()

robot.enable()                        # switch the motors on
left.set_joint_position_speed(0.2)    # slowly

left.move_to_neutral()                # a safe start pose
left.move_by(z=0.10)                  # the hand 10 cm up
head.command_nod()
left.move_by(z=-0.10)                 # and down again
```

Run it:

```bash
python3 my_first_program.py
```

There is no `rclpy.init()`, no node and no spinning: the library does that in the
background. A program is plain Python from top to bottom, and it needs no ROS 2 package
around it.

For a real project, with VS Code, tests and extra Python packages, see
[own-project.md](own-project.md).

## 6. When you are done

The robot stays enabled when your program ends. Disable it, or fold the arms, which
disables it at the end:

```bash
ros2 run baxter_interface tuck_arms -t
```

Disabled, the arms sink down slowly.

## Where to go from here

| You want to | Read | Example |
| --- | --- | --- |
| move an arm, or the hand to a place | [arms.md](arms.md) | 02, 03, 10, 11 |
| move both arms (or arm and head) at the same time | [at-once.md](at-once.md) | 13 |
| turn the head, nod, switch the lights on the head | [head.md](head.md) | 04, 14 |
| open and close a gripper | [grippers.md](grippers.md) | 08 |
| take pictures, show pictures on the screen | [cameras-and-screen.md](cameras-and-screen.md) | 05, 07 |
| react to buttons, switch lights | [buttons-and-lights.md](buttons-and-lights.md) | 06, 11 |
| measure distances, feel bumps | [sensors.md](sensors.md) | 09 |
| see the robot in 3D on your screen | [rviz.md](rviz.md) | |
| plan motions around obstacles | [moveit.md](moveit.md) | 17 |
| send paths from other ROS 2 software | [trajectory-action.md](trajectory-action.md) | 12 |
| start a project of your own in VS Code | [own-project.md](own-project.md) | the template |
| work from another computer | [own-machine.md](own-machine.md) | |
| know what a command-line tool does | [tools.md](tools.md) | |
| understand an error | [troubleshooting.md](troubleshooting.md) | |

The examples are in [examples/](../examples/); each is a short program with comments.

## Three things worth knowing early

**Errors mean something.** The library does not carry on silently when the robot did not
get a command:

| Error | Meaning |
| --- | --- |
| `TimeoutError` | the robot got the command but did not finish in time (something in the way, robot not enabled, target out of range) |
| `ConnectionError` | the command did not get through, or the robot stopped reporting (bridge stopped, cable out, robot off) |
| `ValueError` | what you asked for is not possible (a place the hand cannot reach, a wrong name) |

**Events.** Instead of asking in a loop whether a button is pressed, connect a function;
it is called when it happens:

```python
navigator.button0_changed.connect(my_function)
```

See [buttons-and-lights.md](buttons-and-lights.md).

**One terminal, one setting.** `baxter.sh` changes only the terminal it runs in. A
terminal without it may use another ROS 2 domain or middleware and then does not see the
robot, or sees its topics but gets no answer from its services.

## Without ROS 2

On a computer without ROS 2, `setup.sh` downloads the `client` image instead: a container
with ROS 2, Baxter's messages and the Python library. In the repository's folder:

```bash
docker compose run --rm client python3 examples/01_read_robot.py
docker compose run --rm client ros2 run baxter_interface enable_robot -s
docker compose run --rm client          # a shell in the container; leave it with exit
```

The container sees the repository's folder as its working folder. For programs in a
folder of their own, put the folder's path into `.env` as
`PROJECT_DIR=/home/you/my_baxter_programs`; the container then starts in that folder.
