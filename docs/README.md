# Documentation

Guides for programming the lab's Baxter robots from ROS 2 with the Python library
`baxter_interface`. Setting a computer up is in the
[repository's README](../README.md#on-another-computer) (on a lab machine:
[lab-machines.md](lab-machines.md)); start here once the bridge runs.

## Start here

| Page | Contents |
| --- | --- |
| [quickstart.md](quickstart.md) | from a set-up computer to your first own program |
| [../examples/](../examples/) | short programs with comments, one per part of the robot |
| [tools.md](tools.md) | the command-line tools: what each does and how to use it |
| [own-project.md](own-project.md) | a project of your own: folder layout, VS Code with completion and debugger, extra Python packages |
| [troubleshooting.md](troubleshooting.md) | what an error means and what to check |

## The parts of the robot

| Page | Contents | Classes |
| --- | --- | --- |
| [arms.md](arms.md) | joint angles, the hand to a place, several waypoints in one motion, control loops | `Limb` |
| [at-once.md](at-once.md) | several parts at the same time | `at_once()` |
| [head.md](head.md) | turning the head, nodding; the halo and the sonar lights | `Head`, `HeadLights` |
| [grippers.md](grippers.md) | electric and suction grippers | `Gripper` |
| [cameras-and-screen.md](cameras-and-screen.md) | pictures from the three cameras and on the screen | `Camera`, `Display` |
| [buttons-and-lights.md](buttons-and-lights.md) | navigators, buttons, lights, and how events work | `Navigator`, `DigitalIO`, `AnalogIO` |
| [sensors.md](sensors.md) | sonar ring, hand distance sensors, accelerometers | `Sonar`, `RangeSensor`, `Accelerometer` |
| [io-reference.md](io-reference.md) | the list of every button, light and switch by name | |

## Working with other ROS 2 software

| Page | Contents |
| --- | --- |
| [bridge-machine.md](bridge-machine.md) | the bridge on a Windows laptop: WSL2, and the robot on a cable |
| [own-machine.md](own-machine.md) | programming the robot from another computer than the one running the bridge |
| [lab-machines.md](lab-machines.md) | the university's lab machines: WSL in bridged mode, the robot over the university network, the machine's own ROS 2 (Lyrical) |
| [mobility-base.md](mobility-base.md) | the Mobility Base one robot stands on: a bridge of its own, `MobilityBase` for its state and for driving |
| [rviz.md](rviz.md) | the robot in 3D on your screen (RViz 2) |
| [moveit.md](moveit.md) | MoveIt 2: planned motions that avoid collisions, from the RViz panel or with `MotionPlanner` in a program |
| [two-robots.md](two-robots.md) | two robots (or more) at the same time: a name, a settings file and a bridge per robot, `Robot('one')` in programs |
| [simulation.md](simulation.md) | a simulated Baxter in Gazebo (branch `simulator`, work in progress) |
| [trajectory-action.md](trajectory-action.md) | whole paths from other ROS 2 software such as MoveIt |

## Records

These folders are not guides. They record what was found on the real machines, so the
setup can be rebuilt and compared later.

| Folder | Contents |
| --- | --- |
| [robot-inventory/](robot-inventory/) | everything a robot runs: its topics, services, parameters and message definitions. One folder per robot and date. [io-reference.md](io-reference.md) is a readable extract of its inputs and outputs |

## Limitations

Most guides end with a "Limitations" list: what does not work well, and what has not
been tested on the real robot yet. Use anything in the second group with care the first
time: slowly, and with someone at the e-stop.

Rethink Robotics' own documentation of the robot is kept at
[RethinkRobotics/sdk-docs](https://github.com/RethinkRobotics/sdk-docs/wiki); its
[API reference](https://github.com/RethinkRobotics/sdk-docs/wiki/API-Reference) describes every topic. It was written for earlier software
versions, so some names differ on our robots (the navigators, for example).

More about the project: [../CHANGELOG.md](../CHANGELOG.md) lists every change with its
reason; [../baxter_interface_ros2/README.md](../baxter_interface_ros2/README.md)
describes how the library is built; [../docker/bridge-jazzy/README.md](../docker/bridge-jazzy/README.md)
how the bridge works.
