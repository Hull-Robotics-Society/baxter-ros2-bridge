# baxter-ros2-bridge

Program the lab's Baxter robots from ROS 2 and current Python. The robots run frozen
software (ROS 1 Indigo, Baxter software 1.2.0) that can't be updated, since Rethink
Robotics no longer exists. This repository contains a bridge that runs in Docker: it
talks ROS 1 to the robot and ROS 2 to your programs. It also contains a Python library,
`baxter_interface`, with the same classes as the original Baxter SDK plus a few simpler
ones, and a MoveIt 2 configuration.

```
Baxter (ROS 1 master)  <--ROS 1-->  bridge (Docker, ROS 2 Jazzy)  <--ROS 2-->  your programs
```

```python
from baxter_interface import Limb, RobotEnable

RobotEnable().enable()
left = Limb('left')
left.move_to_neutral()
left.move_by(z=0.10)        # the hand 10 cm up
```

## On a lab machine

In the WSL terminal of one of the university's lab machines:

```bash
git clone https://github.com/Hull-Robotics-Society/baxter-ros2-bridge.git ~/baxter-ros2-bridge
cd ~/baxter-ros2-bridge
bash scripts/setup.sh                              # once per WSL image
bash scripts/add_robot.sh 011603p0009.hull.ac.uk   # every session
docker compose up -d bridge
source ~/baxter_ws/baxter.sh                       # in every terminal for Baxter
ros2 run baxter_interface tuck_arms -u
```

[docs/lab-machines.md](docs/lab-machines.md) explains each step, how to end a session,
and why saving the WSL image matters.

## On another computer

Any Ubuntu computer or WSL2 works the same way, with or without ROS 2 installed:

```bash
bash scripts/setup.sh
bash scripts/add_robot.sh <the robot's name>
docker compose up -d bridge
```

- **The robot's name:** its serial number with `.hull.ac.uk` on the university network.
  On a cable straight to the robot it is `<serial>.local`: set the wired connection's IPv4
  method to "Link-Local Only" and wait about a minute after the robot has started.
- **With ROS 2 installed** (Jazzy or Lyrical), `setup.sh` builds everything in
  `~/baxter_ws`, and `source ~/baxter_ws/baxter.sh` sets up a terminal.
- **Without ROS 2,** programs run in a container:
  `docker compose run --rm client python3 examples/01_read_robot.py`.
- **A Windows laptop:** the bridge runs in WSL2, see [docs/bridge-machine.md](docs/bridge-machine.md).
- **Programs on a second computer:** see [docs/own-machine.md](docs/own-machine.md).

The bridge needs a network path to the robot in both directions: it connects to the robot
to read, and the robot connects back to it to receive commands.

To check everything against the robot, stage by stage (it asks before anything moves):

```bash
bash scripts/robot_check.sh
```

## Documentation

New here? After the setup above, follow the [quickstart](docs/quickstart.md).
[docs/README.md](docs/README.md) lists all guides.

| Page | Contents |
| --- | --- |
| [docs/quickstart.md](docs/quickstart.md) | from a running bridge to your first own program |
| [docs/lab-machines.md](docs/lab-machines.md) | the university's lab machines, step by step |
| [docs/arms.md](docs/arms.md) | joint angles, the hand to a place, control loops |
| [docs/at-once.md](docs/at-once.md) | several parts at the same time: the `at_once()` block |
| [docs/head.md](docs/head.md) | turning the head, nodding; the halo and the sonar lights |
| [docs/grippers.md](docs/grippers.md) | electric and suction grippers |
| [docs/cameras-and-screen.md](docs/cameras-and-screen.md) | pictures from the cameras and on the screen |
| [docs/buttons-and-lights.md](docs/buttons-and-lights.md) | navigators, buttons, lights, and how events work |
| [docs/sensors.md](docs/sensors.md) | sonar, hand distance sensors, accelerometers |
| [docs/io-reference.md](docs/io-reference.md) | every button, light and switch of the robot by name |
| [docs/trajectory-action.md](docs/trajectory-action.md) | whole paths from other ROS 2 software |
| [docs/rviz.md](docs/rviz.md) | the robot in 3D on your screen |
| [docs/moveit.md](docs/moveit.md) | MoveIt 2: motions planned around the robot itself and around things in the way |
| [docs/two-robots.md](docs/two-robots.md) | two robots at the same time, from one computer and one program |
| [docs/simulation.md](docs/simulation.md) | a simulated Baxter in Gazebo (this branch only, work in progress) |
| [docs/bridge-machine.md](docs/bridge-machine.md) | the bridge on a Windows laptop: WSL2 |
| [docs/own-machine.md](docs/own-machine.md) | programming the robot from another computer than the bridge's |
| [docs/mobility-base.md](docs/mobility-base.md) | the Dataspeed Mobility Base one robot stands on |
| [docs/own-project.md](docs/own-project.md) | a project of your own: folder layout, VS Code, extra Python packages |
| [docs/tools.md](docs/tools.md) | the command-line tools: enabling, tucking, cameras, self-test, calibration |
| [docs/troubleshooting.md](docs/troubleshooting.md) | what an error means and what to check |
| [examples/](examples/) | short programs, one per part of the robot |

## Status

Used with the lab's robot 1 (`011603p0009`, Baxter software 1.2.0.57) from a Windows laptop
with WSL2 and from the lab machines: the whole library, the command-line tools, RViz,
MoveIt and the Mobility Base. `CHANGELOG.md` lists what was tested for each release.

Limitations:

- MoveIt's motions run cleanly only at slow speeds (scaling 0.1).
- The trajectory action's velocity mode follows a path loosely.
- The bridge mode `on_demand` pauses all data once per second on the real robot; use the
  default, `fixed_list`.
- Not tested yet: robot 2 (`011602p0013`), two real robots at once, and the scripts
  `setup.sh` and `add_robot.sh` on a lab machine (they take the steps that were tested
  there by hand).

**This branch, `simulator`,** adds a simulated Baxter: Rethink's simulator in Gazebo, run
against the bridge like a real robot, with simulation time in the bridge, the trajectory
action server and MoveIt ([docs/simulation.md](docs/simulation.md)). It is a work in
progress and not part of a release; `main` has everything else.

## Layout

| Folder or file | Contents |
| --- | --- |
| `baxter_interface_ros2/` | the Python library `baxter_interface` for ROS 2, with its own README |
| `examples/` | short example programs |
| `docs/` | the guides listed above |
| `scripts/setup.sh`, `scripts/add_robot.sh`, `baxter.sh` | setting a computer up, writing a robot's settings, setting up a terminal (`setup.sh` links `baxter.sh` into `~/baxter_ws`) |
| `scripts/` | also: the robot check (`robot_check.sh`), the robot inventory, the Mobility Base check, the message port |
| `compose.yaml`, `.env.example` | the containers and their settings |
| `bridge_config/` | the bridge's configuration: which topics and services are bridged, ROS 2 network settings |
| `moveit_config/` | the MoveIt 2 configuration (ROS 2 package `baxter_moveit_config`), with Rethink's original in `reference/` |
| `robot_model/` | the ROS 2 package `baxter_description` for RViz and MoveIt; the model itself is downloaded by `fetch_model.sh` |
| `docker/` | the images: bridge, client (programs without ROS 2 installed), MoveIt, RViz, ROS 1 Noetic for the robot inventory |
| `msgs/ros1/` | Baxter's ROS 1 message packages, unchanged from Rethink's `baxter_common` v1.2.0 |
| `msgs/ros2/` | their ROS 2 versions, generated by `scripts/port_msgs_to_ros2.py` |
| `docs/robot-inventory/` | what the robot runs: topics, services, parameters, message definitions |
| `licenses/` | the licence texts of Rethink Robotics' and Dataspeed's parts (see `NOTICE`) |

Every change to version pins, messages or the topic list goes into `CHANGELOG.md`.
`CONTRIBUTING.md` has the steps of a release.

## License

Apache License 2.0 ([LICENSE](LICENSE)). Parts from Rethink Robotics and Dataspeed keep
their BSD licences; [NOTICE](NOTICE) lists them.

## About these documents

The documentation and guides in this repository were written with the help of Claude
(Anthropic) and checked against the code and against tests on the robots.
