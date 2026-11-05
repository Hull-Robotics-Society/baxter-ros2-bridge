# Changelog

Every change to version pins, message definitions, bridge mappings or the bridged topic
list goes here, newest first, with the reason.

## Unreleased

- Docs: untuck before enabling (an enabled tucked robot pushes its arms out), the robot
  and the base share one e-stop, which stays released when switching off, and the base's
  motors only get power once its power button is pressed, also for the joystick. Removed
  the battery note from the Mobility Base limitations and the section on a newer system
  for the base's computer (not needed by the bridge or the libraries).

## 1.0.0 (4 Nov 2026)

The first public release.

### What it contains

- **The bridge** (`docker/bridge-jazzy/`): ros1_bridge in Docker, between the robot's
  ROS 1 Indigo (Baxter software 1.2.0) and ROS 2 Jazzy, with Baxter's and the Mobility
  Base's messages. It carries the topics and services listed in
  `bridge_config/topics.yaml` (`fixed_list` mode), the robot's software version and its
  own robot model. A guard restarts a bridge process that waits for good for a robot
  service. Robots can have names, so that several run in one ROS 2 system.
- **The Python library `baxter_interface`** for ROS 2: the classes of the Baxter SDK
  v0.7.0, plus `setup()`, `at_once()`, hand moves by place, waypoints in one motion,
  sensors, head lights, cameras and the screen with OpenCV, events, `MobilityBase`, and
  `MotionPlanner` for MoveIt. The SDK's command-line tools, and its joint trajectory
  action server with its three modes.
- **MoveIt 2** (`moveit_config/`): Rethink's original configuration taken over file by
  file, with the robot's own robot model and Rethink's electric grippers. Differences
  from the original are listed in `docs/moveit.md`.
- **RViz** with the robot model (`robot_model/`), for one robot or several.
- **Setup:** `scripts/setup.sh` sets a computer up (Docker, the ready-made images, the
  ROS 2 workspace), `scripts/add_robot.sh` writes a robot's settings, `baxter.sh` sets up a
  terminal. Works with an installed ROS 2 (Jazzy, Lyrical) or, without one, with the
  `client`, `rviz` and `moveit` containers.
- **Ready-made images** on ghcr.io, built by GitHub Actions for each release.
- **The robot check** (`scripts/robot_check.sh`): checks the bridge and the library
  against the real robot, stage by stage.
- Guides for every part of the robot (`docs/`), 17 examples and a project template for
  VS Code.

### Tested

- Robot 1 (`011603p0009`, Baxter software 1.2.0.57) from a Windows laptop with the bridge
  in WSL2: the whole robot check (56 checks: reading, enabling, arms, hand moves,
  waypoints, `at_once()`, the trajectory action in its three modes, head, screen,
  electric grippers, cameras, navigators, head lights, sensors, the SDK's smoke tests).
- MoveIt on robot 1, with and without electric grippers: planned motions from RViz, run
  on the robot.
- The Mobility Base under robot 2: its bridge, `base_status`, and driving with
  `MobilityBase`.
- A university lab machine (Ubuntu 26.04, ROS 2 Lyrical, MoveIt 2.15) with robot 1: the
  bridge from the ready-made image, the examples, RViz and MoveIt from the machine's own
  ROS 2, and two lab machines kept apart with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- Two robots at once, with two stand-ins for the robots.
- The setup scripts, in a ROS 2 Lyrical container made like a lab machine. 153 unit tests.

### Known limitations

- MoveIt's motions run cleanly only at slow speeds (scaling 0.1).
- The trajectory action's velocity mode follows a path loosely.
- The bridge mode `on_demand` pauses all data once per second on the real robot; use
  `fixed_list`.
- Not tested yet: robot 2 itself, two real robots at once, the setup scripts on a lab
  machine, and the parts each guide lists under "Limitations".
