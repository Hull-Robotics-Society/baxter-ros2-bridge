# Two robots (or more) at the same time

Each Baxter is a ROS 1 system of its own, and all Baxters use the same topic names. To
work with several from one computer, every robot gets its own bridge and a name, and its
bridge puts that name in front of everything of the robot in ROS 2:

```
Baxter "one" (ROS 1)  <->  bridge one  <->  /one/robot/joint_states, /one/cameras/..., ...
Baxter "two" (ROS 1)  <->  bridge two  <->  /two/robot/joint_states, /two/cameras/..., ...

your program:  Robot('one') uses the names starting with /one,  Robot('two') those with /two
```

This is the usual way to keep robots apart in ROS 2: a "namespace" per robot, all of them
in one ROS 2 system, where one program or tool can see them together. On the robots
themselves nothing changes.

A robot without a name keeps the plain names (`/robot/joint_states`), and everything in
the other pages applies to it unchanged. That is the case for a single robot.

## Settings: one file per robot

In the repository's folder on the bridge computer, write one settings file per robot
with `add_robot.sh`, giving each robot a name:

```bash
bash scripts/add_robot.sh --file .env.one --name one 011603p0009.hull.ac.uk
bash scripts/add_robot.sh --file .env.two --name two 011602p0013.hull.ac.uk
```

Besides the robot's address and this computer's, each file then has:

| Setting | Value |
| --- | --- |
| `ROBOT_NAME` | **a different name for every robot**, for example `one` and `two`. Letters, digits and single underscores, starting with a letter |
| `COMPOSE_PROJECT_NAME` | `baxter-one`, `baxter-two`: lets the two bridges run next to each other |
| `ROS_DOMAIN_ID` | the same number in all files (42) |
| `BRIDGE_MODE` | `fixed_list`. A bridge with a name does not start in `on_demand` mode |

Add `ROBOT_POSITION=x y z turn` by hand for pictures of all robots together: where the
robot stands in the room, in metres and degrees ([rviz.md](rviz.md)). Without it every
robot stands at `0 0 0 0`.

Use the robot's name for the file too (`.env.one` for `ROBOT_NAME=one`); it is the
`ROBOT_NAME` line that counts. `.env.robot.example` shows all settings of such a file.

With a single robot nothing changes: its settings stay in `.env`, without `ROBOT_NAME`,
and no command needs `--env-file`.

## Start and stop

```bash
docker compose --env-file .env.one up -d bridge
docker compose --env-file .env.two up -d bridge
docker compose --env-file .env.one ps            # "baxter-one-bridge-1"
```

The bridge's log says `Robot name: one` and how many names got the prefix.

```bash
docker compose --env-file .env.one stop bridge
docker compose --env-file .env.two stop bridge
```

## A program for one of the robots

A program written for a single robot runs on either robot unchanged. It is told at its
start which robot it is for:

```bash
BAXTER_ROBOT=two python3 examples/01_read_robot.py
BAXTER_ROBOT=two ros2 run baxter_interface enable_robot -s
```

In the `client` container the settings file does it: the container passes its
`ROBOT_NAME` on to the programs as `BAXTER_ROBOT`
(`docker compose --env-file .env.two run --rm client python3 examples/01_read_robot.py`).
The two ways to name the robot, wherever a program runs:

| Way | Example |
| --- | --- |
| the variable `BAXTER_ROBOT` | `BAXTER_ROBOT=two python3 my_program.py` |
| the ROS 2 namespace argument, as for any ROS 2 program | `python3 my_program.py --ros-args -r __ns:=/two` |

If both are given, the argument wins. Without either, the program means the robot
without a name.

(The examples that take the arm's side as their first argument need it before the ROS 2
argument: `python3 examples/02_move_arm.py left --ros-args -r __ns:=/two`.)

In a VS Code project, add `"BAXTER_ROBOT": "two"` to `"containerEnv"` in
`.devcontainer/devcontainer.json` and rebuild the container.

## A program for both robots

```python
import baxter_interface

one = baxter_interface.Robot('one')
two = baxter_interface.Robot('two')

one.enable()
two.enable()

with baxter_interface.setup():              # connect all parts together
    left_one = one.Limb('left')
    left_two = two.Limb('left')
    head_two = two.Head()

with baxter_interface.at_once():            # both robots move at the same time
    left_one.move_to_neutral()
    left_two.move_to_neutral()
    head_two.set_pan(0.3)
```

A `Robot` makes every part the library has, with the same arguments as the class of that
name: `Limb`, `Gripper`, `Head`, `HeadLights`, `Camera`, `Display`, `Navigator`,
`DigitalIO`, `AnalogIO`, `Sonar`, `RangeSensor`, `Accelerometer`. It also has `enable()`,
`disable()`, `reset()`, `stop()`, `state()`, `software_version()` and `list_cameras()`,
and the attributes `name` and `namespace` (`'/one'`).

- `Robot('one')` looks for that robot for up to 5 seconds. If its bridge is not running,
  or the name is misspelled, it raises a `ConnectionError` that lists the robots found.
- `baxter_interface.known_robots()` returns the names of the robots whose bridges are
  running right now, for example `['one', 'two']`.
- `Robot()` without a name, and the plain classes (`Limb('left')`), mean the robot the
  program was started for (previous section). `Robot('')` is the robot without a name.
- `setup()`, `at_once()` and events work across robots. Commands to the same part of the
  same robot still run one after the other; the left arms of two robots are different
  parts.

```bash
python3 examples/15_two_robots.py
```

## Your own ROS 2 nodes and the `ros2` tools

Everything of a robot is found under its name:

```bash
ros2 topic list | grep ^/one/
ros2 topic hz /one/robot/joint_states
```

The library uses names without a leading slash (`robot/joint_states`), which ROS 2
completes with the namespace of the node. A node of your own that does the same follows
the namespace argument too, as example 12 does:

```bash
python3 examples/12_trajectory_client.py left --ros-args -r __ns:=/one
```

`BAXTER_ROBOT` is this library's own setting: it applies to the library's node and
tools, not to nodes you create yourself. A node of your own can follow it as example 12
does: `Node('my_node', namespace=os.environ.get('BAXTER_ROBOT', ''))`.

## Connecting two robots to one computer

- **Through the lab network:** both robots and the computer on it; `ROS_IP` is the same
  in both files.
- **Through a small switch:** computer and both robots on the switch, the computer's
  wired connection set to "Link-Local Only" as for one robot on a cable. Each robot
  gives itself a `169.254.x.x` address; write each file with the robot's `.local` name
  (in WSL with `ROBOT_IP=`, as in [bridge-machine.md](bridge-machine.md)).
- A robot's link-local address can change when it restarts. Run `add_robot.sh` for it
  again then.

## What to know

- **RViz** shows one named robot or all robots in one picture, see
  [rviz.md](rviz.md#a-robot-with-a-name-and-several-robots-in-one-picture).
- **Frame names inside the robot's own messages have no robot name.** A camera picture
  from robot "two" says it was taken in the frame `left_hand_camera`, not
  `two/left_hand_camera`. The frames of the robot models in RViz do have the name.
- Two bridges are twice the load of one on the bridge machine.
- A program for one robot, started for the other robot, moves the other robot: check
  the file name or the name before a command that moves something.
- One robot may stay without a name next to named ones; `Robot('')` and the plain
  classes reach it.

## How the bridge does it

The bridge program (ros1_bridge) uses one name per topic for both sides and cannot
rename. With `ROBOT_NAME` set, `docker/bridge-jazzy/robot_prefix.py` writes the names
with the prefix into the lists the bridge reads, which is what ROS 2 gets, and
`run_bridge.sh` starts the bridge with one ROS 1 "remapping" argument per name
(`/one/robot/state:=/robot/state`). Only the bridge's ROS 1 half reads these, and uses
the robot's real name again. The bridge program itself is unchanged. `on_demand` mode
has no list to rename, which is why a named robot needs `fixed_list`.

## Limitations

- Not tested yet with two real robots, nor from a VS Code project. Everything on this
  page ran with two stand-ins for the robots (two ROS 1 systems with Baxter's topic
  names, each with its own bridge).
