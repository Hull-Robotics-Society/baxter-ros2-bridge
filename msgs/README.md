# The robots' messages

```
msgs/
├── ros1/   ROS 1 message packages, unchanged from their makers: Baxter's from Rethink's
│           baxter_common v1.2.0, the Mobility Base's from Dataspeed's SDK 1.0.6
└── ros2/   their ROS 2 versions (generated)
```

The bridge can only carry a message if its definition is **exactly** the robot's: ROS 1
checks a checksum of the definition on every connection. Both lab robots run software
1.2.0, and the robot inventory (`docs/robot-inventory/`) confirmed that all recorded
Baxter message types match these files.

`ros1/baxter_*` comes from github.com/RethinkRobotics/baxter_common, tag v1.2.0 (commit
6c4b0f3). `ros1/mobility_base_core_msgs` comes from
bitbucket.org/DataspeedInc/mobility_base_ros, version 1.0.6 (commit b078a15); the lab's
base runs software 1.0.3, and its inventory confirmed that the four message types it
sent have the same checksums as these files.

## The ROS 2 versions are generated

**Do not edit `ros2` by hand.** It is generated from `ros1`:

```bash
python3 scripts/port_msgs_to_ros2.py
```

What the script changes, and why:

| ROS 1 | ROS 2 | Why |
| --- | --- | --- |
| `bool isTurning` | `bool is_turning` | ROS 2 field names must be lowercase snake_case |
| `time timestamp` | `builtin_interfaces/Time timestamp` | ROS 2 has no built-in `time` type |
| `Header header` | `std_msgs/Header header` | ROS 2 needs the full type name |
| `string CMD_GRIP = grip` | `string CMD_GRIP="grip"` | ROS 2 string constants are quoted |

Each changed file starts with a comment listing its changes.

`mapping_rules.yaml` tells ros1_bridge which ROS 1 field belongs to which ROS 2 field, for every
message or service with a renamed field. A rule lists all fields of that message: the bridge copies
only the fields a rule lists. The other two kinds of change need no rule; the bridge handles them.

To build and inspect them in a ROS 2 Jazzy container, from the repository root:

```bash
docker run -it --rm -v "$PWD/msgs/ros2:/ws/src" ros:jazzy-ros-base bash
cd /ws && colcon build && source install/setup.bash
ros2 interface show baxter_core_msgs/msg/HeadState
```

## A robot with another software version

`docker compose run --rm inventory` records a robot's exact definitions in
`docs/robot-inventory/.../definitions/` and compares their checksums with `ros1`. If any
Baxter type differs, replace the files in `ros1` with that version's (from the matching
tag of `baxter_common`, or from the recorded definitions), run the port script, and
rebuild all images. Where the newer software changed messages the library uses (head,
navigators), the library needs the matching change too.
