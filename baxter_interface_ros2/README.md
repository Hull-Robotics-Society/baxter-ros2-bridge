# baxter_interface (ROS 2)

Python interface to Baxter for ROS 2 (Jazzy, Lyrical), through the bridge. Same classes and methods as
the original Baxter SDK's `baxter_interface` v0.7.0, so existing Baxter code ports easily:

```python
from baxter_interface import Limb, RobotEnable

RobotEnable().enable()
left = Limb('left')
print(left.joint_angles())
left.move_to_neutral()
```

**Guides for each part of the robot are in [`docs/`](../docs/quickstart.md), example
programs in [`examples/`](../examples/).** This README describes how the library is built
and what it promises.

No `rclpy.init()` or node is needed: all classes share one ROS 2 node that runs in a
background thread (see `baxter_interface/_core/node.py` for why). To use your own node instead,
pass it in: `Limb('left', node=my_node)`; you then spin it yourself.

## Threads and callbacks

You can connect your own functions to events, for example a navigator button:

```python
def on_button(pressed):
    if pressed:
        limb.move_to_neutral()

navigator.button0_changed.connect(on_button)
```

- Each connected function runs in **a thread of its own**. It may take as long as it likes,
  move the robot, or wait for it; nothing else is held up.
- The calls of one function run in order, one at a time.
- Two *different* connected functions can run at the same moment. The library keeps the
  robot safe from that: blocking commands to the same arm, gripper, head or camera run one
  after the other. Your own variables are yours to protect: if two functions change the
  same variable, use a `threading.Lock`.
- An error in a connected function is printed, and later calls still run.
- Streaming commands (`set_joint_positions`, `set_joint_velocities`, `set_joint_torques`)
  are *not* serialised, since they are meant for your own control loop. Don't stream to
  the same arm from two threads.
- Connected functions only run while your program runs. When the main program reaches its
  end, the program closes, and a connected function that is still busy is abandoned
  (quietly). A program that only reacts to events needs something that keeps it alive at
  the end, for example `input('Press Enter to quit')`.

Details: `_core/node.py` (which threads exist), `_core/callbacks.py`, `_core/locks.py`.

## Starting a program: `setup()`

Every part of the robot you create waits until the bridge has connected it, which takes a
second or two, because the bridge looks for new topics only about once per second.
Created one after the other, five parts wait five times. Create them in a `setup()` block
instead, and the waiting happens once, at the end of the block:

```python
import baxter_interface
from baxter_interface import Head, Limb, Navigator, RobotEnable

with baxter_interface.setup():
    robot = RobotEnable()
    left = Limb('left')
    right = Limb('right')
    head = Head()
    navigator = Navigator('left')

# all five are connected here
robot.enable()
left.move_to_neutral()
```

Inside the block the parts are not ready yet: use them only after it. If one of them
cannot connect, the error is raised at the end of the block and names the part. Without a
block everything works as well, only slower to start.

## What the library promises about the connection

Between your program and the robot sits the bridge, and a bridge can be slow to connect,
stopped, or cut off from the robot. The library is written so that your program finds
out, instead of carrying on as if a command had arrived or a value were current:

- **Creating an object waits for the robot.** `Limb('left')`, `Head()`, `Navigator(...)`
  and the others return only once the first state has arrived, and `Limb` also once its
  commands can reach the robot. If that doesn't happen within 10 seconds they raise an
  error that says what is missing.
- **A command that is sent once is confirmed.** A speed setting, a light, a head or
  gripper command, a picture for the screen: the library waits until the bridge listens,
  sends, and waits for the bridge to confirm that it received the message. If either
  fails you get a `ConnectionError`, not silence.
- **A command that waits for the robot notices if the robot goes quiet.** While
  `move_to_joint_positions()`, `set_pan()` or a blocking gripper command waits, it checks
  that the robot is still reporting. After one second without news it stops and raises a
  `ConnectionError`; the arm then holds its position.
- **Values you read are the newest ones received.** `limb.joint_angles()` returns at
  once and cannot know that the connection was lost a moment ago. In a control loop of
  your own, ask `limb.joint_state_age()`: normally about 0.01 seconds.

Two limits. The confirmation comes from the bridge, not from the robot: it says the
message reached the bridge, which passes it on over a normal network connection. And
streaming commands in a loop of your own (`set_joint_positions` and the like) are sent
without confirmation, because waiting would break the loop's timing; that is what
`joint_state_age()` is for.

The two time limits are `CONNECT_TIMEOUT` and `STATE_TIMEOUT` in `settings.py`.

## Pictures: cameras and the screen

```python
import cv2
from baxter_interface import Camera, Display

camera = Camera('left_hand')      # or 'right_hand', 'head'
camera.open()                     # 640 x 400; camera.open(resolution=(1280, 800)) for more
image = camera.frame()            # the newest picture
cv2.imwrite('photo.png', image)

display = Display()
display.show(image)               # on Baxter's screen, scaled to fit
display.show('hello.png')         # image files work too
```

A picture is an **OpenCV image**: a NumPy array of shape `(height, width, 3)` with blue,
green, red values from 0 to 255. Everything OpenCV offers works on it directly
(`cv2.cvtColor`, `cv2.rectangle`, `cv2.imwrite`, ...), and anything you produce with OpenCV
can go straight to `display.show()`.

To handle every picture as it arrives, connect a function. This shows the live camera
picture on the screen:

```python
camera.frame_received.connect(display.show)
input('Press Enter to quit')
```

- `camera.frame()` hands out each picture once. In a loop it waits for the next new one, so
  the loop runs at the camera's speed.
- A function connected to `frame_received` that is slower than the camera always gets the
  newest picture; the ones in between are skipped.
- Baxter powers only **two of its three cameras at a time**, normally the two hand
  cameras. A camera without power cannot be opened; closing a powered one hands its power
  to the third. To use the head camera: `Camera('head').open(replace='left_hand')`, and to
  go back: `Camera('left_hand').open(replace='head')`. `camera.has_power()` tells which
  state a camera is in, and `camera_control -l` shows all three.
- Exposure, gain and the other settings: `camera.controller` (a `CameraController`).
- The screen has 1024 x 600 pixels; other sizes are scaled, with black borders. A picture
  stays until the next one is shown. `display.clear()` makes the screen black.

## Layout

```
baxter_interface/
├── __init__.py        public API: from baxter_interface import Limb, Gripper, ...
├── settings.py        tolerances
├── robot_enable.py    RobotEnable
├── limb.py            Limb
├── gripper.py         Gripper
├── head.py            Head
├── camera.py          Camera (pictures), CameraController (settings), list_cameras, camera_status
├── display.py         Display (Baxter's screen)
├── sensors.py         Sonar, RangeSensor, Accelerometer
├── head_lights.py     HeadLights (the halo and the sonar lights on the head)
├── robust_controller.py   RobustController (runs the robot's maintenance routines)
├── info.py            robot_info(), software_version(): the robot's ROS 1 parameters
├── images/            the two images that the smoke test and show_image put on Baxter's screen
├── io/                DigitalIO, AnalogIO, Navigator
├── tools/             command-line tools (see below)
└── _core/             internal plumbing: shared node, callback threads, locks, wait_for, Signal, service calls, pictures (OpenCV), setup() and at_once() blocks, path arithmetic (own curve and the SDK's), PID
```

Always import from the top level (`from baxter_interface import Navigator`), not from the
subfolders: the folders may be rearranged, the top-level names stay.

## Compared with the SDK

Beyond the SDK's classes the library adds: `setup()`, `at_once()` (several parts at the
same time, `docs/at-once.md`), hand moves by place (`Limb.move_to_pose`, `move_by`,
`solve_pose`), `Limb.move_through` (several waypoints in one motion), the sensor classes,
`HeadLights`, `Camera` and `Display` with OpenCV images, and events (`Signal`). The SDK's joint
trajectory action server exists as a new ROS 2 program with the SDK's three modes
(`docs/trajectory-action.md`); its `gripper_action` and `head_action` servers are not
ported.

With the bridge in its default `fixed_list` mode, what the classes and tools need is
listed in `bridge_config/topics.yaml`; components with their own topics (each
digital/analog I/O beyond the navigator lights and cuff sensors) must be added there
first. In `on_demand` mode everything works without a list, but that mode pauses all
data once per second on the real robot.

Dependencies beyond ROS 2 itself: OpenCV (`python3-opencv`) and `cv_bridge`
(`ros-<distro>-cv-bridge`) for pictures, `control_msgs` (`ros-<distro>-control-msgs`) for
the joint trajectory action. `scripts/setup.sh` installs them where they are missing, and
all are in the client image.

**Robot software version.** The classes and methods are those of the Baxter SDK v0.7.0;
the messages and topic names underneath are those of robot software 1.2.0, which both lab
robots run. Where the two differ, the SDK v0.7.0 interface is kept:
`head.set_pan(angle, speed=100)` takes 0 to 100, and `wheel_changed` passes the wheel
position.

Differences from the ROS 1 SDK: no `rospy.init_node()`; timeouts raise `TimeoutError`
(was `OSError`); a command that could not be delivered or a robot that stopped reporting
raises `ConnectionError` (the SDK carried on); waits after start-up are up to 10 s instead of 1 to 2 s, since ROS 2 discovery
through the bridge takes longer.

## Command-line tools

What each tool does and how to use it: [`docs/tools.md`](../docs/tools.md). In short:

```bash
ros2 run baxter_interface enable_robot -s    # state (-e enable, -d disable, -r reset, -S stop)
ros2 run baxter_interface robot_info         # software version and other ROS 1 parameters of the robot
ros2 run baxter_interface move_to_neutral    # both arms to the neutral pose
ros2 run baxter_interface tuck_arms -u       # untuck (-t tuck; --timeout N to give up after N s)
ros2 run baxter_interface camera_control -l  # list cameras and which are open (-o NAME, -c NAME, -e)
ros2 run baxter_interface show_image FILE    # picture on Baxter's screen (no FILE: the SDK picture; --black)
ros2 run baxter_interface smoke_test         # self-test of the robot (-t NAME one test, -p continue on failure)
ros2 run baxter_interface calibrate_arm -l left   # arm calibration: changes the robot's calibration
ros2 run baxter_interface tare -l left            # zero the arm's force sensing: same warning
ros2 run baxter_interface joint_trajectory_action_server   # plays paths sent by other ROS 2 programs (-l left; -m position_w_id or velocity; --curve sdk)
```

These are ports of the Baxter SDK's `baxter_tools` (v0.7.0), except `robot_info`,
`show_image` and `joint_trajectory_action_server`, which are new. Not ported:
`update_robot` (installs robot software from a USB stick).

`smoke_test`, `calibrate_arm` and `tare` move the robot by themselves. Clear the area and
have someone at the e-stop. `calibrate_arm` and `tare` also change the robot's stored
calibration and need the gripper removed; they ask for confirmation first.

## Type annotations

Every class, method and function that programs use says what it takes and what it
returns (`def move_by(self, x: float = 0.0, ...) -> None`), and the package is marked as
typed (`py.typed`). Editors such as VS Code use this for completion and to mark mistakes
before a program runs. Checked with pyright, the engine behind VS Code's Pylance, in its
"basic" mode: no findings in the library, the examples, the project template and the
robot check. The command-line tools in `tools/` are annotated only where the checker
needed it.

Three things follow from it:

- A part used before it is connected (only possible inside a `setup()` block) raises a
  `ConnectionError` that says so, instead of an error about `None`.
- `Limb.Point`, `Limb.Quaternion`, `Sonar.Point`, `Accelerometer.Vector` and
  `HeadLights.Halo` are small classes with named number fields. They still behave like
  the tuples they were.
- A signal says what it hands to connected functions: `Signal[bool]` for a button.

## Developing

From the repository root, with the bridge running:

```bash
docker compose run --rm dev
ros2 run baxter_interface enable_robot -s
python3 -m pytest src/baxter_interface_ros2/test -q
```

The client image comes with the library installed from a copy of this folder. The `dev`
shell mounts the repository's folder over that copy, and the installation links to the
Python files instead of copying them (`--symlink-install`), so edits take effect at once.
Build again only after adding a command-line tool or changing `setup.py`:
`colcon build --symlink-install && source install/setup.bash`. After library changes that
other people's programs should get, rebuild the image: `docker compose build client`.

On a computer with ROS 2 installed, the workspace `scripts/setup.sh` builds links to this
folder in the same way: edits take effect at once in every terminal set up with
`baxter.sh`, and `python3 -m pytest baxter_interface_ros2/test -q` from the repository
root runs the tests.

To check the library against the robot, stage by stage: `bash scripts/robot_check.sh` from
the repository root (see the repository's README). The unit tests in `test/`
need no robot.

## License

BSD-3-Clause. Ported from Rethink Robotics' baxter_interface; see `LICENSE`.
