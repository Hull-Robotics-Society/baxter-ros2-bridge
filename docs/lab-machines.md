# The university's lab machines

The lab's Windows computers come with WSL images from the university's WSL manager
(UoH-WSL), with ROS 2 Lyrical installed for teaching. This page takes such a machine from
a fresh WSL image to a moving robot, and explains what is different about these machines.

**Save your WSL image in the WSL manager before you log out.** Otherwise everything
below is gone the next time, and you start again at "Once per WSL image".

## Once per WSL image

In the WSL terminal:

```bash
git clone https://github.com/Hull-Robotics-Society/baxter-ros2-bridge.git ~/baxter-ros2-bridge
cd ~/baxter-ros2-bridge
bash scripts/setup.sh
```

`setup.sh` asks for your password once (for `sudo`). It downloads the bridge's ready-made
image, installs the few ROS packages that may be missing, and builds Baxter's messages,
the Python library, the robot model and the MoveIt configuration in `~/baxter_ws`. When it
is done it says so and names the next step. If it stops, it says why.

## At the start of every session

```bash
cd ~/baxter-ros2-bridge
bash scripts/add_robot.sh 011603p0009.hull.ac.uk     # robot 1; robot 2 is 011602p0013
docker compose up -d bridge
```

`add_robot.sh` looks up the robot's address and this machine's own address, and writes
both into `.env`. Run it every session: the machine's address can change from day to day
and from machine to machine. `docker compose up -d bridge` starts the bridge, or restarts
it with the new settings.

Then, in every terminal you use for Baxter:

```bash
source ~/baxter_ws/baxter.sh
```

and the robot is yours:

```bash
ros2 run baxter_interface tuck_arms -u               # arms out to the working pose
python3 ~/baxter-ros2-bridge/examples/01_read_robot.py
```

Someone at the e-stop whenever the robot is enabled, nobody within reach of the arms.
[quickstart.md](quickstart.md) continues from here, and [examples/](../examples/) has a
short program for every part of the robot.

### RViz and MoveIt

Both are installed on the lab machines; their windows open on the Windows desktop.

```bash
ros2 launch baxter_description view_robot.launch.py   # the robot in 3D (rviz.md)
ros2 launch baxter_moveit_config moveit.launch.py     # planned motions (moveit.md)
```

With Rethink's electric grippers mounted, add `LEFT_ELECTRIC_GRIPPER=1` and
`RIGHT_ELECTRIC_GRIPPER=1` to `.env` and source `baxter.sh` again, or start MoveIt with
`left_electric_gripper:=true right_electric_gripper:=true`. In RViz, keep the speed
scaling at 0.1: faster motions are stopped part of the way ([moveit.md](moveit.md),
"Speeds").

## At the end of a session

1. Stop your programs and MoveIt (Ctrl+C).
2. Fold the arms: `ros2 run baxter_interface tuck_arms -t`. It disables the robot at the
   end. Without tucking: `ros2 run baxter_interface enable_robot -d`.
3. Stop the bridge: `cd ~/baxter-ros2-bridge && docker compose --profile tools down`.
4. Shut the robot down with its power button and wait until it is off.
5. Switch off the robot's power: it starts by itself as soon as it has power again.
   Leave the e-stop released. A robot switched on with its e-stop pressed refuses to be
   enabled ("robot is still resetting").
6. Save the WSL image in the WSL manager, then log out.

## Several lab machines

Every lab machine's WSL is on the same university network. `add_robot.sh` therefore sets
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and `baxter.sh` takes it over: ROS 2 stays on
the machine, and nobody else's programs see your bridge, or yours theirs.

Two lab machines can still each run a bridge to the same robot, and both can then
command it. Agree who drives.

## Updating

```bash
cd ~/baxter-ros2-bridge
git pull
bash scripts/setup.sh
docker compose up -d bridge
```

## What is different about the lab machines

The scripts take care of all of this; it is here for when something does not work.

| | A laptop ([bridge-machine.md](bridge-machine.md)) | A lab machine |
| --- | --- | --- |
| WSL's network | mirrored: WSL has Windows' addresses | bridged: WSL has a university address of its own |
| Way to the robot | a cable straight to the robot | the university network, by the robot's university name |
| ROS 2 for your programs | the `client` container, or ROS 2 Jazzy | the machine's own ROS 2 Lyrical, with MoveIt and RViz |
| ROS 2 middleware in a new terminal | Fast DDS (ROS 2's default) | Cyclone DDS (set by the lab) |
| Rights | administrator on Windows | `sudo` inside WSL only |
| After logging out | everything stays | everything is gone unless the WSL image is saved |

What follows from that:

- **This machine's address (`ROS_IP`).** The robot connects back to the bridge to receive
  commands, at the address in `ROS_IP`. In bridged mode that is WSL's own university
  address, not Windows'. It may change between sessions, which is why `add_robot.sh` runs
  every time. If the bridge reads the robot but commands time out, run it again.
- **The robot's second name.** The robot's own programs announce themselves as
  `<serial>.local`, whatever name it was reached by. `add_robot.sh` writes that name into
  `.env` as `ROBOT_HOSTNAME_2`, and both names lead to the robot inside the containers.
- **No WSL network profile.** The laptop setup needs `DDS_PROFILE=wsl` for WSL's mirrored
  mode. In bridged mode that profile limits ROS 2 to an address that does not exist, the
  bridge reports "All whitelist interfaces were filtered out", and no program sees a
  Baxter topic. `add_robot.sh` sets it only in mirrored mode.
- **Fast DDS in your terminal.** The bridge uses Fast DDS, the lab's terminals Cyclone DDS.
  Topics work between the two, services do not: every call times out ("No answer from
  /cameras/list within 10 s"), and the bridge logs "Change payload size of '24' bytes is
  larger than the history payload size". `baxter.sh` switches the terminal it runs in to
  Fast DDS, and the library warns when a program runs with another middleware. Other
  terminals keep the lab's settings, for other ROS 2 work.
- **Domain 42.** The lab's terminals start in ROS 2 domain 0; the bridge uses 42.
  `baxter.sh` sets it from `.env`.
- **A Python folder only root can read.** In the lab's image the folder of one installed
  Python package (`/usr/local/lib/python3.14/dist-packages/uv-*.dist-info`) can be read by
  root only, and colcon stops on it with "Permission denied". `setup.sh` makes such
  folders readable (`sudo chmod -R a+rX`, which adds read rights only). Building with
  `sudo colcon build` instead leaves a workspace that belongs to root.
- **No firewall rules.** The laptop setup adds two Windows firewall rules, which need
  administrator rights. On the lab machines the robot reached the bridge without them.

## Limitations

- Not tested yet on a lab machine: the scripts themselves (`setup.sh`, `add_robot.sh`,
  `baxter.sh`; the steps they take were done by hand there), example 17, the robot check
  (`scripts/robot_check.sh`), robot 2, and two robots at once.
