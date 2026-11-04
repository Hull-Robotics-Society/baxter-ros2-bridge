# Troubleshooting

Start with the robot check: `bash scripts/robot_check.sh --only network,bridge,read`
reads only, and tells in plain lines what works and what does not.

## A program does not start

| What you see | Likely cause | What to do |
| --- | --- | --- |
| `TimeoutError: Failed to get robot state on /robot/state` (or joint states, head state, ...) after 10 s | the program does not reach the robot through the bridge | the next table |
| `ModuleNotFoundError: No module named 'baxter_interface'` | ROS 2 and the workspace are not set up in this terminal | `source ~/baxter_ws/baxter.sh` (in the `client` container this cannot happen) |
| `ModuleNotFoundError: No module named 'baxter_core_msgs'` | Baxter's messages are not built or not sourced | the same; if it stays, run `bash scripts/setup.sh` again |
| `setup.sh` or `colcon build` stops with `PermissionError: [Errno 13] Permission denied: '/usr/local/lib/python3.../dist-packages/...'` | a Python package folder of the machine can be read by root only (the lab's WSL image: `uv-*.dist-info`) | `setup.sh` makes such folders readable; by hand: `sudo chmod -R a+rX /usr/local/lib/python3.14/dist-packages` (your Python version in the path). Not `sudo colcon build`: the workspace then belongs to root ([lab-machines.md](lab-machines.md)) |

## The program does not reach the robot

Work from the robot outwards:

| Check | Command | If it fails |
| --- | --- | --- |
| Is the robot up? | its screen shows the Rethink picture | booting takes a few minutes |
| Does the bridge machine reach it? | `ping <ROBOT_IP>` on the bridge machine | cable, wrong network, VM adapter not bridged |
| Is the bridge running? | `docker compose ps` | `docker compose up -d bridge` |
| What does the bridge say? | `docker compose logs --tail 30 bridge` | lines about not reaching the ROS master: `ROS_MASTER_URI` or the robot's address in `.env` |
| Was the robot restarted? | | on a direct cable it may have a new address: `bash scripts/add_robot.sh <the robot's name>` again, then `docker compose up -d bridge`. Otherwise `docker compose restart bridge` |
| Same ROS 2 domain? | `echo $ROS_DOMAIN_ID` where the program runs | must equal `ROS_DOMAIN_ID` in the bridge's `.env` |
| Program on another computer? | `ros2 node list` shows `/ros_bridge`? | [own-machine.md](own-machine.md), "If the two computers do not see each other" |
| Is the bridge machine WSL2? | `wslinfo --networking-mode` and `grep DDS_PROFILE .env` | mirrored mode needs `DDS_PROFILE=wsl` ([bridge-machine.md](bridge-machine.md)); NAT and bridged mode (the lab machines) need the line left out ([lab-machines.md](lab-machines.md)) |

## Reading works, commands do not

| What you see | Likely cause |
| --- | --- |
| reading works, `enable()` times out | `ROS_IP` in `.env` is not the address the robot can reach the bridge machine at (the robot connects *back* to receive commands); or the e-stop is pressed; or the next line |
| `enable()` times out although `ROS_IP` is right and the e-stop is released | the robot refuses: it was switched on while its e-stop was pressed, and then answers every enable request with "robot is still resetting" in its own log. Shut it down with its power button, release the e-stop, start it again. So leave the e-stop released when you switch the robot off |
| a head command times out | the robot is not enabled |
| a command that worked a moment ago does nothing, once in a while | `BRIDGE_MODE=on_demand`: that mode pauses all data once per second on the real robot. Use `fixed_list` |
| `enable()` works, the arm does not move, `TimeoutError` after 15 s | the robot was disabled again (e-stop, or a collision was detected): `ros2 run baxter_interface enable_robot -s`, then `-r` to reset and `-e` to enable |
| `ConnectionError: ... nothing listens on ...` | the bridge is not carrying that topic: bridge stopped, or `BRIDGE_MODE=fixed_list` and the topic is not in `bridge_config/topics.yaml` |
| `ConnectionError: No state from the left arm for 1.2 s` | the connection broke during a move: bridge stopped, cable out, robot off. The arm holds its position |
| reading and topics work, but every service call times out (`TimeoutError: No answer from /ExternalTools/.../IKService within 10 s`, `No answer from /cameras/list`), and the bridge logs `Change payload size of '24' bytes is larger than the history payload size` | your shell uses another ROS 2 middleware than the bridge. The lab's shells use Cyclone DDS, the bridge Fast DDS; topics work across the two, services do not: `source ~/baxter_ws/baxter.sh` switches the terminal to Fast DDS ([lab-machines.md](lab-machines.md)) |
| `ValueError: The left arm cannot reach ...` | the place is too far, too close to the body, or the wrist cannot turn that way; nothing moved. Try a place nearer to the current one, or leave out `orientation` |
| the arm trembles or fights a `move_...` command | the joint trajectory action server is running and still holding that arm at the end of its last path; stop the server (Ctrl+C) |
| `TimeoutError: ... fell behind its path` in `move_through()` | the arm is held back, or the path is faster than the arm may go: lower `speed=`, or raise `set_joint_position_speed()` |
| an arm move is refused or stops short | the target is beyond a joint's range, or the robot's collision avoidance stopped the arm before it touched the other arm or the body |

## Cameras

| What you see | Cause |
| --- | --- |
| `... cannot be opened: Baxter runs two of its three cameras at a time` | two other cameras are open; open with `replace=...` ([cameras-and-screen.md](cameras-and-screen.md)) |
| a camera is "not offered" in `camera_control -l` | the robot lists only two at times; close one of the others to use it |
| `No answer from /cameras/...`, and no pictures from any camera | the robot's camera program has stopped answering (it does when asked for a third camera). The bridge restarts its stuck part by itself within a minute (`docker compose logs bridge` shows `bridge_guard: ...`); if `camera_control -l` still gets no answer after that, restart the robot |
| no pictures after `open()` | another program changed the camera in the meantime; `camera_control -l` shows the state, `camera_control -c NAME` then `-o NAME` reopens |

## The robot itself

| What you see | What to do |
| --- | --- |
| the arms are limp and commands have no effect | e-stop pressed or robot in an error state: release the e-stop, `enable_robot -r`, `enable_robot -e` |
| an arm moves oddly after someone leaned on it | `ros2 run baxter_interface tare -l left` re-zeroes the force sensing (the arm moves; it asks first) |
| the gripper ignores commands | not calibrated (`gripper.calibrate()`) or in an error state (`gripper.reset()`) |

## Docker

| What you see | What to do |
| --- | --- |
| files a program wrote belong to `root` | programs in a container run as root; `sudo chown -R $USER <folder>` |
| `docker compose build` builds only the bridge | the other images are tools: `docker compose --profile tools build` |
| RViz: `could not connect to display` | `xhost +local:docker` in a terminal on the desktop, see [rviz.md](rviz.md) |
| a build recompiles the bridge for half an hour | a file under `msgs/` or `docker/bridge-jazzy/` changed; expected then, otherwise check `git status` |
| a change to `.env` has no effect | the bridge keeps the settings it was started with: `docker compose up -d bridge` (it starts again with the new ones) |

## Everything is slow or uneven

Arm motion that stutters, a `ConnectionError` although nothing was unplugged, tests that
fail one time and pass the next. Two known causes:

- `BRIDGE_MODE=on_demand` in `.env`. On the real robot that mode pauses all data for
  about 0.4 s once per second; `fixed_list` does not (`docker/bridge-jazzy/README.md`).
- The machine the bridge runs on stands still for moments. A VirtualBox VM on Windows 11
  does that; [bridge-machine.md](bridge-machine.md) has the measurements, and WSL2 as
  the better way to run the bridge.

## Still stuck

`bash scripts/robot_check.sh` runs everything stage by stage and saves the full output of
every check in `robot_check_logs/<date_time>/`. That folder is what to show someone who
helps you.
