# The bridge (ROS 1 ↔ ROS 2)

This image connects the robot's ROS 1 world to ROS 2 Jazzy. Lab members only ever see the
ROS 2 side.

## What happens inside

```
robot (ROS 1 master)  <--ROS 1-->  [ parameter_bridge ]  <--ROS 2-->  lab machines
                                   converts each message
                                   ROS 1 type <-> ROS 2 type
```

1. **Both ROS versions in one image.** ros1_bridge is a program that is a ROS 1 node and a
   ROS 2 node at the same time, so it needs both installed side by side. ROS 2 Jazzy comes
   from the base image; ROS 1 Noetic from a community PPA that rebuilt it for Ubuntu 24.04.
2. **Both versions of Baxter's messages.** `msgs/ros1` is built with Noetic, `msgs/ros2`
   with Jazzy, each in its own workspace. They are those of robot software 1.2.0.
3. **The bridge is compiled against both.** At build time ros1_bridge looks for every message
   type that exists on both sides (same package and message name, or a rule in
   `mapping_rules.yaml`) and generates a converter for each pair. This is the slow step, and
   why the bridge must be rebuilt whenever a message definition changes.
4. **At start-up** (`run_bridge.sh`) the bridge runs in one of two modes, chosen with
   `BRIDGE_MODE` in `.env`:
   - **`fixed_list`** (default): ros1_bridge's `parameter_bridge` with the lists from
     `bridge_config/topics.yaml`, carried all the time. It runs as **three processes**:
     one for the services, one for the topics that carry commands to the robot (marked
     `direction: to_robot`), one for everything else. One process alone serves everything
     in a single loop and could not keep up with the robot's roughly 60 listed topics at
     100 messages per second: service calls and commands were delayed or lost.
   - **`on_demand`**: ros1_bridge's `dynamic_bridge`. Once per second it looks at
     both sides and connects a topic when one side publishes it and the other listens,
     and disconnects it when that ends. The robot's services are made available in ROS 2
     automatically. Everything the robot offers is reachable (as far as a message type
     for it exists on both sides) without a list. **Not for moving the robot:** see the
     measurements below. Also, ROS 2 command-line tools that first wait for a topic to
     exist (`ros2 topic hz`, `ros2 topic echo` without a type) wait forever, because the
     topic only appears once something listens.

   Measured on robot 1 (bridge in WSL2, both hand cameras streaming):

   | | `fixed_list` | `on_demand` |
   | --- | --- | --- |
   | joint states arriving per second (the robot sends 100) | 105 | 67 to 83 |
   | longest gap between joint states | 17 ms | 330 to 500 ms |
   | longest gap between camera pictures (the robot: 73 ms) | 78 ms | about 520 ms |
   | furthest from the path, trajectory action in position mode | 0.044 rad | 0.184 rad |

   The reason: the on-demand bridge's once-per-second look at the robot (271 topics, 89
   services) takes 0.3 to 0.45 s, and it carries nothing meanwhile. With a small ROS 1
   system on the other side the look takes no time. Changing this would take a change
   to ros1_bridge itself.
5. **Services in `fixed_list` mode.** The robot's ROS 1 services go under `services_2_to_1` in `topics.yaml`
   (a ROS 2 program calls, the robot answers). Before the bridge starts,
   `existing_services.py` drops the ones the ROS 1 side doesn't provide: calling a bridged
   service whose ROS 1 side is missing would stop the services bridge. A service that appears
   later needs a bridge restart. Never put a robot service under `services_1_to_2`: that
   direction registers the service on the ROS 1 master and replaces the robot's own.
6. **Robot information.** ros1_bridge carries topics and services, but not ROS 1
   parameters. `robot_info.py` runs next to the bridge, reads the parameters listed under
   `robot_info_params` in `bridge_config/topics.yaml` (e.g. the robot's software version),
   and publishes them on `/baxter_bridge/robot_info`, which the bridge carries to ROS 2.
   Read them with `ros2 run baxter_interface robot_info`.
   `robot_model.py` does the same for the robot's own robot model (the parameter
   `/robot_description`, with the arms' mounting points measured for each robot): it
   publishes it directly in ROS 2 as `robot/robot_description` (with a robot name
   `/<name>/robot/robot_description`), kept for programs that start later. MoveIt plans
   with it ([docs/moveit.md](../../docs/moveit.md#the-robot-model)).

7. **The guard.** A robot service that fails makes ros1_bridge stop; `run_bridge.sh`
   starts it again. A robot service that *never answers* makes it wait for good, and it
   then answers no other service call (in `on_demand` mode it also connects no more
   topics). `bridge_guard.py` notices that: every few seconds it calls
   `/baxter_bridge/ping`, a service inside this container that answers at once, through
   the bridge. After 30 s without an answer it stops the stuck process, which is started
   again. In `fixed_list` mode that is only the services process; topics keep flowing.
   The log then says `bridge_guard: no answer through the bridge ...`. If the same call
   hangs again, the program on the robot that provides the service is stuck.
   `BRIDGE_GUARD=0` in `.env` switches the guard off.

## Use

See the repository's README. In short, from the repository root:

```bash
bash scripts/add_robot.sh <the robot's name>   # writes .env
docker compose up -d bridge        # downloads the ready-made image the first time
bash scripts/robot_check.sh        # checks everything against the robot
```

`docker compose stop bridge` stops it. `docker compose build bridge` builds the image
here instead of downloading it, which takes about half an hour.

## Changing what is bridged

In `fixed_list` mode, edit `bridge_config/topics.yaml` and restart the bridge; no rebuild
needed. In `on_demand` mode there is nothing to change. That includes the
`robot_info_params` list. A rebuild is only needed when message definitions in `msgs/` change,
or the scripts in this folder (a quick rebuild: only the last step is redone).

## Pinned sources

| What | Pin | Why it matters |
| --- | --- | --- |
| Base image | `ros:jazzy-ros-base-noble` | not pinned by digest: a rebuild gets the newest Jazzy base |
| ROS 1 Noetic | `ppa:ros-for-jammy/noble` | community PPA, amd64 only; if it disappears, build Noetic from source |
| ros1_bridge | `smith-doug/ros1_bridge` commit `8f1b7e9` | fork with action support, used by the community builder |
| Build recipe | `TommyChangUMD/ros-jazzy-ros1-bridge-builder` commit `e05ca56` | what this Dockerfile is based on |
