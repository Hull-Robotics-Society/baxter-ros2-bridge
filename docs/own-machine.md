# Working from another computer

The bridge runs on one computer, the one with the cable (or network path) to the robot.
Your programs can also run on **another computer on the same network**: one computer is
"the bridge", and others program the robot from there with plain ROS 2.

```
Baxter  <--ROS 1-->  bridge computer  <--ROS 2, over the network-->  your computer
```

Your computer never talks to the robot itself. It needs no ROS 1, no robot address and no
robot settings; it only has to see the bridge computer's ROS 2 traffic.

## 1. Open the bridge to the network

By default `add_robot.sh` keeps ROS 2 on the bridge computer
(`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), so that several lab machines do not see each
other. For programs on other computers, write the bridge's settings with `--subnet`
instead, on the bridge computer:

```bash
bash scripts/add_robot.sh --subnet <the robot's name>
docker compose up -d bridge
```

On a Windows laptop with the bridge in WSL (`DDS_PROFILE=wsl` in `.env`), the WSL profile
also keeps ROS 2 on that computer; `bridge_config/dds/wsl.xml` says which line to add for
other computers.

## 2. Set up your computer

Clone the repository and run the setup script, as on the bridge computer:

```bash
git clone https://github.com/Hull-Robotics-Society/baxter-ros2-bridge.git ~/baxter-ros2-bridge
cd ~/baxter-ros2-bridge
bash scripts/setup.sh
```

With ROS 2 installed (Jazzy, Lyrical or another current version), it builds Baxter's
messages and the Python library in `~/baxter_ws`. Without ROS 2, it downloads the
`client` image, a container with all of that. It also downloads the bridge's image,
which this computer does not need. Do not run `add_robot.sh` here: there is no robot to
add.

If the bridge uses another ROS 2 domain than 42, create a file `.env` in the repository
with the line `ROS_DOMAIN_ID=<number>`.

## 3. Run a program

With ROS 2 installed, in every terminal:

```bash
source ~/baxter_ws/baxter.sh
ros2 run baxter_interface enable_robot -s                    # the robot's state
python3 ~/baxter-ros2-bridge/examples/01_read_robot.py
```

With Docker only, in the repository's folder:

```bash
docker compose run --rm client ros2 run baxter_interface enable_robot -s
docker compose run --rm client python3 examples/01_read_robot.py
```

Docker prints warnings that `ROS_MASTER_URI` and other variables are not set. They
belong to the bridge and can be ignored on this computer. For your own programs, put
their folder into `.env` as `PROJECT_DIR=/home/you/my_baxter_programs`; then
`docker compose run --rm client python3 my_program.py` runs
`/home/you/my_baxter_programs/my_program.py`.

Without the repository, the client image alone is enough. Docker downloads it the first
time (`latest` is the newest release; `docker pull` gets a newer one):

```bash
docker run --rm -it --network host --ipc host -e ROS_DOMAIN_ID=42 \
  -v "$PWD":/project -w /project \
  ghcr.io/hull-robotics-society/baxter-client-jazzy:latest python3 my_program.py
```

## What the network needs

1. **Same network.** Your computer and the bridge computer must reach each other
   directly (same Wi-Fi or same switch). Check from your computer:
   `ping <address of the bridge computer>`. If the bridge runs in a virtual machine, the
   VM needs a *bridged* network adapter on that network too.
2. **Same ROS 2 domain.** ROS 2 programs find each other only on the same domain number.
   The bridge uses `ROS_DOMAIN_ID` from its `.env` (42 unless changed).
3. **Finding each other.** ROS 2 programs announce themselves with *multicast* messages.
   Home routers and simple switches pass those on. University and company Wi-Fi often do
   not, and a firewall on either computer can block them. Then name the other computer
   directly (step 5 below).

Check the connection with `ros2 node list` (or
`docker compose run --rm client ros2 node list`): it should list `/ros_bridge`, the
bridge itself. `ros2 topic list` shows the robot's topics from
`bridge_config/topics.yaml`.

## If the two computers do not see each other

Work through this in order:

1. `ping` in both directions. No answer: not the same network, or a firewall.
2. `echo $ROS_DOMAIN_ID` on your computer, and `ROS_DOMAIN_ID` in the bridge computer's
   `.env`: the same number?
3. `ROS_AUTOMATIC_DISCOVERY_RANGE` in the bridge computer's `.env` must be `SUBNET`
   (step 1), and `echo $ROS_AUTOMATIC_DISCOVERY_RANGE` on your computer empty or `SUBNET`.
   After changing `.env`: `docker compose up -d bridge`.
4. Firewall on Ubuntu: `sudo ufw status`. If it is active, allow the other computer:
   `sudo ufw allow from <address of the other computer>`.
5. Multicast blocked by the network: name the other computer directly. On your computer:

   ```bash
   export ROS_STATIC_PEERS=<address of the bridge computer>
   ```

   and on the bridge computer, add `ROS_STATIC_PEERS=<address of your computer>` to the
   `environment:` list of the `bridge` service in `compose.yaml`, then
   `docker compose up -d bridge`.

Docker Desktop on Windows or macOS runs containers inside a hidden virtual machine whose
network is not your computer's network, so ROS 2 programs in it do not find the bridge
computer. Use a Linux computer, WSL2 in mirrored mode, or a Linux VM with a bridged
network adapter.

## Good to know

- **One robot, several computers:** all computers on the domain can read at the same
  time. Commands from two programs to the same arm mix; agree on who moves the robot.
- **The e-stop is at the robot,** not at your desk. Someone must be there while the robot
  is enabled.
- **Wi-Fi:** reading and the `move_...` commands are fine over Wi-Fi. A control loop of
  your own at 100 targets per second (see [arms.md](arms.md)) wants a cable.
- **Two robots:** each robot has its own bridge and a name in front of its topics; your
  program picks the robot by that name. See [two-robots.md](two-robots.md).

## Limitations

- Not tested yet: two separate computers over a real network, and `ROS_STATIC_PEERS`
  with the bridge. The network part of this page is standard ROS 2 behaviour.
