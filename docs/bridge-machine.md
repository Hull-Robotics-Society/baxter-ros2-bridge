# The bridge on a Windows laptop

The bridge needs a Linux system with Docker, and a network path to the robot in both
directions. On a computer that runs Linux itself, the
[README](../README.md#on-another-computer) is all there is to do. On a Windows laptop,
use **WSL2** (Ubuntu inside Windows) in its *mirrored* network mode. This page sets it up
for a robot on a cable straight to the laptop. The university's lab machines come with
WSL in another network mode; for them see [lab-machines.md](lab-machines.md).

## Setup, once

1. **Install WSL with Ubuntu 24.04**, if it is not there yet. In PowerShell:

   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

   `wsl --version` should show WSL 2.0 or newer (mirrored networking needs it, and
   Windows 11 22H2 or newer).

2. **Mirrored networking.** By default WSL sits in a private network of its own, which
   the robot cannot reach. "Mirrored" makes WSL see the same network adapters and
   addresses as Windows. Create the file `C:\Users\<you>\.wslconfig`:

   ```
   [wsl2]
   networkingMode=mirrored
   ```

   Then, in PowerShell, `wsl --shutdown`, wait ten seconds, and open Ubuntu again.
   `wslinfo --networking-mode` in Ubuntu now says `mirrored`.

3. **The repository and the setup script**, in Ubuntu, in the Linux home folder (not
   under `/mnt/c`, which is slow):

   ```bash
   git clone https://github.com/Hull-Robotics-Society/baxter-ros2-bridge.git ~/baxter-ros2-bridge
   cd ~/baxter-ros2-bridge
   bash scripts/setup.sh
   ```

   `setup.sh` installs Docker inside Ubuntu if it is missing. Use that Docker, not
   Docker Desktop: keep Docker Desktop closed while working with the bridge, and switch
   off its WSL integration for this Ubuntu, so that `which docker` answers
   `/usr/bin/docker`. After Docker was installed, open a new Ubuntu window.

4. **Let the robot in.** Windows blocks connections coming into WSL, and the robot has
   to connect back to the bridge. In a PowerShell started with "Run as administrator",
   allow them from the addresses a direct cable gives (169.254.x.x):

   ```powershell
   New-NetFirewallHyperVRule -Name BaxterRobotTcp -DisplayName "Baxter robot into WSL (TCP)" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -RemoteAddresses 169.254.0.0/16 -Action Allow
   New-NetFirewallHyperVRule -Name BaxterRobotUdp -DisplayName "Baxter robot into WSL (UDP)" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -RemoteAddresses 169.254.0.0/16 -Action Allow
   ```

   For a robot on another network, use that network's range instead. To remove the
   rules: `Remove-NetFirewallHyperVRule -Name BaxterRobotTcp, BaxterRobotUdp`.

5. **The network adapter** the robot is plugged into must get its address automatically
   (Settings, Network & internet, Ethernet, the adapter, IP assignment: Automatic). With
   a fixed address there, Windows and WSL are not in the robot's 169.254.x.x range.

## Every session

1. Plug the robot's cable into the laptop and switch the robot on. After about a minute
   Windows gives the adapter a 169.254.x.x address by itself, and WSL sees the same one.
2. Look up the robot's address **in Windows** (PowerShell), because WSL cannot look up
   `.local` names itself. The address can change each time the robot starts:

   ```powershell
   ping -4 011603p0009.local
   ```

3. In Ubuntu, write the settings with that address and start the bridge:

   ```bash
   cd ~/baxter-ros2-bridge
   ROBOT_IP=169.254.x.x bash scripts/add_robot.sh 011603p0009.local
   docker compose up -d bridge
   ```

   `add_robot.sh` finds the laptop's own address towards the robot (`ROS_IP`) and, in
   mirrored mode, sets `DDS_PROFILE=wsl` (see below).

4. Check that the robot is reachable in both directions:

   ```bash
   bash scripts/robot_check.sh --only network
   ```

From here, [quickstart.md](quickstart.md) continues. Without ROS 2 installed in Ubuntu,
programs run in the `client` container.

If the laptop has a second wired adapter that is also plugged in somewhere, unplug it for
the session, or check with `ip route get <robot address>` that the robot is reached
through the right one.

## The WSL network profile

In mirrored mode, whatever is sent to the computer's own address `127.0.0.1` takes a
detour through Windows, and packets larger than about 1470 bytes are lost on it. ROS 2
programs on one computer introduce themselves to each other with larger packets, on
exactly that address, so they never find each other: every program ends with "Failed to
get robot state". The line `DDS_PROFILE=wsl` in `.env` makes them use an address that
stays inside Linux (`bridge_config/dds/wsl.xml` has the details); `baxter.sh` passes it
on to the terminal. With it, only programs on this computer see each other; for programs
on another computer, that file needs one more line.

The line is for mirrored mode only. In NAT or bridged mode the address it names does not
exist, the bridge reports "All whitelist interfaces were filtered out", and no ROS 2
program sees the robot. `add_robot.sh` sets it only in mirrored mode.

## Windows of Linux programs

RViz and MoveIt open their windows on the Windows desktop without further setup (no
`xhost` needed). In the `rviz` and `moveit` containers they draw without the graphics
card and keep several processor threads busy. The line
`COMPOSE_FILE=compose.yaml:compose.wsl-gpu.yaml` in `.env` passes Windows' graphics
driver into those containers instead ([rviz.md](rviz.md)).

## Why not a VirtualBox VM

A VirtualBox VM with a bridged network adapter also works, and the project started with
one. On Windows 11, though, Windows' own hypervisor is usually active (for Memory
Integrity, WSL2, Docker Desktop), and VirtualBox then runs in a much slower fallback mode
(its `VBox.log` says "VT-x is not available" and "Attempting fall back to NEM"). Measured
on one laptop (Intel i7-12700H, 16 GB), with a control loop that wants 100 rounds per
second:

| | Windows itself | VirtualBox VM | WSL2 |
| --- | --- | --- | --- |
| rounds per second achieved | 100.0 | 87 to 92 | 100.0 |
| rounds more than 5 ms late | 0.25 % | 15 to 18 % | none |
| longest pause in 2 to 3 minutes | 26 ms | 1337 ms | 11 ms |
| joint states through the bridge, per second (100 sent) | | 65 to 83 | 99.7 |

In the VM, arm commands arrive unevenly, and now and then a program reports a lost
connection that was really the VM standing still for a second. Raw computing speed is
about the same everywhere; what differs is how evenly time passes.

If you use a VM anyway: Ubuntu 24.04 with a **bridged** adapter on the network card the
robot is plugged into (not NAT; if `ip route get <robot address>` shows `10.0.2.15`, it
goes through NAT), no more than half of the computer's memory, and fewer processors
(4 rather than 8). Then follow the README as on any Linux computer.

## Limitations

- Not tested yet: `setup.sh` and `add_robot.sh` on a laptop with the robot on a cable.
  The setup they replace (Docker installed by hand, `.env` filled in by hand) is what ran
  the whole robot check from WSL2.
