#!/bin/bash
# Sets this computer up for Baxter, once per computer (or per WSL image):
#
#   bash scripts/setup.sh
#
# 1. Installs Docker if it is missing (Ubuntu's packages docker.io and docker-compose-v2).
# 2. Downloads the bridge's ready-made image.
# 3. If ROS 2 is installed: builds Baxter's messages, the Python library, the robot model
#    and, if MoveIt is installed, the MoveIt configuration in ~/baxter_ws, installing the
#    few ROS packages that are missing. Without ROS 2 it downloads the client image
#    instead, in which programs run (docs/own-machine.md, way B).
#    It links baxter.sh into the workspace: source ~/baxter_ws/baxter.sh then sets up a
#    terminal.
#
# Run it again after updating the repository (git pull): it builds again.
# It asks for the sudo password only to install packages, and to make Python package
# folders readable that only root can read (the lab's WSL image has one; colcon stops on
# it). Nothing about the robot is set here: that is scripts/add_robot.sh.
#
# Settings (environment variables): BAXTER_WS, the workspace (default ~/baxter_ws).

set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${BAXTER_WS:-$HOME/baxter_ws}"
LINK="$WS/src/baxter-ros2-bridge"

step() { echo; echo "== $1"; }
stop() { echo; echo "Stopped: $1" >&2; exit 1; }

step "This computer"
# shellcheck disable=SC1091
. /etc/os-release
echo "$PRETTY_NAME"
if command -v wslinfo > /dev/null; then
  echo "WSL, network mode: $(wslinfo --networking-mode 2>/dev/null)"
fi

step "Docker"
NEW_GROUP=0
if ! command -v docker > /dev/null; then
  echo "Docker is not installed; installing Ubuntu's packages docker.io and docker-compose-v2."
  sudo apt-get update && sudo apt-get install -y docker.io docker-compose-v2 \
    || stop "Docker could not be installed (see the lines above)."
  sudo usermod -aG docker "$USER"
  NEW_GROUP=1
fi
DOCKER=(docker)
if ! docker info > /dev/null 2>&1; then
  if sudo docker info > /dev/null 2>&1; then
    DOCKER=(sudo docker)   # this shell is not in the docker group yet
  else
    stop "Docker is installed but not running. Start it (sudo systemctl start docker) and run this again."
  fi
fi
"${DOCKER[@]}" compose version > /dev/null 2>&1 \
  || stop "'docker compose' is missing: sudo apt-get install docker-compose-v2"
"${DOCKER[@]}" --version

step "The bridge's image"
cd "$REPO" || exit 1
"${DOCKER[@]}" compose pull bridge || stop "The image could not be downloaded (no internet connection?)."

DISTRO="${ROS_DISTRO:-$(ls /opt/ros 2>/dev/null | head -n 1)}"
if [ -z "$DISTRO" ] || [ ! -f "/opt/ros/$DISTRO/setup.bash" ]; then
  step "ROS 2"
  echo "No ROS 2 installed. Programs run in the client container instead (docs/own-machine.md, way B)."
  "${DOCKER[@]}" compose pull client || stop "The client image could not be downloaded."
  echo
  echo "Done. Next: bash scripts/add_robot.sh <the robot's name>"
  exit 0
fi

step "ROS 2 $DISTRO: packages"
NEEDED=(python3-colcon-common-extensions python3-opencv
        "ros-$DISTRO-cv-bridge" "ros-$DISTRO-control-msgs"
        "ros-$DISTRO-robot-state-publisher" "ros-$DISTRO-xacro" "ros-$DISTRO-rviz2")
MISSING=()
for p in "${NEEDED[@]}"; do
  dpkg -s "$p" > /dev/null 2>&1 || MISSING+=("$p")
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo "Installing: ${MISSING[*]}"
  sudo apt-get update && sudo apt-get install -y "${MISSING[@]}" \
    || stop "Packages could not be installed (see the lines above)."
else
  echo "All there."
fi
MOVEIT=0
if dpkg -s "ros-$DISTRO-moveit" > /dev/null 2>&1; then
  MOVEIT=1; echo "MoveIt is installed: its configuration is built too."
else
  echo "MoveIt is not installed; for docs/moveit.md: sudo apt install ros-$DISTRO-moveit, then run this again."
fi

step "Python package folders"
# colcon reads every installed Python package's information while it builds the library,
# and stops at a folder it may not read.
UNREADABLE=()
for d in /usr/local/lib/python3*/dist-packages /usr/lib/python3/dist-packages; do
  [ -d "$d" ] || continue
  while IFS= read -r entry; do UNREADABLE+=("$entry"); done \
    < <(find "$d" -mindepth 1 -maxdepth 1 ! -readable 2>/dev/null)
done
if [ ${#UNREADABLE[@]} -gt 0 ]; then
  echo "Only root can read: ${UNREADABLE[*]}"
  echo "Making them readable (this adds read rights, nothing else)."
  sudo chmod -R a+rX "${UNREADABLE[@]}" || stop "chmod failed."
else
  echo "All readable."
fi

step "Workspace $WS"
mkdir -p "$WS/src"
if [ -e "$LINK" ] && [ ! -L "$LINK" ]; then
  stop "$LINK is a folder of its own, probably an older copy. Move it out of $WS/src (mv $LINK ~/) and run this again."
fi
ln -sfn "$REPO" "$LINK"
echo "$LINK -> $REPO"
ln -sfn "$REPO/baxter.sh" "$WS/baxter.sh"
echo "$WS/baxter.sh -> $REPO/baxter.sh"
bash "$REPO/robot_model/fetch_model.sh" || stop "The robot model could not be downloaded."

PACKAGES=(msgs/ros2 baxter_interface_ros2 robot_model)
[ $MOVEIT -eq 1 ] && PACKAGES+=(moveit_config)
step "Build (${PACKAGES[*]})"
(
  cd "$WS" || exit 1
  # shellcheck disable=SC1090
  source "/opt/ros/$DISTRO/setup.bash"
  colcon build --symlink-install --base-paths "${PACKAGES[@]/#/src/baxter-ros2-bridge/}"
) || stop "The build failed (see the lines above; docs/troubleshooting.md)."

echo
echo "Done."
[ $NEW_GROUP -eq 1 ] && echo "Docker was installed: open a new terminal, so that docker works without sudo."
echo "Next, once per robot:   bash scripts/add_robot.sh <the robot's name>"
echo "In every terminal for Baxter:   source $WS/baxter.sh"
