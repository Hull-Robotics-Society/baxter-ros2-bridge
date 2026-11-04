#!/bin/bash
# Writes the settings for one robot into .env, from the robot's name:
#
#   bash scripts/add_robot.sh 011603p0009.hull.ac.uk      # through the university network
#   bash scripts/add_robot.sh 011603p0009.local           # on a cable straight to the robot
#
# It looks up the robot's address, finds this computer's own address towards it, picks
# the ROS 2 network settings for this computer (WSL's network mode), and checks that the
# robot's ROS master answers. Run it again when an address may have changed (a new day
# on a lab machine, the robot restarted on a cable); then restart the bridge.
#
# Options:
#   --file FILE    write FILE instead of .env (several robots: docs/two-robots.md)
#   --name NAME    the robot's name in ROS 2, for several robots (ROBOT_NAME, and
#                  COMPOSE_PROJECT_NAME=baxter-NAME so that the bridges run side by side)
#   --subnet       let other computers on the network reach the bridge
#                  (ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET instead of LOCALHOST;
#                  docs/own-machine.md). The default keeps ROS 2 on this computer.
#   ROBOT_IP=a.b.c.d bash scripts/add_robot.sh NAME   the address, if the name cannot be looked up
#
# An existing file is kept as FILE.old. Everything else in it comes from .env.example.

set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FILE=.env; NAME=""; RANGE=LOCALHOST; ROBOT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --file) FILE="$2"; shift;;
    --name) NAME="$2"; shift;;
    --subnet) RANGE=SUBNET;;
    -h|--help) sed -n '2,23p' "$0"; exit 0;;
    -*) echo "Unknown option $1 (see --help)" >&2; exit 2;;
    *) ROBOT="$1";;
  esac
  shift
done
[ -n "$ROBOT" ] || { sed -n '2,23p' "$0"; exit 2; }
cd "$REPO" || exit 1

stop() { echo "Stopped: $1" >&2; exit 1; }

# The robot's address
if [ -z "${ROBOT_IP:-}" ]; then
  ROBOT_IP="$(getent ahostsv4 "$ROBOT" | awk 'NR == 1 {print $1}')"
  [ -n "$ROBOT_IP" ] || stop "cannot look up $ROBOT. Is the name right, the robot on and this computer on its network?
On a direct cable the robot needs about a minute after start to take an address. If you know
it, give it: ROBOT_IP=<address> bash scripts/add_robot.sh $ROBOT"
fi

# This computer's address towards the robot: the robot connects back to it.
ROS_IP="$(ip route get "$ROBOT_IP" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')"
[ -n "$ROS_IP" ] || stop "no route to $ROBOT_IP from this computer."

# The robot's programs announce themselves as <serial>.local, whatever name it was reached by.
SERIAL="${ROBOT%%.*}"
SECOND=""
[ "$ROBOT" != "$SERIAL.local" ] && SECOND="$SERIAL.local"

# ROS 2 network settings for this computer
DDS=""
if command -v wslinfo > /dev/null && [ "$(wslinfo --networking-mode 2>/dev/null)" = mirrored ]; then
  DDS=wsl   # WSL in mirrored mode loses large packets to 127.0.0.1 (docs/bridge-machine.md)
fi

# Write the file: .env.example with these values filled in.
[ -f "$FILE" ] && mv "$FILE" "$FILE.old" && echo "The previous $FILE is now $FILE.old."
cp .env.example "$FILE"
set_value() {  # set_value KEY VALUE: replaces KEY=... or "# KEY=...", or adds the line
  if grep -q "^$1=" "$FILE"; then sed -i "s|^$1=.*|$1=$2|" "$FILE"
  elif grep -q "^# $1=" "$FILE"; then sed -i "0,/^# $1=.*/s||$1=$2|" "$FILE"
  else echo "$1=$2" >> "$FILE"; fi
}
set_value ROBOT_HOSTNAME "$ROBOT"
set_value ROBOT_IP "$ROBOT_IP"
set_value ROS_MASTER_URI "http://$ROBOT:11311"
set_value ROS_IP "$ROS_IP"
[ -n "$SECOND" ] && set_value ROBOT_HOSTNAME_2 "$SECOND"
[ -n "$DDS" ] && set_value DDS_PROFILE "$DDS"
set_value ROS_AUTOMATIC_DISCOVERY_RANGE "$RANGE"
if [ -n "$NAME" ]; then
  set_value ROBOT_NAME "$NAME"
  # Containers of their own for this robot, so that its bridge runs next to the others.
  set_value COMPOSE_PROJECT_NAME "baxter-$NAME"
fi

echo "Written to $FILE:"
grep -E '^(ROBOT_HOSTNAME|ROBOT_HOSTNAME_2|ROBOT_IP|ROS_MASTER_URI|ROS_IP|ROS_DOMAIN_ID|DDS_PROFILE|ROS_AUTOMATIC_DISCOVERY_RANGE|ROBOT_NAME|COMPOSE_PROJECT_NAME|BRIDGE_MODE)=' "$FILE" \
  | sed 's/^/  /'

# Does the robot's ROS master answer?
echo
if timeout 3 bash -c "echo > /dev/tcp/$ROBOT_IP/11311" 2>/dev/null; then
  echo "The robot's ROS master answers at $ROBOT_IP:11311."
else
  echo "The robot's ROS master does not answer at $ROBOT_IP:11311 (still booting? a few minutes after switching on)."
fi

ARGS=""
[ "$FILE" != .env ] && ARGS=" --env-file $FILE"
echo
echo "Next: docker compose$ARGS up -d bridge     (also after every change to $FILE)"
