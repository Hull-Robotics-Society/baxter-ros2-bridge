#!/bin/bash
# Starts the bridge between the robot (ROS 1) and ROS 2.
#
# Environment (set in .env, see .env.example):
#   ROS_MASTER_URI   the robot's ROS 1 master, e.g. http://011310P0012.local:11311
#   ROS_IP           this machine's address as the robot sees it
#   ROS_DOMAIN_ID    ROS 2 domain the bridged topics appear in
#   BRIDGE_MODE      fixed_list (default) or on_demand; see below
#   ROBOT_POSITION   optional: where the robot stands in the room, "x y z turn" (metres,
#                    degrees), for pictures with several robots; see robot_place.py
#   ROBOT_NAME       optional: a name for this robot, e.g. one. Every topic and service
#                    then has it in front in ROS 2 (/one/robot/joint_states), so that
#                    several robots can be used at the same time (docs/two-robots.md).
#
# Two ways to run the bridge:
#
#   fixed_list  Only the topics and services listed in bridge_config/topics.yaml are
#               carried, all the time, whether a program uses them or not. The mode to
#               work in: on the robot (7 Oct 2026) 100 joint states per second arrived
#               with no gap over 18 ms, and camera pictures as evenly as the robot sent
#               them.
#
#   on_demand   Everything on the robot is reachable, and a topic is only carried while a
#               ROS 2 program uses it: the bridge watches both sides once per second and
#               connects a topic when one side publishes it and the other side listens.
#               The robot's services are made available in ROS 2 automatically. Nothing
#               to configure. BUT on the real robot (271 topics, 89 services) that
#               once-per-second look takes 0.3 to 0.45 s, and the bridge carries nothing
#               meanwhile: every topic pauses once per second and 20 to 30 of 100 joint
#               states are lost (measured 7 Oct 2026). For looking at what a robot
#               offers, not for moving it.
set -eo pipefail

if [ -z "${ROS_MASTER_URI}" ]; then
  echo "ROS_MASTER_URI is not set (see .env.example)" >&2; exit 1
fi
if [ -z "${ROS_IP}" ] && [ -z "${ROS_HOSTNAME}" ]; then
  echo "Set ROS_IP (or ROS_HOSTNAME), otherwise the robot cannot reach the bridge" >&2; exit 1
fi

MODE="${BRIDGE_MODE:-fixed_list}"
if [ "${MODE}" != "on_demand" ] && [ "${MODE}" != "fixed_list" ]; then
  echo "BRIDGE_MODE must be on_demand or fixed_list, not '${MODE}'" >&2; exit 1
fi

# A robot with a name: its topics and services get the name in front on the ROS 2 side
# (robot_prefix.py, further down). The name becomes part of ROS 2 names, hence the rule.
PREFIX=""
if [ -n "${ROBOT_NAME:-}" ]; then
  if ! [[ "${ROBOT_NAME}" =~ ^[A-Za-z][A-Za-z0-9]*(_[A-Za-z0-9]+)*$ ]]; then
    echo "ROBOT_NAME '${ROBOT_NAME}' is not possible: only letters, digits and single"          "underscores, starting with a letter (for example one or baxter_2)" >&2; exit 1
  fi
  if [ "${MODE}" = "on_demand" ]; then
    echo "A robot with a name (ROBOT_NAME=${ROBOT_NAME}) needs BRIDGE_MODE=fixed_list:"          "the name is put in front of the topics of the list in bridge_config/topics.yaml,"          "and on_demand has no such list. Remove ROBOT_NAME or BRIDGE_MODE=on_demand." >&2
    exit 1
  fi
  PREFIX="/${ROBOT_NAME}"
fi

# The configuration is loaded into the ROS 1 parameter server under its own namespace, so
# it does not mix with the robot's own parameters. fixed_list reads its lists from there;
# both modes read robot_info_params.
NS="${BRIDGE_PARAM_NS:-/baxter_bridge}"
CONFIG="${BRIDGE_CONFIG:-/config/topics.yaml}"

# ROS 2 programs on one machine pass data through shared memory (/dev/shm, shared with
# the host: "ipc: host" in compose.yaml). A program that is stopped hard leaves its area
# there, and the leftovers add up until the machine restarts. Fast DDS's own tool removes
# the areas that no running program uses.
if command -v fastdds > /dev/null 2>&1; then
  echo "Shared memory left by ended ROS 2 programs: $(fastdds shm clean 2>&1 | grep -i 'segments cleaned' || echo 'nothing to clean')"
fi

echo "Waiting for the ROS 1 master at ${ROS_MASTER_URI} ..."
until rosparam list > /dev/null 2>&1; do sleep 2; done

echo "Loading ${CONFIG} into ${NS}"
rosparam load "${CONFIG}" "${NS}"

# The two helper scripts below are ROS 1 programs, so they must run in an environment with
# ROS 1 only. In this container's mixed environment Python finds ROS 2's packages first
# (rospy then fails with "cannot import name 'Log' from 'rosgraph_msgs.msg'"). env -i starts
# from an empty environment.
# -u: write each log line at once (output to a log is otherwise held back in blocks).
ROS1_ENV=(HOME="${HOME}" PATH="/usr/local/bin:/usr/bin:/bin" ROS_MASTER_URI="${ROS_MASTER_URI}")
[ -n "${ROS_IP}" ] && ROS1_ENV+=(ROS_IP="${ROS_IP}")
[ -n "${ROS_HOSTNAME}" ] && ROS1_ENV+=(ROS_HOSTNAME="${ROS_HOSTNAME}")
ros1_only() {  # ros1_only <script> <args...>
  env -i "${ROS1_ENV[@]}" bash -c \
    'source /opt/ros/noetic/setup.bash && exec python3 -u "$@"' _ "$@"
}

# ros1_bridge does not carry ROS 1 parameters. This helper publishes the ones listed under
# robot_info_params as a topic, which the bridge then carries to ROS 2. It runs in the
# background; if it stops, the bridge keeps running.
ros1_only /opt/baxter/robot_info.py "${NS}" &

# robot_info.py also offers ${NS}/ping, a service that answers at once, for the guard
# below. Wait until it is there: fixed_list only bridges services that exist right now.
for _ in $(seq 1 15); do
  rosservice list 2> /dev/null | grep -qx "${NS}/ping" && break
  sleep 1
done

# keep_running <name> <command...>
# Runs a bridge process and starts it again if it stops. ros1_bridge stops when a ROS 2
# program calls a service that the ROS 1 side fails to answer (it does not catch that
# error); restarting it here takes two seconds instead of losing the bridge.
# stdbuf -oL: write each log line at once, so "docker compose logs" shows them as they happen.
keep_running() {
  local name="$1"; shift
  while true; do
    stdbuf -oL -eL "$@" || true
    echo "${name}: stopped; starting it again in 2 s"
    sleep 2
  done
}

echo "Bridge mode: ${MODE} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
[ -n "${PREFIX}" ] && echo "Robot name: ${ROBOT_NAME} (in ROS 2 everything of this robot starts with ${PREFIX}/)"

# Where this robot stands in the room (ROBOT_POSITION), for pictures with several
# robots. A ROS 2 program in the background; if it stops, the bridge keeps running.
# Not for the bridge of a second ROS 1 system next to the robot, such as the Mobility
# Base's (BRIDGE_PARAM_NS set): it has no robot to place, and would place a second "base".
if [ "${NS}" = "/baxter_bridge" ]; then
  python3 -u /opt/baxter/robot_place.py "${ROBOT_NAME:-}" "${ROBOT_POSITION:-0 0 0 0}" &
fi

# The robot's own robot model (the ROS 1 parameter /robot_description, measured for each
# robot) as the ROS 2 topic robot/robot_description, for MoveIt. A ROS 2 program in the
# background; if it stops, the bridge keeps running. Not for the Mobility Base's bridge.
if [ "${NS}" = "/baxter_bridge" ]; then
  python3 -u /opt/baxter/robot_model.py "${ROBOT_NAME:-}" &
fi

# The other way a service call can go wrong: the robot NEVER answers. Then ros1_bridge
# does not stop, it waits for good, and answers no other service call (in on_demand mode
# it also connects no more topics). bridge_guard.py notices that by calling ${NS}/ping
# through the bridge, and stops the stuck process; keep_running starts it again.
# BRIDGE_GUARD=0 switches the guard off.
if [ "${BRIDGE_GUARD:-1}" != "0" ]; then
  # With a robot name the guard's own ROS 2 node goes into the robot's namespace too.
  GUARD_NAMESPACE=()
  [ -n "${PREFIX}" ] && GUARD_NAMESPACE=(--ros-args -r "__ns:=${PREFIX}")
  python3 -u /opt/baxter/bridge_guard.py "${PREFIX}${NS}/ping" "${MODE}" "${GUARD_NAMESPACE[@]}" &
fi

if [ "${MODE}" = "on_demand" ]; then
  echo "Starting the on-demand bridge"
  keep_running on_demand_bridge ros2 run ros1_bridge dynamic_bridge
  exit 0
fi

# ----------------------------------------------------------------------------------------
# fixed_list: three bridge processes, for services, commands to the robot, state from the
# robot.
#
# Why three: one process handles everything in a single loop, taking topics before
# services and earlier topics before later ones. The real robot sends about 60 bridged
# topics at 100 messages per second each. With that, the loop never got to the services
# (calls answered after a minute or not at all), and commands from ROS 2 arrived late and
# in bursts (a head that reached its target after the time limit) or never (pictures for
# the screen). Found on the robot, 5 Oct 2026. Processes that don't carry the state flood
# stay responsive: a service call is answered in about 0.02 s.
# ----------------------------------------------------------------------------------------

# Prepare the lists (see existing_services.py):
# - the topics are split into commands to the robot and everything else;
# - only the ROS 1 services that exist right now are bridged: calling a bridged service
#   that the ROS 1 side does not provide would stop the services bridge.
# BRIDGE_CLOCK=1 (the simulator's settings file sets it through SIM_TIME): also carry the
# simulation's clock, /clock, for programs that run on simulation time.
CLOCK_ARG=()
[ "${BRIDGE_CLOCK:-0}" = "1" ] && CLOCK_ARG=(--clock)
ros1_only /opt/baxter/existing_services.py "${NS}" "${CLOCK_ARG[@]}" || true
rosparam set "${NS}/none" "[]"

# A robot with a name: robot_prefix.py puts the name in front of every name in the lists
# (what ROS 2 sees) and returns one ROS 1 remapping argument per name, which makes the
# bridge use the robot's real name on the ROS 1 side. Without a name: no arguments.
REMAPS=()
if [ -n "${PREFIX}" ]; then
  mapfile -t REMAPS < <(ros1_only /opt/baxter/robot_prefix.py "${NS}" "${PREFIX}")
  if [ "${#REMAPS[@]}" -eq 0 ]; then
    echo "The names could not get the prefix ${PREFIX} (see the error above)" >&2; exit 1
  fi
  echo "${#REMAPS[@]} topics and services get ${PREFIX} in front in ROS 2"
fi

# __name: each process needs its own ROS 1 node name. Two ROS 1 nodes with the same name
# would throw each other out.
echo "Starting the services bridge"
keep_running ros_bridge_services ros2 run ros1_bridge parameter_bridge \
  "${NS}/none" "${NS}/services_1_to_2" "${NS}/services_2_to_1" "${REMAPS[@]}" "__name:=ros_bridge_services" &

# existing_services.py has split the topics into ${NS}/topics (state from the robot) and
# ${NS}/topics_to_robot (commands). If that did not happen, everything stays in one list.
if rosparam get "${NS}/topics_to_robot" > /dev/null 2>&1; then
  echo "Starting the commands bridge"
  keep_running ros_bridge_commands ros2 run ros1_bridge parameter_bridge \
    "${NS}/topics_to_robot" "${NS}/none" "${NS}/none" "${REMAPS[@]}" "__name:=ros_bridge_commands" &
else
  echo "WARNING: the topic list was not split; commands share a process with the state topics"
fi

echo "Starting parameter_bridge"
exec stdbuf -oL -eL ros2 run ros1_bridge parameter_bridge "${NS}/topics" "${NS}/none" "${NS}/none" "${REMAPS[@]}"
