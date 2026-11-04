#!/bin/bash
# Robot check: verifies the bridge and the Python library against the REAL robot, stage by
# stage.
#
# It starts with stages that only read. Before every stage that makes the robot do
# something, it says what will happen and waits for you to type "yes".
#
# Before running:
#   - .env made from .env.example and filled in (robot name and address, this
#     machine's address)
#   - the robot switched on and booted, e-stop released, someone next to the e-stop
#   - nobody and nothing within reach of the arms
#
# Usage, from the repository root on the bridge machine:
#   bash scripts/robot_check.sh                  all stages, in order (inventory only if asked)
#   bash scripts/robot_check.sh --list           show the stages
#   bash scripts/robot_check.sh --from arms      start at a stage (earlier ones are skipped)
#   bash scripts/robot_check.sh --only cameras   one stage (several: --only head,screen)
#   bash scripts/robot_check.sh --inventory      all stages, the inventory included
#   bash scripts/robot_check.sh --build          build missing images first (slow; see below)
#
# Stages:
#   network    (reads)  .env, addresses, can the robot be reached
#   inventory  (reads)  records everything about the robot; compares message checksums.
#                       Only with --inventory, --only inventory or --from inventory: for a
#                       new robot, or one with other software than 1.2.0
#   bridge     (reads)  starts the bridge, checks its log
#   read       (reads)  library build, unit tests, state of arms, head, grippers, cameras ...
#   enable     MOVES NOTHING, but powers the arms: enables the robot
#   arms       MOVES: untuck, turn each wrist a little and back, neutral pose, left hand
#              5 cm up and down, both arms and head at once, left arm along a short
#              path (joint trajectory action, in each of its three modes)
#   head       MOVES: head left, right, centre, nod
#   screen     picture on the screen
#   grippers   MOVES: calibrate, close, open (electric grippers only)
#   cameras    opens and closes cameras, live picture on the screen, measures bridge load
#   navigators lights blink (navigators, halo, sonar ring); you press a button, the head nods (a callback moving the robot)
#   smoke      MOVES WIDELY: the SDK's own smoke tests, one by one, each asked separately
#   finish     MOVES: tuck the arms and disable the robot
#
# Results: one line per check (PASS / FAIL / SKIP), and the full output of every check in
# robot_check_logs/<date_time>/, with summary.txt. Copy that folder back afterwards.
#
# Images: the check uses those of the services bridge and client (and inventory, when that
# stage runs), by the names compose.yaml gives them. Missing ones are downloaded
# (docker compose pull); --build builds what still is missing (about half an hour for the
# bridge).

set -o pipefail

STAGES="network inventory bridge read enable arms head screen grippers cameras navigators smoke finish"

# ----------------------------------------------------------------------------------------
# Shared: running one check and recording its result
# ----------------------------------------------------------------------------------------
N=0

record() {  # record PASS|FAIL|SKIP|NOTE "name" "note"
  printf '%-9s %s%s\n' "$1" "$2" "${3:+  ($3)}"
}

# check "name" seconds command...      must succeed within the time limit
check() {
  local name="$1" limit="$2"; shift 2
  N=$((N + 1)); local log; log="$OUT/${STAGE}_$(printf '%02d' "$N")_${name//[^A-Za-z0-9]/_}.log"
  timeout "$limit" "$@" > "$log" 2>&1; local rc=$?
  if [ $rc -eq 0 ]; then record PASS "$name"
  elif [ $rc -eq 124 ]; then record FAIL "$name" "did not finish within ${limit} s"; tail -n 6 "$log" | sed 's/^/            /'
  else record FAIL "$name" "exit code $rc"; tail -n 10 "$log" | sed 's/^/            /'
  fi
  return $rc
}

# show "name" seconds command...       like check, and prints the command's output
show() {
  local name="$1" limit="$2"; shift 2
  N=$((N + 1)); local log; log="$OUT/${STAGE}_$(printf '%02d' "$N")_${name//[^A-Za-z0-9]/_}.log"
  # sed -u: pass every line on at once. Without it sed collects its output while it is not
  # writing to a terminal, and "DO: press the button now" appeared only after the step.
  timeout "$limit" "$@" 2>&1 | tee "$log" | sed -u 's/^/            /'; local rc=${PIPESTATUS[0]}
  if [ $rc -eq 0 ]; then record PASS "$name"
  elif [ $rc -eq 124 ]; then record FAIL "$name" "did not finish within ${limit} s"
  else record FAIL "$name" "exit code $rc"
  fi
  return $rc
}

step() {  # step "name" seconds step-name     one step of robot_check_steps.py, output shown
  show "$1" "$2" python3 -u /scripts/robot_check_steps.py "$3"
}

# ----------------------------------------------------------------------------------------
# Inside part: one stage, run in the dev container (--inside <stage>)
# ----------------------------------------------------------------------------------------
if [ "$1" = "--inside" ]; then
  STAGE="$2"; OUT=/out
  cd /opt/baxter/lib_ws || exit 1
  if [ "$STAGE" != "read" ]; then
    # shellcheck disable=SC1091
    source install/setup.bash
  fi

  case "$STAGE" in
    read)
      check "colcon build" 300 colcon build --symlink-install || exit 1
      # shellcheck disable=SC1091
      source install/setup.bash
      check "unit tests" 180 python3 -m pytest src/baxter_interface_ros2/test -q -p no:cacheprovider
      # A robot with a name (ROBOT_NAME in the settings file, docs/two-robots.md) has the
      # name in front of its topics; the container knows it as BAXTER_ROBOT.
      P="${BAXTER_ROBOT:+/$BAXTER_ROBOT}"
      if ! check "joint states arrive" 20 ros2 topic echo --once "$P/robot/joint_states" sensor_msgs/msg/JointState; then
        echo "No joint states: is the bridge up, and ROS_DOMAIN_ID the same?"; exit 1
      fi
      check "robot state arrives" 20 ros2 topic echo --once "$P/robot/state" baxter_core_msgs/msg/AssemblyState
      show "robot information" 30 ros2 run baxter_interface robot_info
      step "software version fits the messages" 30 read_version
      show "robot state (enable_robot -s)" 30 ros2 run baxter_interface enable_robot -s
      step "time to connect the parts of a typical program" 90 read_connect_time
      step "arms: joint angles and hand positions" 40 read_arms
      step "head state" 30 read_head
      step "grippers: state and type" 40 read_grippers
      step "navigators: wheels, buttons, lights" 60 read_navigators
      step "cameras: list (first real Baxter service call through the bridge)" 60 read_cameras
      step "inverse kinematics service" 40 read_ik
      step "sensors: sonar, hand range, accelerometers" 60 read_sensors
      check "analog I/O overview arrives" 20 ros2 topic echo --once "$P/robot/analog_io_states" baxter_core_msgs/msg/AnalogIOStates
      ;;
    enable)
      if ! show "enable the robot" 40 ros2 run baxter_interface enable_robot -e; then
        echo "            If reading worked but enabling does not: ROS_IP in .env is probably not the"
        echo "            address the robot can reach this machine at (commands travel robot -> here)."
        echo "            Also check: e-stop released? Then: ros2 run baxter_interface enable_robot -r"
      fi
      show "robot state after enabling" 30 ros2 run baxter_interface enable_robot -s
      ;;
    arms)
      show "untuck the arms" 90 ros2 run baxter_interface tuck_arms -u --timeout 60
      step "trace of one wrist move, left arm (for diagnosis; saved as move_trace.csv)" 60 move_trace
      step "turn each wrist by 0.2 rad and back" 90 small_move
      show "both arms to the neutral pose" 60 ros2 run baxter_interface move_to_neutral
      step "left hand 5 cm up and down again (move_by, inverse kinematics)" 90 hand_move
      step "both arms and head at once (at_once), left arm through 3 waypoints (move_through)" 120 together
      # The action server runs only for this check, once in each of its modes. (Started
      # with python3 -m, not ros2 run, so that stopping it stops the server itself and
      # not just a wrapper around it.)
      for mode in position position_w_id velocity; do
        show "left arm along a path, joint trajectory action in ${mode} mode" 120 bash -c \
          "python3 -m baxter_interface.tools.joint_trajectory_action_server -l left -m ${mode} & server=\$!; sleep 8; python3 /examples/12_trajectory_client.py left; rc=\$?; kill \$server; wait \$server 2>/dev/null; exit \$rc"
      done
      ;;
    head)
      step "head: left, right, centre, nod" 90 head_move
      ;;
    screen)
      step "test picture on the screen" 60 screen
      ;;
    screen_reset)
      step "standard picture back on the screen" 40 screen_reset
      ;;
    grippers)
      step "grippers: calibrate, close, open" 120 grippers
      ;;
    cameras)
      show "cameras before" 60 ros2 run baxter_interface camera_control -l
      step "left hand camera, then head camera: pictures, live on the screen" 180 cameras
      show "cameras after" 60 ros2 run baxter_interface camera_control -l
      ;;
    navigators)
      step "navigator lights blink" 60 navigator_lights
      step "head lights: the halo changes colour, the sonar lights run round the ring" 60 head_lights
      step "button press makes the head nod (callback that moves the robot)" 120 button
      ;;
    smoke_*)
      TEST="${STAGE#smoke_}"
      show "smoke_test ${TEST}" 400 bash -c \
        "echo | ros2 run baxter_interface smoke_test -t ${TEST} -o /out/smoke_${TEST}.smoketest; rc=\$?; cat /out/smoke_${TEST}.smoketest; exit \$rc"
      ;;
    finish)
      show "tuck the arms" 120 ros2 run baxter_interface tuck_arms -t --timeout 90
      show "disable the robot" 40 ros2 run baxter_interface enable_robot -d
      show "robot state at the end" 30 ros2 run baxter_interface enable_robot -s
      ;;
    *) echo "Unknown stage: $STAGE" >&2; exit 2;;
  esac
  exit 0
fi

# ----------------------------------------------------------------------------------------
# Host part
# ----------------------------------------------------------------------------------------
FROM=""; ONLY=""; BUILD=0; WITH_INVENTORY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --from) FROM="$2"; shift;;
    --only) ONLY="${2//,/ }"; shift;;
    --build) BUILD=1;;
    --inventory) WITH_INVENTORY=1;;
    --list) sed -n '/^# Stages:/,/^#   finish/p' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
    -h|--help) sed -n '2,47p' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
    *) echo "Unknown option: $1 (see --help)" >&2; exit 2;;
  esac
  shift
done
for s in $FROM $ONLY; do
  echo " $STAGES " | grep -q " $s " || { echo "Unknown stage: $s (see --list)" >&2; exit 2; }
done

cd "$(dirname "$0")/.." || exit 1
# Which settings file: .env, or the one named in ENV_FILE, for example to check against
# the simulator:  ENV_FILE=.env.sim bash scripts/robot_check.sh --from bridge
ENV_FILE="${ENV_FILE:-.env}"
if [ ! -f compose.yaml ] || [ ! -f "$ENV_FILE" ]; then
  echo "Run this from the repository, with a $ENV_FILE file (see .env.example)." >&2; exit 1
fi
export COMPOSE_ENV_FILES="$ENV_FILE"   # every 'docker compose' below uses the same file
# Read the settings the way docker compose does (plain KEY=value lines).
getenv() { sed -n "s/^$1=//p" "$ENV_FILE" | tail -n 1 | tr -d '\r'; }
ROBOT_HOSTNAME="$(getenv ROBOT_HOSTNAME)"; ROBOT_IP="$(getenv ROBOT_IP)"
MASTER="$(getenv ROS_MASTER_URI)"; MY_IP="$(getenv ROS_IP)"
ROBOT_SOFTWARE=1.2.0   # the robot software whose messages the images are built with
# The images' names and tags come from compose.yaml, so a release changes them there only.
# The inventory stage runs only when asked for (see "Stages" at the top).
[ "$FROM" = inventory ] && WITH_INVENTORY=1
echo " $ONLY " | grep -q " inventory " && WITH_INVENTORY=1
SERVICES="bridge client"
[ $WITH_INVENTORY -eq 1 ] && SERVICES="$SERVICES inventory"
IMAGES="$(docker compose config --images $SERVICES 2>/dev/null)"
BRIDGE_MODE="$(getenv BRIDGE_MODE)"; BRIDGE_MODE="${BRIDGE_MODE:-fixed_list}"

# The robot's name is part of the folder name, so that the logs of two robots stay apart.
OUT="$PWD/robot_check_logs/$(date +%Y-%m-%d_%H%M%S)_$(echo "${ROBOT_HOSTNAME:-robot}" | tr -c 'A-Za-z0-9.-' '_')"
mkdir -p "$OUT"
exec > >(tee "$OUT/summary.txt") 2>&1
echo "Full output of every check: $OUT"
echo "Branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null) @ $(git rev-parse --short HEAD 2>/dev/null)"
echo "Robot: ${ROBOT_HOSTNAME:-?} (${ROBOT_IP:-?})   this machine: ${MY_IP:-?}   bridge image: $(echo "$IMAGES" | grep bridge)   bridge mode: ${BRIDGE_MODE}"

wanted() {  # wanted <stage>: should this stage run?
  if [ -n "$ONLY" ]; then echo " $ONLY " | grep -q " $1 "; return; fi
  if [ "$1" = inventory ] && [ $WITH_INVENTORY -eq 0 ]; then return 1; fi
  if [ -n "$FROM" ]; then
    local seen=0 s
    for s in $STAGES; do [ "$s" = "$FROM" ] && seen=1; [ "$s" = "$1" ] && break; done
    [ $seen -eq 1 ]; return
  fi
  return 0
}

# ask "what will happen": true if the user types yes. Everything else skips the stage.
ask() {
  echo
  echo ">>> NEXT: $1"
  printf '>>> Type "yes" to do this, anything else to skip: '
  local answer; read -r answer < /dev/tty
  echo "$answer"
  [ "$answer" = "yes" ]
}

# seen "question": asks what only a person at the robot can tell, and records the answer.
seen() {
  printf '>>> %s [y/n]: ' "$1"
  local answer; read -r answer < /dev/tty
  echo "$answer"
  if [ "$answer" = "y" ] || [ "$answer" = "yes" ]; then record PASS "seen: $1"
  else record FAIL "seen: $1" "answered: ${answer:-nothing}"; fi
}

DEV=""
cleanup() {
  [ -n "$DEV" ] && docker rm -f "$DEV" > /dev/null 2>&1
  [ -n "$STATS_PID" ] && kill "$STATS_PID" 2> /dev/null
  docker compose logs --no-color bridge > "$OUT/bridge.log" 2>&1
  echo
  echo "Result: $(grep -c '^PASS' "$OUT/summary.txt") passed, $(grep -c '^FAIL' "$OUT/summary.txt") FAILED, $(grep -c '^SKIP' "$OUT/summary.txt") skipped. Details: $OUT"
  [ -n "$(docker compose ps --status running -q bridge 2>/dev/null)" ] && \
    echo "The bridge is left running (docker compose stop bridge to stop it)."
}
trap cleanup EXIT

# One dev container for all stages, so the library is built once.
start_dev() {
  [ -n "$DEV" ] && return 0
  DEV="baxter_robot_check_$$"
  docker compose run -d -T --name "$DEV" \
    -v "$PWD/scripts:/scripts:ro" -v "$OUT:/out" \
    dev sleep infinity > /dev/null 2>&1 || { record FAIL "start the dev container"; DEV=""; return 1; }
}
READ_DONE=0
inside() {  # inside <stage>
  start_dev || return 1
  if [ $READ_DONE -eq 0 ] && [ "$1" != "read" ]; then
    # Stages after "read" need the library built: do that quietly if "read" was skipped.
    docker exec "$DEV" /opt/baxter/entrypoint.sh bash -c \
      "cd /opt/baxter/lib_ws && colcon build --symlink-install" > "$OUT/colcon_build.log" 2>&1 \
      || { record FAIL "build the library" "see colcon_build.log"; return 1; }
    READ_DONE=1
  fi
  docker exec "$DEV" /opt/baxter/entrypoint.sh bash /scripts/robot_check.sh --inside "$1"
  [ "$1" = "read" ] && READ_DONE=1
  return 0
}

# --- network ----------------------------------------------------------------------------
STAGE=network
if wanted network; then
  echo; echo "--- network: settings and reachability (reads only)"
  OK=1
  for pair in "ROBOT_HOSTNAME=$ROBOT_HOSTNAME" "ROBOT_IP=$ROBOT_IP" "ROS_MASTER_URI=$MASTER" "ROS_IP=$MY_IP"; do
    case "${pair#*=}" in ""|*x.x*|*xxx*) record FAIL ".env: ${pair%%=*} is not filled in"; OK=0;; esac
  done
  [ $OK -eq 1 ] && record PASS ".env has the robot's name and address and this machine's address"
  case "$MASTER" in
    *"$ROBOT_HOSTNAME"*) record PASS "ROS_MASTER_URI uses the robot's name";;
    *) record NOTE "ROS_MASTER_URI ($MASTER) does not contain ROBOT_HOSTNAME ($ROBOT_HOSTNAME)" "fine if it is the robot's address";;
  esac
  if ip -4 -o addr 2>/dev/null | grep -qw "$MY_IP"; then record PASS "ROS_IP ($MY_IP) is an address of this machine"
  else record FAIL "ROS_IP ($MY_IP) is not an address of this machine" "see: ip -4 -br addr"; ip -4 -br addr | sed 's/^/            /'; fi
  # Which of this machine's addresses is used to reach the robot? That one must be ROS_IP:
  # it is the address the robot connects back to. (A wrong ROS_IP is the classic mistake:
  # reading works, commands never arrive.)
  SRC="$(ip route get "$ROBOT_IP" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -n 1)"
  VIA="$(ip route get "$ROBOT_IP" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -n 1)"
  if [ -z "$SRC" ]; then record FAIL "this machine has no route to the robot's address ($ROBOT_IP)" "is the wired connection up? see: ip -4 -br addr"
  elif [ "$SRC" = "$MY_IP" ]; then record PASS "ROS_IP is the address this machine uses towards the robot (network card $VIA)"
  else record FAIL "ROS_IP is $MY_IP, but this machine reaches the robot from $SRC (network card $VIA)" "set ROS_IP=$SRC in .env"; fi
  case "$SRC" in 10.0.2.*) record FAIL "the robot is being reached through VirtualBox's NAT adapter" "the bridged adapter (Adapter 3) is missing, not connected, or has no address in the robot's range";; esac
  check "robot answers ping ($ROBOT_IP)" 10 ping -c 2 -W 2 "$ROBOT_IP"
  if ! check "ROS master port 11311 is open on the robot" 8 bash -c "exec 3<>/dev/tcp/$ROBOT_IP/11311"; then
    echo "            Is the robot fully booted? It takes a few minutes after switching on."
  fi
  # What the robot's name resolves to on this machine, if it does (informational).
  RESOLVED="$(getent hosts "$ROBOT_HOSTNAME" 2>/dev/null | awk '{print $1}' | head -n 1)"
  if [ -n "$RESOLVED" ] && [ "$RESOLVED" != "$ROBOT_IP" ]; then
    record FAIL "the robot's name resolves to $RESOLVED here, but ROBOT_IP is $ROBOT_IP" "one of them is wrong"
  elif [ -n "$RESOLVED" ]; then record PASS "the robot's name resolves to ROBOT_IP on this machine too"
  else record NOTE "this machine cannot look up $ROBOT_HOSTNAME itself" "not needed: the containers get it from .env"; fi

  missing_images() {
    MISSING=""
    for image in $IMAGES; do
      docker image inspect "$image" > /dev/null 2>&1 || MISSING="$MISSING $image"
    done
  }
  missing_images
  if [ -n "$MISSING" ]; then
    echo "Downloading the ready-made images:$MISSING"
    timeout 1800 docker compose pull $SERVICES > "$OUT/images_pull.log" 2>&1 \
      || echo "Download failed (see images_pull.log)."
    missing_images
  fi
  if [ -z "$MISSING" ]; then record PASS "images exist"
  elif [ $BUILD -eq 1 ]; then
    echo "Building:$MISSING (the bridge image takes about half an hour)"
    check "build images" 7200 docker compose build $SERVICES || exit 1
  else
    record FAIL "images missing:$MISSING" "build them: bash scripts/robot_check.sh --build (about half an hour for the bridge)"; exit 1
  fi
  if grep -q "^FAIL" "$OUT/summary.txt"; then
    echo; echo "Fix the network settings first; nothing else can work without them."; exit 1
  fi
fi

# --- inventory --------------------------------------------------------------------------
STAGE=inventory
if wanted inventory; then
  echo; echo "--- inventory: record everything about the robot (reads only, about 1 minute)"
  # About 6 minutes on the real robot (it asks for the type of each of ~270 topics and ~90
  # services). On 5 Oct 2026 the command did not return although the inventory itself had
  # finished, so a finished inventory counts even if the command is cut off.
  STARTED="$(date +%s)"
  check "robot inventory" 1200 docker compose run --rm -T inventory < /dev/null
  INVENTORY="$(ls -dt docs/robot-inventory/inventory_*/ 2>/dev/null | head -n 1)"
  INVENTORY="${INVENTORY%/}"
  if [ -f "$INVENTORY/summary.txt" ] && [ "$(stat -c %Y "$INVENTORY/summary.txt")" -ge "$STARTED" ]; then
    cp "$INVENTORY/summary.txt" "$OUT/inventory_summary.txt" 2>/dev/null
    sed 's/^/            /' "$INVENTORY/summary.txt"
    DIFFERENT="$(grep -E '^DIFFERENT' "$INVENTORY/md5_compare.txt" 2>/dev/null | grep -c 'baxter')"
    MATCHING="$(grep -E '^MATCH' "$INVENTORY/md5_compare.txt" 2>/dev/null | grep -c 'baxter')"
    if [ "${MATCHING:-0}" -eq 0 ] && [ "${DIFFERENT:-0}" -eq 0 ]; then
      record FAIL "the inventory recorded no Baxter messages, so nothing could be compared" "see rosbag_record.log and bag_info.txt in $INVENTORY"
      exit 1
    elif [ "${DIFFERENT:-0}" -eq 0 ]; then
      record PASS "all $MATCHING recorded Baxter message types match the messages of $ROBOT_SOFTWARE"
    else
      record FAIL "$DIFFERENT Baxter message types differ from the messages of $ROBOT_SOFTWARE" "the robot runs another software version"
      grep -E '^DIFFERENT' "$INVENTORY/md5_compare.txt" | grep baxter | sed 's/^/            /'
      echo; echo "The bridge is built for robot software $ROBOT_SOFTWARE and cannot carry these messages."
      echo "msgs/README.md says how to build it for another version."
      exit 1
    fi
  else
    echo "The inventory could not run; see its log. Continuing is pointless without it."; exit 1
  fi
fi

# --- bridge -----------------------------------------------------------------------------
STAGE=bridge
if wanted bridge; then
  echo; echo "--- bridge: start it and check its log (reads only)"
  check "start bridge" 120 docker compose up -d --force-recreate bridge || exit 1
  for _ in $(seq 1 30); do
    docker compose logs bridge 2>/dev/null | grep -q "Starting parameter_bridge\|Starting the on-demand bridge" && break
    sleep 2
  done
  sleep 15   # let it set everything up
  docker compose logs --no-color bridge > "$OUT/bridge.log" 2>&1
  if grep -q "Starting parameter_bridge\|Starting the on-demand bridge" "$OUT/bridge.log"; then record PASS "bridge reached the robot's ROS master and started (mode: $BRIDGE_MODE)"
  else record FAIL "bridge did not start" "see bridge.log"; tail -n 8 "$OUT/bridge.log" | sed 's/^/            /'; exit 1; fi
  SERVICES_MADE=$(grep -c "Created 2 to 1 bridge for service" "$OUT/bridge.log")

  if [ "$BRIDGE_MODE" = "on_demand" ]; then
    # On demand: topics are connected only when a program uses them, so there is nothing to
    # count yet. The robot's services are all made available at start.
    MISSING_SERVICES=""
    for service in /cameras/list /cameras/open /cameras/close \
                   /ExternalTools/left/PositionKinematicsNode/IKService \
                   /ExternalTools/right/PositionKinematicsNode/IKService; do
      grep -qF "Created 2 to 1 bridge for service $service" "$OUT/bridge.log" || MISSING_SERVICES="$MISSING_SERVICES $service"
    done
    if [ -z "$MISSING_SERVICES" ]; then record PASS "bridge made $SERVICES_MADE of the robot's services available, among them cameras and inverse kinematics"
    else record FAIL "bridge made $SERVICES_MADE services available, but not:$MISSING_SERVICES" "see bridge.log"; fi
    # A "1 to 2" service bridge registers a ROS 2 service on the robot's ROS master. That is
    # harmless for services of ROS 2 programs, but must never carry a name the robot uses.
    WRONG_WAY=$(grep "Created 1 to 2 bridge for service" "$OUT/bridge.log" | grep -c -E "service /(cameras|ExternalTools|robot|robustcontroller)/")
    if [ "$WRONG_WAY" -eq 0 ]; then record PASS "bridge registers none of the robot's service names on the robot's side"
    else record FAIL "bridge registered $WRONG_WAY of the robot's own service names on the robot's side" "stop the bridge: docker compose stop bridge"; exit 1; fi
  else
    grep -h "topics: \|services: " "$OUT/bridge.log" | sed 's/^[^|]*| /            /'
    TOPICS_KEPT=$(sed -n 's/.*topics: bridging \([0-9]*\) configured.*/\1/p' "$OUT/bridge.log" | tail -n 1)
    TOPICS_TRIED=$(grep -c "Trying to create bidirectional bridge for topic" "$OUT/bridge.log")
    if [ -n "$TOPICS_KEPT" ] && [ "$TOPICS_TRIED" -eq "$TOPICS_KEPT" ]; then record PASS "bridge set up all $TOPICS_KEPT topics from topics.yaml"
    else record FAIL "bridge set up $TOPICS_TRIED topics, expected ${TOPICS_KEPT:-?}"; fi
    SERVICES_KEPT=$(sed -n 's/.*services: bridging \([0-9]*\) of .*/\1/p' "$OUT/bridge.log" | tail -n 1)
    if [ -n "$SERVICES_KEPT" ] && [ "$SERVICES_MADE" -eq "$SERVICES_KEPT" ]; then record PASS "bridge set up the $SERVICES_KEPT services the robot provides"
    else record FAIL "bridge set up $SERVICES_MADE services, the robot provides ${SERVICES_KEPT:-?}"; fi
    # The robot provides all six configured services (cameras, inverse kinematics).
    if [ "${SERVICES_KEPT:-0}" -lt 6 ]; then record FAIL "fewer than 6 of the robot's services found" "see the 'services: skipped' lines above"; fi
    if [ "$(grep -c "Created 1 to 2 bridge for service" "$OUT/bridge.log")" -eq 0 ]; then record PASS "bridge registers no services on the robot's side"
    else record FAIL "bridge registers services on the robot's side" "services_1_to_2 must stay empty; stop the bridge"; exit 1; fi
  fi
  FAILURES=$(grep -ci "failed to create" "$OUT/bridge.log")
  if [ "$FAILURES" -eq 0 ]; then record PASS "no 'failed to create' lines in the bridge log"
  else record FAIL "$FAILURES 'failed to create' lines in the bridge log"; grep -i "failed to create" "$OUT/bridge.log" | head -n 10 | sed 's/^/            /'; fi
elif [ -z "$(docker compose ps --status running -q bridge 2>/dev/null)" ]; then
  echo "The bridge is not running, and the bridge stage was skipped. Start it: docker compose up -d bridge" >&2; exit 1
fi

bridge_alive() {
  local id restarts
  id="$(docker compose ps -q bridge 2>/dev/null)"
  restarts="$(docker inspect -f '{{.RestartCount}}' "$id" 2>/dev/null)"
  if [ -z "$(docker compose ps --status running -q bridge 2>/dev/null)" ]; then
    record FAIL "bridge stopped during stage $1" "see bridge.log"; return 1
  elif [ "${restarts:-0}" != "${RESTARTS_SEEN:-0}" ]; then
    record FAIL "bridge crashed and restarted during stage $1" "see bridge.log"; RESTARTS_SEEN="$restarts"; return 1
  fi
  local stops
  stops="$(docker compose logs --no-color bridge 2>/dev/null | grep -c ': stopped; starting it again')"
  if [ "${stops:-0}" != "${STOPS_SEEN:-0}" ]; then
    record FAIL "a bridge process stopped and was started again during stage $1" "see bridge.log (a service call the robot did not answer?)"
    STOPS_SEEN="$stops"; return 1
  fi
  return 0
}

# --- read -------------------------------------------------------------------------------
STAGE=read
if wanted read; then
  echo; echo "--- read: the library reads the robot's state (reads only)"
  inside read; bridge_alive read
fi

# --- stages that command the robot ------------------------------------------------------
run_stage() {  # run_stage <stage> "what will happen"
  STAGE="$1"
  wanted "$1" || return 1
  echo; echo "--- $1"
  if ask "$2"; then inside "$1"; bridge_alive "$1"; return 0; fi
  record SKIP "stage $1"; return 1
}

run_stage enable "ENABLE the robot. The arms get power and hold their position; nothing moves. E-stop within reach?"

run_stage arms "MOVE BOTH ARMS: untuck (arms swing out to the front), turn each wrist a little, then the neutral pose; then the left hand 5 cm up and down, both arms and the head at the same time, and three times the left arm along a short path around the neutral pose (the hand swings roughly 30 cm outwards and back, 11 s each). The second of those runs WITHOUT the robot's collision avoidance for the left arm (position_w_id mode): keep the right arm clear of it. Is the space around the robot clear?"

run_stage head "MOVE THE HEAD: left, right, centre, then one nod."

if run_stage screen "Show a test picture on the robot's screen (nothing moves)."; then
  seen "Did the screen show 'ROS 2 bridge' and the time?"
  inside screen_reset
fi

run_stage grippers "MOVE THE GRIPPERS: calibrate, close, open (electric grippers only). Nothing between the fingers?"

if wanted cameras; then
  # The bridge's processor and memory use while cameras are streaming, every 2 seconds.
  ( while true; do
      docker stats --no-stream --format '{{.Name}} cpu={{.CPUPerc}} mem={{.MemUsage}} net={{.NetIO}}' 2>/dev/null \
        | grep bridge | sed "s/^/$(date +%H:%M:%S) /" >> "$OUT/bridge_load_cameras.log"
      sleep 2
    done ) &
  STATS_PID=$!
fi
if run_stage cameras "Open the left hand camera, then the head camera; live pictures go to the screen. Nothing moves. Takes about a minute."; then
  seen "Did the screen show live camera pictures (first the hand, then the head camera)?"
  echo "            Bridge load while the cameras ran (highest processor use):"
  sort -t= -k2 -n -r "$OUT/bridge_load_cameras.log" 2>/dev/null | head -n 1 | sed 's/^/            /'
  ls "$OUT"/*.png 2>/dev/null | sed 's/^/            saved: /'
fi
[ -n "$STATS_PID" ] && kill "$STATS_PID" 2> /dev/null; STATS_PID=""

if run_stage navigators "Blink the lights of the left arm's navigator, then wait for YOU to press a navigator's wheel: the head nods."; then
  seen "Did the navigator lights blink, and did the head nod after your button press?"
fi

STAGE=smoke
if wanted smoke; then
  echo; echo "--- smoke: the SDK's own smoke tests, one at a time"
  for TEST in Messages Services Head BlinkLEDs Cameras Grippers MoveArms; do
    case "$TEST" in
      Messages|Services) WHAT="reads only";;
      Head) WHAT="MOVES the head, shows a picture";;
      BlinkLEDs) WHAT="blinks all navigator lights";;
      Cameras) WHAT="opens each camera in turn, live picture on the screen, about 40 s";;
      Grippers) WHAT="MOVES both arms to neutral and exercises both grippers; needs TWO ELECTRIC grippers";;
      MoveArms) WHAT="MOVES BOTH ARMS WIDELY through four poses; needs the full space around the robot";;
    esac
    if ask "smoke test $TEST: $WHAT. (Each smoke test enables the robot and disables it at the end.)"; then
      inside "smoke_$TEST"; bridge_alive "smoke $TEST"
    else record SKIP "smoke_test $TEST"; fi
  done
fi

run_stage finish "MOVE BOTH ARMS into the tucked pose and DISABLE the robot (arms then sink slightly)."

echo
echo "All stages done."
