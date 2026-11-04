#!/bin/bash
# Robot inventory: records everything the bridge needs to know about a Baxter robot.
#
# READ ONLY. It lists and records; it never publishes, enables or moves anything.
#
# What it collects, into one folder (plus a .tar.gz of it):
#   summary.txt          overview: robot software version, counts, checksum comparison result
#   environment.txt      when, from where, which ROS master
#   version_params.txt   all parameters mentioning rethink/version
#   params.yaml          the full parameter server
#   nodes.txt, topics.txt, topics_verbose.txt, topic_types.txt
#   machines.txt         the computers the nodes run on (the robot; a Mobility Base's own
#                        computer, ...) and whether this machine can reach each of them
#   mobility_base.txt    the topics, services and nodes with "mobility" in their name
#   services.txt         every service with its type
#   inventory.bag        40 s of every topic except camera images (holds each message
#                        definition and checksum exactly as the robot sends it)
#   bag_info.txt         rosbag info of that bag
#   definitions/         one file per message type: checksum + full definition
#   md5_compare.txt      robot checksum vs. this machine's checksum, per type
#   unchecked_types.txt  types of topics that sent nothing during the recording (e.g. command
#                        topics); their checksums are only verified by commanding the robot
#
# Usage, with the ROS 1 environment set up and ROS_MASTER_URI / ROS_IP pointing at the robot:
#   bash scripts/robot_inventory.sh [output-parent-dir]
# From the repository, in the Noetic container (recommended):
#   docker compose run --rm inventory
#
# Runs on ROS Groovy (the 12.04 workstation) and ROS Noetic. If the Mobility Base runs its
# own ROS master, run it a second time with ROS_MASTER_URI pointing at the base.

set -o pipefail

PARENT="${1:-.}"
HOST_TAG="$(echo "${ROS_MASTER_URI:-unknown}" | sed -e 's#^[a-z]*://##' -e 's#[:/].*##')"
OUT="${PARENT}/inventory_${HOST_TAG}_$(date +%Y-%m-%d_%H%M)"
# 40 s: on the real robot (about 270 topics) "rosbag record -a" needs about 10 s just to
# subscribe to everything; with a 10 s recording the bag came out empty (5 Oct 2026).
BAG_SECONDS="${BAG_SECONDS:-40}"
# Large image topics are left out of the bag, and the 3D point clouds of a Mobility Base's
# laser scanners; their types are still in topic_types.txt.
BAG_EXCLUDE="${BAG_EXCLUDE:-(/cameras/.*|/robot/xdisplay|.*/image.*|.*/compressed.*|.*points.*|.*point_?cloud.*)}"

step() { echo; echo "== $*"; }
# Run a command with a time limit so a stuck call cannot hang the whole inventory.
t() { timeout "${T:-20}" "$@"; }

if [ -z "${ROS_MASTER_URI}" ]; then
  echo "ROS_MASTER_URI is not set. Point it at the robot, e.g. http://<robot-hostname>:11311" >&2
  exit 1
fi

step "Checking the ROS master at ${ROS_MASTER_URI}"
if ! T=10 t rostopic list > /dev/null 2>&1; then
  echo "Cannot reach the ROS master. Check ROS_MASTER_URI, ROS_IP and the network." >&2
  exit 1
fi
mkdir -p "${OUT}/definitions"
echo "Writing to ${OUT}"

step "Environment"
{
  echo "date:           $(date -Iseconds 2>/dev/null || date)"
  echo "ROS_MASTER_URI: ${ROS_MASTER_URI}"
  echo "ROS_IP:         ${ROS_IP:-}  ROS_HOSTNAME: ${ROS_HOSTNAME:-}"
  echo "local ROS:      ${ROS_DISTRO:-unknown} ($(rosversion -d 2>/dev/null))"
  echo "local host:     $(hostname)"
} | tee "${OUT}/environment.txt"

step "Robot software version"
VERSION="$(T=10 t rosparam get /rethink/software_version 2>/dev/null)"
echo "rethink/software_version: ${VERSION:-<not set>}" | tee "${OUT}/version_params.txt"
T=10 t rosparam list 2>/dev/null | grep -iE 'rethink|version' | while read -r p; do
  echo "${p}: $(T=10 t rosparam get "${p}" 2>/dev/null | head -c 300)"
done >> "${OUT}/version_params.txt"
cat "${OUT}/version_params.txt"

step "Parameters"
T=60 t rosparam dump "${OUT}/params.yaml" && echo "params.yaml written"

step "Nodes, topics, services"
t rosnode list > "${OUT}/nodes.txt"
t rostopic list > "${OUT}/topics.txt"
t rostopic list -v > "${OUT}/topics_verbose.txt"
while read -r topic; do
  echo "${topic} $(T=10 t rostopic type "${topic}" 2>/dev/null)"
done < "${OUT}/topics.txt" > "${OUT}/topic_types.txt"
t rosservice list 2>/dev/null | while read -r srv; do
  echo "${srv} $(T=10 t rosservice type "${srv}" 2>/dev/null)"
done > "${OUT}/services.txt"
echo "$(wc -l < "${OUT}/nodes.txt") nodes, $(wc -l < "${OUT}/topics.txt") topics, $(wc -l < "${OUT}/services.txt") services"

step "Machines the nodes run on"
# Every node tells the ROS master the address it can be reached at. Nodes on a second
# computer (the Mobility Base's own one, for example) give that computer's name. Data
# from such a node only arrives here if this machine can find that name and connect to
# it; otherwise its topics are listed above but stay silent.
t rosnode list -a 2>/dev/null > "${OUT}/.node_addresses"
: > "${OUT}/machines.txt"
UNREACHABLE=""
for host in $(sed -n 's#^http://\([^:/]*\).*#\1#p' "${OUT}/.node_addresses" | sort -u); do
  count="$(grep -c "^http://${host}[:/]" "${OUT}/.node_addresses")"
  first="$(grep -m 1 "^http://${host}[:/]" "${OUT}/.node_addresses")"
  port="$(echo "${first}" | sed -n 's#^http://[^:/]*:\([0-9]*\).*#\1#p')"
  node="$(echo "${first}" | awk '{print $2}')"
  address="$(getent hosts "${host}" 2>/dev/null | awk '{print $1; exit}')"
  if [ -z "${address}" ]; then
    state="NAME NOT FOUND from this machine"
    UNREACHABLE="${UNREACHABLE} ${host}"
  elif timeout 3 bash -c "exec 3<>/dev/tcp/${host}/${port}" 2>/dev/null; then
    state="reachable (${address})"
  else
    state="NO ANSWER at ${address}:${port}"
    UNREACHABLE="${UNREACHABLE} ${host}"
  fi
  echo "${host}: ${count} nodes, e.g. ${node}; ${state}" | tee -a "${OUT}/machines.txt"
done
rm -f "${OUT}/.node_addresses"

step "Mobility Base"
{
  echo "# topics (with type)"; grep -i 'mobility' "${OUT}/topic_types.txt"
  echo "# services (with type)"; grep -i 'mobility' "${OUT}/services.txt"
  echo "# nodes"; grep -i 'mobility' "${OUT}/nodes.txt"
} > "${OUT}/mobility_base.txt" 2>/dev/null
echo "$(grep -ci 'mobility' "${OUT}/topics.txt") topics, $(grep -ci 'mobility' "${OUT}/services.txt") services, $(grep -ci 'mobility' "${OUT}/nodes.txt") nodes with \"mobility\" in their name"

step "Recording ${BAG_SECONDS} s of all topics (without images)"
T=$((BAG_SECONDS + 30)) t rosbag record -a -x "${BAG_EXCLUDE}" --duration="${BAG_SECONDS}" \
  -O "${OUT}/inventory.bag" > "${OUT}/rosbag_record.log" 2>&1
if [ -f "${OUT}/inventory.bag" ]; then
  rosbag info "${OUT}/inventory.bag" > "${OUT}/bag_info.txt"
  echo "inventory.bag: $(du -h "${OUT}/inventory.bag" | cut -f1)"
else
  echo "Recording failed, see rosbag_record.log" >&2
fi

step "Message definitions and checksum comparison"
# Reads each type's definition and checksum from the bag (as the robot sent them) and
# compares the checksum with this machine's own message packages (rosmsg md5).
# MATCH = the bridge's messages fit this robot. DIFFERENT = the robot runs another software
# version than the one msgs/ros1 is from (see msgs/README.md).
# Groovy has rosbag for Python 2 ('python'), Noetic for Python 3 ('python3'): use whichever works.
PY=""
for candidate in python python3; do
  if "${candidate}" -c "import rosbag" > /dev/null 2>&1; then PY="${candidate}"; break; fi
done
if [ -f "${OUT}/inventory.bag" ] && [ -z "${PY}" ]; then
  echo "No Python with the rosbag module found; skipping the checksum comparison" >&2
elif [ -f "${OUT}/inventory.bag" ]; then
  "${PY}" - "${OUT}" << 'EOF'
from __future__ import print_function
import os, subprocess, sys
import rosbag
out = sys.argv[1]
bag = rosbag.Bag(os.path.join(out, "inventory.bag"))
types = {}
for c in bag._get_connections():
    types[c.datatype] = (c.md5sum, c.msg_def)
bag.close()
rows = []
for name in sorted(types):
    md5, definition = types[name]
    with open(os.path.join(out, "definitions", name.replace("/", "__") + ".txt"), "w") as f:
        f.write("type: %s\nmd5: %s\n\n%s" % (name, md5, definition))
    try:
        local = subprocess.check_output(["rosmsg", "md5", name], stderr=subprocess.STDOUT)
        local = local.decode() if not isinstance(local, str) else local
        local = local.strip().splitlines()[-1].strip()
    except Exception:
        local = ""
    status = "MATCH" if local == md5 else ("NOT_INSTALLED" if not local or " " in local else "DIFFERENT")
    rows.append((status, name, md5, local))
with open(os.path.join(out, "md5_compare.txt"), "w") as f:
    for row in rows:
        f.write("%-13s %-45s robot=%s local=%s\n" % row)
for status in ("MATCH", "DIFFERENT", "NOT_INSTALLED"):
    print("%-13s %d types" % (status, sum(1 for r in rows if r[0] == status)))
EOF
  grep -E '^DIFFERENT' "${OUT}/md5_compare.txt" | grep -E 'baxter|mobility' || true
fi

step "Types that could not be checked"
# A bag only holds topics that publish during the recording. Command topics (joint_command,
# gripper command, set_super_enable, ...) are only subscribed by the robot, so their checksums
# can't be read without publishing. They are verified later by actually commanding the robot:
# a mismatch then shows up as an "md5sum" error.
awk 'NF > 1 {print $2}' "${OUT}/topic_types.txt" | sort -u > "${OUT}/.all_types"
if [ -f "${OUT}/md5_compare.txt" ]; then
  awk '{print $2}' "${OUT}/md5_compare.txt" | sort -u > "${OUT}/.checked_types"
else
  : > "${OUT}/.checked_types"
fi
comm -23 "${OUT}/.all_types" "${OUT}/.checked_types" > "${OUT}/unchecked_types.txt"
rm -f "${OUT}/.all_types" "${OUT}/.checked_types"
echo "$(wc -l < "${OUT}/unchecked_types.txt") types not checked (no messages during the recording)"

step "Summary"
{
  echo "Baxter inventory, $(date)"
  echo "ROS master:        ${ROS_MASTER_URI}"
  echo "Software version:  ${VERSION:-<not set; see version_params.txt>}"
  echo "Nodes / topics / services: $(wc -l < "${OUT}/nodes.txt") / $(wc -l < "${OUT}/topics.txt") / $(wc -l < "${OUT}/services.txt")"
  if [ -f "${OUT}/md5_compare.txt" ]; then
    echo "Checksums vs. this machine's messages:"
    for s in MATCH DIFFERENT NOT_INSTALLED; do
      echo "  ${s}: $(grep -c "^${s}" "${OUT}/md5_compare.txt")"
    done
    echo "Baxter / Mobility Base types that DIFFER:"
    grep -E '^DIFFERENT' "${OUT}/md5_compare.txt" | grep -E 'baxter|mobility' | sed 's/^/  /' || true
  fi
  echo "Not checked (no messages during the recording; verify by commanding the robot):"
  echo "  $(wc -l < "${OUT}/unchecked_types.txt") types, of which Baxter / Mobility Base:"
  grep -E 'baxter|mobility' "${OUT}/unchecked_types.txt" | sed 's/^/    /' || true
  echo "Mobility Base topics: $(grep -ci 'mobility' "${OUT}/topics.txt") (see mobility_base.txt)"
  echo "Machines the nodes run on: $(wc -l < "${OUT}/machines.txt")"
  sed 's/^/  /' "${OUT}/machines.txt"
  if [ -n "${UNREACHABLE}" ]; then
    echo "NOT REACHABLE from this machine:${UNREACHABLE}"
    echo "  Topics of nodes on these machines are listed but sent nothing here, so their"
    echo "  types are among the unchecked ones. If it is the Mobility Base's computer: put"
    echo "  its name and address into the settings file (BASE_HOSTNAME, BASE_IP, see"
    echo "  .env.example) and run the inventory again."
  fi
} | tee "${OUT}/summary.txt"

tar czf "${OUT}.tar.gz" -C "$(dirname "${OUT}")" "$(basename "${OUT}")"
echo
echo "Done: ${OUT}  (and ${OUT}.tar.gz)"
