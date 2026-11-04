# Sets up the current terminal for working with Baxter (source it, don't run it):
#
#   source ~/baxter_ws/baxter.sh
#
# scripts/setup.sh links it into the workspace; the file itself is in the repository.
#
# Loads ROS 2 and the workspace built by scripts/setup.sh, and sets what the bridge
# expects, taken from this repository's .env: the ROS 2 domain, how far ROS 2 looks for
# other programs, and the ROS 2 middleware (Fast DDS, the bridge's). It also passes on
# which arms carry an electric gripper (LEFT_/RIGHT_ELECTRIC_GRIPPER), for MoveIt. Only this terminal
# changes; other ROS 2 work in other terminals keeps its own settings.
#
# Settings (environment variables): BAXTER_WS, the workspace when this file is sourced
# from the repository itself (default ~/baxter_ws).

# The repository is where the link points to (readlink -f follows it).
_baxter_repo="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
_baxter_ws="${BAXTER_WS:-$HOME/baxter_ws}"
# Sourced through the link in a workspace: that workspace.
if [ -L "${BASH_SOURCE[0]}" ]; then _baxter_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; fi
_baxter_distro="${ROS_DISTRO:-$(ls /opt/ros 2>/dev/null | head -n 1)}"
_baxter_get() { sed -n "s/^$1=//p" "$_baxter_repo/.env" 2>/dev/null | tail -n 1 | tr -d '\r'; }

if [ -z "$_baxter_distro" ] || [ ! -f "/opt/ros/$_baxter_distro/setup.bash" ]; then
  echo "baxter.sh: no ROS 2 installed on this computer; use the client container (docs/own-machine.md)."
elif [ ! -f "$_baxter_ws/install/setup.bash" ]; then
  echo "baxter.sh: $_baxter_ws is not built yet: bash $_baxter_repo/scripts/setup.sh"
else
  # shellcheck disable=SC1090
  source "/opt/ros/$_baxter_distro/setup.bash"
  # shellcheck disable=SC1091
  source "$_baxter_ws/install/setup.bash"

  ROS_DOMAIN_ID="$(_baxter_get ROS_DOMAIN_ID)"
  ROS_AUTOMATIC_DISCOVERY_RANGE="$(_baxter_get ROS_AUTOMATIC_DISCOVERY_RANGE)"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
  # The bridge uses Fast DDS. Topics also work with another middleware (the lab's shells
  # use Cyclone DDS), but services and actions do not.
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  # Which arms carry Rethink's electric gripper, for MoveIt's start file (docs/moveit.md).
  for _baxter_side in LEFT RIGHT; do
    _baxter_value="$(_baxter_get "${_baxter_side}_ELECTRIC_GRIPPER")"
    if [ -n "$_baxter_value" ]; then export "${_baxter_side}_ELECTRIC_GRIPPER=$_baxter_value"; fi
  done
  _baxter_dds="$(_baxter_get DDS_PROFILE)"
  if [ -n "$_baxter_dds" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$_baxter_repo/bridge_config/dds/$_baxter_dds.xml"
  fi
  echo "Baxter: ROS 2 $_baxter_distro, domain $ROS_DOMAIN_ID, discovery $ROS_AUTOMATIC_DISCOVERY_RANGE, Fast DDS${_baxter_dds:+, profile $_baxter_dds}"
fi

unset _baxter_repo _baxter_ws _baxter_distro _baxter_dds _baxter_side _baxter_value
unset -f _baxter_get
