#!/usr/bin/env bash
# Base check: how the Mobility Base's own computer (NUC) is set up. Reads only, changes
# nothing, needs no sudo. Run it FROM the bridge machine; it travels over SSH and the
# result is written on this machine:
#
#   ssh mb@<base address> 'bash -s' < scripts/base_check.sh > base_check_NUC.txt 2>&1
#
# What it answers: system and ROS versions, how the ROS environment is set (and whether
# the base's launch file is found in it), whether anything is set up to start at boot,
# what is running now, network and clock, USB devices, and the last ROS logs (why a start
# failed). docs/mobility-base.md says what the answers mean.

section() { printf '\n===== %s =====\n' "$1"; }
# The ROS environment as a terminal on the base has it: ~/.bashrc stops early in a
# non-interactive shell (such as this one over SSH), so ask an interactive one.
in_terminal() { bash -ic "$1" 2>&1 | grep -v -E 'no job control|cannot set terminal process group'; }

section "Computer"
hostname
lsb_release -ds 2>/dev/null
uname -r
uptime
df -h / | tail -1
free -h | sed -n 2p

section "Clock"
date
timedatectl 2>/dev/null | sed 's/^ *//'

section "ROS environment in a terminal (~/.bashrc)"
grep -n -E 'ROS|source|export|mobility' ~/.bashrc 2>/dev/null
in_terminal 'env | grep -E "^ROS_|MOBILITY|^CMAKE_PREFIX_PATH" | sort'

section "Is the base's launch file found?"
in_terminal 'rosversion -d; rospack find mobility_base_bringup; rospack find mobility_base_driver'
in_terminal 'roslaunch --files mobility_base_bringup mobility_base.launch'

section "Dataspeed and ROS packages"
dpkg -l 2>/dev/null | grep -E 'mobility-base|dataspeed|ros-kinetic-ros-base|ros-kinetic-desktop' | awk '{print $2, $3}'
cat /etc/apt/sources.list.d/*.list 2>/dev/null | grep -v '^#'

section "Anything set up to start at boot?"
echo "-- systemd units naming ros, mobility, robot or base:"
systemctl list-unit-files 2>/dev/null | grep -i -E 'ros|mobility|robot|baxter|dataspeed' || echo "(none)"
echo "-- /etc/systemd/system:"; ls /etc/systemd/system 2>/dev/null
echo "-- upstart jobs (/etc/init) naming ros or mobility:"; grep -l -i -E 'ros|mobility' /etc/init/*.conf 2>/dev/null || echo "(none)"
echo "-- /etc/rc.local:"; grep -v '^#' /etc/rc.local 2>/dev/null | grep . || echo "(empty or missing)"
echo "-- crontab of $(whoami):"; crontab -l 2>/dev/null || echo "(none)"
echo "-- desktop autostart:"; ls ~/.config/autostart 2>/dev/null || echo "(none)"
grep -l -i -E 'ros|mobility' /etc/xdg/autostart/*.desktop 2>/dev/null

section "Running now"
pgrep -a -f 'rosmaster|roscore|roslaunch|mobility_base|rosout' || echo "(no ROS programs running)"

section "Network"
ip -br addr
grep -v -E '^#|^$' /etc/hosts
nmcli -t -f NAME,DEVICE,TYPE con show 2>/dev/null

section "USB devices"
lsusb 2>/dev/null

section "Last ROS logs"
ls -t ~/.ros/log 2>/dev/null | head -5
last=$(ls -td ~/.ros/log/*/ 2>/dev/null | head -1)
if [ -n "$last" ]; then
  echo "-- $last"
  for f in "$last"roslaunch-*.log "$last"master.log; do
    [ -f "$f" ] && { echo "--- $(basename "$f"), last lines:"; tail -15 "$f"; }
  done
fi

section "How ROS was started before (shell history, ROS commands only)"
grep -n -E 'roscore|roslaunch|rosrun|mobility_base' ~/.bash_history 2>/dev/null | tail -25

section "Done"
date
