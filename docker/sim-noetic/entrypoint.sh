#!/bin/bash
# Sets up ROS 1 and the simulator's workspace, then runs the given command.
set -e
source /opt/ros/noetic/setup.bash
source /opt/baxter_sim/devel/setup.bash
exec "$@"
