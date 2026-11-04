#!/bin/bash
# Sets up ROS 1 and ROS 2 side by side, then runs the given command.
# Order matters: underlays (Noetic, Jazzy) first, then our overlays. The bridge
# overlay must be sourced with local_setup.bash, not setup.bash.
source /opt/ros/noetic/setup.bash
source /opt/ros/jazzy/setup.bash
source /opt/baxter/ros1_msgs_ws/install/local_setup.bash
source /opt/baxter/ros2_msgs_ws/install/local_setup.bash
source /opt/baxter/bridge_ws/install/local_setup.bash
exec "$@"
