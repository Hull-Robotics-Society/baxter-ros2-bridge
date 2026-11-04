#!/bin/bash
# Sets up ROS 2 Jazzy with MoveIt, the Baxter messages, the robot model with its MoveIt
# configuration and the Python library, then runs the given command. (Also sourced by
# shells opened in the container; there is no command then.)
source /opt/ros/jazzy/setup.bash
source /opt/baxter/ros2_msgs_ws/install/local_setup.bash
source /opt/baxter/model_ws/install/local_setup.bash
source /opt/baxter/lib_ws/install/local_setup.bash
if [ "$#" -gt 0 ]; then
  exec "$@"
fi
