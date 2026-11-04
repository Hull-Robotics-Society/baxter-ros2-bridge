#!/bin/bash
# Sets up ROS 2 Jazzy plus Baxter's robot model, then runs the given command.
source /opt/ros/jazzy/setup.bash
source /opt/baxter/model_ws/install/local_setup.bash
exec "$@"
