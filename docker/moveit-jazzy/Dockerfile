# Image: MoveIt 2 for Baxter. ROS 2 Jazzy + MoveIt + RViz + Baxter's robot model, the
# MoveIt configuration (moveit_config/) and the Python library (for its joint trajectory
# action server, through which planned motions reach the robot).
#
# Build from the repository root (downloads MoveIt, about 1 GB, and the robot model):
#   docker compose build moveit
# Use: see docs/moveit.md
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# 1. MoveIt and what it draws with. python3-opencv, cv-bridge and control-msgs are what
#    the Python library needs (as in docker/client-jazzy).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      curl \
      python3-pytest \
      python3-opencv \
      ros-jazzy-cv-bridge \
      ros-jazzy-control-msgs \
      ros-jazzy-moveit \
      ros-jazzy-rviz2 \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
 && rm -rf /var/lib/apt/lists/*

# 2. Baxter's messages (as in docker/client-jazzy).
COPY msgs/ros2 /opt/baxter/ros2_msgs_ws/src
RUN cd /opt/baxter/ros2_msgs_ws \
 && source /opt/ros/jazzy/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

# 3. The robot model (downloaded from Rethink's GitHub at a fixed commit, as in
#    docker/rviz-jazzy) and the MoveIt configuration for it.
COPY robot_model/fetch_model.sh /opt/baxter/model_ws/src/baxter_description/fetch_model.sh
RUN bash /opt/baxter/model_ws/src/baxter_description/fetch_model.sh
COPY robot_model/package.xml robot_model/CMakeLists.txt /opt/baxter/model_ws/src/baxter_description/
COPY robot_model/launch /opt/baxter/model_ws/src/baxter_description/launch
COPY robot_model/rviz /opt/baxter/model_ws/src/baxter_description/rviz
COPY robot_model/xacro /opt/baxter/model_ws/src/baxter_description/xacro
COPY moveit_config /opt/baxter/model_ws/src/baxter_moveit_config
RUN cd /opt/baxter/model_ws \
 && source /opt/ros/jazzy/setup.bash \
 && colcon build \
 && rm -rf build log

# 4. The Python library.
COPY baxter_interface_ros2 /opt/baxter/lib_ws/src/baxter_interface_ros2
RUN cd /opt/baxter/lib_ws \
 && source /opt/ros/jazzy/setup.bash \
 && source /opt/baxter/ros2_msgs_ws/install/local_setup.bash \
 && colcon build --symlink-install \
 && rm -rf log

RUN echo "source /opt/baxter/entrypoint.sh" >> /root/.bashrc
COPY docker/moveit-jazzy/entrypoint.sh /opt/baxter/entrypoint.sh
RUN chmod +x /opt/baxter/entrypoint.sh

ENTRYPOINT ["/opt/baxter/entrypoint.sh"]
CMD ["ros2", "launch", "baxter_moveit_config", "moveit.launch.py"]
