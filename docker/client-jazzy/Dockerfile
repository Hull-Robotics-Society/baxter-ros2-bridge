# Image: a plain ROS 2 Jazzy machine that knows Baxter's messages and has the Python
# library baxter_interface installed. This is what a lab member's computer looks like from
# the bridge's point of view: no ROS 1 at all. Programs for the robot run in it:
#   docker compose run --rm client python3 examples/01_read_robot.py
#
# Build from the repository root (a few minutes):
#   docker compose build client
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]

# pytest: for 'colcon test' when developing the Python library in this image.
# cv-bridge, opencv: pictures in the Python library (Camera, Display). cv-bridge converts
# ROS image messages to OpenCV images and brings NumPy with it.
# control-msgs: the FollowJointTrajectory action of the joint trajectory action server.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-pytest \
      python3-opencv \
      ros-jazzy-cv-bridge \
      ros-jazzy-control-msgs \
 && rm -rf /var/lib/apt/lists/*

COPY msgs/ros2 /opt/baxter/ros2_msgs_ws/src
RUN cd /opt/baxter/ros2_msgs_ws \
 && source /opt/ros/jazzy/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

# The Python library. --symlink-install makes the installed library point at the source
# files in src/ instead of copying them: the 'dev' service mounts the repository's copy
# over that folder, and edits take effect without building again.
COPY baxter_interface_ros2 /opt/baxter/lib_ws/src/baxter_interface_ros2
RUN cd /opt/baxter/lib_ws \
 && source /opt/ros/jazzy/setup.bash \
 && source /opt/baxter/ros2_msgs_ws/install/local_setup.bash \
 && colcon build --symlink-install \
 && rm -rf log

# Also set up shells opened later with 'docker compose exec client bash'.
RUN echo "source /opt/baxter/entrypoint.sh" >> /root/.bashrc

COPY docker/client-jazzy/entrypoint.sh /opt/baxter/entrypoint.sh
RUN chmod +x /opt/baxter/entrypoint.sh

ENTRYPOINT ["/opt/baxter/entrypoint.sh"]
CMD ["bash"]
