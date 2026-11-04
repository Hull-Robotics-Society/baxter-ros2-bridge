# Image: RViz 2 with Baxter's robot model, to see the robot's current posture in 3D.
#
# Build from the repository root (a few minutes; downloads the model, about 35 MB, from
# Rethink Robotics' GitHub):
#   docker compose build rviz
# Run (see docs/rviz.md):
#   xhost +local:docker && docker compose run --rm rviz
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]

# xvfb: lets the launch file be tried on a machine without a screen (xvfb-run).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      curl \
      ros-jazzy-rviz2 \
      ros-jazzy-robot-state-publisher \
      xvfb \
 && rm -rf /var/lib/apt/lists/*

# The model itself is not in the repository: fetch_model.sh downloads it at the pinned
# commit of Rethink's baxter_common v1.2.0. The script is copied first and on its own, so
# that Docker downloads again only when the script changes.
COPY robot_model/fetch_model.sh /opt/baxter/model_ws/src/baxter_description/fetch_model.sh
RUN bash /opt/baxter/model_ws/src/baxter_description/fetch_model.sh

COPY robot_model/package.xml robot_model/CMakeLists.txt /opt/baxter/model_ws/src/baxter_description/
COPY robot_model/launch /opt/baxter/model_ws/src/baxter_description/launch
COPY robot_model/rviz /opt/baxter/model_ws/src/baxter_description/rviz
COPY robot_model/xacro /opt/baxter/model_ws/src/baxter_description/xacro
RUN cd /opt/baxter/model_ws \
 && source /opt/ros/jazzy/setup.bash \
 && colcon build \
 && rm -rf build log

# Also set up shells opened later with 'docker compose exec rviz bash'.
RUN echo "source /opt/ros/jazzy/setup.bash && source /opt/baxter/model_ws/install/local_setup.bash" >> /root/.bashrc

COPY docker/rviz-jazzy/entrypoint.sh /opt/baxter/entrypoint.sh
RUN chmod +x /opt/baxter/entrypoint.sh

ENTRYPOINT ["/opt/baxter/entrypoint.sh"]
CMD ["ros2", "launch", "baxter_description", "view_robot.launch.py"]
