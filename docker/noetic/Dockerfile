# Image: ROS 1 Noetic + Baxter message packages, matching the robot's software version.
# Build from the repository root:  docker compose build inventory
FROM ros:noetic-ros-base-focal

# The ROS 1 message definitions live in the repository (msgs/ros1), copied unchanged
# from Rethink's baxter_common v1.2.0.
COPY msgs/ros1/ /tmp/ws/src/

# Build them into /opt/baxter, then delete the temporary build folder.
RUN cd /tmp/ws \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/baxter \
 && rm -rf /tmp/ws

# Make the messages available in every shell, including 'docker exec' sessions.
RUN echo "source /opt/baxter/setup.bash" >> /root/.bashrc

COPY docker/noetic/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
