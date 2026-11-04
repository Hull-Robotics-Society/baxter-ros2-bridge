# Image: Rethink's Baxter simulator (Gazebo with an emulation of the robot's ROS 1
# interface) on ROS 1 Noetic with Gazebo 11.
# Build from the repository root:  docker compose build sim
#
# The simulator was written for older ROS versions (its last release is for Kinetic). The
# sources are Rethink's own, at fixed commits; what had to be changed to build and run
# them here is in patches/ and applied below.
FROM ros:noetic-ros-base-focal

# Gazebo 11 with its ROS 1 connection, the controller framework the simulator's
# controllers plug into, and what its emulation programs are built with.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential \
      ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
      ros-noetic-ros-control ros-noetic-ros-controllers \
      ros-noetic-control-toolbox ros-noetic-realtime-tools \
      ros-noetic-xacro ros-noetic-robot-state-publisher \
      ros-noetic-kdl-parser ros-noetic-tf-conversions \
      ros-noetic-cv-bridge ros-noetic-image-transport \
      qtbase5-dev mesa-utils \
 && rm -rf /var/lib/apt/lists/*

# Rethink's sources at fixed commits:
#   baxter_common     v1.2.0          messages and the robot model (as in msgs/ and robot_model/)
#   baxter_simulator  kinetic-devel   the newest state of the simulator (v1.2.12 plus fixes)
ARG BAXTER_COMMON_COMMIT=6c4b0f375fe4e356a3b12df26ef7c0d5e58df86e
ARG BAXTER_SIMULATOR_COMMIT=1802e9365110b9c1ddb066880635a63966bce9b3
RUN mkdir -p /opt/baxter_sim/src && cd /opt/baxter_sim/src \
 && git clone https://github.com/RethinkRobotics/baxter_common.git \
 && git -C baxter_common checkout --quiet ${BAXTER_COMMON_COMMIT} \
 && git clone https://github.com/RethinkRobotics/baxter_simulator.git \
 && git -C baxter_simulator checkout --quiet ${BAXTER_SIMULATOR_COMMIT} \
 # The example programs need Rethink's ROS 1 Python library, which is Python 2. Programs
 # for the simulated robot are written with the ROS 2 library, through the bridge.
 && touch baxter_simulator/baxter_sim_examples/CATKIN_IGNORE

COPY docker/sim-noetic/patches/ /opt/baxter_sim/patches/
RUN cd /opt/baxter_sim/src \
 && for patch in /opt/baxter_sim/patches/*.patch; do \
      [ -e "$patch" ] || continue; \
      echo "applying $patch"; \
      dir=$(basename "$patch" | cut -d- -f1); \
      git -C "$dir" apply --verbose "$patch" || exit 1; \
    done

RUN cd /opt/baxter_sim \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/baxter_sim/devel/setup.bash" >> /root/.bashrc

# This project's additions: a launch file around Rethink's, and a helper that publishes
# what the real robot has and the simulator lacks. The helper is placed in one of the
# simulator's packages so that the launch file can start it like any other program.
COPY docker/sim-noetic/baxter_sim.launch /opt/baxter_sim/baxter_sim.launch
COPY docker/sim-noetic/sim_extras.py /opt/baxter_sim/src/baxter_simulator/baxter_sim_hardware/scripts/sim_extras.py
COPY docker/sim-noetic/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /opt/baxter_sim/src/baxter_simulator/baxter_sim_hardware/scripts/sim_extras.py
ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "/opt/baxter_sim/baxter_sim.launch"]
