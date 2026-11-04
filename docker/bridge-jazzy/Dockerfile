# Image: the bridge. ROS 2 Jazzy + ROS 1 Noetic + Baxter messages (both versions) + ros1_bridge.
#
# Build from the repository root (takes 30-60 minutes the first time):
#   docker compose build bridge
#
# Based on https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder (commit e05ca56),
# changed to (a) build our own messages from msgs/ros1 and msgs/ros2, (b) install ROS base
# instead of desktop, so ros1_bridge compiles fewer message pairs, and (c) run the bridge
# directly instead of exporting a tarball.
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# Parallel compile jobs for ros1_bridge. Each job needs about 2 GB of RAM; 2 is safe for a 6 GB VM.
ARG BUILD_JOBS=2

# ros1_bridge fork with action support, pinned to the commit the community builder uses.
ARG ROS1_BRIDGE_REPO=https://github.com/smith-doug/ros1_bridge.git
ARG ROS1_BRIDGE_COMMIT=8f1b7e903e1fb4462218db659bb4981228c12206

# 1. ROS 1 Noetic. There are no official Noetic packages for Ubuntu 24.04; this community
#    PPA rebuilds them for 24.04 (amd64 only). If it ever disappears, Noetic has to be built
#    from source instead.
#    g++-11 instead of Ubuntu 24.04's default g++ 13: the community builder compiles
#    ros1_bridge with g++-11, so we keep that known-working setting.
RUN apt-get update \
 && apt-get install -y --no-install-recommends software-properties-common \
 && add-apt-repository -y ppa:ros-for-jammy/noble \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-noetic-ros-base \
      g++-11 gcc-11 git libboost-dev pkg-config python3-yaml \
 && rm -rf /var/lib/apt/lists/* \
 && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 11 \
 && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11

# 2. Baxter messages, each built against its own ROS version in its own workspace.
#    ROS_DISTRO is unset so that each setup script sets up a clean environment.
#    They are those of robot software 1.2.0 and must match the robot exactly.
COPY msgs/ros1 /opt/baxter/ros1_msgs_ws/src
RUN cd /opt/baxter/ros1_msgs_ws \
 && unset ROS_DISTRO \
 && source /opt/ros/noetic/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

COPY msgs/ros2 /opt/baxter/ros2_msgs_ws/src
RUN cd /opt/baxter/ros2_msgs_ws \
 && unset ROS_DISTRO \
 && source /opt/ros/jazzy/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

# 3. ros1_bridge. At build time it looks for message types that exist in BOTH ROS 1 and ROS 2
#    (same package and message name, or a mapping rule) and compiles a converter for each pair.
#    That is why both message workspaces are sourced first, and why this step is slow.
#    ROS_DISTRO=humble selects the action API that Jazzy shares with Humble.
#
#    One line of ros1_bridge is changed (the sed below; the grep after it fails the build
#    if the line is no longer there to change): the bridge listens to ROS 2 "best effort",
#    which means a message that gets lost on the way is not sent again. Under load that
#    lost commands from ROS 2 to the robot: about 1 in 8 arm commands on the real robot,
#    and most pictures for the screen, which are large (5 Oct 2026). With the change it
#    listens "reliable": lost messages are repeated. Consequence: a ROS 2 program that
#    publishes best effort itself (sensor data settings) is no longer heard by the
#    bridge; publishers with the default settings are.
RUN git clone "${ROS1_BRIDGE_REPO}" /opt/baxter/bridge_ws/src/ros1_bridge \
 && git -C /opt/baxter/bridge_ws/src/ros1_bridge checkout "${ROS1_BRIDGE_COMMIT}" \
 && sed -i 's/auto subscriber_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(subscriber_queue_size));/auto subscriber_qos = rclcpp::QoS(rclcpp::KeepLast(subscriber_queue_size));/' \
      /opt/baxter/bridge_ws/src/ros1_bridge/src/bridge.cpp \
 && grep -q 'auto subscriber_qos = rclcpp::QoS(rclcpp::KeepLast(subscriber_queue_size));' \
      /opt/baxter/bridge_ws/src/ros1_bridge/src/bridge.cpp \
 && source /opt/ros/noetic/setup.bash \
 && source /opt/ros/jazzy/setup.bash \
 && source /opt/baxter/ros1_msgs_ws/install/local_setup.bash \
 && source /opt/baxter/ros2_msgs_ws/install/local_setup.bash \
 && cd /opt/baxter/bridge_ws \
 && ROS_DISTRO=humble MAKEFLAGS="-j${BUILD_JOBS}" colcon build \
      --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

# 4. Start-up: entrypoint.sh sets up the environment, run_bridge.sh loads the topic list
#    and starts the bridge. The topic list itself is mounted at runtime (see compose.yaml).
#    robot_info.py passes ROS 1 parameters (e.g. the software version) on to ROS 2,
#    robot_model.py the robot's own robot model.
COPY docker/bridge-jazzy/entrypoint.sh \
     docker/bridge-jazzy/run_bridge.sh \
     docker/bridge-jazzy/robot_info.py \
     docker/bridge-jazzy/existing_services.py \
     docker/bridge-jazzy/robot_prefix.py \
     docker/bridge-jazzy/robot_place.py \
     docker/bridge-jazzy/robot_model.py \
     docker/bridge-jazzy/bridge_guard.py \
     /opt/baxter/
RUN chmod +x /opt/baxter/entrypoint.sh /opt/baxter/run_bridge.sh

ENTRYPOINT ["/opt/baxter/entrypoint.sh"]
CMD ["/opt/baxter/run_bridge.sh"]
