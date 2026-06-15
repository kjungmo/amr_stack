# syntax=docker/dockerfile:1
# Reproducible build + test of the AMR stack on ROS 2 Jazzy (Ubuntu 24.04).
#
# This Dockerfile lives on the `jazzy` branch; on `humble` it is provided for
# reference. Build:  docker build -f docker/jazzy.Dockerfile -t amr_stack:jazzy .
#                    docker run --rm amr_stack:jazzy
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-lc"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      libyaml-cpp-dev \
      ros-jazzy-launch-testing \
      ros-jazzy-launch-testing-ament-cmake \
      ros-jazzy-launch-testing-ros \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/

RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --event-handlers console_direct+

CMD source /opt/ros/jazzy/setup.bash \
 && source install/setup.bash \
 && colcon test \
 && colcon test-result --all
