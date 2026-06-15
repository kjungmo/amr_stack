# syntax=docker/dockerfile:1
# Reproducible build + test of the AMR stack on ROS 2 Humble (Ubuntu 22.04).
#
# Build:  docker build -f docker/humble.Dockerfile -t amr_stack:humble .
# Test:   docker run --rm amr_stack:humble   # runs colcon test + test-result
#
# The image builds the whole colcon workspace and runs the gtest unit tests and
# the launch_testing integration tests (test_slam.py, test_nav.py). RViz is
# installed so the bringup launch files work in a GUI-forwarded container.
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      libyaml-cpp-dev \
      ros-humble-launch-testing \
      ros-humble-launch-testing-ament-cmake \
      ros-humble-launch-testing-ros \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/

# Build the workspace.
RUN source /opt/ros/humble/setup.bash \
 && colcon build --event-handlers console_direct+

# Default command: run all tests and print the aggregated result.
CMD source /opt/ros/humble/setup.bash \
 && source install/setup.bash \
 && colcon test \
 && colcon test-result --all
