#!/usr/bin/env bash
# Build the whole workspace and run all tests (gtest unit + launch_testing).
# Works against a sourced ROS 2 Humble environment (native, Docker, or RoboStack).
#
#   ./scripts/verify.sh
#
# RoboStack example:
#   micromamba run -n ros2_humble bash -c 'unset PYTHONPATH && ./scripts/verify.sh'
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

if ! command -v colcon >/dev/null 2>&1; then
  echo "error: colcon not found. Source a ROS 2 Humble environment first." >&2
  exit 1
fi

echo "==> colcon build"
colcon build

echo "==> colcon test"
# shellcheck disable=SC1091
source install/setup.bash
colcon test

echo "==> results"
colcon test-result --all
