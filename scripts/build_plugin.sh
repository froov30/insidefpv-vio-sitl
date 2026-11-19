#!/usr/bin/env bash
# Builds the Gazebo plugin that bridges Iris physics and ArduPilot SITL.
# Run once, or again after you change anything under third_party/ardupilot_gazebo.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

# -j2 is deliberate. This laptop has 8 cores but about 8 GB of RAM, and
# a Gazebo compile plus a desktop session will be killed by the OOM
# killer at -j8. Two compile jobs stay inside the free memory.
# RapidJSON is not installed system-wide (no sudo). The headers were
# unpacked under third_party/sysroot and CPATH points at them, but CMake's
# find_package(RapidJSON REQUIRED) also needs the config file, which lives
# under usr/share/cmake/RapidJSON. Passing it here is what makes the
# configure step succeed; the earlier build directory recorded
# RapidJSON_DIR=NOTFOUND.
cmake -S "${ROOT}/third_party/ardupilot_gazebo" \
  -B "${ROOT}/third_party/ardupilot_gazebo/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DRapidJSON_DIR="${ROOT}/third_party/sysroot/usr/share/cmake/RapidJSON" \
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
# The ament flag is not cosmetic. ardupilot_gazebo's CMakeLists calls
# find_package(ament_cmake QUIET) and builds as a ROS package if it is found.
# ROS Humble is installed on this machine, so a configure run from a shell
# that had sourced /opt/ros/humble/setup.bash caches ament_cmake_DIR in
# CMakeCache.txt, and every later configure tries to use it. env.sh removes
# the ROS directories from PYTHONPATH, so that half-present ament fails with
# "No module named 'ament_package'" and the build stops. Disabling the
# package lookup is what keeps this a no-ROS build regardless of who
# configured the directory before.
cmake --build "${ROOT}/third_party/ardupilot_gazebo/build" -j2
echo "Built ${ROOT}/third_party/ardupilot_gazebo/build/libArduPilotPlugin.so"
