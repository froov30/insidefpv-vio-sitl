#!/usr/bin/env bash
# Shell environment for this project.
#
# Source this file in every terminal that starts Gazebo or SITL:
#   source scripts/env.sh
#
# Nothing here starts a process. It only tells each program where the
# other programs installed their files.

# This project must not use ROS. On this machine the login shell sources
# /opt/ros/humble/setup.bash, which puts ROS libraries on LD_LIBRARY_PATH
# and ROS Python packages on PYTHONPATH. Those are inherited by every
# process started from this shell. Removing them here is what makes the
# "no ROS" claim verifiable at runtime instead of only in the source:
#   - no ROS message package can be imported by accident,
#   - no ROS shared library can satisfy a symbol for gz or SITL,
#   - nothing below needs those variables.
# ROS_DISTRO and friends are unset so a stray `import rclpy` fails loudly
# rather than silently working.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH ROS_LOCALHOST_ONLY

# Drop any path element that points into a ROS installation. The loop is
# written out because it has to work in plain /bin/sh-compatible bash
# without external tools (grep/tr) that could be missing in a container.
_strip_ros_path() {
  local input="$1"
  local output=""
  local element
  local IFS=:
  for element in $input; do
    case "$element" in
      /opt/ros*|*ros*_ws*)
        # Matches /opt/ros/humble/... and any ROS workspace install
        # space such as ros2_ws/install. The pattern is deliberately
        # narrow: a bare *ros* would also match ordinary paths.
        continue
        ;;
    esac
    if [[ -n "$element" ]]; then
      output="${output:+${output}:}${element}"
    fi
  done
  printf '%s' "$output"
}
# Assign, then export, rather than exporting the substitution directly.
# `export X=$(...)` reports export's own status and hides a failing command
# substitution (shellcheck SC2155); the two-line form keeps that status
# visible and behaves identically for a caller.
PYTHONPATH="$(_strip_ros_path "${PYTHONPATH:-}")"
export PYTHONPATH
LD_LIBRARY_PATH="$(_strip_ros_path "${LD_LIBRARY_PATH:-}")"
export LD_LIBRARY_PATH

# CMake searches CMAKE_PREFIX_PATH for package configs. If this shell had
# sourced /opt/ros/humble/setup.bash, that variable points into the ROS
# install, and ardupilot_gazebo's find_package(ament_cmake QUIET) then turns
# the plugin into a ROS package - which this project must not build. Strip
# ROS entries here as well, so cmake sees the same no-ROS environment as
# python and the dynamic linker. build_plugin.sh also disables that package
# lookup outright, because the build directory may have cached it before.
CMAKE_PREFIX_PATH="$(_strip_ros_path "${CMAKE_PREFIX_PATH:-}")"
export CMAKE_PREFIX_PATH

# Folder that contains scripts/, logs/, and third_party/.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Gazebo Harmonic is already installed on this machine (gz-sim 8).
# ardupilot_gazebo's CMakeLists selects Harmonic when GZ_VERSION=harmonic.
export GZ_VERSION=harmonic

# Gazebo loads ArduPilotPlugin by filename. This directory is where
# `cmake --build` writes libArduPilotPlugin.so. Without this variable
# Gazebo starts, but the Iris model cannot talk to the flight controller.
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/third_party/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

# Worlds and models are referenced as model://iris_with_gimbal and by
# a world filename such as vio_runway.sdf. Gazebo searches this path for
# both. The repository's own worlds/ directory comes first: it holds
# vio_runway.sdf, the stock iris_runway world plus the VIO calibration
# targets described in that file. The upstream directory follows it so
# model://runway, model://iris_with_gimbal and the gimbal meshes still
# resolve.
export GZ_SIM_RESOURCE_PATH="${ROOT}/worlds:${ROOT}/third_party/ardupilot_gazebo/models:${ROOT}/third_party/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

# sim_vehicle.py lives here. Putting it on PATH means the run script can
# call it by name from any working directory.
export PATH="${ROOT}/third_party/ardupilot/Tools/autotest:${PATH}"

# RapidJSON headers were unpacked here because rapidjson-dev is not
# installed system-wide and sudo is not available. CPATH is an extra
# include directory that gcc/g++ always search. The plugin build needs it.
export CPATH="${ROOT}/third_party/sysroot/usr/include${CPATH:+:${CPATH}}"

echo "insideFPV_task environment loaded"
echo "  GZ_VERSION=${GZ_VERSION}"
echo "  plugin path=${ROOT}/third_party/ardupilot_gazebo/build"
echo "  ardupilot=${ROOT}/third_party/ardupilot"
