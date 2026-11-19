#!/usr/bin/env bash
# Shell environment for this project.
#
# Source this file in every terminal that starts Gazebo or SITL:
#   source scripts/env.sh
#
# Nothing here starts a process. It only tells each program where the
# other programs installed their files.

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
# the world filename iris_runway.sdf. Gazebo searches this path for both.
export GZ_SIM_RESOURCE_PATH="${ROOT}/third_party/ardupilot_gazebo/models:${ROOT}/third_party/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

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
