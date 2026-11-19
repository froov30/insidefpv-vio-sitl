#!/usr/bin/env bash
# Starts the Gazebo world that contains the Iris quadcopter.
#
# Run this in its own terminal and leave it open. The window is the
# video you record for the takeoff and landing.
#
# What this process owns:
#   - The physics step (1 ms, from iris_runway.sdf).
#   - The Iris body, four rotors, IMU, GPS, barometer, and gimbal camera.
#   - ArduPilotPlugin, which waits on UDP port 9002 for the flight controller.
#
# It does not arm, take off, or land. That is scripts/takeoff_land.py.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

PLUGIN="${ROOT}/third_party/ardupilot_gazebo/build/libArduPilotPlugin.so"
if [[ ! -f "${PLUGIN}" ]]; then
  echo "ArduPilotPlugin is not built: ${PLUGIN}" >&2
  echo "Build it with: bash scripts/build_plugin.sh" >&2
  exit 1
fi

# -r starts the simulation clock immediately. If you omit -r, Gazebo
# opens paused and SITL sits in lockstep until you press play, so the
# takeoff script will time out waiting for a position estimate.
#
# The world is worlds/vio_runway.sdf, which is the stock iris_runway
# world with a field of static calibration targets added so the camera
# has trackable texture. Its <world> name is still iris_runway, so every
# topic name in this project (world/iris_runway/model/.../camera/image)
# is unchanged. See the comment inside the world file.
#
# --headless runs the server only and renders with the EGL backend. It is
# what the automated tests and the dataset recorder use: no desktop
# session, and camera sensors still produce frames because
# --headless-rendering is passed with -s. Without that flag the image
# topic exists but never publishes.
#
# stdbuf -oL -eL matters when this is redirected to a log file. C++
# iostreams are block-buffered on a file, so Gazebo's messages sat in a
# 4 KB buffer and a caller reading the log saw nothing: an earlier version
# of run_sim.sh waited three minutes for "ArduPilot ready to fly" while
# the plugin had printed it 0.4 s after launch. Line buffering makes the
# log usable while Gazebo is running.
if [[ "${1:-}" == "--headless" ]]; then
  exec stdbuf -oL -eL gz sim -s -r --headless-rendering -v4 vio_runway.sdf
fi

exec stdbuf -oL -eL gz sim -v4 -r vio_runway.sdf
