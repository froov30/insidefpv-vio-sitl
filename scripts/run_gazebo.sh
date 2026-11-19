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
exec gz sim -v4 -r iris_runway.sdf
