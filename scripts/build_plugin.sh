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
cmake -S "${ROOT}/third_party/ardupilot_gazebo" \
  -B "${ROOT}/third_party/ardupilot_gazebo/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "${ROOT}/third_party/ardupilot_gazebo/build" -j2
echo "Built ${ROOT}/third_party/ardupilot_gazebo/build/libArduPilotPlugin.so"
