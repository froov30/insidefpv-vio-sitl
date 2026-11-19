#!/usr/bin/env bash
# Configures and compiles only the ArduCopter SITL binary.
# Plane, rover, and the hardware boards are not built.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}/third_party/ardupilot"

# --board sitl compiles the flight code for this PC instead of a
# flight-controller MCU. The binary is build/sitl/bin/arducopter.
# A second configure is cheap if the board is already sitl.
./waf configure --board sitl
./waf copter -j2

echo "Built ${ROOT}/third_party/ardupilot/build/sitl/bin/arducopter"
