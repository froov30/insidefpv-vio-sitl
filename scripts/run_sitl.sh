#!/usr/bin/env bash
# Starts ArduCopter SITL and connects it to the Gazebo Iris.
#
# Start scripts/run_gazebo.sh first. This process is the flight controller.
# It has no joystick and no ground station. scripts/takeoff_land.py is
# the ground station, and it connects to TCP port 5760.
#
# Decision: --model JSON
#   The default SITL model flies an internal multicopter that Gazebo
#   never sees. JSON tells SITL that Gazebo owns the physics. Sensor
#   packets come in, and motor commands go out, as JSON on UDP 9002.
#   That port is <fdm_port_in> in iris_with_ardupilot/model.sdf.
#
# Decision: -f gazebo-iris
#   Loads Tools/autotest/default_params/gazebo-iris.parm on top of the
#   normal copter parameters. That file sets FRAME_CLASS=1 (quad) and
#   FRAME_TYPE=1 (X), which matches the Iris motor order in the model.
#
# Decision: --no-mavproxy
#   MAVProxy is a ground station. This task's ground station is our own
#   script, so a second one would own the link and hide the commands.
#   SITL still listens for a MAVLink client on TCP 127.0.0.1:5760.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

BINARY="${ROOT}/third_party/ardupilot/build/sitl/bin/arducopter"
if [[ ! -x "${BINARY}" ]]; then
  echo "ArduCopter SITL is not built: ${BINARY}" >&2
  echo "Build it with: bash scripts/build_sitl.sh" >&2
  exit 1
fi

# eeprom.bin and terrain live in the working directory. Keeping them in
# logs/sitl means a re-run does not wipe a parameter file sitting in the
# source tree, and the home directory stays clean.
mkdir -p "${ROOT}/logs/sitl"
cd "${ROOT}/logs/sitl"

# sim_vehicle.py rebuilds by default. build_sitl.sh already compiled
# the binary, and a rebuild here takes minutes and can change the
# binary you just tested. --no-rebuild skips waf.
#
# The flight-controller console is an xterm. ArduPilot minimizes it
# unless SITL_RITW_MINIMIZE=0. Leave it visible: pre-arm failures
# ("Need Position Estimate", "EKF attitude is bad") are printed there.
export SITL_RITW_MINIMIZE=0

# --wipe on the first argument is optional. Pass --wipe as the first
# argument of this script when you want SITL to forget previous parameters.
WIPE=()
if [[ "${1:-}" == "--wipe" ]]; then
  WIPE=(--wipe-eeprom)
  shift
fi

exec sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --model JSON \
  --no-mavproxy \
  --no-rebuild \
  "${WIPE[@]}" \
  "$@"
