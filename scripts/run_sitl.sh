#!/usr/bin/env bash
# Starts ArduCopter SITL and links it to the Gazebo Iris.
#
# Start scripts/run_gazebo.sh first. This process is the flight controller.
# It has no joystick and no ground station: scripts/takeoff_land.py is the
# ground station and it connects to TCP 127.0.0.1:5760.
#
# Decision: run build/sitl/bin/arducopter directly, not sim_vehicle.py
#   sim_vehicle.py -v ArduCopter -f gazebo-iris is a wrapper. For that
#   frame name (Tools/autotest/pysim/vehicleinfo.json, "gazebo-iris") it
#   only chooses the binary and two --defaults files, then opens the
#   flight-controller console in an xterm. The xterm is why the earlier
#   failing runs left no evidence: "Waiting for connection", the EKF
#   start-up lines and every PreArm rejection went to a window, not to a
#   file. This script runs the identical binary with the identical
#   arguments and leaves stdout+stderr on the caller's terminal, so
#   run_sim.sh and the tests can capture them. The equivalent command
#   line printed by sim_vehicle.py in the original run was:
#     arducopter --model JSON --speedup 1 --slave 0 \
#       --defaults <copter.parm>,<gazebo-iris.parm> \
#       --sim-address=127.0.0.1 -I0
#
# Decision: --model JSON
#   The default SITL model flies an internal multicopter that Gazebo never
#   sees. JSON tells SITL that Gazebo owns the physics. Sensor packets come
#   in and motor commands go out as JSON over UDP; ArduPilotPlugIn listens
#   on <fdm_port_in> 9002 in models/iris_with_gimbal/model.sdf.
#
# Decision: two --defaults files
#   copter.parm is the normal ArduCopter starting parameter set.
#   gazebo-iris.parm sets FRAME_CLASS=1 (quad) and FRAME_TYPE=1 (X), which
#   matches the Iris motor order in the SDF.
#
# Decision: -I0
#   Instance 0. It selects base TCP port 5760 for SERIAL0 and is what the
#   ground station connection string assumes.
#
# SERIAL0 protocol, and the one rule this script cannot enforce
#   SITL's SERIAL0 default is "tcp:0:wait" (libraries/AP_HAL_SITL/
#   SITL_State.h). SITL binds 5760, prints "Waiting for connection ...."
#   and blocks in accept() until a ground station connects. It accepts
#   ONE client and never re-accepts after that client disconnects. A SITL
#   left running from an earlier attempt therefore keeps port 5760 open
#   in the kernel backlog while the flight controller has moved on, so a
#   new client "connects" and then receives nothing. That is the failure
#   that produced "No heartbeat" in the first runs of this project.
#   Always start SITL for each flight, and let exactly one client connect.

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

# eeprom.bin and the DataFlash .BIN files live in the working directory.
# Keeping them in logs/sitl keeps the source tree clean, and a re-run
# reuses the same parameter storage unless --wipe is given.
mkdir -p "${ROOT}/logs/sitl"
cd "${ROOT}/logs/sitl"

# --wipe erases eeprom.bin, so every parameter falls back to the two
# --defaults files. Use it when a bad parameter is suspected, or when the
# aircraft behaves differently from the .parm files. It does not change
# the flight you are about to fly.
WIPE=()
if [[ "${1:-}" == "--wipe" ]]; then
  WIPE=(--wipe)
fi

exec "${BINARY}" \
  --model JSON \
  --speedup 1 \
  --slave 0 \
  --defaults "${ROOT}/third_party/ardupilot/Tools/autotest/default_params/copter.parm,${ROOT}/third_party/ardupilot/Tools/autotest/default_params/gazebo-iris.parm" \
  --sim-address=127.0.0.1 \
  -I0 \
  "${WIPE[@]}"
