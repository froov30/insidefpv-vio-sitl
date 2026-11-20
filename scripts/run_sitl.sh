#!/usr/bin/env bash
# Starts ArduCopter SITL and links it to the Gazebo Iris.
#
# This process is the flight controller. It has no joystick and no ground
# station: scripts/mavlink_link.py is the ground station and it connects to
# TCP 127.0.0.1:5760.
#
# Usage
#   ./scripts/run_sitl.sh            # start, wait for SERIAL0, then hold
#   ./scripts/run_sitl.sh --wipe     # same, but discard the stored params
#   Ctrl-C stops SITL; it flushes its DataFlash log on the way out.
#
# It waits for Gazebo's UDP 9002 before starting, so this terminal and
# run_gazebo.sh may be started in either order. That wait also removes the
# race where SITL starts first and its first JSON frames are lost.
#
# Decision: run build/sitl/bin/arducopter directly, not sim_vehicle.py
#   sim_vehicle.py -v ArduCopter -f gazebo-iris is a wrapper. For that
#   frame name (Tools/autotest/pysim/vehicleinfo.json, "gazebo-iris") it
#   only chooses the binary and two --defaults files, then opens the
#   flight-controller console in an xterm. The xterm is the problem: the
#   "Waiting for connection" line, the EKF start-up lines and every PreArm
#   rejection go to a window instead of to a file, so a finished run cannot
#   be inspected afterwards. This script runs the identical binary with the
#   identical arguments and leaves stdout+stderr on the caller's terminal, so
#   run_sim.sh and the tests can capture them. The equivalent command line
#   is:
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
#   and blocks in accept() until a ground station connects. It serves one
#   client at a time and does re-accept after a disconnect: the console
#   prints "Closed connection on SERIAL0", and a later client is taken by
#   UARTDriver::_check_connection() ("New connection on SERIAL0").
#   The danger is not a missing re-accept, it is a stale SITL. One left
#   running from an earlier attempt keeps 5760 bound, so a fresh SITL
#   cannot start ("An ArduCopter SITL is already running") while the old
#   process answers the next client with state from that attempt - or with
#   nothing at all if its Gazebo is gone. That is what produces the
#   "No heartbeat" failure.
#   Always start SITL for each flight, and let exactly one client connect.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

BINARY="${ROOT}/third_party/ardupilot/build/sitl/bin/arducopter"
if [[ ! -x "${BINARY}" ]]; then
  echo "ArduCopter SITL is not built: ${BINARY}" >&2
  echo "Build it with: bash scripts/build.sh sitl" >&2
  exit 1
fi

# How long to wait for Gazebo's plugin socket, and how long SITL may take to
# bind SERIAL0. Both are finite so a broken environment fails with a message
# rather than hanging in a terminal nobody is watching.
PLUGIN_WAIT_S="${PLUGIN_WAIT_S:-180}"
SITL_READY_TIMEOUT_S="${SITL_READY_TIMEOUT_S:-120}"

# Refuse rather than kill. A leftover SITL is the single worst failure in this
# project: it keeps 5760 open, the next client's connect() succeeds from the
# listen backlog, and it then waits forever for a heartbeat. Starting a second
# copy on top of that only makes the confusion harder to read, so this script
# stops and names the fix instead.
if sitl_running; then
  echo "An ArduCopter SITL is already running." >&2
  echo "It keeps 5760 bound and would answer with state from that attempt." >&2
  echo "Stop the previous run first:  ./scripts/stop_sim.sh" >&2
  exit 1
fi
for port in 5760 5762 5763; do
  if port_in_use tcp "${port}"; then
    echo "TCP ${port} is already bound by another process." >&2
    echo "Stop the previous simulator with: ./scripts/stop_sim.sh" >&2
    exit 1
  fi
done

# Order independence: the plugin owns 9002 and SITL needs it before its first
# JSON frame, so waiting here is what lets the two terminals start in either
# order without losing samples.
echo "Waiting for Gazebo's plugin socket on UDP 9002 ..."
if ! wait_for_socket udp 9002 "${PLUGIN_WAIT_S}" "Gazebo/ArduPilotPlugin" ""; then
  echo "Nothing is listening on UDP 9002, so Gazebo is not up." >&2
  echo "Start it first, in another terminal:" >&2
  echo "  ./scripts/run_gazebo.sh" >&2
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

# stdbuf for the same reason as Gazebo: the console is redirected to a file
# by run_sim.sh, and it must still be readable while SITL runs, because this
# log is Task 2's evidence for the JSON handshake and for the accept() wait.
stdbuf -oL -eL "${BINARY}" \
  --model JSON \
  --speedup 1 \
  --slave 0 \
  --defaults "${ROOT}/third_party/ardupilot/Tools/autotest/default_params/copter.parm,${ROOT}/third_party/ardupilot/Tools/autotest/default_params/gazebo-iris.parm" \
  --sim-address=127.0.0.1 \
  -I0 \
  "${WIPE[@]}" &
SITL_PID=$!

cleanup() {
  stop_gracefully "${SITL_PID}"
}
trap cleanup EXIT INT TERM

# SERIAL0 listening on 5760 means the flight code has reached accept(): the
# next client to connect is served, one client at a time.
if ! wait_for_socket tcp 5760 "${SITL_READY_TIMEOUT_S}" \
     "ArduCopter SERIAL0" ""; then
  echo "SITL started but SERIAL0 never bound TCP 5760." >&2
  echo "Read the messages above; the usual cause is a second SITL or a" >&2
  echo "Gazebo that is not running this world." >&2
  exit 1
fi

echo
echo "SITL_READY  (SERIAL0 tcp:5760, logging endpoints 5762 and 5763)"
echo "  GPS and EKF health are reported to the flight script, not here."
echo "  SERIAL0 serves one client at a time, so a readiness probe here would"
echo "  hold the slot the ground station is about to need - the wait lives"
echo "  in the script that will do the flying."
echo "  next, in a third terminal: ./scripts/run_task2.sh"
echo "  or for the dataset run:     ./scripts/run_task3_record.sh"

set +e
wait "${SITL_PID}"
STATUS=$?
set -e
exit "${STATUS}"
