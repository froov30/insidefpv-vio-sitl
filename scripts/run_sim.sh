#!/usr/bin/env bash
# Starts one fresh Gazebo + ArduCopter SITL pair, in the correct order,
# with process hygiene and timeouts, and holds them until Ctrl-C.
#
# Why this script exists
#   SITL's SERIAL0 is a single-client TCP server that accepts exactly one
#   ground station and never re-accepts (see scripts/run_sitl.sh). A SITL
#   left over from an earlier attempt therefore looks alive - port 5760
#   is still listed - while nobody can ever receive telemetry from it.
#   Every run in this project starts by killing such leftovers, which is
#   what turned "No heartbeat" from a guess into a reproducible state.
#
#   The two processes are also strictly ordered. ArduPilotPlugin has to
#   be loaded and its UDP socket bound before SITL's JSON link can come
#   online, so Gazebo goes first and this script waits for the plugin's
#   own "ArduPilot ready to fly" line before starting the flight code.
#
# Usage
#   bash scripts/run_sim.sh              # windowed Gazebo (record video)
#   bash scripts/run_sim.sh --headless   # server only, camera rendered
#   Ctrl-C stops both processes.
#
# Outputs
#   logs/sim/gazebo.log        Gazebo console, including plugin messages
#   logs/sim/sitl_console.log  flight controller console: EKF, PreArm, ...
#   "SIM_READY" on stdout once both are up and SITL is waiting for a GCS.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

LOG_DIR="${ROOT}/logs/sim"
mkdir -p "${LOG_DIR}"

# How long each process may take to reach its readiness marker. Gazebo has
# to build an Ogre2 render context on this machine, which is software
# rendering (llvmpipe) and takes 20-90 s depending on what else is running.
# 180 s is generous but finite: if the marker is not reached the script
# prints the log tail and exits non-zero instead of leaving a half-started
# pair behind. Override with GAZEBO_READY_TIMEOUT_S for a slower machine.
GAZEBO_READY_TIMEOUT_S="${GAZEBO_READY_TIMEOUT_S:-180}"
SITL_READY_TIMEOUT_S="${SITL_READY_TIMEOUT_S:-120}"

HEADLESS=""
if [[ "${1:-}" == "--headless" ]]; then
  HEADLESS="--headless"
  shift
fi

# Kill only processes that belong to this project's simulation. The
# patterns are specific on purpose: "arducopter" alone would also match an
# editor buffer name, and "gz sim" alone would match another Gazebo.
kill_stale() {
  pkill -f "third_party/ardupilot/build/sitl/bin/arducopter" 2>/dev/null || true
  # "runway" rather than a specific filename: the project has run both
  # worlds/iris_runway.sdf and worlds/vio_runway.sdf, and naming only one
  # leaves the other alive on UDP 9002.
  pkill -f "gz sim .*runway\.sdf" 2>/dev/null || true
  pkill -f "gz-sim-server.*runway\.sdf" 2>/dev/null || true
  # Give the kernel a moment to release TCP 5760 and UDP 9002. Without
  # this, the next bind can fail with "Address already in use" and the
  # failure looks like a plugin problem.
  sleep 1
}

kill_stale

GAZEBO_PID=""
SITL_PID=""

cleanup() {
  # Called on Ctrl-C, on any exit path, and after one process dies. Both
  # children are asked to stop; SITL handles SIGTERM and flushes its
  # DataFlash log, which is why it is not SIGKILLed first.
  if [[ -n "${SITL_PID}" ]] && kill -0 "${SITL_PID}" 2>/dev/null; then
    kill -TERM "${SITL_PID}" 2>/dev/null || true
  fi
  if [[ -n "${GAZEBO_PID}" ]] && kill -0 "${GAZEBO_PID}" 2>/dev/null; then
    kill -TERM "${GAZEBO_PID}" 2>/dev/null || true
  fi
  # Gazebo can be inside a blocking render-context call when it is signalled
  # and ignore SIGTERM for a long time; waiting on it without a limit is how
  # an earlier version of this script hung forever after reporting an error.
  # Ten seconds is enough for SITL's log flush, after which the pair is
  # killed.
  local waited=0
  while (( waited < 100 )); do
    local alive=0
    if [[ -n "${SITL_PID}" ]] && kill -0 "${SITL_PID}" 2>/dev/null; then alive=1; fi
    if [[ -n "${GAZEBO_PID}" ]] && kill -0 "${GAZEBO_PID}" 2>/dev/null; then alive=1; fi
    if (( alive == 0 )); then break; fi
    sleep 0.1
    waited=$(( waited + 1 ))
  done
  if [[ -n "${SITL_PID}" ]] && kill -0 "${SITL_PID}" 2>/dev/null; then
    kill -KILL "${SITL_PID}" 2>/dev/null || true
  fi
  if [[ -n "${GAZEBO_PID}" ]] && kill -0 "${GAZEBO_PID}" 2>/dev/null; then
    kill -KILL "${GAZEBO_PID}" 2>/dev/null || true
  fi
  [[ -n "${SITL_PID}" ]] && wait "${SITL_PID}" 2>/dev/null || true
  [[ -n "${GAZEBO_PID}" ]] && wait "${GAZEBO_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_socket() {
  # wait_for_socket <tcp|udp> <port> <timeout seconds> <label> <log file>
  #
  # Readiness is decided by whether the socket exists, not by a line in a
  # log. The socket is the thing the next process actually needs:
  #   UDP 9002 bound  = ArduPilotPlugin ran InitSockets(), so it will
  #                     accept the JSON state packets SITL sends.
  #   TCP 5760 bound  = SITL created SERIAL0 and is blocked in accept().
  # Reading the log was the original approach and produced a false failure:
  # Gazebo had been ready for 0.4 s while its stdout sat in a block buffer.
  local proto="$1" port="$2" timeout_s="$3" label="$4" log="$5"
  local flag listing
  local deadline=$(( SECONDS + timeout_s ))
  # -l is needed for both protocols. A bound-but-unconnected UDP socket is
  # a "listening" socket as far as ss is concerned, and without -l ss only
  # shows connected ones, so the plugin's UDP 9002 socket was invisible and
  # every check failed while the port was in fact bound.
  case "${proto}" in
    tcp) flag="-tln" ;;
    udp) flag="-uln" ;;
    *)   echo "ERROR: unknown protocol ${proto}" >&2; return 1 ;;
  esac
  while (( SECONDS < deadline )); do
    # The listing is captured and matched in bash, not piped into
    # `grep -q`. This script runs with `set -o pipefail`, and a grep -q that
    # exits at the first match kills ss with SIGPIPE, which makes the whole
    # pipeline report failure.
    listing="$(ss "${flag}" -H 2>/dev/null || true)"
    # The colon keeps 5760 from matching 15760, and the trailing space
    # matches the column gap before the peer address.
    case "${listing}" in
      *":${port} "*) return 0 ;;
      *":${port}"$'\n'*) return 0 ;;
    esac
    sleep 0.1
  done
  echo "ERROR: ${label} did not bind ${proto} ${port} within ${timeout_s}s" >&2
  echo "---- ${log} (tail) ----" >&2
  tail -n 40 "${log}" >&2 || true
  return 1
}

echo "Starting Gazebo ${HEADLESS:-(windowed)}"
# shellcheck disable=SC2086
bash "${ROOT}/scripts/run_gazebo.sh" ${HEADLESS} > "${LOG_DIR}/gazebo.log" 2>&1 &
GAZEBO_PID=$!

# The plugin binds UDP 9002 only after it has found the IMU, the four rotor
# joints and the SDF parameters. Waiting for the socket removes the race
# where SITL starts first and its first JSON frames are lost.
wait_for_socket udp 9002 "${GAZEBO_READY_TIMEOUT_S}" "Gazebo/ArduPilotPlugin" \
  "${LOG_DIR}/gazebo.log"
echo "Gazebo is up (pid ${GAZEBO_PID})"

echo "Starting ArduCopter SITL"
# stdbuf for the same reason as Gazebo: the console is redirected to a file
# and must still be readable while SITL runs, because this log is Task 2's
# evidence for the EKF and for the accept() wait.
stdbuf -oL -eL bash "${ROOT}/scripts/run_sitl.sh" \
  > "${LOG_DIR}/sitl_console.log" 2>&1 &
SITL_PID=$!

# SERIAL0 listening on 5760 means the flight code has reached accept() and
# the next ground station to connect becomes its one and only client.
wait_for_socket tcp 5760 "${SITL_READY_TIMEOUT_S}" "ArduCopter SITL" \
  "${LOG_DIR}/sitl_console.log"
echo "SITL is up (pid ${SITL_PID})"

echo "SIM_READY"
echo "  gazebo log: ${LOG_DIR}/gazebo.log"
echo "  sitl log:   ${LOG_DIR}/sitl_console.log"
echo "  ground station: python3 scripts/takeoff_land.py"

# Hold the pair. If either process exits, stop the other and return its
# status so a crash never leaves a half-running simulation behind.
set +e
wait -n "${GAZEBO_PID}" "${SITL_PID}"
STATUS=$?
set -e
echo "A simulator process exited (status ${STATUS}); stopping the pair"
exit "${STATUS}"
