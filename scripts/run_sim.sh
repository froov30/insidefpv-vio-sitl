#!/usr/bin/env bash
# Starts one fresh Gazebo + ArduCopter SITL pair, in the correct order,
# with process hygiene and timeouts, and holds them until Ctrl-C.
#
# Why this script exists
#   A SITL left over from an earlier attempt keeps TCP 5760 and answers the
#   next client with state from that attempt - or, if the Gazebo it was
#   paired with is gone, with no heartbeat at all (see scripts/run_sitl.sh).
#   It cannot simply be replaced either, because a fresh SITL cannot bind
#   5760 while it holds the port. Every run therefore starts by killing such
#   leftovers, which is what makes "No heartbeat" a reproducible state rather
#   than an intermittent one.
#
#   The two processes are also strictly ordered. ArduPilotPlugin has to
#   be loaded and its UDP socket bound before SITL's JSON link can come
#   online, so Gazebo goes first and this script waits until UDP 9002 is
#   bound (wait_for_socket) before starting the flight code. No log line is
#   used as the trigger: this plugin reports its bind through gzlog, which
#   is not on stdout at all.
#
#   The waits below prove that *a* socket exists, not whose it is. That gap
#   allows a false readiness: a leftover SITL blocked in accept() can ignore
#   SIGTERM, survive cleanup, keep TCP 5760 listening, and let this script
#   announce "SITL is up" while the SITL it has just started has exited with
#   "already running". Hence two guards: refuse to start if any simulation
#   port is still held after cleanup, and after each socket wait confirm that
#   the child this script started is still alive.
#
# Usage
#   ./scripts/run_sim.sh              # windowed Gazebo (record video)
#   ./scripts/run_sim.sh --headless   # server only, camera rendered
#   Ctrl-C stops both processes.
#
# For the demo, the three-terminal shape is the one to show, because each
# process prints its own readiness line and the audience can see the order:
#   ./scripts/run_gazebo.sh   ->  ./scripts/run_sitl.sh   ->  ./scripts/run_task2.sh
# This script remains the one-command form used by scripts/test_task2.py.
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
# to build an Ogre2 render context, which without a GPU falls back to
# software rendering (llvmpipe) and takes 20-90 s depending on what else is
# running. 180 s is generous but finite: if the marker is not reached the
# script prints the log tail and exits non-zero instead of leaving a
# half-started pair behind. Override with GAZEBO_READY_TIMEOUT_S for a slower
# machine.
GAZEBO_READY_TIMEOUT_S="${GAZEBO_READY_TIMEOUT_S:-180}"
SITL_READY_TIMEOUT_S="${SITL_READY_TIMEOUT_S:-120}"

HEADLESS=""
if [[ "${1:-}" == "--headless" ]]; then
  HEADLESS="--headless"
  shift
fi

# Kill only processes that belong to this project's simulation. The paths and
# patterns live in env.sh so that stop_sim.sh, run_sim.sh and the two
# individual scripts cannot drift apart.
kill_stale() {
  pkill -f "${SITL_BINARY}" 2>/dev/null || true
  pkill -f "gz sim .*runway\.sdf" 2>/dev/null || true
  pkill -f "gz-sim-server.*runway\.sdf" 2>/dev/null || true
  # A forked Gazebo server carries no world name on its command line, so the
  # patterns above cannot see it; the port it holds can. Without this, an
  # orphan from a crashed run makes every later start fail with "already in
  # use", which is exactly what the acceptance test cannot tolerate.
  kill_port_owner udp 9002 >/dev/null 2>&1 || true
  kill_port_owner tcp 5760 >/dev/null 2>&1 || true
  # Give the kernel a moment to release TCP 5760 and UDP 9002. Without
  # this, the next bind can fail with "Address already in use" and the
  # failure looks like a plugin problem.
  sleep 1
}

kill_stale

# Cleanup cannot beat a process that refuses to die, and the socket waits
# below would then succeed against the wrong process. Say which port is still
# held and stop, rather than reporting a pair that does not exist.
if port_in_use udp 9002 || port_in_use tcp 5760; then
  echo "A leftover simulator still holds a simulation port after cleanup." >&2
  ss -ulnp 2>/dev/null | grep -E ':9002' >&2 || true
  ss -tlnp 2>/dev/null | grep -E ':576[0-3]' >&2 || true
  echo "Stop it by hand, then run this script again." >&2
  exit 1
fi

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
  # and ignore SIGTERM for a long time; waiting on it without a limit hangs
  # the script forever after it has already reported an error. Ten seconds is
  # enough for SITL's log flush, after which the pair is killed.
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
  if [[ -n "${SITL_PID}" ]]; then wait "${SITL_PID}" 2>/dev/null || true; fi
  if [[ -n "${GAZEBO_PID}" ]]; then wait "${GAZEBO_PID}" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

# wait_for_socket() lives in env.sh, shared with run_gazebo.sh, run_sitl.sh
# and stop_sim.sh. It decides readiness from the socket rather than from a log
# line, because the socket is the thing the next process actually needs:
#   UDP 9002 bound  = ArduPilotPlugin ran InitSockets(), so it will accept
#                     the JSON state packets SITL sends.
#   TCP 5760 bound  = SITL created SERIAL0 and is blocked in accept().
# Reading the log instead produces false failures: Gazebo can be ready for
# seconds while its stdout sits in a block buffer.

echo "Starting Gazebo ${HEADLESS:-(windowed)}"
# shellcheck disable=SC2086
bash "${ROOT}/scripts/run_gazebo.sh" ${HEADLESS} > "${LOG_DIR}/gazebo.log" 2>&1 &
GAZEBO_PID=$!

# The plugin binds UDP 9002 at the end of its Configure(), and SITL needs
# that socket to exist before its first JSON frame, so waiting here removes
# the race where SITL starts first and those frames are lost.
wait_for_socket udp 9002 "${GAZEBO_READY_TIMEOUT_S}" "Gazebo/ArduPilotPlugin" \
  "${LOG_DIR}/gazebo.log"
if ! child_running "${GAZEBO_PID}"; then
  echo "The Gazebo supervisor exited instead of starting Gazebo." >&2
  tail -n 20 "${LOG_DIR}/gazebo.log" >&2 || true
  exit 1
fi
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
if ! child_running "${SITL_PID}"; then
  echo "The SITL supervisor exited instead of starting SITL:" >&2
  tail -n 20 "${LOG_DIR}/sitl_console.log" >&2 || true
  exit 1
fi
echo "SITL is up (pid ${SITL_PID})"

echo "SIM_READY"
echo "  gazebo log: ${LOG_DIR}/gazebo.log"
echo "  sitl log:   ${LOG_DIR}/sitl_console.log"
echo "  ground station: python3 scripts/mavlink_link.py"

# Hold the pair. If either process exits, stop the other and return its
# status so a crash never leaves a half-running simulation behind.
set +e
wait -n "${GAZEBO_PID}" "${SITL_PID}"
STATUS=$?
set -e
echo "A simulator process exited (status ${STATUS}); stopping the pair"
exit "${STATUS}"
