#!/usr/bin/env bash
# Starts the Gazebo world that contains the Iris quadcopter.
#
# Run this in its own terminal, wait for the GAZEBO_READY line, and leave it
# open. The window is the video you record for the takeoff and landing.
#
# Usage
#   ./scripts/run_gazebo.sh              # windowed, the video run
#   ./scripts/run_gazebo.sh --headless   # server only, camera still renders
#   Ctrl-C stops Gazebo and nothing else.
#
# The three-terminal shape this project uses
#   T1  ./scripts/run_gazebo.sh     -> GAZEBO_READY
#   T2  ./scripts/run_sitl.sh       -> SITL_READY (waits for T1 by itself)
#   T3  ./scripts/run_task2.sh      -> flies, records, writes evidence
# They may be started in any order: each one waits for the socket the other
# owns instead of guessing that the other is up.
#
# What this process owns:
#   - The physics step (1 ms, from iris_runway.sdf).
#   - The Iris body, four rotors, IMU, GPS, barometer, and gimbal camera.
#   - ArduPilotPlugin, which waits on UDP port 9002 for the flight controller.
#
# It does not arm, take off, or land. That is scripts/mavlink_link.py (its
# ground-station half).

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

PLUGIN="${ROOT}/third_party/ardupilot_gazebo/build/libArduPilotPlugin.so"

# How long Gazebo may take to bind the plugin socket. It has to build an
# Ogre2 render context, which without a GPU falls back to software rendering
# (llvmpipe) and takes 20-90 s depending on load. 180 s is generous but
# finite, so a broken environment fails with a message instead of hanging.
GAZEBO_READY_TIMEOUT_S="${GAZEBO_READY_TIMEOUT_S:-180}"
if [[ ! -f "${PLUGIN}" ]]; then
  echo "ArduPilotPlugin is not built: ${PLUGIN}" >&2
  echo "Build it with: bash scripts/build.sh plugin" >&2
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
HEADLESS=""
if [[ "${1:-}" == "--headless" ]]; then
  HEADLESS="--headless"
  shift
fi

# Refuse rather than kill. A Gazebo already running is either another
# terminal the caller forgot about, or a leftover from a crashed run. Killing
# it here would end someone's recording mid-flight; asking the caller to stop
# it explicitly costs one command and can never destroy work.
if gazebo_running; then
  echo "A Gazebo for this world is already running." >&2
  echo "Stop it with: ./scripts/stop_sim.sh" >&2
  exit 1
fi
if port_in_use udp 9002; then
  echo "UDP 9002 is already bound: another process owns the plugin port." >&2
  echo "Stop the previous simulator with: ./scripts/stop_sim.sh" >&2
  exit 1
fi

# stdbuf -oL -eL matters when the caller redirects this to a log file (which
# run_sim.sh does). C++ iostreams are block-buffered on a file, so Gazebo's
# messages sit in a 4 KB buffer and a reader sees nothing while the events
# are happening. Line buffering makes the console - and the log - live.
if [[ -n "${HEADLESS}" ]]; then
  GZ_ARGS=(-s -r --headless-rendering -v4 vio_runway.sdf)
else
  GZ_ARGS=(-r -v4 vio_runway.sdf)
fi

stdbuf -oL -eL gz sim "${GZ_ARGS[@]}" &
GZ_PID=$!

# Ctrl-C, a crash, or a supervisor's SIGTERM all end in the same place: the
# server is asked to stop, then killed if it is stuck inside a render call.
cleanup() {
  stop_gracefully "${GZ_PID}"
}
trap cleanup EXIT INT TERM

echo "Starting Gazebo ${HEADLESS:-(windowed)} (pid ${GZ_PID})"

# The socket, not a log line, is the readiness signal. The plugin binds
# <fdm_port_in> at the end of Configure(), once it has parsed the SDF and its
# control channels, and it reports that through gzlog, which goes to
# Gazebo's log stream rather than to this console - so there is no stdout
# line to grep for even ignoring buffering. Getting here means the plugin is
# loaded and its socket exists. The IMU topic and the rotor wiring are
# resolved on the first simulation step (PreUpdate), and a JSON packet that
# SITL sends in between simply waits in the socket's receive buffer.
if ! wait_for_socket udp 9002 "${GAZEBO_READY_TIMEOUT_S}" \
     "Gazebo/ArduPilotPlugin" ""; then
  echo "Gazebo started but the plugin never bound UDP 9002." >&2
  echo "Check the messages above for a plugin load error, then run:" >&2
  echo "  bash scripts/build.sh plugin" >&2
  exit 1
fi

echo
echo "GAZEBO_READY  (world vio_runway, plugin bound on UDP 9002)"
echo "  leave this terminal open; Ctrl-C stops Gazebo"
echo "  next, in a second terminal: ./scripts/run_sitl.sh"

set +e
wait "${GZ_PID}"
STATUS=$?
set -e
exit "${STATUS}"
