#!/usr/bin/env bash
# Shell environment and shared helpers for this project.
#
# Source this file in every terminal and from every wrapper:
#   source scripts/env.sh
#
# Two halves, both sourced and neither of them starting a process:
#   1. the environment   where Gazebo, SITL and the plugin keep their files,
#                        and the ROS strip that keeps this a no-ROS project
#   2. the helpers       "is it up yet", "which processes belong to this
#                        project", "stop them" - one definition each, so the
#                        run scripts, stop_sim.sh and the acceptance test
#                        cannot disagree about what ready means or what may
#                        be killed
#
# Why readiness is decided by a socket and not by a log line
#   The plugin reports its socket through gzlog, so there is no line on
#   Gazebo's stdout to wait for - and the messages that do reach stdout are
#   C++ iostreams, which are block-buffered when redirected to a file and
#   lose their tail entirely if the process has to be SIGKILLed. A socket
#   either exists or it does not, so a socket check is exact and returns in
#   milliseconds where a log-line wait sits out its whole timeout.

# This project must not use ROS. A login shell that has sourced
# /opt/ros/*/setup.bash puts ROS libraries on LD_LIBRARY_PATH and ROS Python
# packages on PYTHONPATH, which every process started from that shell then
# inherits. Removing them here is what makes the "no ROS" claim verifiable at
# runtime instead of only in the source:
#   - no ROS message package can be imported by accident,
#   - no ROS shared library can satisfy a symbol for gz or SITL,
#   - nothing below needs those variables.
# ROS_DISTRO and friends are unset so a stray `import rclpy` fails loudly
# rather than silently working.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH ROS_LOCALHOST_ONLY

# Drop any path element that points into a ROS installation. The loop is
# written out because it has to work in plain /bin/sh-compatible bash
# without external tools (grep/tr) that could be missing in a container.
_strip_ros_path() {
  local input="$1"
  local output=""
  local element
  local IFS=:
  for element in $input; do
    case "$element" in
      /opt/ros*|*ros*_ws*)
        # Matches /opt/ros/humble/... and any ROS workspace install
        # space such as ros2_ws/install. The pattern is deliberately
        # narrow: a bare *ros* would also match ordinary paths.
        continue
        ;;
    esac
    if [[ -n "$element" ]]; then
      output="${output:+${output}:}${element}"
    fi
  done
  printf '%s' "$output"
}
# Assign, then export, rather than exporting the substitution directly.
# `export X=$(...)` reports export's own status and hides a failing command
# substitution (shellcheck SC2155); the two-line form keeps that status
# visible and behaves identically for a caller.
PYTHONPATH="$(_strip_ros_path "${PYTHONPATH:-}")"
export PYTHONPATH
LD_LIBRARY_PATH="$(_strip_ros_path "${LD_LIBRARY_PATH:-}")"
export LD_LIBRARY_PATH

# CMake searches CMAKE_PREFIX_PATH for package configs. After a shell has
# sourced /opt/ros/*/setup.bash, that variable points into the ROS install,
# and ardupilot_gazebo's find_package(ament_cmake QUIET) then turns the
# plugin into a ROS package - which this project must not build. Strip ROS
# entries here as well, so cmake sees the same no-ROS environment as python
# and the dynamic linker. build.sh also disables that package lookup
# outright, because a build directory may have cached it earlier.
CMAKE_PREFIX_PATH="$(_strip_ros_path "${CMAKE_PREFIX_PATH:-}")"
export CMAKE_PREFIX_PATH

# Folder that contains scripts/, logs/, and third_party/.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Gazebo Harmonic (gz-sim 8) is the simulator this project targets, and
# ardupilot_gazebo's CMakeLists selects it when GZ_VERSION=harmonic.
export GZ_VERSION=harmonic

# Gazebo loads ArduPilotPlugin by filename. This directory is where
# `cmake --build` writes libArduPilotPlugin.so. Without this variable
# Gazebo starts, but the Iris model cannot talk to the flight controller.
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/third_party/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

# Worlds and models are referenced as model://iris_with_gimbal and by
# a world filename such as vio_runway.sdf. Gazebo searches this path for
# both. The repository's own worlds/ directory comes first: it holds
# vio_runway.sdf, the stock iris_runway world plus the VIO calibration
# targets described in that file. The upstream directory follows it so
# model://runway, model://iris_with_gimbal and the gimbal meshes still
# resolve.
export GZ_SIM_RESOURCE_PATH="${ROOT}/worlds:${ROOT}/third_party/ardupilot_gazebo/models:${ROOT}/third_party/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

# sim_vehicle.py lives here. Putting it on PATH means the run script can
# call it by name from any working directory.
export PATH="${ROOT}/third_party/ardupilot/Tools/autotest:${PATH}"

# RapidJSON headers live here when the system has no rapidjson-dev install:
# build.sh downloads the package and unpacks it into third_party/sysroot.
# CPATH is an extra include directory that gcc/g++ always search, so the
# plugin build finds the headers without any flag of its own.
export CPATH="${ROOT}/third_party/sysroot/usr/include${CPATH:+:${CPATH}}"

echo "insideFPV_task environment loaded"
echo "  GZ_VERSION=${GZ_VERSION}"
echo "  plugin path=${ROOT}/third_party/ardupilot_gazebo/build"
echo "  ardupilot=${ROOT}/third_party/ardupilot"


# ---------------------------------------------------------------------------
# Process tests and cleanup
# ---------------------------------------------------------------------------

# Every pattern here is anchored and path-specific on purpose: "arducopter"
# alone also matches an editor buffer and "gz sim" alone also matches an
# unrelated Gazebo session. A pattern that is too broad kills somebody else's
# work, which is worse than not killing anything.

# Absolute paths of the two binaries this project owns. Matching on the full
# path is what keeps "arducopter" from matching anything else on the box.
SITL_BINARY="${ROOT}/third_party/ardupilot/build/sitl/bin/arducopter"

# True when this project's SITL binary is running. A leftover SITL is the
# whole "No heartbeat" failure: it keeps TCP 5760 bound, so the
# fresh SITL cannot start, while the stale one answers the next client with
# state from an earlier attempt.
sitl_running() {
  pgrep -f "${SITL_BINARY}" >/dev/null 2>&1
}

# True when a Gazebo server is running this project's world. "runway" rather
# than one filename, because the project has used both worlds/iris_runway.sdf
# and worlds/vio_runway.sdf, and naming only one leaves the other alive on
# UDP 9002.
gazebo_running() {
  pgrep -f "gz sim .*runway\.sdf" >/dev/null 2>&1 ||
    pgrep -f "gz-sim-server.*runway\.sdf" >/dev/null 2>&1
}

# True when <proto> port <port> is bound. Only listening sockets are
# considered, which is deliberate: a TCP port still in TIME_WAIT after a
# clean shutdown appears in `ss -tan` but not in `ss -tln`, and treating
# that as "in use" would refuse to start a pair seconds after a correct
# stop.
#
# -l is also required for UDP. A bound-but-unconnected UDP socket counts as
# "listening" for ss, and without -l ss only lists connected ones, so the
# plugin's 9002 socket was invisible and every check failed while the port
# was in fact bound.
port_in_use() {
  local proto="$1" port="$2" flag listing
  case "${proto}" in
    tcp) flag="-tln" ;;
    udp) flag="-uln" ;;
    *)   echo "port_in_use: unknown protocol ${proto}" >&2; return 2 ;;
  esac
  listing="$(ss "${flag}" -H 2>/dev/null || true)"
  # The colon keeps 5760 from matching 15760; the trailing space matches the
  # column gap before the peer address, and the newline case catches a port
  # that is last on the line.
  case "${listing}" in
    *":${port} "*) return 0 ;;
    *":${port}"$'\n'*) return 0 ;;
  esac
  return 1
}

# PIDs that currently hold <proto> <port>, one per line.
#
# Needed because a process cannot always be named by its command line: the
# Gazebo server re-execs itself as plain "gz sim server", leaving the world
# filename behind in the parent it forked from. An aborted run can leave two
# such orphans holding UDP 9002 while every pattern in stop_sim.sh misses
# them. The port is the reliable identity: on this project 9002 is the
# plugin's <fdm_port_in> and 5760-5763 are SITL's SERIAL0-2, so whatever owns
# them is this project's simulator by construction.
port_owner_pids() {
  local proto="$1" port="$2" flag
  case "${proto}" in
    tcp) flag="-tlnp" ;;
    udp) flag="-ulnp" ;;
    *)   echo "port_owner_pids: unknown protocol ${proto}" >&2; return 2 ;;
  esac
  ss "${flag}" -H 2>/dev/null \
    | awk -v needle=":${port}" 'index($0, needle)' \
    | grep -o 'pid=[0-9]*' \
    | cut -d= -f2 \
    | sort -u \
    || true
}

# Stop whatever owns <proto> <port>, and make sure it actually stopped.
#
# Returns 0 when it stopped something, 1 when the port had no owner (the
# normal case, so callers can say "and nothing else was left behind").
#
# SIGTERM alone is not enough. SITL handles SIGTERM with a shutdown flag
# that its main loop polls, and that loop is parked inside accept() whenever
# no ground station is connected - so it never sees the flag and stays alive
# holding TCP 5760. The next pair then starts, finds 5760 already bound,
# refuses to start a second SITL, and the flight script connects to the
# stale one and reports "No heartbeat" after a timeout. A leftover that
# survives SIGTERM therefore has to be SIGKILLed, which is why this looks at
# the process again instead of printing "Stopped" the moment the signal is
# sent.
kill_port_owner() {
  local proto="$1" port="$2" pid pids waited alive
  pids="$(port_owner_pids "${proto}" "${port}")"
  [[ -z "${pids}" ]] && return 1

  for pid in ${pids}; do
    kill -TERM "${pid}" 2>/dev/null || true
  done

  # 2 s is generous for a SIGTERM handler; anything still alive at the end
  # of it is blocked somewhere that cannot run its handler.
  waited=0
  while (( waited < 20 )); do
    alive=0
    for pid in ${pids}; do
      kill -0 "${pid}" 2>/dev/null && alive=1
    done
    (( alive == 0 )) && break
    sleep 0.1
    waited=$(( waited + 1 ))
  done

  for pid in ${pids}; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL "${pid}" 2>/dev/null || true
      echo "Killed pid ${pid} holding ${proto} ${port} (ignored SIGTERM)"
    else
      echo "Stopped pid ${pid} holding ${proto} ${port}"
    fi
  done
  return 0
}

# True while a supervisor's own child is still running. Used after a socket
# wait, because a port can be bound by somebody else's process: a supervisor
# can announce "SITL is up" while the SITL it has just started is already
# dead, because the port it waited for belonged to a leftover.
child_running() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

# Ask a process to stop, then insist. SITL handles SIGTERM by flushing its
# DataFlash log, so it is never SIGKILLed first; Gazebo can be inside a
# blocking render-context call and ignore SIGTERM, which is why the wait is
# bounded instead of unbounded.
stop_gracefully() {
  local pid="$1" waited=0
  [[ -z "${pid}" ]] && return 0
  kill -0 "${pid}" 2>/dev/null || return 0
  kill -TERM "${pid}" 2>/dev/null || true
  while (( waited < 100 )); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.1
    waited=$(( waited + 1 ))
  done
  kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

# ---------------------------------------------------------------------------
# Readiness
# ---------------------------------------------------------------------------

# wait_for_socket <tcp|udp> <port> <timeout seconds> <label> [log file]
#
# Returns 0 as soon as the port is bound, 1 after the timeout with the log
# tail on stderr. The socket is the thing the next process actually needs:
#   UDP 9002 bound  = ArduPilotPlugin ran InitSockets(), so it will accept
#                     the JSON state packets SITL sends.
#   TCP 5760 bound  = SITL created SERIAL0 and is waiting in accept().
#
# The listing is captured and matched in bash rather than piped into
# `grep -q`: the callers run with `set -o pipefail`, and a grep -q that
# exits on its first match kills ss with SIGPIPE, turning success into a
# reported failure.
wait_for_socket() {
  local proto="$1" port="$2" timeout_s="$3" label="$4" log="${5:-}"
  local deadline=$(( SECONDS + timeout_s ))
  while (( SECONDS < deadline )); do
    if port_in_use "${proto}" "${port}"; then
      return 0
    fi
    sleep 0.1
  done
  echo "ERROR: ${label} did not bind ${proto} ${port} within ${timeout_s}s" >&2
  if [[ -n "${log}" && -f "${log}" ]]; then
    echo "---- ${log} (tail) ----" >&2
    tail -n 40 "${log}" >&2 || true
  fi
  return 1
}

# Refuse to start a flight while half the pair is missing.
#
# The flight programs connect once and then wait for a heartbeat, so if SITL
# was simply never started they can only report "no heartbeat" a minute
# later - and by that point the gimbal has already answered over gz-transport,
# which makes the link look healthy and sends the reader looking for the wrong
# cause. A port check costs a millisecond, names the terminal to open, and is
# safe with respect to SERIAL0's single client slot because it never connects.
#
# Returns 0 when both halves are up, 1 after printing what is missing.
require_pair_for_flight() {
  if ! port_in_use udp 9002; then
    echo "Gazebo is not ready: nothing is bound on UDP 9002." >&2
    echo "Start it in another terminal:  ./scripts/run_gazebo.sh" >&2
    echo "and wait for the GAZEBO_READY line." >&2
    return 1
  fi
  if ! port_in_use tcp 5760; then
    echo "SITL is not listening on TCP 5760." >&2
    echo "Start it in a second terminal:  ./scripts/run_sitl.sh" >&2
    echo "and wait for the SITL_READY line. It can be up a few seconds" >&2
    echo "before it answers the first heartbeat." >&2
    return 1
  fi
  return 0
}
