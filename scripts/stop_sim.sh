#!/usr/bin/env bash
# Stops a Gazebo + SITL pair, however it was started.
#
# Three shapes have to be covered, because each leaves a different process at
# the top of the tree:
#   ./scripts/run_sim.sh              one supervisor over both processes
#   ./scripts/run_gazebo.sh           a supervisor over Gazebo
#   ./scripts/run_sitl.sh             a supervisor over SITL
#
# This exists because SITL's single-client SERIAL0 makes a leftover
# flight controller worse than no flight controller: port 5760 stays
# open, so the next ground station connects successfully and then waits
# forever for a heartbeat that will never come. Use this script, or
# Ctrl-C in the terminal that started the pair, before every new flight.
#
# The patterns match only this project's simulator, so an unrelated
# Gazebo session on the machine is not touched.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

STOPPED=0

# Stops every process whose command line matches, and only says so once the
# process is actually gone. Sending SIGTERM and printing "Stopped" in the
# same breath is not enough: a SITL parked in accept() never runs its SIGTERM
# handler, so the message would be true of the signal and false of the
# process. stop_gracefully waits, then insists.
stop_pattern() {
  local pattern="$1"
  local label="$2"
  local pid
  for pid in $(pgrep -f "${pattern}" 2>/dev/null || true); do
    stop_gracefully "${pid}"
    echo "Stopped ${label} (pid ${pid})"
    STOPPED=1
  done
  return 0
}

# The world name is deliberately not part of the pattern: scripts/run_gazebo.sh
# has used both iris_runway.sdf and worlds/vio_runway.sdf, and a pattern that
# names one of them silently leaves the other running.
#
# Each pattern is anchored with ^ so that it matches the process's own
# command line and not an editor buffer, a grep, or a shell whose command
# happens to mention the script.
stop_pattern "^bash \.?/?scripts/run_sim\.sh" "simulator supervisor"
# Supervisors first, then the children. Stopping a supervisor lets it run its
# own cleanup and flush SITL's DataFlash log; the child patterns below are the
# fallback for a supervisor that has already been killed.
stop_pattern "^bash \.?/?scripts/run_gazebo\.sh" "Gazebo supervisor"
stop_pattern "^bash \.?/?scripts/run_sitl\.sh"   "SITL supervisor"
stop_pattern "third_party/ardupilot/build/sitl/bin/arducopter" "ArduCopter SITL"
stop_pattern "^gz sim .*runway\.sdf" "Gazebo"
stop_pattern "gz-sim-server.*runway\.sdf" "Gazebo server"

# Final sweep by port, because a command-line pattern cannot see a forked
# Gazebo server: it re-execs itself as plain "gz sim server" and keeps the
# world name only in the parent. Two such orphans hold UDP 9002 and make every
# later start refuse. The ports are the reliable identity - 9002 is the
# plugin's fdm port in the world file and 5760-5763 are SITL's SERIAL0-2 - so
# whatever holds them is this simulator.
kill_port_owner udp 9002 && STOPPED=1
for port in 5760 5762 5763; do
  kill_port_owner tcp "${port}" && STOPPED=1
done

if [[ "${STOPPED}" -eq 0 ]]; then
  echo "No simulator processes were running"
fi

# Wait briefly for sockets to be released before reporting. A caller that
# starts a new pair immediately after this script must not race the
# kernel's TIME_WAIT state on TCP 5760.
sleep 1
