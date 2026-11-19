#!/usr/bin/env bash
# Stops a Gazebo + SITL pair started by scripts/run_sim.sh.
#
# This exists because SITL's single-client SERIAL0 makes a leftover
# flight controller worse than no flight controller: port 5760 stays
# open, so the next ground station connects successfully and then waits
# forever for a heartbeat that will never come. Use this script, or
# Ctrl-C in the run_sim.sh terminal, before every new flight.
#
# The patterns match only this project's simulator, so an unrelated
# Gazebo session on the machine is not touched.

set -euo pipefail

STOPPED=0

stop_pattern() {
  local pattern="$1"
  local label="$2"
  if pgrep -f "${pattern}" > /dev/null 2>&1; then
    pkill -TERM -f "${pattern}" 2>/dev/null || true
    echo "Stopped ${label}"
    STOPPED=1
  fi
}

# The world name is deliberately not part of the pattern: scripts/run_gazebo.sh
# has used both iris_runway.sdf and worlds/vio_runway.sdf, and a pattern that
# names one of them silently leaves the other running.
#
# Each pattern is anchored with ^ so that it matches the process's own
# command line and not an editor buffer, a grep, or a shell whose command
# happens to mention the script.
stop_pattern "^bash scripts/run_sim.sh" "simulator supervisor"
stop_pattern "^${ROOT:-.}/scripts/run_sim.sh" "simulator supervisor"
stop_pattern "third_party/ardupilot/build/sitl/bin/arducopter" "ArduCopter SITL"
stop_pattern "^gz sim .*runway\.sdf" "Gazebo"
stop_pattern "gz-sim-server.*runway\.sdf" "Gazebo server"

if [[ "${STOPPED}" -eq 0 ]]; then
  echo "No simulator processes were running"
fi

# Wait briefly for sockets to be released before reporting. A caller that
# starts a new pair immediately after this script must not race the
# kernel's TIME_WAIT state on TCP 5760.
sleep 1
