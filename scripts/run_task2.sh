#!/usr/bin/env bash
# Task 2: arm, climb to 5 m, hold 8 s, land - and record the same flight.
#
# Run this in the third terminal, after ./scripts/run_gazebo.sh and
# ./scripts/run_sitl.sh have printed GAZEBO_READY and SITL_READY. Starting it
# before either of those is refused immediately rather than a minute later:
# the two ports are checked first, so the terminal that is missing is named
# instead of surfacing as "no heartbeat". Once the link is up, the script
# waits for a 3-D GPS fix and the EKF flags before it sends an arm command,
# and prints the autopilot's own refusal text if the wait goes wrong.
#
# Everything this flight produces lands in one directory:
#   results/task2/video_<stamp>/takeoff_land.mp4    gimbal camera, 10 fps
#   results/task2/video_<stamp>/flight.csv          one row per position update
#   results/task2/video_<stamp>/flight.summary.json peak altitude, phases, mp4 frames
# With --no-video there is no mp4, so the directory is called flight_<stamp>
# instead of video_<stamp>.
# The CSV is the evidence and the summary is derived from it, which is why the
# acceptance test can compare the two against each other.
#
# Why the numbers are fixed here rather than left to the caller
#   The acceptance test asserts a peak altitude inside [4.5, 5.5] m, and the
#   hover has to be 8 s of autopilot time. README.md quotes the same flight.
#   A wrapper whose defaults disagreed with the documented result would
#   produce evidence that contradicts the document it is meant to support, so
#   the defaults are those numbers. Any argument given on the command line
#   still wins, which is what makes an experiment possible.
#
# Usage
#   ./scripts/run_task2.sh
#   ./scripts/run_task2.sh --altitude 6      # experiment only, breaks the test
#   ./scripts/run_task2.sh --no-video        # fly without writing an mp4
#   ./scripts/run_task2.sh --test            # the acceptance test instead
#   ./scripts/run_task2.sh --test --repeat 1 # one run of it, for a quick check
#
# Stop the pair afterwards with Ctrl-C in its two terminals, or:
#   ./scripts/stop_sim.sh

set -euo pipefail

# --help prints the Usage block from this file's header and stops. It runs
# before anything else, including the environment banner, so asking what a
# script does needs neither a simulator nor a configured shell. The text
# cannot drift from the header because it is the header.
case "${1:-}" in
  -h|--help)
    awk '/^# Usage$/{p=1; next} p && /^#?$/{exit} p{sub(/^# ?/,""); print}' \
      "${BASH_SOURCE[0]}"
    exit 0
    ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

# --test is this wrapper's other mode, and the reason Task 2 has one entry
# point: fly it, or prove it. The test starts and stops its own simulator
# pair twice from cold, so nothing else may be using the simulator - it does
# not clean up beforehand, because a warm machine is exactly the condition
# that hides a stale-SITL fault. Every flag after --test goes to the test.
if [[ "${1:-}" == "--test" ]]; then
  shift
  echo "Task 2 acceptance test"
  echo "  its own simulator pair per run, from cold"
  echo "  --repeat <n> runs in a row (default 2)"
  echo "  results: ${ROOT}/results/task2/"
  exec python3 "${ROOT}/scripts/test_task2.py" "$@"
fi

# --no-video is this wrapper's own flag. mavlink_link.py records only when
# --video <path> is given, so switching it off means not passing that pair of
# arguments at all. Everything else is passed through untouched, and a later
# --video on the command line overrides the default below.
VIDEO_ON=1
PASSTHROUGH=()
for arg in "$@"; do
  case "${arg}" in
    --no-video) VIDEO_ON=0 ;;
    *)          PASSTHROUGH+=("${arg}") ;;
  esac
done

# Refuse before anything else happens: no output directory, no gimbal call,
# no half-written evidence from a flight that could not have flown. See
# require_pair_for_flight() for why a port check and not a connection.
require_pair_for_flight || exit 1

# One directory per flight, named the same way the earlier evidence is named,
# so results/task2/ reads as a list of flights rather than a pile of files.
# The flag is parsed first because the name depends on it: a directory called
# video_<stamp> with no mp4 in it would be a small lie in the evidence.
STAMP="${STAMP:-$(date +%Y%m%d_%H%M%S)}"
if [[ "${VIDEO_ON}" -eq 1 ]]; then
  OUT="${ROOT}/results/task2/video_${STAMP}"
else
  OUT="${ROOT}/results/task2/flight_${STAMP}"
fi
mkdir -p "${OUT}"

VIDEO_ARGS=()
if [[ "${VIDEO_ON}" -eq 1 ]]; then
  VIDEO_ARGS=(--video "${OUT}/takeoff_land.mp4")
fi

echo "Task 2 - takeoff, hover, land"
echo "  evidence: ${OUT}"
echo "  waiting for GPS/EKF before arming"

# exec keeps one process for the whole flight, so Ctrl-C reaches the script
# directly and its cleanup still closes the video file.
exec python3 "${ROOT}/scripts/mavlink_link.py" \
  --altitude 5 \
  --hover 8 \
  --gimbal-pitch 0.7 \
  "${VIDEO_ARGS[@]}" \
  --log "${OUT}/flight.csv" \
  "${PASSTHROUGH[@]}"
