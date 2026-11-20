#!/usr/bin/env bash
# Task 3, first half: fly the square and record the dataset.
#
# One process, deliberately. The flight and the recording are the same event:
# the command link flies it, a second MAVLink link logs it, and the camera
# subscription is already live before the first leg starts. Splitting those
# into two programs would mean the recording could start late, stop early, or
# run against a flight it was not told about - and the dataset would then be a
# record of two overlapping but different things.
#
# Run this in the third terminal, after ./scripts/run_gazebo.sh and
# ./scripts/run_sitl.sh. Both ports are checked before anything starts, so a
# half-started pair is refused on the spot instead of after a timeout. It
# waits for GPS/EKF, then:
#   climb to 5 m, four 8 m legs (north, east, south, west), 2 s hold at each
#   corner, land, 2 s more recording after the motors disarm.
#
# Output: results/task3/dataset_<stamp>/
#   images/000001.png ...   lossless frames at the sensor rate. Written here
#                           and not committed: ~27 MB per recording
#   frames.csv              frame index, sim time, wall time, autopilot time
#   imu.csv                 RAW_IMU and SCALED_IMU2 at 50 Hz
#   attitude.csv            ATTITUDE at 20 Hz
#   local_position.csv      LOCAL_POSITION_NED at 10 Hz (the metric scale)
#   gps_raw.csv             GPS_RAW_INT at 5 Hz (the reference track)
#   dataset.json            intrinsics, achieved rates, clock offset, counts
#
# Usage
#   ./scripts/run_task3_record.sh           # the committed flight
#   ./scripts/run_task3_record.sh --leg 6   # experiment only
#
# Exit status is 0 when the flight completed and 1 when it was cut short; a
# partial dataset is still written and still usable, because CsvWriter flushes
# every row.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

# Refuse before python starts: if SITL is missing, record_dataset.py would
# spend a minute failing to connect and leave nothing behind.
require_pair_for_flight || exit 1

# Snapshot the datasets that already exist, so "the dataset just written" can
# be identified by difference instead of by "newest". If the flight dies before
# creating its directory, newest is the previous run, and printing that path
# would point a reviewer at evidence this run did not produce.
BEFORE="$(find "${ROOT}/results/task3" -maxdepth 1 -type d -name 'dataset_*' \
  2>/dev/null | sort | tail -n 1 || true)"

echo "Task 3 - square flight and dataset recording"
echo "  waiting for GPS/EKF before arming"
echo "  Ctrl-C aborts the flight; the partial dataset stays on disk"

set +e
python3 "${ROOT}/scripts/record_dataset.py" "$@"
STATUS=$?
set -e

AFTER="$(find "${ROOT}/results/task3" -maxdepth 1 -type d -name 'dataset_*' \
  2>/dev/null | sort | tail -n 1 || true)"

if [[ -n "${AFTER}" && "${AFTER}" != "${BEFORE}" ]]; then
  echo
  echo "Dataset: ${AFTER}"
fi
echo "Next:  ./scripts/run_task3_vio.sh"
exit "${STATUS}"
