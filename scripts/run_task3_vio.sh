#!/usr/bin/env bash
# Task 3, second half: the offline VIO on the newest dataset, then its plot.
#
# The estimator reads the recorded files and writes a trajectory; it never
# talks to the simulator, so this step works after the pair has been stopped
# and produces the same numbers every time it is run on the same dataset.
#
# Output
#   results/task3/vio_trajectory.csv    one row per frame, metric NED
#   results/task3/vio_trajectory.json   the estimator's own checks
#   results/task3/vio_trajectory.png    GPS against VIO, drawn by
#                                       compare.py --plot-vio; follows
#                                       --output, same stem as the CSV
#
# Usage
#   ./scripts/run_task3_vio.sh                 # newest dataset
#   ./scripts/run_task3_vio.sh --show          # open the plot in a window
#   ./scripts/run_task3_vio.sh --dataset results/task3/dataset_<stamp>
#
# Runtime is about 20 s for the 435 frames of one recording; it scales with
# the frame count, and nothing in it is random.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

# --show belongs to this wrapper, not to the estimator, so it is consumed
# here and the image is opened with the desktop's viewer at the end.
# Everything else goes to vio.py untouched, including --dataset and --output.
SHOW=0
ARGS=()
for arg in "$@"; do
  case "${arg}" in
    --show) SHOW=1 ;;
    *)      ARGS+=("${arg}") ;;
  esac
done

# The figure has to read the same dataset as the estimator, and the same
# output file, or it draws one run's trajectory against another run's frames.
# The reference track and the clock bridge come from compare.py, which takes
# frames.csv from the dataset, so a redirected --dataset has to be forwarded
# as well as a redirected --output. Both spellings (--dataset X and
# --dataset=X) are accepted because the wrappers are meant to be typed by hand.
DATASET=""
TRAJECTORY="${ROOT}/results/task3/vio_trajectory.csv"
PREVIOUS=""
for arg in ${ARGS[@]+"${ARGS[@]}"}; do
  case "${PREVIOUS}" in
    --dataset) DATASET="${arg}" ;;
    --output)  TRAJECTORY="${arg}" ;;
  esac
  case "${arg}" in
    --dataset=*) DATASET="${arg#*=}" ;;
    --output=*)  TRAJECTORY="${arg#*=}" ;;
  esac
  PREVIOUS="${arg}"
done

# The PNG follows the trajectory - same directory, same stem - for the same
# reason: a redirected run must not leave its plot beside another run's
# numbers. compare.py --plot-vio derives it that way itself.
case "${TRAJECTORY}" in
  *.csv) PLOT_OUTPUT="${TRAJECTORY%.csv}.png" ;;
  *)     PLOT_OUTPUT="${TRAJECTORY}.png" ;;
esac

python3 "${ROOT}/scripts/vio.py" ${ARGS[@]+"${ARGS[@]}"}

PLOT_ARGS=(--plot-vio --trajectory "${TRAJECTORY}")
if [[ -n "${DATASET}" ]]; then
  PLOT_ARGS+=(--dataset "${DATASET}")
fi
python3 "${ROOT}/scripts/compare.py" "${PLOT_ARGS[@]}"

if [[ "${SHOW}" -eq 0 ]]; then
  exit 0
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo "No DISPLAY, so the plot was saved but not opened:"
  echo "  ${PLOT_OUTPUT}"
  exit 0
fi
# Opened with the desktop's own viewer, like the Task 4 wrapper does: failing
# to open a window must never look like failing to compute.
if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "${PLOT_OUTPUT}" >/dev/null 2>&1 &
  wait || true
else
  echo "  ${PLOT_OUTPUT}"
fi
