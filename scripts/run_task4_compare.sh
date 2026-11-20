#!/usr/bin/env bash
# Task 4: compare the VIO trajectory with the recorded GPS track, and show it.
#
# Three alignments are reported, in increasing order of how much is fitted:
#   raw            nothing is fitted; both tracks are moved to their first
#                  sample. This is the headline number.
#   heading-only   one rotation about vertical. Shows how much of the error is
#                  a heading offset rather than a shape error.
#   Umeyama        rotation, translation and scale, horizontal plane only. An
#                  upper bound, labelled as one, never quoted as accuracy.
#
# Output, in results/task4/
#   metrics.csv     29 rows: 24 alignment metrics plus 5 summary values
#   compare.json    the same numbers plus origins, counts and output paths
#   xy_overlay.png  GPS, VIO raw and VIO heading-aligned on one grid
#   altitude.png    reference altitude against the VIO's own vertical axis
#   error.png       horizontal error against distance flown
#
# Nothing here is random: same dataset in, same numbers out, which is what
# makes the figures in README.md reproducible from the committed logs.
#
# Usage
#   ./scripts/run_task4_compare.sh              # compute, then open the plots
#   ./scripts/run_task4_compare.sh --no-show    # compute only
#   ./scripts/run_task4_compare.sh --dataset results/task3/dataset_<stamp>

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

# --no-show is this wrapper's flag: compare.py never opens a window. The
# images are opened with the desktop's own viewer, so a headless run only has
# to pass --no-show (or simply have no DISPLAY).
SHOW=1
ARGS=()
for arg in "$@"; do
  case "${arg}" in
    --no-show) SHOW=0 ;;
    *)         ARGS+=("${arg}") ;;
  esac
done

# The plots have to be opened from wherever compare.py actually wrote them.
OUTPUT="${ROOT}/results/task4"
PREVIOUS=""
for arg in ${ARGS[@]+"${ARGS[@]}"}; do
  if [[ "${PREVIOUS}" == "--output" ]]; then
    OUTPUT="${arg}"
  fi
  PREVIOUS="${arg}"
done

python3 "${ROOT}/scripts/compare.py" ${ARGS[@]+"${ARGS[@]}"}

echo
echo "Written to ${OUTPUT}"
if [[ "${SHOW}" -eq 0 ]]; then
  echo "Plots not opened (--no-show)."
  exit 0
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo "No DISPLAY, so the plots were saved but not opened:"
  ls -1 "${OUTPUT}"/*.png 2>/dev/null || true
  exit 0
fi

# One viewer per plot, backgrounded so this script returns. xdg-open picks
# whatever the desktop uses; if it is missing the paths are printed instead,
# because failing to open a window must not look like failing to compute.
for plot in xy_overlay.png altitude.png error.png; do
  if [[ -f "${OUTPUT}/${plot}" ]]; then
    if command -v xdg-open >/dev/null 2>&1; then
      xdg-open "${OUTPUT}/${plot}" >/dev/null 2>&1 &
    else
      echo "  ${OUTPUT}/${plot}"
    fi
  fi
done
wait || true
