#!/usr/bin/env bash
# Builds everything this project compiles, in the order the run scripts need it.
#
#   1. libArduPilotPlugin.so  the Gazebo system plugin that carries the Iris
#                             state to SITL and the servo commands back
#   2. arducopter             the SITL flight controller, board "sitl"
#
# The first run also fetches what is deliberately not committed: the two
# upstream repositories at pinned revisions, and the RapidJSON headers the
# plugin's CMakeLists requires. They are several hundred megabytes of source
# that would bury the diff, so third_party/ is git-ignored and this script is
# what keeps "clone, build, fly" true. Everything is fetched once; later runs
# reuse the checkout.
#
# Every step is incremental, so this is also the command to use while
# iterating: only what changed is recompiled.
#
# Usage
#   ./scripts/build.sh            # sources, then both builds, in order
#   ./scripts/build.sh plugin     # only the Gazebo plugin
#   ./scripts/build.sh sitl       # only the flight controller
#   ./scripts/build.sh deps       # only fetch the sources, if that was the failure

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

# ---------------------------------------------------------------------------
# Sources (fetched once, pinned)
# ---------------------------------------------------------------------------

# Both revisions are what the committed flight was recorded with. A different
# revision is a different simulator, and the numbers quoted in README.md would
# no longer describe it, so changing either one means re-recording the dataset.
ARDUPILOT_REPO="https://github.com/ArduPilot/ardupilot.git"
ARDUPILOT_REV="5cdd6165d25adcdf6b94d7537436fc4c99bd6ce8"
GAZEBO_REPO="https://github.com/ArduPilot/ardupilot_gazebo.git"
GAZEBO_REV="f116b03552d05c8d0b0bed5ed67033a1183121d1"

fetch_revision() {
  # fetch_revision <directory> <url> <revision>
  #
  # A shallow fetch of one commit rather than a clone: the history is most of
  # the download and none of the build. The result is a detached checkout on
  # purpose - this is a pinned dependency, not a branch to pull.
  local dir="$1" url="$2" rev="$3"
  if [[ -e "${dir}/.git" ]]; then
    return 0
  fi
  if [[ -e "${dir}" ]]; then
    echo "ERROR: ${dir} exists but is not a git checkout." >&2
    echo "Move it aside and run this script again." >&2
    exit 1
  fi
  echo "Fetching ${url}"
  echo "         revision ${rev}"
  mkdir -p "${dir}"
  if ! git -C "${dir}" init -q \
     || ! git -C "${dir}" remote add origin "${url}" \
     || ! git -C "${dir}" fetch -q --depth 1 origin "${rev}" \
     || ! git -C "${dir}" checkout -q FETCH_HEAD; then
    rm -rf "${dir}"
    echo "ERROR: could not fetch ${url} at ${rev}." >&2
    echo "Network access is needed once; the checkout is reused afterwards." >&2
    exit 1
  fi
}

resolve_rapidjson() {
  # Sets RAPIDJSON_DIR to a directory holding RapidJSONConfig.cmake.
  #
  # The plugin's CMakeLists calls find_package(RapidJSON REQUIRED), which
  # wants a package config, not just headers on the include path. RapidJSON is
  # header-only, and the distribution package installs the headers and that
  # config together, so it is the smallest thing that satisfies the call. A
  # system install is used when present; otherwise the package is downloaded
  # and unpacked into third_party/ without needing root.
  local candidate
  for candidate in \
    "${ROOT}/third_party/sysroot/usr/share/cmake/RapidJSON" \
    "/usr/share/cmake/RapidJSON" \
    "/usr/lib/cmake/RapidJSON" \
    "/usr/lib/x86_64-linux-gnu/cmake/RapidJSON"; do
    if [[ -f "${candidate}/RapidJSONConfig.cmake" ]]; then
      RAPIDJSON_DIR="${candidate}"
      return 0
    fi
  done

  local work="${ROOT}/third_party/.rapidjson" deb
  mkdir -p "${work}"
  echo "Fetching the RapidJSON headers (no system install found)"
  if ! ( cd "${work}" && apt-get download rapidjson-dev >/dev/null 2>&1 ); then
    echo "ERROR: could not download rapidjson-dev." >&2
    echo "Install it (sudo apt install rapidjson-dev), or unpack the package" >&2
    echo "into ${ROOT}/third_party/sysroot by hand." >&2
    exit 1
  fi
  deb="$(find "${work}" -maxdepth 1 -name 'rapidjson-dev_*.deb' | head -n 1)"
  if [[ -z "${deb}" ]] || ! dpkg-deb -x "${deb}" "${ROOT}/third_party/sysroot"; then
    echo "ERROR: could not unpack ${deb} into third_party/sysroot" >&2
    exit 1
  fi
  rm -rf "${work}"
  RAPIDJSON_DIR="${ROOT}/third_party/sysroot/usr/share/cmake/RapidJSON"
}

fetch_sources() {
  fetch_revision "${ROOT}/third_party/ardupilot_gazebo" \
    "${GAZEBO_REPO}" "${GAZEBO_REV}"
  fetch_revision "${ROOT}/third_party/ardupilot" \
    "${ARDUPILOT_REPO}" "${ARDUPILOT_REV}"
  # SITL is built from the flight-controller tree, whose waf build needs its
  # own submodules (the MAVLink headers and the CAN libraries among them).
  # A submodule records its git directory in a .git *file*, which is why the
  # presence test uses -e rather than -d.
  #
  # Shallow first: the build needs one revision of each submodule, not its
  # history. A shallow submodule clone lands on the tip of the default branch,
  # so it only succeeds when that tip is the revision the parent pins; when it
  # is not, git reports a missing commit and the full clone is the fallback.
  if [[ ! -e "${ROOT}/third_party/ardupilot/modules/mavlink/.git" ]]; then
    echo "Fetching the ArduPilot submodules (shallow)"
    if ! git -C "${ROOT}/third_party/ardupilot" submodule update --init \
           --recursive --depth 1 --jobs 4; then
      echo "A shallow fetch could not reach the pinned revisions;"
      echo "repeating with full history (larger download, same result)"
      git -C "${ROOT}/third_party/ardupilot" submodule update --init \
        --recursive --jobs 4 || {
        echo "ERROR: could not fetch the ArduPilot submodules." >&2
        exit 1
      }
    fi
  fi
}

# ---------------------------------------------------------------------------
# Gazebo plugin
# ---------------------------------------------------------------------------

# -j2 rather than -j$(nproc). A Gazebo compile running beside a desktop
# session is killed by the OOM killer below about 8 GB of RAM, and the peak of
# this build is one large translation unit per job. Two jobs fit any machine
# that can run Gazebo at all.
#
# RapidJSON is header-only, and CMake's find_package(RapidJSON REQUIRED) wants
# the package config rather than headers on the include path.
# resolve_rapidjson() above locates it, system-wide or in the unpacked copy
# under third_party/sysroot, and passes the directory here; without an
# explicit path a configure run records RapidJSON_DIR-NOTFOUND and stops.
#
# The ament flag is not cosmetic. ardupilot_gazebo's CMakeLists calls
# find_package(ament_cmake QUIET) and builds as a ROS package when it finds
# one. On a machine with ROS installed, a configure run from a shell that had
# sourced /opt/ros/*/setup.bash caches ament_cmake_DIR in CMakeCache.txt, and
# every later configure tries to use it. env.sh strips the ROS directories
# from PYTHONPATH, so that half-present ament fails with "No module named
# 'ament_package'" and the build stops. Disabling the package lookup keeps
# this a no-ROS build whatever the caller's shell had sourced.
build_plugin() {
  resolve_rapidjson
  cmake -S "${ROOT}/third_party/ardupilot_gazebo" \
    -B "${ROOT}/third_party/ardupilot_gazebo/build" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DRapidJSON_DIR="${RAPIDJSON_DIR}" \
    -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
  cmake --build "${ROOT}/third_party/ardupilot_gazebo/build" -j2
  echo "Built ${ROOT}/third_party/ardupilot_gazebo/build/libArduPilotPlugin.so"
}

# ---------------------------------------------------------------------------
# SITL flight controller
# ---------------------------------------------------------------------------

# --board sitl compiles the flight code for a desktop CPU instead of a
# flight-controller MCU. Plane, rover and the hardware boards are not built.
# The binary is build/sitl/bin/arducopter, which is what run_sitl.sh starts.
build_sitl() {
  cd "${ROOT}/third_party/ardupilot"
  ./waf configure --board sitl
  ./waf copter -j2
  echo "Built ${ROOT}/third_party/ardupilot/build/sitl/bin/arducopter"
}

case "${1:-all}" in
  deps)
    fetch_sources
    resolve_rapidjson
    echo "Sources ready in ${ROOT}/third_party"
    ;;
  plugin) fetch_sources; build_plugin ;;
  sitl)   fetch_sources; build_sitl ;;
  all)    fetch_sources; build_plugin; build_sitl ;;
  *)
    echo "usage: $(basename "$0") [deps|plugin|sitl|all]" >&2
    exit 2
    ;;
esac
