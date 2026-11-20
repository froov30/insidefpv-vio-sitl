# Simulated drone: ArduPilot SITL + Gazebo, with visual odometry

A four-part take-home. Build the simulator stack, fly an aircraft in it,
record what its camera and inertial sensors see, then recover the trajectory
from that recording alone and measure it against GPS.

Gazebo Harmonic owns the physics, ArduPilot SITL owns the flight control, and
the scripts here are the ground station. There is no ROS anywhere: the camera
is read through Gazebo's transport bindings and the aircraft is commanded over
MAVLink with `pymavlink`. Python, OpenCV and NumPy, and nothing else.

## The four tasks

| | Task | What was delivered |
|---|---|---|
| 1 | Build and run the stack | The ArduPilot Gazebo plugin compiled against Gazebo Harmonic, and an ArduCopter SITL flying a JSON-modelled airframe. Both `ldd` checks report zero ROS libraries. |
| 2 | Take off, hover, land | Guided takeoff to 5 m, 8 s hover, land, disarm. The two-cold-run acceptance test passes; nine flights are kept in `results/task2/`, five of them with the camera view recorded at 640x480 / 10 fps. |
| 3 | Record a dataset for VIO | One 8 m square at 5 m altitude: 435 camera frames at 10 Hz, `RAW_IMU` at 50 Hz, attitude at 20 Hz, GPS at 5 Hz. The camera intrinsics are read live from the sensor, not hardcoded. |
| 4 | Visual odometry against GPS | A monocular VIO that uses the flight controller's gyro as a rotation prior and a ground-plane model for metric translation. 0.654 m horizontal RMSE over a 54.9 m path, 0.55 m closure, no fitted parameters. |

## Running it

Nothing is vendored. The first build fetches ArduPilot, the Gazebo plugin and
the RapidJSON headers at pinned revisions into `third_party/`, then compiles
both halves. That fetch is the only step that needs the network and it happens
once: expect about 2 GB of disk and a few minutes for it, most of which is the
ArduPilot tree. `bash scripts/build.sh deps` does just the fetching, which is
useful when the network is the part that failed.

```bash
bash scripts/build.sh           # once, several minutes: fetch, plugin, SITL

# Two terminals, in this order. Each prints a readiness line and waits.
./scripts/run_gazebo.sh                # simulator
./scripts/run_sitl.sh                  # flight controller

# Third terminal: fly the aircraft, or run the acceptance check.
./scripts/run_task2.sh                 # take off, hover, land
./scripts/run_task2.sh --test          # two cold runs, ten assertions each
```

`./scripts/run_sim.sh --headless` starts both halves in one terminal if that is
easier. `./scripts/stop_sim.sh` ends the pair.

Task 3 and 4, with the simulator still running:

```bash
./scripts/run_task3_record.sh          # fly the square, record the dataset
./scripts/run_task3_vio.sh             # offline VIO, no simulator needed
./scripts/run_task4_compare.sh --no-show
```

The record step has to come first, because camera frames are not committed:
they are 27 MB per recording, so only `results/task3/dataset_*/images/` is
absent from this repository, and the logs and trajectories beside them are
present. `run_task3_vio.sh` says so if the frames it needs are missing.
`run_task4_compare.sh` needs no frames and reproduces the numbers below from
the committed CSVs alone.

Every wrapper is a fixed command line over one Python entry point, so the
arguments are visible rather than hidden, and each one checks that the
simulator is actually up before it starts flying.

## How the odometry works

Five steps per pair of frames:

1. **Features.** Shi-Tomasi corners, pyramidal Lucas-Kanade tracking, then a
   forward-backward check that discards matches which do not return to where
   they started. Without that check, corners leaving the frame enter the
   geometry as real motion.
2. **Rotation from the gyro.** `RAW_IMU` is integrated across each frame
   interval, including the partial samples at both ends, and conjugated into
   camera axes.
3. **Translation from a plane model.** The ground is a plane, so two views are
   related by `x2 ~ (R + t n^T / d) x1`. With `R` known from step 2 this is
   linear in `u = t/d`, a three-parameter least-squares problem.
4. **Metric scale from height.** `d` is the camera's distance to the ground,
   which for a downward camera is the aircraft's height. Each step is `-d u`,
   rotated into NED before it is accumulated.
5. **Cross-check.** `findEssentialMat` and `recoverPose` run on the same
   correspondences and their answers are recorded as evidence, never used as
   the pose.

The essential matrix was the first approach and was rejected on measurements:
on near-hover pairs its translation direction is dominated by pixel noise, and
on high-flow pairs over a flat ground plane it has a two-fold ambiguity that
the cheirality test does not resolve. Across this flight the two estimators
disagree by a median of 44.4 degrees in direction, and that number is in the
output rather than in a comment.

## What the numbers mean

| | |
|---|---|
| Horizontal RMSE against GPS | 0.654 m over an 8 m square |
| Closure error after 54.9 m of path | 0.552 m |
| Gyro vs vision rotation agreement | 0.4 degrees median |
| Vertical RMSE | 0.746 m, the weakest of the three axes |

The 0.654 m is a raw alignment: both tracks are moved to a common origin and
nothing else. No rotation and no scale are fitted. The fitted-similarity
variant is reported separately as an upper bound on the track's shape.

## What it does not do

- One flight, one world, simulator GPS. There is no cross-validation run.
- Scale comes from height, so a height error is a scale error; the odometry is
  not fully self-contained.
- No loop closure and no bundle adjustment. Frame-to-frame integration only, so
  nothing ever corrects what has already been accumulated.
- Gazebo's camera has no rolling shutter or motion blur, and its IMU has no
  bias instability, so these numbers are easier than hardware would give.

## Requirements

Ubuntu 22.04 is what this was developed and tested on. Beyond a C++ toolchain
and `git`, the build needs:

- **Gazebo Harmonic**, with its development packages: `gz-sim8`,
  `libgz-sim8-dev`, `libgz-transport13-dev`, `libgz-msgs10-dev`,
  `libgz-cmake3-dev`, `libgz-plugin2-dev` and `libgz-utils2-dev`. On 22.04
  these come from the OSRF repository (`packages.osrfoundation.org`), which
  has to be added to apt first.
- **Runtime libraries the plugin links**: `libopencv-dev`,
  `libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev`, `pkg-config`,
  `cmake`, `g++`.
- **Python 3** with `pymavlink`, `numpy`, `opencv-python`, `matplotlib`, and
  the `gz.transport13` bindings that the Gazebo packages install.
- **`rapidjson-dev`**, or the network access for `build.sh` to fetch and
  unpack the Debian package into `third_party/sysroot` (it needs `apt-get`
  and `dpkg-deb`, not root).
- `shellcheck` if you want to run the static check the scripts are clean for.

No ROS installation is needed or used. `scripts/env.sh` strips ROS variables
from whatever shell it is sourced into, so that claim can be checked at
runtime and not only by reading the source.
