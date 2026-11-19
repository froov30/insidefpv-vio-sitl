# Defence

This document is the walkthrough a reviewer can use instead of asking me
questions: what each file does, why each decision was made, what every
constant controls, what I got wrong on the way, and which command reproduces
each claim. Where a number appears, the file it comes from is named.

## Environment these results were produced on

| | |
|---|---|
| OS | Ubuntu 22.04 |
| Gazebo | Harmonic, `gz sim` 8.15.0, server-only with headless rendering |
| Flight controller | ArduCopter 4.8.0-dev, SITL, built from `third_party/ardupilot` |
| Bridge | `ArduPilotPlugin` from `third_party/ardupilot_gazebo`, built at `-j2` |
| Python | 3.10.12 |
| OpenCV | 4.5.4 |
| numpy | 1.21.5 |
| pymavlink | 2.4.49 |
| ROS | installed on the machine, **not used** by anything here (section 6) |

## 1. Map of the repository

```
scripts/
  env.sh                 shell environment; strips ROS paths, sets GZ_* and CPATH
  build_plugin.sh        builds libArduPilotPlugin.so
  build_sitl.sh          builds build/sitl/bin/arducopter
  run_gazebo.sh          starts the world only
  run_sitl.sh            starts the flight controller only
  run_sim.sh             starts both, in order, with process hygiene
  stop_sim.sh            stops a pair left behind by an earlier run
  mavlink_link.py        the only place that opens MAVLink or knows its units
  gimbal.py              points the camera down over gz-transport
  camera.py              camera topic names, image decoding, mp4 recording
  takeoff_land.py        Task 2 flight: takeoff, hover, land, disarm
  test_task2.py          Task 2 acceptance test; runs the flight twice
  record_dataset.py      Task 3 dataset: frames + IMU + attitude + GPS, square
  vio.py                 Task 3 VIO over a recorded dataset
  compare.py             Task 4 comparison against GPS, metrics and plots
worlds/
  vio_runway.sdf         iris_runway plus checkerboard calibration targets
results/
  task2/                 two test runs and the recorded video, each with logs
  task3/                 camera topics, one dataset, the VIO trajectory
  task4/                 metrics.csv, compare.json, three figures
logs/
  diag/                  the one-off scripts that diagnosed the VIO failures
third_party/             vendored ArduPilot, ardupilot_gazebo, rapidjson headers
```

`third_party/` is git-ignored and rebuilt by the two build scripts. The
recorded camera images are git-ignored too (`results/**/dataset_*/images/`)
because they are 472 PNGs; `record_dataset.py` regenerates them exactly, and
the timestamps and intrinsics that describe them are committed.

## 2. Task status and evidence

| Task | Status | Evidence |
|---|---|---|
| 1. SITL + Gazebo on Linux | done | `scripts/build_*.sh`, `scripts/run_*.sh`; `logs/build_fresh_*.log` |
| 2. Takeoff, hover, land, video | done, passed twice in two sessions | `results/task2/run1_20261120_030252/`, `run2_20261120_030444/`, `run1_20261120_011449/`, `run2_20261120_011605/`, `video_20261120_011831/`, `VIDEO_RUN.md` |
| 3. Dataset + VIO | done | `results/task3/dataset_20261120_012619/`, `results/task3/vio_trajectory.csv` |
| 4. Compare against GPS | done | `results/task4/metrics.csv`, `compare.json`, three PNGs, `REPORT.md` |

## 3. Per-file walkthrough

### `scripts/env.sh`

Sources the environment every other script inherits. It does two jobs that
are not obvious:

- **It removes ROS.** The login shell on this machine sources
  `/opt/ros/humble/setup.bash`, so `PYTHONPATH`, `LD_LIBRARY_PATH` and
  `CMAKE_PREFIX_PATH` all point into `/opt/ros`. `env.sh` unsets
  `ROS_DISTRO` and friends, then filters every element that points into a ROS
  install out of those three variables with `_strip_ros_path`. That is what
  makes "no ROS" a runtime property: `import rclpy` fails, and cmake cannot
  find `ament_cmake`.
- **It sets the paths Gazebo and the build need.** `GZ_SIM_SYSTEM_PLUGIN_PATH`
  for the plugin, `GZ_SIM_RESOURCE_PATH` with the repo's `worlds/` first,
  `CPATH` for the RapidJSON headers under `third_party/sysroot` (system-wide
  rapidjson is absent and there is no sudo).

Change nothing here without re-running both builds; `GZ_VERSION=harmonic` in
this file is what selects the Harmonic branch of ardupilot_gazebo's CMake.

### `scripts/build_plugin.sh`

Configures and builds `libArduPilotPlugin.so` at `-j2`. Two constants matter:

- `-j2`, not `-j8`: the machine has 8 GB of RAM and a Gazebo compile at `-j8`
  is killed by the OOM killer.
- `-DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON`: ardupilot_gazebo's
  `CMakeLists.txt` builds as a ROS package if `ament_cmake` is found. A
  configure that ever ran with ROS paths present caches `ament_cmake_DIR`,
  and later configures then die with `No module named 'ament_package'`
  because `env.sh` has removed the ROS Python path. Disabling the lookup is
  the fix; it was found during the acceptance pass, not guessed (section 7).

### `scripts/build_sitl.sh`

`./waf configure --board sitl` then `./waf copter -j2`. Only the copter
binary is built; planes, rovers and the hardware boards are not. The binary
lands at `third_party/ardupilot/build/sitl/bin/arducopter`.

### `scripts/run_gazebo.sh` and `scripts/run_sitl.sh`

Two processes, started separately so each can be inspected.

`run_gazebo.sh` runs `gz sim -r` (start the clock immediately; without `-r`
Gazebo opens paused and SITL waits in lockstep forever) with
`--headless-rendering` in headless mode so the camera still renders.

`run_sitl.sh` runs the arducopter binary **directly** rather than through
`sim_vehicle.py`, because `sim_vehicle.py` for `gazebo-iris` opens the
flight-controller console in an xterm, and every PreArm rejection and EKF
message went to that window instead of a file. The comment at the top of the
file records the exact `sim_vehicle.py` command line it replaces, so the
equivalence can be checked.

### `scripts/run_sim.sh` and `scripts/stop_sim.sh`

`run_sim.sh` starts both, in the only order that works: Gazebo first, wait
for the plugin to print "ArduPilot ready to fly", then SITL. It kills
leftovers on the way in, which is the fix for the heartbeat bug in section 7.
It prints `SIM_READY` once SITL is waiting for a ground station, which is the
line `test_task2.py` waits for.

`stop_sim.sh` stops the pair. Both match process patterns that contain this
project's own paths, so an unrelated Gazebo session on the machine is not
touched.

### `scripts/mavlink_link.py`

The single MAVLink module. Everything else calls into it, so units are
converted in exactly one place (`RAW_IMU` milli-g to m/s², `ALTITUDE` mm to m,
`GPS_RAW_INT` degrees × 1e7, and so on, tabulated in `unit_note()`).

Non-obvious decisions, each commented in the file:

- `EKF_READY_MASK = 1 | 16 | 32` is the flag set ArduCopter requires before
  it accepts an arm. A smaller mask produces "PreArm: Need Position Estimate".
- `TRANSIENT_PREARM_PATTERNS` lists only the refusals that clear by waiting
  (sensor consistency, EKF variance, GPS settling). Anything else fails the
  run immediately instead of retrying, so a real fault is visible.
- `POSITION_ONLY_MASK = 4088` is the documented "ignore everything but
  position, yaw is ignored" type mask for `SET_POSITION_TARGET_LOCAL_NED`.
- `SETPOINT_RATE_HZ = 5` re-sends the guided setpoint so a lost packet does
  not drop the target.

### `scripts/gimbal.py`

Publishes `gz.msgs.Double` to `/gimbal/cmd_pitch`. No ROS service, no plugin
beyond the model's own `JointPositionController`. `PITCH_DOWN_RAD = 0.7`
(40°) because at level the frame is half sky; at 0.7 the ground fills the
image, which is what a feature-based VIO needs. The command is republished for
`PUBLISH_DURATION_S` because the controller subscribes after the model loads
and a single publish can be lost.

### `scripts/camera.py`

Topic names in one place (a typo in a topic name does not raise - subscribe
succeeds and never calls back), the `RGB_INT8` → BGR conversion with a payload
length check, and `VideoRecorder` for Task 2's mp4. Frames are written from
the transport callback: at 10 Hz a 640x480 mp4v encode is a couple of
milliseconds, so a writer thread would only add a queue that could drop the
frames the video exists to prove.

### `scripts/takeoff_land.py`

Task 2. Guided mode, arm, take off to 5 m, hold for `HOVER_SECONDS`, land,
wait for disarm, then summarise the CSV it wrote. The summary
(`flight.summary.json`) is computed from the CSV, not from script state, so it
cannot claim a flight the log does not show. `--video` is optional, which is
why the automated test does not need the camera in the loop.

### `scripts/test_task2.py`

Task 2's acceptance test. It starts a supervised pair per run, flies, stops
the pair, collects the logs into the run directory, and then checks the CSV:

1. enough samples (`MIN_SAMPLES`),
2. phase order climb → hover → land,
3. peak altitude inside 5 m ± 0.5 m,
4. final altitude below `LANDED_MAX_ALT_M`,
5. the autopilot disarmed,
6. the flight script exited successfully.

`--repeat` defaults to 2 and the loop stops at the first failure, because a
later pass would not be "twice in a row".

### `scripts/record_dataset.py`

Task 3's recorder. Two MAVLink links: commands on 5760, logging on 5762
(SERIAL1). The logging link is not an optimisation: `recv_match(type=...)`
**discards** messages that are not of the requested type, so a single link
polled for `ATTITUDE` silently eats `RAW_IMU`. Separate links make the
requested rates independent.

Frames are timestamped from the Gazebo image stamp and paired with the
autopilot clock in the same row, so the offset between the two clocks is
measured per frame rather than assumed; the measured spread was 0.000 s.

Intrinsics are read from the live `camera_info` topic (`read_intrinsics()`)
and stored in `dataset.json` with the gimbal pitch. Nothing downstream
hardcodes `fx`.

### `scripts/vio.py`

Task 3's VIO. The module docstring states the method, why the essential
matrix was rejected, how the metric scale is obtained and what its weakness
is, and exactly what is never used for the pose (the reference north/east
position). Per frame pair:

1. Shi-Tomasi corners, pyramidal LK, forward-backward check.
2. Rotation from integrating the 50 Hz gyro across the interval, with both
   partial edge samples included, conjugated into camera axes.
3. Translation from the plane homography with the rotation held fixed: a
   3-parameter linear least-squares problem in `u = t / d`, iterated twice
   with a residual cut.
4. Metric step `-d·u` with `d` the camera height above the ground.
5. Composition into NED, then the next frame.
6. `findEssentialMat` + `recoverPose` run on the same correspondences purely
   as a cross-check; both disagreement angles are written to the CSV. The
   median direction disagreement (33.6°) is the recorded evidence for the
   choice, not an opinion.

`rotation_source` is written per frame, so a reviewer can see that 471 of 472
frames used the gyro and one did not.

### `scripts/compare.py`

Task 4. Projects the raw GPS fixes onto a tangent plane at the first fix,
checks that projection against the autopilot's own `LOCAL_POSITION_NED`
(median 0.01 m, max 0.52 m), matches VIO rows to reference samples through
the `boot_time_s` column, and then reports three alignments: raw (translate
only, the headline), heading-only, and a 2-D Umeyama similarity that is
labelled as a fitted upper bound. The Umeyama fit is horizontal-only on
purpose: a 3-D fit came out *worse* than raw because it spent horizontal
accuracy on the vertical axis.

Outputs `metrics.csv`, `compare.json` and the three figures.

### `worlds/vio_runway.sdf`

`iris_runway` plus a set of checkerboard calibration targets on both sides of
the runway. They have no `<collision>`, so they do not change the physics or
the flight; they exist to give the camera texture and, if someone wants to,
a calibration target. The world keeps the name `iris_runway` so the camera
topic names in `camera.py` stay valid.

### `logs/diag/`

Eight one-off scripts from the VIO investigation (synthetic scene with known
motion, per-pair error tables, flow gates, homography decompositions, full
trajectory integration). They are not part of the deliverable pipeline, but
they are kept because every claim in section 7 is reproducible with one of
them.

## 4. Constants

### `scripts/vio.py`

| Constant | Value | Role | Doubling | Halving |
|---|---|---|---|---|
| `MAX_FEATURES` | 200 | corners per frame | more texture to fit, ~2x feature time, more outliers from weak corners | fewer, faster, and the plane fit loses redundancy |
| `FEATURE_QUALITY` | 0.01 | Shi-Tomasi acceptance vs strongest corner | more corners, including weak ones that track poorly | fewer, stronger corners; can starve the fit |
| `MIN_FEATURE_DISTANCE_PX` | 12 | min spacing | fewer overlapping corners; LK has a unique match | corners within one 21 px window compete and LK picks wrong |
| `LK_WINDOW_PX` | 21 | LK patch | follows larger motion, less precise on small motion | precise on small motion, loses fast corners near the bottom of the frame |
| `LK_PYRAMID_LEVELS` | 3 | LK pyramid | survives ~4x more motion per frame, at 4x cost | fails above ~25 px of flow (measured case) |
| `FB_MAX_ERROR_PX` | 1.0 | forward-backward acceptance | keeps more tracks, including drifted ones that enter the geometry as motion | discards good tracks in fast-motion frames |
| `RANSAC_THRESHOLD_PX` | 1.0 | essential-matrix inlier threshold | more inliers, worse pose | fewer, tighter inliers, may fail the sample |
| `RANSAC_PROBABILITY` | 0.999 | RANSAC confidence | more iterations, no accuracy gain here | fewer iterations, occasional bad model |
| `MIN_INLIERS` | 12 | minimum correspondences | accepts weak pairs that bias the step | rejects usable pairs; 8 is the eight-point minimum, below 12 the fit is unstable |
| `PLANE_RESIDUAL_PX` | 1.5 | plane-fit residual cut | accepts more, keeps biased points | rejects valid points in fast frames, fit loses support |
| `PLANE_ITERATIONS` | 2 | refine passes | more rejection, starts losing real fast-motion corners | one pass keeps obvious outliers |

The effect of changing any of these is visible in the summary of
`vio_trajectory.json`: `inliers_mean`, `inliers_min` and
`essential_direction_disagreement_deg_median` move first.

### `scripts/compare.py`

| Constant | Value | Role | Doubling | Halving |
|---|---|---|---|---|
| `EARTH_RADIUS_M` | 6378137.0 | tangent-plane projection | 0.5 m error per 8 m (wrong radius) | same, opposite sign |
| `MIN_SAMPLES` | 10 | minimum before a metric exists | needs more data, safer | a handful of points produce precise-looking nonsense |
| `HEADING_COARSE_DEG` | 1.0 | coarse heading search | 2x search cost, no better answer | can skip over the true minimum on a noisy error curve |
| `HEADING_FINE_DEG` | 0.1 | refine step | fits noise; offset is expected small and smooth | stops before the true minimum |
| `FIGURE_DPI` | 150 | plot resolution | sharper, ~2x file size | smaller, harder to read |

### `scripts/record_dataset.py`

| Constant | Value | Role | Doubling | Halving |
|---|---|---|---|---|
| `FIRST_MESSAGE_TIMEOUT_S` | 30 | wait for first frame / camera_info | slower to report a dead camera | false failure on a slow cold start |
| `FRAME_QUEUE_SIZE` | 40 | callback → writer queue | more slack for a slow disk, more memory | drops frames when a write stalls |
| `PNG_COMPRESSION` | 3 | PNG level | ~4x CPU for a small size gain at level 9 | larger files, no accuracy change |
| `TARGET_ALTITUDE_M` | 5.0 | square altitude | a bigger footprint, coarser ground resolution | less parallax, weaker VIO |
| `ALTITUDE_TOLERANCE_M` | 0.5 | climb acceptance | starts the legs while still climbing | waits on barometer noise, may time out |
| `LEG_M` | 8.0 | square side | longer legs, more drift accumulated per leg | less texture motion, more low-parallax frames |
| `CORNER_HOVER_S` | 2.0 | stop at each corner | longer dataset, more zero-motion frames to test drift | corners are cut, less settling time |
| `LEG_TIMEOUT_S` | 60 | per-leg wall clock | slower failure detection | times out on a lockstep slowdown (RTF was 0.842) |
| `POST_LAND_SECONDS` | 2.0 | record after disarm | dataset ends later with the aircraft still on the ground | dataset ends at touchdown |
| `READY/CLIMB/LAND_TIMEOUT_S` | 90/90/120 | wall-clock budgets | never fires, a hung run wastes 4 minutes | fires on a slow machine, false failure |
| `STREAM_RATES` | 50/50/20/10/10/5/2 Hz | logger rates | more samples per frame, more bandwidth | below 20 Hz the gyro can no longer be integrated between frames |
| `COMMAND_RATES` | 10/2/2 Hz | command-link rates | more traffic on the link that also sends commands | too slow to notice readiness |
| `GIMBAL_PITCH_RAD` | 0.7 | camera pitch | frame sees the horizon and sky | frame sees less ground; 0 is the stock level view |

### `scripts/mavlink_link.py`, `takeoff_land.py`, `test_task2.py`

| Constant | Value | Role | Doubling | Halving |
|---|---|---|---|---|
| `CONNECTION` | tcp 5760 | SITL command port | invalid port, no connection | same |
| `EKF_READY_MASK` | 1\|16\|32 | arm prerequisites | accepts a worse estimate, ArduCopter still refuses | declares ready too early |
| `POSITION_ONLY_MASK` | 4088 | setpoint type mask | enables velocity/accel fields that are then ignored | changes which fields are honoured |
| `SETPOINT_RATE_HZ` | 5.0 | guided setpoint rate | more traffic, no benefit | a dropped packet leaves a stale target longer |
| `TARGET_ALTITUDE_M` (task 2) | 5.0 | climb target | a 10 m flight; Task 3 comparison no longer matches | a 2.5 m flight, less to see |
| `ALTITUDE_TOLERANCE_M` | 0.5 | arrival band | declares arrival while climbing | waits on barometer noise |
| `HOVER_SECONDS` | 8.0 | hold time | longer flight, same result | too short to see a hover |
| `POSITION_RATE_HZ` | 10 | CSV rate | denser log, more CPU | coarser altitude check |
| `MIN_SAMPLES` (test) | 50 | minimum CSV rows | stricter, slower to fail on a wrong setup | a broken telemetry setup could still pass |
| `FLIGHT_TIMEOUT_S` | 420 | whole-flight budget | a hung run burns 14 minutes | false failure on a slow machine |
| `EXPECTED_PHASES` | climb, hover, land | phase order | — | order is what the test is checking |

## 5. Likely questions

**"Is this really not using ROS?"**
Section 6 has the greps and the `ldd` output. Practically: no
`import rclpy`/`rospy` anywhere, no ROS library linked into the plugin, SITL
binary or `gz`, no ROS plugin in the world, and `env.sh` actively removes ROS
paths so an accidental dependency fails loudly rather than working by
accident. The camera is read with the gz-transport Python bindings; the
gimbal is driven by a gz-transport `Double`; the autopilot is spoken to with
pymavlink.

**"Why did the plugin build as a ROS package at all?"**
Because ROS Humble is installed and ardupilot_gazebo builds as an ament
package when it finds `ament_cmake`. The build script now disables that
lookup explicitly. This surfaced during the acceptance pass, which is exactly
what a clean build is for.

**"Why not use the GPS or the autopilot's position for the pose?"**
That would make the comparison circular. The VIO uses the gyro (an inertial
rate), the attitude's roll/pitch (tilt, observable from an accelerometer) and
the height as a range; it never reads north/east. Section "What is never used
for the pose" in `vio.py` and the code path in `run_vio` make that checkable.

**"Is the metric scale cheating?"**
No, but it is a stated limitation. Scale is taken from the camera height
above the ground, the same principle as a downward camera plus a rangefinder
or barometer on a real aircraft. It is stated in the docstring and in
`REPORT.md`, including the failure mode: anything standing above the ground
biases the step.

**"Why is the vertical axis so bad?"**
Motion toward a plane is nearly unobservable from the image alone; the plane
normal direction is the weakest part of the solve, and errors there
accumulate. Measured: 1.43 m vertical RMSE against 0.195 m north. A nominal
gimbal pitch that is off by a fraction of a degree tilts the assumed normal
and lands mostly on this axis. `REPORT.md` quantifies both.

**"Why not ORB-SLAM or VINS-Fusion?"**
They are ROS-centric or need a full IMU preintegration implementation, and
the brief asks for a documented, inspectable pipeline. The point here is that
every failure mode in a simple pipeline was found, measured and reported,
not that the pipeline is state of the art.

**"Two runs passing is luck, not reproducibility."**
`test_task2.py` runs the whole thing twice from cold starts (fresh Gazebo,
fresh SITL, leftovers killed), asserts from the CSVs, and stops at the first
failure. Both summaries report `ok: true`, 5.06 m peak, 0.0 m final.

**"How do I know the video is the real flight?"**
`VIDEO_RUN.md` records the checks: frame count and fps from OpenCV, and
brightness at three instants (75.4 start, 114.0 mid-flight, 76.1 after
landing), which shows the pixels change, plus the frame count recorded in the
same run's `flight.summary.json`.

**"Why 5 m and 8 m?"**
5 m is the brief. 8 m legs at 5 m altitude keep the ground in frame and give
about 40° of parallax across a corner, which is what a monocular solver needs;
shorter legs put most frames into the low-parallax regime where this
pipeline's predecessor (the essential matrix) failed by 50-175°.

**"Why is there a separate logger link?"**
`recv_match(type=...)` throws away non-matching messages. On one link, the
messages polled for one stream are consumed while waiting for another, and
the 50 Hz IMU silently drops to whatever the polling loop allows. 5762 is
SERIAL1; 5763 is SERIAL2, available if a third consumer is ever needed.

**"What happens if a stale SITL is still running?"**
Port 5760 is still listening, the TCP connect succeeds from the listen
backlog, and no heartbeat ever arrives because SERIAL0 accepts exactly one
client and never re-accepts. That is the bug in section 7; `run_sim.sh` and
`stop_sim.sh` kill leftovers, and `mavlink_link.connect()` names the script in
its error message.

**"Why are the diagnostics still in the repo?"**
They are the evidence for section 7. Each one is small and runnable, and each
claim about a failure mode points at one of them.

**"What would you do with another day?"**
Pose-graph loop closure over the 472 frames (needs the trajectories stored,
not recomputed); estimate the gimbal pitch as a state instead of assuming 0.7;
add an accelerometer-based velocity prior to attack the vertical axis; and
run a straight-line test to separate closure error from drift rate.

## 6. The acceptance evidence

### Builds

```
$ bash scripts/build_plugin.sh
... Built target ArduPilotPlugin / CameraZoomPlugin / GstCameraPlugin
Built .../third_party/ardupilot_gazebo/build/libArduPilotPlugin.so

$ bash scripts/build_sitl.sh
bin/arducopter   4835382  218205  246728   5053587
'copter' finished successfully (5.326s)
Built .../third_party/ardupilot/build/sitl/bin/arducopter
```

A clean configure was also proved from an empty directory
(`third_party/ardupilot_gazebo/build_fresh`, git-ignored):
its `CMakeCache.txt` contains no `ament_cmake_DIR` - only
`CMAKE_DISABLE_FIND_PACKAGE_ament_cmake:UNINITIALIZED=ON` - and the resulting
`libArduPilotPlugin.so` was built and checked. Logs:
`logs/build_fresh_configure.log`, `logs/build_fresh_build.log`.

### Static checks

```
$ python3 -m py_compile scripts/*.py logs/diag/*.py      # ok
$ bash -n scripts/*.sh                                    # ok
$ shellcheck -S warning scripts/*.sh                      # exit 0
```

### No ROS

```
$ grep -rnE "^\s*(import|from)\s+(rclpy|rospy|roslib|tf2_ros|sensor_msgs|cv_bridge|geometry_msgs|nav_msgs|gz_ros)" scripts/*.py
(no matches)
$ grep -rniE "\bros\b|gazebo_ros|libgazebo_ros" worlds/
(no matches)
$ ldd third_party/ardupilot_gazebo/build/libArduPilotPlugin.so | grep -icE "ros|rclcpp|rosidl"
0
$ ldd third_party/ardupilot/build/sitl/bin/arducopter | grep -icE "ros|rclcpp|rosidl"
0
$ ldd "$(command -v gz)" | grep -icE "ros|rclcpp|rosidl"
0
```

### Pipeline runs

```
$ python3 scripts/vio.py
Frames with a timestamp: 472, RAW_IMU samples: 2358
Inliers: mean 74.4, min 11
Rotation: gyro-vision disagreement median 0.276 deg
Direction: essential-matrix vs plane median 33.558 deg
Path length: 47.137 m

$ python3 scripts/compare.py
GPS-derived track against the autopilot's own local estimate: median 0.01 m, max 0.52 m
Raw alignment      : ATE RMSE 0.385 m (horizontal), max 1.077 m
Heading aligned    : ATE RMSE 0.352 m at -1.80 deg
Umeyama fit        : ATE RMSE 0.275 m (scale 0.966, labelled as a fitted upper bound)
```

### Task 2 flight test

```
$ python3 scripts/test_task2.py

=== Run 1 -> results/task2/run1_20261120_030252 ===
  [PASS] ground station exited 0: exit 0
  [PASS] flight.csv readable: 339 rows
  [PASS] samples >= minimum: 339 >= 50
  [PASS] phases in order: ['climb', 'hover', 'land']
  [PASS] peak altitude in band: 5.06 m in [4.5, 5.5]
  [PASS] landed below threshold: 0.00 m <= 0.5
  [PASS] flight duration plausible: 33.80 s autopilot time
  [PASS] summary agrees with csv: summary 5.06 csv 5.060
  [PASS] disarmed at end: MAV_MODE_FLAG_SAFETY_ARMED cleared
  [PASS] DataFlash log present: 00000008.BIN (1941504 bytes)

=== Run 2 -> results/task2/run2_20261120_030444 ===
  [PASS] ground station exited 0: exit 0
  [PASS] flight.csv readable: 339 rows
  [PASS] samples >= minimum: 339 >= 50
  [PASS] phases in order: ['climb', 'hover', 'land']
  [PASS] peak altitude in band: 5.06 m in [4.5, 5.5]
  [PASS] landed below threshold: 0.00 m <= 0.5
  [PASS] flight duration plausible: 33.80 s autopilot time
  [PASS] summary agrees with csv: summary 5.06 csv 5.060
  [PASS] disarmed at end: MAV_MODE_FLAG_SAFETY_ARMED cleared
  [PASS] DataFlash log present: 00000009.BIN (1994752 bytes)

RESULT: PASS
```

These are cold starts: fresh Gazebo, fresh SITL, leftovers killed first. The
earlier pair `run1_20261120_011449` / `run2_20261120_011605` produced the
same numbers (339 rows, 5.06 m peak, 0.00 m final, disarmed), which is the
reproducibility check on a different session.

## 7. What went wrong, and how it was found

Every item below was found by measuring on this machine, not by reading
documentation and assuming.

1. **"No heartbeat".** SERIAL0 accepts one TCP client and never re-accepts;
   a leftover SITL keeps the port open so the next client connects and waits
   forever. Fix: kill leftovers before every run (`run_sim.sh`,
   `stop_sim.sh`) and say so in the error text. The evidence is in the
   earlier `logs/` captures and in the comment block at the top of
   `run_sim.sh`.
2. **VIO produced 2.3 m of an 8 m square.** Traced with six diagnostics to
   four causes, not guessed:
   - the essential matrix's translation is a unit vector, so a near-zero
     motion frame was integrated as a full step (`logs/diag/diag_vio.py`,
     `diag_gate.py`);
   - the coplanar two-fold ambiguity gave ~90° errors on high-flow frames
     with a flipped `y` (`diag_real_pair.py`);
   - the gyro integration dropped the partial edge samples of each interval,
     reporting 0.80x the true rotation (4.41° where attitude said 5.13°);
   - the gyro integration had the wrong sign, 12° of error on a 6° turn.
   The fix was the R-fixed plane solve plus a corrected gyro interval; the
   plane geometry itself was proved correct on a synthetic scene with known
   motion first (`diag_synthetic.py`, median 4.2° error).
3. **`d(H-R)n` gave 21 m for a 0.3 m step.** `findHomography` returns `H` up
   to a scale. Solving `H = λR + w nᵀ` failed too (λ≈0.8) because a *pixel*
   homography was being compared with a *normalized* model; `K⁻¹HK` fixed it
   and gave 0.30-0.32 m against a true 0.30 m (`diag_plane.py`).
4. **Free 8-DOF homography put the step direction 91° off.** It splits
   rotation and translation wrongly when both are free. Fixing `R` from the
   gyro and solving only `u` removed it.
5. **Umeyama made the error worse (1.467 m vs raw 0.385 m).** The 3-D fit
   traded horizontal accuracy for the weak vertical axis (scale 0.736).
   Fitting the horizontal plane alone gives 0.275 m at scale 0.966; both are
   reported, labelled as fitted.
6. **The plugin build failed on `ament_cmake`.** A configure run from a
   ROS-sourced shell cached `ament_cmake_DIR`; `env.sh` removes the ROS
   Python path, so the half-present package died with
   `No module named 'ament_package'`. Fixed by disabling the lookup in
   `build_plugin.sh` and stripping `CMAKE_PREFIX_PATH` in `env.sh`; verified
   with a clean configure in a fresh directory.

## 8. Known weaknesses

- One flight, one world, one 47 s run. No cross-validation.
- The reference is the simulator's own GPS (`fix_type` 6). Real GPS has
  multipath and bias.
- Vertical RMSE 1.43 m: the VIO ends 2.2 m below its start. Explained, not
  hidden, and not tuned away.
- Drift per metre (0.0098 m/m) is a *closure* figure from a closed square; it
  is not a random-walk rate and should not be extrapolated.
- The camera and IMU are ideal: no rolling shutter, no blur, no bias
  instability.
- Real time factor was 0.842, so the sensor rates are what the simulator
  produced.
- Recorded images are git-ignored; re-run `record_dataset.py` to regenerate
  them.
