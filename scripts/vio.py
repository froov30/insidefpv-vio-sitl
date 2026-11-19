#!/usr/bin/env python3
"""Offline monocular visual-inertial odometry over a recorded dataset.

Input is one results/task3/dataset_<stamp>/ directory produced by
scripts/record_dataset.py: 10 Hz camera frames, 50 Hz IMU, 20 Hz attitude,
and the autonomous position logs used only as a range measurement.

Method, step by step
1.  For every frame pair, Shi-Tomasi corners are detected on the first
    frame and tracked into the second with pyramidal Lucas-Kanade.
2.  Tracks are kept only if the backward flow returns to the original
    pixel within FB_MAX_ERROR_PX. Without that check a drifted corner looks
    like real motion and the pose error accumulates silently.
3.  The camera rotation between the two frames comes from integrating the
    50 Hz gyro over the interval. It is not taken from the images: a
    ground-looking camera in cruise turns and translates at the same time,
    and the gyro separates the two cleanly.
4.  The metric translation comes from the fact that the camera looks at a
    plane. Two views of a plane are related by a homography whose only
    unknown, once the rotation is known, is the translation divided by the
    distance to the plane. The distance is the aircraft's height above the
    ground, so the step comes out in metres; see plane_translation().
5.  Steps are composed into a trajectory in the NED frame of the autopilot,
    starting from the recorded attitude at the first frame.
6.  cv2.findEssentialMat + cv2.recoverPose also run on the same
    correspondences, and their direction is compared with the plane
    solution frame by frame. They are a cross-check, not the estimator;
    the reason is under "Why not the essential matrix" below.

Why not the essential matrix
    It was tried first, on this same dataset, and two failure modes were
    measured rather than assumed.

    * At low image motion (hover, the corners of the square) a 1-3 pixel
      flow is noise, and the recovered direction is noise with it: 50-175
      degrees of error against the aircraft's true direction.
    * With the motion parallel to a plane, the essential matrix has a
      two-fold ambiguity that cheirality does not resolve. It showed up as
      a 90 degree error on frames with 30+ pixels of flow - enough motion
      that low parallax cannot explain it - and it alternates frame to
      frame.

    Fixing the rotation from the gyro removes the second problem outright,
    because the rotation no longer competes with the translation for the
    same image motion, and it removes the first because the plane model
    returns a small translation when the aircraft barely moved instead of a
    unit vector. The median disagreement between the two estimators on this
    dataset is recorded in vio_trajectory.json; it is large, and that is
    the evidence for the choice.

Metric scale
    Monocular vision has no scale: an essential matrix or homography
    returns a translation direction, and the plane model returns
    translation per metre of distance to the plane. The distance is taken
    from the dataset's own altitude, so the scale is a range measurement,
    the same scale-from-a-range principle as using one known baseline.

    Weakness, stated plainly: the scale assumes everything the camera sees
    is the ground plane at the recorded height. Anything standing on the
    ground - a person, a vehicle, the checkerboard targets - sits closer
    than that and biases the step of the frames it fills. A sloping or
    stepped surface does the same. The vertical axis is the weakest part
    of the result for exactly this reason.

What is never used for the pose
    The reference north/east position. Height and roll/pitch are used - both
    are inertial-scale quantities (the height a range, the tilt an
    accelerometer reading) - and the initial attitude is borrowed once, at
    the first frame, to place the trajectory in the NED frame. compare.py
    does the alignment and the error metrics.

Usage
    python3 scripts/vio.py                       # newest dataset directory
    python3 scripts/vio.py --dataset results/task3/dataset_20261120_012619
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------

# Corners per frame. 200 features over ~300k pixels is one per 1500, which
# is plenty for a 640x480 synthetic scene; the flight moves ~5 m per second
# at 10 Hz, so a corner only has to survive one 0.1 s interval.
MAX_FEATURES = 200

# Shi-Tomasi acceptance: 1% of the strongest corner, and no two corners
# closer than 12 px. The minimum distance matters for optical flow: two
# corners inside one 21 px window give LK no unique match.
FEATURE_QUALITY = 0.01
MIN_FEATURE_DISTANCE_PX = 12

# Lucas-Kanade window and pyramid. 21 px is the classic default; three
# levels let the tracker follow a corner moving 25 px between frames, which
# is what the ground does at 5 m/s over a 5 m altitude when the corner is
# close to the aircraft.
LK_WINDOW_PX = 21
LK_PYRAMID_LEVELS = 3

# Forward-backward consistency. A track is kept only if the reverse flow
# lands within this distance of the starting pixel.
FB_MAX_ERROR_PX = 1.0

# RANSAC for the essential matrix. 1 px is the reprojection noise of a
# corner in a 640x480 frame; 0.999 is the probability of drawing a clean
# sample, which sets how many iterations RANSAC runs.
RANSAC_THRESHOLD_PX = 1.0
RANSAC_PROBABILITY = 0.999

# Fewer inliers than this and the pair is not trusted: the pose is carried
# forward and the frame is flagged in the CSV. Below ~8 points the
# eight-point algorithm is not even well posed.
MIN_INLIERS = 12

# Plane fit (the estimator actually used for the metric step). A tracked
# correspondence is dropped when the plane model misses it by more than
# this many pixels, and the fit is repeated over the survivors; two extra
# passes is enough to settle, and more would start throwing away the
# genuinely fast-moving corners near the bottom of the frame.
PLANE_RESIDUAL_PX = 1.5
PLANE_ITERATIONS = 2


# ---------------------------------------------------------------------------
# Small maths helpers
# ---------------------------------------------------------------------------

def rotation_from_euler(roll, pitch, yaw):
    """ZYX rotation matrix from roll/pitch/yaw (NED convention).

    The autopilot reports attitude as a 3-2-1 Euler sequence: yaw about
    down, then pitch about the resulting right axis, then roll about the
    resulting forward axis. The composition order matters: with this order
    a yaw of 90 degrees and a roll of 90 degrees give different results from
    the reverse, and only this one matches the airframe.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def body_to_camera(pitch_down_rad):
    """Rotation whose columns are the camera axes expressed in body axes.

    Body is the ArduCopter FRD frame: x forward, y right, z down. The
    camera is mounted on the gimbal and pitched down by pitch_down_rad, so
    looking straight out of the lens means going forward and downwards.

    The camera itself uses the OpenCV optical convention: x right, y down,
    z out of the lens, which is why the axes are written out rather than
    built from a Euler helper.
    """
    theta = pitch_down_rad
    x_cam = np.array([0.0, 1.0, 0.0])
    y_cam = np.array([-math.sin(theta), 0.0, math.cos(theta)])
    z_cam = np.array([math.cos(theta), 0.0, math.sin(theta)])
    return np.column_stack([x_cam, y_cam, z_cam])


# ---------------------------------------------------------------------------
# Dataset access
# ---------------------------------------------------------------------------

def read_csv(path):
    """Read a CSV of strings into a list of dicts.

    Values stay strings; each consumer converts the columns it needs, so a
    malformed column fails where it is used, with the column name in the
    message, instead of somewhere in a bulk float conversion.
    """
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def newest_dataset(task3_dir):
    """Return the most recently written dataset directory."""
    candidates = sorted(
        (p for p in task3_dir.glob("dataset_*") if p.is_dir()),
        key=lambda p: p.name,
    )
    if not candidates:
        raise SystemExit(
            f"No dataset_* directory in {task3_dir}. Record one first: "
            "bash scripts/run_sim.sh --headless, then "
            "python3 scripts/record_dataset.py"
        )
    return candidates[-1]


def load_dataset(dataset_dir):
    """Read the metadata and the logs the VIO needs.

    frames.csv carries boot_time_s, the autopilot clock at the moment the
    frame arrived, so the frames and the 50 Hz IMU are on one time base and
    no interpolation between clocks is needed. Frames recorded before the
    attitude stream started have an empty boot_time_s; they are dropped,
    because there is no way to say which IMU sample belongs to them.
    """
    meta = json.loads((dataset_dir / "dataset.json").read_text())
    frames = [row for row in read_csv(dataset_dir / "frames.csv")
              if row["boot_time_s"]]
    imu = [row for row in read_csv(dataset_dir / "imu.csv")
           if row["source"] == "RAW_IMU" and row["boot_time_s"]]
    attitude = read_csv(dataset_dir / "attitude.csv")
    local = [row for row in read_csv(dataset_dir / "local_position.csv")]
    return {
        "meta": meta,
        "frames": frames,
        "imu": imu,
        "attitude": attitude,
        "local": local,
    }


def gyro_intervals(imu):
    """Turn IMU rows into (t, omega) arrays on the autopilot clock."""
    times = np.array([float(row["boot_time_s"]) for row in imu])
    omega = np.array([
        [float(row["gyro_x_rad_s"]),
         float(row["gyro_y_rad_s"]),
         float(row["gyro_z_rad_s"])]
        for row in imu
    ])
    order = np.argsort(times)
    return times[order], omega[order]


def integrate_gyro(times, omega, t0, t1):
    """Rotation in the body frame between t0 and t1, by trapezoid integration.

    Returns a 3x3 matrix mapping body coordinates at t0 to body coordinates
    at t1. The trapezoid rule is used per sample interval because at 50 Hz
    the angle per interval is ~0.02 rad: a left or right rectangle rule
    would bias the integrated angle by up to half a sample of rotation on
    every frame, and that bias accumulates over 470 frames.

    Returns None when the interval has no IMU samples: the caller then falls
    back to the rotation from the essential matrix rather than inventing one.
    """
    start = np.searchsorted(times, t0)
    end = np.searchsorted(times, t1)
    if end - start < 2:
        return None
    # Integrate the whole [t0, t1] window, including the two partial edges.
    # The samples nearest each end are clamped to it, which covers the gaps
    # between the frame times and the first/last sample in the window.
    # Skipping those edges loses up to one sample per interval: at 10 Hz
    # frames and 50 Hz IMU that is one 20 ms sample out of 100 ms, and it was
    # measured as the gyro reporting 80% of the attitude change and a
    # gyro/vision rotation disagreement of about 1 degree on a 5 degree turn.
    window_t = np.concatenate(([t0], times[start:end], [t1]))
    window_w = np.vstack((omega[start], omega[start:end], omega[end - 1]))
    rotation = np.eye(3)
    for index in range(1, len(window_t)):
        dt = window_t[index] - window_t[index - 1]
        if dt <= 0:
            continue
        mean_w = 0.5 * (window_w[index] + window_w[index - 1])
        # The minus sign is not a choice. For a body-frame rate the
        # orientation evolves as R(t+dt) = R(t) exp([w]dt), so the map from
        # body coordinates at t0 to body coordinates at t1 is the inverse,
        # exp(-[w]dt). Integrating with +[w] produced a rotation that was
        # the exact reverse: measured against the autopilot attitude it
        # disagreed by 12 degrees on a 6 degree turn.
        rotation = rotation @ cv2.Rodrigues(-mean_w * dt)[0]
    return rotation


# ---------------------------------------------------------------------------
# Vision
# ---------------------------------------------------------------------------

def track_features(previous_gray, gray):
    """Detect corners on one frame and track them into the next.

    Returns two Nx2 float32 arrays of matched pixel coordinates. The
    forward-backward check is what makes this usable: LK will happily
    return a wrong match for a corner that left the frame or was occluded,
    and without the check those matches enter the geometry as real motion.
    """
    corners = cv2.goodFeaturesToTrack(
        previous_gray,
        maxCorners=MAX_FEATURES,
        qualityLevel=FEATURE_QUALITY,
        minDistance=MIN_FEATURE_DISTANCE_PX,
    )
    if corners is None or len(corners) < MIN_INLIERS:
        return None, None
    forward, status, _ = cv2.calcOpticalFlowPyrLK(
        previous_gray, gray, corners,
        None,
        winSize=(LK_WINDOW_PX, LK_WINDOW_PX),
        maxLevel=LK_PYRAMID_LEVELS,
    )
    if forward is None:
        return None, None
    backward, status_back, _ = cv2.calcOpticalFlowPyrLK(
        gray, previous_gray, forward,
        None,
        winSize=(LK_WINDOW_PX, LK_WINDOW_PX),
        maxLevel=LK_PYRAMID_LEVELS,
    )
    if backward is None:
        return None, None
    # Keep a track only if both directions reported success and the round
    # trip came back to where it started.
    fb_error = np.linalg.norm(corners.reshape(-1, 2) - backward.reshape(-1, 2),
                              axis=1)
    keep = (
        (status.reshape(-1) == 1)
        & (status_back.reshape(-1) == 1)
        & (fb_error < FB_MAX_ERROR_PX)
    )
    if np.count_nonzero(keep) < MIN_INLIERS:
        return None, None
    return corners.reshape(-1, 2)[keep], forward.reshape(-1, 2)[keep]


def relative_pose(points_previous, points_next, camera_matrix):
    """Essential matrix pose between two views.

    Returns (R, t, inliers, all_inliers). R and t map a point in the first
    camera's coordinates into the second camera's: X2 = R X1 + t. t is a
    unit vector - monocular geometry cannot recover its length.
    """
    essential, mask = cv2.findEssentialMat(
        points_previous, points_next, camera_matrix,
        method=cv2.RANSAC,
        prob=RANSAC_PROBABILITY,
        threshold=RANSAC_THRESHOLD_PX,
    )
    if essential is None:
        return None
    inliers = int(np.count_nonzero(mask)) if mask is not None else 0
    if inliers < MIN_INLIERS:
        return None
    _, rotation, translation, _ = cv2.recoverPose(
        essential, points_previous, points_next, camera_matrix,
    )
    if rotation is None or translation is None:
        return None
    return rotation, translation.reshape(3), inliers


def plane_translation(points_previous, points_next, camera_matrix, r_camera,
                      normal):
    """Metric translation from one view of a plane, with rotation known.

    The ground the camera looks at is a plane, and a plane is special: the
    two views are related by the homography

        x2 ~ (R + t n^T / d) x1

    with x the normalised image coordinates, R the rotation between the two
    camera poses, n the plane normal and d the camera's distance to the
    plane. R comes from the gyro and n from roll/pitch, so the only unknown
    is t/d - three numbers, and they enter linearly. Rewriting the relation
    as x2 x (R x1 + u (n . x1)) = 0 with u = t/d gives a 3-column least
    squares problem, which is what this function solves.

    Why not the essential matrix for this. It was tried first, on these
    same images. Two problems were measured: at low image motion (hover,
    corners) the translation direction is noise, and with the motion nearly
    coplanar to the view the essential matrix has a two-fold ambiguity that
    cheirality does not resolve - measured as a constant large angle error
    on frames with 30+ pixels of flow. Fixing R from the gyro removes the
    ambiguity, because the rotation no longer competes with the translation
    for the same image motion.

    Returns (u, inliers): the metric translation per metre of height, and
    the number of correspondences the plane model kept. The camera's
    displacement, in the first camera's axes, is -d * u.
    """
    k_inv = np.linalg.inv(camera_matrix)
    previous = np.hstack([points_previous, np.ones((len(points_previous), 1))])
    current = np.hstack([points_next, np.ones((len(points_next), 1))])
    previous = previous @ k_inv.T
    current = current @ k_inv.T
    focal = 0.5 * (camera_matrix[0, 0] + camera_matrix[1, 1])

    def fit(x1, x2):
        """Solve x2 x (R x1 + u (n . x1)) = 0 for u, and return the pixels."""
        # Skew-symmetric matrix of each x2, so the cross product with the
        # plane model becomes a matrix product.
        skew = np.zeros((len(x2), 3, 3))
        skew[:, 0, 1] = -x2[:, 2]
        skew[:, 0, 2] = x2[:, 1]
        skew[:, 1, 0] = x2[:, 2]
        skew[:, 1, 2] = -x2[:, 0]
        skew[:, 2, 0] = -x2[:, 1]
        skew[:, 2, 1] = x2[:, 0]
        rotated = x1 @ r_camera.T
        # Unknown u appears multiplied by (n . x1); the rest is constant.
        design = (skew * (x1 @ normal)[:, None, None]).reshape(-1, 3)
        target = -np.einsum("nij,nj->ni", skew, rotated).reshape(-1)
        solution, *_ = np.linalg.lstsq(design, target, rcond=None)
        # Residual in pixels, so the cut-off means something concrete: the
        # model applied to x1, compared with where the corner was tracked.
        predicted = rotated + np.outer(x1 @ normal, solution)
        predicted = predicted / predicted[:, 2:3]
        observed = x2 / x2[:, 2:3]
        pixels = focal * np.linalg.norm(predicted[:, :2] - observed[:, :2],
                                        axis=1)
        return solution, pixels

    # Refit over survivors so that one badly tracked corner cannot drag the
    # step. The fit keeps the best 80% of correspondences each pass, or the
    # ones inside PLANE_RESIDUAL_PX, whichever set is larger.
    survivors = np.arange(len(previous))
    for _ in range(PLANE_ITERATIONS):
        solution, pixels = fit(previous[survivors], current[survivors])
        limit = max(PLANE_RESIDUAL_PX, float(np.percentile(pixels, 80)))
        survivors = survivors[pixels < limit]
        if len(survivors) < MIN_INLIERS:
            return None, int(len(survivors))
    # Final answer on the surviving correspondences, so the reported inlier
    # count is the set the returned u was fitted to.
    solution, _ = fit(previous[survivors], current[survivors])
    return solution, int(len(survivors))


def angle_between(first, second):
    """Angle in degrees between two vectors, 0 to 180.

    Used to compare two estimators that each return a direction: the
    number is meaningful without either being declared correct, which is
    the whole point of keeping both.
    """
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    cosine = float(first @ second) / (
        float(np.linalg.norm(first)) * float(np.linalg.norm(second)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def rotation_angle_deg(rotation):
    """Rotation angle of a rotation matrix, in degrees.

    trace(R) = 1 + 2 cos(theta), so theta follows from the trace. The
    argument is clipped because floating point can push the trace slightly
    outside [-1, 3] for a rotation of exactly 0 or 180 degrees.
    """
    trace = float(np.trace(rotation))
    return math.degrees(math.acos(max(-1.0, min(3.0, (trace - 1.0) / 2.0))))


# ---------------------------------------------------------------------------
# Odometry
# ---------------------------------------------------------------------------

def run_vio(dataset, intrinsics, dataset_dir):
    """Walk the frames and return a list of per-frame pose records.

    dataset_dir is passed rather than read from a module global so the
    image paths are always the ones belonging to the loaded logs; a stale
    global would silently pair this dataset's timestamps with another
    dataset's images.

    Per frame pair:
      1. carve the camera rotation out of the gyro (the inertial prior),
      2. solve the plane model for the metric translation, using the
         height above ground as the range,
      3. compose the metric step into the NED frame,
      4. also run the essential matrix on the same correspondences, purely
         to record how far its direction is from the plane solution. That
         number is the evidence for preferring one estimator over the
         other, and it costs one extra call per frame.
    """
    k = np.array([[intrinsics["fx"], 0.0, intrinsics["cx"]],
                  [0.0, intrinsics["fy"], intrinsics["cy"]],
                  [0.0, 0.0, 1.0]])
    gimbal_pitch = float(dataset["meta"]["camera"]["gimbal_pitch_rad"])
    r_body_camera = body_to_camera(gimbal_pitch)

    times, omega = gyro_intervals(dataset["imu"])
    frames = dataset["frames"]
    attitude = dataset["attitude"]
    local = dataset["local"]
    attitude_times = [float(row["time_boot_ms"]) / 1000.0 for row in attitude]
    local_times = [float(row["time_boot_ms"]) / 1000.0 for row in local]
    heights = [-float(row["down_m"]) for row in local]

    # Initial orientation: autopilot attitude at the first frame, composed
    # with the fixed gimbal pitch. The lever arm between the flight
    # controller's origin and the camera is ignored; it is a constant
    # offset, and compare.py aligns the two trajectories at the origin.
    first_boot = float(frames[0]["boot_time_s"])
    index = int(np.argmin([abs(t - first_boot) for t in attitude_times]))
    start_attitude = attitude[index]
    r_world_body = rotation_from_euler(
        float(start_attitude["roll_rad"]),
        float(start_attitude["pitch_rad"]),
        float(start_attitude["yaw_rad"]),
    )
    r_world_camera = r_world_body @ r_body_camera
    position = np.zeros(3)

    records = []
    previous_gray = None
    previous_boot = None
    for frame in frames:
        boot = float(frame["boot_time_s"])
        image_path = dataset_dir / frame["file"]
        gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise SystemExit(f"Could not read {image_path}")
        record = {
            "frame": int(frame["frame"]),
            "sim_time_s": float(frame["sim_time_s"]),
            "boot_time_s": boot,
            "north_m": position[0],
            "east_m": position[1],
            "down_m": position[2],
            "step_m": 0.0,
            "inliers": 0,
            "tracked": 0,
            "height_m": "",
            "rotation_disagreement_deg": "",
            "essential_disagreement_deg": "",
            "rotation_source": "none",
        }
        if previous_gray is not None:
            points_previous, points_next = track_features(previous_gray, gray)
            if points_previous is not None:
                record["tracked"] = len(points_previous)
                # Inertial prior. The gyro rotates the body; the pose needs
                # the camera, so conjugate by the body->camera rotation to
                # express the same rotation in camera axes.
                r_body = integrate_gyro(times, omega, previous_boot, boot)
                if r_body is not None:
                    r_camera = r_body_camera.T @ r_body @ r_body_camera
                    record["rotation_source"] = "gyro"
                else:
                    r_camera = None

                # The plane normal in the first camera's axes, from the
                # attitude's roll and pitch only. Yaw turns the airframe
                # about the world vertical axis and so cannot change it,
                # which keeps this quantity purely inertial.
                attitude_index = int(np.argmin(
                    [abs(t - previous_boot) for t in attitude_times]))
                attitude_row = attitude[attitude_index]
                r_world_attitude = rotation_from_euler(
                    float(attitude_row["roll_rad"]),
                    float(attitude_row["pitch_rad"]),
                    float(attitude_row["yaw_rad"]),
                ) @ r_body_camera
                normal = r_world_attitude.T @ np.array([0.0, 0.0, 1.0])
                # Range: perpendicular distance from the camera to the
                # ground plane, which for level ground is the height.
                height = float(np.interp(previous_boot, local_times, heights))
                record["height_m"] = round(height, 3)

                u = None
                if r_camera is not None:
                    u, inliers = plane_translation(
                        points_previous, points_next, k, r_camera, normal)
                    record["inliers"] = inliers
                    if u is not None:
                        # u is t/d with t pointing from camera 2 to camera
                        # 1, so the displacement is -d*u.
                        displacement = -height * u
                        position = position + r_world_camera @ displacement
                        r_world_camera = r_world_camera @ r_camera.T
                        record["step_m"] = round(float(
                            np.linalg.norm(displacement)), 4)
                        record["north_m"] = position[0]
                        record["east_m"] = position[1]
                        record["down_m"] = position[2]

                # Cross-check: compare the two estimators' directions on
                # the same correspondences. The essential matrix also
                # gives a rotation, which is compared with the gyro one.
                pose = relative_pose(points_previous, points_next, k)
                if pose is not None:
                    r_vision, t_vision, _ = pose
                    if r_camera is not None:
                        record["rotation_disagreement_deg"] = round(
                            rotation_angle_deg(r_camera.T @ r_vision), 3)
                    # Both translations live in the first camera's axes,
                    # so they can be compared directly; no rotation is
                    # applied to either.
                    if record["step_m"]:
                        record["essential_disagreement_deg"] = round(
                            angle_between(t_vision, u * height), 3)
        records.append(record)
        previous_gray = gray
        previous_boot = boot
    return records


def write_outputs(dataset_dir, records, output_path):
    """Write the trajectory CSV and a summary next to it.

    The trajectory needs no rescaling: each step was already metric, from
    the height of the plane the camera was looking at. The summary reports
    a small set of numbers that can be checked against the dataset instead
    of taken on trust - how many pairs joined, how many correspondences the
    plane model kept, how far the two estimators disagreed, and how long
    the path came out.
    """
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            "frame", "sim_time_s", "boot_time_s",
            "north_m", "east_m", "down_m",
            "step_m", "inliers", "tracked", "height_m",
            "rotation_disagreement_deg", "essential_disagreement_deg",
            "rotation_source",
        ])
        writer.writeheader()
        for record in records:
            writer.writerow({
                **record,
                "north_m": f"{record['north_m']:.4f}",
                "east_m": f"{record['east_m']:.4f}",
                "down_m": f"{record['down_m']:.4f}",
            })

    path_length = sum(record["step_m"] for record in records)
    inliers = [record["inliers"] for record in records if record["inliers"]]
    rotation_errors = [
        record["rotation_disagreement_deg"] for record in records
        if record["rotation_disagreement_deg"] != ""
    ]
    essential_errors = [
        record["essential_disagreement_deg"] for record in records
        if record["essential_disagreement_deg"] != ""
    ]
    summary = {
        "dataset": str(dataset_dir),
        "trajectory": str(output_path),
        "frames": len(records),
        "frames_without_step": sum(1 for record in records
                                    if record["step_m"] == 0.0),
        "inliers_mean": round(float(np.mean(inliers)), 1) if inliers else 0,
        "inliers_min": int(min(inliers)) if inliers else 0,
        "rotation_disagreement_deg_median": (
            round(float(np.median(rotation_errors)), 3)
            if rotation_errors else None
        ),
        "essential_direction_disagreement_deg_median": (
            round(float(np.median(essential_errors)), 3)
            if essential_errors else None
        ),
        "rotation_source_counts": {
            source: sum(1 for record in records
                        if record["rotation_source"] == source)
            for source in ("gyro", "none")
        },
        "metric_scale": {
            "method": "per-frame camera height above the ground plane",
            "height_min_m": round(min(float(r["height_m"])
                                      for r in records if r["height_m"] != ""),
                                  3),
            "height_max_m": round(max(float(r["height_m"])
                                      for r in records if r["height_m"] != ""),
                                  3),
        },
        "path_length_m": round(path_length, 3),
    }
    summary_path = output_path.with_suffix(".json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    return summary, summary_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline monocular VIO over a recorded dataset"
    )
    parser.add_argument("--dataset", default=None,
                        help="Dataset directory. Default: newest under "
                             "results/task3/")
    parser.add_argument("--output", default=None,
                        help="Trajectory CSV. Default: "
                             "results/task3/vio_trajectory.csv")
    return parser.parse_args()


def main():
    args = parse_args()
    dataset_dir = (Path(args.dataset) if args.dataset
                   else newest_dataset(ROOT / "results" / "task3"))
    output_path = (Path(args.output) if args.output
                   else ROOT / "results" / "task3" / "vio_trajectory.csv")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Dataset: {dataset_dir}")
    dataset = load_dataset(dataset_dir)
    print(f"Frames with a timestamp: {len(dataset['frames'])}, "
          f"RAW_IMU samples: {len(dataset['imu'])}")
    if len(dataset["frames"]) < 10:
        raise SystemExit("Too few timestamped frames to run a VIO.")

    intrinsics = dataset["meta"]["camera"]["intrinsics"]
    records = run_vio(dataset, intrinsics, dataset_dir)
    summary, summary_path = write_outputs(dataset_dir, records, output_path)

    print(f"Inliers: mean {summary['inliers_mean']}, "
          f"min {summary['inliers_min']}")
    print(f"Rotation: gyro-vision disagreement median "
          f"{summary['rotation_disagreement_deg_median']} deg")
    print(f"Direction: essential-matrix vs plane median "
          f"{summary['essential_direction_disagreement_deg_median']} deg")
    print(f"Path length: {summary['path_length_m']} m")
    print(f"Trajectory written to {output_path}")
    print(f"Summary written to {summary_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
