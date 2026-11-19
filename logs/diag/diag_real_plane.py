#!/usr/bin/env python3
"""One-off diagnostic: plane-homography metric steps on the real dataset.

Not a deliverable. Same frame pairs as diag_real_pair.py, but the step is
taken from the plane homography with the gyro rotation, the ground normal
from roll/pitch and the camera height as the range. Prints direction error
and step length against the reference, so the estimator can be judged on
both counts before it goes into vio.py.
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import vio  # noqa: E402

DATASET = ROOT / "results" / "task3" / "dataset_20261120_012619"


def plane_step(points_previous, points_next, k, r_camera, normal, distance):
    """Metric translation between two views of a plane at a known range.

    Returns (t, inliers) with t in the first camera's axes, or None.
    """
    homography, mask = vio.cv2.findHomography(
        points_previous, points_next, vio.cv2.RANSAC, 1.5)
    if homography is None:
        return None
    inliers = int(np.count_nonzero(mask))
    normalized = np.linalg.inv(k) @ homography @ k
    design = np.zeros((9, 4))
    design[:, 0] = r_camera.reshape(9)
    for axis in range(3):
        design[axis * 3:(axis + 1) * 3, axis + 1] = normal
    solution, *_ = np.linalg.lstsq(design, normalized.reshape(9), rcond=None)
    lam, w = solution[0], solution[1:]
    if abs(lam) < 1e-6:
        return None
    return distance * w / lam, inliers


def main():
    dataset = vio.load_dataset(DATASET)
    intrinsics = dataset["meta"]["camera"]["intrinsics"]
    k = np.array([[intrinsics["fx"], 0.0, intrinsics["cx"]],
                  [0.0, intrinsics["fy"], intrinsics["cy"]],
                  [0.0, 0.0, 1.0]])
    r_body_camera = vio.body_to_camera(
        float(dataset["meta"]["camera"]["gimbal_pitch_rad"]))
    times, omega = vio.gyro_intervals(dataset["imu"])
    frames = dataset["frames"]
    local = dataset["local"]
    attitude = dataset["attitude"]
    local_times = [float(row["time_boot_ms"]) / 1000.0 for row in local]
    attitude_times = [float(row["time_boot_ms"]) / 1000.0 for row in attitude]
    heights = [-float(row["down_m"]) for row in local]

    start, stop = 150, 200
    previous_gray = None
    previous_boot = None
    direction_errors = []
    length_errors = []
    for index in range(start, stop):
        frame = frames[index]
        boot = float(frame["boot_time_s"])
        gray = vio.cv2.imread(str(DATASET / frame["file"]), vio.cv2.IMREAD_GRAYSCALE)
        if previous_gray is not None:
            a, b = vio.track_features(previous_gray, gray)
            r_body = vio.integrate_gyro(times, omega, previous_boot, boot)
            if a is not None and r_body is not None:
                r_camera = r_body_camera.T @ r_body @ r_body_camera
                att_index = int(np.argmin([abs(t - boot) for t in attitude_times]))
                row = attitude[att_index]
                # Ground normal in the first camera's axes. It depends only
                # on roll, pitch and the fixed gimbal angle: yaw turns the
                # body about the world vertical, which leaves it unchanged.
                r_world_camera = vio.rotation_from_euler(
                    float(row["roll_rad"]), float(row["pitch_rad"]),
                    float(row["yaw_rad"])) @ r_body_camera
                normal = r_world_camera.T @ np.array([0.0, 0.0, 1.0])
                height = float(np.interp(boot, local_times, heights))
                result = plane_step(a, b, k, r_camera, normal, height)
                if result is not None:
                    t, inliers = result
                    displacement = -t          # t points camera 2 -> 1
                    velocity = np.array([
                        np.interp(boot, local_times,
                                  [float(r["vn_m_s"]) for r in local]),
                        np.interp(boot, local_times,
                                  [float(r["ve_m_s"]) for r in local]),
                        np.interp(boot, local_times,
                                  [float(r["vd_m_s"]) for r in local]),
                    ])
                    true_step = velocity * (boot - previous_boot)
                    if np.linalg.norm(true_step) > 0.02:
                        cos = float(
                            (displacement @ true_step)
                            / (np.linalg.norm(displacement)
                               * np.linalg.norm(true_step)))
                        direction_errors.append(
                            math.degrees(math.acos(max(-1.0, min(1.0, cos)))))
                        length_errors.append(
                            np.linalg.norm(displacement)
                            / np.linalg.norm(true_step))
                        print(f"{index}: inl {inliers:3d} "
                              f"step {np.linalg.norm(displacement):.3f} m "
                              f"(truth {np.linalg.norm(true_step):.3f}) "
                              f"ratio {length_errors[-1]:.2f} "
                              f"dir err {direction_errors[-1]:5.1f} deg "
                              f"speed {np.linalg.norm(velocity):4.2f}")
        previous_gray = gray
        previous_boot = boot

    direction_errors = np.array(direction_errors)
    length_errors = np.array(length_errors)
    print(f"pairs {len(direction_errors)}  direction median "
          f"{np.median(direction_errors):.1f} deg, below 20 deg "
          f"{np.mean(direction_errors < 20):.0%}")
    print(f"step length ratio: median {np.median(length_errors):.2f}, "
          f"p10 {np.percentile(length_errors, 10):.2f}, "
          f"p90 {np.percentile(length_errors, 90):.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
