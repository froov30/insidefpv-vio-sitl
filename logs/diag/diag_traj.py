#!/usr/bin/env python3
"""One-off diagnostic: full-flight metric integration with the plane solve.

Not a deliverable. Integrates the metric step from the R-fixed plane fit
over all 472 frames and prints how the resulting trajectory compares with
the reference square, with and without a minimum-parallax gate.
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import vio  # noqa: E402

DATASET = ROOT / "results" / "task3" / "dataset_20261120_012619"


def main():
    dataset = vio.load_dataset(DATASET)
    intrinsics = dataset["meta"]["camera"]["intrinsics"]
    camera_matrix = np.array([[intrinsics["fx"], 0.0, intrinsics["cx"]],
                              [0.0, intrinsics["fy"], intrinsics["cy"]],
                              [0.0, 0.0, 1.0]])
    r_body_camera = vio.body_to_camera(
        float(dataset["meta"]["camera"]["gimbal_pitch_rad"]))
    times, omega = vio.gyro_intervals(dataset["imu"])
    frames = dataset["frames"]
    attitude = dataset["attitude"]
    local = dataset["local"]
    attitude_times = [float(row["time_boot_ms"]) / 1000.0 for row in attitude]
    local_times = [float(row["time_boot_ms"]) / 1000.0 for row in local]
    heights = [-float(row["down_m"]) for row in local]

    first = float(frames[0]["boot_time_s"])
    index = int(np.argmin([abs(t - first) for t in attitude_times]))
    start = attitude[index]
    r_world_camera = vio.rotation_from_euler(
        float(start["roll_rad"]), float(start["pitch_rad"]),
        float(start["yaw_rad"])) @ r_body_camera

    position = np.zeros(3)
    trail = []
    steps = []
    previous_gray = None
    previous_boot = None
    for frame in frames:
        boot = float(frame["boot_time_s"])
        gray = vio.cv2.imread(str(DATASET / frame["file"]), vio.cv2.IMREAD_GRAYSCALE)
        if previous_gray is not None:
            a, b = vio.track_features(previous_gray, gray)
            r_body = vio.integrate_gyro(times, omega, previous_boot, boot)
            if a is not None and r_body is not None:
                r_camera = r_body_camera.T @ r_body @ r_body_camera
                att_index = int(np.argmin([abs(t - boot) for t in attitude_times]))
                row = attitude[att_index]
                r_world_attitude = vio.rotation_from_euler(
                    float(row["roll_rad"]), float(row["pitch_rad"]),
                    float(row["yaw_rad"])) @ r_body_camera
                normal = r_world_attitude.T @ np.array([0.0, 0.0, 1.0])
                height = float(np.interp(boot, local_times, heights))
                flow = float(np.median(np.linalg.norm(b - a, axis=1)))
                u = vio.plane_translation(a, b, camera_matrix, r_camera, normal)
                if u[0] is not None:
                    displacement = -height * u[0]
                    position = position + r_world_camera @ displacement
                    steps.append((boot, flow, float(np.linalg.norm(displacement))))
                r_world_camera = r_world_camera @ r_camera.T
        trail.append(position.copy())
        previous_gray = gray
        previous_boot = boot
    trail = np.array(trail)
    print(f"trajectory ranges: N {trail[:, 0].max() - trail[:, 0].min():.2f} "
          f"E {trail[:, 1].max() - trail[:, 1].min():.2f} "
          f"D {trail[:, 2].max() - trail[:, 2].min():.2f}")
    print(f"final: {np.round(trail[-1], 2)}   reference final: "
          f"{np.round([float(local[-1]['north_m']), float(local[-1]['east_m']), float(local[-1]['down_m'])], 2)}")
    steps = np.array(steps)
    print("step length vs median flow (buckets):")
    for low, high in ((0, 2), (2, 5), (5, 10), (10, 20), (20, 100)):
        pick = (steps[:, 1] >= low) & (steps[:, 1] < high)
        if pick.any():
            print(f"  flow {low:3d}-{high:3d} px: {pick.sum():3d} steps, "
                  f"median length {np.median(steps[pick, 2]):.3f} m, "
                  f"p90 {np.percentile(steps[pick, 2], 90):.3f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
