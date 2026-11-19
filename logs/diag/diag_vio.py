#!/usr/bin/env python3
"""One-off diagnostic: is the essential-matrix translation direction usable?

Not a deliverable. It reuses scripts/vio.py and asks three questions about
every frame pair:

1. What is the estimated displacement direction in world NED?
2. What is the aircraft's real velocity direction at that moment?
3. How much of the estimated direction is vertical (down) versus horizontal?

A ground-looking camera sees a plane, and a planar scene is exactly where
findEssentialMat is degenerate, so this measures the damage rather than
assuming there is none.
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import vio  # noqa: E402

DATASET = ROOT / "results" / "task3" / "dataset_20261120_012619"


def reference_velocity(local, t):
    """Interpolated velocity (north, east, down) at autopilot time t."""
    times = [float(row["time_boot_ms"]) / 1000.0 for row in local]
    vn = [float(row["vn_m_s"]) for row in local]
    ve = [float(row["ve_m_s"]) for row in local]
    vd = [float(row["vd_m_s"]) for row in local]
    return np.array([np.interp(t, times, vn),
                     np.interp(t, times, ve),
                     np.interp(t, times, vd)])


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
    attitude = dataset["attitude"]
    local = dataset["local"]
    attitude_times = [float(row["time_boot_ms"]) / 1000.0 for row in attitude]

    first = float(frames[0]["boot_time_s"])
    index = int(np.argmin([abs(t - first) for t in attitude_times]))
    start = attitude[index]
    r_world_body = vio.rotation_from_euler(
        float(start["roll_rad"]), float(start["pitch_rad"]),
        float(start["yaw_rad"]))
    r_world_camera = r_world_body @ r_body_camera

    previous_gray = None
    previous_boot = None
    vertical_fraction = []
    horizontal_angle = []
    angles_3d = []
    for frame in frames:
        boot = float(frame["boot_time_s"])
        gray = vio.cv2.imread(str(DATASET / frame["file"]), vio.cv2.IMREAD_GRAYSCALE)
        if previous_gray is not None:
            previous, current = vio.track_features(previous_gray, gray)
            pose = (vio.relative_pose(previous, current, k)
                    if previous is not None else None)
            if pose is not None:
                r_vision, t_vision, inliers = pose
                r_body = vio.integrate_gyro(times, omega, previous_boot, boot)
                r_camera = (r_body_camera.T @ r_body @ r_body_camera
                            if r_body is not None else r_vision)
                displacement = r_world_camera @ (-r_camera.T @ t_vision)
                velocity = reference_velocity(local, boot)
                # Ignore near-stationary moments: their direction is noise.
                if np.linalg.norm(velocity[:2]) > 0.3:
                    vertical_fraction.append(
                        abs(displacement[2]) / np.linalg.norm(displacement))
                    v_hat = velocity / np.linalg.norm(velocity)
                    d_hat = displacement / np.linalg.norm(displacement)
                    cos_xy = (d_hat[0] * v_hat[0] + d_hat[1] * v_hat[1])
                    horizontal_angle.append(math.degrees(math.acos(
                        max(-1.0, min(1.0, cos_xy)))))
                    angles_3d.append(math.degrees(math.acos(
                        max(-1.0, min(1.0, float(d_hat @ v_hat))))))
                r_world_camera = r_world_camera @ r_camera.T
        previous_gray = gray
        previous_boot = boot

    vf = np.array(vertical_fraction)
    ha = np.array(horizontal_angle)
    a3 = np.array(angles_3d)
    print(f"moving frame pairs: {len(vf)}")
    print(f"|dz|/|d| median {np.median(vf):.3f}  "
          f"mean {vf.mean():.3f}  >0.8: {np.mean(vf > 0.8):.1%}")
    print(f"horizontal direction error vs reference: median {np.median(ha):.1f} deg"
          f"  (uniform random would be ~90)")
    print(f"3D direction error vs reference: median {np.median(a3):.1f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
