#!/usr/bin/env python3
"""One-off diagnostic: essential matrix vs homography on this scene.

Not a deliverable. The dataset is a down-tilted camera looking almost
entirely at the runway/ground plane, and a plane is the classic degenerate
case for the eight-point essential-matrix solver: many essential matrices
fit a coplanar point set, so the recovered translation direction is not
unique. This script tracks one set of corners per frame pair and compares
the direction implied by (a) the essential matrix and (b) the homography
decomposition, against the aircraft's real velocity.
"""

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
    return np.array([
        np.interp(t, times, [float(r["vn_m_s"]) for r in local]),
        np.interp(t, times, [float(r["ve_m_s"]) for r in local]),
        np.interp(t, times, [float(r["vd_m_s"]) for r in local]),
    ])


def homography_direction(points_previous, points_next, k, ground_normal):
    """Translation direction from the plane homography, most plausible sign.

    decomposeHomographyMat returns four (R, t, n) solutions. The plane
    normal is known here - it is straight up in the world - so the solution
    whose normal matches it is picked. Both signs of t are returned because
    the caller has to break that tie with data: the decomposition only says
    the motion is along a line.
    """
    homography, mask = vio.cv2.findHomography(
        points_previous, points_next, vio.cv2.RANSAC, 1.0)
    if homography is None:
        return None
    count, rotations, translations, normals = vio.cv2.decomposeHomographyMat(
        homography, k)
    if count == 0:
        return None
    best = None
    for index in range(count):
        normal = normals[index].reshape(3)
        alignment = abs(float(normal @ ground_normal))
        if best is None or alignment > best[0]:
            best = (alignment, translations[index].reshape(3),
                    rotations[index], normal)
    _, translation, rotation, normal = best
    return translation, rotation, normal


def angle_between(a, b):
    """Angle in degrees between two vectors, 0..180."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b)))))


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
    r_world_camera = vio.rotation_from_euler(
        float(start["roll_rad"]), float(start["pitch_rad"]),
        float(start["yaw_rad"])) @ r_body_camera

    errors_essential = []
    errors_homography = []
    errors_essential_3d = []
    errors_homography_3d = []
    previous_gray = None
    previous_boot = None
    for frame in frames:
        boot = float(frame["boot_time_s"])
        gray = vio.cv2.imread(str(DATASET / frame["file"]), vio.cv2.IMREAD_GRAYSCALE)
        if previous_gray is not None:
            points_previous, points_next = vio.track_features(previous_gray, gray)
            if points_previous is not None:
                pose = vio.relative_pose(points_previous, points_next, k)
                r_body = vio.integrate_gyro(times, omega, previous_boot, boot)
                if pose is not None and r_body is not None:
                    r_vision, t_vision, _ = pose
                    r_camera = r_body_camera.T @ r_body @ r_body_camera
                    velocity = reference_velocity(local, boot)
                    if np.linalg.norm(velocity[:2]) > 0.3:
                        truth = r_world_camera.T @ velocity
                        # Ground normal in the first camera's axes: world up
                        # is (0, 0, -1) in NED.
                        ground_normal = r_world_camera.T @ np.array([0, 0, -1.0])
                        result = homography_direction(
                            points_previous, points_next, k, ground_normal)
                        if result is not None:
                            t_homography, _, _ = result
                            displacement = r_world_camera @ (
                                -r_camera.T @ t_vision)
                            truth_hat = truth / np.linalg.norm(truth)
                            d_hat = displacement / np.linalg.norm(displacement)
                            errors_essential.append(
                                angle_between(d_hat[:2], truth_hat[:2]))
                            errors_essential_3d.append(
                                angle_between(d_hat, truth_hat))
                            # Homography t is given in the first camera's
                            # axes; try both signs and keep the better one,
                            # which is fair to the method: the sign is a
                            # cheirality question, not a modelling one.
                            for sign in (1.0, -1.0):
                                h_hat = r_world_camera @ (sign * t_homography)
                                h_hat = h_hat / np.linalg.norm(h_hat)
                                if sign > 0:
                                    best_2d = angle_between(h_hat[:2],
                                                            truth_hat[:2])
                                    best_3d = angle_between(h_hat, truth_hat)
                                else:
                                    best_2d = min(best_2d, angle_between(
                                        h_hat[:2], truth_hat[:2]))
                                    best_3d = min(best_3d, angle_between(
                                        h_hat, truth_hat))
                            errors_homography.append(best_2d)
                            errors_homography_3d.append(best_3d)
                    r_world_camera = r_world_camera @ r_camera.T
        previous_gray = gray
        previous_boot = boot

    ee = np.array(errors_essential)
    eh = np.array(errors_homography)
    ee3 = np.array(errors_essential_3d)
    eh3 = np.array(errors_homography_3d)
    print(f"pairs compared: {len(ee)}")
    print(f"essential   horizontal error: median {np.median(ee):6.1f} deg"
          f"  3D median {np.median(ee3):6.1f} deg")
    print(f"homography  horizontal error: median {np.median(eh):6.1f} deg"
          f"  3D median {np.median(eh3):6.1f} deg")
    print(f"essential   within 30 deg: {np.mean(ee < 30):.1%} 3D "
          f"{np.mean(ee3 < 30):.1%}")
    print(f"homography  within 30 deg: {np.mean(eh < 30):.1%} 3D "
          f"{np.mean(eh3 < 30):.1%}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
