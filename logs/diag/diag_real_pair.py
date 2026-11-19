#!/usr/bin/env python3
"""One-off diagnostic: per-pair look at the real dataset's vision geometry.

Not a deliverable. For a run of consecutive frames it prints, per pair, the
pixel flow statistics, the translation direction the essential matrix
recovers, and the direction the aircraft actually moved, both in the first
camera's axes. A constant disagreement means a convention problem; a random
one means the matches are not describing real motion.
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import vio  # noqa: E402

DATASET = ROOT / "results" / "task3" / "dataset_20261120_012619"


def angle(a, b):
    """Angle in degrees between two vectors."""
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
    frames = dataset["frames"]
    local = dataset["local"]
    attitude = dataset["attitude"]
    local_times = [float(row["time_boot_ms"]) / 1000.0 for row in local]
    attitude_times = [float(row["time_boot_ms"]) / 1000.0 for row in attitude]

    # Frames 150-200 sit inside the north and east legs of the square.
    start, stop = 150, 200
    previous_gray = None
    previous_boot = None
    errors = []
    for index in range(start, stop):
        frame = frames[index]
        boot = float(frame["boot_time_s"])
        gray = vio.cv2.imread(str(DATASET / frame["file"]), vio.cv2.IMREAD_GRAYSCALE)
        if previous_gray is not None:
            a, b = vio.track_features(previous_gray, gray)
            if a is None:
                print(f"{index}: no tracks")
            else:
                pose = vio.relative_pose(a, b, k)
                flow = b - a
                speed = np.linalg.norm(flow, axis=1)
                # Truth velocity in world NED, then in this camera's axes.
                velocity = np.array([
                    np.interp(boot, local_times, [float(r["vn_m_s"]) for r in local]),
                    np.interp(boot, local_times, [float(r["ve_m_s"]) for r in local]),
                    np.interp(boot, local_times, [float(r["vd_m_s"]) for r in local]),
                ])
                att_index = int(np.argmin([abs(t - boot) for t in attitude_times]))
                row = attitude[att_index]
                r_world_camera = vio.rotation_from_euler(
                    float(row["roll_rad"]), float(row["pitch_rad"]),
                    float(row["yaw_rad"])) @ r_body_camera
                truth_camera = r_world_camera.T @ velocity
                if pose is None:
                    print(f"{index}: flow median {np.median(speed):5.1f} px, "
                          f"no pose")
                else:
                    _, t, inliers = pose
                    # -R^T t is the displacement in the first camera's axes.
                    direction = -t / np.linalg.norm(t)
                    error = angle(direction, truth_camera)
                    errors.append(error)
                    print(f"{index}: flow {np.median(speed):5.1f} px  "
                          f"tracks {len(a):3d} inl {inliers:3d}  "
                          f"est {np.round(direction, 2)}  "
                          f"truth {np.round(truth_camera / np.linalg.norm(truth_camera), 2)}"
                          f"  err {error:5.1f} deg  speed "
                          f"{np.linalg.norm(velocity):4.2f} m/s")
        previous_gray = gray
        previous_boot = boot
    errors = np.array(errors)
    print(f"angle error: median {np.median(errors):.1f} deg, "
          f"below 30 deg {np.mean(errors < 30):.0%}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
