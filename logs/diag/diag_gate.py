#!/usr/bin/env python3
"""One-off diagnostic: sweep a parallax gate on the real dataset.

Not a deliverable. Two things are measured for each candidate threshold:

1. how many frame pairs survive,
2. what the unscaled trajectory spans in north, east and down.

The second number is the important one. Each accepted pair contributes a
unit translation, so a frame pair with almost no motion (hover, corner) is
a full metre of invented travel; the gate is the only thing that stops
those pixels-as-noise steps from swamping the real motion.
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import vio  # noqa: E402

DATASET = ROOT / "results" / "task3" / "dataset_20261120_012619"


def integrate(gate_px):
    """Unit-step integrate the trajectory with a median-flow gate."""
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
    attitude_times = [float(row["time_boot_ms"]) / 1000.0 for row in attitude]
    first = float(frames[0]["boot_time_s"])
    index = int(np.argmin([abs(t - first) for t in attitude_times]))
    start = attitude[index]
    r_world_camera = vio.rotation_from_euler(
        float(start["roll_rad"]), float(start["pitch_rad"]),
        float(start["yaw_rad"])) @ r_body_camera

    position = np.zeros(3)
    path = 0.0
    accepted = 0
    pairs = 0
    previous_gray = None
    previous_boot = None
    trail = []
    for frame in frames:
        boot = float(frame["boot_time_s"])
        gray = vio.cv2.imread(str(DATASET / frame["file"]), vio.cv2.IMREAD_GRAYSCALE)
        if previous_gray is not None:
            pairs += 1
            a, b = vio.track_features(previous_gray, gray)
            pose = vio.relative_pose(a, b, k) if a is not None else None
            if pose is not None:
                flow = np.median(np.linalg.norm(b - a, axis=1))
                if flow >= gate_px:
                    r_vision, t_vision, _ = pose
                    r_body = vio.integrate_gyro(times, omega, previous_boot,
                                               boot)
                    r_camera = (r_body_camera.T @ r_body @ r_body_camera
                                if r_body is not None else r_vision)
                    step = r_world_camera @ (-r_camera.T @ t_vision)
                    position = position + step
                    path += float(np.linalg.norm(step))
                    r_world_camera = r_world_camera @ r_camera.T
                    accepted += 1
        trail.append(position.copy())
        previous_gray = gray
        previous_boot = boot
    trail = np.array(trail)
    return {
        "gate": gate_px,
        "accepted": accepted,
        "pairs": pairs,
        "north_range": trail[:, 0].max() - trail[:, 0].min(),
        "east_range": trail[:, 1].max() - trail[:, 1].min(),
        "down_range": trail[:, 2].max() - trail[:, 2].min(),
        "path": path,
        "final": trail[-1],
    }


def main():
    print(f"{'gate':>5} {'ok':>4} {'pairs':>6} {'N range':>8} {'E range':>8} "
          f"{'D range':>8} {'path':>8}  final (n, e, d)")
    for gate in (0.0, 2.0, 4.0, 6.0, 8.0, 12.0, 16.0):
        result = integrate(gate)
        print(f"{result['gate']:5.1f} {result['accepted']:4d} "
              f"{result['pairs']:6d} {result['north_range']:8.1f} "
              f"{result['east_range']:8.1f} {result['down_range']:8.1f} "
              f"{result['path']:8.1f}  "
              f"({result['final'][0]:.1f}, {result['final'][1]:.1f}, "
              f"{result['final'][2]:.1f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
