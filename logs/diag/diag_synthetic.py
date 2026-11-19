#!/usr/bin/env python3
"""One-off diagnostic: does scripts/vio.py's geometry recover a known motion?

Not a deliverable. Two synthetic views of a textured ground plane are
rendered with a known camera translation, then pushed through exactly the
functions the real dataset uses (track_features, relative_pose). If the
recovered translation direction does not match the one that was rendered,
the bug is in the script, not in the recorded images.
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import vio  # noqa: E402

HEIGHT_M = 5.0
WIDTH_PX, HEIGHT_PX = 640, 480
FX, FY, CX, CY = 205.4696, 205.4697, 320.0, 240.0
PITCH_DOWN_RAD = 0.7
STEP_M = 0.3            # 3 m/s at 10 Hz
STEPS = 12              # walk a whole leg, not just one pair


def render(north, east, r_world_camera):
    """Render the plane seen by a camera at (north, east, -HEIGHT_M).

    For every pixel the view ray in camera axes is rotated into world axes,
    intersected with the ground plane (down = 0) and sampled with a
    checkerboard so the image has corners to track.
    """
    u, v = np.meshgrid(np.arange(WIDTH_PX), np.arange(HEIGHT_PX))
    rays = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, dtype=float)],
                    axis=-1)
    rays = rays @ r_world_camera.T          # camera axes -> world axes
    # The camera is at down = -HEIGHT_M and the ground at down = 0, so the
    # ray parameter to the ground is HEIGHT_M / ray_down. Rays pointing up
    # never hit it, and those pixels stay black.
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = HEIGHT_M / rays[..., 2]
    hit_north = north + scale * rays[..., 0]
    hit_east = east + scale * rays[..., 1]
    # Smooth periodic texture: a fine checkerboard plus a coarse one, so
    # Shi-Tomasi finds corners everywhere and LK has a unique match.
    fine = ((np.floor(hit_north * 2.0) + np.floor(hit_east * 2.0)) % 2)
    coarse = ((np.floor(hit_north * 0.4) + np.floor(hit_east * 0.4)) % 2)
    image = 60 + 70 * fine + 40 * coarse
    image = np.nan_to_num(image, nan=0.0).clip(0, 255).astype(np.uint8)
    return image


def main():
    r_world_body = vio.rotation_from_euler(0.0, 0.0, 0.0)
    r_world_camera = r_world_body @ vio.body_to_camera(PITCH_DOWN_RAD)
    k = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])

    # Truth: the camera walks north (world +x) STEP_M per frame.
    truth_direction_camera = r_world_camera.T @ np.array([1.0, 0.0, 0.0])
    print("truth direction in camera axes:",
          np.round(truth_direction_camera, 3))

    errors = []
    inliers = []
    previous = None
    for index in range(STEPS):
        image = render(index * STEP_M, 0.0, r_world_camera)
        if previous is not None:
            a, b = vio.track_features(previous, image)
            if a is None:
                print(f"step {index}: tracking failed")
            else:
                pose = vio.relative_pose(a, b, k)
                if pose is None:
                    print(f"step {index}: no pose")
                else:
                    _, t, count = pose
                    # t is the direction from camera-1 to camera-2? It is
                    # t in X2 = R X1 + t, so the camera moved by -R^T t.
                    direction = -t / np.linalg.norm(t)
                    error = math.degrees(math.acos(max(-1.0, min(1.0, float(
                        direction @ truth_direction_camera)))))
                    errors.append(error)
                    inliers.append(count)
                    print(f"step {index}: tracked {len(a):3d} inliers {count:3d} "
                          f"t={np.round(t, 3)} angle_error={error:6.1f} deg")
        previous = image
    print(f"median angle error: {np.median(errors):.1f} deg over "
          f"{len(errors)} steps, median inliers {np.median(inliers):.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
