#!/usr/bin/env python3
"""One-off diagnostic: metric translation from the plane homography.

Not a deliverable. The ground the camera sees is a plane, and the camera's
perpendicular distance to it is the altitude, which the dataset already
has. A plane-induced homography satisfies x2 ~ H x1 with

    H = R + t n^T / d

so with the rotation R from the gyro, the plane normal n and the distance d
both known, the metric translation falls straight out:

    t = d (H - R) n

This computes that on synthetic images with a known 0.3 m step and checks
the direction and the length, so the convention is settled before it is
used on real frames.
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
STEP_M = 0.3
STEPS = 6


def render(north, east, r_world_camera):
    """Render the ground plane seen from (north, east, -HEIGHT_M)."""
    u, v = np.meshgrid(np.arange(WIDTH_PX), np.arange(HEIGHT_PX))
    rays = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, dtype=float)],
                    axis=-1)
    rays = rays @ r_world_camera.T
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = HEIGHT_M / rays[..., 2]
    hit_north = north + scale * rays[..., 0]
    hit_east = east + scale * rays[..., 1]
    fine = ((np.floor(hit_north * 2.0) + np.floor(hit_east * 2.0)) % 2)
    coarse = ((np.floor(hit_north * 0.4) + np.floor(hit_east * 0.4)) % 2)
    image = 60 + 70 * fine + 40 * coarse
    return np.nan_to_num(image, nan=0.0).clip(0, 255).astype(np.uint8)


def main():
    r_world_camera = vio.body_to_camera(PITCH_DOWN_RAD)
    k = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
    r_camera = np.eye(3)                       # no rotation between frames
    # Plane normal in camera axes, pointing from the camera to the ground:
    # world "down" is (0, 0, +1) in NED. With this orientation the plane is
    # n^T X = d for every ground point X, d = HEIGHT_M.
    normal = r_world_camera.T @ np.array([0.0, 0.0, 1.0])
    truth = r_world_camera.T @ np.array([STEP_M, 0.0, 0.0])
    print("normal in camera axes:", np.round(normal, 3))
    print("truth step in camera axes:", np.round(truth, 3))

    previous = None
    for index in range(STEPS):
        image = render(index * STEP_M, 0.0, r_world_camera)
        if previous is not None:
            a, b = vio.track_features(previous, image)
            homography, mask = vio.cv2.findHomography(a, b, vio.cv2.RANSAC, 1.5)
            inliers = int(np.count_nonzero(mask))
            # findHomography works in pixels; the plane model lives in
            # normalised camera coordinates, so undo the calibration first.
            kinv = np.linalg.inv(k)
            normalized = kinv @ homography @ k
            # The returned matrix is also only defined up to a scale, so
            # solve H = lam * R + w n^T for (lam, w) by least squares; the
            # model is linear in those four unknowns. Then t = d * w / lam.
            design = np.zeros((9, 4))
            design[:, 0] = r_camera.reshape(9)
            for axis in range(3):
                design[axis * 3:(axis + 1) * 3, axis + 1] = normal
            solution, *_ = np.linalg.lstsq(design, normalized.reshape(9),
                                           rcond=None)
            lam, w = solution[0], solution[1:]
            estimate = HEIGHT_M * w / lam
            error = math.degrees(math.acos(max(-1.0, min(1.0, float(
                estimate @ truth / (np.linalg.norm(estimate)
                                    * np.linalg.norm(truth)))))))
            print(f"step {index}: inliers {inliers:3d} lam {lam:7.4f} "
                  f"t={np.round(estimate, 3)} |t|={np.linalg.norm(estimate):.3f}"
                  f" (truth {STEP_M}) angle {error:5.1f} deg")
        previous = image
    return 0


if __name__ == "__main__":
    sys.exit(main())
