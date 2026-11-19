#!/usr/bin/env python3
"""Compare the recorded GPS track with the VIO trajectory.

Reads one results/task3/dataset_<stamp>/ directory plus the trajectory that
scripts/vio.py wrote from it, and produces the numbers and plots that
REPORT.md quotes:

    results/task4/metrics.csv      every metric, its value, its unit
    results/task4/xy_overlay.png   both tracks, top-down
    results/task4/altitude.png     altitude against time
    results/task4/error.png        horizontal error against distance flown

Inputs
    gps_raw.csv          GPS_RAW_INT: latitude, longitude, altitude
    local_position.csv   the autopilot's own local NED estimate
    frames.csv           used only to bridge the two clocks
    vio_trajectory.csv   the VIO, in NED metres, on the autopilot clock

Method
    The reference track is built from the raw GPS fixes: every latitude and
    longitude is projected onto a local tangent plane through the first fix,
    which is the same flat-earth approximation the autopilot uses over a
    flight this short (about 13 km/h at the equator is 1e-4 of a degree, and
    the projection error over 30 m is millimetres). The autopilot's own
    local_position is then used as an independent check on that projection,
    not as the reference itself, so the comparison is "VIO against the
    consumer-grade position source" and not "VIO against itself".

    Both tracks are moved to the origin at the first common instant. That is
    the raw alignment and it is the headline number: nothing is rotated, no
    scale is fitted, so the ATE says what the VIO actually produced. Two
    optional alignments are reported next to it, clearly not as headlines:
    a heading-only rotation about the vertical, and a full Umeyama
    similarity fit (rotation + translation + scale), which is the best any
    rigid alignment could do and therefore an upper bound on the VIO's
    usefulness rather than a measure of its accuracy.

Nothing here is written by hand; every number in the CSV comes from the
files in the dataset directory.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# WGS84 equatorial radius. The projection below is a tangent plane, so the
# error grows with the square of the distance from the origin; over 50 m it
# is micrometres, and it is the radius of the Earth in the equatorial sense
# because latitude, not the geoid, is what is being differentiated.
EARTH_RADIUS_M = 6378137.0

# Minimum samples before a metric means anything. A handful of points would
# produce a number that looks precise and is not.
MIN_SAMPLES = 10

# Coarse-to-fine search for the heading offset. One degree then a tenth of a
# degree is enough: the VIO's world frame is initialised from the reference
# attitude, so the true offset is expected to be small and smooth, and a
# finer grid would be fitting noise.
HEADING_COARSE_DEG = 1.0
HEADING_FINE_DEG = 0.1

# Plot sizes and colours are fixed here so the three figures look like one
# set. 150 dpi keeps the PNGs readable without bloating the repository.
FIGURE_DPI = 150
COLOUR_REFERENCE = "#1f4e79"
COLOUR_RAW = "#c00000"
COLOUR_ALIGNED = "#2e8b57"


# ---------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------

def read_csv(path):
    """Read a CSV into a list of dicts of strings.

    Column-by-column conversion happens where the column is used, so a bad
    value names its column in the error rather than failing silently in a
    bulk float() call.
    """
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def column(rows, name):
    """One column as a float array."""
    return np.array([float(row[name]) for row in rows])


# ---------------------------------------------------------------------------
# Reference track
# ---------------------------------------------------------------------------

def gps_to_local(gps_rows):
    """Project raw latitude/longitude onto a tangent plane at the first fix.

    Returns (wall_times, north_m, east_m, up_m, origin). The origin is kept
    so the report can state where the flight was, in the same units the
    dataset records.
    """
    latitude = column(gps_rows, "lat_deg")
    longitude = column(gps_rows, "lon_deg")
    altitude = column(gps_rows, "alt_m")
    lat0 = latitude[0]
    lon0 = longitude[0]
    north = np.radians(latitude - lat0) * EARTH_RADIUS_M
    # A degree of longitude is shorter than a degree of latitude by
    # cos(latitude); at this latitude the difference is about 18%.
    east = np.radians(longitude - lon0) * EARTH_RADIUS_M * math.cos(
        math.radians(lat0))
    up = altitude - altitude[0]
    return (column(gps_rows, "wall_time_s"), north, east, up,
            {"latitude_deg": lat0, "longitude_deg": lon0})


def boot_to_wall(frames):
    """Map autopilot clock to recording wall clock, by interpolation.

    The VIO is timed on the autopilot clock and the GPS rows are timed on
    the wall clock, so the two have to be bridged. frames.csv carries both
    for every image, which makes the mapping a measurement rather than an
    assumption.
    """
    return column(frames, "boot_time_s"), column(frames, "wall_time_s")


def reference_at(vio_rows, boot, wall, gps_wall, gps_north, gps_east, gps_up):
    """Reference track sampled at the VIO's timestamps.

    Each VIO row is placed on the wall clock, then each GPS channel is
    interpolated there. Interpolating at the VIO's instants rather than
    resampling the VIO keeps the reference error out of the residual; the
    GPS updates at 5 Hz against the camera's 10 Hz, and a linear
    interpolation over 0.2 s at 4 m/s is a straight line across 0.8 m of
    travel.
    """
    wall_time = np.interp(column(vio_rows, "boot_time_s"), boot, wall)
    north = np.interp(wall_time, gps_wall, gps_north)
    east = np.interp(wall_time, gps_wall, gps_east)
    up = np.interp(wall_time, gps_wall, gps_up)
    return wall_time, north, east, up


# ---------------------------------------------------------------------------
# Alignment
# ---------------------------------------------------------------------------

def rotate_xy(north, east, angle_rad):
    """Rotate a horizontal track about the vertical axis."""
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return cosine * north - sine * east, sine * north + cosine * east


def heading_offset(vio_north, vio_east, ref_north, ref_east):
    """Heading that best aligns the VIO to the reference, in degrees.

    Searched on a grid rather than solved, because the quantity that
    matters - the horizontal error - is not differentiable at the origin
    and a closed form would need a weighting nobody asked for. The grid
    starts at one degree and refines once by ten, so the answer is within
    0.1 degree of the best on that axis.
    """
    best_angle, best_error = 0.0, float("inf")
    angle = -180.0
    while angle <= 180.0:
        rotated_north, rotated_east = rotate_xy(
            vio_north, vio_east, math.radians(angle))
        error = float(np.sqrt(np.mean(
            (rotated_north - ref_north) ** 2
            + (rotated_east - ref_east) ** 2)))
        if error < best_error:
            best_angle, best_error = angle, error
        angle += HEADING_COARSE_DEG
    angle = best_angle - HEADING_COARSE_DEG
    while angle <= best_angle + HEADING_COARSE_DEG:
        rotated_north, rotated_east = rotate_xy(
            vio_north, vio_east, math.radians(angle))
        error = float(np.sqrt(np.mean(
            (rotated_north - ref_north) ** 2
            + (rotated_east - ref_east) ** 2)))
        if error < best_error:
            best_angle, best_error = angle, error
        angle += HEADING_FINE_DEG
    return best_angle


def umeyama(source, target):
    """Least-squares similarity transform: target ~= scale * R * source + t.

    The closed-form solution (Umeyama 1991): centre both clouds, take the
    SVD of the cross-covariance, and the rotation is U D V^T with D fixed up
    so the result is a proper rotation, not a reflection. The scale is the
    ratio of the singular values, which is exactly the freedom this
    comparison must not grant the VIO silently - it is reported, not used.

    Written for any number of dimensions, because this script fits it in
    the horizontal plane only. A full 3D fit here would trade horizontal
    accuracy for vertical: the VIO's vertical drift is larger than its
    horizontal error, so the optimiser would shrink everything to chase the
    altitude error and make the horizontal match worse. The horizontal fit
    answers the question the comparison is actually asking - is the XY
    error a consistent scale or rotation that could be calibrated out, or
    is it genuine drift?
    """
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centred = source - source_mean
    target_centred = target - target_mean
    covariance = target_centred.T @ source_centred / len(source)
    u, singular, vt = np.linalg.svd(covariance)
    correction = np.eye(source.shape[1])
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vt
    variance = float(np.mean(np.sum(source_centred ** 2, axis=1)))
    scale = float(np.trace(np.diag(singular) @ correction)) / variance
    translation = target_mean - scale * rotation @ source_mean
    return scale, rotation, translation


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def horizontal_errors(north, east, ref_north, ref_east):
    """Per-sample horizontal error."""
    return np.sqrt((north - ref_north) ** 2 + (east - ref_east) ** 2)


def ate_rmse(horizontal, vertical):
    """Horizontal and 3D root-mean-square error."""
    return (float(np.sqrt(np.mean(horizontal ** 2))),
            float(np.sqrt(np.mean(horizontal ** 2 + vertical ** 2))))


def collect_metrics(alignment_name, north, east, down, ref_north, ref_east,
                    ref_up, path_length):
    """One row per metric for one alignment."""
    horizontal = horizontal_errors(north, east, ref_north, ref_east)
    vertical = down + ref_up          # down is +ve down, ref_up is +ve up
    horizontal_rmse, three_d_rmse = ate_rmse(horizontal, vertical)
    final = float(horizontal[-1])
    rows = [
        (f"{alignment_name}_ate_rmse_horizontal_m", horizontal_rmse,
         "root mean square of the per-sample horizontal error"),
        (f"{alignment_name}_ate_rmse_3d_m", three_d_rmse,
         "root mean square of the 3-axis error"),
        (f"{alignment_name}_error_max_horizontal_m", float(horizontal.max()),
         "worst single sample"),
        (f"{alignment_name}_error_final_horizontal_m", final,
         "horizontal error at touchdown"),
        (f"{alignment_name}_rmse_north_m",
         float(np.sqrt(np.mean((north - ref_north) ** 2))), "per-axis"),
        (f"{alignment_name}_rmse_east_m",
         float(np.sqrt(np.mean((east - ref_east) ** 2))), "per-axis"),
        (f"{alignment_name}_rmse_down_m",
         float(np.sqrt(np.mean(vertical ** 2))), "per-axis, down positive"),
        (f"{alignment_name}_drift_per_metre",
         final / path_length if path_length > 0 else float("nan"),
         "final horizontal error divided by VIO path length"),
    ]
    return rows, horizontal


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def make_plots(output_dir, wall_time, ref_north, ref_east, ref_up,
               raw_north, raw_east, raw_down, aligned_north, aligned_east,
               raw_error, aligned_error, path_distance):
    """Write the three figures REPORT.md refers to."""
    import matplotlib
    # No display is available on the machine that runs the automated side of
    # this task, so the backend is chosen up front rather than discovered
    # when the first figure fails to appear.
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # 1. Top-down overlay. Equal aspect: a square that measures square on
    # the ground has to measure square on the page, or the picture quietly
    # misreports the flight.
    figure, axes = plt.subplots(figsize=(6.4, 6.4))
    axes.plot(ref_east, ref_north, color=COLOUR_REFERENCE, linewidth=1.8,
              label="GPS")
    axes.plot(raw_east, raw_north, color=COLOUR_RAW, linewidth=1.2,
              label="VIO (raw alignment)")
    axes.plot(aligned_east, aligned_north, color=COLOUR_ALIGNED,
              linewidth=1.2, linestyle="--", label="VIO (heading aligned)")
    axes.plot([raw_east[0]], [raw_north[0]], marker="o", color="black",
              markersize=5, label="start")
    axes.set_xlabel("east (m)")
    axes.set_ylabel("north (m)")
    axes.set_title("Horizontal track: GPS against VIO")
    axes.set_aspect("equal", adjustable="datalim")
    axes.grid(alpha=0.3)
    axes.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(output_dir / "xy_overlay.png", dpi=FIGURE_DPI)
    plt.close(figure)

    # 2. Altitude. The reference is the GPS altitude above the first fix;
    # the VIO's down axis is negated so that both curves rise on takeoff.
    figure, axes = plt.subplots(figsize=(7.2, 3.6))
    axes.plot(wall_time, ref_up, color=COLOUR_REFERENCE, linewidth=1.8,
              label="GPS altitude above first fix")
    axes.plot(wall_time, -raw_down, color=COLOUR_RAW, linewidth=1.2,
              label="VIO height (raw alignment)")
    axes.set_xlabel("time on the recording clock (s)")
    axes.set_ylabel("height (m)")
    axes.set_title("Altitude: GPS against VIO")
    axes.grid(alpha=0.3)
    axes.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(output_dir / "altitude.png", dpi=FIGURE_DPI)
    plt.close(figure)

    # 3. Error against distance flown, which is what drift is about: an
    # error of a metre after ten metres is a different aircraft from one
    # metre after a kilometre.
    figure, axes = plt.subplots(figsize=(7.2, 3.6))
    axes.plot(path_distance, raw_error, color=COLOUR_RAW, linewidth=1.4,
              label="raw alignment")
    axes.plot(path_distance, aligned_error, color=COLOUR_ALIGNED,
              linewidth=1.2, linestyle="--", label="heading aligned")
    axes.set_xlabel("VIO path length (m)")
    axes.set_ylabel("horizontal error (m)")
    axes.set_title("Horizontal error against distance flown")
    axes.grid(alpha=0.3)
    axes.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(output_dir / "error.png", dpi=FIGURE_DPI)
    plt.close(figure)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def newest_dataset(task3_dir):
    """Most recent dataset directory, matching vio.py's own rule."""
    candidates = sorted(
        (p for p in task3_dir.glob("dataset_*") if p.is_dir()),
        key=lambda p: p.name,
    )
    if not candidates:
        raise SystemExit(f"No dataset_* directory in {task3_dir}.")
    return candidates[-1]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare a VIO trajectory with the recorded GPS track"
    )
    parser.add_argument("--dataset", default=None,
                        help="Dataset directory. Default: newest under "
                             "results/task3/")
    parser.add_argument("--trajectory", default=None,
                        help="VIO CSV. Default: results/task3/"
                             "vio_trajectory.csv")
    parser.add_argument("--output", default=None,
                        help="Output directory. Default: results/task4")
    return parser.parse_args()


def main():
    args = parse_args()
    dataset_dir = (Path(args.dataset) if args.dataset
                   else newest_dataset(ROOT / "results" / "task3"))
    trajectory_path = (Path(args.trajectory) if args.trajectory
                       else ROOT / "results" / "task3" / "vio_trajectory.csv")
    output_dir = (Path(args.output) if args.output
                  else ROOT / "results" / "task4")
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Dataset: {dataset_dir}")
    print(f"Trajectory: {trajectory_path}")

    # --- reference track -------------------------------------------------
    gps_rows = read_csv(dataset_dir / "gps_raw.csv")
    if len(gps_rows) < MIN_SAMPLES:
        raise SystemExit(f"Only {len(gps_rows)} GPS fixes; nothing to "
                         "compare against.")
    gps_wall, gps_north, gps_east, gps_up, origin = gps_to_local(gps_rows)
    print(f"GPS origin: {origin['latitude_deg']:.7f}, "
          f"{origin['longitude_deg']:.7f}, {len(gps_rows)} fixes")

    # Independent check on the projection: the autopilot's own local NED
    # estimate covers the same ground, so a projection bug would show up
    # here as a difference that grows with distance from the origin.
    local_rows = read_csv(dataset_dir / "local_position.csv")
    local_wall = (np.interp(column(local_rows, "time_boot_ms") / 1000.0,
                            column(read_csv(dataset_dir / "frames.csv"),
                                   "boot_time_s"),
                            column(read_csv(dataset_dir / "frames.csv"),
                                   "wall_time_s")))
    check_north = np.interp(local_wall, gps_wall, gps_north)
    check_east = np.interp(local_wall, gps_wall, gps_east)
    check_error = np.sqrt((check_north - column(local_rows, "north_m")) ** 2
                          + (check_east - column(local_rows, "east_m")) ** 2)
    print(f"GPS-derived track against the autopilot's own local estimate: "
          f"median {np.median(check_error):.2f} m, "
          f"max {check_error.max():.2f} m")

    # --- the VIO track ---------------------------------------------------
    vio_rows = read_csv(trajectory_path)
    if len(vio_rows) < MIN_SAMPLES:
        raise SystemExit(f"Only {len(vio_rows)} VIO samples.")
    frames = read_csv(dataset_dir / "frames.csv")
    boot, wall = boot_to_wall(frames)
    wall_time, ref_north, ref_east, ref_up = reference_at(
        vio_rows, boot, wall, gps_wall, gps_north, gps_east, gps_up)

    vio_north = column(vio_rows, "north_m")
    vio_east = column(vio_rows, "east_m")
    vio_down = column(vio_rows, "down_m")
    steps = column(vio_rows, "step_m")
    path_length = float(steps.sum())
    path_distance = np.cumsum(steps)

    # Move both tracks to the origin at the first common instant. This is
    # the whole of the raw alignment: no rotation, no scale.
    ref_north = ref_north - ref_north[0]
    ref_east = ref_east - ref_east[0]
    ref_up = ref_up - ref_up[0]
    vio_north = vio_north - vio_north[0]
    vio_east = vio_east - vio_east[0]
    vio_down = vio_down - vio_down[0]

    # --- raw alignment (headline) ---------------------------------------
    raw_rows, raw_error = collect_metrics(
        "raw", vio_north, vio_east, vio_down,
        ref_north, ref_east, ref_up, path_length)

    # --- heading-only alignment ------------------------------------------
    offset_deg = heading_offset(vio_north, vio_east, ref_north, ref_east)
    aligned_north, aligned_east = rotate_xy(
        vio_north, vio_east, math.radians(offset_deg))
    aligned_rows, aligned_error = collect_metrics(
        "heading_aligned", aligned_north, aligned_east, vio_down,
        ref_north, ref_east, ref_up, path_length)

    # --- Umeyama similarity fit (labelled, not a headline) ---------------
    # Horizontal plane only; see the note in umeyama() for why a 3D fit
    # would report a better-looking vertical at the cost of the XY match.
    target = np.column_stack([ref_north, ref_east])
    source = np.column_stack([vio_north, vio_east])
    scale, rotation, translation = umeyama(source, target)
    fitted = (scale * (rotation @ source.T)).T + translation
    fitted_rows, _ = collect_metrics(
        "umeyama", fitted[:, 0], fitted[:, 1], vio_down,
        ref_north, ref_east, ref_up, path_length)

    # --- write ------------------------------------------------------------
    metrics_path = output_dir / "metrics.csv"
    with metrics_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value", "unit", "note"])
        for name, value, note in raw_rows + aligned_rows + fitted_rows:
            writer.writerow([name, f"{value:.4f}", "m", note])
        writer.writerow(["vio_path_length", f"{path_length:.3f}", "m",
                         "sum of the VIO's per-frame steps"])
        writer.writerow(["heading_offset", f"{offset_deg:.2f}", "deg",
                         "rotation that best aligns the VIO to the GPS track"])
        writer.writerow(["umeyama_scale", f"{scale:.4f}", "-",
                         "fitted similarity scale, reported not applied "
                         "anywhere else"])
        writer.writerow(["gps_samples", len(gps_rows), "-",
                         f"fix_type {gps_rows[0]['fix_type']}"])
        writer.writerow(["vio_samples", len(vio_rows), "-",
                         "one per camera frame"])

    make_plots(output_dir, wall_time, ref_north, ref_east, ref_up,
               vio_north, vio_east, vio_down, aligned_north, aligned_east,
               raw_error, aligned_error, path_distance)

    # --- summary on stdout ------------------------------------------------
    summary = {
        "dataset": str(dataset_dir),
        "trajectory": str(trajectory_path),
        "samples": int(len(vio_rows)),
        "gps_samples": len(gps_rows),
        "gps_origin": origin,
        "gps_projection_check_median_m": round(float(np.median(check_error)), 3),
        "gps_projection_check_max_m": round(float(check_error.max()), 3),
        "vio_path_length_m": round(path_length, 3),
        "heading_offset_deg": round(offset_deg, 3),
        "umeyama_scale": round(scale, 4),
        "metrics": {
            name: round(value, 4) for name, value, _ in
            raw_rows + aligned_rows + fitted_rows
        },
        "outputs": {
            "metrics": str(metrics_path),
            "plots": [str(output_dir / name) for name in
                      ("xy_overlay.png", "altitude.png", "error.png")],
        },
    }
    summary_path = output_dir / "compare.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")

    print(f"Path length: {path_length:.2f} m over {len(vio_rows)} frames")
    print(f"Raw alignment      : ATE RMSE "
          f"{summary['metrics']['raw_ate_rmse_horizontal_m']:.3f} m "
          f"(horizontal), max "
          f"{summary['metrics']['raw_error_max_horizontal_m']:.3f} m")
    print(f"Heading aligned    : ATE RMSE "
          f"{summary['metrics']['heading_aligned_ate_rmse_horizontal_m']:.3f} m"
          f" at {offset_deg:.2f} deg")
    print(f"Umeyama fit        : ATE RMSE "
          f"{summary['metrics']['umeyama_ate_rmse_horizontal_m']:.3f} m "
          f"(scale {scale:.3f}, labelled as a fitted upper bound)")
    print(f"Metrics written to {metrics_path}")
    print(f"Summary written to {summary_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
