#!/usr/bin/env python3
"""Guided takeoff, hover and landing for the Gazebo Iris.

This script is the ground station. It does not simulate the aircraft and it
does not run the flight controller. It speaks MAVLink to ArduCopter SITL,
which is already flying the Iris inside Gazebo.

Start order, each in its own terminal (or use scripts/run_sim.sh, which
starts the first two together and kills leftovers):

    bash scripts/run_sim.sh
    python3 scripts/takeoff_land.py

Outputs
    A CSV sampled at POSITION_RATE_HZ with one row per GLOBAL_POSITION_INT
    and a phase column (climb / hover / land), and a summary JSON next to
    it with the peak altitude, the phases that were actually flown and
    whether the flight controller disarmed. scripts/test_task2.py asserts
    on both files; nothing in this project trusts the terminal output.

    With --video the gimbal camera is also recorded to an mp4 for the same
    flight, so the video evidence and the position log describe one run
    rather than two. See VIDEO_RUN.md.

Change the flight by changing the constants below. Each comment says what
that number does to the aircraft.
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil

import mavlink_link


# ---------------------------------------------------------------------------
# Flight numbers. These are the values an interviewer will ask you to change.
# ---------------------------------------------------------------------------

# Metres above the point ArduCopter stored as home. This is not a GPS
# altitude and it is not a Gazebo z coordinate. 5 becomes a 5 m climb; 10
# becomes a 10 m climb. The climb stops when relative_alt reaches
# (this value - ALTITUDE_TOLERANCE_M).
TARGET_ALTITUDE_M = 5.0

# How close to TARGET_ALTITUDE_M counts as "arrived", in metres. 0.5 means
# a reading of 4.5 m is accepted for a 5 m target. Smaller waits longer and
# can time out on barometer noise; larger reports success while the
# aircraft is still climbing.
ALTITUDE_TOLERANCE_M = 0.5

# Seconds of flight-controller time to hold position after the climb. The
# wait uses the autopilot clock (time_boot_ms), so a slow simulation still
# hovers this long in simulated time. 0 skips the hover and starts the
# landing as soon as the altitude check passes.
HOVER_SECONDS = 8.0

# How long each phase may take, in wall-clock seconds. Gazebo runs in
# lockstep with SITL, so a busy CPU makes these expire before the simulated
# clock has caught up. Increase them on a slow machine. They do not change
# the path the aircraft flies.
READY_TIMEOUT_S = 90.0
CLIMB_TIMEOUT_S = 90.0
LAND_TIMEOUT_S = 120.0

# Position log rate in Hz. Raising it makes the CSV denser and costs CPU;
# lowering it makes the altitude check coarser. EKF_STATUS_REPORT and
# GPS_RAW_INT only need to be fast enough to notice readiness.
POSITION_RATE_HZ = 10.0
STATUS_RATE_HZ = 2.0

# TCP port sim_vehicle.py / build/sitl/bin/arducopter listen on for -I0.
CONNECTION = mavlink_link.CONNECTION


def log_row(writer, msg, phase):
    """Write one position sample.

    Altitudes are stored in metres so the CSV can be plotted without
    remembering the MAVLink millimetre convention. Velocities are MAVLink
    cm/s, stored in m/s. The raw message is not modified.
    """
    writer.writerow({
        "time_boot_ms": msg.time_boot_ms,
        "phase": phase,
        "lat_deg": msg.lat / 1e7,
        "lon_deg": msg.lon / 1e7,
        "relative_alt_m": msg.relative_alt / 1000.0,
        "amsl_alt_m": msg.alt / 1000.0,
        "vx_m_s": msg.vx / 100.0,
        "vy_m_s": msg.vy / 100.0,
        "vz_m_s": msg.vz / 100.0,
    })


def wait_for_altitude(master, writer, altitude_m, tolerance_m, timeout_s):
    """Climb until relative altitude reaches the target band.

    The climb loop itself lives in mavlink_link, because Task 3 records the
    same climb around a different CSV schema. This wrapper only supplies
    the "climb" phase label, so the two scripts cannot drift apart on what
    counts as having taken off.
    """
    return mavlink_link.wait_for_altitude(
        master, altitude_m, tolerance_m, timeout_s,
        position_callback=lambda msg: log_row(writer, msg, "climb"),
    )


def hover(master, writer, start_msg, hover_s):
    """Hold for hover_s seconds of autopilot time.

    GUIDED already holds the takeoff position; this function sends no new
    setpoint. It waits on the autopilot clock, so the hover length is
    independent of how fast the desktop renders. Changing HOVER_SECONDS
    changes only this wait; it does not touch the hold gains.
    """
    if hover_s <= 0:
        return
    start_ms = start_msg.time_boot_ms
    print(f"Hovering for {hover_s:.0f} s of autopilot time")
    last_print = 0.0
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is None:
            raise TimeoutError("Position stream stopped during hover")
        log_row(writer, msg, "hover")
        elapsed_s = (msg.time_boot_ms - start_ms) / 1000.0
        now = time.time()
        if now - last_print > 1.0:
            print(
                f"  hover  t={elapsed_s:.1f} s  "
                f"relative_alt={msg.relative_alt / 1000.0:.2f} m"
            )
            last_print = now
        if elapsed_s >= hover_s:
            return


def land(master, writer, timeout_s):
    """Enter LAND mode and wait until the motors disarm.

    The descent and the disarm wait are mavlink_link.land(); ArduCopter
    disarms itself on touchdown, so nothing here sends a disarm command.
    Sending disarm in the air would drop the Iris.
    """
    mavlink_link.land(
        master, timeout_s,
        position_callback=lambda msg: log_row(writer, msg, "land"),
    )


def summarise_csv(csv_path, disarmed):
    """Read the CSV back and write <csv>.summary.json.

    Reading the file instead of tracking counters in the flight loop means
    the summary cannot disagree with the evidence. The test asserts on this
    file and on the CSV; there is no third source of truth.
    """
    phases = []
    peak_alt = 0.0
    final_alt = 0.0
    samples = 0
    duration_s = 0.0
    first_ms = None
    last_ms = None
    with csv_path.open() as handle:
        for row in csv.DictReader(handle):
            samples += 1
            phase = row["phase"]
            if phase not in phases:
                phases.append(phase)
            alt = float(row["relative_alt_m"])
            peak_alt = max(peak_alt, alt)
            final_alt = alt
            boot_ms = int(row["time_boot_ms"])
            if first_ms is None:
                first_ms = boot_ms
            last_ms = boot_ms
    if first_ms is not None and last_ms is not None:
        duration_s = (last_ms - first_ms) / 1000.0
    return {
        "csv": str(csv_path),
        "samples": samples,
        "phases": phases,
        "peak_relative_alt_m": round(peak_alt, 3),
        "final_relative_alt_m": round(final_alt, 3),
        "autopilot_duration_s": round(duration_s, 2),
        "disarmed": bool(disarmed),
    }


def parse_args():
    parser = argparse.ArgumentParser(description="Guided takeoff and landing")
    # Command-line values override the constants. The constants stay the
    # documented defaults, so the file itself shows the flight that was run.
    parser.add_argument("--altitude", type=float, default=TARGET_ALTITUDE_M,
                        help="Takeoff height above home, metres")
    parser.add_argument("--hover", type=float, default=HOVER_SECONDS,
                        help="Hover time, seconds of autopilot time")
    parser.add_argument("--connect", default=CONNECTION,
                        help="pymavlink connection string")
    parser.add_argument("--log", default=None,
                        help="CSV path. Default is logs/takeoff_land_<time>.csv")
    parser.add_argument("--gimbal-pitch", type=float, default=None,
                        help="Optional: point the gimbal camera down by this "
                             "many radians through Gazebo transport (no ROS)")
    parser.add_argument("--video", default=None,
                        help="Optional: record the gimbal camera to this mp4 "
                             "for the duration of the flight")
    return parser.parse_args()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    log_dir = root / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    if args.log is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f"takeoff_land_{stamp}.csv"
    else:
        log_path = Path(args.log)
        log_path.parent.mkdir(parents=True, exist_ok=True)

    if args.gimbal_pitch is not None:
        # Imported lazily: the takeoff/landing test does not need Gazebo
        # transport, and the module is only present with gz-sim installed.
        import gimbal

        gimbal.set_pitch(args.gimbal_pitch)

    master = mavlink_link.connect(args.connect)
    mavlink_link.request_telemetry(master, {
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: POSITION_RATE_HZ,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: STATUS_RATE_HZ,
        mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: STATUS_RATE_HZ,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: STATUS_RATE_HZ,
    })
    mavlink_link.wait_until_ready(master, READY_TIMEOUT_S)

    # The video starts before the arm command, so it contains the takeoff
    # from the first metre rather than starting mid-climb. Imported lazily
    # for the same reason as gimbal above.
    video = None
    if args.video is not None:
        import camera

        video_path = Path(args.video)
        video_path.parent.mkdir(parents=True, exist_ok=True)
        video = camera.VideoRecorder(video_path)
        print(f"Recording the camera to {video_path}")

    started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    disarmed = False
    try:
        with log_path.open("w", newline="", buffering=1) as handle:
            # buffering=1 = line buffered: a crash still leaves every row that
            # had been received, which is what makes a failed run diagnosable.
            writer = csv.DictWriter(handle, fieldnames=[
                "time_boot_ms", "phase", "lat_deg", "lon_deg",
                "relative_alt_m", "amsl_alt_m", "vx_m_s", "vy_m_s", "vz_m_s",
            ])
            writer.writeheader()
            # GUIDED before arm so the takeoff command is legal the moment the
            # motors start. Arming in STABILIZE and then changing mode also
            # works, but the aircraft can yaw on the ground in between.
            mavlink_link.set_mode(master, "GUIDED")
            mavlink_link.arm(master)
            mavlink_link.takeoff(master, args.altitude)
            arrived = wait_for_altitude(
                master, writer, args.altitude, ALTITUDE_TOLERANCE_M,
                CLIMB_TIMEOUT_S,
            )
            hover(master, writer, arrived, args.hover)
            land(master, writer, LAND_TIMEOUT_S)
            disarmed = True
    finally:
        # Closed here rather than after the summary: a flight that raises
        # still leaves a playable mp4 of whatever happened.
        if video is not None:
            video.close()

    summary = summarise_csv(log_path, disarmed)
    summary.update({
        "started_utc": started_utc,
        "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "connection": args.connect,
        "target_altitude_m": args.altitude,
        "target_altitude_tolerance_m": ALTITUDE_TOLERANCE_M,
        "hover_seconds": args.hover,
        "ok": True,
    })
    if video is not None:
        # Recorded in the summary so the mp4 can be tied to this exact run:
        # the frame count is what distinguishes a real recording from an
        # empty file that happens to exist.
        summary.update({
            "video": str(args.video),
            "video_frames": video.frames,
        })
    summary_path = log_path.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")

    print(f"Position log written to {log_path}")
    print(f"Summary written to {summary_path}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
