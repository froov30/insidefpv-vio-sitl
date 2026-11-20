#!/usr/bin/env python3
"""Records a timestamped camera + IMU + GPS dataset for Task 3.

Task 3 needs three things recorded at the same time, from three different
sources, with no ROS anywhere in the path:

    camera frames     Gazebo transport, topic .../sensor/camera/image
    camera intrinsics Gazebo transport, topic .../sensor/camera/camera_info
    IMU + GPS         MAVLink from ArduCopter SITL, on a link of their own

The MAVLink side uses two TCP connections: SERIAL0 (5760) for the flight
commands and SERIAL1 (5762) for logging. One link cannot do both, because
the command helpers block in recv_match(type=...) and that call throws away
every message of another type. See MavlinkLogger.

The flight is a square, flown in GUIDED: climb to 5 m, fly four 8 m legs
(north, east, south, west, returning to the start), hold still at each
corner, then land. A square is used because every leg is a known-length
straight line, so the trajectory the VIO produces can be checked against a
ground truth that does not come from the VIO itself.

Output layout
    results/task3/dataset_<stamp>/
        images/000001.png ...        lossless frames, named by frame index
        frames.csv                   one row per frame: sim time, wall time,
                                     autopilot time at capture
        imu.csv                      RAW_IMU and SCALED_IMU2
        attitude.csv                 ATTITUDE
        local_position.csv           LOCAL_POSITION_NED
        gps_raw.csv                  GPS_RAW_INT
        dataset.json                 intrinsics, rates, clock offset, counts

Time bases, and why there are three
    sim_time_s      Gazebo's clock, carried in each image header
    boot_time_s     ArduCopter's time_boot_ms; all flight data uses this
    wall_time_s     the recording host's clock
    Neither clock is the other, so frames.csv also carries the autopilot
    time that was current when the frame arrived. dataset.json reports the
    median and spread of (boot - sim) so the analysis can state how well
    the two clocks were tied together instead of assuming they were.

Usage (the simulator must already be running)
    bash scripts/run_sim.sh --headless      # in another terminal
    python3 scripts/record_dataset.py
"""

import argparse
import csv
import json
import queue
import statistics
import sys
import threading
import time
from pathlib import Path

import cv2

import gz.transport13 as gzt
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.image_pb2 import Image
from pymavlink import mavutil

import camera
import mavlink_link

ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# Camera topics
# ---------------------------------------------------------------------------

# The topic names and the RGB_INT8 -> BGR decoding live in scripts/camera.py,
# because mavlink_link.py --video uses the same ones. A typo in a topic name
# does not raise: subscribe() succeeds and simply never calls back.
IMAGE_TOPIC = camera.IMAGE_TOPIC
INFO_TOPIC = camera.INFO_TOPIC

# How long to wait for the first frame and for camera_info. Gazebo publishes
# only while a rendering loop is alive; if rendering is disabled entirely
# there is no image at all, and the timeout is what reports that.
FIRST_MESSAGE_TIMEOUT_S = 30.0

# Queue between the transport callback and the disk writer. 40 frames is
# 4 s of video at 10 Hz: enough to absorb a slow disk write, small enough
# that a wedged writer cannot grow memory without bound.
FRAME_QUEUE_SIZE = 40

# PNG compression level 3. Frames are synthetic and flat-shaded, so level 3
# already gives a big file-size win; level 9 costs ~4x the CPU for very
# little more, and at 10 Hz the writer has to keep up with the camera.
PNG_COMPRESSION = 3


# ---------------------------------------------------------------------------
# Flight
# ---------------------------------------------------------------------------

# Height the square is flown at, metres above home. Same number as Task 2 on
# purpose: the two tasks then describe the same flight and can be compared.
TARGET_ALTITUDE_M = 5.0

# How close to TARGET_ALTITUDE_M counts as arrived, in metres, for the one
# wait_for_altitude() call before the legs start. Same tolerance as Task 2 so
# the two flights can be compared; a smaller value waits longer on barometer
# noise, a larger one starts the square while still climbing.
ALTITUDE_TOLERANCE_M = 0.5

# Side length of the square, metres. 8 m at 5 m altitude keeps the whole
# ground pattern in view while still producing ~40 degrees of parallax
# across a corner, which is what a monocular VIO needs to estimate motion.
LEG_M = 8.0

# Time spent motionless at each corner. The VIO trajectory should show the
# same near-zero displacement over these windows; if it drifts while the
# aircraft is not moving, that is visible immediately in the plots.
CORNER_HOVER_S = 2.0

# Per-leg time budget in wall-clock seconds. 8 m at ArduCopter's default
# GUIDED speed of 5 m/s should take ~2 s; 60 s absorbs lockstep slowdown
# without hiding a real failure.
LEG_TIMEOUT_S = 60.0

# Extra recording time after the motors disarm, so the dataset ends with the
# aircraft on the ground rather than at the moment of touchdown.
POST_LAND_SECONDS = 2.0

READY_TIMEOUT_S = 90.0
CLIMB_TIMEOUT_S = 90.0
LAND_TIMEOUT_S = 120.0

# Camera pitch used for the whole dataset. See scripts/camera.py for why the
# camera is pointed down: at level, most of the frame is sky.
GIMBAL_PITCH_RAD = camera.PITCH_DOWN_RAD

# MAVLink stream rates, for the logging link. The camera runs at 10 Hz;
# 50 Hz IMU gives five samples per frame and 20 Hz attitude gives two, which
# is what lets vio.py integrate rotation between frames instead of
# interpolating a single sample. 10 Hz position is the rate the flight
# controller estimates at.
#
# These are rates in autopilot time. The simulation can run well below real
# time (real_time_factor in dataset.json records the factor achieved), so 40 s
# of wall-clock recording can cover far less simulated flight.
STREAM_RATES = {
    mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU: 50.0,
    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2: 50.0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 20.0,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10.0,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10.0,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 5.0,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2.0,
}

# The flight-command link needs far less: a heartbeat, a position to aim the
# takeoff at, the local position goto_local_ned() measures arrival against,
# and the GPS/EKF state that wait_until_ready() polls. Asking for the 50 Hz
# streams here as well would only add traffic.
#
# LOCAL_POSITION_NED is requested explicitly and that is not optional. This
# SITL does not stream it on SERIAL0 unless asked (SR0_POSITION is 0), and a
# missing stream is indistinguishable from "not there yet" inside
# goto_local_ned(): the setpoint keeps being re-sent while the arrival test
# waits for a message that will never arrive. The aircraft reaches the first
# corner, holds there for LEG_TIMEOUT_S, and the square silently becomes a
# hover. Requesting it here is what makes the square conditional on nothing
# but the code.
COMMAND_RATES = {
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10.0,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10.0,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2.0,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2.0,
}

# SITL exposes three MAVLink endpoints on this configuration: SERIAL0 on
# TCP 5760 (the flight commands) and SERIAL1/SERIAL2 on TCP 5762/5763.
# The logger gets its own; see MavlinkLogger for why one link is not enough.
LOGGER_CONNECTION = "tcp:127.0.0.1:5762"


# ---------------------------------------------------------------------------
# CSV writing
# ---------------------------------------------------------------------------

class CsvWriter:
    """Append-only CSV with a fixed header, flushed on every row.

    Flushing every row is slower than buffering, but a killed run still
    leaves a complete prefix of the data. For a dataset that is the point:
    a partial flight should still be analysable and should not pretend to
    be complete (dataset.json records whether the flight finished).
    """

    def __init__(self, path, fieldnames):
        self.path = path
        self.handle = path.open("w", newline="")
        self.writer = csv.DictWriter(self.handle, fieldnames=fieldnames)
        self.writer.writeheader()
        self.handle.flush()

    def write(self, row):
        self.writer.writerow(row)
        self.handle.flush()

    def close(self):
        self.handle.close()


class SharedClock:
    """The newest autopilot time seen by the MAVLink thread.

    The camera callback runs on a Gazebo transport thread and cannot read
    from the MAVLink socket. It reads this instead, which is why frames.csv
    can say "the autopilot clock read X when this frame arrived". The value
    is a plain float under a lock; the lock is only held for the assignment.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._boot_s = None

    def update(self, boot_ms):
        with self._lock:
            self._boot_s = boot_ms / 1000.0

    @property
    def boot_s(self):
        with self._lock:
            return self._boot_s


class ImageRecorder:
    """Subscribes to the camera and writes frames to disk on a second thread.

    Two threads on purpose: the transport callback must return quickly or
    Gazebo will queue (and eventually drop) images, while PNG encoding and
    the disk write take milliseconds. The queue decouples them and the drop
    counter makes any overflow visible in dataset.json instead of silently
    shortening the dataset.
    """

    def __init__(self, image_topic, images_dir, clock, wall_origin):
        self.image_topic = image_topic
        self.images_dir = images_dir
        self.clock = clock
        self.wall_origin = wall_origin
        self.rows = []
        self.dropped = 0
        self.frame_index = 0
        self.queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()
        self.node = gzt.Node()
        self.first_frame = threading.Event()
        self.subscribed = self.node.subscribe(Image, image_topic, self._on_image)

    def _on_image(self, message):
        """Transport callback: copy the frame and hand it to the writer.

        The protobuf message is reused by the transport library after the
        callback returns, so the pixel buffer has to be copied here. The
        copy is 921600 bytes at 640x480x3 and takes well under a millisecond.
        """
        self.first_frame.set()
        # RGB_INT8 straight from the sensor to a BGR copy; see camera.py for
        # why the payload length is checked before reshaping.
        frame = camera.image_to_bgr(message)
        item = {
            "frame": frame,
            "sim_time_s": message.header.stamp.sec
            + message.header.stamp.nsec * 1e-9,
            "wall_time_s": time.time() - self.wall_origin,
            "boot_time_s": self.clock.boot_s,
        }
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            # Counted, not retried: blocking here would stall the transport
            # thread and lose more frames than this one.
            self.dropped += 1

    def _write_loop(self):
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                self.frame_index += 1
                name = f"{self.frame_index:06d}.png"
                # image_to_bgr already delivered BGR, which is what cv2
                # writes; the VIO converts to grayscale anyway.
                cv2.imwrite(
                    str(self.images_dir / name),
                    item["frame"],
                    [cv2.IMWRITE_PNG_COMPRESSION, PNG_COMPRESSION],
                )
                boot = item["boot_time_s"]
                self.rows.append({
                    "frame": self.frame_index,
                    "file": f"images/{name}",
                    "sim_time_s": f"{item['sim_time_s']:.6f}",
                    "wall_time_s": f"{item['wall_time_s']:.6f}",
                    "boot_time_s": "" if boot is None else f"{boot:.6f}",
                })
            finally:
                self.queue.task_done()

    def stop(self):
        """Stop accepting frames, drain the queue, and return the rows."""
        try:
            self.node.unsubscribe(self.image_topic)
        except Exception:  # noqa: BLE001 - teardown must not mask a real error
            pass
        self.queue.put(None)
        self.queue.join()
        self._thread.join(timeout=10)
        return self.rows


def read_intrinsics(timeout_s=FIRST_MESSAGE_TIMEOUT_S):
    """Return the live camera_info message as a dict.

    The intrinsics are read from Gazebo, never hardcoded: the VIO uses
    fx, fy, cx, cy to turn pixel motion into bearings, so a wrong focal
    length silently scales every distance the pipeline reports. The
    payload is saved verbatim in dataset.json as well, so a reader can
    check the numbers against the sensor definition.

    gz.msgs.CameraInfo mirrors the ROS message of the same name, so the
    values are packed matrices rather than named fields. Layout as read
    from the live topic on this build:

        intrinsics.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]   row-major 3x3
        distortion.k = [k1, k2, p1, p2, k3]              model 0 = none
        projection.p = 3x4 matrix for a rectified camera

    There is no intrinsics.fx field; the focal lengths live in k. The sensor
    advertises no distortion model, which is why every coefficient is zero
    and the VIO can treat the camera as a pinhole with no undistortion step.
    """
    holder = {}
    event = threading.Event()

    def callback(message):
        if event.is_set():
            return
        holder["message"] = message
        event.set()

    node = gzt.Node()
    node.subscribe(CameraInfo, INFO_TOPIC, callback)
    if not event.wait(timeout_s):
        raise TimeoutError(
            f"No camera_info on {INFO_TOPIC} within {timeout_s:.0f} s. "
            "Is Gazebo running with rendering enabled? "
            "scripts/run_gazebo.sh --headless keeps rendering on."
        )
    message = holder["message"]
    k = list(message.intrinsics.k)
    if len(k) != 9:
        raise RuntimeError(
            f"camera_info intrinsics.k has {len(k)} values, expected 9 for "
            "a 3x3 matrix. The message layout is not the one this function "
            "was written against."
        )
    d = list(message.distortion.k)
    return {
        "width": message.width,
        "height": message.height,
        "fx": k[0],
        "fy": k[4],
        "cx": k[2],
        "cy": k[5],
        "skew": k[1],
        "intrinsics_matrix": k,
        "projection_matrix": list(message.projection.p),
        "distortion_model": message.distortion.model,
        "distortion": {
            name: (d[index] if index < len(d) else 0.0)
            for index, name in enumerate(("k1", "k2", "p1", "p2", "k3"))
        },
    }


# ---------------------------------------------------------------------------
# MAVLink logging
# ---------------------------------------------------------------------------

def make_loggers(log_dir):
    """Create one CSV per MAVLink message group.

    Separate files, not one wide table: the streams arrive at different
    rates (50 Hz IMU, 10 Hz position), and merging them by nearest
    timestamp would either duplicate IMU rows or interpolate them. The
    analysis resamples explicitly, where the choice is visible.
    """
    loggers = {
        "imu": CsvWriter(log_dir / "imu.csv", [
            "wall_time_s", "time_usec", "boot_time_s", "source",
            "acc_x_m_s2", "acc_y_m_s2", "acc_z_m_s2",
            "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
        ]),
        "attitude": CsvWriter(log_dir / "attitude.csv", [
            "wall_time_s", "time_boot_ms", "roll_rad", "pitch_rad",
            "yaw_rad", "roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s",
        ]),
        "local_position": CsvWriter(log_dir / "local_position.csv", [
            "wall_time_s", "time_boot_ms", "north_m", "east_m", "down_m",
            "vn_m_s", "ve_m_s", "vd_m_s",
        ]),
        "gps_raw": CsvWriter(log_dir / "gps_raw.csv", [
            "wall_time_s", "time_usec", "fix_type", "lat_deg", "lon_deg",
            "alt_m", "eph_m", "epv_m", "vel_m_s", "satellites",
        ]),
    }
    return loggers


# Units used by every conversion in log_message(), in one place.
#
# MAVLink is not in SI. Getting a factor of 1000 wrong here would make the VIO
# report a wrong scale and the comparison meaningless, so each conversion is
# stated next to the field that uses it:
#     accelerometer   mG     (1 g = 1000)  -> m/s^2  * 9.80665 / 1000
#     gyroscope       mrad/s               -> rad/s   / 1000
#     altitude        mm                   -> m       / 1000
#     velocity        cm/s                 -> m/s     / 100
#     latitude/long   deg * 1e7            -> deg     / 1e7


# MAVLink RAW_IMU reports acceleration in milli-g, not m/s^2. This is the
# standard gravity used to convert, matching the ArduPilot definition so the
# CSV numbers agree with what the flight controller itself computed.
GRAVITY_M_S2 = 9.80665


def log_message(message, loggers, wall_origin, clock):
    """Write one MAVLink message to the matching CSV. Returns True if written.

    The EKF_STATUS_REPORT is deliberately not recorded: it is only used to
    decide when the aircraft is ready to arm, and the analysis has no use
    for it.
    """
    kind = message.get_type()
    wall_s = f"{time.time() - wall_origin:.6f}"
    if kind in ("RAW_IMU", "SCALED_IMU2"):
        # Same three axes, different timestamp field: RAW_IMU carries
        # time_usec (microseconds since boot), SCALED_IMU2 carries
        # time_boot_ms (milliseconds). Reading time_usec from a SCALED_IMU2
        # raises AttributeError, so the two fields are not interchangeable.
        #
        # SITL can send a zero timestamp. When it does, the time base of that
        # row is genuinely unknown, so the column is left empty instead of
        # being filled with a guess; load_dataset() in vio.py then drops the
        # row, which shortens the gyro window by one sample. The committed
        # dataset has no such rows (2180 RAW_IMU samples, all timestamped).
        if kind == "RAW_IMU":
            raw_time_usec = message.time_usec
            boot_s = f"{raw_time_usec / 1e6:.6f}" if raw_time_usec else ""
        else:
            raw_time_usec = ""
            boot_s = (
                f"{message.time_boot_ms / 1000.0:.6f}"
                if message.time_boot_ms else ""
            )
        loggers["imu"].write({
            "wall_time_s": wall_s,
            "time_usec": raw_time_usec,
            "boot_time_s": boot_s,
            "source": kind,
            "acc_x_m_s2": f"{message.xacc * GRAVITY_M_S2 / 1000.0:.5f}",
            "acc_y_m_s2": f"{message.yacc * GRAVITY_M_S2 / 1000.0:.5f}",
            "acc_z_m_s2": f"{message.zacc * GRAVITY_M_S2 / 1000.0:.5f}",
            "gyro_x_rad_s": f"{message.xgyro / 1000.0:.6f}",
            "gyro_y_rad_s": f"{message.ygyro / 1000.0:.6f}",
            "gyro_z_rad_s": f"{message.zgyro / 1000.0:.6f}",
        })
        return True
    if kind == "ATTITUDE":
        clock.update(message.time_boot_ms)
        loggers["attitude"].write({
            "wall_time_s": wall_s,
            "time_boot_ms": message.time_boot_ms,
            "roll_rad": f"{message.roll:.6f}",
            "pitch_rad": f"{message.pitch:.6f}",
            "yaw_rad": f"{message.yaw:.6f}",
            "roll_rate_rad_s": f"{message.rollspeed:.6f}",
            "pitch_rate_rad_s": f"{message.pitchspeed:.6f}",
            "yaw_rate_rad_s": f"{message.yawspeed:.6f}",
        })
        return True
    if kind == "LOCAL_POSITION_NED":
        clock.update(message.time_boot_ms)
        loggers["local_position"].write({
            "wall_time_s": wall_s,
            "time_boot_ms": message.time_boot_ms,
            "north_m": f"{message.x:.3f}",
            "east_m": f"{message.y:.3f}",
            "down_m": f"{message.z:.3f}",
            "vn_m_s": f"{message.vx:.3f}",
            "ve_m_s": f"{message.vy:.3f}",
            "vd_m_s": f"{message.vz:.3f}",
        })
        return True
    if kind == "GPS_RAW_INT":
        loggers["gps_raw"].write({
            "wall_time_s": wall_s,
            "time_usec": message.time_usec,
            "fix_type": message.fix_type,
            "lat_deg": f"{message.lat / 1e7:.9f}",
            "lon_deg": f"{message.lon / 1e7:.9f}",
            "alt_m": f"{message.alt / 1000.0:.3f}",
            "eph_m": f"{message.eph / 100.0:.3f}",
            "epv_m": f"{message.epv / 100.0:.3f}",
            "vel_m_s": f"{message.vel / 100.0:.3f}",
            "satellites": message.satellites_visible,
        })
        return True
    return False


class MavlinkLogger(threading.Thread):
    """Reads one MAVLink link and writes every message to the CSVs.

    A dedicated link, not the one the flight commands use. pymavlink's
    recv_match(type=...) discards every message it reads that is not of the
    requested type, so a command waiting five seconds for a
    GLOBAL_POSITION_INT throws away every IMU and attitude sample that
    arrives meanwhile. Sharing one link for everything therefore records the
    IMU at 4.8 Hz instead of 50 Hz, which leaves the VIO with one gyro sample
    every other frame.

    This thread owns its link and never lets a sample go unread.
    """

    def __init__(self, master, loggers, wall_origin, clock):
        super().__init__(daemon=True)
        self.master = master
        self.loggers = loggers
        self.wall_origin = wall_origin
        self.clock = clock
        self.messages = 0
        self.errors = 0
        self.type_counts = {}
        # Not named _stop: threading.Thread already has a _stop() method,
        # and overwriting it makes join() raise "'Event' object is not
        # callable" at teardown.
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            message = self.master.recv_match(blocking=True, timeout=0.5)
            if message is None:
                continue
            kind = message.get_type()
            self.messages += 1
            self.type_counts[kind] = self.type_counts.get(kind, 0) + 1
            try:
                log_message(message, self.loggers, self.wall_origin,
                            self.clock)
            except Exception as exc:  # noqa: BLE001 - must not kill the log
                # A malformed message must not stop the thread: that would
                # truncate the dataset silently, with no error in the output.
                self.errors += 1
                print(f"WARNING: could not log {kind}: {exc}")

    def stop(self):
        self._stop_event.set()
        self.join(timeout=3)


# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------

def fly_square(master):
    """Fly the recording profile and return the phase list.

    Nothing is logged here: MavlinkLogger already holds every MAVLink
    message from its own link, so this function only commands the aircraft
    and waits for it to comply.

    Phases are recorded in dataset.json so the analysis can label the
    trajectory without re-deriving which leg it was on. The corners are
    cumulative positions: each leg is LEG_M long and the fourth leg brings
    the aircraft back over the arming point.
    """
    phases = []
    mavlink_link.set_mode(master, "GUIDED")
    phases.append("guided")
    mavlink_link.arm(master)
    phases.append("armed")
    mavlink_link.takeoff(master, TARGET_ALTITUDE_M)
    phases.append("takeoff")
    mavlink_link.wait_for_altitude(
        master, TARGET_ALTITUDE_M, ALTITUDE_TOLERANCE_M, CLIMB_TIMEOUT_S,
    )
    phases.append("altitude")

    corners = [
        ("leg_north", LEG_M, 0.0),
        ("leg_east", LEG_M, LEG_M),
        ("leg_south", 0.0, LEG_M),
        ("leg_west", 0.0, 0.0),
    ]
    for name, north_m, east_m in corners:
        mavlink_link.goto_local_ned(
            master, north_m, east_m, -TARGET_ALTITUDE_M,
            LEG_TIMEOUT_S, 0.5, None, CORNER_HOVER_S,
        )
        phases.append(name)

    mavlink_link.land(master, LAND_TIMEOUT_S)
    phases.append("land")
    return phases


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def clock_offset_stats(rows):
    """Describe how well the autopilot clock and the Gazebo clock agree.

    For every frame the offset (boot - sim) is computed. If the two clocks
    ran at the same rate the offsets would be identical; the spread is the
    honest measure of how far they drifted apart during the flight. The
    median is the single number the analysis uses to place a frame on the
    IMU timeline.
    """
    offsets = [
        float(row["boot_time_s"]) - float(row["sim_time_s"])
        for row in rows
        if row["boot_time_s"] not in ("", None)
    ]
    if not offsets:
        return {"samples": 0}
    return {
        "samples": len(offsets),
        "median_s": round(statistics.median(offsets), 4),
        "min_s": round(min(offsets), 4),
        "max_s": round(max(offsets), 4),
        "spread_s": round(max(offsets) - min(offsets), 4),
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record a camera + IMU + GPS dataset by flying a square"
    )
    parser.add_argument("--output", default=None,
                        help="Dataset directory. Default results/task3/dataset_<stamp>")
    parser.add_argument("--leg", type=float, default=LEG_M,
                        help="Square side length in metres")
    parser.add_argument("--altitude", type=float, default=TARGET_ALTITUDE_M,
                        help="Flight altitude in metres above home")
    parser.add_argument("--connect", default=mavlink_link.CONNECTION,
                        help="pymavlink connection string for the flight "
                             "commands")
    parser.add_argument("--logger-connect", default=LOGGER_CONNECTION,
                        help="pymavlink connection string for the logging "
                             "link, which must not be the command link")
    return parser.parse_args()


def main():
    args = parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if args.output is None:
        dataset_dir = ROOT / "results" / "task3" / f"dataset_{stamp}"
    else:
        dataset_dir = Path(args.output)
    images_dir = dataset_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    print(f"Dataset directory: {dataset_dir}")

    # Intrinsics first: if the camera is not publishing there is no point
    # arming an aircraft, and a dataset without intrinsics cannot be used.
    intrinsics = read_intrinsics()
    print(
        f"Camera: {intrinsics['width']}x{intrinsics['height']} "
        f"fx={intrinsics['fx']:.4f} fy={intrinsics['fy']:.4f} "
        f"cx={intrinsics['cx']:.2f} cy={intrinsics['cy']:.2f}"
    )

    # Point the camera down before the first frame is recorded, so the whole
    # dataset shares one camera orientation. The gimbal holds the target
    # angle: it does not drift back to level.
    camera.set_pitch(GIMBAL_PITCH_RAD)

    master = mavlink_link.connect(args.connect)
    # The command link only carries what the commands need to read.
    mavlink_link.request_telemetry(master, COMMAND_RATES)
    mavlink_link.wait_until_ready(master, READY_TIMEOUT_S)

    # The logger link is a second connection on purpose; see MavlinkLogger.
    # It is opened after the aircraft is ready so its streams do not start
    # during the GPS/EKF wait, and it is closed after the post-landing pause.
    logger_master = mavlink_link.connect(args.logger_connect)
    mavlink_link.request_telemetry(logger_master, STREAM_RATES)

    loggers = make_loggers(dataset_dir)
    wall_origin = time.time()
    clock = SharedClock()
    logger_thread = MavlinkLogger(logger_master, loggers, wall_origin, clock)
    logger_thread.start()

    recorder = ImageRecorder(IMAGE_TOPIC, images_dir, clock, wall_origin)
    if not recorder.subscribed:
        raise RuntimeError(f"Could not subscribe to {IMAGE_TOPIC}")
    if not recorder.first_frame.wait(FIRST_MESSAGE_TIMEOUT_S):
        raise TimeoutError(
            f"No camera image on {IMAGE_TOPIC} within "
            f"{FIRST_MESSAGE_TIMEOUT_S:.0f} s. Run `gz topic -l | grep camera` "
            "to confirm the topic name."
        )
    print("Camera streaming; starting the square")

    flight_ok = False
    phases = []
    try:
        phases = fly_square(master)
        flight_ok = True
    finally:
        # Stop recording whatever happened. A partial flight is still a
        # dataset, as long as dataset.json says it was partial. The logger
        # is stopped before the CSV handles are closed, otherwise its thread
        # would still be writing to closed files.
        time.sleep(POST_LAND_SECONDS)
        logger_thread.stop()
        if logger_thread.errors:
            print(f"WARNING: {logger_thread.errors} messages could not be "
                  "logged; see the messages above")
        frame_rows = recorder.stop()
        for logger in loggers.values():
            logger.close()

    frames_csv = CsvWriter(dataset_dir / "frames.csv", [
        "frame", "file", "sim_time_s", "wall_time_s", "boot_time_s",
    ])
    for row in frame_rows:
        frames_csv.write(row)
    frames_csv.close()

    sim_times = [float(row["sim_time_s"]) for row in frame_rows]
    wall_times = [float(row["wall_time_s"]) for row in frame_rows]
    sim_span = sim_times[-1] - sim_times[0] if len(sim_times) > 1 else 0.0
    wall_span = wall_times[-1] - wall_times[0] if len(wall_times) > 1 else 0.0
    if sim_span > 0:
        # Achieved rate per second of simulated time. This is the number
        # that matters: the simulation runs slower than real time, so
        # messages per wall-clock second understate what the VIO receives.
        achieved = {
            kind: round(count / sim_span, 2)
            for kind, count in sorted(logger_thread.type_counts.items())
        }
    else:
        achieved = {}
    dataset = {
        "kind": "task3_camera_imu_gps",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                     time.gmtime(wall_origin)),
        "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "flight_completed": flight_ok,
        "phases": phases,
        "camera": {
            "image_topic": IMAGE_TOPIC,
            "info_topic": INFO_TOPIC,
            "transport": "gz-transport13 Python bindings (no ROS)",
            "intrinsics": intrinsics,
            "gimbal_pitch_rad": GIMBAL_PITCH_RAD,
            "format": "RGB_INT8 PNG, colour channel flipped to BGR for cv2",
        },
        "frames": {
            "count": len(frame_rows),
            "dropped": recorder.dropped,
            "mean_interval_s": (
                round((sim_times[-1] - sim_times[0]) / (len(sim_times) - 1), 4)
                if len(sim_times) > 1 else None
            ),
        },
        "streams": {
            "requested_hz": {
                # Message names, not ids: "RAW_IMU": 50.0 reads better in a
                # report than "27": 50.0.
                mavutil.mavlink.mavlink_map[key].msgname: value
                for key, value in STREAM_RATES.items()
            },
            "achieved_hz": achieved,
            "sim_duration_s": round(sim_span, 3),
            "real_time_factor": (
                round(sim_span / wall_span, 3) if wall_span > 0 else None
            ),
            "logger_link": args.logger_connect,
            "command_link": args.connect,
            "units": {
                "imu_acceleration": "m/s^2",
                "imu_gyro": "rad/s",
                "attitude": "rad",
                "local_position": "m, m/s",
                "time": "seconds on each message's own clock",
            },
        },
        "clock": {
            "note": "boot_time_s is ArduCopter time_boot_ms; sim_time_s is "
                    "the Gazebo image stamp. The offset is measured, not assumed.",
            "boot_minus_sim": clock_offset_stats(frame_rows),
        },
        "flight": {
            "target_altitude_m": args.altitude,
            "leg_length_m": args.leg,
            "corner_hover_s": CORNER_HOVER_S,
        },
    }
    (dataset_dir / "dataset.json").write_text(json.dumps(dataset, indent=2) + "\n")

    print(f"Frames: {len(frame_rows)} (+{recorder.dropped} dropped)")
    print(f"Clock offset: {dataset['clock']['boot_minus_sim']}")
    print(f"Written: {dataset_dir / 'dataset.json'}")
    return 0 if flight_ok else 1


if __name__ == "__main__":
    sys.exit(main())
