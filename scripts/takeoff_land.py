#!/usr/bin/env python3
"""Guided takeoff, hover, and landing for the Gazebo Iris.

This script is the ground station. It does not simulate the aircraft and
it does not run the flight controller. It speaks MAVLink to ArduCopter
SITL, which is already flying the Iris inside Gazebo.

Start order, each in its own terminal:

    bash scripts/run_gazebo.sh
    bash scripts/run_sitl.sh
    python3 scripts/takeoff_land.py

Change the flight by changing the constants below. Each comment says
what that number does to the aircraft.
"""

import argparse
import csv
import sys
import time
from pathlib import Path

from pymavlink import mavutil


# ---------------------------------------------------------------------------
# Flight numbers. These are the values an interviewer will ask you to change.
# ---------------------------------------------------------------------------

# Metres above the point ArduCopter stored as home. This is not a GPS
# altitude and it is not a Gazebo z coordinate. 5 becomes a 5 m climb;
# 10 becomes a 10 m climb. The Iris will not stop lower than
# (this value - ALTITUDE_TOLERANCE_M).
TARGET_ALTITUDE_M = 5.0

# How close to TARGET_ALTITUDE_M counts as "arrived", in metres.
# 0.5 means a reading of 4.5 m is accepted for a 5 m target. Making this
# smaller waits longer and can time out on barometer noise. Making it
# larger reports success while the aircraft is still climbing.
ALTITUDE_TOLERANCE_M = 0.5

# Seconds of flight-controller time to hold position after the climb.
# This uses the autopilot clock (time_boot_ms), so a slow simulation
# still hovers for this long in simulated time. 0 skips the hover and
# starts the landing as soon as the altitude check passes.
HOVER_SECONDS = 8.0

# Below this height, after LAND mode is set, the aircraft is treated as
# down. LAND_ALT_THRESH in ArduCopter is also 0.2 m by default; the
# motors disarm only after the controller itself decides it has landed.
LANDED_ALTITUDE_M = 0.3

# How long each phase may take, in wall-clock seconds. Gazebo is in
# lockstep with SITL, so a busy CPU makes these expire before the
# simulated clock has caught up. Increase them if the machine is slow.
# They do not change the path the aircraft flies.
READY_TIMEOUT_S = 90.0
CLIMB_TIMEOUT_S = 90.0
LAND_TIMEOUT_S = 120.0

# MAVLink message rates. The first number is messages per second.
# GLOBAL_POSITION_INT is the fused position the plot in task 4 will use.
# Raising it makes the CSV denser and uses more CPU. Lowering it makes
# the altitude check coarser.
POSITION_RATE_HZ = 10.0
SYS_STATUS_RATE_HZ = 2.0

# TCP port sim_vehicle.py listens on when MAVProxy is not started.
# Change this only if you also change the SITL serial port.
CONNECTION = "tcp:127.0.0.1:5760"

# EKF flags that must all be set before arming. Arming with a smaller
# mask can pass the script and still be rejected by the flight controller.
# Bits are defined by MAVLink EKF_STATUS_FLAGS:
#   1  attitude, 16 horizontal position, 32 vertical position.
EKF_READY_MASK = 1 | 16 | 32


def connect(connection):
    """Open the MAVLink link and wait until ArduCopter identifies itself.

    tcp:127.0.0.1:5760 is a client connection to SITL's serial0 port.
    udpin:0.0.0.0:14550 would be used instead if MAVProxy were forwarding
    telemetry. This script talks to SITL directly, so it uses TCP.
    """
    print(f"Connecting to {connection}")
    master = mavutil.mavlink_connection(connection)
    # The heartbeat carries the vehicle type and the current flight mode.
    # target_system and target_component are filled in from it. Commands
    # sent before this call would be addressed to system 0 and ignored.
    heartbeat = master.wait_heartbeat(timeout=30)
    if heartbeat is None:
        raise TimeoutError(
            "No heartbeat. SITL is not running, or something else is "
            "already connected to TCP port 5760."
        )
    print(
        f"Heartbeat from system {master.target_system} "
        f"component {master.target_component}"
    )
    return master


def set_message_interval(master, message_id, rate_hz):
    """Ask SITL to send one message type at rate_hz.

    MAV_CMD_SET_MESSAGE_INTERVAL takes the interval in microseconds.
    rate_hz of 10 becomes 100000 us. rate_hz of 0 would be an interval
    of 0, which MAVLink defines as "default rate", not "off".
    """
    interval_us = int(1_000_000 / rate_hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # confirmation
        message_id,
        interval_us,
        0, 0, 0, 0, 0,
    )


def request_telemetry(master):
    """Turn on the two streams this script reads.

    SITL's default stream is slow. Without this, the altitude check can
    miss the climb and then time out, even though the aircraft did fly.
    """
    set_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, POSITION_RATE_HZ
    )
    set_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, SYS_STATUS_RATE_HZ
    )
    set_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, SYS_STATUS_RATE_HZ
    )
    set_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, SYS_STATUS_RATE_HZ
    )


def wait_until_ready(master, timeout_s):
    """Block until GPS has a 3D fix and the EKF is willing to fly.

    Arming before this returns is the usual reason SITL answers with
    "PreArm: Need Position Estimate" or "PreArm: EKF attitude is bad".
    Gazebo has to be publishing the IMU and the navsat, and SITL has to
    have accepted that JSON stream on UDP port 9002.
    """
    print("Waiting for GPS 3D fix and EKF position")
    deadline = time.time() + timeout_s
    gps_ok = False
    ekf_ok = False
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            # Pre-arm failures are only reported as text. Print all of
            # them so a rejected arm is explainable from this log.
            print(f"FC: {msg.text}")
        elif kind == "GPS_RAW_INT":
            # fix_type: 0 none, 1 no fix, 2 2D, 3 3D. Arming wants 3.
            gps_ok = msg.fix_type >= 3
        elif kind == "EKF_STATUS_REPORT":
            ekf_ok = (msg.flags & EKF_READY_MASK) == EKF_READY_MASK
        if gps_ok and ekf_ok:
            print("GPS and EKF ready")
            return
    raise TimeoutError(
        "GPS/EKF did not become ready. Gazebo is probably not running, "
        "or it was left paused, or SITL is not using --model JSON."
    )


def set_mode(master, mode_name, timeout_s=15):
    """Switch ArduCopter flight mode and wait until the heartbeat agrees.

    mode_mapping() is read from the heartbeat. GUIDED is mode 4 and LAND
    is mode 9 on the current ArduCopter, but the script uses the names
    so a firmware change that renumbers modes still works.

    GUIDED accepts position and takeoff commands. STABILIZE ignores
    MAV_CMD_NAV_TAKEOFF, which is why the mode is set before arming.
    """
    mapping = master.mode_mapping()
    if mode_name not in mapping:
        raise KeyError(f"Mode {mode_name} is not in {sorted(mapping)}")
    mode_id = mapping[mode_name]
    print(f"Setting mode {mode_name} (custom_mode {mode_id})")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is not None and hb.custom_mode == mode_id:
            print(f"Mode is {mode_name}")
            return
    raise TimeoutError(f"Flight controller did not enter {mode_name}")


def command_long(master, command, params, timeout_s=10):
    """Send a COMMAND_LONG and return the matching COMMAND_ACK.

    param list is seven floats, MAVLink param1..param7. The result field
    of the ack is MAV_RESULT: 0 accepted, 2 denied, 4 failed, 5 in progress.
    A denied arm leaves the motors stopped. This function does not retry.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        *params,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"FC: {msg.text}")
        elif msg.get_type() == "COMMAND_ACK" and msg.command == command:
            return msg
    raise TimeoutError(f"No COMMAND_ACK for command {command}")


def arm(master):
    """Arm the motors.

    param1 = 1 arms. param1 = 0 would disarm. param2 = 0 runs the normal
    pre-arm checks. param2 = 21196 skips those checks; this script does
    not do that, because a forced arm hides a broken GPS/EKF link.
    """
    print("Arming")
    ack = command_long(
        master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        (1, 0, 0, 0, 0, 0, 0),
    )
    if ack.result not in (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    ):
        raise RuntimeError(f"Arm denied, MAV_RESULT={ack.result}")
    deadline = time.time() + 20
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        # base_mode bit 128 (MAV_MODE_FLAG_SAFETY_ARMED) is the armed flag.
        if hb is not None and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Motors armed")
            return
    raise TimeoutError("COMMAND_ACK said armed, but the heartbeat never showed it")


def current_position(master, timeout_s=5):
    """Return the latest GLOBAL_POSITION_INT.

    lat and lon are degrees * 1e7. relative_alt and alt are millimetres.
    relative_alt is height above the EKF origin, which on this SITL is
    the arming home. That is the number compared with TARGET_ALTITUDE_M.
    """
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout_s)
    if msg is None:
        raise TimeoutError("No GLOBAL_POSITION_INT. Telemetry rate was not accepted.")
    return msg


def takeoff(master, altitude_m):
    """Command a vertical takeoff to altitude_m above home.

    COMMAND_INT is used instead of COMMAND_LONG so the altitude frame is
    explicit. MAV_FRAME_GLOBAL_RELATIVE_ALT_INT means z is metres above
    home, not metres above the WGS84 ellipsoid.

    x and y are the current latitude and longitude. Sending 0, 0 would
    command a takeoff toward the Gulf of Guinea. param1..param4 are
    unused by ArduCopter for this command and stay 0.
    """
    here = current_position(master)
    print(
        f"Takeoff to {altitude_m:.1f} m above home "
        f"at lat {here.lat / 1e7:.7f}, lon {here.lon / 1e7:.7f}"
    )
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # current; unused by ArduCopter
        0,  # autocontinue
        0, 0, 0, 0,
        here.lat,
        here.lon,
        altitude_m,
    )
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"FC: {msg.text}")
        elif (
            msg.get_type() == "COMMAND_ACK"
            and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        ):
            if msg.result not in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                raise RuntimeError(f"Takeoff denied, MAV_RESULT={msg.result}")
            print("Takeoff accepted")
            return
    raise TimeoutError("No ack for MAV_CMD_NAV_TAKEOFF")


def log_row(writer, msg, phase):
    """Write one position sample.

    Altitudes are stored in metres so the CSV can be plotted without
    remembering the MAVLink millimetre convention. The raw message is
    not modified.
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

    The loop also writes the CSV. A sample is printed about once a
    second so the terminal itself is a readable record of the climb.
    """
    print(f"Waiting until altitude >= {altitude_m - tolerance_m:.2f} m")
    deadline = time.time() + timeout_s
    last_print = 0.0
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"FC: {msg.text}")
            continue
        if msg.get_type() != "GLOBAL_POSITION_INT":
            continue
        alt_m = msg.relative_alt / 1000.0
        log_row(writer, msg, "climb")
        now = time.time()
        if now - last_print > 1.0:
            print(f"  climb  relative_alt={alt_m:.2f} m")
            last_print = now
        if alt_m >= altitude_m - tolerance_m:
            print(f"Reached {alt_m:.2f} m")
            return msg
    raise TimeoutError(
        f"Did not reach {altitude_m} m. Check that GUIDED takeoff was "
        "accepted and that Gazebo's real-time factor is near 1."
    )


def hover(master, writer, start_msg, hover_s):
    """Hold for hover_s seconds of autopilot time.

    GUIDED mode already holds the takeoff position. This function does
    not send another setpoint. It only waits and logs. Changing
    HOVER_SECONDS changes the wait. It does not change the hold gains.
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

    LAND descends at the WPNAV_SPEED_DN / LAND_SPEED parameters already
    in the copter parameter file. This script does not change those
    speeds. A lower LAND_SPEED in the parameter file makes this phase
    take longer; the timeout above is the only number in this file that
    limits it.

    The function does not send a disarm command. ArduCopter disarms
    itself after it detects touchdown. Sending disarm while still in
    the air would drop the Iris.
    """
    set_mode(master, "LAND")
    print("Landing. Waiting for the flight controller to disarm.")
    deadline = time.time() + timeout_s
    last_print = 0.0
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            print(f"FC: {msg.text}")
        elif kind == "GLOBAL_POSITION_INT":
            log_row(writer, msg, "land")
            now = time.time()
            if now - last_print > 1.0:
                print(f"  land   relative_alt={msg.relative_alt / 1000.0:.2f} m")
                last_print = now
        elif kind == "HEARTBEAT":
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                print("Motors disarmed. Landing complete.")
                return
    raise TimeoutError("LAND mode did not disarm within the timeout")


def parse_args():
    parser = argparse.ArgumentParser(description="Guided takeoff and landing")
    # Command-line values override the constants. The constants stay the
    # documented defaults so the file itself shows the flight you ran.
    parser.add_argument("--altitude", type=float, default=TARGET_ALTITUDE_M,
                        help="Takeoff height above home, metres")
    parser.add_argument("--hover", type=float, default=HOVER_SECONDS,
                        help="Hover time, seconds of autopilot time")
    parser.add_argument("--connect", default=CONNECTION,
                        help="pymavlink connection string")
    parser.add_argument("--log", default=None,
                        help="CSV path. Default is logs/takeoff_land_<time>.csv")
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

    master = connect(args.connect)
    request_telemetry(master)
    wait_until_ready(master, READY_TIMEOUT_S)

    with log_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            "time_boot_ms", "phase", "lat_deg", "lon_deg",
            "relative_alt_m", "amsl_alt_m", "vx_m_s", "vy_m_s", "vz_m_s",
        ])
        writer.writeheader()
        # GUIDED before arm so the takeoff command is legal the moment
        # the motors start. Arming in STABILIZE and then changing mode
        # also works, but the aircraft can yaw on the ground in between.
        set_mode(master, "GUIDED")
        arm(master)
        takeoff(master, args.altitude)
        arrived = wait_for_altitude(
            master, writer, args.altitude, ALTITUDE_TOLERANCE_M, CLIMB_TIMEOUT_S
        )
        hover(master, writer, arrived, args.hover)
        land(master, writer, LAND_TIMEOUT_S)

    print(f"Position log written to {log_path}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
