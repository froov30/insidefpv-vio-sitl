#!/usr/bin/env python3
"""MAVLink plumbing shared by the ground-station scripts.

This module is the only place in the project that opens a MAVLink link and
the only place that knows MAVLink units. Every caller - takeoff_land.py,
record_dataset.py and test_task2.py - gets the same connection, the same
readiness test, and the same mode-change/ack handling.

MAVLink is the protocol ArduCopter speaks. pymavlink is a pure-Python
implementation, so no ROS and no MAVProxy are involved anywhere.

One connection rule matters more than any other here. SITL's SERIAL0 is
TCP 127.0.0.1:5760 with the "wait for client" option
(libraries/AP_HAL_SITL/SITL_State.h). It accepts exactly one client and
never accepts another. If this module connects to a SITL left over from an
earlier run, the TCP connection succeeds (the kernel completes the
handshake from the listen backlog) but no heartbeat ever arrives, because
the flight controller's socket belongs to a client that is already gone.
scripts/run_sim.sh kills such leftovers before every run; the error text
in connect() names that script for exactly this reason.
"""

import time

from pymavlink import mavutil

# TCP port sim_vehicle.py and build/sitl/bin/arducopter listen on for
# instance -I0. Change it only together with scripts/run_sitl.sh.
CONNECTION = "tcp:127.0.0.1:5760"

# EKF_STATUS_FLAGS bits that must all be set before arming is accepted:
#   1  attitude estimate good
#   16 horizontal position estimate good
#   32 vertical position estimate good
# A smaller mask lets the script declare readiness while ArduCopter still
# rejects the arm with "PreArm: Need Position Estimate".
EKF_READY_MASK = 1 | 16 | 32

# MAV_RESULT values that mean "the command was taken". IN_PROGRESS is used
# by long-running commands; treating it as failure would abort a valid
# takeoff.
ACCEPTED_RESULTS = (
    mavutil.mavlink.MAV_RESULT_ACCEPTED,
    mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
)

# Pre-arm refusals that are a matter of elapsed time, not of a wrong
# configuration. Each one is quoted from the ArduPilot source that emits it:
#
#   "Accels inconsistent" / "Gyros inconsistent"
#       AP_Arming::ins_accels_consistent() and ins_gyros_consistent() require
#       the IMUs to agree continuously for AP_ARMING_IMU_CONSISTENCY_CHECK_TIME_MS
#       (10 000 ms, AP_Arming.cpp). On a freshly started SITL the sensor
#       models have not converged yet, so this is the normal first answer.
#   "EKF attitude is bad"
#       ArduCopter/AP_Arming_Copter.cpp ins_checks(), until gyro bias settles.
#   "EKF <name> variance"
#       mandatory_position_checks() compares the EKF variances with
#       FS_EKF_THRESH; they decay as the filter converges.
#   "Need Position Estimate", "AHRS: ...", "System not initialised",
#       "GPS glitching" - all wait for the EKF/GPS to settle.
#
# Everything else ("Compass not calibrated", "Hardware safety switch",
# "RC not found", "Motors: ...") never clears by waiting and must fail the
# run immediately. This list is only consulted for those refusals, and
# every retry prints the autopilot's own reason, so a genuine fault is
# still visible in the log rather than hidden behind a retry loop.
TRANSIENT_PREARM_PATTERNS = (
    "accels inconsistent",
    "gyros inconsistent",
    "ekf attitude is bad",
    "ekf position variance",
    "ekf velocity variance",
    "ekf height variance",
    "ekf compass variance",
    "need position estimate",
    "ahrs: ",
    "system not initialised",
    "gps glitching",
)


def connect(connection=CONNECTION, timeout_s=30):
    """Open the link and wait for the first heartbeat.

    wait_heartbeat() both reads the heartbeat and fills in target_system
    and target_component. Every command sent before it would be addressed
    to system 0 and silently ignored, so nothing else happens until this
    returns.
    """
    print(f"Connecting to {connection}")
    master = mavutil.mavlink_connection(connection)
    heartbeat = master.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise TimeoutError(
            f"No heartbeat within {timeout_s:.0f} s. Either SITL is not "
            "running, or a SITL from an earlier attempt still owns TCP "
            "5760 and can no longer answer. Start a fresh pair with "
            "`bash scripts/run_sim.sh` (it kills stale processes first)."
        )
    print(
        f"Heartbeat from system {master.target_system} "
        f"component {master.target_component}"
    )
    return master


def set_message_interval(master, message_id, rate_hz):
    """Ask the flight controller to send one message type at rate_hz.

    The command takes microseconds: 10 Hz becomes 100000 us. An interval
    of 0 means "default rate" in MAVLink, not "off", which is why this
    function never passes 0.
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


def request_telemetry(master, rates):
    """Turn on the message streams the caller needs.

    rates maps a MAVLINK_MSG_ID_* constant to a rate in Hz. SITL's default
    stream is too slow for a 10 Hz position log: without this the altitude
    check can miss the whole climb and time out even though the aircraft
    flew.
    """
    for message_id, rate_hz in rates.items():
        set_message_interval(master, message_id, rate_hz)


def wait_until_ready(master, timeout_s, log_pre_arm=True):
    """Block until GPS has a 3D fix and the EKF is willing to fly.

    Arming before this returns is the usual cause of "PreArm: Need
    Position Estimate" or "PreArm: EKF attitude is bad". In this project
    readiness also proves the Gazebo link works: GPS and the EKF can only
    be good if ArduPilotPlugin is feeding IMU and position data over
    UDP 9002.
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
        if kind == "STATUSTEXT" and log_pre_arm:
            # Pre-arm failures are only reported as text. Printing them is
            # what makes a rejected arm explainable from the log.
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
        "or it was left paused, or SITL is not using --model JSON. "
        "Check logs/sim/gazebo.log for 'ArduPilot ready to fly' and "
        "logs/sim/sitl_console.log for EKF lines."
    )


def set_mode(master, mode_name, timeout_s=15):
    """Switch flight mode and wait until the heartbeat agrees.

    mode_mapping() is read from the heartbeat, so a firmware change that
    renumbers modes does not break this. GUIDED accepts takeoff and
    position setpoints; STABILIZE ignores both.
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
    """Send COMMAND_LONG and return the matching COMMAND_ACK.

    params is seven floats (MAVLink param1..param7). The ack's result is
    MAV_RESULT: 0 accepted, 2 denied, 4 failed, 5 in progress. This
    function does not retry: a retry loop would hide a systematic denial.
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


def drain_status_text(master, duration_s=1.0):
    """Collect the STATUSTEXT messages that arrive over the next second.

    ArduCopter answers a refused arm with COMMAND_ACK first and the human
    readable reason as STATUSTEXT immediately after it. A caller that
    returns the moment the ack arrives loses that reason, which is exactly
    how the first takeoff in this project failed with an unexplained
    MAV_RESULT=4.

    recv_match(type=...) discards the other message types it reads on the
    way, which is acceptable here: they are periodic streams that arrive
    again within a second.
    """
    texts = []
    deadline = time.time() + duration_s
    while time.time() < deadline:
        msg = master.recv_match(
            type="STATUSTEXT", blocking=True,
            timeout=max(0.01, deadline - time.time()),
        )
        if msg is None:
            break
        texts.append(msg.text)
    return texts


def arm(master, timeout_s=45, retry_interval_s=2.0):
    """Arm the motors, waiting out pre-arm checks that are time based.

    param1=1 arms, param2=0 runs the normal pre-arm checks. param2=21196
    would skip them; this project never does that, because a forced arm
    hides exactly the GPS/EKF failure the test is meant to catch.

    ArduCopter refuses an arm for two very different kinds of reason: a
    check that is waiting on time (the IMUs agreeing for 10 s, the EKF
    variance decaying) and a check that is a real fault. MAVLink exposes
    neither over a query - the only way to learn which one it is, is to ask
    to arm and read the reason. So the arm is repeated, but only while the
    reason matches TRANSIENT_PREARM_PATTERNS, only for timeout_s seconds,
    and every refusal is printed. A persistent fault still fails the run,
    with the autopilot's own words in the error.
    """
    deadline = time.time() + timeout_s
    attempt = 0
    while True:
        attempt += 1
        print(f"Arming (attempt {attempt})")
        ack = command_long(
            master,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1, 0, 0, 0, 0, 0, 0),
        )
        if ack.result in ACCEPTED_RESULTS:
            break

        # The reason, if there is one, is sent right after the ack.
        texts = drain_status_text(master)
        for text in texts:
            print(f"FC: {text}")
        reason = "; ".join(texts) if texts else (
            f"MAV_RESULT={ack.result} and no pre-arm text"
        )
        transient = any(
            pattern in text.lower()
            for text in texts
            for pattern in TRANSIENT_PREARM_PATTERNS
        )
        if not transient or time.time() + retry_interval_s >= deadline:
            raise RuntimeError(f"Arm denied: {reason}")
        print(
            f"Arm refused ({reason}); retrying in {retry_interval_s:.1f} s. "
            "This check is time based and clears on its own."
        )
        time.sleep(retry_interval_s)

    # The ack only says the request was accepted; the armed flag in the
    # heartbeat is the state that the rest of the flight depends on.
    deadline = time.time() + 10
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        # base_mode bit 128 (MAV_MODE_FLAG_SAFETY_ARMED) is the armed flag.
        if hb is not None and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Motors armed")
            return
    raise TimeoutError("COMMAND_ACK said armed, but the heartbeat never showed it")


def current_position(master, timeout_s=5):
    """Return the latest GLOBAL_POSITION_INT.

    lat/lon are degrees * 1e7; alt/relative_alt are millimetres.
    relative_alt is height above the EKF origin, which on this SITL is the
    arming home. It is the altitude every horizontal constant in this
    project is written against.
    """
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout_s)
    if msg is None:
        raise TimeoutError("No GLOBAL_POSITION_INT. Telemetry rate was not accepted.")
    return msg


def takeoff(master, altitude_m, timeout_s=10):
    """Command a vertical takeoff to altitude_m above home.

    The frame is MAV_FRAME_GLOBAL_RELATIVE_ALT (3), not the _INT variant.
    ArduCopter checks it literally before doing anything else:

        if (packet.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT) {
            return MAV_RESULT_DENIED;
        }
        ArduCopter/GCS_MAVLink_Copter.cpp, handle_MAV_CMD_NAV_TAKEOFF

    COMMAND_INT with MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (6) is the natural
    choice on paper and is what this function sent first; the answer was
    MAV_RESULT_DENIED (2) with no status text, because frame 6 is not 3.
    z is still metres above home in either case, x and y are still degE7;
    ArduCopter ignores both for a vertical takeoff.
    """
    here = current_position(master)
    print(
        f"Takeoff to {altitude_m:.1f} m above home "
        f"at lat {here.lat / 1e7:.7f}, lon {here.lon / 1e7:.7f}"
    )
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # current; unused by ArduCopter
        0,  # autocontinue
        0, 0, 0, 0,
        here.lat,
        here.lon,
        int(round(altitude_m)),
    )
    deadline = time.time() + timeout_s
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
            if msg.result not in ACCEPTED_RESULTS:
                # As with the arm, the reason arrives as STATUSTEXT just
                # after the ack, so it is read before failing.
                texts = drain_status_text(master)
                for text in texts:
                    print(f"FC: {text}")
                reason = "; ".join(texts) if texts else "no status text"
                raise RuntimeError(
                    f"Takeoff denied: MAV_RESULT={msg.result} ({reason})"
                )
            print("Takeoff accepted")
            return
    raise TimeoutError("No ack for MAV_CMD_NAV_TAKEOFF")


def wait_for_altitude(master, altitude_m, tolerance_m=0.5, timeout_s=90,
                      position_callback=None, print_interval_s=1.0):
    """Block until relative_alt reaches altitude_m - tolerance_m.

    relative_alt is height above the EKF origin (the arming point here),
    not GPS altitude and not a Gazebo coordinate. position_callback is
    called with every GLOBAL_POSITION_INT so a caller can log the climb;
    the return value is the message at which the band was entered.

    The band is one-sided on purpose: overshooting the target is still a
    success (the aircraft flew higher than asked), while stopping below
    the band would mean the takeoff command did not take effect.
    """
    target_m = altitude_m - tolerance_m
    print(f"Waiting until altitude >= {target_m:.2f} m")
    deadline = time.time() + timeout_s
    last_print = 0.0
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            print(f"FC: {msg.text}")
            continue
        if kind != "GLOBAL_POSITION_INT":
            continue
        if position_callback is not None:
            position_callback(msg)
        alt_m = msg.relative_alt / 1000.0
        now = time.time()
        if now - last_print > print_interval_s:
            print(f"  climb  relative_alt={alt_m:.2f} m")
            last_print = now
        if alt_m >= target_m:
            print(f"Reached {alt_m:.2f} m")
            return msg
    raise TimeoutError(
        f"Did not reach {altitude_m} m. Check that GUIDED takeoff was "
        "accepted and that Gazebo's real-time factor is near 1."
    )


def land(master, timeout_s=120, position_callback=None, print_interval_s=1.0):
    """Switch to LAND and return once the flight controller disarms.

    LAND descends using the WPNAV_SPEED_DN / LAND_SPEED parameters already
    in the parameter files; nothing here changes them. No disarm command
    is sent: ArduCopter disarms itself on touchdown, and disarming in the
    air would drop the aircraft.
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
            if position_callback is not None:
                position_callback(msg)
            now = time.time()
            if now - last_print > print_interval_s:
                print(f"  land   relative_alt={msg.relative_alt / 1000.0:.2f} m")
                last_print = now
        elif kind == "HEARTBEAT":
            # base_mode bit 128 clear means the motors are disarmed, which
            # ArduCopter only does after it has detected the ground.
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Motors disarmed. Landing complete.")
                return
    raise TimeoutError("LAND mode did not disarm within the timeout")


# Type mask for SET_POSITION_TARGET_LOCAL_NED that uses position only:
# velocity, acceleration, yaw and yaw rate are ignored. The bits that stay
# set (1..3, 7..9) are "ignore", so this value is the documented
# 0b0000111111111000.
POSITION_ONLY_MASK = 0b0000111111111000

# Spacing of the repeated position setpoints. ArduCopter's GUIDED mode
# accepts a single setpoint, but re-sending it keeps the target alive if a
# packet is lost and makes the target visible in the logs.
SETPOINT_RATE_HZ = 5.0


def goto_local_ned(master, north_m, east_m, down_m, timeout_s=30,
                   acceptance_m=0.5, position_callback=None, dwell_s=0.0):
    """Fly to a position in metres, in the EKF's local NED frame.

    north_m and east_m are metres from home; down_m is metres down, so
    -5.0 is 5 m above home. ArduCopter accepts this frame in GUIDED. The
    function resends the setpoint at SETPOINT_RATE_HZ and returns the last
    GLOBAL_POSITION_INT once LOCAL_POSITION_NED is inside acceptance_m.

    dwell_s keeps sending the same setpoint for that many additional
    seconds after arrival. record_dataset.py uses it to hold still at each
    corner of the square, which is what gives the VIO a stationary period
    to check drift against.

    position_callback, if given, is called with every GLOBAL_POSITION_INT
    received while flying. record_dataset.py uses it to log the GPS track
    without opening a second reader on the same socket.

    Why two message types: the setpoint is compared with the flight
    controller's own local estimate (LOCAL_POSITION_NED), which is already
    in metres and shares the EKF origin, so no lat/lon conversion is
    needed in the control loop. GLOBAL_POSITION_INT is what gets logged
    and what the metric report is computed from.
    """
    interval_s = 1.0 / SETPOINT_RATE_HZ
    deadline = time.time() + timeout_s
    reached_at = None
    last_send = 0.0
    last_gps = None
    print(f"GUIDED setpoint: N {north_m:+.1f} m, E {east_m:+.1f} m, "
          f"D {down_m:+.1f} m")
    while time.time() < deadline:
        now = time.time()
        if now - last_send >= interval_s:
            master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms; 0 means "now" for this message
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                POSITION_ONLY_MASK,
                north_m, east_m, down_m,
                0, 0, 0,   # vx, vy, vz (ignored by the mask)
                0, 0, 0,   # afx, afy, afz (ignored by the mask)
                0, 0,      # yaw, yaw_rate (ignored by the mask)
            )
            last_send = now
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"],
            blocking=True,
            timeout=0.2,
        )
        if msg is None:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            last_gps = msg
            if position_callback is not None:
                position_callback(msg)
        else:
            dn = msg.x - north_m
            de = msg.y - east_m
            if dn * dn + de * de <= acceptance_m * acceptance_m:
                if reached_at is None:
                    reached_at = now
                    print(f"Reached setpoint N {north_m:+.1f} E {east_m:+.1f}"
                          + (f", holding {dwell_s:.1f} s" if dwell_s > 0 else ""))
                if now - reached_at >= dwell_s:
                    return last_gps
    raise TimeoutError(
        f"Did not reach N {north_m:.1f} E {east_m:.1f} within {timeout_s:.0f} s"
    )
