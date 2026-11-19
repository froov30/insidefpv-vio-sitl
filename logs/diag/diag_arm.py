#!/usr/bin/env python3
"""One-off diagnostic: trace readiness from a cold start, then try to arm.

The first real takeoff on a freshly started simulator was refused with
MAV_RESULT=4 straight after "GPS and EKF ready". scripts/mavlink_link.py
declares readiness from three EKF_STATUS_FLAGS bits, but ArduCopter also
requires the EKF variances to be below FS_EKF_THRESH before it arms
(ArduCopter/AP_Arming_Copter.cpp, mandatory_position_checks). This script
records both, and prints every STATUSTEXT around the arm attempt, so the
refusal is attributable to a specific check instead of guessed at.

Not part of the deliverable; the finding goes into the handover notes.
"""

import sys
import time

from pymavlink import mavutil

CONNECTION = "tcp:127.0.0.1:5760"

# The mask scripts/mavlink_link.py gates on: attitude + horizontal and
# vertical position estimate. Kept identical on purpose - the point is to
# show what this mask does and does not guarantee.
EKF_READY_MASK = 1 | 16 | 32

# Long enough for a cold SITL to get a GPS fix and converge the EKF.
READY_TIMEOUT_S = 120


def main():
    master = mavutil.mavlink_connection(CONNECTION, autoreconnect=False)
    print(f"Connecting to {CONNECTION}")
    master.wait_heartbeat(timeout=30)
    print(f"Heartbeat from system {master.target_system} "
          f"component {master.target_component}")

    # Turn on everything at 4 Hz. The default streams are slower than the
    # EKF status changes, and a slow stream would make it impossible to say
    # which state the aircraft was in when the arm was refused.
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
    )

    # Watch readiness. Each EKF_STATUS_REPORT line is one sample: fix type,
    # flag word, and the three variances ArduCopter compares against
    # FS_EKF_THRESH. Only transitions are printed, so the output stays
    # readable while still showing every state change.
    print("--- readiness trace (flags/variances, transitions only) ---")
    deadline = time.time() + READY_TIMEOUT_S
    gps_fix = None
    last_line = None
    ready = False
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "GPS_RAW_INT":
            if msg.fix_type != gps_fix:
                gps_fix = msg.fix_type
                print(f"  t={READY_TIMEOUT_S - (deadline - time.time()):5.1f}s "
                      f"GPS fix_type={gps_fix} sats={msg.satellites_visible}")
        elif kind == "EKF_STATUS_REPORT":
            flags_ok = (msg.flags & EKF_READY_MASK) == EKF_READY_MASK
            line = (
                f"  +{READY_TIMEOUT_S - (deadline - time.time()):5.1f}s "
                f"flags=0x{msg.flags:04x} mask_ok={flags_ok} "
                f"vel_var={msg.velocity_variance:.4f} "
                f"pos_h_var={msg.pos_horiz_variance:.4f} "
                f"pos_v_var={msg.pos_vert_variance:.4f}"
            )
            if line.split("s ", 1)[1] != last_line:
                print(line)
                last_line = line.split("s ", 1)[1]
            if flags_ok:
                ready = True
                break
        elif kind == "STATUSTEXT":
            print(f"  FC: {msg.text}")
    if not ready:
        print(f"EKF mask never satisfied within {READY_TIMEOUT_S} s")
        return 1

    # Ask for the parameter ArduCopter compares the variances against, so
    # the numbers above can be judged against the threshold actually in use.
    print("--- parameters ---")
    for name in ("FS_EKF_THRESH", "ARMING_CHECK", "EK3_SRC1_POSXY"):
        master.mav.param_request_read_send(
            master.target_system, master.target_component, name.encode(), -1,
        )
        # Match on the name: SITL also streams PARAM_VALUE for requests
        # that are still queued, and the first reply is not necessarily
        # the one just asked for. The first request of a session can also
        # be dropped while the autopilot is busy starting up, so the
        # timeout is reported rather than treated as "parameter missing".
        found = False
        end = time.time() + 5
        while time.time() < end:
            reply = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if reply is None:
                continue
            if reply.param_id == name:
                print(f"  {reply.param_id}: {reply.param_value:g}")
                found = True
                break
        if not found:
            print(f"  {name}: <no reply>")

    print("--- attempting arm ---")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0,
    )
    # Keep reading well past the COMMAND_ACK. ArduCopter sends the reason
    # as STATUSTEXT immediately after the ack, and mavlink_link.arm() used
    # to raise on the ack and drop that text, which is why the first
    # failure had no explanation anywhere.
    end = time.time() + 8
    while time.time() < end:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            print(f"  [{msg.severity}] {msg.text}")
        elif kind == "COMMAND_ACK":
            print(f"  ACK command={msg.command} result={msg.result}")
        elif kind == "EKF_STATUS_REPORT":
            print(f"  EKF flags=0x{msg.flags:04x} "
                  f"vel_var={msg.velocity_variance:.4f} "
                  f"pos_h_var={msg.pos_horiz_variance:.4f} "
                  f"pos_v_var={msg.pos_vert_variance:.4f}")

    # Leave the aircraft disarmed whatever happened, so the next run starts
    # from a known state.
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
