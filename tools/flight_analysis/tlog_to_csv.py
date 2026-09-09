#!/usr/bin/env python3
"""Converts one Mission Planner .tlog into a SINGLE merged CSV -- so a flight
can be pulled straight into Excel/pandas for analysis without asking Claude
to hand-write a parsing script for every new log (see docs/attitude-lqr.md's
sibling doc for context, or just: this replaces the one-off scratch scripts
from the 2026-08-24 flight-log analysis session).

One row per ATTITUDE sample (usually the highest-rate message in a tlog);
every other message type is nearest-timestamp-joined onto that row. An
`airborne` column (relative_alt > 5 m) is included so you can filter to just
the flying portion in Excel/pandas without re-deriving that threshold.

Run: (from this directory, using the project-local venv)
    .venv/Scripts/python.exe tlog_to_csv.py "path/to/2026-08-23 17-29-36.tlog"

Or with no argument at all, to auto-pick the MOST RECENTLY MODIFIED .tlog
under Mission Planner's logs folder (prints which file it picked -- always
double check this against what you actually just flew; log folders often
have several short throwaway sessions from connection tests mixed in with
the real flight, see the 2026-08-24 session's mixup with a bench-test tlog):
    .venv/Scripts/python.exe tlog_to_csv.py

Output: <tlog_name>.csv next to the source .tlog, unless -o/--output is given.

Requires: pymavlink (already installed into this directory's .venv). Deliberately
NOT using pandas here -- stdlib csv is enough for this and keeps the venv small.
"""

import argparse
import csv
import glob
import math
import os
import sys
from collections import defaultdict

from pymavlink import mavutil

MODE_MAP = {0: "MANUAL", 5: "FBWA", 10: "AUTO", 4: "GUIDED"}
ARMED_BIT = 128  # MAV_MODE_FLAG_SAFETY_ARMED

DEFAULT_LOG_ROOTS = [
    os.path.expandvars(r"%USERPROFILE%\Documents\Mission Planner\logs"),
]


def find_latest_tlog():
    candidates = []
    for root in DEFAULT_LOG_ROOTS:
        candidates.extend(glob.glob(os.path.join(root, "**", "*.tlog"), recursive=True))
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def nearest(rows_by_t, t):
    """rows_by_t: list of (t_rel_s, dict). Returns the dict closest to t, or
    None if the list is empty. Linear scan is fine here -- a few thousand
    rows per message type, once per output row, still finishes in seconds."""
    if not rows_by_t:
        return None
    return min(rows_by_t, key=lambda item: abs(item[0] - t))[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("tlog", nargs="?", help="Path to a .tlog. Omit to auto-pick the most recently modified one.")
    parser.add_argument("-o", "--output", help="Output CSV path. Defaults to <tlog>.csv next to the source file.")
    args = parser.parse_args()

    tlog_path = args.tlog
    if tlog_path is None:
        tlog_path = find_latest_tlog()
        if tlog_path is None:
            print("No .tlog given and none found under:", DEFAULT_LOG_ROOTS, file=sys.stderr)
            sys.exit(1)
        print(f"[auto] No path given -- using most recently modified .tlog:\n  {tlog_path}")
        print("[auto] Double-check this is really the flight you mean (short bench-test tlogs count too).")

    out_path = args.output or (os.path.splitext(tlog_path)[0] + ".csv")

    mlog = mavutil.mavlink_connection(tlog_path, dialect="common")
    rows_by_type = defaultdict(list)  # type -> list of (t_rel_s, dict)
    first_ts = None

    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            continue
        t = msg._timestamp
        if first_ts is None:
            first_ts = t
        rows_by_type[mtype].append((t - first_ts, msg.to_dict()))

    att = rows_by_type.get("ATTITUDE", [])
    if not att:
        print("No ATTITUDE messages in this tlog -- nothing to export.", file=sys.stderr)
        sys.exit(1)

    hb = rows_by_type.get("HEARTBEAT", [])
    rc = rows_by_type.get("RC_CHANNELS_RAW", [])
    servo = rows_by_type.get("SERVO_OUTPUT_RAW", [])
    vfr = rows_by_type.get("VFR_HUD", [])
    gpos = rows_by_type.get("GLOBAL_POSITION_INT", [])
    batt = rows_by_type.get("BATTERY_STATUS", [])
    nvf = rows_by_type.get("NAMED_VALUE_FLOAT", [])

    # NAMED_VALUE_FLOAT carries MHN_ROLL/MHN_PITCH/MHN_YAW (Mahony filter
    # compare, see docs/attitude-mahony-filter.md) -- split by name so each
    # can be nearest-joined independently, same as any other message type.
    nvf_by_name = defaultdict(list)
    for t, d in nvf:
        name = d.get("name", "")
        if isinstance(name, (bytes, bytearray)):
            name = name.decode(errors="replace")
        name = name.rstrip("\x00")
        nvf_by_name[name].append((t, d))

    fields = [
        "t_rel_s", "mode", "armed", "airborne",
        "roll_deg", "pitch_deg", "yaw_deg",
        "rollrate_dps", "pitchrate_dps", "yawrate_dps",
        "mahony_roll_deg", "mahony_pitch_deg", "mahony_yaw_deg",
        "ch1_roll_pwm", "ch2_pitch_pwm", "ch3_throttle_pwm", "ch4_yaw_pwm",
        "ch5_pwm", "ch6_pwm", "ch7_pwm", "ch8_pwm",
        "servo1_ail_pwm", "servo2_ele_pwm", "servo3_thr_pwm", "servo4_rud_pwm",
        "throttle_pct", "airspeed_mps", "groundspeed_mps", "heading_deg", "climb_mps",
        "alt_rel_m", "alt_abs_m", "lat_deg", "lon_deg",
        "battery_voltage_v", "battery_pct",
    ]

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for t, a in att:
            hb_d = nearest(hb, t) or {}
            rc_d = nearest(rc, t) or {}
            sv_d = nearest(servo, t) or {}
            vfr_d = nearest(vfr, t) or {}
            gp_d = nearest(gpos, t) or {}
            bt_d = nearest(batt, t) or {}
            mr = nearest(nvf_by_name.get("MHN_ROLL", []), t) or {}
            mp = nearest(nvf_by_name.get("MHN_PITCH", []), t) or {}
            my = nearest(nvf_by_name.get("MHN_YAW", []), t) or {}

            custom_mode = hb_d.get("custom_mode", -1)
            base_mode = hb_d.get("base_mode", 0)
            armed = 1 if (base_mode & ARMED_BIT) else 0
            mode = MODE_MAP.get(custom_mode, f"UNK({custom_mode})")
            alt_rel_m = gp_d.get("relative_alt", 0) / 1000.0
            airborne = 1 if alt_rel_m > 5.0 else 0

            row = [
                f"{t:.3f}", mode, armed, airborne,
                f"{math.degrees(a.get('roll', 0.0)):.3f}",
                f"{math.degrees(a.get('pitch', 0.0)):.3f}",
                f"{math.degrees(a.get('yaw', 0.0)):.3f}",
                f"{math.degrees(a.get('rollspeed', 0.0)):.3f}",
                f"{math.degrees(a.get('pitchspeed', 0.0)):.3f}",
                f"{math.degrees(a.get('yawspeed', 0.0)):.3f}",
                mr.get("value", ""), mp.get("value", ""), my.get("value", ""),
                rc_d.get("chan1_raw", ""), rc_d.get("chan2_raw", ""), rc_d.get("chan3_raw", ""),
                rc_d.get("chan4_raw", ""), rc_d.get("chan5_raw", ""), rc_d.get("chan6_raw", ""),
                rc_d.get("chan7_raw", ""), rc_d.get("chan8_raw", ""),
                sv_d.get("servo1_raw", ""), sv_d.get("servo2_raw", ""), sv_d.get("servo3_raw", ""),
                sv_d.get("servo4_raw", ""),
                vfr_d.get("throttle", ""), vfr_d.get("airspeed", ""), vfr_d.get("groundspeed", ""),
                vfr_d.get("heading", ""), vfr_d.get("climb", ""),
                f"{alt_rel_m:.2f}", f"{gp_d.get('alt', 0) / 1000.0:.2f}",
                f"{gp_d.get('lat', 0) / 1e7:.7f}" if gp_d.get("lat") else "",
                f"{gp_d.get('lon', 0) / 1e7:.7f}" if gp_d.get("lon") else "",
                bt_d.get("voltages", [None])[0] / 1000.0 if bt_d.get("voltages") else "",
                bt_d.get("battery_remaining", ""),
            ]
            w.writerow(row)

    print(f"[ok] {out_path} ({len(att)} rows)")


if __name__ == "__main__":
    main()
