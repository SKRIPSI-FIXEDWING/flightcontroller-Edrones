"""Uji sdlog_io + analyze_sdlog_runs dengan file SINTETIS yang meniru format firmware.

Daftar kolom diambil langsung dari src/communication/FlightLog.cpp (kColumns),
jadi uji ini gagal bila pembaca dan firmware tidak sepakat soal kolom.

Jalankan:  .venv/Scripts/python.exe test_sdlog_io.py
"""
from __future__ import annotations

import math
import os
import re
import tempfile

import numpy as np

import analyze_sdlog_runs as an
from sdlog_io import read_sdlog

HERE = os.path.dirname(os.path.abspath(__file__))
FLIGHTLOG_CPP = os.path.join(HERE, "..", "..", "src", "communication", "FlightLog.cpp")


def firmware_columns() -> list[str]:
    src = open(FLIGHTLOG_CPP, encoding="utf-8").read()
    block = re.search(r"constexpr const char\* kColumns =(.*?);", src, re.S).group(1)
    text = "".join(re.findall(r'"(.*?)"', block)).replace("\\r\\n", "")
    return text.split(",")


def make_file(path: str) -> None:
    cols = firmware_columns()
    rng = np.random.default_rng(7)
    lines = [
        "# EDRONES_FLIGHT_LOG format=3 (docs/flight-log-csv.md)",
        "# fw_tag=TEST build=Oct 25 2026 12:00:00",
        "# file=LOG001.CSV boot_ms=1000 log_period_ms=50",
        "#SNAPSHOT,BOOT,1000",
        "# EFFECTIVE l1_damping=0.707 l1_xtrack_i=0.200 l1_period_now=22.00 wp_radius_m=15.0 roll_limit_deg=45.0",
        "# PARAM NAVL1_PERIOD=22 NAVL1_DAMPING=0.707 NAVL1_XTRACK_I=0.2",
        "# HOME,-70000000,1100000000,0",
        "# WP_FIELDS seq,lat_e7,lon_e7,alt_cm,radius_m(0=default WP_RADIUS)",
        "# WP,0,-70000000,1100000000,0,0.0",
        "# WP,1,-70010000,1100000000,6000,0.0",
        ",".join(cols),
    ]
    events = {
        5.0: "#EVENT,{ms},ARM",
        10.0: "#EVENT,{ms},MODE,AUTO",
        40.0: "#EVENT,{ms},DIST_ON,+250",
        43.0: "#EVENT,{ms},DIST_OFF",
        70.0: "#EVENT,{ms},WP_REACHED,1,radius",
        100.0: "#EVENT,{ms},MODE,MANU",
        110.0: "#EVENT,{ms},MODE,AUTO",
        150.0: "#EVENT,{ms},MODE,FBWA",
    }
    pending = sorted(events)
    tau = 2.0
    for k in range(int(160 * 20)):
        t = k * 0.05
        t_ms = int(1000 + t * 1000)
        while pending and pending[0] <= t:
            lines.append(events[pending.pop(0)].format(ms=t_ms))
        auto1 = 10.0 <= t < 100.0
        auto2 = 110.0 <= t < 150.0
        in_auto = auto1 or auto2
        cte = 0.0
        if auto1:
            if 40.0 <= t < 43.0:
                cte = 20.0 * (t - 40.0) / 3.0          # grows to 20 m during the disturbance
            elif t >= 43.0:
                cte = 20.0 * math.exp(-(t - 43.0) / tau)  # recovers: |cte| <= 5 m after 2*ln(4) = 2.77 s
        if auto1 and t < 15.0:
            cte = 30.0                                   # entry leg (leg_from_wp = 0): must be excluded
        cte += rng.normal(0, 0.2) if in_auto else 0.0
        row = {c: 0.0 for c in cols}
        row.update(
            time_boot_ms=t_ms, mode="AUTO" if in_auto else "MANU", armed=1 if t >= 5 else 0,
            gps_fix=2, gps_sats=12, leg_from_wp=1 if (in_auto and not (auto1 and t < 15.0)) else 0,
            wp_prev=0 if in_auto else -1, wp_next=1 if in_auto else -1,
            cte_m=cte if in_auto else 0.0, roll_deg=rng.normal(0, 3), nav_roll_deg=0.0,
            l1_period_s=14.0 if auto2 else 22.0, airspeed_mps=18.0,
            gs_mps=19.0, ch3=1500, dist_on=1 if (auto1 and 40 <= t < 43) else 0,
            dist_us=250 if (auto1 and 40 <= t < 43) else 0,
        )
        lines.append(",".join(str(row[c]) for c in cols))
    lines.append("1234,5,1,AU")  # truncated last row (power cut)
    with open(path, "w", newline="") as handle:
        handle.write("\r\n".join(lines) + "\r\n")


def check(cond: bool, message: str) -> None:
    if not cond:
        raise AssertionError(message)
    print("ok  ", message)


def main() -> None:
    cols = firmware_columns()
    check(len(cols) == 56, f"firmware kColumns has 56 columns (got {len(cols)})")

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "LOG001.CSV")
        make_file(path)
        log = read_sdlog(path)

        check(log.columns == cols, "reader columns == firmware columns")
        check(log.bad_rows == 1, "truncated last row counted as bad, not parsed")
        check(log.n_rows == 160 * 20, f"data rows parsed ({log.n_rows})")
        check(log.meta.get("fw_tag") == "TEST", "header key=value metadata")
        check(len(log.snapshots) == 1 and log.snapshots[0].tag == "BOOT", "boot snapshot found")
        check(log.snapshots[0].effective["roll_limit_deg"] == 45.0, "# EFFECTIVE parsed")
        check(log.snapshots[0].params["NAVL1_PERIOD"] == 22.0, "# PARAM parsed")
        check(len(log.snapshots[0].waypoints) == 2, "# WP parsed")
        kinds = [e.kind for e in log.events]
        check(kinds.count("MODE") == 4 and "DIST_ON" in kinds and "WP_REACHED" in kinds, "events parsed")
        check(np.allclose(log.cte_right_positive(), -log.col["cte_m"]), "cte sign helper")

        runs, dist_rows = an.analyze_log(log, band=5.0, hold_s=3.0, limit_m=45.0)
        check(len(runs) == 2, f"two AUTO runs found ({len(runs)})")
        r1, r2 = runs
        check(r1["fuzzy_on"] == 0 and r2["fuzzy_on"] == 0, "format 3 has no fuzzy_on column: every run is fixed L1")
        check(abs(r1["n_eval"] - (90 - 5) * 20) <= 2, f"entry leg (leg_from_wp=0) excluded ({r1['n_eval']} rows)")
        check(r1["wp_claims"] == "1:radius", "WP claim reason from event")
        check(r2["cte_abs_mean"] < 0.5 and r2["pct_within_limit"] == 100.0, "calm run metrics")
        check(r1["dist_windows"] == 1 and r1["dist_level_us"] == 250, "disturbance window paired")

        check(len(dist_rows) == 1, "one disturbance window analysed")
        d = dist_rows[0]
        expected = 2.0 * math.log(4.0)
        check(d["recovered"] and abs(d["recovery_s"] - expected) < 0.1,
              f"recovery time {d['recovery_s']:.2f} s ~ {expected:.2f} s")
        check(abs(d["peak_abs_cte"] - 20.0) < 1.5, f"peak |cte| {d['peak_abs_cte']:.1f} m ~ 20 m")
        check(d["overshoot_m"] < 1.0, f"no overshoot ({d['overshoot_m']:.2f} m)")

        an.compare_methods(runs, dist_rows, paired=False)  # must not raise

    # Statistics against known exact values.
    _, p = an.mann_whitney_u([1, 2, 3, 4, 5], [6, 7, 8, 9, 10])
    check(abs(p - 2 / 252) < 1e-9, f"Mann-Whitney exact p = 2/252 (got {p:.6f})")
    u, p = an.mann_whitney_u([1, 2, 3], [1, 2, 3])
    check(p == 1.0, "Mann-Whitney identical samples p = 1")
    _, p = an.wilcoxon_signed_rank([2, 3, 4, 5, 6, 7, 8, 9], [1, 1, 1, 1, 1, 1, 1, 1])
    check(abs(p - 2 / 256) < 1e-9, f"Wilcoxon exact p = 2/256 (got {p:.6f})")
    _, p = an.mann_whitney_u(np.arange(30), np.arange(30) + 40)  # normal approximation branch
    check(p < 1e-6, f"Mann-Whitney large samples separated (p={p:.2e})")

    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()
