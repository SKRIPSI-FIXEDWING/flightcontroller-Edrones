"""Shared CSV loader for fc::DataLogger's barometer output (see
docs/data-logger-usb.md). Used by plot_baro_log.py, kalman_tune.py, and
compare_altitude_estimators.py so all three tools parse the exact same log
formats the same way.

Handles, on top of a clean CSV:
  - the PuTTY session-start banner line ("=~=~=...")
  - the CSV header line itself, if present ("seq,timestamp_us,...")
  - a corrupted leading character on an otherwise-valid row (observed once in
    the field: "c5874,..." -- a stray byte from a USB CDC hiccup)
  - a truncated final line if the capture was stopped mid-row
  - the 6-column format (before raw_altitude_m existed), the 7-column format
    (before comp_altitude_m/comp_climb_rate_mps existed), the 9-column format
    (before comp_accel_up_mss existed), and the current 10-column format
    (Option 1 Kalman + Option 2 complementary filter + its accel input, see
    docs/altitude-complementary-filter.md)
"""

import re

import numpy as np

_ROW_RE = re.compile(r"^[^\d\-]*(-?\d.*)$")


def load_baro_csv(path):
    """Returns a dict of 1D numpy arrays: seq, t_us, t_s, pressure_pa,
    temperature_c, altitude_m, raw_altitude_m, climb_rate_mps,
    comp_altitude_m, comp_climb_rate_mps, comp_accel_up_mss.

    raw_altitude_m is NaN for rows/files that predate that column (6-column
    format). comp_altitude_m/comp_climb_rate_mps are NaN for files that
    predate the Option 2 complementary filter (6- or 7-column format).
    comp_accel_up_mss is additionally NaN for the 9-column format (before the
    accel diagnostic column existed).
    """
    rows = []
    skipped = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith("=~=") or line.startswith("seq,"):
                continue
            m = _ROW_RE.match(line)
            if not m:
                skipped.append((line_no, line))
                continue
            parts = m.group(1).split(",")
            if len(parts) not in (6, 7, 9, 10):
                skipped.append((line_no, line))
                continue
            try:
                vals = [float(x) for x in parts]
            except ValueError:
                skipped.append((line_no, line))
                continue
            if len(vals) == 6:
                seq, t_us, p, temp, alt, climb = vals
                raw_alt = comp_alt = comp_climb = comp_accel = float("nan")
            elif len(vals) == 7:
                seq, t_us, p, temp, alt, raw_alt, climb = vals
                comp_alt = comp_climb = comp_accel = float("nan")
            elif len(vals) == 9:
                seq, t_us, p, temp, alt, raw_alt, climb, comp_alt, comp_climb = vals
                comp_accel = float("nan")
            else:
                seq, t_us, p, temp, alt, raw_alt, climb, comp_alt, comp_climb, comp_accel = vals
            rows.append((seq, t_us, p, temp, alt, raw_alt, climb, comp_alt, comp_climb, comp_accel))

    if not rows:
        raise ValueError(f"No parseable data rows found in {path}")

    arr = np.array(rows, dtype=float)
    seq, t_us, p, temp, alt, raw_alt, climb, comp_alt, comp_climb, comp_accel = arr.T
    t_s = (t_us - t_us[0]) / 1e6

    return {
        "seq": seq,
        "t_us": t_us,
        "t_s": t_s,
        "pressure_pa": p,
        "temperature_c": temp,
        "altitude_m": alt,
        "raw_altitude_m": raw_alt,
        "climb_rate_mps": climb,
        "comp_altitude_m": comp_alt,
        "comp_climb_rate_mps": comp_climb,
        "comp_accel_up_mss": comp_accel,
        "skipped_lines": skipped,
        "has_raw_altitude": not np.all(np.isnan(raw_alt)),
        "has_complementary": not np.all(np.isnan(comp_alt)),
        "has_accel_diagnostic": not np.all(np.isnan(comp_accel)),
    }
