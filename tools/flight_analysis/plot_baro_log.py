#!/usr/bin/env python3
"""Plots pressure/temperature/altitude/climb-rate from an fc::DataLogger
barometer CSV capture (see docs/data-logger-usb.md for the log format and how
to capture one from SerialUSB1).

Run: (from this directory, using the project-local venv)
    .venv/Scripts/python.exe plot_baro_log.py "path/to/AKUSISI-BARO ....txt"

Requires: numpy, matplotlib (installed into tools/flight_analysis/.venv, see
requirements.txt -- NOT the system Python).
"""

import argparse
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from baro_log_io import load_baro_csv


def plot_baro_log(log_path, out_path=None, ema_alpha=0.08):
    data = load_baro_csv(log_path)
    t_s = data["t_s"]
    p = data["pressure_pa"]
    temp = data["temperature_c"]
    alt = data["altitude_m"]
    raw_alt = data["raw_altitude_m"]
    climb = data["climb_rate_mps"]

    if data["skipped_lines"]:
        print(f"Skipped {len(data['skipped_lines'])} unparseable line(s):")
        for line_no, line in data["skipped_lines"][:10]:
            print(f"  line {line_no}: {line!r}")

    n_panels = 4
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(t_s, p, lw=0.5, color="#2b6cb0")
    axes[0].axhline(p.mean(), color="#e53e3e", ls="--", lw=1,
                     label=f"mean={p.mean():.1f} Pa ({p.mean() / 100:.1f} hPa)")
    axes[0].set_ylabel("Pressure (Pa)")
    axes[0].set_title(f"Pressure: std={p.std():.2f} Pa, p2p={p.max() - p.min():.2f} Pa")
    axes[0].legend(fontsize=8, loc="best")

    axes[1].plot(t_s, temp, lw=0.8, color="#dd6b20")
    axes[1].set_ylabel("Temperature (C)")
    axes[1].set_title(f"Temperature: {temp[0]:.2f}C -> {temp[-1]:.2f}C over {t_s[-1]:.0f}s")

    if data["has_raw_altitude"]:
        axes[2].plot(t_s, raw_alt, lw=0.4, color="#cbd5e0",
                     label=f"raw (pre-Kalman), std={raw_alt.std():.3f} m")
    axes[2].plot(t_s, alt, lw=1.1, color="#2f855a",
                 label=f"Option 1: Kalman (baro-only), std={alt.std():.3f} m")
    if data["has_complementary"]:
        comp_alt = data["comp_altitude_m"]
        axes[2].plot(t_s, comp_alt, lw=1.1, color="#805ad5", ls="--",
                     label=f"Option 2: complementary (baro+accel), std={comp_alt.std():.3f} m")
    axes[2].axhline(0, color="black", lw=0.8, ls=":")
    axes[2].set_ylabel("Altitude (m)")
    axes[2].set_title(f"Altitude: bias={alt.mean():.3f} m, {alt[0]:.3f}m -> {alt[-1]:.3f}m over {t_s[-1]:.0f}s")
    axes[2].legend(fontsize=8, loc="best")

    axes[3].plot(t_s, climb, lw=0.4, color="#a0aec0")
    axes[3].axhline(0, color="black", lw=0.8, ls=":")
    axes[3].set_ylabel("Climb rate (m/s)")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_title(f"Climb rate: std={climb.std():.3f} m/s")

    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle(os.path.basename(log_path), fontsize=10, y=1.0)
    fig.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(log_path)[0] + "_plot.png"
    fig.savefig(out_path, dpi=140)
    print(f"Saved {out_path}")
    return out_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_path", help="Path to a DataLogger CSV/PuTTY-log capture")
    parser.add_argument("--out", default=None, help="Output PNG path (default: <log_path>_plot.png)")
    args = parser.parse_args()
    plot_baro_log(args.log_path, args.out)
