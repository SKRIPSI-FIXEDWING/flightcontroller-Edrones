#!/usr/bin/env python3
"""Compares Option 1 (fc::Barometer's onboard Kalman filter, baro-only)
against Option 2 (fc::AltitudeComplementaryFilter, baro+BNO055 accel), from a
DataLogger CSV capture that has both (9-column format -- see
docs/altitude-complementary-filter.md and docs/data-logger-usb.md).

This does NOT decide which option is "better" -- that depends on what the
data actually shows for a given test (stationary bench vs. real vertical
motion). It reports the same statistics side by side so you can judge:
  - On a stationary bench: lower std is better (true altitude is constant).
  - On real vertical motion (a known step/ramp): lower lag AND acceptable
    noise both matter -- report both, do not just pick the smoother one.

Run: (from this directory, using the project-local venv)
    .venv/Scripts/python.exe compare_altitude_estimators.py "path/to/AKUSISI-BARO ....txt"
"""

import argparse
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from baro_log_io import load_baro_csv


def run(log_path, out_path=None):
    data = load_baro_csv(log_path)
    if not data["has_complementary"]:
        raise ValueError(
            f"{log_path} has no comp_altitude_m/comp_climb_rate_mps columns "
            "(log predates the Option 2 complementary filter, 9-column "
            "format). Capture a fresh log with the current firmware first."
        )

    t_s = data["t_s"]
    alt1 = data["altitude_m"]           # Option 1: Kalman
    climb1 = data["climb_rate_mps"]
    alt2 = data["comp_altitude_m"]      # Option 2: complementary
    climb2 = data["comp_climb_rate_mps"]

    print(f"N={len(t_s)} samples over {t_s[-1]:.1f}s\n")
    print(f"{'Metric':<28} {'Option 1 (Kalman)':>20} {'Option 2 (Complementary)':>26}")
    print(f"{'Altitude std (m)':<28} {alt1.std():>20.4f} {alt2.std():>26.4f}")
    print(f"{'Altitude bias/mean (m)':<28} {alt1.mean():>20.4f} {alt2.mean():>26.4f}")
    print(f"{'Altitude min/max (m)':<28} {f'{alt1.min():.2f}/{alt1.max():.2f}':>20} "
          f"{f'{alt2.min():.2f}/{alt2.max():.2f}':>26}")
    print(f"{'Climb rate std (m/s)':<28} {climb1.std():>20.4f} {climb2.std():>26.4f}")

    diff = alt2 - alt1
    print(f"\nOption 2 - Option 1 difference: mean={diff.mean():.4f} m, std={diff.std():.4f} m, "
          f"max abs={np.abs(diff).max():.4f} m")

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    axes[0].plot(t_s, alt1, lw=1.0, color="#2f855a", label=f"Option 1: Kalman (std={alt1.std():.3f} m)")
    axes[0].plot(t_s, alt2, lw=1.0, color="#805ad5", ls="--", label=f"Option 2: complementary (std={alt2.std():.3f} m)")
    axes[0].axhline(0, color="black", lw=0.6, ls=":")
    axes[0].set_ylabel("Altitude (m)")
    axes[0].set_title(f"{os.path.basename(log_path)} -- altitude estimate comparison")
    axes[0].legend(fontsize=8)

    axes[1].plot(t_s, climb1, lw=0.5, color="#2f855a", alpha=0.7, label=f"Option 1 (std={climb1.std():.3f} m/s)")
    axes[1].plot(t_s, climb2, lw=0.5, color="#805ad5", alpha=0.7, label=f"Option 2 (std={climb2.std():.3f} m/s)")
    axes[1].axhline(0, color="black", lw=0.6, ls=":")
    axes[1].set_ylabel("Climb rate (m/s)")
    axes[1].legend(fontsize=8)

    axes[2].plot(t_s, diff, lw=0.6, color="#dd6b20")
    axes[2].axhline(0, color="black", lw=0.6, ls=":")
    axes[2].set_ylabel("Option 2 - Option 1 (m)")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title(f"Difference: mean={diff.mean():.4f} m, std={diff.std():.4f} m")

    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(log_path)[0] + "_compare.png"
    fig.savefig(out_path, dpi=140)
    print(f"\nSaved {out_path}")
    return out_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log_path", help="Path to a 9-column DataLogger CSV capture")
    parser.add_argument("--out", default=None, help="Output PNG path (default: <log_path>_compare.png)")
    args = parser.parse_args()
    run(args.log_path, args.out)
