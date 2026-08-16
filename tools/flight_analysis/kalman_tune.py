#!/usr/bin/env python3
"""Sweeps the barometer Kalman filter's measurement_noise_r (and, optionally,
process_noise_q) against a real raw_altitude_m capture, to check whether the
firmware's current tuning is actually near the noise-minimizing point --
rather than guessing from "R should equal measured noise variance" (that
textbook rule is for statistical optimality under a random-walk model, NOT
for minimizing output noise -- the two are not the same thing for this
filter; see docs/barometer-ms5611.md for how this was learned the hard way).

The simulated filter recurrence is an exact copy of
lib/math/Filter/KalmanFilter1D.cpp -- keep the two in sync if that file
changes.

Where to actually change the tuning in firmware: BarometerConfig in
include/drivers/Barometer.h --
    float kalman_measurement_noise_r = 5.0f;   // R
    float kalman_process_noise_q = 0.1f;       // Q
    float kalman_initial_estimate_error_p = 1.0f;
No other file needs to change -- fc::Barometer::begin() reads these via
config_ and calls altitude_kalman_.configure(...) with them.

Requires a log captured AFTER the raw_altitude_m column was added (7-column
DataLogger CSV, see docs/data-logger-usb.md) -- the sweep needs the
pre-Kalman signal to test candidate filters against.

Run: (from this directory, using the project-local venv)
    .venv/Scripts/python.exe kalman_tune.py "path/to/AKUSISI-BARO ....txt"
"""

import argparse
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from baro_log_io import load_baro_csv

# Mirrors BarometerConfig's defaults in include/drivers/Barometer.h.
FIRMWARE_DEFAULT_R = 5.0
FIRMWARE_DEFAULT_Q = 0.1
FIRMWARE_DEFAULT_P0 = 1.0


def kf_sim(z, r, q, p0=FIRMWARE_DEFAULT_P0):
    """Exact port of KalmanFilter1D::update()'s recurrence."""
    est = 0.0
    err = p0
    out = np.empty_like(z)
    for i, m in enumerate(z):
        pred_err = err + q
        gain = pred_err / (pred_err + r)
        est = est + gain * (m - est)
        err = (1.0 - gain) * pred_err
        out[i] = est
    return out


def step_response_settle_time(r, q, dt_s, noise_std, step_m=1.0, n_pre=300, n_post=300, seed=0):
    """Synthetic step-response check: how long (s) until the filtered output
    sustains >=90% of a step_m altitude change, given noise at noise_std.
    A proxy for how much lag a given (R, Q) trades off against noise
    reduction -- this bench log alone (stationary) can't show that trade-off.
    """
    rng = np.random.default_rng(seed)
    sig = np.concatenate([np.zeros(n_pre), np.full(n_post, step_m)])
    noisy = sig + rng.normal(0, noise_std, len(sig))
    filt = kf_sim(noisy, r, q)
    post = filt[n_pre:]
    target = 0.9 * step_m
    for i in range(len(post) - 10):
        if np.all(post[i:i + 10] >= target):
            return i * dt_s
    return float("nan")


def run(log_path, r_default=FIRMWARE_DEFAULT_R, q_default=FIRMWARE_DEFAULT_Q,
        r_min=0.1, r_max=200.0, out_path=None):
    data = load_baro_csv(log_path)
    if not data["has_raw_altitude"]:
        raise ValueError(
            f"{log_path} has no raw_altitude_m column (old 6-column log, "
            "predates the raw-altitude logging change). Capture a fresh log "
            "with the current firmware first -- see docs/data-logger-usb.md."
        )
    raw_alt = data["raw_altitude_m"]
    dt_s = float(np.median(np.diff(data["t_us"]))) / 1e6

    r_range = np.logspace(np.log10(r_min), np.log10(r_max), 60)
    stds = np.array([kf_sim(raw_alt, r, q_default).std() for r in r_range])
    best_idx = int(np.argmin(stds))
    best_r = r_range[best_idx]
    default_std = kf_sim(raw_alt, r_default, q_default).std()

    print(f"Raw altitude std (pre-Kalman): {raw_alt.std():.4f} m")
    print(f"Firmware default R={r_default}, Q={q_default}: filtered std={default_std:.4f} m")
    print(f"Sweep minimum at R={best_r:.2f} (Q={q_default} fixed): filtered std={stds[best_idx]:.4f} m "
          f"({(1 - stds[best_idx] / default_std) * 100:+.1f}% vs default)")

    noise_std = float(raw_alt.std())
    print(f"\nStep-response settle time to 90% of a 1m step (noise_std={noise_std:.3f} m/dt={dt_s * 1000:.0f}ms):")
    for r, q, label in [
        (r_default, q_default, "firmware default"),
        (best_r, q_default, "sweep minimum"),
        (r_default * 0.4, q_default, "lower R (faster?)"),
        (r_default * 2.5, q_default, "higher R (smoother?)"),
    ]:
        t_settle = step_response_settle_time(r, q, dt_s, noise_std)
        print(f"  R={r:7.2f} Q={q:.3f}  settle={t_settle:.2f}s  ({label})")

    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(r_range, stds, color="#2b6cb0", lw=1.8)
    ax.set_xscale("log")
    ax.axvline(r_default, color="#2f855a", ls="--", lw=1.5,
               label=f"firmware default R={r_default:g} (std {default_std:.4f} m)")
    ax.scatter([r_default], [default_std], color="#2f855a", zorder=5)
    ax.axvline(best_r, color="#e53e3e", ls=":", lw=1.2,
               label=f"sweep minimum R={best_r:.1f} (std {stds[best_idx]:.4f} m)")
    ax.set_xlabel(f"Kalman measurement_noise_r (m^2), Q fixed at {q_default:g}")
    ax.set_ylabel("Filtered altitude std (m)")
    ax.set_title(f"R sweep on {os.path.basename(log_path)}\n"
                 "Lower R = filter trusts each sample more = LESS smoothing (higher std) -- "
                 "the curve's minimum, not R=0, is the best noise point.")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(log_path)[0] + "_kalman_sweep.png"
    fig.savefig(out_path, dpi=140)
    print(f"\nSaved {out_path}")
    return out_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log_path", help="Path to a DataLogger CSV capture (must have raw_altitude_m)")
    parser.add_argument("--r-default", type=float, default=FIRMWARE_DEFAULT_R, help="Current firmware R to mark on the plot")
    parser.add_argument("--q-default", type=float, default=FIRMWARE_DEFAULT_Q, help="Q to hold fixed while sweeping R")
    parser.add_argument("--r-min", type=float, default=0.1)
    parser.add_argument("--r-max", type=float, default=200.0)
    parser.add_argument("--out", default=None, help="Output PNG path (default: <log_path>_kalman_sweep.png)")
    args = parser.parse_args()
    run(args.log_path, args.r_default, args.q_default, args.r_min, args.r_max, args.out)
