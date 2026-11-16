#!/usr/bin/env python3
"""Grafik 13 (lingkaran loiter fase stabil) dari log SD, padanan analyze_tlog_loiter.py.

Dipakai bila telemetri tlog banyak putus (titik posisi jarang, lingkaran tampak bersegi).
Log SD merekam setiap fix GPS (gps_new = 1), jadi lintasannya rapat.

  pusat    WP loiter dari snapshot misi (# WP,...), default WP terakhir
  galat    wp_dist_m - R (jarak ke pusat dihitung FC), + = di luar lingkaran
  stabil   mulai putaran ke-2 tiap loiter (loiter_turn_index_runs, sama dengan analyze_fuzzy_sdlog.py)
  segmen   tiap rentang loiter berurutan = satu "terbang N", urutannya sama dengan grafik tlog

Keluaran: <out>/13_lingkaran_steady_terbang<N>_sd.png

Pemakaian:
    python plot_loiter_steady_sdlog.py LOG.CSV -o flight_.../terbang_.../loiter --radius 100 [--tol 10]
"""
import argparse
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_fuzzy_sdlog import loiter_turn_index_runs
from sdlog_io import read_sdlog


def ring(r, n=240):
    a = np.linspace(0, 2 * np.pi, n)
    return r * np.cos(a), r * np.sin(a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="file log SD (CSV)")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--radius", type=float, required=True, help="WP_LOITER_RAD [m]")
    ap.add_argument("--tol", type=float, default=10.0, help="toleransi galat radius [m]")
    ap.add_argument("--loiter-wp", type=int, default=None, help="nomor WP loiter (default: WP terakhir di misi)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    log = read_sdlog(args.log)
    c = log.col
    t = c["time_boot_ms"] / 1000.0
    wps = [s for s in log.snapshots if s.waypoints][-1].waypoints
    lw = args.loiter_wp if args.loiter_wp is not None else max(w["seq"] for w in wps)
    wp = next(w for w in wps if w["seq"] == lw)
    lat0, lon0 = wp["lat_e7"], wp["lon_e7"]
    R, tol = args.radius, args.tol
    params = log.effective_at(float(c["time_boot_ms"][-1]))
    period, damp = params.get("fuzzy_base_period", 14.0), params.get("l1_damping", 0.707)

    auto = np.array([m == "AUTO" for m in c["mode"]]) & (c["armed"] > 0) & (c["wp_next"] == lw)
    idx = np.flatnonzero(auto)
    if len(idx) == 0:
        raise SystemExit("tidak ada loiter di log ini")
    turn = loiter_turn_index_runs(c["lat_e7"], c["lon_e7"], idx)
    cuts = np.flatnonzero(np.diff(idx) > 1) + 1
    for k, (a, b) in enumerate(zip(np.r_[0, cuts], np.r_[cuts, len(idx)])):
        seg, tr = idx[a:b], turn[a:b]
        st = seg[tr >= 1]
        st = st[(c["gps_new"][st] > 0) & (c["gps_fix"][st] >= 2)]  # hanya fix baru: tanpa tangga 20 Hz
        if len(st) < 5:
            print("loiter %d kurang dari 1 putaran, dilewati" % (k + 1))
            continue
        y = (c["lat_e7"][st] - lat0) * 1e-7 * 111320.0
        x = (c["lon_e7"][st] - lon0) * 1e-7 * 111320.0 * math.cos(math.radians(lat0 * 1e-7))
        err = c["wp_dist_m"][st] - R
        rms, emax = float(np.sqrt(np.mean(err ** 2))), float(np.max(np.abs(err)))
        cw = np.mean(np.diff(np.unwrap(np.arctan2(y, x)))) < 0

        fig, ax = plt.subplots(figsize=(8.5, 8.2))
        ax.plot(*ring(R + tol), ":", color="navy", lw=1.8, label="Toleransi ±%.0f m" % tol)
        if R - tol > 0:
            ax.plot(*ring(R - tol), ":", color="navy", lw=1.8)
        ax.plot(*ring(R), "-", color="red", lw=1.6, label="Setpoint (lingkaran R=%.0f m)" % R)
        ax.plot(x, y, "-", color="black", lw=2.6, label="Lintasan UAV (loiter steady)")
        ax.plot(0, 0, "r+", ms=14, mew=2)
        ax.annotate("pusat", (0, 0), xytext=(6, -14), textcoords="offset points", color="red", fontsize=9)
        hr = math.radians(90.0 - float(c["course_deg"][st[0]]))
        fwd, side = np.array([math.cos(hr), math.sin(hr)]), np.array([-math.sin(hr), math.cos(hr)])
        p0 = np.array([x[0], y[0]])
        sz = max(R, 40.0) / 6.0
        tri = [p0 + fwd * sz, p0 - fwd * sz * 0.6 + side * sz * 0.65, p0 - fwd * sz * 0.6 - side * sz * 0.65]
        ax.add_patch(plt.Polygon(tri, closed=True, fc="darkorange", ec="k", zorder=6))
        ax.annotate("UAV", tuple(p0), xytext=(12, 10), textcoords="offset points", fontsize=12)
        i = int(np.argmax(np.abs(err)))
        ax.plot(x[i], y[i], "o", ms=9, mfc="none", mec="m", mew=2, zorder=6)
        ax.annotate("galat maks %.1f m" % abs(err[i]), (x[i], y[i]), xytext=(12, 2),
                    textcoords="offset points", color="m", fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlabel("Jarak sumbu-x / timur dari pusat (m)")
        ax.set_ylabel("Jarak sumbu-y / utara dari pusat (m)")
        ax.set_title("Loiter %d steady (mulai putaran ke-2) -- RMS %.1f m, maks %.1f m (%s, t_boot=%.0f-%.0f s, log SD)"
                     % (k + 1, rms, emax, "CW" if cw else "CCW", t[st[0]], t[st[-1]]), fontsize=10)
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=3, fontsize=9, frameon=False)
        ax.tick_params(direction="in")
        fig.text(0.99, 0.005, "NAVL1_PERIOD=%g s, NAVL1_DAMPING=%.3f" % (period, damp),
                 ha="right", va="bottom", fontsize=8, color="0.35")
        fig.tight_layout()
        path = os.path.join(args.out, "13_lingkaran_steady_terbang%d_sd.png" % (k + 1))
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print("loiter %d: %d titik GPS, RMS %.1f m, maks %.1f m -> %s" % (k + 1, len(st), rms, emax, path))


if __name__ == "__main__":
    main()
