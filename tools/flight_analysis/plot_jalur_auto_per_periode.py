#!/usr/bin/env python3
"""Gambar jalur penerbangan mode AUTO untuk tiap periode L1 (uji L1 tetap 30/10/2026).

Membaca folder hasil analyze_tlog_l1_auto.py (flight_2026-10-30_P11 ... P19):
  - cte_per_sampel.csv  -> posisi (x timur, y utara) selama AUTO
  - run.txt             -> koordinat waypoint (baris "misi:")
  - ringkasan_auto.csv  -> durasi AUTO dan L1 distance rata-rata

Keluaran (folder OUT):
  jalur_auto_P11.png ... jalur_auto_P19.png   satu gambar per periode
  jalur_auto_semua_periode.png                panel 2x3 + tumpang tindih lima periode

Judul memakai periode dari nama folder, BUKAN dari param di tlog, karena
analyze_tlog_l1_auto.py mengambil NAVL1_PERIOD pertama yang muncul di log
(bisa nilai lama sebelum diganti di lapangan).

Run:  python plot_jalur_auto_per_periode.py
"""
import ast
import csv
import math
import os
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "bab5_jalur_auto_L1_tetap_6wp")
PERIODS = [11, 13, 15, 17, 19]
TOL_M = 45.0
WP_RADIUS_M = 15.0
COLORS = {11: "#1b9e77", 13: "#d95f02", 15: "#000000", 17: "#7570b3", 19: "#e7298a"}
os.makedirs(OUT, exist_ok=True)


def load_run(p):
    d = os.path.join(HERE, "flight_2026-10-30_P%d" % p)
    with open(os.path.join(d, "run.txt"), encoding="utf8") as f:
        txt = f.read()
    m = re.search(r"misi: (\[.*\])", txt)
    s = re.sub(r"np\.float64\(([-\d.eE+]+)\)", r"\1", m.group(1))
    mission = [(i, x, y) for i, x, y, _ in ast.literal_eval(s)]
    xs, ys, legs = [], [], []
    with open(os.path.join(d, "cte_per_sampel.csv"), encoding="utf8") as f:
        for r in csv.DictReader(f):
            xs.append(float(r["x_timur_m"]))
            ys.append(float(r["y_utara_m"]))
            legs.append(int(r["wp_aktif"]))
    with open(os.path.join(d, "ringkasan_auto.csv"), encoding="utf8") as f:
        summ = next(csv.DictReader(f))
    return mission, np.array(xs), np.array(ys), summ


def corridor(ax, pts, tol):
    """Garis putus-putus sejajar tiap segmen WP1->WP2 ... pada jarak +-tol."""
    for a, b in zip(pts[1:], pts[2:]):
        a, b = np.array(a), np.array(b)
        d = b - a
        n = np.array([-d[1], d[0]]) / np.hypot(*d)
        for sgn in (1, -1):
            ax.plot(*zip(a + sgn * tol * n, b + sgn * tol * n), ":", color="navy", lw=1.1, zorder=1)


def draw(ax, p, mission, x, y, summ, title_extra=True):
    pts = [(m[1], m[2]) for m in mission]
    corridor(ax, pts, TOL_M)
    wx, wy = zip(*pts[1:])
    ax.plot(wx, wy, "-", color="red", lw=1.6, zorder=2, label="Jalur rencana (WP1–WP6)")
    for i, (px, py) in enumerate(pts[1:], 1):
        ax.add_patch(plt.Circle((px, py), WP_RADIUS_M, fill=False, ls=":", color="0.3", lw=0.9, zorder=2))
        ax.annotate("WP%d" % i, (px, py), xytext=(5, 5), textcoords="offset points", color="red", fontsize=9, weight="bold")
    ax.plot(x, y, "-", color=COLORS[p] if COLORS[p] != "#000000" else "k", lw=2.2, zorder=4, label="Lintasan UAV (AUTO)")
    ax.plot(x[0], y[0], "o", ms=8, mfc="white", mec="k", mew=1.6, zorder=5, label="Awal AUTO")
    ax.plot(x[-1], y[-1], "s", ms=8, color="k", zorder=5, label="Akhir AUTO")
    ax.plot(0, 0, "^", ms=9, color="k", zorder=5, label="HOME")
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("Timur dari HOME (m)")
    ax.set_ylabel("Utara dari HOME (m)")
    t = "Periode L1 = %d s" % p
    if title_extra:
        t += "  (L1 distance rata-rata %.0f m)" % float(summ["l1_mean"])
    ax.set_title(t, fontsize=11)


def main():
    runs = {p: load_run(p) for p in PERIODS}
    for p in PERIODS:
        mission, x, y, summ = runs[p]
        fig, ax = plt.subplots(figsize=(7.2, 7.6))
        draw(ax, p, mission, x, y, summ)
        ax.plot([], [], ":", color="navy", label="Toleransi ±45 m")
        ax.legend(loc="lower left", fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "jalur_auto_P%d.png" % p), dpi=170)
        plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(17, 11.5))
    axes = axes.ravel()
    for ax, p in zip(axes, PERIODS):
        mission, x, y, summ = runs[p]
        draw(ax, p, mission, x, y, summ)
        ax.set_xlabel("")
        ax.set_ylabel("")
    ax = axes[5]
    mission, *_ = runs[15]
    pts = [(m[1], m[2]) for m in mission]
    corridor(ax, pts, TOL_M)
    wx, wy = zip(*pts[1:])
    ax.plot(wx, wy, "-", color="red", lw=1.6, zorder=2)
    for i, (px, py) in enumerate(pts[1:], 1):
        ax.annotate("WP%d" % i, (px, py), xytext=(5, 5), textcoords="offset points", color="red", fontsize=9, weight="bold")
    for p in PERIODS:
        _, x, y, _ = runs[p]
        ax.plot(x, y, "-", color=COLORS[p], lw=1.5, label="%d s" % p, zorder=3)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title("Tumpang tindih kelima periode", fontsize=11)
    ax.legend(title="Periode L1", fontsize=8, loc="lower left")
    handles = [
        plt.Line2D([], [], color="red", lw=1.6),
        plt.Line2D([], [], color="navy", ls=":", lw=1.2),
        plt.Line2D([], [], color="k", lw=2.2),
        plt.Line2D([], [], marker="o", mfc="white", mec="k", ls="", ms=8),
        plt.Line2D([], [], marker="s", color="k", ls="", ms=8),
        plt.Line2D([], [], marker="^", color="k", ls="", ms=8),
    ]
    fig.legend(handles, ["Jalur rencana", "Toleransi ±45 m", "Lintasan UAV (AUTO)", "Awal AUTO", "Akhir AUTO", "HOME"],
               loc="lower center", ncol=6, fontsize=10)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(os.path.join(OUT, "jalur_auto_semua_periode.png"), dpi=140)
    plt.close(fig)
    print("selesai ->", OUT)


if __name__ == "__main__":
    main()
