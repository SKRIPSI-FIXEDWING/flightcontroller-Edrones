#!/usr/bin/env python3
"""Grafik CTE terhadap waktu mode AUTO untuk tiap periode L1 (uji L1 tetap 30/10/2026).

Sumber: flight_2026-10-30_P*/cte_per_sampel.csv (hasil analyze_tlog_l1_auto.py).
Judul memakai periode dari nama folder, bukan dari param di tlog.
Leg pertama (entry->WP1) diarsir abu-abu karena tidak dievaluasi.
Garis vertikal tipis = pergantian waypoint aktif.

Keluaran di bab5_jalur_auto_L1_tetap_6wp/:
  cte_waktu_P11.png ... cte_waktu_P19.png, cte_waktu_semua_periode.png
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "bab5_jalur_auto_L1_tetap_6wp")
PERIODS = [11, 13, 15, 17, 19]
TOL = 45.0
os.makedirs(OUT, exist_ok=True)


def load(p):
    t, e, wp = [], [], []
    with open(os.path.join(HERE, "flight_2026-10-30_P%d" % p, "cte_per_sampel.csv"), encoding="utf8") as f:
        for r in csv.DictReader(f):
            t.append(float(r["t_s"]))
            e.append(float(r["cte_m(+kanan)"]))
            wp.append(int(r["wp_aktif"]))
    t = np.array(t)
    return t - t[0], np.array(e), np.array(wp)


def draw(ax, p, t, e, wp):
    ax.axhspan(-TOL, TOL, color="#e8f0ff", zorder=0)
    ax.axhline(TOL, color="navy", ls=":", lw=1.1)
    ax.axhline(-TOL, color="navy", ls=":", lw=1.1)
    ax.axhline(0, color="0.4", lw=0.8)
    first = wp == wp[0]
    t_entry_end = t[first][-1]
    ax.axvspan(0, t_entry_end, color="0.85", zorder=1)
    ax.text(t_entry_end / 2, 53, "leg masuk\n(tidak dievaluasi)", ha="center", va="top", fontsize=8, color="0.3")
    for i in np.where(np.diff(wp) != 0)[0] + 1:
        ax.axvline(t[i], color="0.6", lw=0.7, ls="--", zorder=1)
        ax.text(t[i], -62, "WP%d" % wp[i], fontsize=8, ha="center", color="0.3")
    ax.plot(t, e, "-", color="k", lw=1.6, zorder=3)
    m = ~first
    rms = float(np.sqrt(np.mean(e[m] ** 2)))
    ax.set_title("Periode L1 = %d s   (RMS leg dievaluasi %.1f m, |e| maks %.1f m)" % (p, rms, np.abs(e[m]).max()), fontsize=10.5)
    ax.set_ylim(-68, 58)
    ax.set_ylabel("CTE (m), + = kanan garis")
    ax.grid(alpha=0.25)


def main():
    data = {p: load(p) for p in PERIODS}
    for p in PERIODS:
        fig, ax = plt.subplots(figsize=(9, 3.8))
        draw(ax, p, *data[p])
        ax.set_xlabel("Waktu sejak mode AUTO aktif (s)")
        ax.plot([], [], ":", color="navy", label="Toleransi ±%d m" % TOL)
        ax.legend(loc="lower right", fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "cte_waktu_P%d.png" % p), dpi=170)
        plt.close(fig)

    fig, axes = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
    for ax, p in zip(axes, PERIODS):
        draw(ax, p, *data[p])
    axes[-1].set_xlabel("Waktu sejak mode AUTO aktif (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "cte_waktu_semua_periode.png"), dpi=130)
    plt.close(fig)
    print("selesai ->", OUT)


if __name__ == "__main__":
    main()
