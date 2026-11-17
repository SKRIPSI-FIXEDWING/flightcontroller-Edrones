"""Pembanding loiter R80 TANPA gangguan: satu putaran fase stabil sebelum pulsa pertama, L1 tetap vs L1 + fuzzy.

Pakai (dari tools/flight_analysis):
    python3 flight_2026-11-17/skrip/banding_putaran_bebas.py -o <folder> --radius 80 \
        "LABEL|LOG.CSV|t0|t1|utama" "LABEL|LOG.CSV|t0|t1|tambahan" ...
t0/t1 = t_boot [s] awal dan akhir putaran (batas putaran dari loiter_turn_index seluruh loiter, lihat ringkasan).
Kolom terakhir: "utama" = dipakai di grafik pembanding, "tambahan" = hanya masuk tabel (cek konsistensi).

galat e = wp_dist_m - R (+ = di luar orbit). Pusat = WP loiter dari misi di log SD.
Keluaran: banding_putaran_bebas.csv, 01_orbit_putaran_bebas.png, 02_galat_period_roll_vs_sudut.png
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_kejadian_toleransi import find_events, fit_wind, local_xy
from sdlog_io import read_sdlog


def load(spec, R, tol, bank):
    label, path, t0, t1, peran = spec.split("|")
    t0, t1 = float(t0), float(t1)
    log = read_sdlog(path)
    c = log.col
    t = c["time_boot_ms"] / 1000.0
    snaps = [s for s in log.snapshots if s.waypoints]
    lw = max(w["seq"] for w in snaps[-1].waypoints)
    cwp = next(w for w in snaps[-1].waypoints if w["seq"] == lw)
    m = (t >= t0) & (t <= t1) & (c["wp_next"] == lw) & np.array([x == "AUTO" for x in c["mode"]])
    i = np.flatnonzero(m)
    pulsa = [e.t_s for e in log.events_of("DIST_ON") if t0 <= e.t_s <= t1]
    eff = log.effective_at(float(c["time_boot_ms"][i[0]]))
    fz = bool(c["fuzzy_on"][i].max() > 0)
    e = c["wp_dist_m"][i] - R
    X, Y = local_xy(c["lat_e7"][i], c["lon_e7"][i], cwp["lat_e7"], cwp["lon_e7"])
    ang = np.unwrap(np.arctan2(Y, X))
    sudut = np.degrees(np.abs(ang - ang[0]))
    we, wn, tas = fit_wind(c["gs_mps"][i], c["course_deg"][i])
    ev = find_events(t[i], e, tol, 1.0)
    per = c["l1_period_s"][i]
    d = dict(
        label=label, peran=peran, metode="L1 + fuzzy" if fz else "L1 tetap", log=os.path.basename(path),
        t_boot=f"{t0:.1f}-{t1:.1f}", durasi_s=round(t1 - t0, 1), putaran=round(sudut[-1] / 360, 2),
        pulsa_di_jendela=len(pulsa),
        angin_mps=round(math.hypot(we, wn), 1), angin_dari_deg=round((math.degrees(math.atan2(we, wn)) + 180) % 360),
        galat_min_m=round(float(e.min()), 1), galat_maks_m=round(float(e.max()), 1),
        galat_rata_m=round(float(e.mean()), 1), galat_sd_m=round(float(e.std()), 1),
        galat_rms_m=round(float(np.sqrt((e ** 2).mean())), 1),
        kejadian_luar_tol=len(ev),
        period_min_s=round(float(per.min()), 2), period_maks_s=round(float(per.max()), 2),
        period_rata_s=round(float(per.mean()), 2), period_dasar_s=eff.get("fuzzy_base_period", 14.0),
        nav_roll_min_deg=round(float(c["nav_roll_deg"][i].min()), 1),
        nav_roll_maks_deg=round(float(c["nav_roll_deg"][i].max()), 1),
        roll_jenuh_pct=round(100 * float((np.abs(c["nav_roll_deg"][i]) >= bank - 0.5).mean())),
        gs_rata_mps=round(float(c["gs_mps"][i].mean()), 1), gs_min_mps=round(float(c["gs_mps"][i].min()), 1),
        gs_maks_mps=round(float(c["gs_mps"][i].max()), 1),
        airspeed_rata_mps=round(float(c["airspeed_mps"][i].mean()), 1), tas_fit_mps=round(float(tas), 1),
        throttle_ch3_us=round(float(np.median(c["ch3"][i]))),
        alt_min_m=round(float(c["alt_fused_m"][i].min()), 1), alt_maks_m=round(float(c["alt_fused_m"][i].max()), 1),
    )
    arr = dict(t=t[i] - t[i][0], e=e, X=X, Y=Y, sudut=sudut, per=per, nav_roll=c["nav_roll_deg"][i],
               gs=c["gs_mps"][i], wind=(we, wn))
    return d, arr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("spec", nargs="+")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--radius", type=float, required=True)
    ap.add_argument("--tol", type=float, default=10.0)
    ap.add_argument("--bank", type=float, default=45.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows, arrs = [], []
    for s in a.spec:
        d, arr = load(s, a.radius, a.tol, a.bank)
        rows.append(d)
        arrs.append(arr)
        print(d)
    with open(os.path.join(a.out, "banding_putaran_bebas.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    utama = [(d, x) for d, x in zip(rows, arrs) if d["peran"] == "utama"]
    warna = {"L1 tetap": "tab:blue", "L1 + fuzzy": "tab:green"}
    R, tol = a.radius, a.tol
    th = np.linspace(0, 2 * np.pi, 361)

    fig, axs = plt.subplots(1, len(utama), figsize=(6.2 * len(utama), 6.2))
    axs = np.atleast_1d(axs)
    for ax, (d, x) in zip(axs, utama):
        ax.plot(R * np.cos(th), R * np.sin(th), color="red", lw=1.8, label=f"setpoint R = {R:.0f} m")
        for r in (R - tol, R + tol):
            ax.plot(r * np.cos(th), r * np.sin(th), "k--", lw=0.9)
        ax.plot([], [], "k--", lw=0.9, label=f"toleransi ±{tol:.0f} m")
        ax.plot(x["X"], x["Y"], color=warna[d["metode"]], lw=2, label="lintasan (1 putaran)")
        ax.plot(x["X"][0], x["Y"][0], "o", color="k", ms=6, label="awal putaran")
        we, wn = x["wind"]
        wm = math.hypot(we, wn)
        if wm > 0.2:
            k = 12 / wm  # panah 24 m, berpusat di pojok kiri atas, menunjuk ke arah angin bertiup
            ox, oy = -R * 1.15, R * 1.2
            ax.annotate("", xy=(ox + we * k, oy + wn * k), xytext=(ox - we * k, oy - wn * k),
                        arrowprops=dict(arrowstyle="->", color="0.3", lw=1.5))
        ax.text(-R * 1.38, R * 1.0, f"angin {wm:.1f} m/s", fontsize=8, color="0.3")
        per = (f"NAVL1_PERIOD = {d['period_dasar_s']:.0f} s" if d["metode"] == "L1 tetap" else
               f"NAVL1_PERIOD dasar {d['period_dasar_s']:.0f} s, fuzzy {d['period_min_s']:.1f}–{d['period_maks_s']:.1f} s")
        ax.set_title(f"{d['metode']} ({d['label']}), tanpa gangguan\n{per}\n"
                     f"galat {d['galat_min_m']:+.1f} … {d['galat_maks_m']:+.1f} m, keluar ±{tol:.0f} m: "
                     f"{d['kejadian_luar_tol']} kejadian", fontsize=9.5)
        ax.set_aspect("equal")
        lim = R * 1.45
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_xlabel("timur [m]")
        ax.set_ylabel("utara [m]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7.5, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "01_orbit_putaran_bebas.png"), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for d, x in utama:
        col = warna[d["metode"]]
        lab = f"{d['metode']} ({d['label']})"
        ax[0].plot(x["sudut"], x["e"], color=col, lw=1.6, label=lab)
        ax[1].plot(x["sudut"], x["per"], color=col, lw=1.6, label=lab)
        ax[2].plot(x["sudut"], x["nav_roll"], color=col, lw=1.4, label=lab)
    ax[0].axhline(0, color="red", lw=1.6, label="setpoint (galat 0)")
    for v in (-tol, tol):
        ax[0].axhline(v, color="k", ls="--", lw=0.9)
    ax[0].set_ylabel(f"galat radius [m]\n(+ = luar, R = {R:.0f} m)")
    ax[0].legend(fontsize=8)
    ax[0].set_title(f"Satu putaran loiter fase stabil tanpa gangguan, R = {R:.0f} m (toleransi ±{tol:.0f} m putus-putus)")
    ax[1].axhline(14, color="red", lw=1.2, ls=":", label="NAVL1_PERIOD = 14 s")
    ax[1].set_ylabel("period L1 [s]")
    ax[1].legend(fontsize=8)
    ax[2].axhline(a.bank, color="k", ls=":", lw=1, label=f"batas bank {a.bank:.0f}°")
    ax[2].set_ylabel("nav_roll [°]")
    ax[2].set_xlabel("sudut orbit sejak awal putaran [°]")
    ax[2].legend(fontsize=8)
    for x_ in ax:
        x_.grid(alpha=0.3)
    ax[2].set_xlim(0, 360)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "02_galat_period_roll_vs_sudut.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
