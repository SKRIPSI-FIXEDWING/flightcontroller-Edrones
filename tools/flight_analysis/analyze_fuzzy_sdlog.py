#!/usr/bin/env python3
"""Analisis perilaku fuzzy self-tuning L1 dari log SD card (fc::FlightLog v2).

Menjawab "fuzzy-nya bekerja apa tidak, dan kapan": dari kolom cte_m, de_mps,
fuzzy_out, l1_dist_m. Dipisah per fase misi:
  AUTO     leg lurus (wp_next != WP loiter)
  entry    loiter putaran pertama (masuk ke lingkaran)
  steady   loiter setelah putaran pertama

Periode L1 yang dipakai DIHITUNG ULANG dari l1_dist_m = (1/pi)*zeta*T*Vg:
firmware <= 2026-10-29 mencatat l1_period_s yang salah (diam) di loiter. Untuk
log yang lebih baru, hasilnya sama dengan kolom l1_period_s.

Keluaran (folder -o):
  01_fuzzy_vs_waktu.png      |e|, d|e|/dt, dT dan periode, roll vs nav_roll
  02_aktivasi_aturan.png     kontribusi tiap aturan Tabel 3.6 per fase
  03_peta_kerja.png          titik (|e|, d|e|/dt) di atas peta keluaran fuzzy
  ringkasan_fuzzy.csv        per fase: durasi, |e| rata-rata/RMS, % waktu Turun/Tetap/Naik, periode
  aturan_per_fase.csv        bobot tiap aturan per fase (% dari total aktivasi)

Pemakaian:
    python analyze_fuzzy_sdlog.py "2026-10-29_10-26-20.CSV" -o flight_2026-10-29/fuzzy [--loiter-wp 4]
"""
import argparse
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fuzzy_mf_from_data as fz
from sdlog_io import read_sdlog

# Breakpoint firmware (include/navigation/FuzzyL1Tuner.h, dari data 26 okt).
E, D, O = fz.build_sets(3.0, 10.0, 30.0, 1.25, 2.25)
E_MAX, D_MAX = 60.0, 4.5
RULE_NAMES = [(dn, en, fz.RULES[dn][en]) for dn in ("Negatif", "Nol", "Positif") for en in ("Kecil", "Sedang", "Besar")]
PHASES = ("AUTO", "entry", "steady")
PHASE_COLOR = {"AUTO": "tab:blue", "entry": "tab:orange", "steady": "tab:green"}


def firing(e_abs, de):
    """Kekuatan tiap aturan (min AND), dengan input dijepit seperti firmware."""
    e = np.clip(e_abs, 0.0, E_MAX)
    d = np.clip(de, -D_MAX, D_MAX)
    out = np.zeros((len(e), len(RULE_NAMES)))
    for k, (dn, en, _) in enumerate(RULE_NAMES):
        out[:, k] = np.minimum(fz.trap(d, *D[dn]), fz.trap(e, *E[en]))
    return out


def loiter_turn_index(lat, lon):
    """Nomor putaran (0 = pertama) dari sudut kumulatif terhadap pusat rata-rata."""
    y = (lat - lat.mean()) * 1.11e-2
    x = (lon - lon.mean()) * 1.11e-2 * np.cos(np.radians(lat.mean() * 1e-7))
    ang = np.unwrap(np.arctan2(y, x))
    return np.floor(np.abs(ang - ang[0]) / (2 * np.pi)).astype(int)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="file log SD (CSV)")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--loiter-wp", type=int, default=None, help="nomor WP loiter (default: WP terakhir di misi)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    log = read_sdlog(args.log)
    c = log.col
    t = c["time_boot_ms"] / 1000.0
    eff = log.effective_at(float(c["time_boot_ms"][-1]))
    damping = eff.get("l1_damping", 0.707)
    base = eff.get("fuzzy_base_period", 14.0)

    wps = [s for s in log.snapshots if s.waypoints]
    loiter_wp = args.loiter_wp if args.loiter_wp is not None else max(w["seq"] for w in wps[-1].waypoints)

    auto = np.array([m in ("AUTO",) for m in c["mode"]]) & (c["wp_next"] > 0) & (c["armed"] > 0)
    in_loiter = auto & (c["wp_next"] == loiter_wp)
    phase = np.full(len(t), "", dtype=object)
    phase[auto & ~in_loiter] = "AUTO"
    if in_loiter.any():
        idx = np.flatnonzero(in_loiter)
        turn = loiter_turn_index(c["lat_e7"][idx], c["lon_e7"][idx])
        phase[idx[turn == 0]] = "entry"
        phase[idx[turn >= 1]] = "steady"
    m = phase != ""
    if not m.any():
        raise SystemExit("tidak ada data AUTO/loiter di log ini")

    e_abs = np.abs(c["cte_m"])
    de = c["de_mps"]
    dT = c["fuzzy_out"]
    gs = np.maximum(c["gs_mps"], 1.0)
    period = c["l1_dist_m"] / (0.3183099 * damping * gs)
    period_ok = c["gs_mps"] > 5.0  # di bawah ini L1 memakai jarak minimum, bukan T
    act = firing(e_abs, de)

    # ---- ringkasan per fase ----
    rows, rule_rows = [], []
    for ph in PHASES:
        s = phase == ph
        if not s.any():
            continue
        n = s.sum()
        dur = n * float(np.median(np.diff(t[m])))
        p = period[s & period_ok]
        rows.append(dict(
            fase=ph, durasi_s=round(dur, 1),
            e_mean_abs_m=round(float(e_abs[s].mean()), 2), e_rms_m=round(float(np.sqrt(np.mean(e_abs[s] ** 2))), 2),
            e_max_m=round(float(e_abs[s].max()), 1),
            pct_turun=round(100 * float(np.mean(dT[s] < -0.25)), 1),
            pct_tetap=round(100 * float(np.mean(np.abs(dT[s]) <= 0.25)), 1),
            pct_naik=round(100 * float(np.mean(dT[s] > 0.25)), 1),
            dT_mean_s=round(float(dT[s].mean()), 2),
            periode_mean_s=round(float(p.mean()), 2) if len(p) else float("nan"),
            periode_min_s=round(float(p.min()), 2) if len(p) else float("nan"),
            periode_max_s=round(float(p.max()), 2) if len(p) else float("nan"),
            pct_periode_di_batas=round(100 * float(np.mean((p < 12.05) | (p > 15.95))), 1) if len(p) else float("nan"),
            gs_mean=round(float(c["gs_mps"][s].mean()), 1),
            nav_roll_sat=round(float(np.mean(np.abs(c["nav_roll_deg"][s]) >= eff.get("roll_limit_deg", 40) - 5.5)), 2),
        ))
        tot = act[s].sum()
        rule_rows.append(dict(fase=ph, **{f"R{k + 1} {dn[:3]}/{en}->{on}": round(100 * float(act[s, k].sum() / tot), 1)
                                          for k, (dn, en, on) in enumerate(RULE_NAMES)}))
    for name, data in (("ringkasan_fuzzy.csv", rows), ("aturan_per_fase.csv", rule_rows)):
        with open(os.path.join(args.out, name), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(data[0].keys()))
            w.writeheader()
            w.writerows(data)

    # ---- 01 time series ----
    i0, i1 = np.flatnonzero(m)[[0, -1]]
    sl = slice(i0, i1 + 1)
    tt = t[sl] - t[i0]
    fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for ph in PHASES:
        s = (phase[sl] == ph)
        if s.any():
            for ax in axs:
                ax.fill_between(tt, 0, 1, where=s, transform=ax.get_xaxis_transform(), color=PHASE_COLOR[ph],
                                alpha=0.08, lw=0)
    axs[0].plot(tt, e_abs[sl], color="k", lw=0.8)
    for y in (3, 10, 30):
        axs[0].axhline(y, color="0.6", ls=":", lw=0.8)
    axs[0].set_ylabel("|e| [m]")
    axs[0].set_ylim(0, min(100, e_abs[sl].max() * 1.05 + 1))
    axs[1].plot(tt, np.clip(de[sl], -10, 10), color="tab:purple", lw=0.6)
    for y in (-2.25, -1.25, 1.25, 2.25):
        axs[1].axhline(y, color="0.6", ls=":", lw=0.8)
    axs[1].set_ylabel("d|e|/dt [m/s]\n(dijepit ±10)")
    axs[2].plot(tt, base + dT[sl], color="tab:red", lw=0.9, label="14 + dT (keluaran fuzzy)")
    pp = np.where(period_ok[sl], period[sl], np.nan)
    axs[2].plot(tt, pp, color="tab:blue", lw=0.6, alpha=0.7, label="periode dari l1_dist_m")
    axs[2].set_ylim(11, 17)
    axs[2].set_ylabel("periode L1 [s]")
    axs[2].legend(fontsize=8, loc="upper right")
    axs[3].plot(tt, c["nav_roll_deg"][sl], color="tab:orange", lw=0.8, label="nav_roll")
    axs[3].plot(tt, c["roll_deg"][sl], color="k", lw=0.6, label="roll")
    axs[3].set_ylabel("roll [deg]")
    axs[3].set_xlabel("waktu sejak masuk AUTO [s]")
    axs[3].legend(fontsize=8, loc="upper right")
    for ax in axs:
        ax.grid(alpha=0.3)
    handles = [plt.Rectangle((0, 0), 1, 1, color=PHASE_COLOR[p], alpha=0.25) for p in PHASES]
    axs[0].legend(handles, ["AUTO (leg lurus)", "loiter putaran 1 (masuk)", "loiter stabil"], fontsize=8,
                  loc="upper right")
    fig.suptitle(os.path.basename(args.log) + "  - fuzzy self-tuning L1")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "01_fuzzy_vs_waktu.png"), dpi=150)
    plt.close(fig)

    # ---- 02 rule activation ----
    fig, ax = plt.subplots(figsize=(11, 4.5))
    labels = [f"R{k + 1}\n{dn[:3]}/{en}\n->{on}" for k, (dn, en, on) in enumerate(RULE_NAMES)]
    width = 0.8 / max(1, len(rule_rows))
    for j, rr in enumerate(rule_rows):
        vals = [v for k, v in rr.items() if k != "fase"]
        ax.bar(np.arange(9) + (j - (len(rule_rows) - 1) / 2) * width, vals, width, color=PHASE_COLOR[rr["fase"]],
               label=rr["fase"])
    ax.set_xticks(np.arange(9))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("% dari total aktivasi aturan")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    ax.set_title("Aturan Tabel 3.6 yang bekerja, per fase")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "02_aktivasi_aturan.png"), dpi=150)
    plt.close(fig)

    # ---- 03 operating map ----
    eg = np.linspace(0, E_MAX, 121)
    dg = np.linspace(-D_MAX, D_MAX, 91)
    Z = np.array([[fz.infer(x, y, E, D, O) for x in eg] for y in dg])
    fig, ax = plt.subplots(figsize=(9, 6))
    cf = ax.contourf(eg, dg, Z, levels=np.linspace(-2, 2, 17), cmap="RdBu")
    fig.colorbar(cf, ax=ax, label="dT keluaran fuzzy [s]")
    for ph in PHASES:
        s = phase == ph
        if s.any():
            ax.scatter(np.clip(e_abs[s], 0, E_MAX), np.clip(de[s], -D_MAX, D_MAX), s=4, color=PHASE_COLOR[ph],
                       edgecolor="none", alpha=0.6, label=ph)
    ax.set_xlabel("|e| [m] (dijepit 60)")
    ax.set_ylabel("d|e|/dt [m/s] (dijepit ±4,5)")
    ax.legend(markerscale=3)
    ax.set_title("Titik kerja terbang di atas peta keluaran fuzzy (merah = Turun, biru = Naik)")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "03_peta_kerja.png"), dpi=150)
    plt.close(fig)

    for r in rows:
        print(r)
    for r in rule_rows:
        print(r)
    print("hasil ->", args.out)


if __name__ == "__main__":
    main()
