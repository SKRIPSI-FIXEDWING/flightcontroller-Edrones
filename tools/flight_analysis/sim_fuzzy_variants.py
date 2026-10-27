#!/usr/bin/env python3
"""Varian rancangan fuzzy self-tuning, diuji di simulator sim_guidance_compare.py
(firmware TIDAK diubah). Pertanyaannya: rancangan fuzzy mana yang benar-benar
mempercepat pemulihan loiter setelah pulsa gangguan, dibandingkan L1 tetap?

Varian:
  L1 tetap P14            pembanding utama (period terbaik data terbang)
  L1 tetap P12            period tetap yang paling agresif yang pernah diterbangkan
  L1 tetap P14 bank45     batas bank dinaikkan permanen (kontrol untuk varian "+bank")
  Fuzzy T3.6              rancangan sekarang (Tabel 3.6, dT -2..+2, period 12-16)
  Fuzzy tanpa Naik        R1 (Kecil & Negatif) = Tetap
  Fuzzy Turun -4          tanpa Naik, Turun sampai -4 s (period 10-14)
  Fuzzy T3.6 + bank       keluaran Turun juga menaikkan batas bank 35 -> 45 deg
  Fuzzy tanpa Naik + bank sama, dengan R1 = Tetap

Run:  python sim_fuzzy_variants.py
"""

import copy
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fuzzy_mf_from_data as fz
import sim_guidance_compare as sim

OUT = os.path.join(sim.HERE, "sim_fuzzy_variants_2026-10-26")
BANK_HI = 45.0
RULES_T36 = copy.deepcopy(fz.RULES)


def make_lut(dt_out, no_naik):
    fz.DT_OUT = dt_out
    fz.RULES = copy.deepcopy(RULES_T36)
    if no_naik:
        fz.RULES["Negatif"]["Kecil"] = "Tetap"
    lut = sim.FuzzyLUT()
    fz.DT_OUT = 2.0
    fz.RULES = copy.deepcopy(RULES_T36)
    return lut


def bank_from_dT(dT, dt_out=2.0):
    """Turun penuh (dT = -dt_out) -> BANK_HI; Tetap/Naik -> LOITER_BANK."""
    return sim.LOITER_BANK + (BANK_HI - sim.LOITER_BANK) * min(max(-dT / dt_out, 0.0), 1.0)


def variants():
    lut_t36 = make_lut(2.0, False)
    lut_nonaik = make_lut(2.0, True)
    lut_down4 = make_lut(4.0, True)
    # (nama, metode, period dasar, (min,max), lut, bank tetap, bank_fn)
    return [
        ("L1 tetap P14", "L1 tetap", 14.0, (12, 16), None, None, None),
        ("L1 tetap P12", "L1 tetap", 12.0, (12, 16), None, None, None),
        ("L1 tetap P14 bank45", "L1 tetap", 14.0, (12, 16), None, BANK_HI, None),
        ("Fuzzy T3.6", "L1 + fuzzy", 14.0, (12, 16), lut_t36, None, None),
        ("Fuzzy tanpa Naik", "L1 + fuzzy", 14.0, (12, 16), lut_nonaik, None, None),
        ("Fuzzy Turun -4", "L1 + fuzzy", 14.0, (10, 14), lut_down4, None, None),
        ("Fuzzy T3.6 + bank", "L1 + fuzzy", 14.0, (12, 16), lut_t36, None, bank_from_dT),
        ("Fuzzy tanpa Naik + bank", "L1 + fuzzy", 14.0, (12, 16), lut_nonaik, None, bank_from_dT),
    ]


def main():
    os.makedirs(OUT, exist_ok=True)
    settle, gap, n_pulse = 60.0, 45.0, 8
    pulses = [settle + i * gap for i in range(n_pulse)]
    t_end = pulses[-1] + gap
    rows, examples = [], {}
    for R in (112.0, 90.0):
        for name, method, base, (pmin, pmax), lut, bank, bank_fn in variants():
            sim.PERIOD, sim.P_MIN, sim.P_MAX = base, pmin, pmax
            # steady tanpa gangguan
            L0 = sim.run(method, "loiter", [], 0.0, 200.0, R=R, fuzzy=lut, bank=bank, bank_fn=bank_fn)
            s0 = L0[L0[:, 0] > 40, 3]
            steady_rms = float(np.sqrt(np.mean(s0 ** 2)))
            steady_out = 100.0 * float(np.mean(np.abs(s0) > sim.TOL))
            for amp in (20.0, 30.0):
                per = []
                for sgn in (+1, -1):
                    L = sim.run(method, "loiter", pulses, sgn * amp, t_end, R=R, fuzzy=lut, bank=bank,
                                bank_fn=bank_fn)
                    for p in pulses:
                        r = sim.pulse_metrics(L, p)
                        m = (L[:, 0] >= p) & (L[:, 0] < p + 40)
                        r["roll_max"] = float(np.max(np.abs(L[m, 5])))
                        r["frac_bank_gt35"] = float(np.mean(np.abs(L[m, 4]) > sim.LOITER_BANK + 0.01))
                        per.append(r)
                    if amp == 30.0 and sgn == -1:
                        examples[(R, name)] = L
                agg = {k: float(np.nanmean([r[k] for r in per]))
                       for k in ("peak_abs", "overshoot", "recovery_s", "rms", "pct_out_tol", "roll_max",
                                 "frac_bank_gt35")}
                agg["gagal_pulih"] = sum(1 for r in per if math.isnan(r["recovery_s"]))
                rows.append(dict(radius=R, varian=name, amp_deg=amp, steady_rms=steady_rms,
                                 steady_pct_out_tol=steady_out, **agg))
    sim.PERIOD, sim.P_MIN, sim.P_MAX = 14.0, 12.0, 16.0

    with open(os.path.join(OUT, "ringkasan_varian.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})

    hdr = (f"{'R':>4s} {'amp':>3s} {'varian':24s} {'stRMS':>6s} {'st%out':>6s} {'puncak':>6s} {'oversh':>6s} "
           f"{'pulih':>6s} {'RMS40':>6s} {'%out':>5s} {'rollmax':>7s} {'b>35':>5s}")
    print(hdr)
    for r in rows:
        print(f"{r['radius']:4.0f} {r['amp_deg']:3.0f} {r['varian']:24s} {r['steady_rms']:6.2f} "
              f"{r['steady_pct_out_tol']:6.1f} {r['peak_abs']:6.1f} {r['overshoot']:6.1f} {r['recovery_s']:6.2f} "
              f"{r['rms']:6.2f} {r['pct_out_tol']:5.1f} {r['roll_max']:7.1f} {r['frac_bank_gt35']:5.2f}")

    # ---- grafik batang: per radius, amp 30 ----
    names = [v[0] for v in variants()]
    colors = ["tab:blue", "tab:cyan", "tab:purple", "tab:red", "tab:orange", "tab:brown", "tab:pink", "tab:olive"]
    for R in (112.0, 90.0):
        fig, axs = plt.subplots(1, 4, figsize=(16, 4.6))
        for ax, (k, lab) in zip(axs, [("steady_rms", "RMS galat tanpa gangguan [m]"), ("peak_abs", "galat puncak [m]"),
                                      ("recovery_s", "waktu pulih [s]"), ("rms", "RMS galat 40 s [m]")]):
            for amp, hatch, off in ((20.0, "", -0.2), (30.0, "//", 0.2)):
                if k == "steady_rms" and amp == 30.0:
                    continue
                v = [next(r[k] for r in rows if r["radius"] == R and r["varian"] == n and r["amp_deg"] == amp)
                     for n in names]
                ax.bar(np.arange(len(names)) + (0 if k == "steady_rms" else off), v,
                       0.8 if k == "steady_rms" else 0.4, color=colors, hatch=hatch, edgecolor="k", lw=0.3)
            ax.set_xticks(np.arange(len(names)))
            ax.set_xticklabels(names, rotation=60, ha="right", fontsize=7)
            ax.set_title(lab, fontsize=10)
            ax.grid(axis="y", alpha=0.3)
        fig.suptitle(f"Varian fuzzy, loiter R {R:.0f} m (batang polos = gangguan 20°, arsir = 30°)")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"01_varian_R{R:.0f}.png"), dpi=150)
        plt.close(fig)

    # ---- galat vs waktu contoh: 30 deg ke luar, pulsa ke-3 ----
    p = pulses[2]
    for R in (112.0, 90.0):
        fig, axs = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
        for n, c in zip(names, colors):
            if n not in ("L1 tetap P14", "L1 tetap P12", "Fuzzy T3.6", "Fuzzy tanpa Naik + bank", "L1 tetap P14 bank45"):
                continue
            L = examples[(R, n)]
            m = (L[:, 0] >= p - 5) & (L[:, 0] < p + 35)
            axs[0].plot(L[m, 0] - p, L[m, 3], color=c, label=n)
            axs[1].plot(L[m, 0] - p, L[m, 5], color=c, label=n)
        axs[0].axhspan(-sim.RECOVERY_BAND, sim.RECOVERY_BAND, color="green", alpha=0.08)
        for ax in axs:
            ax.axvspan(0, sim.DIST_MS / 1000, color="k", alpha=0.12)
            ax.grid(alpha=0.3)
        axs[0].set_ylabel("galat radius [m] (+ luar)")
        axs[1].set_ylabel("roll aktual [deg]")
        axs[1].set_xlabel("waktu sejak pulsa mulai [s]")
        axs[0].legend(fontsize=8)
        axs[0].set_title(f"Loiter R {R:.0f} m, pulsa 30° ke luar")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"02_contoh_R{R:.0f}.png"), dpi=150)
        plt.close(fig)
    print(f"\nhasil -> {OUT}")


if __name__ == "__main__":
    main()
