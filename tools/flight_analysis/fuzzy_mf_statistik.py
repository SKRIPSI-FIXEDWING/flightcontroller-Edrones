"""
Pendukung penjelasan "kenapa median/persentil, bukan rata-rata ± standar deviasi"
untuk fungsi keanggotaan fuzzy L1 (data terbang 2026-10-26, NAVL1_PERIOD 12-16 s).

Keluaran (di folder fuzzy_mf_2026-10-26/):
  statistik_mean_vs_median.csv          tabel mean, SD, mean+SD, median, P75, P90
  05_mean_sd_vs_median.png              histogram + garis mean/SD vs median/persentil
  06_mf_error_beranotasi.png            MF |e| dengan asal tiap breakpoint
  07_mf_perubahan_error_beranotasi.png  MF Δe dengan asal tiap breakpoint
  08_mf_output_beranotasi.png           MF ΔT periode L1

    python fuzzy_mf_statistik.py

Data dan pembersihan sama persis dengan fuzzy_mf_from_data.py (load_segments).
"""
import csv
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import fuzzy_mf_from_data as fm

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "fuzzy_mf_2026-10-26")
FLIGHTS = ["flight_2026-10-26_b", "flight_2026-10-26_d", "flight_2026-10-26_e",
           "flight_2026-10-26", "flight_2026-10-26_f"]
TOL = fm.TOL_M

# breakpoint firmware (FuzzyL1TunerConfig)
EK, ES, EB, D0, DB, DT = 3.0, 10.0, 30.0, 1.25, 2.25, fm.DT_OUT


def load():
    le, ld, ae = [], [], []
    for d in FLIGHTS:
        p = os.path.join(HERE, d)
        e, x = fm.load_segments(os.path.join(p, "loiter", "galat_per_sampel.csv"), "galat_m(+luar)", "segmen")
        le.append(e); ld.append(x)
        e, _ = fm.load_segments(os.path.join(p, "cte_per_sampel.csv"), "cte_m(+kanan)", "wp_aktif")
        ae.append(e)
    le, ld = np.concatenate(le), np.concatenate(ld)
    return le, ld, np.concatenate([le] + ae)


def stats(x):
    return dict(n=len(x), mean=x.mean(), sd=x.std(), mean_sd=x.mean() + x.std(),
                median=np.median(x), p75=np.percentile(x, 75), p90=np.percentile(x, 90), max=x.max())


def main():
    os.makedirs(OUT, exist_ok=True)
    lo_e, lo_d, all_e = load()
    inside = lo_e <= TOL
    groups = [
        ("|e| loiter, semua", "m", lo_e, "-"),
        ("|e| loiter, di dalam toleransi", "m", lo_e[inside], f"Kecil = median = {EK:g} m"),
        ("|e| semua, di luar toleransi", "m", all_e[all_e > TOL], f"Besar = median = {EB:g} m"),
        ("|Δe| loiter, di dalam toleransi", "m/s", np.abs(lo_d[inside]),
         f"Nol = P75 = {D0:g}; Neg/Pos = P90 = {DB:g} m/s"),
    ]

    # ---- tabel ----
    with open(os.path.join(OUT, "statistik_mean_vs_median.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kelompok_data", "satuan", "n", "mean", "sd", "mean+sd", "median", "p75", "p90", "max",
                    "dipakai_firmware"])
        print(f"{'kelompok':34s} {'n':>5s} {'mean':>6s} {'SD':>6s} {'mean+SD':>8s} {'median':>7s} {'P75':>6s} {'P90':>6s}")
        for name, unit, x, used in groups:
            s = stats(x)
            w.writerow([name, unit, s["n"]] + [f"{s[k]:.2f}" for k in
                       ("mean", "sd", "mean_sd", "median", "p75", "p90", "max")] + [used])
            print(f"{name:34s} {s['n']:5d} {s['mean']:6.2f} {s['sd']:6.2f} {s['mean_sd']:8.2f} "
                  f"{s['median']:7.2f} {s['p75']:6.2f} {s['p90']:6.2f}")

    # ---- 05: mean±SD vs median/persentil ----
    fig, axs = plt.subplots(2, 1, figsize=(8, 7))
    panels = [
        (axs[0], lo_e, (0, 80), 80, "|e| cross-track error loiter [m]",
         "(a) Error loiter: rata-rata ditarik oleh sedikit nilai ekstrem",
         [("median", np.median(lo_e), "tab:green", "-")]),
        (axs[1], np.abs(lo_d[inside]), (0, 8), 64, "|Δe| saat loiter di dalam toleransi [m/s]",
         "(b) Kecepatan mendekat/menjauh: mean+SD jauh di atas kondisi normal",
         [("P75", np.percentile(np.abs(lo_d[inside]), 75), "tab:green", "-"),
          ("P90", np.percentile(np.abs(lo_d[inside]), 90), "tab:green", "--")]),
    ]
    for ax, x, xr, bins, xl, title, robust in panels:
        ax.hist(np.clip(x, *xr), bins=bins, range=xr, color="0.8", edgecolor="0.6")
        m, sd = x.mean(), x.std()
        ax.axvline(m, color="tab:red", lw=2, label=f"rata-rata = {m:.2f}")
        ax.axvline(m + sd, color="tab:red", lw=2, ls="--", label=f"rata-rata + SD = {m + sd:.2f}")
        for lab, v, c, ls in robust:
            ax.axvline(v, color=c, lw=2, ls=ls, label=f"{lab} = {v:.2f}")
        ax.set_xlim(*xr); ax.set_xlabel(xl); ax.set_ylabel("jumlah sampel")
        ax.set_title(title, fontsize=10); ax.legend(fontsize=9); ax.grid(alpha=0.3)
    axs[0].axvspan(0, TOL, color="tab:green", alpha=0.07)
    axs[0].text(TOL, axs[0].get_ylim()[1] * 0.92, f"  batas toleransi {TOL:g} m", fontsize=8, color="tab:green")
    fig.suptitle("Rata-rata ± SD vs median/persentil — data terbang 26 Okt 2026 (NAVL1_PERIOD 12–16 s)",
                 fontsize=11)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "05_mean_sd_vs_median.png"), dpi=150); plt.close(fig)

    # ---- 06-08: MF beranotasi ----
    E, D, O = fm.build_sets(EK, ES, EB, D0, DB)
    colors = {"Kecil": "tab:green", "Sedang": "tab:orange", "Besar": "tab:red",
              "Negatif": "tab:blue", "Nol": "tab:gray", "Positif": "tab:red",
              "Turun": "tab:blue", "Tetap": "tab:gray", "Naik": "tab:red"}

    def mf(fname, sets, xr, xlabel, title, data, marks):
        fig, ax = plt.subplots(figsize=(8, 4.2))
        if data is not None:
            ax2 = ax.twinx()
            ax2.hist(np.clip(data, *xr), bins=60, range=xr, color="0.88")
            ax2.set_ylabel("jumlah sampel (data terbang)", color="0.5")
            ax.set_zorder(ax2.get_zorder() + 1); ax.patch.set_visible(False)
        x = np.linspace(*xr, 1000)
        for k, s in sets.items():
            ax.plot(x, fm.trap(x, *s), lw=2.2, color=colors[k], label=k)
        for v, txt in marks:
            ax.axvline(v, color="k", lw=0.8, ls=":")
            ax.text(v, 1.10, txt, rotation=0, ha="center", va="bottom", fontsize=7.5,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="0.7", alpha=0.95))
        ax.set_ylim(0, 1.32); ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0]); ax.set_xlim(*xr)
        ax.set_xlabel(xlabel); ax.set_ylabel("derajat keanggotaan μ")
        ax.set_title(title, fontsize=10); ax.legend(loc="center right"); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, fname), dpi=150); plt.close(fig)

    mf("06_mf_error_beranotasi.png", E, (0, 2 * EB), "|e| cross-track error [m]",
       "Fungsi keanggotaan |e| — asal tiap titik dari data terbang", all_e,
       [(EK, f"{EK:g} m\nmedian error\ndi dalam toleransi"),
        (ES, f"{ES:g} m\nbatas\ntoleransi"),
        (EB, f"{EB:g} m\nmedian error\ndi luar toleransi")])
    mf("07_mf_perubahan_error_beranotasi.png", D, (-2 * DB, 2 * DB),
       "Δe = d|e|/dt [m/s]   (− mendekati lintasan, + menjauhi)",
       "Fungsi keanggotaan Δe — asal tiap titik dari data terbang", lo_d[inside],
       [(-DB, f"−{DB:g}\nP90"), (-D0, f"−{D0:g}\nP75"), (D0, f"+{D0:g}\nP75"), (DB, f"+{DB:g}\nP90")])
    mf("08_mf_output_beranotasi.png", O, (-2 * DT, 2 * DT), "ΔT periode L1 [s]   (periode = NAVL1_PERIOD + ΔT)",
       "Fungsi keanggotaan ΔT — Turun = lebih tegas, Naik = lebih halus", None,
       [(-DT, f"−{DT:g} s\nperiode 12 s"), (0, "0 s\nperiode 14 s"), (DT, f"+{DT:g} s\nperiode 16 s")])

    print(f"\ntabel & gambar -> {OUT}")


if __name__ == "__main__":
    main()
