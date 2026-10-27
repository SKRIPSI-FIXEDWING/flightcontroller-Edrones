"""
Menurunkan breakpoint fungsi keanggotaan fuzzy self-tuning periode L1 dari
data terbang (proposal 3.5.3: "rentang nilai ... ditentukan berdasarkan hasil
analisis data lapangan"), lalu menggambar fungsi keanggotaan (Gambar 3.6-3.8)
dan permukaan aturan (Tabel 3.6).

Masukan: CSV per sampel keluaran analyze_tlog_loiter.py (loiter/galat_per_sampel.csv)
dan analyze_tlog_l1_auto.py (cte_per_sampel.csv) di tiap folder terbang.

    python fuzzy_mf_from_data.py flight_2026-10-26_b=12 flight_2026-10-26_d=13 \
        flight_2026-10-26_e=14 flight_2026-10-26=15 flight_2026-10-26_f=16

Definisi (sama dengan firmware, FuzzyL1Tuner):
  e   = |galat lintasan| [m]   (loiter: jarak ke pusat - radius; AUTO: cross-track)
  de  = d|e|/dt [m/s]          (negatif = mendekati lintasan, positif = menjauh)

Aturan penurunan breakpoint (semua dari data, TOL = batas toleransi):
  e_K = median e sampel loiter di dalam toleransi      -> Kecil penuh sampai e_K
  e_S = TOL                                           -> puncak Sedang
  e_B = median e sampel di luar toleransi             -> Besar penuh mulai e_B
  d_0 = persentil 75 |de| sampel loiter di dalam toleransi -> Nol penuh di [-d_0, d_0]
  d_B = persentil 90 |de| sampel loiter di dalam toleransi -> Negatif/Positif penuh di luar +-d_B
Keluaran dT [s]: Turun/Tetap/Naik = segitiga berpuncak -DT/0/+DT (DT = 2 s, sehingga
periode dasar 14 s bergerak di 12-16 s, rentang yang sudah diterbangkan).
"""
import argparse
import csv
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
TOL_M = 10.0          # batas toleransi galat (sama dengan analyze_tlog_loiter.py --tol)
SKIP_FIRST = 2        # sampel awal tiap segmen dibuang (lompatan cte saat ganti WP)
MAX_RATE = 30.0       # |de| di atas ini = glitch GPS, dibuang
DT_OUT = 2.0          # puncak Turun/Naik [s]


def load_segments(path, err_col, seg_col):
    rows = list(csv.DictReader(open(path)))
    seg = np.array([r[seg_col] for r in rows])
    t = np.array([float(r["t_s"]) for r in rows])
    e = np.array([float(r[err_col]) for r in rows])
    out = []
    for s in dict.fromkeys(seg):
        m = seg == s
        tt, ee = t[m], e[m]
        dt = np.diff(tt)
        de_abs = np.sign(ee[1:]) * np.diff(ee) / np.where(dt > 0, dt, np.nan)
        ok = (dt > 0.1) & (dt < 0.5) & (np.abs(de_abs) < MAX_RATE)
        ok[:SKIP_FIRST] = False
        out.append((np.abs(ee[1:][ok]), de_abs[ok]))
    if not out:
        return np.array([]), np.array([])
    return np.concatenate([o[0] for o in out]), np.concatenate([o[1] for o in out])


# ---- fungsi keanggotaan & inferensi, cermin eFLL (FuzzySet a,b,c,d) ----
def trap(x, a, b, c, d):
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)
    if a == b:
        y[x <= b] = 1.0
    else:
        m = (x >= a) & (x < b)
        y[m] = (x[m] - a) / (b - a)
    y[(x >= b) & (x <= c)] = 1.0
    if c == d:
        y[x >= c] = 1.0
    else:
        m = (x > c) & (x <= d)
        y[m] = (d - x[m]) / (d - c)
    return y


def build_sets(eK, eS, eB, d0, dB):
    E = {"Kecil": (0, 0, eK, eS), "Sedang": (eK, eS, eS, eB), "Besar": (eS, eB, 2 * eB, 2 * eB)}
    D = {"Negatif": (-2 * dB, -2 * dB, -dB, -d0), "Nol": (-dB, -d0, d0, dB), "Positif": (d0, dB, 2 * dB, 2 * dB)}
    O = {"Turun": (-2 * DT_OUT, -DT_OUT, -DT_OUT, 0), "Tetap": (-DT_OUT, 0, 0, DT_OUT), "Naik": (0, DT_OUT, DT_OUT, 2 * DT_OUT)}
    return E, D, O


# Tabel 3.6 proposal: RULES[de][e]
RULES = {
    "Negatif": {"Kecil": "Naik", "Sedang": "Tetap", "Besar": "Turun"},
    "Nol": {"Kecil": "Tetap", "Sedang": "Tetap", "Besar": "Turun"},
    "Positif": {"Kecil": "Tetap", "Sedang": "Turun", "Besar": "Turun"},
}


def infer(e, de, E, D, O):
    """Min (AND), max (agregasi), centroid -- seperti eFLL."""
    act = {k: 0.0 for k in O}
    for dn, row in RULES.items():
        md = float(trap([de], *D[dn])[0])
        for en, on in row.items():
            act[on] = max(act[on], min(md, float(trap([e], *E[en])[0])))
    x = np.linspace(-2 * DT_OUT, 2 * DT_OUT, 801)
    mu = np.max([np.minimum(trap(x, *O[k]), act[k]) for k in O], axis=0)
    return float(np.sum(x * mu) / np.sum(mu)) if mu.sum() > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("terbang", nargs="+", help="folder=NAVL1_PERIOD, mis. flight_2026-10-26_b=12")
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "fuzzy_mf_2026-10-26"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    lo_e, lo_d, au_e, au_d = [], [], [], []
    print("terbang                 P   n_loiter  n_auto")
    for item in args.terbang:
        folder, period = item.split("=")
        path = os.path.join(HERE, folder)
        e1, d1 = load_segments(os.path.join(path, "loiter", "galat_per_sampel.csv"), "galat_m(+luar)", "segmen") \
            if os.path.exists(os.path.join(path, "loiter", "galat_per_sampel.csv")) else (np.array([]), np.array([]))
        e2, d2 = load_segments(os.path.join(path, "cte_per_sampel.csv"), "cte_m(+kanan)", "wp_aktif") \
            if os.path.exists(os.path.join(path, "cte_per_sampel.csv")) else (np.array([]), np.array([]))
        lo_e.append(e1); lo_d.append(d1); au_e.append(e2); au_d.append(d2)
        print(f"{folder:22s} {period:>3s} {len(e1):9d} {len(e2):7d}")
    lo_e, lo_d = np.concatenate(lo_e), np.concatenate(lo_d)
    all_e = np.concatenate([lo_e] + au_e)
    all_d = np.concatenate([lo_d] + au_d)

    inside = lo_e <= TOL_M
    eK = float(np.median(lo_e[inside]))
    eS = TOL_M
    eB = float(np.median(all_e[all_e > TOL_M]))
    d0 = float(np.percentile(np.abs(lo_d[inside]), 75))
    dB = float(np.percentile(np.abs(lo_d[inside]), 90))
    r = lambda v, q=0.5: round(v / q) * q  # dibulatkan untuk firmware
    eK, eB, d0, dB = r(eK), r(eB, 5.0), r(d0, 0.25), r(dB, 0.25)

    print(f"\nsampel: loiter {len(lo_e)} (di dalam toleransi {inside.sum()}), total {len(all_e)}")
    print(f"e : Kecil penuh <= {eK} m, puncak Sedang = {eS} m (toleransi), Besar penuh >= {eB} m")
    print(f"de: Nol penuh di +-{d0} m/s, Negatif/Positif penuh di luar +-{dB} m/s")
    E, D, O = build_sets(eK, eS, eB, d0, dB)
    print("\n// FuzzyL1TunerConfig (include/navigation/FuzzyL1Tuner.h)")
    for name, s in [("e_small", E["Kecil"]), ("e_medium", E["Sedang"]), ("e_large", E["Besar"]),
                    ("de_negative", D["Negatif"]), ("de_zero", D["Nol"]), ("de_positive", D["Positif"]),
                    ("out_down", O["Turun"]), ("out_hold", O["Tetap"]), ("out_up", O["Naik"])]:
        print(f"FuzzyTrapezoid {name}{{{', '.join(f'{v:g}f' for v in s)}}};")

    # ---- Gambar 3.6 / 3.7 / 3.8 ----
    colors = {"Kecil": "tab:green", "Sedang": "tab:orange", "Besar": "tab:red",
              "Negatif": "tab:blue", "Nol": "tab:gray", "Positif": "tab:red",
              "Turun": "tab:blue", "Tetap": "tab:gray", "Naik": "tab:red"}

    def mf_plot(fname, sets, xr, xlabel, title, data=None):
        fig, ax = plt.subplots(figsize=(7, 3.6))
        if data is not None:
            ax2 = ax.twinx()
            ax2.hist(np.clip(data, *xr), bins=60, range=xr, color="0.85")
            ax2.set_ylabel("jumlah sampel", color="0.5")
            ax.set_zorder(ax2.get_zorder() + 1); ax.patch.set_visible(False)
        x = np.linspace(*xr, 1000)
        for k, s in sets.items():
            ax.plot(x, trap(x, *s), lw=2, color=colors[k], label=k)
        ax.set_ylim(0, 1.08); ax.set_xlim(*xr)
        ax.set_xlabel(xlabel); ax.set_ylabel("derajat keanggotaan μ"); ax.set_title(title)
        ax.legend(loc="center right"); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(os.path.join(args.out, fname), dpi=150); plt.close(fig)

    mf_plot("01_mf_cross_track_error.png", E, (0, 2 * eB), "|e| cross-track error [m]",
            "Fungsi keanggotaan cross-track error (histogram = data terbang)", all_e)
    mf_plot("02_mf_perubahan_error.png", D, (-2 * dB - 2, 2 * dB + 2), "Δe = d|e|/dt [m/s]  (− mendekati, + menjauh)",
            "Fungsi keanggotaan perubahan cross-track error", all_d)
    mf_plot("03_mf_penyesuaian_periode.png", O, (-2 * DT_OUT, 2 * DT_OUT), "ΔT periode L1 [s]",
            "Fungsi keanggotaan penyesuaian periode L1")

    ee = np.linspace(0, 2 * eB, 61)
    dd = np.linspace(-2 * dB, 2 * dB, 61)
    Z = np.array([[infer(a, b, E, D, O) for a in ee] for b in dd])
    fig, ax = plt.subplots(figsize=(7, 4.6))
    pc = ax.pcolormesh(ee, dd, Z, cmap="coolwarm", vmin=-DT_OUT, vmax=DT_OUT, shading="auto")
    fig.colorbar(pc, label="ΔT [s]  (− = lebih agresif)")
    ax.set_xlabel("|e| [m]"); ax.set_ylabel("Δe [m/s]"); ax.set_title("Permukaan aturan fuzzy (Tabel 3.6)")
    fig.tight_layout(); fig.savefig(os.path.join(args.out, "04_permukaan_aturan.png"), dpi=150); plt.close(fig)

    with open(os.path.join(args.out, "breakpoint.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["himpunan", "a", "b", "c", "d"])
        for sets in (E, D, O):
            for k, s in sets.items():
                w.writerow([k, *s])
    print("\nTitik uji (|e| m, Δe m/s) -> ΔT s:")
    for a, b in [(0, 0), (2, -dB * 1.5), (eS, 0), (eS, dB * 1.5), (eB, 0), (eB, -dB * 1.5), (2, dB * 1.5)]:
        print(f"  ({a:5.1f}, {b:6.2f}) -> {infer(a, b, E, D, O):+.3f}")
    print(f"\ngambar & breakpoint.csv -> {args.out}")


if __name__ == "__main__":
    main()
