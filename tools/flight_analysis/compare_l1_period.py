#!/usr/bin/env python3
"""Perbandingan metrik LOITER dan AUTO antar NAVL1_PERIOD dari beberapa terbang.

Membaca CSV keluaran analyze_tlog_l1_auto.py (root folder terbang) dan
analyze_tlog_loiter.py --bagian loiter (subfolder loiter/), lalu menyusun
satu baris per terbang. Folder tanpa CSV (mis. terbang tanpa loiter/AUTO)
dilewati dengan catatan.

Galat loiter dipisah dua:
  total  = seluruh segmen loiter, termasuk transien entry
  steady = mulai putaran ke-2 (putaran pertama berisi entry ke lingkaran)
Angin ikut dilaporkan karena memengaruhi galat dan berbeda tiap terbang.

Pemakaian:
    python compare_l1_period.py flight_2026-10-26_b=12 flight_2026-10-26_d=13 ... [-o out_dir] [--tol 10]
"""
import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LEG_ORDER = ["entry->WP1", "WP1->WP2", "WP2->WP3", "WP3->WP4"]


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def loiter_metrics(folder, tol):
    lo = os.path.join(folder, "loiter")
    summ = read_csv(os.path.join(lo, "ringkasan_loiter.csv"))
    if not summ:
        return None
    s = summ[0]
    if len(summ) > 1:
        print("CATATAN: %s punya %d segmen loiter; hanya segmen 1 dipakai" % (folder, len(summ)))
    turns = [r for r in read_csv(os.path.join(lo, "metrik_per_putaran.csv")) if r["segmen"] == "1"]
    samp = [r for r in read_csv(os.path.join(lo, "galat_per_sampel.csv")) if r["segmen"] == "1"]
    t = np.array([num(r["t_s"]) for r in samp])
    err = np.array([num(r["galat_m(+luar)"]) for r in samp])
    t2 = num(turns[1]["t_mulai"]) if len(turns) >= 2 else np.nan
    st = err[t >= t2] if np.isfinite(t2) else np.array([])
    return dict(
        loiter_durasi_s=num(s["durasi"]),
        jumlah_putaran=num(s["jumlah_putaran"]),
        radius_cmd_m=num(s["radius_cmd"]),
        angin_mps=num(s["angin_perkiraan"]),
        airspeed_mean=num(s["airspeed_mean"]),
        konvergen_s=num(s["konvergen_s"]),
        galat_rms_total_m=num(s["galat_rms"]),
        galat_max_total_m=num(s["galat_max_abs"]),
        persen_lewat_tol_total=num(s["persen_lewat_tol"]),
        galat_rms_steady_m=round(float(np.sqrt(np.mean(st ** 2))), 2) if len(st) else np.nan,
        galat_mean_steady_m=round(float(np.mean(st)), 2) if len(st) else np.nan,
        galat_max_steady_m=round(float(np.max(np.abs(st))), 2) if len(st) else np.nan,
        persen_lewat_tol_steady=round(100.0 * float(np.mean(np.abs(st) > tol)), 1) if len(st) else np.nan,
        loiter_roll_track_rmse=num(s["roll_track_rmse"]),
        loiter_nav_roll_sat=num(s["nav_roll_sat"]),
        alt_galat_rms_m=num(s["alt_galat_rms"]),
    )


def auto_metrics(folder):
    summ = read_csv(os.path.join(folder, "ringkasan_auto.csv"))
    if not summ:
        return None
    s = summ[0]
    legs = {r["leg"]: r for r in read_csv(os.path.join(folder, "metrik_per_leg.csv")) if r["terbang"] == "1"}
    d = dict(
        auto_durasi_s=num(s["durasi"]),
        cte_mean_abs_m=num(s["cte_mean_abs"]),
        cte_rms_m=num(s["cte_rms"]),
        cte_max_abs_m=num(s["cte_max_abs"]),
        persen_lewat_45m=num(s["persen_lewat_45m"]),
        auto_roll_track_rmse=num(s["roll_track_rmse"]),
        auto_nav_roll_sat=num(s["nav_roll_sat"]),
        l1_dist_mean_m=num(s["l1_mean"]),
    )
    for leg in LEG_ORDER:
        d["cte_mean_abs_%s" % leg] = num(legs[leg]["cte_mean_abs_m"]) if leg in legs else np.nan
    return d


def write_csv(path, rows):
    keys = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if isinstance(v, float) and math.isnan(v) else v) for k, v in r.items()})


def label_points(ax, P, Y, names):
    for p, y, n in zip(P, Y, names):
        if np.isfinite(y):
            ax.annotate(n, (p, y), textcoords="offset points", xytext=(5, 4), fontsize=7, color="0.35")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("terbang", nargs="+", help="folder=NAVL1_PERIOD, mis. flight_2026-10-26_b=12")
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "perbandingan_L1"))
    ap.add_argument("--tol", type=float, default=10.0, help="toleransi galat radius loiter [m]")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    lrows, arows = [], []
    for item in args.terbang:
        folder, period = item.rsplit("=", 1)
        path = folder if os.path.isabs(folder) else os.path.join(HERE, folder)
        base = dict(terbang=os.path.basename(folder.rstrip("/")), navl1_period_s=float(period))
        lm, am = loiter_metrics(path, args.tol), auto_metrics(path)
        if lm is None:
            print("CATATAN: %s tanpa ringkasan loiter, dilewati" % folder)
        else:
            lrows.append(dict(base, **lm))
        if am is None:
            print("CATATAN: %s tanpa ringkasan AUTO, dilewati" % folder)
        else:
            arows.append(dict(base, **am))
    lrows.sort(key=lambda r: r["navl1_period_s"])
    arows.sort(key=lambda r: r["navl1_period_s"])
    if lrows:
        write_csv(os.path.join(args.out, "perbandingan_loiter.csv"), lrows)
    if arows:
        write_csv(os.path.join(args.out, "perbandingan_auto.csv"), arows)

    # ---- 01 loiter
    if lrows:
        P = np.array([r["navl1_period_s"] for r in lrows])
        names = [r["terbang"].replace("flight_", "") for r in lrows]
        col = lambda k: np.array([r[k] for r in lrows], dtype=float)
        fig, ax = plt.subplots(2, 2, figsize=(13, 9))
        a = ax[0, 0]
        a.plot(P, col("galat_rms_steady_m"), "o-", color="tab:green", label="RMS steady (putaran 2+)")
        a.plot(P, col("galat_max_steady_m"), "s--", color="tab:green", alpha=0.6, label="|galat| maks steady")
        a.axhline(args.tol, color="r", ls=":", label="toleransi %.0f m" % args.tol)
        label_points(a, P, col("galat_rms_steady_m"), names)
        a.set_title("Galat radius setelah entry")
        a.set_ylabel("m")
        a.legend(fontsize=8)
        a = ax[0, 1]
        a.plot(P, col("galat_rms_total_m"), "o-", color="tab:orange", label="RMS total (termasuk entry)")
        a.plot(P, col("galat_max_total_m"), "s--", color="tab:orange", alpha=0.6, label="|galat| maks total")
        label_points(a, P, col("galat_rms_total_m"), names)
        a.set_title("Galat radius seluruh loiter")
        a.set_ylabel("m")
        a.legend(fontsize=8)
        a = ax[1, 0]
        a.plot(P, col("konvergen_s"), "o-", color="tab:blue", label="waktu konvergen (|galat|<=5 m)")
        a2 = a.twinx()
        a2.plot(P, col("persen_lewat_tol_steady"), "s--", color="tab:red", label="%% steady di luar ±%.0f m" % args.tol)
        a2.set_ylabel("% waktu", color="tab:red")
        a.set_title("Konvergensi entry & pelanggaran toleransi")
        a.set_ylabel("s")
        h1, l1 = a.get_legend_handles_labels()
        h2, l2 = a2.get_legend_handles_labels()
        a.legend(h1 + h2, l1 + l2, fontsize=8)
        a = ax[1, 1]
        a.bar(P - 0.15, col("angin_mps"), width=0.3, color="0.6", label="angin perkiraan [m/s]")
        a.bar(P + 0.15, col("loiter_roll_track_rmse"), width=0.3, color="tab:purple", label="RMSE roll vs nav_roll [deg]")
        a.set_title("Kondisi pembanding (angin berbeda tiap terbang)")
        a.legend(fontsize=8)
        for a in ax.flat:
            a.set_xlabel("NAVL1_PERIOD [s]")
            a.set_xticks(P)
            a.grid(alpha=0.3)
        fig.suptitle("Perbandingan LOITER antar NAVL1_PERIOD")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "01_perbandingan_loiter.png"), dpi=120)
        plt.close(fig)

    # ---- 02 AUTO
    if arows:
        P = np.array([r["navl1_period_s"] for r in arows])
        names = [r["terbang"].replace("flight_", "") for r in arows]
        col = lambda k: np.array([r[k] for r in arows], dtype=float)
        fig, ax = plt.subplots(1, 3, figsize=(16, 5))
        a = ax[0]
        a.plot(P, col("cte_mean_abs_m"), "o-", label="|CTE| rata2")
        a.plot(P, col("cte_rms_m"), "s-", label="CTE RMS")
        a.plot(P, col("cte_max_abs_m"), "^--", label="|CTE| maks")
        a.axhline(45, color="r", ls=":", label="batas 45 m")
        label_points(a, P, col("cte_mean_abs_m"), names)
        a.set_title("Cross-track error AUTO")
        a.set_ylabel("m")
        a.legend(fontsize=8)
        a = ax[1]
        w = 0.8 / len(LEG_ORDER)
        for i, leg in enumerate(LEG_ORDER):
            a.bar(P - 0.4 + w * (i + 0.5), col("cte_mean_abs_%s" % leg), width=w, label=leg)
        a.set_title("|CTE| rata2 per leg")
        a.set_ylabel("m")
        a.legend(fontsize=8)
        a = ax[2]
        a.plot(P, col("auto_roll_track_rmse"), "o-", color="tab:purple", label="RMSE roll vs nav_roll [deg]")
        a2 = a.twinx()
        a2.plot(P, 100 * col("auto_nav_roll_sat"), "s--", color="tab:red", label="nav_roll jenuh [%]")
        a2.set_ylabel("%", color="tab:red")
        h1, l1 = a.get_legend_handles_labels()
        h2, l2 = a2.get_legend_handles_labels()
        a.legend(h1 + h2, l1 + l2, fontsize=8)
        a.set_title("Kendali roll di AUTO")
        for a in ax:
            a.set_xlabel("NAVL1_PERIOD [s]")
            a.set_xticks(P)
            a.grid(alpha=0.3)
        fig.suptitle("Perbandingan AUTO antar NAVL1_PERIOD")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "02_perbandingan_auto.png"), dpi=120)
        plt.close(fig)

    print("selesai ->", args.out)


if __name__ == "__main__":
    main()
