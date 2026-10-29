#!/usr/bin/env python3
"""Perbandingan loiter antar terbang (semua tanggal) dari CSV analyze_tlog_loiter.py.

Membaca <folder>/loiter/ringkasan_loiter.csv (atau <folder>/ringkasan_loiter.csv
untuk folder lama tanpa subfolder loiter/) dan ringkasan_steady.csv, lalu
menyusun satu baris per terbang dengan label kondisinya. Folder tanpa
ringkasan_steady.csv (loiter < 1 putaran) tetap masuk dengan kolom steady kosong.

Keluaran (folder -o):
  perbandingan_loiter_semua.csv
  01_loiter_steady_semua.png   galat steady (RMS, maks, % keluar), nav_roll jenuh dan GS maks per terbang
  02_loiter_entry_semua.png    galat seluruh loiter (termasuk entry) dan waktu konvergen per terbang

Pemakaian:
    python compare_loiter_flights.py "flight_2026-10-26_b=26b P12" "flight_2026-10-29_d=29d fuzzy+gangguan" ... [-o out_dir]
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


def read_first(*paths):
    for p in paths:
        if os.path.exists(p):
            with open(p, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            if rows:
                return rows[0]
    return None


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("terbang", nargs="+", help='folder=label, mis. "flight_2026-10-26_b=26b P12"')
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "perbandingan_loiter_semua"))
    ap.add_argument("--tol", type=float, default=10.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows = []
    for item in args.terbang:
        folder, label = item.split("=", 1)
        path = folder if os.path.isabs(folder) else os.path.join(HERE, folder)
        s = read_first(os.path.join(path, "loiter", "ringkasan_loiter.csv"), os.path.join(path, "ringkasan_loiter.csv"))
        if s is None:
            print("CATATAN: %s tanpa ringkasan loiter, dilewati" % folder)
            continue
        st = read_first(os.path.join(path, "loiter", "ringkasan_steady.csv"), os.path.join(path, "ringkasan_steady.csv")) or {}
        rows.append(dict(
            terbang=os.path.basename(folder.rstrip("/")), label=label,
            radius_cmd_m=num(s["radius_cmd"]), jumlah_putaran=num(s["jumlah_putaran"]),
            angin_mps=num(s["angin_perkiraan"]), airspeed_mean=num(s["airspeed_mean"]), gs_mean=num(s["gs_mean"]),
            konvergen_s=num(s["konvergen_s"]), galat_rms_total_m=num(s["galat_rms"]),
            galat_max_total_m=num(s["galat_max_abs"]), persen_lewat_tol_total=num(s["persen_lewat_tol"]),
            steady_durasi_s=num(st.get("durasi_s")), steady_putaran=num(st.get("putaran")),
            steady_galat_rms_m=num(st.get("galat_rms_m")), steady_galat_max_m=num(st.get("galat_max_abs_m")),
            steady_persen_lewat_tol=num(st.get("persen_lewat_tol")),
            steady_nav_roll_sat=num(st.get("nav_roll_sat")), steady_gs_max=num(st.get("gs_max")),
            steady_roll_track_rmse=num(st.get("roll_track_rmse")), steady_alt_galat_rms=num(st.get("alt_galat_rms")),
        ))
    with open(os.path.join(args.out, "perbandingan_loiter_semua.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if isinstance(v, float) and math.isnan(v) else v) for k, v in r.items()})

    lab = [r["label"] for r in rows]
    x = np.arange(len(rows))
    col = lambda k: np.array([r[k] for r in rows], dtype=float)

    fig, ax = plt.subplots(3, 1, figsize=(max(10, 0.9 * len(rows) + 3), 11), sharex=True)
    ax[0].bar(x - 0.2, col("steady_galat_rms_m"), width=0.4, color="tab:green", label="RMS steady")
    ax[0].bar(x + 0.2, col("steady_galat_max_m"), width=0.4, color="tab:green", alpha=0.45, label="|galat| maks steady")
    ax[0].axhline(args.tol, color="r", ls=":", label="toleransi %.0f m" % args.tol)
    ax[0].set_ylabel("galat radius [m]")
    ax[0].set_title("Loiter steady (mulai putaran ke-2) per terbang -- kosong = loiter < 1 putaran")
    ax[0].legend(fontsize=8)
    ax[1].bar(x, col("steady_persen_lewat_tol"), color="tab:red", alpha=0.7)
    ax[1].set_ylabel("%% waktu di luar ±%.0f m" % args.tol)
    ax[2].bar(x - 0.2, 100 * col("steady_nav_roll_sat"), width=0.4, color="tab:purple", label="nav_roll jenuh [%]")
    a2 = ax[2].twinx()
    a2.plot(x, col("steady_gs_max"), "ko-", label="GS maks steady [m/s]")
    a2.plot(x, col("angin_mps"), "s--", color="0.5", label="angin perkiraan [m/s]")
    a2.set_ylabel("m/s")
    h1, l1 = ax[2].get_legend_handles_labels()
    h2, l2 = a2.get_legend_handles_labels()
    ax[2].legend(h1 + h2, l1 + l2, fontsize=8, loc="lower center", ncol=3)
    ax[2].set_ylabel("%")
    ax[2].set_xticks(x)
    ax[2].set_xticklabels(lab, rotation=30, ha="right", fontsize=8)
    for a in ax:
        a.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "01_loiter_steady_semua.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(2, 1, figsize=(max(10, 0.9 * len(rows) + 3), 8), sharex=True)
    ax[0].bar(x - 0.2, col("galat_rms_total_m"), width=0.4, color="tab:orange", label="RMS total")
    ax[0].bar(x + 0.2, col("galat_max_total_m"), width=0.4, color="tab:orange", alpha=0.45, label="|galat| maks total")
    ax[0].set_ylabel("galat radius [m]")
    ax[0].set_title("Seluruh loiter (termasuk masuk ke lingkaran) per terbang")
    ax[0].legend(fontsize=8)
    ax[1].bar(x, col("konvergen_s"), color="tab:blue")
    ax[1].set_ylabel("waktu konvergen [s]")
    ax[1].set_xticks(x)
    ax[1].set_xticklabels(lab, rotation=30, ha="right", fontsize=8)
    for a in ax:
        a.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "02_loiter_entry_semua.png"), dpi=120)
    plt.close(fig)
    print("selesai ->", args.out)


if __name__ == "__main__":
    main()
