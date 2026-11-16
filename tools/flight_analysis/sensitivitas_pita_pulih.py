#!/usr/bin/env python3
"""Dasar dan uji sensitivitas pita pulih (|e| <= BAND selama HOLD detik).

Pita pulih dipakai di analyze_tlog_l1_auto.py, build_fuzzy_database.py (5 m, 2 s)
untuk waktu pulih, fase tunak/transisi, dan jangkar E_MAX desain fuzzy. Skrip ini
mengukur dari database_sampel.csv:
  1. derau cepat e: e dikurangi rata-rata bergerak 1 s (5 sampel);
  2. kecepatan lateral |de/dt| saat pesawat memotong garis pada fase transisi,
     dan waktu yang dibutuhkan untuk melintasi pita selebar 2*BAND;
  3. untuk BAND = 3, 4, 5, 6, 8 m dan HOLD = 1, 2, 3 s: waktu pulih dan |e| tunak
     per periode, jumlah leg yang tidak pulih, E_MAX/DE_MAX dan rule base Sugeno.

Keluaran: <out>/sensitivitas_pita_pulih.csv dan ringkasan di layar.

Run:
    python sensitivitas_pita_pulih.py -o fuzzy_sugeno_2026-10-30
"""

import argparse
import collections
import csv
import os

import numpy as np

import design_fuzzy_sugeno as DZ

BANDS = (3.0, 4.0, 5.0, 6.0, 8.0)
HOLDS = (1.0, 2.0, 3.0)


def load(path):
    legs = collections.OrderedDict()
    for r in csv.DictReader(open(path)):
        legs.setdefault((float(r["period_s"]), r["leg"]), []).append(r)
    return {k: dict(t=np.array([float(r["t_leg_s"]) for r in v]), e=np.array([float(r["e_m"]) for r in v]),
                    de=np.array([float(r["de_mps"]) for r in v]), er=np.array([float(r["e_rate_mps"]) for r in v]),
                    fase=np.array([r["fase"] for r in v]))
            for k, v in legs.items()}


def recovery_index(t, e, band, hold):
    inside = np.abs(e) <= band
    for j in range(len(t)):
        if inside[j]:
            jj = np.where(t >= t[j] + hold)[0]
            if len(jj) and inside[j:jj[0] + 1].all():
                return j
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", default=DZ.DB_DEFAULT)
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()
    legs = load(args.db)

    res = np.concatenate([d["e"][2:-2] - np.convolve(d["e"], np.ones(5) / 5, mode="valid")
                          for d in legs.values() if len(d["e"]) >= 10])
    print("derau cepat e (e - rata-rata 1 s): std %.2f m, p99 |.| %.2f m" % (res.std(), np.percentile(np.abs(res), 99)))

    cross = np.array([abs(d["er"][i]) for d in legs.values() for i in np.where(np.diff(np.sign(d["e"])) != 0)[0]
                      if d["fase"][i] == "transisi"])
    print("memotong garis saat transisi: n = %d, |de/dt| p10 / p50 / p90 = %.1f / %.1f / %.1f m/s"
          % ((len(cross),) + tuple(np.percentile(cross, [10, 50, 90]))))
    for band in BANDS:
        print("  pita +-%g m dilintasi dalam %.2f s (p50) / %.2f s (kecepatan p10)"
              % (band, 2 * band / np.median(cross), 2 * band / np.percentile(cross, 10)))

    rows = []
    for band in BANDS:
        for hold in HOLDS:
            L2, pul, tun, gagal = {}, collections.defaultdict(list), collections.defaultdict(list), collections.Counter()
            for k, d in legs.items():
                j = recovery_index(d["t"], d["e"], band, hold)
                fase = np.array(["transisi"] * len(d["t"]), dtype=object)
                if j is None:
                    gagal[k[0]] += 1
                else:
                    fase[j:] = "tunak"
                    pul[k[0]].append(d["t"][j] - d["t"][0])
                    tun[k[0]] += list(np.abs(d["e"][j:]))
                L2[k] = dict(d, fase=fase)
            DZ.RECOVERY_BAND_M = band
            a = DZ.anchors(L2)
            rules = DZ.choose_rules(DZ.rule_evidence(L2, DZ.HORIZON_S, a["E_MAX"], a["DE_MAX"])[0])
            row = dict(pita_m=band, tahan_s=hold, E_MAX=a["E_MAX"], DE_MAX=a["DE_MAX"],
                       rule_base=" ".join("%g" % rules[(i, j)]["periode"] for i in range(3) for j in range(3)))
            for p in DZ.PERIODS:
                row["pulih_%gs" % p] = round(float(np.mean(pul[p])), 1) if pul[p] else ""
                row["e_tunak_%gs" % p] = round(float(np.mean(tun[p])), 2) if tun[p] else ""
                row["leg_tak_pulih_%gs" % p] = gagal[p]
            rows.append(row)
    DZ.RECOVERY_BAND_M = 5.0

    with open(os.path.join(args.out, "sensitivitas_pita_pulih.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("\npita tahan | pulih rata-rata 11..19 s | |e| tunak 11..19 s | leg tak pulih 11..19 | E_MAX DE_MAX | rule R1..R9")
    for r in rows:
        print("%3g  %3g | %s | %s | %s | %g %g | %s" % (
            r["pita_m"], r["tahan_s"], " ".join("%4s" % r["pulih_%gs" % p] for p in DZ.PERIODS),
            " ".join("%4s" % r["e_tunak_%gs" % p] for p in DZ.PERIODS),
            " ".join("%d" % r["leg_tak_pulih_%gs" % p] for p in DZ.PERIODS), r["E_MAX"], r["DE_MAX"], r["rule_base"]))
    print("selesai ->", args.out)


if __name__ == "__main__":
    main()
