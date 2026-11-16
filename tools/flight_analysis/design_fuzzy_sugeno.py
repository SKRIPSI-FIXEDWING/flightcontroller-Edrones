#!/usr/bin/env python3
"""Desain fuzzy Sugeno orde-0 untuk periode L1 dari data uji L1 konvensional.

Hanya analisis offline: firmware tidak disentuh. Masukan = database_sampel.csv
hasil build_fuzzy_database.py (lima terbang 30/10/2026, periode 11..19 s).

Arsitektur (sama dengan catatan fuzzy-PID: masukan dinormalisasi, keluaran
ternormalisasi dikali faktor skala):
  masukan  e_f  = |e|            -> e_fn  = min(1, e_f / E_MAX)
           de_f = d|e|/dt        -> de_fn = clip(de_f / DE_MAX, -1, 1)
  himpunan e_fn  : Kecil, Sedang, Besar      titik 0,25 - 0,5 - 0,75
           de_fn : Negatif, Zero, Positif    titik -0,5 - 0 - 0,5
  inferensi       alpha_i = min(mu_A(e_fn), mu_B(de_fn))
  defuzzifikasi   z* = sum(alpha_i c_i) / sum(alpha_i),  c_i in {-1, -0,5, 0, 0,5, 1}
                  periode = P0 + S z*,  P0 = 15 s, S = 4 s  -> 11..19 s (rentang yang diterbangkan)

E_MAX dan DE_MAX ditetapkan dari jangkar data (lihat anchors()). Konsekuen
c_i tiap aturan dipilih dari data: untuk tiap daerah aturan dan tiap periode
dihitung biaya C = |e| rata-rata H detik ke depan + overshoot sisi seberang
dalam H detik, dibobot alpha_i sampel; periode dengan C terkecil dipakai.

Keluaran (folder -o):
  parameter_fuzzy.json      E_MAX, DE_MAX, himpunan, rule base, P0, S
  bukti_rule.csv            C, J, O, n_eff per aturan per periode (H utama)
  sensitivitas_horizon.csv  pilihan aturan untuk H = 4, 6, 8 s
  putar_ulang_periode.csv   periode Sugeno per sampel (putar ulang loop terbuka)
  contoh_perhitungan.txt    perhitungan langkah demi langkah untuk sampel nyata
  01..05_*.png

Run:
    python design_fuzzy_sugeno.py -o fuzzy_sugeno_2026-10-30
"""

import argparse
import collections
import csv
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DB_DEFAULT = os.path.join(HERE, "fuzzy_database_2026-10-30", "database_sampel.csv")

PERIODS = (11.0, 13.0, 15.0, 17.0, 19.0)
P0, S = 15.0, 4.0                      # periode dasar dan faktor skala -> 11..19 s
RECOVERY_BAND_M = 5.0                  # pita pulih yang dipakai build_fuzzy_database.py
HORIZON_S = 6.0                        # ~ waktu pulih rata-rata 6-8 s
HORIZONS_SENS = (4.0, 6.0, 8.0)
MIN_NEFF = 5.0                         # sampel efektif minimum agar periode ikut dipilih
E_NAMES = ("Kecil", "Sedang", "Besar")
D_NAMES = ("Negatif", "Zero", "Positif")


def up(x, a, b):
    return np.clip((np.asarray(x, float) - a) / (b - a), 0.0, 1.0)


def mu_e(u):
    """Kecil/Sedang/Besar pada e_fn (0..1)."""
    return np.array([1 - up(u, .25, .5), np.minimum(up(u, .25, .5), 1 - up(u, .5, .75)), up(u, .5, .75)])


def mu_de(y):
    """Negatif/Zero/Positif pada de_fn (-1..1)."""
    return np.array([1 - up(y, -.5, 0), np.minimum(up(y, -.5, 0), 1 - up(y, 0, .5)), up(y, 0, .5)])


def load(path):
    rows = list(csv.DictReader(open(path)))
    legs = collections.OrderedDict()
    for r in rows:
        legs.setdefault((float(r["period_s"]), r["leg"]), []).append(r)
    out = {}
    for k, rr in legs.items():
        out[k] = dict(t=np.array([float(r["t_leg_s"]) for r in rr]), e=np.array([float(r["e_m"]) for r in rr]),
                      de=np.array([float(r["de_mps"]) for r in rr]), fase=np.array([r["fase"] for r in rr]))
    return out


def anchors(legs):
    """E_MAX dan DE_MAX dari dua jangkar data masing-masing, dirata-rata lalu dibulatkan ke 1."""
    ae = np.concatenate([np.abs(d["e"]) for d in legs.values()])
    de = np.concatenate([d["de"] for d in legs.values()])
    fase = np.concatenate([d["fase"] for d in legs.values()])
    t = np.concatenate([d["t"] for d in legs.values()])
    tun, tr = fase == "tunak", fase == "transisi"
    # |e|: Kecil penuh sampai 0,25 E_MAX = pita pulih; Besar penuh mulai 0,75 E_MAX = median |e| transisi
    e_a1 = 4.0 * RECOVERY_BAND_M
    e_a2 = float(np.median(ae[tr])) / 0.75
    # d|e|/dt: crossover Zero (mu = 0,5) di +-0,25 DE_MAX = p95 |d|e|/dt| tunak;
    #          Negatif penuh di -0,5 DE_MAX = p10 d|e|/dt transisi sesudah 3 s awal leg (kembali ke garis)
    d_a1 = float(np.percentile(np.abs(de[tun]), 95)) / 0.25
    d_a2 = -float(np.percentile(de[tr & (t > 3.0)], 10)) / 0.5
    return dict(e_anchor_pita_pulih=e_a1, e_anchor_median_transisi=e_a2, E_MAX=float(round((e_a1 + e_a2) / 2)),
                de_anchor_p95_tunak=d_a1, de_anchor_p10_transisi=d_a2, DE_MAX=float(round((d_a1 + d_a2) / 2)),
                median_e_transisi=float(np.median(ae[tr])), p95_de_tunak=float(np.percentile(np.abs(de[tun]), 95)),
                p10_de_transisi_3s=float(np.percentile(de[tr & (t > 3.0)], 10)))


def normalize(abs_e, de, E_MAX, DE_MAX):
    return np.minimum(1.0, np.asarray(abs_e) / E_MAX), np.clip(np.asarray(de) / DE_MAX, -1.0, 1.0)


def future_samples(legs, H):
    """Per sampel: periode, |e|, d|e|/dt, J = |e| rata-rata (t, t+H], O = overshoot sisi seberang dalam (t, t+H]."""
    out = []
    for (p, _), d in legs.items():
        t, e, de = d["t"], d["e"], d["de"]
        for k in range(len(t)):
            m = (t > t[k]) & (t <= t[k] + H)
            if t[-1] < t[k] + H or m.sum() < 5:
                continue
            s0 = np.sign(e[k]) if e[k] != 0 else 1.0
            opp = e[m][np.sign(e[m]) == -s0]
            out.append((p, abs(e[k]), de[k], np.abs(e[m]).mean(), np.abs(opp).max() if len(opp) else 0.0))
    return np.array(out)


def rule_evidence(legs, H, E_MAX, DE_MAX):
    A = future_samples(legs, H)
    en, dn = normalize(A[:, 1], A[:, 2], E_MAX, DE_MAX)
    me, md = mu_e(en), mu_de(dn)
    ev = {}
    for i in range(3):
        for j in range(3):
            w = np.minimum(me[i], md[j])
            for p in PERIODS:
                m = A[:, 0] == p
                n = float(w[m].sum())
                J = float((w[m] * A[m, 3]).sum() / n) if n > 0 else float("nan")
                O = float((w[m] * A[m, 4]).sum() / n) if n > 0 else float("nan")
                ev[(i, j, p)] = dict(n_eff=n, J=J, O=O, C=J + O)
    return ev, len(A)


def choose_rules(ev):
    """Periode dengan C terkecil di antara periode ber-n_eff >= MIN_NEFF; keyakinan dari selisih ke peringkat 2."""
    rules = {}
    for i in range(3):
        for j in range(3):
            cand = sorted((ev[(i, j, p)]["C"], p) for p in PERIODS if ev[(i, j, p)]["n_eff"] >= MIN_NEFF)
            if not cand:
                rules[(i, j)] = dict(periode=P0, c=0.0, keyakinan="tidak ada data (pakai P0)", margin=float("nan"))
                continue
            (c1, p1) = cand[0]
            margin = (cand[1][0] - c1) / c1 if len(cand) > 1 else float("nan")
            nwin = ev[(i, j, p1)]["n_eff"]
            conf = "kuat" if margin >= 0.15 and nwin >= 15 else ("sedang" if margin >= 0.05 and nwin >= 10 else "lemah")
            rules[(i, j)] = dict(periode=p1, c=(p1 - P0) / S, keyakinan=conf, margin=margin)
    return rules


def infer(abs_e, de, rules, E_MAX, DE_MAX):
    en, dn = normalize(abs_e, de, E_MAX, DE_MAX)
    me, md = mu_e(en), mu_de(dn)
    alpha = np.minimum(me[:, None], md[None, :])          # alpha[i, j] = min(mu_e_i, mu_de_j)
    c = np.array([[rules[(i, j)]["c"] for j in range(3)] for i in range(3)])
    z = float((alpha * c).sum() / alpha.sum())
    return dict(e_fn=float(en), de_fn=float(dn), mu_e=me, mu_de=md, alpha=alpha, c=c, z=z, periode=P0 + S * z)


def explain(sample, rules, E_MAX, DE_MAX):
    abs_e, de, label = sample["abs_e"], sample["de"], sample["label"]
    r = infer(abs_e, de, rules, E_MAX, DE_MAX)
    L = ["=" * 78, "%s  (%s)" % (label, sample["src"]), "=" * 78,
         "LANGKAH 1 - NORMALISASI",
         "  e_f  = |e|     = %.2f m       e_fn  = min(1, %.2f / %.0f) = %.3f" % (abs_e, abs_e, E_MAX, r["e_fn"]),
         "  de_f = d|e|/dt = %+.2f m/s    de_fn = clip(%+.2f / %.0f, -1, 1) = %+.3f" % (de, de, DE_MAX, r["de_fn"]),
         "LANGKAH 2 - FUZZIFIKASI"]
    L += ["  mu_%-7s(e_fn)  = %.3f" % (n, v) for n, v in zip(E_NAMES, r["mu_e"])]
    L += ["  mu_%-7s(de_fn) = %.3f" % (n, v) for n, v in zip(D_NAMES, r["mu_de"])]
    L += ["LANGKAH 3 - INFERENSI (alpha = min) dan LANGKAH 4 - DEFUZZIFIKASI (weighted average)",
          "  %-22s %-20s %7s %6s %9s" % ("aturan aktif", "min(...)", "alpha", "c_i", "alpha*c_i")]
    num = 0.0
    for i in range(3):
        for j in range(3):
            a = r["alpha"][i, j]
            if a > 0:
                num += a * r["c"][i, j]
                L.append("  %-22s min(%.3f; %.3f) %7.3f %+6.2f %+9.4f" % (
                    "%s & %s" % (E_NAMES[i], D_NAMES[j]), r["mu_e"][i], r["mu_de"][j], a, r["c"][i, j], a * r["c"][i, j]))
    L += ["  sum alpha = %.3f   sum alpha*c = %+.4f" % (r["alpha"].sum(), num),
          "  z* = %+.4f / %.3f = %+.4f" % (num, r["alpha"].sum(), r["z"]),
          "  periode = P0 + S z* = %.0f + %.0f x (%+.4f) = %.2f s" % (P0, S, r["z"], r["periode"]), ""]
    return "\n".join(L)


def pick_examples(legs):
    """Tiga sampel nyata: tunak, sedang kembali ke garis, awal leg (geometri antisipasi belokan)."""
    allr = [(p, leg, d["t"][k], abs(d["e"][k]), d["de"][k]) for (p, leg), d in legs.items() for k in range(len(d["t"]))]
    targets = [("Contoh A: tunak dekat garis", 2.0, -0.5), ("Contoh B: sedang kembali ke garis", 12.0, -5.0),
               ("Contoh C: awal leg sesudah ganti WP", 30.0, -18.0)]
    out = []
    for label, te, td in targets:
        best = min(allr, key=lambda s: ((s[3] - te) / 5.0) ** 2 + ((s[4] - td) / 2.0) ** 2)
        out.append(dict(label=label, abs_e=best[3], de=best[4],
                        src="periode %g s, leg %s, t_leg = %.1f s" % (best[0], best[1], best[2])))
    return out


def plots(out, legs, prm, rules, ev):
    E_MAX, DE_MAX = prm["E_MAX"], prm["DE_MAX"]
    ae = np.concatenate([np.abs(d["e"]) for d in legs.values()])
    de = np.concatenate([d["de"] for d in legs.values()])
    cols_e, cols_d = ("tab:blue", "tab:orange", "tab:red"), ("tab:green", "0.45", "tab:purple")

    fig, ax = plt.subplots(1, 2, figsize=(12, 4))
    for a, data, lo, hi, scale, fn, names, cols, xl in (
            (ax[0], ae, 0, 45, E_MAX, lambda x: mu_e(np.minimum(1, x / E_MAX)), E_NAMES, cols_e, "|e| (m)"),
            (ax[1], de, -24, 24, DE_MAX, lambda x: mu_de(np.clip(x / DE_MAX, -1, 1)), D_NAMES, cols_d, "d|e|/dt (m/s)")):
        h, b = np.histogram(data, bins=np.linspace(lo, hi, 49))
        a.bar(b[:-1], h / h.max(), width=b[1] - b[0], align="edge", color="0.85", label="sebaran data (ternormalisasi)")
        x = np.linspace(lo, hi, 600)
        for k in range(3):
            a.plot(x, fn(x)[k], color=cols[k], lw=2.2, label=names[k])
        a.set_xlabel(xl)
        a.set_ylabel("derajat keanggotaan")
        a.grid(alpha=.3)
        a.legend(fontsize=8)
    ax[0].set_title("|e|: E_MAX = %g m (titik %g / %g / %g m)" % (E_MAX, .25 * E_MAX, .5 * E_MAX, .75 * E_MAX), fontsize=10)
    ax[1].set_title("d|e|/dt: DE_MAX = %g m/s (titik %g / 0 / %g m/s)" % (DE_MAX, -.5 * DE_MAX, .5 * DE_MAX), fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "01_fungsi_keanggotaan.png"), dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(3, 3, figsize=(11, 8), sharex=True)
    for i in range(3):
        for j in range(3):
            a = axes[i, j]
            Cs = [ev[(i, j, p)]["C"] for p in PERIODS]
            ns = [ev[(i, j, p)]["n_eff"] for p in PERIODS]
            bars = a.bar([str(int(p)) for p in PERIODS], Cs,
                         color=["tab:blue" if p == rules[(i, j)]["periode"] else ("0.75" if n >= MIN_NEFF else "white")
                                for p, n in zip(PERIODS, ns)], edgecolor="0.4")
            for b_, n in zip(bars, ns):
                a.text(b_.get_x() + b_.get_width() / 2, b_.get_height(), "n=%.0f" % n, ha="center", va="bottom", fontsize=7)
            a.set_title("%s & %s -> %g s (%s)" % (E_NAMES[i], D_NAMES[j], rules[(i, j)]["periode"], rules[(i, j)]["keyakinan"]), fontsize=9)
            a.grid(alpha=.3, axis="y")
            if j == 0:
                a.set_ylabel("biaya C (m)")
    for a in axes[-1]:
        a.set_xlabel("periode L1 (s)")
    fig.suptitle("Bukti per aturan: C = |e| rata-rata %g s ke depan + overshoot seberang (biru = dipilih, putih = n_eff < %g)"
                 % (HORIZON_S, MIN_NEFF), fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "02_bukti_rule.png"), dpi=130)
    plt.close(fig)

    E, Dd = np.meshgrid(np.linspace(0, 30, 121), np.linspace(-20, 20, 161))
    P = np.vectorize(lambda a, b: infer(a, b, rules, E_MAX, DE_MAX)["periode"])(E, Dd)
    fig, ax = plt.subplots(figsize=(7.5, 5))
    cs = ax.contourf(E, Dd, P, levels=np.arange(10.75, 19.5, 0.5), cmap="viridis")
    fig.colorbar(cs, label="periode L1 (s)")
    ax.scatter(np.minimum(ae, 30), np.clip(de, -20, 20), s=2, color="w", alpha=.35, label="sampel terbang")
    ax.set_xlabel("|e| (m)")
    ax.set_ylabel("d|e|/dt (m/s)   negatif = mendekati garis")
    ax.set_title("Permukaan kendali Sugeno: periode L1 = f(|e|, d|e|/dt)")
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "03_permukaan_kendali.png"), dpi=140)
    plt.close(fig)

    leg_names = sorted({leg for (_, leg) in legs}, key=lambda x: int(x.split("WP")[-1]))
    fig, axes = plt.subplots(len(leg_names), 2, figsize=(12, 2.3 * len(leg_names)), sharex="col")
    for r_, leg in enumerate(leg_names):
        d = legs.get((P0, leg))
        if d is None:
            continue
        Pz = [infer(abs(e), x, rules, E_MAX, DE_MAX)["periode"] for e, x in zip(d["e"], d["de"])]
        axes[r_, 0].plot(d["t"], d["e"], color="k", lw=1.3)
        axes[r_, 0].axhspan(-RECOVERY_BAND_M, RECOVERY_BAND_M, color="g", alpha=.1)
        axes[r_, 0].set_ylabel("e (m)")
        axes[r_, 0].set_title("terbang periode %g s, %s" % (P0, leg), fontsize=9, loc="left")
        axes[r_, 1].plot(d["t"], Pz, color="tab:blue", lw=1.6)
        axes[r_, 1].axhline(P0, color="0.5", ls="--", lw=1)
        axes[r_, 1].set_ylim(10.5, 19.5)
        axes[r_, 1].set_ylabel("periode (s)")
        for a in axes[r_]:
            a.grid(alpha=.3)
    axes[-1, 0].set_xlabel("waktu sejak awal leg (s)")
    axes[-1, 1].set_xlabel("waktu sejak awal leg (s)")
    fig.suptitle("Putar ulang loop terbuka: e terbang 15 s (kiri) dan periode yang akan diminta Sugeno (kanan)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "04_putar_ulang_15s.png"), dpi=120)
    plt.close(fig)

    allP = np.array([infer(abs(e), x, rules, E_MAX, DE_MAX)["periode"] for d in legs.values() for e, x in zip(d["e"], d["de"])])
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.hist(allP, bins=np.arange(10.75, 19.5, 0.5), color="tab:blue", edgecolor="w")
    ax.set_xlabel("periode keluaran Sugeno (s)")
    ax.set_ylabel("jumlah sampel")
    ax.set_title("Sebaran keluaran pada %d sampel kelima terbang (putar ulang)" % len(allP), fontsize=10)
    ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "05_histogram_keluaran.png"), dpi=140)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", default=DB_DEFAULT)
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    legs = load(args.db)
    prm = anchors(legs)
    E_MAX, DE_MAX = prm["E_MAX"], prm["DE_MAX"]
    print("jangkar:", json.dumps({k: round(v, 2) for k, v in prm.items()}))

    ev, n_fut = rule_evidence(legs, HORIZON_S, E_MAX, DE_MAX)
    rules = choose_rules(ev)
    print("sampel dengan horizon %g s: %d" % (HORIZON_S, n_fut))
    for i in range(3):
        print("  %-7s" % E_NAMES[i], "  ".join("%-8s -> %4g s (c=%+.1f, %s, margin %.0f%%)" % (
            D_NAMES[j], rules[(i, j)]["periode"], rules[(i, j)]["c"], rules[(i, j)]["keyakinan"],
            100 * rules[(i, j)]["margin"]) for j in range(3)))

    with open(os.path.join(args.out, "bukti_rule.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["aturan_e", "aturan_de", "periode_s", "n_eff", "J_m", "O_m", "C_m", "dipilih"])
        for i in range(3):
            for j in range(3):
                for p in PERIODS:
                    x = ev[(i, j, p)]
                    w.writerow([E_NAMES[i], D_NAMES[j], p, round(x["n_eff"], 1), round(x["J"], 2), round(x["O"], 2),
                                round(x["C"], 2), int(p == rules[(i, j)]["periode"])])

    with open(os.path.join(args.out, "sensitivitas_horizon.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["aturan"] + ["H_%gs" % h for h in HORIZONS_SENS])
        sens = {h: choose_rules(rule_evidence(legs, h, E_MAX, DE_MAX)[0]) for h in HORIZONS_SENS}
        for i in range(3):
            for j in range(3):
                w.writerow(["%s & %s" % (E_NAMES[i], D_NAMES[j])] + [sens[h][(i, j)]["periode"] for h in HORIZONS_SENS])
                print("  sensitivitas %-18s" % ("%s & %s" % (E_NAMES[i], D_NAMES[j])),
                      [sens[h][(i, j)]["periode"] for h in HORIZONS_SENS])

    with open(os.path.join(args.out, "putar_ulang_periode.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["period_s", "leg", "t_leg_s", "e_m", "de_mps", "e_fn", "de_fn", "z", "periode_sugeno_s"])
        for (p, leg), d in legs.items():
            for k in range(len(d["t"])):
                r = infer(abs(d["e"][k]), d["de"][k], rules, E_MAX, DE_MAX)
                w.writerow([p, leg, d["t"][k], d["e"][k], d["de"][k], round(r["e_fn"], 4), round(r["de_fn"], 4),
                            round(r["z"], 4), round(r["periode"], 3)])

    ex = pick_examples(legs)
    with open(os.path.join(args.out, "contoh_perhitungan.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(explain(s, rules, E_MAX, DE_MAX) for s in ex))
    print("\n".join(explain(s, rules, E_MAX, DE_MAX) for s in ex))

    with open(os.path.join(args.out, "parameter_fuzzy.json"), "w") as f:
        json.dump(dict(
            sumber=os.path.relpath(args.db, HERE), horizon_s=HORIZON_S, min_n_eff=MIN_NEFF, jangkar=prm,
            E_MAX_m=E_MAX, DE_MAX_mps=DE_MAX,
            himpunan_e_fn={"Kecil": [0, 0, .25, .5], "Sedang": [.25, .5, .5, .75], "Besar": [.5, .75, 1, 1]},
            himpunan_de_fn={"Negatif": [-1, -1, -.5, 0], "Zero": [-.5, 0, 0, .5], "Positif": [0, .5, 1, 1]},
            operator_dan="min", defuzzifikasi="weighted average (Sugeno orde-0)", P0_s=P0, S_s=S,
            rule_base=[dict(e=E_NAMES[i], de=D_NAMES[j], c=rules[(i, j)]["c"], periode_s=rules[(i, j)]["periode"],
                            keyakinan=rules[(i, j)]["keyakinan"]) for i in range(3) for j in range(3)]), f, indent=2)

    plots(args.out, legs, prm, rules, ev)
    allP = np.array([infer(abs(e), x, rules, E_MAX, DE_MAX)["periode"] for d in legs.values() for e, x in zip(d["e"], d["de"])])
    print("keluaran putar ulang: min %.2f  p5 %.2f  median %.2f  p95 %.2f  maks %.2f" % tuple(np.percentile(allP, [0, 5, 50, 95, 100])))
    print("selesai ->", args.out)


if __name__ == "__main__":
    main()
