#!/usr/bin/env python3
"""Database penyusunan fuzzy dari uji L1 konvensional dengan beberapa periode.

Menggabungkan hasil analyze_tlog_l1_auto.py (cte_per_sampel.csv per terbang)
dengan tlog-nya, lalu untuk tiap periode L1 menghitung:
  e  = cross-track error (m, + = kanan garis), |e|
  de = d|e|/dt (m/s, + = menjauhi garis) = sign(e) * (v_GPS . n), proyeksi
       kecepatan GPS ke normal garis leg -- sama dengan xtrack_vel_mps firmware.
       Turunan numerik posisi 5 Hz memberi puncak palsu > groundspeed.
  fase per sampel: "transisi" (dari awal leg sampai |e| <= 5 m bertahan 2 s)
       atau "tunak" (sesudahnya)
Leg masuk->WP1 dibuang (mode langsung ke WP, tidak ada garis yang dilacak).

Keluaran (folder -o):
  database_sampel.csv     satu baris per sampel (bahan himpunan fuzzy e, de)
  metrik_transisi.csv     satu baris per leg per periode (e awal, overshoot seberang, pulih, ...)
  ringkasan_per_period.csv
  01_boxplot_e.png, 02_bidang_fase_e_de.png, 03_tren_metrik.png,
  04_histogram_e_de.png, 05_cte_vs_waktu_leg.png

Run:
    python build_fuzzy_database.py -o fuzzy_database_2026-10-30 \
        "11|flight_2026-10-30_P11|C:/.../2026-10-30 10-01-14.tlog" ...
"""

import argparse
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

RECOVERY_BAND_M = 5.0
RECOVERY_HOLD_S = 2.0
CTE_LIMIT_M = 45.0
E_BANDS = (5.0, 15.0, 45.0)


def read_tlog(path):
    """Roll, nav_roll, GPS course/speed; waktu relatif = ATTITUDE pertama, sama dengan analyze_tlog_l1_auto."""
    conn = mavutil.mavlink_connection(path, dialect="ardupilotmega")
    att, nav, gps = [], [], []
    t0 = None
    while True:
        m = conn.recv_match(type=["ATTITUDE", "NAV_CONTROLLER_OUTPUT", "GPS_RAW_INT", "VFR_HUD"])
        if m is None:
            break
        if t0 is None:
            if m.get_type() != "ATTITUDE":
                continue
            t0 = m._timestamp
        t = m._timestamp - t0
        typ = m.get_type()
        if typ == "ATTITUDE":
            att.append((t, np.degrees(m.roll)))
        elif typ == "NAV_CONTROLLER_OUTPUT":
            nav.append((t, m.nav_roll))
        elif typ == "GPS_RAW_INT":
            gps.append((t, m.vel / 100.0, m.cog / 100.0))
    return np.array(att), np.array(nav), np.array(gps)


def wind_fit(gps, t_a, t_b):
    """Groundspeed = Va + W cos(course - arah angin): kuadrat terkecil."""
    s = (gps[:, 0] >= t_a) & (gps[:, 0] <= t_b) & (gps[:, 1] > 3)
    c = np.radians(gps[s, 2])
    X = np.c_[np.ones(s.sum()), np.cos(c), np.sin(c)]
    k, *_ = np.linalg.lstsq(X, gps[s, 1], rcond=None)
    return float(k[0]), float(np.hypot(k[1], k[2])), float((np.degrees(np.arctan2(k[2], k[1])) + 180) % 360)


def recovery_index(t, e):
    inside = np.abs(e) <= RECOVERY_BAND_M
    for j in range(len(t)):
        if inside[j]:
            jj = np.where(t >= t[j] + RECOVERY_HOLD_S)[0]
            if len(jj) and inside[j:jj[0] + 1].all():
                return j
    return None


def process(period, folder, tlog):
    rows = list(csv.DictReader(open(os.path.join(folder, "cte_per_sampel.csv"))))
    att, nav, gps = read_tlog(tlog)
    samples, legs = [], []
    seqs = sorted({int(r["wp_aktif"]) for r in rows if int(r["wp_aktif"]) > 1})
    for seq in seqs:
        rr = [r for r in rows if int(r["wp_aktif"]) == seq]
        t = np.array([float(r["t_s"]) for r in rr])
        e = np.array([float(r["cte_m(+kanan)"]) for r in rr])
        gs = np.array([float(r["groundspeed_mps"]) for r in rr])
        l1 = np.array([float(r["l1_distance_m"]) for r in rr])
        if len(t) < 5:
            continue
        # Arah leg dari (x, y) = A + along*u + e*n, u = (c, s), n = (s, -c)
        # (konvensi signed_cte: e > 0 = kanan garis), kuadrat terkecil.
        x = np.array([float(r["x_timur_m"]) for r in rr])
        y = np.array([float(r["y_utara_m"]) for r in rr])
        along = np.array([float(r["along_m"]) for r in rr])
        one, zero = np.ones_like(x), np.zeros_like(x)
        M = np.r_[np.c_[one, zero, along, e], np.c_[zero, one, -e, along]]
        sol, *_ = np.linalg.lstsq(M, np.r_[x, y], rcond=None)
        c_u, s_u = sol[2], sol[3]
        nrm = np.hypot(c_u, s_u)
        n_vec = np.array([s_u, -c_u]) / nrm
        vel = np.interp(t, gps[:, 0], gps[:, 1])
        cog = np.radians(np.interp(t, gps[:, 0], np.degrees(np.unwrap(np.radians(gps[:, 2])))))
        e_rate = vel * np.sin(cog) * n_vec[0] + vel * np.cos(cog) * n_vec[1]   # de/dt bertanda
        de = np.sign(e) * e_rate                                              # d|e|/dt
        roll = np.interp(t, att[:, 0], att[:, 1])
        nav_roll = np.interp(t, nav[:, 0], nav[:, 1])
        j_rec = recovery_index(t, e)
        fase = np.array(["transisi"] * len(t), dtype=object)
        if j_rec is not None:
            fase[j_rec:] = "tunak"
        for k in range(len(t)):
            samples.append(dict(period_s=period, leg="WP%d->WP%d" % (seq - 1, seq), t_s=round(t[k], 2),
                                t_leg_s=round(t[k] - t[0], 2), e_m=round(e[k], 2), abs_e_m=round(abs(e[k]), 2),
                                de_mps=round(de[k], 3), e_rate_mps=round(e_rate[k], 3), fase=fase[k],
                                gs_mps=round(gs[k], 2), l1_dist_m=round(l1[k], 1),
                                nav_roll_deg=round(nav_roll[k], 1), roll_deg=round(roll[k], 1)))
        i_pk = int(np.argmax(np.abs(e)))
        # Overshoot sejati: simpangan maksimum di SISI SEBERANG garis sesudah
        # pertama kali memotongnya. |e| di awal leg bukan overshoot: dengan
        # WP_TURN_ANT, WP berganti ~d m sebelum WP sehingga e awal ~ d sin(dpsi).
        s0 = np.sign(e[0]) if e[0] != 0 else 1.0
        cross = np.where(np.sign(e) == -s0)[0]
        overshoot = float(np.max(np.abs(e[cross]))) if len(cross) else 0.0
        steady = np.abs(e[j_rec:]) if j_rec is not None else np.array([])
        legs.append(dict(period_s=period, leg="WP%d->WP%d" % (seq - 1, seq), durasi_s=round(t[-1] - t[0], 1),
                         e_awal_m=round(e[0], 1), e_puncak_m=round(abs(e[i_pk]), 1),
                         t_puncak_s=round(t[i_pk] - t[0], 1),
                         overshoot_seberang_m=round(overshoot, 1),
                         pulih_s=round(t[j_rec] - t[0], 1) if j_rec is not None else "",
                         de_maks_mps=round(float(np.max(np.abs(de))), 2),
                         zero_cross=int(np.sum(np.diff(np.sign(e)) != 0)),
                         e_tunak_mean_m=round(float(steady.mean()), 2) if len(steady) else "",
                         nav_roll_jenuh_frac=round(float(np.mean(np.abs(nav_roll) >= 44.0)), 2)))
    t_all = np.array([s["t_s"] for s in samples])
    wind = wind_fit(gps, t_all.min(), t_all.max())
    return samples, legs, wind


def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


def summarize(period, samples, legs, wind):
    e = np.array([s["e_m"] for s in samples])
    ae = np.abs(e)
    de = np.array([s["de_mps"] for s in samples])
    fase = np.array([s["fase"] for s in samples])
    tun = ae[fase == "tunak"]
    rec = [l["pulih_s"] for l in legs if l["pulih_s"] != ""]
    gs = np.array([s["gs_mps"] for s in samples])
    l1 = np.array([s["l1_dist_m"] for s in samples])
    return dict(
        period_s=period, n_sampel=len(e), n_leg=len(legs),
        e_abs_mean_m=round(ae.mean(), 2), e_rms_m=round(float(np.sqrt(np.mean(e ** 2))), 2),
        e_bias_m=round(e.mean(), 2), e_abs_p50_m=round(pct(ae, 50), 2), e_abs_p90_m=round(pct(ae, 90), 2),
        e_abs_p95_m=round(pct(ae, 95), 2), e_abs_max_m=round(ae.max(), 2),
        e_tunak_mean_m=round(tun.mean(), 2) if len(tun) else "",
        de_abs_p50_mps=round(pct(np.abs(de), 50), 2), de_abs_p90_mps=round(pct(np.abs(de), 90), 2),
        de_maks_menjauh_mps=round(de.max(), 2), de_maks_mendekat_mps=round(de.min(), 2),
        pulih_mean_s=round(float(np.mean(rec)), 1) if rec else "", pulih_maks_s=round(max(rec), 1) if rec else "",
        e_awal_leg_mean_m=round(float(np.mean([abs(l["e_awal_m"]) for l in legs])), 1),
        overshoot_mean_m=round(float(np.mean([l["overshoot_seberang_m"] for l in legs])), 1),
        overshoot_maks_m=round(max(l["overshoot_seberang_m"] for l in legs), 1),
        zero_cross_total=int(sum(l["zero_cross"] for l in legs)),
        pct_e_le5=round(100 * np.mean(ae <= E_BANDS[0]), 1),
        pct_e_5_15=round(100 * np.mean((ae > E_BANDS[0]) & (ae <= E_BANDS[1])), 1),
        pct_e_15_45=round(100 * np.mean((ae > E_BANDS[1]) & (ae <= E_BANDS[2])), 1),
        pct_e_gt45=round(100 * np.mean(ae > E_BANDS[2]), 1),
        pct_waktu_transisi=round(100 * np.mean(fase == "transisi"), 1),
        gs_mean_mps=round(gs.mean(), 1), l1_dist_mean_m=round(l1.mean(), 1),
        angin_mps=round(wind[1], 1), angin_dari_deg=round(wind[2]), va_est_mps=round(wind[0], 1),
    )


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def plots(out, periods, data, summ):
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(periods)))

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.boxplot([np.abs([s["e_m"] for s in data[p][0]]) for p in periods], showfliers=True, flierprops=dict(ms=2))
    ax.set_xticks(range(1, len(periods) + 1))
    ax.set_xticklabels(["%g" % p for p in periods])
    for y, lab in zip(E_BANDS, ("5 m (pulih)", "15 m", "45 m (toleransi)")):
        ax.axhline(y, ls=":", color="r" if y == CTE_LIMIT_M else "0.5", lw=1)
        ax.text(len(periods) + 0.45, y, lab, fontsize=7, va="center")
    ax.set_xlabel("periode L1 (s)")
    ax.set_ylabel("|e| cross-track error (m)")
    ax.set_title("Sebaran |e| per periode L1 (leg WP1 dst., 30/10/2026)")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "01_boxplot_e.png"), dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(1, len(periods), figsize=(3.4 * len(periods), 3.8), sharex=True, sharey=True)
    for ax, p, c in zip(axes, periods, colors):
        s = data[p][0]
        ae = np.array([x["abs_e_m"] for x in s])
        de = np.array([x["de_mps"] for x in s])
        tr = np.array([x["fase"] == "transisi" for x in s])
        ax.scatter(ae[~tr], de[~tr], s=4, color="0.6", label="tunak")
        ax.scatter(ae[tr], de[tr], s=5, color=c, label="transisi")
        ax.axhline(0, color="k", lw=0.6)
        for v in E_BANDS:
            ax.axvline(v, ls=":", color="r" if v == CTE_LIMIT_M else "0.6", lw=0.8)
        ax.set_title("L1 period %g s" % p, fontsize=10)
        ax.set_xlabel("|e| (m)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("d|e|/dt (m/s)  + menjauhi garis")
    axes[0].legend(fontsize=7, markerscale=3)
    fig.suptitle("Bidang fase |e| vs d|e|/dt: bahan himpunan fuzzy", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "02_bidang_fase_e_de.png"), dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(2, 2, figsize=(10, 7))
    P = np.array(periods, dtype=float)
    def series(key):
        return np.array([float(summ[p][key]) if summ[p][key] != "" else np.nan for p in periods])
    ax[0, 0].plot(P, series("e_abs_mean_m"), "o-", label="|e| rata-rata")
    ax[0, 0].plot(P, series("e_rms_m"), "s-", label="RMS")
    ax[0, 0].plot(P, series("e_tunak_mean_m"), "^-", label="|e| tunak")
    ax[0, 0].set_ylabel("m")
    ax[0, 1].plot(P, series("e_abs_p95_m"), "o-", label="|e| p95")
    ax[0, 1].plot(P, series("e_awal_leg_mean_m"), "d-", label="|e| awal leg (geometri antisipasi)")
    ax[0, 1].plot(P, series("overshoot_mean_m"), "s-", label="overshoot seberang rata-rata")
    ax[0, 1].plot(P, series("e_abs_max_m"), "^-", label="|e| maks")
    ax[0, 1].axhline(CTE_LIMIT_M, color="r", ls=":", lw=1, label="toleransi 45 m")
    ax[0, 1].set_ylabel("m")
    ax[1, 0].plot(P, series("pulih_mean_s"), "o-", label="waktu pulih rata-rata")
    ax[1, 0].plot(P, series("pulih_maks_s"), "s-", label="waktu pulih maks")
    ax[1, 0].set_ylabel("s")
    ax[1, 1].plot(P, series("de_abs_p90_mps"), "o-", label="|d|e|/dt| p90")
    ax[1, 1].plot(P, series("de_maks_menjauh_mps"), "s-", label="d|e|/dt maks menjauh")
    ax[1, 1].plot(P, -series("de_maks_mendekat_mps"), "^-", label="|d|e|/dt| maks mendekat")
    ax[1, 1].set_ylabel("m/s")
    for a in ax.flat:
        a.set_xlabel("periode L1 (s)")
        a.set_xticks(P)
        a.grid(alpha=0.3)
        a.legend(fontsize=7)
    fig.suptitle("Tren metrik terhadap periode L1 (satu terbang per periode)", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "03_tren_metrik.png"), dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for p, c in zip(periods, colors):
        s = data[p][0]
        ae = np.abs([x["e_m"] for x in s])
        de = np.array([x["de_mps"] for x in s])
        ax[0].hist(ae, bins=np.arange(0, 46, 2.5), histtype="step", lw=1.5, color=c, density=True, label="%g s" % p)
        ax[1].hist(de, bins=np.arange(-15, 15.5, 1), histtype="step", lw=1.5, color=c, density=True, label="%g s" % p)
    ax[0].set_xlabel("|e| (m)")
    ax[1].set_xlabel("d|e|/dt (m/s)")
    for a in ax:
        a.set_ylabel("kerapatan")
        a.grid(alpha=0.3)
        a.legend(fontsize=7, title="periode L1")
    fig.suptitle("Distribusi |e| dan d|e|/dt", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "04_histogram_e_de.png"), dpi=140)
    plt.close(fig)

    leg_names = sorted({s["leg"] for p in periods for s in data[p][0]}, key=lambda x: int(x.split("WP")[-1]))
    fig, axes = plt.subplots(len(leg_names), 1, figsize=(9, 2.2 * len(leg_names)), sharex=True)
    for ax, leg in zip(np.atleast_1d(axes), leg_names):
        for p, c in zip(periods, colors):
            s = [x for x in data[p][0] if x["leg"] == leg]
            ax.plot([x["t_leg_s"] for x in s], [x["e_m"] for x in s], color=c, lw=1.3, label="%g s" % p)
        ax.axhspan(-RECOVERY_BAND_M, RECOVERY_BAND_M, color="g", alpha=0.08)
        ax.set_ylabel("e (m)")
        ax.set_title(leg, fontsize=9, loc="left")
        ax.grid(alpha=0.3)
    np.atleast_1d(axes)[0].legend(fontsize=7, ncol=len(periods), title="periode L1")
    np.atleast_1d(axes)[-1].set_xlabel("waktu sejak awal leg (s)")
    fig.suptitle("Respons e per leg untuk tiap periode L1 (pita hijau = +-5 m)", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "05_cte_vs_waktu_leg.png"), dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", help='"periode|folder_hasil_analyze|path.tlog"')
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    data, summ = {}, {}
    for spec in args.runs:
        p, folder, tlog = spec.split("|")
        p = float(p)
        samples, legs, wind = process(p, folder, tlog)
        data[p] = (samples, legs, wind)
        summ[p] = summarize(p, samples, legs, wind)
        print("periode %g: %d sampel, %d leg" % (p, len(samples), len(legs)))
    periods = sorted(data)
    write_csv(os.path.join(args.out, "database_sampel.csv"), [s for p in periods for s in data[p][0]])
    write_csv(os.path.join(args.out, "metrik_transisi.csv"), [l for p in periods for l in data[p][1]])
    write_csv(os.path.join(args.out, "ringkasan_per_period.csv"), [summ[p] for p in periods])
    plots(args.out, periods, data, summ)
    for p in periods:
        print(summ[p])
    print("selesai ->", args.out)


if __name__ == "__main__":
    main()
