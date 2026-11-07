#!/usr/bin/env python3
"""Respons terhadap gangguan roll (CH7) dan analisis kecepatan GS/ASP dari log SD (fc::FlightLog v2).

GANGGUAN (folder <out>/gangguan)
  Tiap event DIST_ON dinilai terhadap galat lintasan:
    AUTO (leg ke WP)  e = |cte_m|                       toleransi 45 m
    loiter            e = |wp_dist_m - radius| (pusat = WP loiter)   toleransi --tol (10 m)
  e_sebelum  median e 2 s sebelum gangguan
  e_puncak   e maksimum dalam 15 s setelah gangguan (dipotong di gangguan berikutnya / ganti WP)
  pulih_s    detik sejak gangguan sampai, sesudah puncak, e <= max(e_sebelum, 5) + 2 m dan bertahan 2 s
  Penilaian:
    tak terukur  leg AUTO tanpa garis (posisi -> WP1: cte_m selalu 0)
    terpotong  ganti WP / gangguan berikutnya datang sebelum pulih, tanpa keluar toleransi
    tercampur  e_sebelum > 15 m (galat masuk loiter / belokan belum selesai), atau nav_roll jenuh di batas
               bank > 40% waktu setelah gangguan (galat dari batas bank, mis. angin belakang), sehingga
               respons tidak bisa dipisahkan dari penyebab lain
    baik       kenaikan <= 5 m, pulih <= 10 s, tidak keluar toleransi
    cukup      kenaikan <= 10 m, pulih <= 20 s, tidak keluar toleransi
    buruk      selain itu
  01_lokasi_gangguan.png       lintasan per fase + titik gangguan bernomor, warna = penilaian
  02_respons_per_gangguan.png  galat dan roll - nav_roll sekitar tiap gangguan
  03_cte_vs_waktu.png          cross-track error (AUTO) dan galat radius (loiter) vs waktu: setpoint 0, toleransi, gangguan
  respons_gangguan.csv

KECEPATAN (folder <out>/kecepatan)
  Groundspeed (GPS) dan airspeed (sensor) per fase: FBWA (di udara), AUTO, entry, steady.
  Angin diperkirakan dari loiter steady: vektor kecepatan tanah = vektor udara + angin,
  |v_tanah - angin| ~ konstan -> fit lingkaran. TAS perkiraan dibandingkan airspeed sensor.
  01_kecepatan_vs_waktu.png    GS, ASP, ASP-GS, throttle, berlatar fase, garis = gangguan
  02_angin_dari_loiter.png     vektor kecepatan tanah di loiter + lingkaran fit
  ringkasan_kecepatan.csv

Pemakaian:
    python analyze_sdlog_gangguan_kecepatan.py "2026-10-29_11-31-11.CSV" -o flight_2026-10-29_c --radius 110.6
"""
import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_fuzzy_sdlog import PHASE_COLOR, loiter_turn_index
from sdlog_io import read_sdlog

AUTO_TOL_M = 45.0
PRE_S, POST_S, HOLD_S = 2.0, 15.0, 2.0
VERDICT_COLOR = {"baik": "tab:green", "cukup": "tab:orange", "buruk": "tab:red", "tercampur": "0.55",
                 "terpotong": "lightskyblue", "tak terukur": "white"}
PHASE_COLOR = dict(PHASE_COLOR, FBWA="0.6")


def local_xy(lat_e7, lon_e7, lat0_e7, lon0_e7):
    """Timur, utara [m] relatif titik acuan."""
    y = (np.asarray(lat_e7, float) - lat0_e7) * 1e-7 * 111320.0
    x = (np.asarray(lon_e7, float) - lon0_e7) * 1e-7 * 111320.0 * math.cos(math.radians(lat0_e7 * 1e-7))
    return x, y


def recovery(tt, e, t0, thr):
    """Detik sejak t0 sampai e <= thr dan bertahan HOLD_S; nan bila tidak pulih."""
    m = tt >= t0
    ts, es = tt[m], e[m]
    ok = es <= thr
    start = None
    for k in range(len(ts)):
        if ok[k]:
            if start is None:
                start = ts[k]
            if ts[k] - start >= HOLD_S:
                return start - t0
        else:
            start = None
    return np.nan


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if isinstance(v, float) and math.isnan(v) else v) for k, v in r.items()})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="file log SD (CSV)")
    ap.add_argument("-o", "--out", required=True, help="folder terbang; hasil di <out>/gangguan dan <out>/kecepatan")
    ap.add_argument("--radius", type=float, default=None,
                    help="radius loiter efektif [m] (LOIT_RAD dari tlog); default median wp_dist_m di loiter steady")
    ap.add_argument("--tol", type=float, default=10.0, help="toleransi galat radius loiter [m]")
    ap.add_argument("--bank", type=float, default=35.0, help="LOIT_BANK_DEG [deg] untuk deteksi nav_roll jenuh")
    ap.add_argument("--loiter-wp", type=int, default=None, help="nomor WP loiter (default: WP terakhir di misi)")
    args = ap.parse_args()

    log = read_sdlog(args.log)
    c = log.col
    t = c["time_boot_ms"] / 1000.0
    eff = log.effective_at(float(c["time_boot_ms"][-1]))
    snaps = [s for s in log.snapshots if s.waypoints]
    wps = snaps[-1].waypoints
    loiter_wp = args.loiter_wp if args.loiter_wp is not None else max(w["seq"] for w in wps)
    home = next((s.home for s in reversed(log.snapshots) if s.home), (wps[0]["lat_e7"], wps[0]["lon_e7"], 0))
    X, Y = local_xy(c["lat_e7"], c["lon_e7"], home[0], home[1])
    wpx, wpy = local_xy([w["lat_e7"] for w in wps], [w["lon_e7"] for w in wps], home[0], home[1])
    cwp = next(w for w in wps if w["seq"] == loiter_wp)
    cx, cy = local_xy(cwp["lat_e7"], cwp["lon_e7"], home[0], home[1])

    # ---- fase (sama dengan analyze_fuzzy_sdlog) + FBWA di udara
    armed = c["armed"] > 0
    is_auto = np.array([m == "AUTO" for m in c["mode"]])
    auto = is_auto & (c["wp_next"] > 0) & armed
    in_loiter = auto & (c["wp_next"] == loiter_wp)
    phase = np.full(len(t), "", dtype=object)
    phase[armed & ~is_auto & (c["gs_mps"] > 8.0)] = "FBWA"
    phase[auto & ~in_loiter] = "AUTO"
    if in_loiter.any():
        idx = np.flatnonzero(in_loiter)
        turn = loiter_turn_index(c["lat_e7"][idx], c["lon_e7"][idx])
        phase[idx[turn == 0]] = "entry"
        phase[idx[turn >= 1]] = "steady"
    steady = phase == "steady"
    R = args.radius if args.radius else (float(np.median(c["wp_dist_m"][steady])) if steady.any() else np.nan)
    e_loit = np.abs(c["wp_dist_m"] - R)
    e_auto = np.abs(c["cte_m"])
    in_loit_ph = np.isin(phase, ("entry", "steady"))
    e = np.where(in_loit_ph, e_loit, e_auto)
    roll_err = c["roll_deg"] - c["nav_roll_deg"]
    events = [ev for ev in log.events if ev.kind == "DIST_ON"]
    t_on = [ev.t_s for ev in events]

    # ================================================================ gangguan
    rows = []
    if events:
        out_g = os.path.join(args.out, "gangguan")
        os.makedirs(out_g, exist_ok=True)
        for k, ev in enumerate(events):
            t0 = ev.t_s
            t_end = min(t0 + POST_S, t_on[k + 1] if k + 1 < len(t_on) else np.inf)
            i0 = min(int(np.searchsorted(t, t0)), len(t) - 1)
            # ganti WP = garis baru, lompatan CTE bukan respons gangguan
            wp_chg = np.flatnonzero((t > t0) & (c["wp_next"] != c["wp_next"][i0]))
            if len(wp_chg):
                t_end = min(t_end, float(t[wp_chg[0]]))
            ph = phase[min(i0, len(t) - 1)] or str(c["mode"][min(i0, len(t) - 1)])
            pre = (t >= t0 - PRE_S) & (t < t0)
            post = (t >= t0) & (t <= t_end)
            loit = ph in ("entry", "steady")
            tol = args.tol if loit else AUTO_TOL_M
            e_pre = float(np.median(e[pre])) if pre.any() else np.nan
            ip = np.flatnonzero(post)
            k_pk = ip[int(np.argmax(e[ip]))] if len(ip) else i0
            e_pk = float(e[k_pk])
            t_pk = float(t[k_pk])
            pulih = recovery(t, e, t_pk, max(e_pre, 5.0) + 2.0)
            pulih = pulih + (t_pk - t0) if np.isfinite(pulih) else np.nan
            if np.isfinite(pulih) and t0 + pulih > t_end:
                pulih = np.nan  # gangguan berikutnya / ganti WP datang sebelum pulih
            r3 = (t >= t0) & (t <= t0 + 3.0)
            keluar = bool(np.any(e[post] > tol)) and e_pk > e_pre  # sudah di luar sebelum gangguan: bukan akibat gangguan
            naik = e_pk - e_pre
            lim = args.bank if loit else eff.get("roll_limit_deg", 40.0)
            sat = float(np.mean(np.abs(c["nav_roll_deg"][post]) >= lim - 0.5)) if post.any() else 0.0
            if not loit and c["leg_from_wp"][i0] < 0.5:
                verdict = "tak terukur"
            elif e_pre > 15.0 or sat > 0.4:
                verdict = "tercampur"
            elif not np.isfinite(pulih) and t_end < t0 + POST_S and not keluar:
                verdict = "terpotong"
            elif naik <= 5.0 and np.isfinite(pulih) and pulih <= 10.0 and not keluar:
                verdict = "baik"
            elif naik <= 10.0 and np.isfinite(pulih) and pulih <= 20.0 and not keluar:
                verdict = "cukup"
            else:
                verdict = "buruk"
            rows.append(dict(
                no=k + 1, t_s=round(t0, 1), durasi_s=round(next((x.t_s for x in log.events
                                                             if x.kind == "DIST_OFF" and x.t_s > t0), t0) - t0, 2),
                dist_us=ev.args[0] if ev.args else "",
                fase=ph, wp_next=int(c["wp_next"][min(i0, len(t) - 1)]),
                x_timur_m=round(float(X[i0]), 1), y_utara_m=round(float(Y[i0]), 1),
                lat=round(float(c["lat_e7"][i0]) * 1e-7, 7), lon=round(float(c["lon_e7"][i0]) * 1e-7, 7),
                alt_m=round(float(c["alt_fused_m"][i0]), 1),
                gs_mps=round(float(c["gs_mps"][i0]), 1), airspeed_mps=round(float(c["airspeed_mps"][i0]), 1),
                galat="radius" if loit else "cte", toleransi_m=tol,
                e_sebelum_m=round(e_pre, 1), e_puncak_m=round(e_pk, 1), kenaikan_m=round(naik, 1),
                t_puncak_s=round(float(t[k_pk] - t0), 1), pulih_s=round(pulih, 1) if np.isfinite(pulih) else np.nan,
                keluar_toleransi="YA" if keluar else "tidak",
                nav_roll_jenuh_pct=round(100 * sat, 0),
                catatan=("nav_roll jenuh %.0f%% (GS %.0f m/s): galat dari batas bank, bukan gangguan"
                         % (100 * sat, float(c["gs_mps"][post].max())) if sat > 0.4 else
                         "galat sudah besar sebelum gangguan" if e_pre > 15.0 else
                         "leg tanpa garis, cte_m = 0" if (not loit and c["leg_from_wp"][i0] < 0.5) else ""),
                roll_dev_maks_deg=round(float(np.max(np.abs(roll_err[r3]))), 1) if r3.any() else np.nan,
                penilaian=verdict,
            ))
        write_csv(os.path.join(out_g, "respons_gangguan.csv"), rows)

        # ---- 01 lokasi
        fig, ax = plt.subplots(figsize=(11, 11))
        live = phase != ""
        ax.plot(X[armed], Y[armed], color="0.85", lw=1, zorder=1)
        for ph in ("FBWA", "AUTO", "entry", "steady"):
            m = phase == ph
            if m.any():
                ax.plot(np.where(m, X, np.nan), np.where(m, Y, np.nan), color=PHASE_COLOR[ph], lw=1.8, label=ph, zorder=2)
        th = np.linspace(0, 2 * np.pi, 240)
        if np.isfinite(R):
            ax.plot(cx + R * np.sin(th), cy + R * np.cos(th), "--", color="red", lw=1.4, label="lingkaran loiter R=%.0f m" % R)
            ax.plot(cx + (R + args.tol) * np.sin(th), cy + (R + args.tol) * np.cos(th), ":", color="navy", lw=1.2,
                    label="toleransi ±%.0f m" % args.tol)
            ax.plot(cx + (R - args.tol) * np.sin(th), cy + (R - args.tol) * np.cos(th), ":", color="navy", lw=1.2)
        ax.plot(wpx, wpy, "ks", ms=6)
        for w, x, y in zip(wps, wpx, wpy):
            ax.annotate("WP%d" % w["seq"], (x, y), textcoords="offset points", xytext=(5, 5), fontsize=9)
        for r in rows:
            ax.plot(r["x_timur_m"], r["y_utara_m"], "o", ms=15, mfc=VERDICT_COLOR[r["penilaian"]], mec="k", zorder=5)
            ax.annotate(str(r["no"]), (r["x_timur_m"], r["y_utara_m"]), ha="center", va="center", fontsize=8,
                        weight="bold", color="white", zorder=6)
        for v, col in VERDICT_COLOR.items():
            ax.plot([], [], "o", ms=10, mfc=col, mec="k", label="gangguan: %s" % v)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_xlabel("timur dari HOME [m]")
        ax.set_ylabel("utara dari HOME [m]")
        ax.legend(loc="upper left", fontsize=8)
        ax.set_title("Lokasi gangguan roll (%d event, %s us) -- nomor = urutan, warna = penilaian respons"
                     % (len(rows), rows[0]["dist_us"]))
        fig.tight_layout()
        fig.savefig(os.path.join(out_g, "01_lokasi_gangguan.png"), dpi=130)
        plt.close(fig)

        # ---- 02 respons per gangguan
        n = len(rows)
        ncol = min(4, n)
        nrow = int(math.ceil(n / ncol))
        fig, axs = plt.subplots(nrow, ncol, figsize=(4.2 * ncol, 3.3 * nrow), squeeze=False)
        for a in axs.flat[n:]:
            a.axis("off")
        for a, r in zip(axs.flat, rows):
            t0 = r["t_s"]
            m = (t >= t0 - 3) & (t <= t0 + POST_S)
            a.axvspan(0, r["durasi_s"], color="gold", alpha=0.35, lw=0)
            a.plot(t[m] - t0, e[m], color=VERDICT_COLOR[r["penilaian"]], lw=1.6)
            a.axhline(r["toleransi_m"], color="r", ls=":", lw=0.8)
            a.axhline(r["e_sebelum_m"], color="k", ls="--", lw=0.6)
            a2 = a.twinx()
            a2.plot(t[m] - t0, roll_err[m], color="tab:purple", lw=0.7, alpha=0.7)
            a2.set_ylim(-60, 60)
            a2.tick_params(labelsize=7, colors="tab:purple")
            a.set_ylim(0, max(r["toleransi_m"] * 1.2, float(np.max(e[m])) * 1.1 if m.any() else 1))
            a.set_title("#%d %s t=%.0f s: %s\nnaik %.1f m, pulih %s" % (
                r["no"], r["fase"], t0, r["penilaian"], r["kenaikan_m"],
                "%.1f s" % r["pulih_s"] if isinstance(r["pulih_s"], float) and np.isfinite(r["pulih_s"]) else "-"),
                fontsize=8)
            a.tick_params(labelsize=7)
            a.grid(alpha=0.3)
        fig.supxlabel("detik sejak gangguan (kuning = gangguan aktif)")
        fig.supylabel("galat [m] (radius di loiter, |CTE| di AUTO); ungu = roll - nav_roll [deg]")
        fig.tight_layout()
        fig.savefig(os.path.join(out_g, "02_respons_per_gangguan.png"), dpi=120)
        plt.close(fig)

        # ---- 03 cross-track error vs waktu: AUTO (CTE ke garis antar-WP) dan loiter (galat radius)
        t_auto0 = float(t[np.flatnonzero(auto)[0]])
        panels = [("AUTO", auto & ~in_loiter, c["cte_m"], AUTO_TOL_M,
                   "cross-track error [m]\n(+ = kiri garis)"),
                  ("LOITER", in_loiter, c["wp_dist_m"] - R, args.tol,
                   "galat radius [m]\n(+ = di luar lingkaran)")]
        panels = [p for p in panels if p[1].any()]
        fig, axs = plt.subplots(len(panels), 1, figsize=(15, 4.2 * len(panels)), squeeze=False)
        for a, (name, m, y, tol, ylab) in zip(axs[:, 0], panels):
            tt = t[m] - t_auto0
            yy = np.where(np.diff(np.r_[t[m], np.inf]) > 0.5, np.nan, y[m])  # putus di celah data
            a.plot(tt, yy, color="tab:blue", lw=1.4, label="aktual")
            a.axhline(0, color="red", lw=1.6, label="setpoint (0 m)")
            a.axhline(tol, color="k", ls=":", lw=1.6, label="batas toleransi ±%.0f m" % tol)
            a.axhline(-tol, color="k", ls=":", lw=1.6)
            if name == "AUTO":
                nl = m & (c["leg_from_wp"] < 0.5)
                if nl.any():
                    a.axvspan(t[nl][0] - t_auto0, t[nl][-1] - t_auto0, color="0.85", lw=0,
                              label="leg posisi -> WP1 (tanpa garis, CTE = 0)")
                for ev in log.events:
                    if ev.kind == "WP_REACHED" and t[m][0] <= ev.t_s <= t[m][-1] + 0.5:
                        a.axvline(ev.t_s - t_auto0, color="0.4", ls="--", lw=0.8)
                        a.annotate("WP%s" % ev.args[0], (ev.t_s - t_auto0, 0.02), xycoords=("data", "axes fraction"),
                                   xytext=(3, 0), textcoords="offset points", fontsize=8, color="0.3")
                a.plot([], [], "--", color="0.4", lw=0.8, label="WP tercapai (ganti garis)")
            else:
                st = m & steady
                if st.any():
                    a.axvline(t[st][0] - t_auto0, color="tab:green", ls="--", lw=1.0,
                              label="mulai loiter steady (putaran ke-2)")
            for r in rows:
                if tt[0] - 1 <= r["t_s"] - t_auto0 <= tt[-1] + 1:
                    x0 = r["t_s"] - t_auto0
                    a.axvspan(x0, x0 + r["durasi_s"], color="gold", alpha=0.6, lw=0)
                    a.annotate("#%d" % r["no"], (x0, 1.0), xycoords=("data", "axes fraction"),
                               xytext=(0, 2), textcoords="offset points", ha="left", va="bottom", fontsize=8)
            a.axvspan(np.nan, np.nan, color="gold", alpha=0.6, label="gangguan roll %s us" % rows[0]["dist_us"])
            lim = max(tol * 1.3, float(np.nanmax(np.abs(yy))) * 1.1)
            a.set_ylim(-lim, lim)
            a.set_xlim(tt[0], tt[-1])
            a.set_ylabel(ylab)
            a.set_xlabel("waktu sejak masuk AUTO [s]")
            a.set_title(name, loc="left", fontsize=10, weight="bold")
            a.grid(alpha=0.3)
            a.legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.01, 1.0), borderaxespad=0)
        fig.tight_layout()
        fig.savefig(os.path.join(out_g, "03_cte_vs_waktu.png"), dpi=130)
        plt.close(fig)

    # ================================================================ kecepatan
    out_k = os.path.join(args.out, "kecepatan")
    os.makedirs(out_k, exist_ok=True)
    gs, asp = c["gs_mps"], c["airspeed_mps"]
    vn = gs * np.cos(np.radians(c["course_deg"]))
    ve = gs * np.sin(np.radians(c["course_deg"]))
    wind = (np.nan, np.nan, np.nan)
    if steady.sum() > 50:
        A = np.c_[2 * vn[steady], 2 * ve[steady], np.ones(steady.sum())]
        sol = np.linalg.lstsq(A, vn[steady] ** 2 + ve[steady] ** 2, rcond=None)[0]
        wn, we = sol[0], sol[1]
        wind = (wn, we, math.sqrt(max(sol[2] + wn ** 2 + we ** 2, 0.0)))
    wspd = math.hypot(wind[0], wind[1]) if np.isfinite(wind[0]) else np.nan
    wfrom = (math.degrees(math.atan2(-wind[1], -wind[0])) % 360) if np.isfinite(wind[0]) else np.nan
    krows = []
    for ph in ("FBWA", "AUTO", "entry", "steady"):
        m = phase == ph
        if not m.any():
            continue
        a_ok = asp[m] > 1.0
        krows.append(dict(
            fase=ph, durasi_s=round(m.sum() * float(np.median(np.diff(t))), 1),
            gs_mean=round(float(gs[m].mean()), 1), gs_min=round(float(gs[m].min()), 1), gs_max=round(float(gs[m].max()), 1),
            asp_mean=round(float(asp[m][a_ok].mean()), 1) if a_ok.any() else np.nan,
            asp_min=round(float(asp[m][a_ok].min()), 1) if a_ok.any() else np.nan,
            asp_max=round(float(asp[m][a_ok].max()), 1) if a_ok.any() else np.nan,
            asp_minus_gs_mean=round(float((asp[m] - gs[m])[a_ok].mean()), 1) if a_ok.any() else np.nan,
            persen_asp_nol=round(100.0 * float(np.mean(~a_ok)), 1),
            throttle_us_mean=round(float(c["thr_out_us"][m].mean()), 0),
            batt_v_min=round(float(c["batt_v"][m].min()), 2),
            angin_mps=round(wspd, 1) if ph == "steady" else np.nan,
            angin_dari_deg=round(wfrom, 0) if ph == "steady" else np.nan,
            tas_perkiraan_mps=round(wind[2], 1) if ph == "steady" else np.nan,
            rasio_asp_tas=round(float(asp[m][a_ok].mean()) / wind[2], 3)
            if ph == "steady" and a_ok.any() and np.isfinite(wind[2]) and wind[2] > 0 else np.nan,
        ))
    if krows:
        write_csv(os.path.join(out_k, "ringkasan_kecepatan.csv"), krows)

    live = phase != ""
    if live.any():
        i0, i1 = np.flatnonzero(live)[[0, -1]]
        sl = slice(max(i0 - 100, 0), i1 + 100)
        fig, axs = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        for a in axs:
            for ph in ("FBWA", "AUTO", "entry", "steady"):
                m = phase[sl] == ph
                if m.any():
                    tt = t[sl][m]
                    for s0, s1 in zip(np.r_[tt[0], tt[1:][np.diff(tt) > 0.5]], np.r_[tt[:-1][np.diff(tt) > 0.5], tt[-1]]):
                        a.axvspan(s0, s1, color=PHASE_COLOR[ph], alpha=0.12, lw=0)
            for t0 in t_on:
                a.axvline(t0, color="gold", lw=1.5)
            a.grid(alpha=0.3)
        axs[0].plot(t[sl], gs[sl], label="groundspeed (GPS)")
        axs[0].plot(t[sl], asp[sl], label="airspeed (sensor)")
        if np.isfinite(wind[2]):
            axs[0].axhline(wind[2], color="k", ls="--", lw=0.8, label="TAS perkiraan dari loiter %.1f m/s" % wind[2])
        axs[0].set_ylabel("m/s")
        axs[0].legend(fontsize=8, loc="upper left")
        axs[0].set_title("Kecepatan -- latar: abu FBWA, biru AUTO, oranye entry, hijau steady; garis kuning = gangguan")
        axs[1].plot(t[sl], asp[sl] - gs[sl], color="tab:brown", lw=0.8)
        axs[1].axhline(0, color="k", lw=0.6)
        axs[1].set_ylabel("ASP - GS [m/s]\n(+ = angin dari depan)")
        axs[2].plot(t[sl], c["thr_out_us"][sl], color="0.3", lw=0.8, label="throttle PWM")
        axs[2].set_ylabel("throttle [us]")
        a2 = axs[2].twinx()
        a2.plot(t[sl], c["batt_v"][sl], color="tab:red", lw=0.8)
        a2.set_ylabel("baterai [V]", color="tab:red")
        axs[2].set_xlabel("time_boot [s]")
        fig.tight_layout()
        fig.savefig(os.path.join(out_k, "01_kecepatan_vs_waktu.png"), dpi=120)
        plt.close(fig)

    if steady.sum() > 50:
        fig, ax = plt.subplots(figsize=(7, 7))
        sc = ax.scatter(ve[steady], vn[steady], c=gs[steady], s=6, cmap="viridis")
        th = np.linspace(0, 2 * np.pi, 240)
        ax.plot(wind[1] + wind[2] * np.sin(th), wind[0] + wind[2] * np.cos(th), "r--", lw=1.2,
                label="fit: TAS %.1f m/s" % wind[2])
        ax.annotate("", (wind[1], wind[0]), (0, 0), arrowprops=dict(arrowstyle="->", color="r", lw=2))
        ax.plot(0, 0, "k+", ms=12)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_xlabel("v timur [m/s]")
        ax.set_ylabel("v utara [m/s]")
        ax.set_title("Vektor kecepatan tanah di loiter steady\nangin %.1f m/s dari %.0f deg (panah = arah tiupan)"
                     % (wspd, wfrom))
        fig.colorbar(sc, label="groundspeed [m/s]")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out_k, "02_angin_dari_loiter.png"), dpi=120)
        plt.close(fig)

    for r in rows:
        print("gangguan:", r)
    for r in krows:
        print("kecepatan:", r)
    print("hasil ->", args.out)


if __name__ == "__main__":
    main()
