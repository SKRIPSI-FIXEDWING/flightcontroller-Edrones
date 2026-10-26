"""Analisis LOITER + ringkasan seluruh mode terbang dari satu tlog Mission Planner.

Padanan analyze_tlog_l1_auto.py untuk loiter. Keluaran bernomor sama dengan
analisis AUTO supaya mudah dibandingkan:

  LOITER (setpoint = lingkaran pusat C, radius R; galat = jarak ke C - R)
    01_lingkaran_rencana_vs_aktual.png   lintasan semua terbang + lingkaran perintah
    02_lintasan_loiter_dan_L1.png        per segmen: lintasan relatif pusat, warna waktu, lingkaran L1
    03_galat_radius_vs_waktu.png         galat radius, |d galat/dt|, jarak ke pusat vs radius
    04_attitude_loiter_roll_pitch.png    roll/nav_roll, aileron, pitch/nav_pitch, elevator
    05_altitude_kecepatan_throttle.png   altitude vs target, groundspeed/airspeed, throttle
    06_kualitas_gps.png                  satelit, HDOP, usia posisi (seluruh log)
    07_metrik_per_putaran.png            metrik tiap putaran 360 deg (padanan metrik per leg)
    08_timeline_seluruh_log.png          roll, altitude, groundspeed, berlatar mode efektif
    11_lingkaran_rencana_vs_aktual_terbang<N>.png  gaya jurnal per segmen
    galat_per_sampel.csv, metrik_per_putaran.csv, ringkasan_loiter.csv, deviasi_maksimum.csv

  SELURUH MODE (MANUAL / FBWA / AUTO / LOITER / ...)
    09_lintasan_seluruh_mode.png         lintasan berwarna mode
    10_attitude_seluruh_mode.png         roll/pitch vs perintah + PWM sepanjang terbang
    12_statistik_per_mode.png            durasi, roll, altitude, kecepatan per mode
    ringkasan_per_mode.csv

Segmen loiter dideteksi dari:
  1. NAMED_VALUE_FLOAT LOIT_* (firmware >= 2026-10-19; dikirim hanya saat
     wahana benar-benar berputar). Termasuk item LOITER_* di misi AUTO --
     firmware lama melaporkan mode AUTO selama itu, firmware >= 2026-10-20
     melaporkan LOITER. Pusat direkonstruksi dari posisi - (LOIT_N, LOIT_E).
  2. Mode LOITER (custom_mode 12) tanpa LOIT_* (firmware lama): pusat = posisi
     saat masuk mode, radius = |WP_LOITER_RAD| (min 20 m).
"Mode efektif" = mode HEARTBEAT, tetapi AUTO yang sedang berputar dihitung LOITER.

Pemakaian:
    python analyze_tlog_loiter.py "path/ke/2026-10-19 16-12-37.tlog" [-o out_dir] [--tol 10] [--bagian semua|loiter|full]
"""
import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_tlog_l1_auto import MODE_COLOR, fc_time, load, mode_intervals, series

R2D = 180.0 / np.pi
G = 9.80665
GAP_S = 2.0              # jeda antar sampel LOIT_* lebih dari ini = segmen baru
RECOVERY_BAND_M = 5.0    # |galat| dianggap konvergen bila <= nilai ini ...
RECOVERY_HOLD_S = 2.0    # ... selama minimal sekian detik
L1_K = 0.3183099         # L1 = (1/pi) * damping * period * groundspeed
FC = (1, 1)
MODE_ORDER = ["MANUAL", "FBWA", "AUTO", "LOITER", "GUIDED", "RTL"]
MODE_COLOR_LINE = {"MANUAL": "0.45", "FBWA": "tab:blue", "AUTO": "darkorange", "LOITER": "tab:green",
                   "GUIDED": "tab:purple", "RTL": "tab:pink"}


def named_series(D, name, t0):
    rows = [d for d in D["NAMED_VALUE_FLOAT"] if d["name"] == name]
    if not rows:
        return np.array([]), np.array([])
    t = fc_time(rows, t0)
    o = np.argsort(t, kind="stable")
    return t[o], np.array([d["value"] for d in rows])[o]


def last_param(D, names, default=np.nan):
    v = default
    for p in D["PARAM_VALUE"]:
        if p["param_id"] in names:
            v = p["param_value"]
    return v


def shade_iv(ax, iv, labels=False):
    seen = set()
    for a, b, n in iv:
        ax.axvspan(a, b, color=MODE_COLOR.get(n, "#eee"), alpha=0.6, lw=0,
                   label=(n if labels and n not in seen else None))
        seen.add(n)


def subtract(iv, cuts):
    """Interval [a,b] dikurangi daftar interval cuts."""
    out = []
    for a, b in iv:
        parts = [(a, b)]
        for c, d in cuts:
            nxt = []
            for x, y in parts:
                if d <= x or c >= y:
                    nxt.append((x, y))
                    continue
                if c > x:
                    nxt.append((x, c))
                if d < y:
                    nxt.append((d, y))
            parts = nxt
        out += [(x, y) for x, y in parts if y - x > 0.05]
    return out


def ring(cx, cy, r, n=240):
    th = np.linspace(0, 2 * np.pi, n)
    return cx + r * np.sin(th), cy + r * np.cos(th)


def recovery_time(tt, err):
    inside = np.abs(err) <= RECOVERY_BAND_M
    for j in range(len(tt)):
        if inside[j]:
            jj = np.where(tt >= tt[j] + RECOVERY_HOLD_S)[0]
            if len(jj) and inside[j:jj[0] + 1].all():
                return tt[j] - tt[0]
    return np.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tlog")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--tol", type=float, default=10.0, help="toleransi galat radius [m] (default 10; AUTO tetap 45)")
    ap.add_argument("--bagian", choices=("semua", "loiter", "full"), default="semua",
                    help="loiter = hanya analisis LOITER (01-08, 11 + CSV loiter); "
                         "full = hanya seluruh penerbangan (06, 08, 09, 10, 12 + ringkasan_per_mode); semua = keduanya")
    args = ap.parse_args()
    do_loiter = args.bagian in ("semua", "loiter")
    do_full = args.bagian in ("semua", "full")
    TOL = args.tol
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "loiter")
    os.makedirs(out, exist_ok=True)

    D = load(args.tlog)
    t0 = D["ATTITUDE"][0]["_t"]
    ta, roll = series(D, "ATTITUDE", "roll", t0, R2D)
    _, pitch = series(D, "ATTITUDE", "pitch", t0, R2D)
    T_END = ta[-1]
    iv_raw = mode_intervals(D, t0, T_END)

    tp, lat = series(D, "GLOBAL_POSITION_INT", "lat", t0, 1e-7)
    _, lon = series(D, "GLOBAL_POSITION_INT", "lon", t0, 1e-7)
    tv, gs = series(D, "VFR_HUD", "groundspeed", t0)
    _, aspd = series(D, "VFR_HUD", "airspeed", t0)
    _, thr = series(D, "VFR_HUD", "throttle", t0)
    _, alt = series(D, "VFR_HUD", "alt", t0)
    tn, nav_roll = series(D, "NAV_CONTROLLER_OUTPUT", "nav_roll", t0)
    _, nav_pitch = series(D, "NAV_CONTROLLER_OUTPUT", "nav_pitch", t0)
    _, wp_dist = series(D, "NAV_CONTROLLER_OUTPUT", "wp_dist", t0)
    _, xtrack_tlm = series(D, "NAV_CONTROLLER_OUTPUT", "xtrack_error", t0)
    ts, ail = series(D, "SERVO_OUTPUT_RAW", "servo1_raw", t0)
    _, ele = series(D, "SERVO_OUTPUT_RAW", "servo2_raw", t0)
    tr, ch3 = series(D, "RC_CHANNELS", "chan3_raw", t0)
    tg, sats = series(D, "GPS_RAW_INT", "satellites_visible", t0)
    _, eph = series(D, "GPS_RAW_INT", "eph", t0, 0.01)
    _, fix = series(D, "GPS_RAW_INT", "fix_type", t0)
    _, glat = series(D, "GPS_RAW_INT", "lat", t0)
    _, cog = series(D, "GPS_RAW_INT", "cog", t0, 0.01)
    tm, mseq = series(D, "MISSION_CURRENT", "seq", t0) if D.get("MISSION_CURRENT") else (np.array([]), np.array([]))

    tl, ldst = named_series(D, "LOIT_DST", t0)
    # tiap NAMED_VALUE_FLOAT punya waktunya sendiri: paket bisa hilang, jumlah sampel tidak selalu sama
    tR, lrad = named_series(D, "LOIT_RAD", t0)
    tErr, lerr = named_series(D, "LOIT_ERR", t0)
    tdir, ldir = named_series(D, "LOIT_DIR", t0)
    tN, lN = named_series(D, "LOIT_N", t0)
    tE, lE = named_series(D, "LOIT_E", t0)
    if len(tN) and len(tE):
        lE = np.interp(tN, tE, lE)  # pasangkan E ke waktu sampel N (bukan per indeks)
        tE = tN
    tB, lbnk = named_series(D, "LOIT_BNK", t0)
    alt_named = {k: named_series(D, k, t0) for k in ("ALT_FUSED", "ALT_BARO", "ALT_GPS")}

    P = dict(
        roll_lim=last_param(D, ("ROLL_LIMIT_DEG",)), bank=last_param(D, ("LOIT_BANK_DEG",)),
        rad=last_param(D, ("WP_LOITER_RAD",), -50.0), sign=last_param(D, ("LOIT_ROLL_SIGN",)),
        loit_alt=last_param(D, ("LOIT_ALT",)), cruise=last_param(D, ("AIRSPEED_CRUISE",)),
        l1_period=last_param(D, ("NAVL1_PERIOD", "L1_PERIOD"), 22.0),
        l1_damp=last_param(D, ("NAVL1_DAMPING", "L1_DAMPING"), 0.707),
    )
    print("param:", P)

    # ---- koordinat lokal (asal = HOME bila ada, else posisi pertama saat armed)
    home = None
    for typ, sc in (("MISSION_ITEM_INT", 1e-7), ("MISSION_ITEM", 1.0)):
        for d in D.get(typ, []):
            if d["seq"] == 0 and d["x"] != 0:
                home = (d["x"] * sc, d["y"] * sc)
    if home is None:
        home = (float(np.median(lat)), float(np.median(lon)))
    kx = 111320.0 * np.cos(np.radians(home[0]))
    PX = (lon - home[1]) * kx
    PY = (lat - home[0]) * 111320.0
    gs_p = np.interp(tp, tv, gs)
    l1_dist = L1_K * P["l1_damp"] * P["l1_period"] * gs_p

    # ================================================================ segmen loiter
    segs = []  # dict(a, b, cx, cy, R(t-fn), dir, sumber)
    if len(tl):
        cuts = [0] + list(np.where(np.diff(tl) > GAP_S)[0] + 1) + [len(tl)]
        k = min(len(tN), len(tE))
        for a_i, b_i in zip(cuts[:-1], cuts[1:]):
            if b_i - a_i < 5:
                continue
            a, b = float(tl[a_i]), float(tl[b_i - 1])
            mgs = (tv >= a) & (tv <= b)
            if b - a < 5.0 or (mgs.any() and float(np.median(gs[mgs])) < 5.0):
                print("CATATAN: LOIT_* %.1f-%.1f s diabaikan (terlalu singkat / wahana diam di darat)" % (a, b))
                continue
            mne = (tN[:k] >= a) & (tN[:k] <= b)
            if mne.sum() >= 3:
                cx = float(np.median(np.interp(tN[:k][mne], tp, PX) - lE[:k][mne]))
                cy = float(np.median(np.interp(tN[:k][mne], tp, PY) - lN[:k][mne]))
            else:  # tanpa LOIT_N/E: pusat tak diketahui -> perkirakan dari titik terjauh/terdekat
                cx, cy = float(np.mean(PX[(tp >= a) & (tp <= b)])), float(np.mean(PY[(tp >= a) & (tp <= b)]))
            md = (tdir >= a) & (tdir <= b)
            seg_mode = None
            for x, y, n, arm in iv_raw:
                if x <= a < y:
                    seg_mode = n
            segs.append(dict(a=a, b=b, cx=cx, cy=cy, t_rad=tR[(tR >= a) & (tR <= b)] if ((tR >= a) & (tR <= b)).sum() >= 2 else tl[a_i:b_i],
                             v_rad=lrad[(tR >= a) & (tR <= b)] if ((tR >= a) & (tR <= b)).sum() >= 2 else ldst[a_i:b_i] * 0 + max(abs(P["rad"]), 20.0),
                             dir=int(np.sign(np.median(ldir[md]))) if md.any() else (1 if P["rad"] >= 0 else -1),
                             sumber="LOIT_* (%s)" % ("item misi AUTO" if seg_mode == "AUTO" else "mode LOITER")))
    covered = [(s["a"], s["b"]) for s in segs]
    for a, b, n, arm in iv_raw:
        if n != "LOITER" or not arm:
            continue
        for x, y in subtract([(a, b)], covered):
            if y - x < 5:
                continue
            i0 = int(np.searchsorted(tp, x))
            r = max(abs(P["rad"]), 20.0)
            segs.append(dict(a=x, b=y, cx=float(PX[min(i0, len(PX) - 1)]), cy=float(PY[min(i0, len(PY) - 1)]),
                             t_rad=np.array([x, y]), v_rad=np.array([r, r]), dir=1 if P["rad"] >= 0 else -1,
                             sumber="mode LOITER (tanpa LOIT_*, pusat = titik masuk)"))
    segs.sort(key=lambda s: s["a"])

    # ---- mode efektif: AUTO yang sedang berputar = LOITER
    loit_iv = [(s["a"], s["b"]) for s in segs]
    iv = []  # (a, b, nama) hanya saat armed
    for a, b, n, arm in iv_raw:
        if not arm:
            continue
        if n == "AUTO":
            for x, y in subtract([(a, b)], loit_iv):
                iv.append((x, y, "AUTO"))
            for x, y in loit_iv:
                lo, hi = max(a, x), min(b, y)
                if hi - lo > 0.05:
                    iv.append((lo, hi, "LOITER"))
        elif n == "LOITER":
            iv.append((a, b, "LOITER"))
        else:
            iv.append((a, b, n))
    iv.sort()
    merged = []  # gabungkan interval bersebelahan dengan mode sama (celah < 1 s)
    for a, b, n in iv:
        if merged and merged[-1][2] == n and a - merged[-1][1] < 1.0:
            merged[-1] = (merged[-1][0], b, n)
        else:
            merged.append((a, b, n))
    iv = merged
    flights = []
    for a, b, n in iv:
        if not flights or a > flights[-1][1] + 1.0:
            flights.append([a, b])
        else:
            flights[-1][1] = max(flights[-1][1], b)
    print("terbang (armed):", [(round(a, 1), round(b, 1)) for a, b in flights])
    print("mode efektif:", [(round(a, 1), round(b, 1), n) for a, b, n in iv])

    if not segs:
        print("Tidak ada segmen loiter (tidak ada LOIT_* maupun mode LOITER). Hanya analisis seluruh mode.")

    # ================================================================ galat per sampel + per putaran
    rows, laps, summary, deviasi, S_all = [], [], [], [], []
    for si, s in enumerate(segs):
        a, b = s["a"], s["b"]
        m = (tp >= a) & (tp <= b)
        tt, x, y = tp[m], PX[m] - s["cx"], PY[m] - s["cy"]
        R = np.interp(tt, s["t_rad"], s["v_rad"])
        d = np.hypot(x, y)
        err = d - R
        bear = np.degrees(np.arctan2(x, y))  # dari pusat ke wahana, derajat kompas
        dang = np.concatenate([[0.0], (np.diff(bear) + 180.0) % 360.0 - 180.0])
        cum = np.cumsum(dang)  # + = CW
        rate = np.gradient(err, tt) if len(tt) > 2 else np.zeros_like(err)
        S = np.column_stack([tt, x, y, d, R, err, cum, gs_p[m], l1_dist[m]])
        S_all.append(S)
        for j in range(len(tt)):
            rows.append([si + 1] + list(S[j]))

        # putaran: tiap kelipatan 360 deg dari |sudut kumulatif| (arah aktual)
        act_dir = int(np.sign(cum[-1])) if abs(cum[-1]) > 30 else 0
        acum = np.abs(cum)
        edges = [0]
        kk = 1
        for j in range(len(acum)):
            if acum[j] >= 360.0 * kk:
                edges.append(j)
                kk += 1
        if len(acum) - 1 > edges[-1]:
            edges.append(len(acum) - 1)
        for li in range(len(edges) - 1):
            j0, j1 = edges[li], edges[li + 1]
            if j1 - j0 < 3:
                continue
            sl = slice(j0, j1 + 1)
            t_a, t_b = tt[j0], tt[j1]
            mr = (ta >= t_a) & (ta <= t_b)
            mn = (tn >= t_a) & (tn <= t_b)
            mv = (tv >= t_a) & (tv <= t_b)
            bank = float(np.median(lbnk[(tB >= t_a) & (tB <= t_b)])) if ((tB >= t_a) & (tB <= t_b)).any() else P["bank"]
            sweep = float(acum[j1] - acum[j0])
            laps.append(dict(
                segmen=si + 1, putaran=li + 1, t_mulai=round(t_a, 1), durasi_s=round(t_b - t_a, 1),
                sudut_deg=round(sweep, 0), lengkap="YA" if sweep >= 355 else "parsial",
                radius_cmd_m=round(float(np.mean(R[sl])), 1), radius_aktual_mean_m=round(float(np.mean(d[sl])), 1),
                radius_aktual_std_m=round(float(np.std(d[sl])), 1),
                galat_mean_m=round(float(np.mean(err[sl])), 1),
                galat_mean_abs_m=round(float(np.mean(np.abs(err[sl]))), 1),
                galat_rms_m=round(float(np.sqrt(np.mean(err[sl] ** 2))), 1),
                galat_max_abs_m=round(float(np.max(np.abs(err[sl]))), 1),
                dgalat_max_abs_mps=round(float(np.max(np.abs(rate[sl]))), 2),
                lewat_tol="YA" if np.max(np.abs(err[sl])) > TOL else "tidak",
                gs_mean=round(float(np.mean(gs[mv])), 1) if mv.any() else np.nan,
                gs_min=round(float(np.min(gs[mv])), 1) if mv.any() else np.nan,
                gs_max=round(float(np.max(gs[mv])), 1) if mv.any() else np.nan,
                airspeed_mean=round(float(np.mean(aspd[mv])), 1) if mv.any() else np.nan,
                alt_mean=round(float(np.mean(alt[mv])), 1) if mv.any() else np.nan,
                alt_std=round(float(np.std(alt[mv])), 1) if mv.any() else np.nan,
                roll_mean=round(float(np.mean(roll[mr])), 1) if mr.any() else np.nan,
                nav_roll_sat_frac=round(float(np.mean(np.abs(nav_roll[mn]) >= bank - 0.5)), 2)
                if mn.any() and np.isfinite(bank) else np.nan,
            ))

        # ringkasan per segmen
        mt = (ta >= a) & (ta <= b)
        mn = (tn >= a) & (tn <= b)
        mv = (tv >= a) & (tv <= b)
        mb = (tB >= a) & (tB <= b)
        bank = float(np.median(lbnk[mb])) if mb.any() else P["bank"]
        va = float(np.mean(aspd[mv])) if mv.any() else np.nan
        r_min = va ** 2 / (G * math.tan(math.radians(bank))) if np.isfinite(bank) and bank > 0 else np.nan
        nr_at_att = nav_roll[np.clip(np.searchsorted(tn, ta[mt], side="right") - 1, 0, len(tn) - 1)]
        rec = recovery_time(tt, err)
        tgt_alt = P["loit_alt"] if np.isfinite(P["loit_alt"]) and P["loit_alt"] > 0 else float(alt[mv][0]) if mv.any() else np.nan
        wp_now = int(np.interp(a, tm, mseq)) if len(tm) else -1
        # item TERAKHIR yang tercatat untuk seq ini (misi bisa di-upload ulang)
        cands = sorted(((d["_t"], d["command"]) for typ in ("MISSION_ITEM_INT", "MISSION_ITEM")
                        for d in D.get(typ, []) if d["seq"] == wp_now and d["_t"] - t0 <= a), key=lambda x: x[0])
        cmd_now = cands[-1][1] if cands else None
        loiter_cmds = {17: "LOITER_UNLIM", 18: "LOITER_TURNS", 19: "LOITER_TIME"}
        if cmd_now in loiter_cmds and s["sumber"].startswith("LOIT_*"):
            s["sumber"] = "LOIT_* (item misi WP%d %s di AUTO)" % (wp_now, loiter_cmds[cmd_now])
        summary.append(dict(
            segmen=si + 1, sumber=s["sumber"],
            wp_aktif=wp_now,
            loiter_mulai=round(a, 1), loiter_selesai=round(b, 1), durasi=round(b - a, 1),
            radius_cmd=round(float(np.median(s["v_rad"])), 1),
            arah_cmd="CW" if s["dir"] > 0 else "CCW",
            arah_aktual={1: "CW", -1: "CCW", 0: "-"}[act_dir],
            sudut_putar_total=round(float(cum[-1]), 0), jumlah_putaran=round(abs(float(cum[-1])) / 360.0, 2),
            galat_mean=round(float(np.mean(err)), 1),
            galat_mean_abs=round(float(np.mean(np.abs(err))), 1), galat_rms=round(float(np.sqrt(np.mean(err ** 2))), 1),
            galat_max_abs=round(float(np.max(np.abs(err))), 1),
            persen_lewat_tol=round(100 * float(np.mean(np.abs(err) > TOL)), 1),
            dgalat_max=round(float(np.max(np.abs(rate))), 1),
            konvergen_s=("-" if np.isnan(rec) else round(float(rec), 1)),
            radius_aktual_mean=round(float(np.mean(d)), 1),
            airspeed_mean=round(va, 1), gs_mean=round(float(np.mean(gs[mv])), 1) if mv.any() else np.nan,
            angin_perkiraan=round((float(np.max(gs[mv])) - float(np.min(gs[mv]))) / 2, 1) if mv.any() else np.nan,
            l1_mean=round(float(np.mean(l1_dist[m])), 0),
            batas_bank=round(bank, 1) if np.isfinite(bank) else np.nan,
            radius_min_terbang=round(float(r_min), 0) if np.isfinite(r_min) else np.nan,
            alt_target=round(tgt_alt, 1) if np.isfinite(tgt_alt) else np.nan,
            alt_galat_rms=round(float(np.sqrt(np.mean((alt[mv] - tgt_alt) ** 2))), 1) if mv.any() and np.isfinite(tgt_alt) else np.nan,
            roll_std=round(float(np.std(roll[mt])), 1),
            roll_min=round(float(np.min(roll[mt])), 1), roll_max=round(float(np.max(roll[mt])), 1),
            nav_roll_sat=round(float(np.mean(np.abs(nav_roll[mn]) >= bank - 0.5)), 2) if mn.any() and np.isfinite(bank) else np.nan,
            roll_track_rmse=round(float(np.sqrt(np.mean((roll[mt] - nr_at_att) ** 2))), 1) if mt.any() else np.nan,
            corr_roll_navroll=round(float(np.corrcoef(roll[mt], nr_at_att)[0, 1]), 2) if mt.sum() > 2 else np.nan,
            pusat_x=round(s["cx"], 1), pusat_y=round(s["cy"], 1),
        ))
        i_max = int(np.argmax(np.abs(err)))
        deviasi.append(dict(segmen=si + 1, galat_maks=float(abs(err[i_max])), t_galat_maks=float(tt[i_max]),
                            jarak_pusat_saat_itu=float(d[i_max]), radius_cmd=float(R[i_max]),
                            jarak_pusat_mean=float(np.mean(d)),
                            persen_luar_toleransi=100.0 * float(np.mean(np.abs(err) > TOL))))

    def write_csv(name, data, header=None):
        if not data:
            return
        with open(os.path.join(out, name), "w", newline="") as f:
            if header:
                w = csv.writer(f)
                w.writerow(header)
                for r in data:
                    w.writerow([r[0]] + ["%.3f" % v for v in r[1:]])
            else:
                w = csv.DictWriter(f, fieldnames=list(data[0].keys()))
                w.writeheader()
                w.writerows(data)

    if not do_loiter:
        rows, laps, summary, deviasi = [], [], [], []
    write_csv("galat_per_sampel.csv", rows,
              ["segmen", "t_s", "x_dari_pusat_m", "y_dari_pusat_m", "jarak_ke_pusat_m", "radius_cmd_m",
               "galat_m(+luar)", "sudut_kumulatif_deg(+CW)", "groundspeed_mps", "l1_distance_m"])
    write_csv("metrik_per_putaran.csv", laps)
    write_csv("ringkasan_loiter.csv", summary)
    write_csv("deviasi_maksimum.csv", deviasi)

    # ================================================================ PLOT LOITER
    scol = ["tab:green", "tab:red", "tab:purple", "tab:brown"]
    NS = max(len(segs), 1)

    if segs and do_loiter:
        # ---- 01 lingkaran rencana vs aktual (semua)
        fig, ax = plt.subplots(figsize=(11, 11))
        for fa, fb in flights:
            mm = (tp >= fa) & (tp <= fb)
            ax.plot(PX[mm], PY[mm], color="0.75", lw=1, zorder=1)
        ax.plot([], [], color="0.75", label="mode lain (FBWA/AUTO)")
        for si, s in enumerate(segs):
            mm = (tp >= s["a"]) & (tp <= s["b"])
            c = scol[si % len(scol)]
            ax.plot(PX[mm], PY[mm], color=c, lw=2.2, zorder=3,
                    label="loiter %d (%.0f-%.0f s)" % (si + 1, s["a"], s["b"]))
            ax.plot(PX[mm][0], PY[mm][0], "o", color=c, ms=9, mfc="white", mew=2, zorder=4)
            R = float(np.median(s["v_rad"]))
            ax.plot(*ring(s["cx"], s["cy"], R), "k--", lw=1.4, zorder=2)
            ax.plot(s["cx"], s["cy"], "k+", ms=14, mew=2)
            ax.annotate("L%d R=%.0f m %s" % (si + 1, R, "CW" if s["dir"] > 0 else "CCW"), (s["cx"], s["cy"]),
                        textcoords="offset points", xytext=(8, 8), fontsize=10, weight="bold")
        ax.plot([], [], "k--", label="lingkaran perintah")
        ax.plot(0, 0, "k^", ms=12, label="HOME / asal")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_xlabel("timur dari HOME [m]")
        ax.set_ylabel("utara dari HOME [m]")
        ax.legend(loc="upper left", fontsize=9)
        ax.set_title("Lingkaran loiter perintah vs lintasan aktual\nLOIT_BANK_DEG=%s, ROLL_LIMIT_DEG=%s, "
                     "LOIT_ROLL_SIGN=%s, NAVL1_PERIOD=%.0f s" % (P["bank"], P["roll_lim"], P["sign"], P["l1_period"]))
        fig.tight_layout()
        fig.savefig(os.path.join(out, "01_lingkaran_rencana_vs_aktual.png"), dpi=130)
        plt.close(fig)

        # ---- 02 per segmen: lintasan relatif pusat + L1
        fig, axes = plt.subplots(1, NS, figsize=(9 * NS, 9), squeeze=False)
        for si, s in enumerate(segs):
            ax = axes[0, si]
            S = S_all[si]
            R = float(np.median(s["v_rad"]))
            pa = s["a"] - 30
            mw = (tp >= pa) & (tp <= s["a"])
            ax.plot(PX[mw] - s["cx"], PY[mw] - s["cy"], color="0.7", lw=1, label="30 s sebelum loiter")
            ax.plot(*ring(0, 0, R), "k--", lw=1.4, label="radius perintah %.0f m" % R)
            ax.fill(*ring(0, 0, R + TOL), color="0.9", zorder=0)
            ax.fill(*ring(0, 0, max(R - TOL, 0)), color="white", zorder=0)
            sc = ax.scatter(S[:, 1], S[:, 2], c=S[:, 0] - s["a"], cmap="viridis", s=10, zorder=3)
            plt.colorbar(sc, ax=ax, shrink=0.7, label="detik sejak loiter")
            for j in range(0, len(S), 25):
                ax.add_patch(plt.Circle((S[j, 1], S[j, 2]), S[j, 8], fill=False, color="orange", alpha=0.35, lw=0.8))
            ax.plot([], [], color="orange", label="lingkaran L1 distance (contoh)")
            ax.plot(0, 0, "k+", ms=14, mew=2)
            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
            ax.legend(loc="upper left", fontsize=8)
            ax.set_xlabel("timur dari pusat [m]")
            ax.set_ylabel("utara dari pusat [m]")
            sm = summary[si]
            ax.set_title("Loiter %d: perintah %s, aktual %s (%.0f deg) -- abu-abu = ±%.0f m"
                         % (si + 1, sm["arah_cmd"], sm["arah_aktual"], sm["sudut_putar_total"], TOL))
        fig.tight_layout()
        fig.savefig(os.path.join(out, "02_lintasan_loiter_dan_L1.png"), dpi=120)
        plt.close(fig)

        # ---- 03 galat radius vs waktu
        fig, axes = plt.subplots(3, NS, figsize=(9 * NS, 11), squeeze=False, sharex="col")
        for si, s in enumerate(segs):
            S = S_all[si]
            ax = axes[:, si]
            ax[0].plot(S[:, 0], S[:, 5], "b.-", ms=3, lw=1, label="galat dari GPS (+ = di luar lingkaran)")
            ml = (tErr >= s["a"]) & (tErr <= s["b"])
            if ml.any():
                ax[0].plot(tErr[ml], lerr[ml], "rs", ms=3, label="LOIT_ERR telemetri FC")
            for yv in (TOL, -TOL):
                ax[0].axhline(yv, color="r", ls="--", lw=1)
            ax[0].axhline(0, color="k", lw=0.6)
            ax[0].axhspan(-RECOVERY_BAND_M, RECOVERY_BAND_M, color="green", alpha=0.12,
                          label="pita konvergen ±%g m" % RECOVERY_BAND_M)
            ax[0].set_ylabel("galat radius [m]")
            ax[0].legend(fontsize=8, loc="upper left")
            ax[0].set_title("Loiter %d: galat radius (jarak ke pusat - radius)" % (si + 1))
            ax[1].plot(S[:, 0], np.abs(np.gradient(S[:, 5], S[:, 0])), "m", lw=1, label="|d galat/dt| [m/s]")
            ax[1].set_ylabel("m/s")
            ax[1].legend(fontsize=8)
            ax[2].plot(S[:, 0], S[:, 3], "k", lw=1, label="jarak ke pusat (GPS)")
            mn = (tn >= s["a"]) & (tn <= s["b"])
            ax[2].plot(tn[mn], wp_dist[mn], "o", color="gray", ms=3, label="wp_dist telemetri")
            ax[2].plot(S[:, 0], S[:, 4], "r", lw=1.3, label="radius perintah")
            ax[2].plot(S[:, 0], S[:, 8], color="orange", lw=1.3, label="L1 distance (hitung)")
            ax2 = ax[2].twinx()
            ax2.plot(S[:, 0], S[:, 6] / 360.0, "b", lw=1)
            ax2.set_ylabel("putaran kumulatif (+ = CW)", color="b")
            ax[2].set_ylabel("m")
            ax[2].set_xlabel("waktu log [s]")
            ax[2].legend(fontsize=8, loc="upper left")
            for L in laps:
                if L["segmen"] == si + 1 and L["putaran"] > 1:
                    for x in ax:
                        x.axvline(L["t_mulai"], color="green", lw=0.7, alpha=0.6)
            for x in ax:
                x.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "03_galat_radius_vs_waktu.png"), dpi=120)
        plt.close(fig)

        # ---- 04 kendali attitude saat loiter
        fig, axes = plt.subplots(4, NS, figsize=(9 * NS, 12), squeeze=False, sharex="col")
        for si, s in enumerate(segs):
            a, b = s["a"], s["b"]
            pa, pb = a - 5, b + 5
            ax = axes[:, si]
            mt = (ta >= pa) & (ta <= pb)
            mn = (tn >= pa) & (tn <= pb)
            ms_ = (ts >= pa) & (ts <= pb)
            bank = summary[si]["batas_bank"]
            for x in ax:
                x.axvspan(a, b, color=MODE_COLOR["LOITER"], alpha=0.6, lw=0)
                x.grid(alpha=0.3)
            ax[0].plot(ta[mt], roll[mt], "b", lw=1, label="roll aktual")
            ax[0].step(tn[mn], nav_roll[mn], "r", where="post", lw=1.4, label="nav_roll L1")
            if np.isfinite(bank):
                ax[0].axhline(bank, color="gray", ls=":")
                ax[0].axhline(-bank, color="gray", ls=":", label="batas bank loiter %.0f" % bank)
            ax[0].set_ylabel("roll [deg] (+ = kanan)")
            ax[0].legend(fontsize=8)
            ax[0].set_title("Loiter %d: kendali attitude (RMSE roll %.1f deg, korelasi %.2f)"
                            % (si + 1, summary[si]["roll_track_rmse"], summary[si]["corr_roll_navroll"]))
            ax[1].plot(ts[ms_], ail[ms_], "b", lw=0.8, label="aileron PWM")
            ax[1].set_ylabel("PWM")
            ax[1].legend(fontsize=8)
            ax[2].plot(ta[mt], pitch[mt], "g", lw=1, label="pitch aktual")
            ax[2].step(tn[mn], nav_pitch[mn], "r", where="post", lw=1.4, label="nav_pitch TECS")
            ax[2].set_ylabel("pitch [deg]")
            ax[2].legend(fontsize=8)
            ax[3].plot(ts[ms_], ele[ms_], "g", lw=0.8, label="elevator PWM")
            ax[3].set_ylabel("PWM")
            ax[3].set_xlabel("waktu log [s]")
            ax[3].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "04_attitude_loiter_roll_pitch.png"), dpi=120)
        plt.close(fig)

        # ---- 05 altitude, kecepatan, throttle
        fig, axes = plt.subplots(3, NS, figsize=(9 * NS, 10), squeeze=False, sharex="col")
        for si, s in enumerate(segs):
            a, b = s["a"], s["b"]
            pa, pb = a - 20, b + 20
            ax = axes[:, si]
            for x in ax:
                x.axvspan(a, b, color=MODE_COLOR["LOITER"], alpha=0.6, lw=0)
                x.grid(alpha=0.3)
                x.set_xlim(pa, pb)
            mv = (tv >= pa) & (tv <= pb)
            ax[0].plot(tv[mv], alt[mv], "k", label="alt VFR_HUD")
            for k_, c in (("ALT_FUSED", "b"), ("ALT_BARO", "orange"), ("ALT_GPS", "g")):
                t_, v_ = alt_named[k_]
                mm = (t_ >= pa) & (t_ <= pb)
                if mm.any():
                    ax[0].plot(t_[mm], v_[mm], c, lw=0.8, label=k_)
            if np.isfinite(summary[si]["alt_target"]):
                ax[0].axhline(summary[si]["alt_target"], color="r", ls="--",
                              label="target %.0f m (RMS galat %.1f m)" % (summary[si]["alt_target"], summary[si]["alt_galat_rms"]))
            ax[0].set_ylabel("m")
            ax[0].legend(fontsize=8)
            ax[0].set_title("Loiter %d: altitude & kecepatan" % (si + 1))
            ax[1].plot(tv[mv], gs[mv], "b", label="groundspeed")
            ax[1].plot(tv[mv], aspd[mv], "r", label="airspeed")
            if np.isfinite(P["cruise"]):
                ax[1].axhline(P["cruise"], color="r", ls=":", lw=0.8, label="AIRSPEED_CRUISE")
            ax[1].set_ylabel("m/s")
            ax[1].legend(fontsize=8)
            ax[2].plot(tv[mv], thr[mv], "k", label="throttle VFR_HUD %")
            mr_ = (tr >= pa) & (tr <= pb)
            ax[2].plot(tr[mr_], (ch3[mr_] - 988) / 10.24, "orange", lw=0.8, label="stick throttle ch3 %")
            ax[2].set_ylabel("%")
            ax[2].set_xlabel("waktu log [s]")
            ax[2].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "05_altitude_kecepatan_throttle.png"), dpi=120)
        plt.close(fig)

    # ---- 06 kualitas GPS (seluruh log)
    fig, ax = plt.subplots(3, 1, figsize=(15, 9), sharex=True)
    for x in ax:
        shade_iv(x, iv, labels=(x is ax[0]))
        x.grid(alpha=0.3)
    ax[0].plot(tg, sats, label="satelit")
    ax[0].plot(tg, fix * 5, "k", lw=0.8, label="fix_type x5 (15 = 3D)")
    ax[0].legend(fontsize=8, loc="upper right")
    ax[1].plot(tg, eph, "r", label="HDOP (eph/100)")
    ax[1].legend(fontsize=8)
    age = np.zeros_like(tg)
    last_t = tg[0]
    for i in range(1, len(tg)):
        if glat[i] != glat[i - 1]:
            last_t = tg[i]
        age[i] = tg[i] - last_t
    ax[2].plot(tg, age, "m", lw=0.8, label="usia posisi GPS (s sejak lat berubah) -- proxy")
    ax[2].set_ylabel("s")
    ax[2].set_xlabel("waktu log [s]")
    ax[2].legend(fontsize=8)
    fig.suptitle("Kualitas GPS sepanjang log (latar = mode efektif)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "06_kualitas_gps.png"), dpi=120)
    plt.close(fig)

    # ---- 07 metrik per putaran
    if laps:
        fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
        labels = ["L%d P%d%s" % (L["segmen"], L["putaran"], "" if L["lengkap"] == "YA" else "*") for L in laps]
        xs = np.arange(len(laps))
        cols = [scol[(L["segmen"] - 1) % len(scol)] for L in laps]
        ax[0].bar(xs - 0.2, [L["galat_mean_abs_m"] for L in laps], 0.4, color=cols, alpha=0.5, label="|galat| rata-rata")
        ax[0].bar(xs + 0.2, [L["galat_max_abs_m"] for L in laps], 0.4, color=cols, label="|galat| maks")
        ax[0].axhline(TOL, color="r", ls="--", label="toleransi %.0f m" % TOL)
        ax[0].set_ylabel("m")
        ax[0].legend(fontsize=8)
        ax[0].set_title("Galat radius per putaran (* = putaran parsial)")
        ax[1].bar(xs - 0.2, [L["radius_cmd_m"] for L in laps], 0.4, color="0.8", label="radius perintah")
        ax[1].bar(xs + 0.2, [L["radius_aktual_mean_m"] for L in laps], 0.4, color=cols,
                  yerr=[L["radius_aktual_std_m"] for L in laps], capsize=3, label="radius aktual (rata2 ± std)")
        ax[1].set_ylabel("m")
        ax[1].legend(fontsize=8)
        ax[1].set_title("Radius perintah vs aktual")
        ax[2].bar(xs, [L["durasi_s"] for L in laps], color=cols, label="durasi putaran")
        ax2 = ax[2].twinx()
        ax2.plot(xs, [L["nav_roll_sat_frac"] * 100 for L in laps], "ko-", label="nav_roll jenuh %")
        ax2.set_ylabel("% waktu nav_roll di batas bank")
        ax2.set_ylim(0, 105)
        ax[2].set_ylabel("s")
        ax[2].set_title("Durasi putaran & kejenuhan nav_roll")
        ax[2].legend(fontsize=8, loc="upper left")
        ax2.legend(fontsize=8, loc="upper right")
        for x in ax:
            x.set_xticks(xs)
            x.set_xticklabels(labels, rotation=70, fontsize=8)
            x.grid(alpha=0.3, axis="y")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "07_metrik_per_putaran.png"), dpi=120)
        plt.close(fig)

    # ---- 08 timeline seluruh log
    fig, ax = plt.subplots(3, 1, figsize=(16, 8), sharex=True)
    for x in ax:
        shade_iv(x, iv, labels=(x is ax[0]))
        x.grid(alpha=0.3)
    ax[0].plot(ta, roll, "b", lw=0.6, label="roll")
    ax[0].legend(fontsize=8, loc="upper right")
    ax[1].plot(tv, alt, "k", lw=0.8, label="alt")
    ax[1].legend(fontsize=8)
    ax[2].plot(tv, gs, "b", lw=0.8, label="groundspeed")
    ax[2].plot(tv, aspd, "m", lw=0.8, label="airspeed")
    ax[2].legend(fontsize=8)
    ax[2].set_xlabel("waktu log [s]")
    for d in D["STATUSTEXT"]:
        if d["_src"] == FC and ("REBOOT" in d["text"] or "failed" in d["text"]):
            for x in ax:
                x.axvline(d["_t"] - t0, color="r", lw=0.8)
    fig.suptitle("Timeline seluruh log -- latar = mode efektif (hijau = LOITER, termasuk item loiter di AUTO); "
                 "merah: REBOOT / init failed")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "08_timeline_seluruh_log.png"), dpi=120)
    plt.close(fig)

    # ---- 11 gaya jurnal per segmen
    for si, s in enumerate(segs if do_loiter else []):
        S = S_all[si]
        R = float(np.median(s["v_rad"]))
        fig, ax = plt.subplots(figsize=(8.5, 8.2))
        ax.plot(*ring(0, 0, R + TOL), ":", color="navy", lw=1.8, label="Toleransi ±%.0f m" % TOL)
        if R - TOL > 0:
            ax.plot(*ring(0, 0, R - TOL), ":", color="navy", lw=1.8)
        ax.plot(*ring(0, 0, R), "-", color="red", lw=1.6, label="Setpoint (lingkaran R=%.0f m)" % R)
        ax.plot(S[:, 1], S[:, 2], "-", color="black", lw=2.6, label="Lintasan UAV (LOITER)")
        ax.plot(0, 0, "r+", ms=14, mew=2)
        ax.annotate("pusat", (0, 0), xytext=(6, -14), textcoords="offset points", color="red", fontsize=9)
        cog_a = float(np.interp(s["a"], tg, cog))
        hr = math.radians(90.0 - cog_a)
        fwd, side = np.array([math.cos(hr), math.sin(hr)]), np.array([-math.sin(hr), math.cos(hr)])
        p0 = np.array([S[0, 1], S[0, 2]])
        sz = max(R, 40.0) / 6.0
        tri = [p0 + fwd * sz, p0 - fwd * sz * 0.6 + side * sz * 0.65, p0 - fwd * sz * 0.6 - side * sz * 0.65]
        ax.add_patch(plt.Polygon(tri, closed=True, fc="darkorange", ec="k", zorder=6))
        ax.annotate("UAV", (p0[0], p0[1]), xytext=(12, 10), textcoords="offset points", fontsize=12)
        i_max = int(np.argmax(np.abs(S[:, 5])))
        ax.plot(S[i_max, 1], S[i_max, 2], "o", ms=9, mfc="none", mec="m", mew=2, zorder=6)
        ax.annotate("galat maks %.1f m" % abs(S[i_max, 5]), (S[i_max, 1], S[i_max, 2]), xytext=(12, 2),
                    textcoords="offset points", color="m", fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlabel("Jarak sumbu-x / timur dari pusat (m)")
        ax.set_ylabel("Jarak sumbu-y / utara dari pusat (m)")
        ax.set_title("Loiter %d -- lingkaran setpoint vs lintasan aktual (%s, t=%.0f-%.0f s)"
                     % (si + 1, summary[si]["arah_cmd"], s["a"], s["b"]), fontsize=10)
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=3, fontsize=9, frameon=False)
        ax.tick_params(direction="in")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "11_lingkaran_rencana_vs_aktual_terbang%d.png" % (si + 1)), dpi=150)
        plt.close(fig)

    # ================================================================ SELURUH MODE
    mode_rows = []
    if not do_full:
        for r in summary:
            print("loiter:", r)
        print("selesai ->", out)
        return
    for name in MODE_ORDER + sorted({n for _, _, n in iv} - set(MODE_ORDER)):
        ivs = [(a, b) for a, b, n in iv if n == name]
        if not ivs:
            continue

        def mask(t):
            mm = np.zeros(len(t), bool)
            for a, b in ivs:
                mm |= (t >= a) & (t <= b)
            return mm

        mt, mv, mn, mp_ = mask(ta), mask(tv), mask(tn), mask(tp)
        dist = 0.0
        for a, b in ivs:
            mm = (tp >= a) & (tp <= b)
            if mm.sum() > 1:
                dist += float(np.sum(np.hypot(np.diff(PX[mm]), np.diff(PY[mm]))))
        has_nav = name in ("AUTO", "LOITER", "GUIDED", "RTL") and mn.any() and mt.any()
        if has_nav:
            nr = nav_roll[np.clip(np.searchsorted(tn, ta[mt], side="right") - 1, 0, len(tn) - 1)]
            npch = nav_pitch[np.clip(np.searchsorted(tn, ta[mt], side="right") - 1, 0, len(tn) - 1)]
        mode_rows.append(dict(
            mode=name, jumlah_segmen=len(ivs), durasi_s=round(sum(b - a for a, b in ivs), 1),
            jarak_tempuh_m=round(dist, 0),
            roll_mean=round(float(np.mean(roll[mt])), 1) if mt.any() else np.nan,
            roll_std=round(float(np.std(roll[mt])), 1) if mt.any() else np.nan,
            roll_min=round(float(np.min(roll[mt])), 1) if mt.any() else np.nan,
            roll_max=round(float(np.max(roll[mt])), 1) if mt.any() else np.nan,
            pitch_mean=round(float(np.mean(pitch[mt])), 1) if mt.any() else np.nan,
            pitch_std=round(float(np.std(pitch[mt])), 1) if mt.any() else np.nan,
            alt_mean=round(float(np.mean(alt[mv])), 1) if mv.any() else np.nan,
            alt_min=round(float(np.min(alt[mv])), 1) if mv.any() else np.nan,
            alt_max=round(float(np.max(alt[mv])), 1) if mv.any() else np.nan,
            gs_mean=round(float(np.mean(gs[mv])), 1) if mv.any() else np.nan,
            airspeed_mean=round(float(np.mean(aspd[mv])), 1) if mv.any() else np.nan,
            airspeed_min=round(float(np.min(aspd[mv])), 1) if mv.any() else np.nan,
            throttle_mean=round(float(np.mean(thr[mv])), 1) if mv.any() else np.nan,
            roll_track_rmse=round(float(np.sqrt(np.mean((roll[mt] - nr) ** 2))), 1) if has_nav else "-",
            pitch_track_rmse=round(float(np.sqrt(np.mean((pitch[mt] - npch) ** 2))), 1) if has_nav else "-",
            persen_roll_lewat_40=round(100 * float(np.mean(np.abs(roll[mt]) > 40)), 1) if mt.any() else np.nan,
        ))
    write_csv("ringkasan_per_mode.csv", mode_rows)

    # ---- 09 lintasan berwarna mode
    fig, ax = plt.subplots(figsize=(11, 11))
    seen = set()
    for a, b, n in iv:
        mm = (tp >= a) & (tp <= b)
        ax.plot(PX[mm], PY[mm], color=MODE_COLOR_LINE.get(n, "k"), lw=2.0 if n == "LOITER" else 1.4,
                label=None if n in seen else n)
        seen.add(n)
    for si, s in enumerate(segs):
        ax.plot(*ring(s["cx"], s["cy"], float(np.median(s["v_rad"]))), "k--", lw=1,
                label="lingkaran loiter perintah" if si == 0 else None)
    ax.plot(0, 0, "k^", ms=12, label="HOME / asal")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("timur [m]")
    ax.set_ylabel("utara [m]")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("Lintasan seluruh terbang berwarna mode efektif")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "09_lintasan_seluruh_mode.png"), dpi=130)
    plt.close(fig)

    # ---- 10 attitude seluruh mode
    fa, fb = (flights[0][0] - 5, flights[-1][1] + 5) if flights else (0, T_END)
    fig, ax = plt.subplots(4, 1, figsize=(18, 12), sharex=True)
    mt = (ta >= fa) & (ta <= fb)
    mn = (tn >= fa) & (tn <= fb)
    ms_ = (ts >= fa) & (ts <= fb)
    for x in ax:
        shade_iv(x, iv, labels=(x is ax[0]))
        x.grid(alpha=0.3)
        x.set_xlim(fa, fb)
    # nav_roll/nav_pitch hanya bermakna saat mode navigasi
    nav_mask_n = np.zeros(len(tn), bool)
    for a, b, n in iv:
        if n in ("AUTO", "LOITER", "GUIDED", "RTL"):
            nav_mask_n |= (tn >= a) & (tn <= b)
    nr_plot = np.where(nav_mask_n, nav_roll, np.nan)
    np_plot = np.where(nav_mask_n, nav_pitch, np.nan)
    ax[0].plot(ta[mt], roll[mt], "b", lw=0.7, label="roll aktual")
    ax[0].step(tn[mn], nr_plot[mn], "r", where="post", lw=1, label="nav_roll (AUTO/LOITER)")
    for lim, lab in ((P["roll_lim"], "ROLL_LIMIT_DEG"), (P["bank"], "LOIT_BANK_DEG")):
        if np.isfinite(lim):
            ax[0].axhline(lim, color="gray", ls=":", lw=0.8)
            ax[0].axhline(-lim, color="gray", ls=":", lw=0.8)
    ax[0].set_ylabel("roll [deg]")
    ax[0].legend(fontsize=8, loc="lower left")
    ax[1].plot(ts[ms_], ail[ms_], "b", lw=0.6, label="aileron PWM")
    ax[1].set_ylabel("PWM")
    ax[1].legend(fontsize=8)
    ax[2].plot(ta[mt], pitch[mt], "g", lw=0.7, label="pitch aktual")
    ax[2].step(tn[mn], np_plot[mn], "r", where="post", lw=1, label="nav_pitch (AUTO/LOITER)")
    ax[2].set_ylabel("pitch [deg]")
    ax[2].legend(fontsize=8)
    ax[3].plot(ts[ms_], ele[ms_], "g", lw=0.6, label="elevator PWM")
    ax[3].set_ylabel("PWM")
    ax[3].set_xlabel("waktu log [s]")
    ax[3].legend(fontsize=8)
    fig.suptitle("Kendali attitude sepanjang terbang (latar = mode efektif)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "10_attitude_seluruh_mode.png"), dpi=120)
    plt.close(fig)

    # ---- 12 statistik per mode
    if mode_rows:
        names = [r["mode"] for r in mode_rows]
        cols = [MODE_COLOR_LINE.get(n, "k") for n in names]
        xs = np.arange(len(names))
        fig, ax = plt.subplots(2, 2, figsize=(15, 10))
        ax[0, 0].bar(xs, [r["durasi_s"] for r in mode_rows], color=cols)
        for i, r in enumerate(mode_rows):
            ax[0, 0].annotate("%.0f s\n%.0f m" % (r["durasi_s"], r["jarak_tempuh_m"]), (i, r["durasi_s"]),
                              ha="center", va="bottom", fontsize=8)
        ax[0, 0].set_title("Durasi & jarak tempuh per mode")
        ax[0, 0].set_ylabel("s")
        ax[0, 1].bar(xs, [r["roll_std"] for r in mode_rows], color=cols, alpha=0.6, label="std roll")
        ax[0, 1].errorbar(xs, [r["roll_mean"] for r in mode_rows],
                          yerr=[[r["roll_mean"] - r["roll_min"] for r in mode_rows],
                                [r["roll_max"] - r["roll_mean"] for r in mode_rows]],
                          fmt="ko", capsize=4, label="rata2 (min..maks)")
        ax[0, 1].axhline(0, color="k", lw=0.6)
        ax[0, 1].set_title("Roll per mode")
        ax[0, 1].set_ylabel("deg")
        ax[0, 1].legend(fontsize=8)
        ax[1, 0].errorbar(xs, [r["alt_mean"] for r in mode_rows],
                          yerr=[[r["alt_mean"] - r["alt_min"] for r in mode_rows],
                                [r["alt_max"] - r["alt_mean"] for r in mode_rows]],
                          fmt="o", color="k", capsize=4)
        ax[1, 0].set_title("Altitude per mode (rata2, min..maks)")
        ax[1, 0].set_ylabel("m")
        ax[1, 1].bar(xs - 0.2, [r["gs_mean"] for r in mode_rows], 0.4, color=cols, alpha=0.5, label="groundspeed")
        ax[1, 1].bar(xs + 0.2, [r["airspeed_mean"] for r in mode_rows], 0.4, color=cols, label="airspeed")
        if np.isfinite(P["cruise"]):
            ax[1, 1].axhline(P["cruise"], color="r", ls=":", label="AIRSPEED_CRUISE")
        ax[1, 1].set_ylim(0, 1.3 * max(max(r["gs_mean"], r["airspeed_mean"]) for r in mode_rows))
        ax[1, 1].set_title("Kecepatan rata-rata per mode")
        ax[1, 1].set_ylabel("m/s")
        ax[1, 1].legend(fontsize=8, loc="upper left", ncol=3)
        for x in ax.flat:
            x.set_xticks(xs)
            x.set_xticklabels(names)
            x.grid(alpha=0.3, axis="y")
        fig.suptitle("Statistik per mode terbang (mode efektif, hanya saat armed)")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "12_statistik_per_mode.png"), dpi=120)
        plt.close(fig)

    for r in summary:
        print("loiter:", r)
    for L in laps:
        print("putaran:", L)
    for dv in deviasi:
        print("deviasi:", {k: (round(v, 1) if isinstance(v, float) else v) for k, v in dv.items()})
    for r in mode_rows:
        print("mode:", r)
    print("selesai ->", out)


if __name__ == "__main__":
    main()
