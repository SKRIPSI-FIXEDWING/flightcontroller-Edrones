#!/usr/bin/env python3
"""Analisis AUTO (kendali attitude + navigasi L1) dari satu tlog Mission Planner.

Dibuat untuk uji terbang 2026-10-15 (L1 periode tetap). Menghitung metrik
evaluasi proposal per segmen (leg) antar-waypoint:
  CTE rata-rata, CTE maksimum, perubahan CTE (dCTE/dt), waktu pemulihan,
  overshoot saat transisi, kestabilan respons, dan pelanggaran batas 45 m.

CTE dihitung ulang dari GLOBAL_POSITION_INT (~5 Hz) terhadap garis
prev_wp -> next_wp, mengikuti Navigation::setNextWaypoint(): leg pertama
dimulai dari posisi saat masuk AUTO, leg berikutnya dari WP sebelumnya.
NAV_CONTROLLER_OUTPUT (1 Hz) hanya dipakai sebagai pembanding.

Run:
    python analyze_tlog_l1_auto.py "path/ke/2026-10-15 12-37-26.tlog" [-o out_dir]
"""

import argparse
import collections
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

R2D = 180.0 / np.pi
MODE_NAME = {0: "MANUAL", 5: "FBWA", 10: "AUTO", 4: "GUIDED"}
MODE_COLOR = {"FBWA": "#cfe8ff", "AUTO": "#ffd9b3", "MANUAL": "#e6e6e6"}
CTE_LIMIT_M = 45.0          # batas proposal
RECOVERY_BAND_M = 5.0       # |CTE| dianggap pulih bila <= nilai ini ...
RECOVERY_HOLD_S = 2.0       # ... selama minimal sekian detik
WP_RADIUS_M = 10.0          # fallback; ditimpa param WP_RADIUS dari log bila ada
NAV_ROLL_LIMIT = 36.0       # NavigationConfig::roll_limit_deg
L1_K = 0.3183099            # L1 = (1/pi) * damping * period * groundspeed


def load(path):
    conn = mavutil.mavlink_connection(path, dialect="ardupilotmega")
    D = collections.defaultdict(list)
    while True:
        m = conn.recv_match()
        if m is None:
            break
        d = m.to_dict()
        d["_t"] = m._timestamp
        d["_src"] = (m.get_srcSystem(), m.get_srcComponent())
        D[m.get_type()].append(d)
    return D


def fc_time(rows, t0, usec=False):
    """Waktu dari jam FC (time_boot_ms) dipetakan ke sumbu waktu log.

    Timestamp tlog (_t) adalah waktu TERIMA di GCS: paket datang berkelompok
    sehingga banyak sampel ber-_t sama dan dCTE/dt meledak. Jam FC rapat dan
    monoton; offset-nya ke _t diambil median per sesi boot (reset saat reboot).
    SERVO_OUTPUT_RAW firmware ini mengisi time_usec dengan millis(), jadi
    diperlakukan sebagai milidetik juga. GPS_RAW_INT.time_usec benar-benar
    mikrodetik (usec=True).
    """
    key = "time_boot_ms" if "time_boot_ms" in rows[0] else "time_usec"
    tb = np.array([r[key] for r in rows], float) / (1e6 if usec else 1000.0)
    tr = np.array([r["_t"] - t0 for r in rows])
    out = np.empty_like(tb)
    cuts = [0] + list(np.where(np.diff(tb) < -1.0)[0] + 1) + [len(tb)]
    for a, b in zip(cuts[:-1], cuts[1:]):
        out[a:b] = tb[a:b] + np.median(tr[a:b] - tb[a:b])
    return out


def series(D, typ, key, t0, scale=1.0, src=None):
    rows = [d for d in D[typ] if src is None or d["_src"] == src]
    has_clock = "time_boot_ms" in rows[0] or typ in ("SERVO_OUTPUT_RAW", "GPS_RAW_INT")
    t = (fc_time(rows, t0, usec=(typ == "GPS_RAW_INT")) if has_clock
         else np.array([d["_t"] - t0 for d in rows]))
    v = np.array([d[key] * scale for d in rows], dtype=float)
    o = np.argsort(t, kind="stable")
    return t[o], v[o]


def mode_intervals(D, t0, t_end):
    ev, last = [], None
    for h in D["HEARTBEAT"]:
        if h["type"] != 1:
            continue
        key = (h["custom_mode"], bool(h["base_mode"] & 128))
        if key != last:
            ev.append((h["_t"] - t0, key))
            last = key
    out = []
    for i, (t, (cm, armed)) in enumerate(ev):
        te = ev[i + 1][0] if i + 1 < len(ev) else t_end
        out.append((t, te, MODE_NAME.get(cm, "UNK%d" % cm), armed))
    return out


def last_mission(D, t0):
    """Item misi terakhir yang lengkap (download FC->GCS atau upload GCS->FC)."""
    best = {}
    for typ, scale in (("MISSION_ITEM_INT", 1e-7), ("MISSION_ITEM", 1.0)):
        for d in D.get(typ, []):
            best[d["seq"]] = (d["x"] * scale, d["y"] * scale, d["z"], d["param2"], d["_t"] - t0)
    n = max(best) + 1
    return [best[i] for i in range(n) if i in best]


def signed_cte(px, py, ax, ay, bx, by):
    """CTE bertanda: positif = wahana di KANAN garis A->B. Juga proporsi along-track."""
    dx, dy = bx - ax, by - ay
    L = np.hypot(dx, dy)
    rx, ry = px - ax, py - ay
    cte = -(dx * ry - dy * rx) / L
    along = (rx * dx + ry * dy) / L
    return cte, along, L


def corridor(xs, ys, d):
    """Garis batas koridor toleransi +-d di sekitar polyline (sambungan miter).

    Pengganti shapely.buffer(d, cap_style=flat, join_style=mitre): offset kiri
    (maju) + offset kanan (mundur) + tutup ujung datar -> satu kontur tertutup.
    """
    P = np.c_[xs, ys].astype(float)
    seg = np.diff(P, axis=0)
    u = seg / np.linalg.norm(seg, axis=1)[:, None]
    n = np.c_[-u[:, 1], u[:, 0]]  # normal kiri
    left, right = [], []
    for i in range(len(P)):
        if i == 0:
            m, k = n[0], 1.0
        elif i == len(P) - 1:
            m, k = n[-1], 1.0
        else:
            b = n[i - 1] + n[i]
            m = b / np.linalg.norm(b)
            k = 1.0 / max(float(m @ n[i]), 0.2)  # panjang miter
        left.append(P[i] + m * d * k)
        right.append(P[i] - m * d * k)
    ring = np.array(left + right[::-1] + [left[0]])
    return ring[:, 0], ring[:, 1]


def dist_to_polyline(px, py, xs, ys):
    """Jarak terdekat tiap titik ke polyline (tanpa memandang WP aktif)."""
    best = np.full(len(px), np.inf)
    for i in range(len(xs) - 1):
        ax, ay, bx, by = xs[i], ys[i], xs[i + 1], ys[i + 1]
        dx, dy = bx - ax, by - ay
        t = np.clip(((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
        best = np.minimum(best, np.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def shade(ax, iv, labels=False):
    seen = set()
    for a, b, n, armed in iv:
        if not armed:
            continue
        ax.axvspan(a, b, color=MODE_COLOR.get(n, "#eee"), alpha=0.6, lw=0,
                   label=(n if labels and n not in seen else None))
        seen.add(n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tlog")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--plan", default=None,
                    help="file .waypoints Mission Planner, dipakai bila tlog tidak memuat daftar WP lengkap")
    args = ap.parse_args()
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_2026-10-15")
    os.makedirs(out, exist_ok=True)

    D = load(args.tlog)
    FC = (1, 1)
    t0 = D["ATTITUDE"][0]["_t"]
    ta, roll = series(D, "ATTITUDE", "roll", t0, R2D)
    _, pitch = series(D, "ATTITUDE", "pitch", t0, R2D)
    T_END = ta[-1]
    iv = mode_intervals(D, t0, T_END)
    autos = [(a, b) for a, b, n, arm in iv if n == "AUTO" and arm]
    flights = []  # (t_arm, t_disarm)
    for a, b, n, arm in iv:
        if arm and (not flights or a > flights[-1][1] + 1.0):
            flights.append([a, b])
        elif arm:
            flights[-1][1] = b

    global WP_RADIUS_M
    WP_RADIUS_M = next((p["param_value"] for p in D["PARAM_VALUE"] if p["param_id"] == "WP_RADIUS"), WP_RADIUS_M)
    mission = last_mission(D, t0)
    if args.plan:
        rows_p = [ln.split("	") for ln in open(args.plan, encoding="utf-8").read().splitlines()[1:] if ln.strip()]
        plan = [(float(r[8]), float(r[9]), float(r[10]), float(r[5]), 0.0) for r in rows_p]
        if len(mission) > 1 and len(mission) != len(plan):
            print("PERINGATAN: misi di tlog (%d item) beda dengan plan (%d item)" % (len(mission), len(plan)))
        if len(mission) >= 1 and (abs(mission[0][0] - plan[0][0]) > 1e-5 or abs(mission[0][1] - plan[0][1]) > 1e-5):
            print("PERINGATAN: HOME di tlog beda dengan HOME di plan")
        if len(mission) < len(plan):
            mission = plan
    home_lat, home_lon = mission[0][0], mission[0][1]
    kx = 111320.0 * np.cos(np.radians(home_lat))

    def xy(lat, lon):
        return (np.asarray(lon) - home_lon) * kx, (np.asarray(lat) - home_lat) * 111320.0

    WX, WY = xy([m[0] for m in mission], [m[1] for m in mission])
    WZ = np.array([m[2] for m in mission])

    tp, lat = series(D, "GLOBAL_POSITION_INT", "lat", t0, 1e-7)
    _, lon = series(D, "GLOBAL_POSITION_INT", "lon", t0, 1e-7)
    _, rel_alt = series(D, "GLOBAL_POSITION_INT", "relative_alt", t0, 1e-3)
    PX, PY = xy(lat, lon)
    tv, gs = series(D, "VFR_HUD", "groundspeed", t0)
    _, airspeed = series(D, "VFR_HUD", "airspeed", t0)
    _, thr = series(D, "VFR_HUD", "throttle", t0)
    _, alt_hud = series(D, "VFR_HUD", "alt", t0)
    tn, nav_roll = series(D, "NAV_CONTROLLER_OUTPUT", "nav_roll", t0)
    _, nav_pitch = series(D, "NAV_CONTROLLER_OUTPUT", "nav_pitch", t0)
    _, xtrack_tlm = series(D, "NAV_CONTROLLER_OUTPUT", "xtrack_error", t0)
    _, wp_dist_tlm = series(D, "NAV_CONTROLLER_OUTPUT", "wp_dist", t0)
    _, nav_bearing = series(D, "NAV_CONTROLLER_OUTPUT", "nav_bearing", t0)
    ts, ail = series(D, "SERVO_OUTPUT_RAW", "servo1_raw", t0)
    _, ele = series(D, "SERVO_OUTPUT_RAW", "servo2_raw", t0)
    tr, ch3 = series(D, "RC_CHANNELS", "chan3_raw", t0)
    tg, sats = series(D, "GPS_RAW_INT", "satellites_visible", t0)
    _, eph = series(D, "GPS_RAW_INT", "eph", t0, 0.01)
    _, fix = series(D, "GPS_RAW_INT", "fix_type", t0)
    _, glat = series(D, "GPS_RAW_INT", "lat", t0)
    _, cog = series(D, "GPS_RAW_INT", "cog", t0, 0.01)
    named = {}
    for name in sorted({d["name"] for d in D["NAMED_VALUE_FLOAT"]}):
        rows_n = [d for d in D["NAMED_VALUE_FLOAT"] if d["name"] == name]
        tt_n = fc_time(rows_n, t0)
        o = np.argsort(tt_n, kind="stable")
        named[name] = (tt_n[o], np.array([d["value"] for d in rows_n])[o])
    reached = [(d["_t"] - t0, d["seq"]) for d in D.get("MISSION_ITEM_REACHED", [])]

    # nama lama (L1_*) untuk log sebelum 2026-10-18, nama ArduPlane (NAVL1_*) sesudahnya
    l1_period = next((p["param_value"] for p in D["PARAM_VALUE"] if p["param_id"] in ("NAVL1_PERIOD", "L1_PERIOD")), 22.0)
    l1_damp = next((p["param_value"] for p in D["PARAM_VALUE"] if p["param_id"] in ("NAVL1_DAMPING", "L1_DAMPING")), 0.707)
    gs_p = np.interp(tp, tv, gs)
    l1_dist = L1_K * l1_damp * l1_period * gs_p

    # ------------------------------------------------------------------ CTE per sampel AUTO
    rows = []          # per-sampel
    legs = []          # per-leg metrik
    flight_samples = []
    for fi, (a, b) in enumerate(autos):
        m = (tp >= a) & (tp <= b)
        idx = np.where(m)[0]
        ev = sorted([(t, s) for t, s in reached if a <= t <= b + 0.5])
        # Pesan MISSION_ITEM_REACHED bisa hilang di telemetri (mis. log
        # 2026-10-19 11-57: WP7 tidak terkirim). Isi celah seq dengan waktu
        # jarak terdekat ke WP itu di antara klaim sebelum dan sesudahnya,
        # agar CTE tidak dihitung terhadap garis leg yang sudah lewat.
        filled = []
        for k_ev, (t_ev, s_ev) in enumerate(ev):
            if filled and s_ev > filled[-1][1] + 1:
                t_prev = filled[-1][0]
                for miss in range(filled[-1][1] + 1, s_ev):
                    if miss >= len(mission):
                        break
                    mm_ = (tp > t_prev) & (tp < t_ev)
                    if mm_.sum() < 2:
                        break
                    dd = np.hypot(PX[mm_] - WX[miss], PY[mm_] - WY[miss])
                    t_prev = float(tp[mm_][np.argmin(dd)])
                    filled.append((t_prev, miss))
                    print("CATATAN: klaim WP%d tidak ada di telemetri; diperkirakan t=%.1f s (jarak terdekat %.1f m)"
                          % (miss, t_prev, dd.min()))
            filled.append((t_ev, s_ev))
        ev = filled
        # batas leg: [a, t_reach1], [t_reach1, t_reach2], ...
        bounds = [a] + [t for t, _ in ev] + [b]
        seqs = [ev[0][1] if ev else 1] + [s + 1 for _, s in ev]
        start_xy = (PX[idx[0]], PY[idx[0]])
        samp = []
        for k in range(len(bounds) - 1):
            seq = seqs[k]
            if seq >= len(mission):
                continue
            A = start_xy if k == 0 else (WX[seq - 1], WY[seq - 1])
            B = (WX[seq], WY[seq])
            mm = (tp >= bounds[k]) & (tp < bounds[k + 1])
            if mm.sum() < 2:
                continue
            cte, along, L = signed_cte(PX[mm], PY[mm], A[0], A[1], B[0], B[1])
            dist_wp = np.hypot(PX[mm] - B[0], PY[mm] - B[1])
            tt = tp[mm]
            for j in range(len(tt)):
                samp.append((fi + 1, seq, tt[j], PX[mm][j], PY[mm][j], cte[j], along[j], dist_wp[j],
                             gs_p[mm][j], l1_dist[mm][j]))
            rate = np.gradient(cte, tt) if len(tt) > 2 else np.zeros_like(cte)
            # pemulihan: waktu pertama |CTE|<=band dan bertahan RECOVERY_HOLD_S
            rec = np.nan
            inside = np.abs(cte) <= RECOVERY_BAND_M
            for j in range(len(tt)):
                if inside[j]:
                    jj = np.where(tt >= tt[j] + RECOVERY_HOLD_S)[0]
                    if len(jj) and inside[j:jj[0] + 1].all():
                        rec = tt[j] - tt[0]
                        break
            # overshoot: setelah CTE pertama kali berganti tanda, puncak di sisi seberang
            s0 = np.sign(cte[0]) if cte[0] != 0 else 1.0
            cross = np.where(np.sign(cte) == -s0)[0]
            overshoot = float(np.max(np.abs(cte[cross]))) if len(cross) else 0.0
            zc = int(np.sum(np.diff(np.sign(cte)) != 0))
            how = "-"
            t_end_leg = bounds[k + 1]
            hit = [s for t, s in ev if abs(t - t_end_leg) < 1e-6]
            closest = float(dist_wp.min())
            if hit:
                how = ("radius<=%gm" % WP_RADIUS_M) if closest <= WP_RADIUS_M + 2 else "finish-line (lewat garis)"
            else:
                how = "tidak tercapai (AUTO dihentikan)"
            mr = (ta >= bounds[k]) & (ta < bounds[k + 1])
            mn = (tn >= bounds[k]) & (tn < bounds[k + 1])
            legs.append(dict(
                terbang=fi + 1, leg="%s->WP%d" % ("entry" if k == 0 else "WP%d" % (seq - 1), seq),
                t_mulai=round(tt[0], 1), durasi_s=round(tt[-1] - tt[0], 1), panjang_leg_m=round(L, 1),
                cte_mean_abs_m=round(float(np.mean(np.abs(cte))), 1),
                cte_rms_m=round(float(np.sqrt(np.mean(cte ** 2))), 1),
                cte_max_abs_m=round(float(np.max(np.abs(cte))), 1),
                cte_awal_m=round(float(cte[0]), 1),
                dcte_mean_abs_mps=round(float(np.mean(np.abs(rate))), 2),
                dcte_max_abs_mps=round(float(np.max(np.abs(rate))), 2),
                pemulihan_s=("-" if np.isnan(rec) else round(float(rec), 1)),
                overshoot_m=round(overshoot, 1), zero_cross=zc,
                lewat_45m="YA" if np.max(np.abs(cte)) > CTE_LIMIT_M else "tidak",
                jarak_terdekat_wp_m=round(closest, 1), cara_wp_tercapai=how,
                gs_mean=round(float(np.mean(gs_p[mm])), 1), l1_dist_mean_m=round(float(np.mean(l1_dist[mm])), 0),
                roll_mean=round(float(np.mean(roll[mr])), 1) if mr.any() else np.nan,
                nav_roll_sat_frac=round(float(np.mean(np.abs(nav_roll[mn]) >= NAV_ROLL_LIMIT - 0.5)), 2) if mn.any() else np.nan,
            ))
        flight_samples.append(np.array([s[2:] for s in samp]) if samp else np.zeros((0, 8)))
        rows += samp

    with open(os.path.join(out, "cte_per_sampel.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["terbang", "wp_aktif", "t_s", "x_timur_m", "y_utara_m", "cte_m(+kanan)", "along_m",
                    "jarak_ke_wp_m", "groundspeed_mps", "l1_distance_m"])
        for r in rows:
            w.writerow([r[0], r[1]] + ["%.3f" % v for v in r[2:]])
    with open(os.path.join(out, "metrik_per_leg.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(legs[0].keys()))
        w.writeheader()
        w.writerows(legs)

    # ringkasan per terbang
    summary = []
    for fi, S in enumerate(flight_samples):
        if not len(S):
            continue
        cte = S[:, 3]
        tt = S[:, 0]
        a, b = autos[fi]
        mt = (ta >= a) & (ta <= b)
        mn = (tn >= a) & (tn <= b)
        nr_at_att = nav_roll[np.clip(np.searchsorted(tn, ta[mt], side="right") - 1, 0, len(tn) - 1)]
        summary.append(dict(
            terbang=fi + 1, auto_mulai=round(a, 1), auto_selesai=round(b, 1), durasi=round(b - a, 1),
            cte_mean_abs=round(float(np.mean(np.abs(cte))), 1), cte_rms=round(float(np.sqrt(np.mean(cte ** 2))), 1),
            cte_max_abs=round(float(np.max(np.abs(cte))), 1),
            persen_lewat_45m=round(100 * float(np.mean(np.abs(cte) > CTE_LIMIT_M)), 1),
            dcte_max=round(float(np.max(np.abs(np.gradient(cte, tt)))), 1),
            gs_mean=round(float(np.mean(S[:, 6])), 1), l1_mean=round(float(np.mean(S[:, 7])), 0),
            roll_std=round(float(np.std(roll[mt])), 1),
            roll_min=round(float(np.min(roll[mt])), 1), roll_max=round(float(np.max(roll[mt])), 1),
            nav_roll_sat=round(float(np.mean(np.abs(nav_roll[mn]) >= NAV_ROLL_LIMIT - 0.5)), 2),
            roll_track_rmse=round(float(np.sqrt(np.mean((roll[mt] - nr_at_att) ** 2))), 1),
            corr_roll_navroll=round(float(np.corrcoef(roll[mt], nr_at_att)[0, 1]), 2),
        ))
    with open(os.path.join(out, "ringkasan_auto.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)

    # ================================================================== PLOTS
    fcol = ["tab:red", "tab:purple"]

    # ---- 01 jalur rencana vs aktual (semua)
    fig, ax = plt.subplots(figsize=(11, 11))
    for (fa, fb) in flights:
        m = (tp >= fa) & (tp <= fb)
        ax.plot(PX[m], PY[m], color="0.75", lw=1, zorder=1)
    ax.plot([], [], color="0.75", label="FBWA (manual pilot)")
    for fi, (a, b) in enumerate(autos):
        m = (tp >= a) & (tp <= b)
        ax.plot(PX[m], PY[m], color=fcol[fi % 2], lw=2.2, zorder=3, label="AUTO terbang %d (%.0f-%.0f s)" % (fi + 1, a, b))
        ax.plot(PX[m][0], PY[m][0], "o", color=fcol[fi % 2], ms=9, mfc="white", mew=2, zorder=4)
    ax.plot(WX[1:], WY[1:], "k--", lw=1.6, zorder=2, label="jalur rencana (WP1..WP%d)" % (len(mission) - 1))
    for i in range(1, len(mission)):
        ax.add_patch(plt.Circle((WX[i], WY[i]), WP_RADIUS_M, fill=False, ls=":", color="k", lw=0.8))
        ax.annotate("WP%d" % i, (WX[i], WY[i]), textcoords="offset points", xytext=(6, 6), fontsize=10, weight="bold")
    ax.plot(WX[0], WY[0], "k^", ms=12, label="HOME")
    for t, s in reached:
        i = np.argmin(np.abs(tp - t))
        ax.plot(PX[i], PY[i], "x", color="green", ms=9, mew=2, zorder=5)
    ax.plot([], [], "gx", ms=9, mew=2, label="titik 'WP REACHED' diumumkan")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("timur dari HOME [m]")
    ax.set_ylabel("utara dari HOME [m]")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("Jalur rencana vs jalur aktual (lingkaran titik = radius WP %g m)\nNAVL1_PERIOD=%.0f s, NAVL1_DAMPING=%.3f, fuzzy OFF" % (WP_RADIUS_M, l1_period, l1_damp))
    fig.tight_layout()
    fig.savefig(os.path.join(out, "01_jalur_rencana_vs_aktual.png"), dpi=130)
    plt.close(fig)

    # ---- 02 per terbang: zoom jalur + L1 distance
    fig, axes = plt.subplots(1, len(autos), figsize=(9 * len(autos), 9), squeeze=False)
    for fi, (a, b) in enumerate(autos):
        ax = axes[0, fi]
        m = (tp >= a) & (tp <= b)
        ax.plot(WX[1:], WY[1:], "k--", lw=1.5, label="rencana")
        for i in range(1, len(mission)):
            ax.add_patch(plt.Circle((WX[i], WY[i]), WP_RADIUS_M, fill=False, ls=":", color="k"))
            ax.annotate("WP%d" % i, (WX[i], WY[i]), textcoords="offset points", xytext=(6, 6), weight="bold")
        sc = ax.scatter(PX[m], PY[m], c=tp[m] - a, cmap="viridis", s=10, zorder=3)
        plt.colorbar(sc, ax=ax, shrink=0.7, label="detik sejak AUTO")
        # lingkaran L1 di beberapa titik
        for j in np.where(m)[0][::25]:
            ax.add_patch(plt.Circle((PX[j], PY[j]), l1_dist[j], fill=False, color="orange", alpha=0.35, lw=0.8))
        ax.plot([], [], color="orange", label="lingkaran L1 distance (contoh)")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
        ax.set_title("Terbang %d AUTO: jalur aktual berwarna waktu" % (fi + 1))
        ax.set_xlim(WX[1:].min() - 120, WX[1:].max() + 120)
        ax.set_ylim(WY[1:].min() - 120, WY[1:].max() + 120)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "02_jalur_per_terbang_dan_L1.png"), dpi=120)
    plt.close(fig)

    # ---- 03 CTE vs waktu
    fig, axes = plt.subplots(3, len(autos), figsize=(9 * len(autos), 11), squeeze=False, sharex="col")
    for fi, (a, b) in enumerate(autos):
        S = flight_samples[fi]
        ax = axes[:, fi]
        ax[0].plot(S[:, 0], S[:, 3], "b.-", ms=3, lw=1, label="CTE dari GPS (+ = kanan garis)")
        mn = (tn >= a) & (tn <= b)
        ax[0].plot(tn[mn], xtrack_tlm[mn], "rs", ms=4, label="xtrack telemetri FC (1 Hz)")
        for y in (CTE_LIMIT_M, -CTE_LIMIT_M):
            ax[0].axhline(y, color="r", ls="--", lw=1)
        ax[0].axhline(0, color="k", lw=0.6)
        ax[0].axhspan(-RECOVERY_BAND_M, RECOVERY_BAND_M, color="green", alpha=0.12, label="pita pulih ±%g m" % RECOVERY_BAND_M)
        ax[0].set_ylabel("CTE [m]")
        ax[0].legend(fontsize=8, loc="lower left")
        ax[0].set_title("Terbang %d: cross-track error terhadap garis antar-WP" % (fi + 1))
        ax[1].plot(S[:, 0], np.abs(np.gradient(S[:, 3], S[:, 0])), "m", lw=1, label="|dCTE/dt| [m/s]")
        ax[1].set_ylabel("m/s")
        ax[1].legend(fontsize=8)
        ax[2].plot(S[:, 0], S[:, 5], "k", lw=1, label="jarak ke WP aktif (GPS)")
        ax[2].plot(tn[mn], wp_dist_tlm[mn], "o", color="gray", ms=3, label="wp_dist telemetri")
        ax[2].plot(S[:, 0], S[:, 7], color="orange", lw=1.3, label="L1 distance (hitung)")
        ax[2].axhline(WP_RADIUS_M, color="g", ls=":", label="radius WP %g m" % WP_RADIUS_M)
        ax2 = ax[2].twinx()
        ax2.step(S[:, 0], [r[1] for r in rows if r[0] == fi + 1], "b", where="post", lw=1)
        ax2.set_ylabel("WP aktif", color="b")
        ax[2].set_ylabel("m")
        ax[2].set_xlabel("waktu log [s]")
        ax[2].legend(fontsize=8, loc="upper left")
        for t, s in reached:
            if a <= t <= b + 0.5:
                for x in ax:
                    x.axvline(t, color="green", lw=0.7, alpha=0.6)
                ax[0].annotate("WP%d" % s, (t, ax[0].get_ylim()[1] * 0.9), fontsize=8, color="green")
        for x in ax:
            x.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "03_cte_vs_waktu.png"), dpi=120)
    plt.close(fig)

    # ---- 04 kendali attitude AUTO
    fig, axes = plt.subplots(4, len(autos), figsize=(9 * len(autos), 12), squeeze=False, sharex="col")
    for fi, (a, b) in enumerate(autos):
        pa, pb = a - 5, b + 5
        ax = axes[:, fi]
        mt = (ta >= pa) & (ta <= pb)
        mn = (tn >= pa) & (tn <= pb)
        ms_ = (ts >= pa) & (ts <= pb)
        for x in ax:
            x.axvspan(a, b, color=MODE_COLOR["AUTO"], alpha=0.6, lw=0)
            x.grid(alpha=0.3)
        ax[0].plot(ta[mt], roll[mt], "b", lw=1, label="roll aktual")
        ax[0].step(tn[mn], nav_roll[mn], "r", where="post", lw=1.4, label="nav_roll L1 (1 Hz)")
        ax[0].axhline(NAV_ROLL_LIMIT, color="gray", ls=":")
        ax[0].axhline(-NAV_ROLL_LIMIT, color="gray", ls=":")
        ax[0].set_ylabel("roll [deg]")
        ax[0].legend(fontsize=8)
        ax[0].set_title("Terbang %d: kendali attitude di AUTO" % (fi + 1))
        ax[1].plot(ts[ms_], ail[ms_], "b", lw=0.8, label="aileron PWM")
        ax[1].set_ylabel("PWM")
        ax[1].legend(fontsize=8)
        ax[2].plot(ta[mt], pitch[mt], "g", lw=1, label="pitch aktual")
        ax[2].step(tn[mn], nav_pitch[mn], "r", where="post", lw=1.4, label="nav_pitch TECS (1 Hz)")
        ax[2].set_ylabel("pitch [deg]")
        ax[2].legend(fontsize=8)
        ax[3].plot(ts[ms_], ele[ms_], "g", lw=0.8, label="elevator PWM")
        ax[3].set_ylabel("PWM")
        ax[3].set_xlabel("waktu log [s]")
        ax[3].legend(fontsize=8)
        for t, s in reached:
            if pa <= t <= pb:
                for x in ax:
                    x.axvline(t, color="green", lw=0.7, alpha=0.6)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "04_attitude_auto_roll_pitch.png"), dpi=120)
    plt.close(fig)

    # ---- 05 altitude, speed, throttle
    fig, axes = plt.subplots(3, len(autos), figsize=(9 * len(autos), 10), squeeze=False, sharex="col")
    for fi, (a, b) in enumerate(autos):
        pa, pb = a - 20, b + 20
        ax = axes[:, fi]
        for x in ax:
            x.axvspan(a, b, color=MODE_COLOR["AUTO"], alpha=0.6, lw=0)
            x.grid(alpha=0.3)
            x.set_xlim(pa, pb)
        mv = (tv >= pa) & (tv <= pb)
        ax[0].plot(tv[mv], alt_hud[mv], "k", label="alt VFR_HUD")
        for k, c in (("ALT_FUSED", "b"), ("ALT_BARO", "orange"), ("ALT_GPS", "g")):
            if k in named:
                t_, v_ = named[k]
                mm = (t_ >= pa) & (t_ <= pb)
                ax[0].plot(t_[mm], v_[mm], c, lw=0.8, label=k)
        ax[0].axhline(WZ[1], color="r", ls="--", label="alt WP %.0f m" % WZ[1])
        ax[0].set_ylabel("m")
        ax[0].legend(fontsize=8)
        ax[0].set_title("Terbang %d: altitude & kecepatan" % (fi + 1))
        ax[1].plot(tv[mv], gs[mv], "b", label="groundspeed")
        ax[1].plot(tv[mv], airspeed[mv], "r", label="airspeed (0 = sensor gagal init)")
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

    # ---- 06 GPS kualitas + usia data
    fig, ax = plt.subplots(3, 1, figsize=(15, 9), sharex=True)
    for x in ax:
        shade(x, iv, labels=(x is ax[0]))
        x.grid(alpha=0.3)
    ax[0].plot(tg, sats, label="satelit")
    ax[0].plot(tg, fix * 5, "k", lw=0.8, label="fix_type x5 (15 = 3D)")
    ax[0].legend(fontsize=8, loc="upper right")
    ax[1].plot(tg, eph, "r", label="HDOP (eph/100)")
    ax[1].legend(fontsize=8)
    # usia posisi: waktu sejak nilai lat terakhir berubah
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
    fig.suptitle("Kualitas GPS sepanjang log")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "06_kualitas_gps.png"), dpi=120)
    plt.close(fig)

    # ---- 07 metrik per leg (bar)
    fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
    labels = ["T%d %s" % (L["terbang"], L["leg"]) for L in legs]
    xs = np.arange(len(legs))
    cols = [fcol[(L["terbang"] - 1) % 2] for L in legs]
    ax[0].bar(xs - 0.2, [L["cte_mean_abs_m"] for L in legs], 0.4, color=cols, alpha=0.5, label="|CTE| rata-rata")
    ax[0].bar(xs + 0.2, [L["cte_max_abs_m"] for L in legs], 0.4, color=cols, label="|CTE| maks")
    ax[0].axhline(CTE_LIMIT_M, color="r", ls="--", label="batas 45 m")
    ax[0].set_ylabel("m")
    ax[0].legend(fontsize=8)
    ax[0].set_title("CTE per leg")
    ax[1].bar(xs, [L["overshoot_m"] for L in legs], color=cols)
    ax[1].set_title("Overshoot (puncak CTE di sisi seberang garis)")
    ax[1].set_ylabel("m")
    rec = [L["pemulihan_s"] if L["pemulihan_s"] != "-" else 0 for L in legs]
    ax[2].bar(xs, [L["durasi_s"] for L in legs], color="0.85", label="durasi leg")
    ax[2].bar(xs, rec, color=cols, label="waktu pulih |CTE|<=5 m (0 = tidak pulih)")
    ax[2].set_title("Waktu pemulihan vs durasi leg")
    ax[2].set_ylabel("s")
    ax[2].legend(fontsize=8)
    for x in ax:
        x.set_xticks(xs)
        x.set_xticklabels(labels, rotation=70, fontsize=7)
        x.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "07_metrik_per_leg.png"), dpi=120)
    plt.close(fig)

    # ---- 08 timeline seluruh log
    fig, ax = plt.subplots(3, 1, figsize=(16, 8), sharex=True)
    for x in ax:
        shade(x, iv, labels=(x is ax[0]))
        x.grid(alpha=0.3)
    ax[0].plot(ta, roll, "b", lw=0.6, label="roll")
    ax[0].legend(fontsize=8, loc="upper right")
    ax[1].plot(tv, alt_hud, "k", lw=0.8, label="alt")
    ax[1].legend(fontsize=8)
    ax[2].plot(tv, gs, "b", lw=0.8, label="groundspeed")
    ax[2].legend(fontsize=8)
    ax[2].set_xlabel("waktu log [s]")
    for d in D["STATUSTEXT"]:
        if d["_src"] == FC and ("REBOOT" in d["text"] or "failed" in d["text"]):
            for x in ax:
                x.axvline(d["_t"] - t0, color="r", lw=0.8)
    fig.suptitle("Timeline seluruh log (merah: REBOOT / init failed)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "08_timeline_seluruh_log.png"), dpi=120)
    plt.close(fig)

    # ---- 11 gaya jurnal: setpoint vs lintasan + koridor toleransi, per terbang
    deviasi = []
    rx, ry = WX[1:], WY[1:]          # jalur rencana WP1..WPn (x = timur, y = utara)
    cx, cy = corridor(rx, ry, CTE_LIMIT_M)
    for fi, (a, b) in enumerate(autos):
        S = flight_samples[fi]
        # Lintasan dipotong saat WP terakhir tercapai: sesudahnya fase akhir misi
        # (MIS_DONE), bukan bagian jalur rencana. Sampel CTE sudah berhenti di situ.
        t_end = float(S[-1, 0]) if len(S) else b
        m = (tp >= a) & (tp <= t_end)
        x, y = PX[m], PY[m]
        # Leg masuk->WP1 diterbangkan "langsung ke WP" (Navigation::updateWaypointNav,
        # crosstrack = false: prev_wp = posisi sekarang tiap siklus), jadi tidak ada
        # garis yang dilacak L1. Deviasi maks diambil dari leg WP1 dst. bila ada.
        seq_f = np.array([r[1] for r in rows if r[0] == fi + 1])
        cand = np.where(seq_f > 1)[0] if (seq_f > 1).any() else np.arange(len(seq_f))
        i_cte = int(cand[np.argmax(np.abs(S[cand, 3]))])
        wp_cte = int(seq_f[i_cte])
        # Leg pertama dimulai dari posisi masuk AUTO (Navigation::setNextWaypoint),
        # jadi titik itu ikut jalur acuan; tanpa ini fase masuk terhitung deviasi.
        dpath = dist_to_polyline(x, y, np.r_[x[0], rx], np.r_[y[0], ry])
        i_path = int(np.argmax(dpath))
        deviasi.append(dict(terbang=fi + 1, cte_maks=float(abs(S[i_cte, 3])), t_cte=float(S[i_cte, 0]),
                            wp_aktif=wp_cte,
                            jarak_jalur_maks=float(dpath[i_path]), t_jalur=float(tp[m][i_path]),
                            jarak_jalur_mean=float(dpath.mean()),
                            persen_luar_toleransi=100.0 * float(np.mean(dpath > CTE_LIMIT_M))))
        fig, ax = plt.subplots(figsize=(8.5, 8.2))
        ax.plot(cx, cy, ":", color="navy", lw=1.8, label="Toleransi ±%.0f m" % CTE_LIMIT_M)
        ax.plot(rx, ry, "-", color="red", lw=1.6, label="Setpoint (jalur rencana)")
        ax.plot(x, y, "-", color="black", lw=2.6, label="Lintasan UAV (AUTO)")
        for j in range(len(rx)):
            ax.annotate("WP%d" % (j + 1), (rx[j], ry[j]), fontsize=9, color="red",
                        xytext=(5, -12), textcoords="offset points")
        # penanda UAV: segitiga di titik masuk AUTO, menghadap arah terbang
        cog_a = float(np.interp(a, tg, cog))          # course GPS (derajat kompas)
        hdg = 90.0 - cog_a                            # -> sudut matematis (x timur, y utara)
        hr = math.radians(hdg)
        fwd, side = np.array([math.cos(hr), math.sin(hr)]), np.array([-math.sin(hr), math.cos(hr)])
        p0 = np.array([x[0], y[0]])
        tri = [p0 + fwd * 12, p0 - fwd * 7 + side * 8, p0 - fwd * 7 - side * 8]
        ax.add_patch(plt.Polygon(tri, closed=True, fc="darkorange", ec="k", zorder=6))
        ax.annotate("UAV", (x[0], y[0]), xytext=(12, 10), textcoords="offset points", fontsize=12)
        # titik deviasi terbesar: |CTE| maks terhadap leg aktif (metrik proposal)
        xp, yp = S[i_cte, 1], S[i_cte, 2]
        ax.plot(xp, yp, "o", ms=9, mfc="none", mec="m", mew=2, zorder=6)
        leg_name = ("WP%d-WP%d" % (wp_cte - 1, wp_cte)) if wp_cte > 1 else "masuk-WP%d" % wp_cte
        ax.annotate("deviasi maks %.1f m\n(%s)" % (abs(S[i_cte, 3]), leg_name), (xp, yp), xytext=(10, -26),
                    textcoords="offset points", color="m", fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlabel("Jarak sumbu-x / timur (m)")
        ax.set_ylabel("Jarak sumbu-y / utara (m)")
        ax.set_title("Terbang %d -- jalur rencana vs lintasan aktual (AUTO, t=%.0f-%.0f s)" % (fi + 1, a, t_end), fontsize=10)
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=3, fontsize=9, frameon=False)
        ax.tick_params(direction="in")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "11_jalur_rencana_vs_aktual_terbang%d.png" % (fi + 1)), dpi=150)
        plt.close(fig)
    with open(os.path.join(out, "deviasi_maksimum.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(deviasi[0].keys()))
        w.writeheader()
        w.writerows(deviasi)
    for dv in deviasi:
        print("deviasi:", {k: (round(v, 1) if isinstance(v, float) else v) for k, v in dv.items()})

    print("misi:", [(i, round(WX[i], 1), round(WY[i], 1), WZ[i]) for i in range(len(mission))])
    print("AUTO:", autos, "terbang:", flights)
    for s in summary:
        print(s)
    for L in legs:
        print(L)
    print("selesai ->", out)


if __name__ == "__main__":
    main()
