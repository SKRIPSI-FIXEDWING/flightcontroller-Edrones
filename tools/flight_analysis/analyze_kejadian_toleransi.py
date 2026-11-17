#!/usr/bin/env python3
"""Kejadian keluar toleransi di loiter fase stabil (per kejadian, tanpa rata-rata/RMS) dari log SD.

Arahan pembimbing (komentar draft 14 Nov 2026): galat loiter tidak dilaporkan sebagai rata-rata,
RMS, atau % waktu, tetapi per kejadian: setiap kali |galat radius| > toleransi dicek satu per satu
(jenis, puncak, durasi, sisi orbit), beserta penyebab (pulsa gangguan, saturasi roll, kecepatan
tanah searah angin) dan apa yang dilakukan L1/fuzzy saat itu (period, dT).

Definisi
  galat e      = wp_dist_m - R (bertanda): + = di luar orbit (radius melebar), - = di dalam orbit
  fase stabil  = loiter mulai putaran ke-2 (loiter_turn_index >= 1), per segmen AUTO
  kejadian     = rentang berurutan |e| > tol (default 10 m); dua rentang yang berjarak < --gabung s
                 (default 1 s) digabung sebagai satu kejadian
  jenis        = overshoot  bila puncak di luar orbit (e > +tol, radius melebar)
                 undershoot bila puncak di dalam orbit (e < -tol, memotong ke dalam)
  durasi       = waktu sejak |e| pertama > tol sampai kembali <= tol
  sisi orbit   = arah wahana dilihat dari pusat orbit saat puncak (U, TL, T, ... ; 0 deg = utara)
  angin        = fit lingkaran vektor kecepatan tanah di fase stabil (|v_tanah - w| ~ konstan);
                 arah terbang saat puncak: searah angin (komponen angin sepanjang course > +50% |w|),
                 lawan angin (< -50%), atau menyamping
  nomor pulsa  = P1 = pulsa pertama di fase stabil; pulsa di 15 s sebelumnya diberi P0, P-1, ...
  pulsa        = DIST_ON terakhir yang dimulai antara 15 s sebelum kejadian mulai dan saat puncak (bila tidak ada:
                 pulsa pertama yang mulai sesudah puncak tetapi masih di dalam kejadian, t_sejak_pulsa negatif);
                 kosong = tidak ada pulsa, kejadian bukan akibat gangguan.
                 pulsa_sesudah_mulai = ya: pulsa baru diberikan saat galat SUDAH di luar toleransi, jadi pulsa
                 bukan penyebab awal kejadian (tidak dihitung "terkait pulsa" di rekap)
  roll jenuh   = |nav_roll| >= --bank - 0.5 deg; dilaporkan % waktu jenuh selama kejadian dan ya/tidak saat puncak
  period, dT   = l1_period_s saat puncak dan min-maks selama kejadian; dT = period - period dasar
  awal_stabil  = kejadian sudah berlangsung saat fase stabil dimulai (sisa dari putaran masuk)

Argumen: satu atau lebih SPEC "LABEL|LOG.CSV|R[|t0|t1[|segmen[|s0-s1]]]" (t0/t1 = jendela waktu log [s], boleh kosong).
Satu log bisa berisi beberapa segmen AUTO (pilot keluar-masuk AUTO); tiap segmen diberi akhiran a, b, c.
"segmen" = huruf segmen yang dipakai (mis. "b"); kosong = semua.
"s0-s1" = potong fase stabil ke t_boot [s0, s1] (salah satu boleh kosong), dihitung SESUDAH putaran ditentukan
dari seluruh loiter. Dipakai untuk memisahkan putaran tanpa gangguan (sebelum pulsa pertama) dari bagian bergangguan
dalam satu segmen AUTO yang sama.

Keluaran (folder -o):
  kejadian_toleransi.csv            satu baris per kejadian
  rekap_per_penerbangan.csv         satu baris per segmen: jumlah overshoot/undershoot, puncak, durasi,
                                    berapa yang terkait pulsa / roll jenuh / searah angin
  rekap_kejadian.md                 tabel yang sama dalam markdown (bahan Bab 5)
  <LABEL>_galat_waktu.png           galat vs waktu (setpoint merah, +/-tol putus-putus, kejadian diarsir
                                    dan dinomori, pulsa), nav_roll vs batas, period dan GS
  <LABEL>_orbit.png                 jejak orbit fase stabil: segmen kejadian diwarnai jenisnya, nomor di puncak,
                                    lokasi pulsa, panah angin

Pemakaian:
    python analyze_kejadian_toleransi.py "T3|2026-11-16_10-50-34.CSV|100|49|532.3" -o kejadian_2026-11-16
"""
import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_fuzzy_sdlog import loiter_turn_index
from sdlog_io import read_sdlog

SECTORS = ("U", "TL", "T", "Tg", "S", "BD", "B", "BL")  # utara, timur laut, timur, tenggara, ...
PULSE_WINDOW_S = 15.0
JENIS_COLOR = {"overshoot": "tab:red", "undershoot": "tab:blue"}


def runs(mask):
    out, s = [], None
    for i, f in enumerate(mask):
        if f and s is None:
            s = i
        if not f and s is not None:
            out.append((s, i))
            s = None
    if s is not None:
        out.append((s, len(mask)))
    return out


def local_xy(lat_e7, lon_e7, lat0_e7, lon0_e7):
    lat_e7, lon_e7 = np.asarray(lat_e7, float), np.asarray(lon_e7, float)
    y = (lat_e7 - lat0_e7) * 1e-7 * 111320.0
    x = (lon_e7 - lon0_e7) * 1e-7 * 111320.0 * math.cos(math.radians(lat0_e7 * 1e-7))
    return x, y


def sector(bearing_deg):
    return SECTORS[int(((bearing_deg % 360) + 22.5) // 45) % 8]


def fit_wind(gs, course_deg):
    """Angin (w_timur, w_utara) dari |v_tanah - w| = konstan (least squares)."""
    ve = gs * np.sin(np.radians(course_deg))
    vn = gs * np.cos(np.radians(course_deg))
    A = np.c_[2 * ve, 2 * vn, np.ones_like(ve)]
    we, wn, k = np.linalg.lstsq(A, ve ** 2 + vn ** 2, rcond=None)[0]
    return float(we), float(wn), float(math.sqrt(max(k + we ** 2 + wn ** 2, 0.0)))


def find_events(ts, e, tol, merge_s):
    out = runs(np.abs(e) > tol)
    merged = []
    for a, b in out:
        if merged and ts[a] - ts[merged[-1][1] - 1] < merge_s:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    return merged


def analyse_segment(label, c, t, st, idx_loit, R, tol, bank, merge_s, base_period, center, pulses):
    """st = indeks baris fase stabil (berurutan), idx_loit = seluruh loiter segmen (untuk awal_stabil)."""
    ts = t[st]
    e = c["wp_dist_m"][st] - R
    cx, cy = center
    X, Y = local_xy(c["lat_e7"][st], c["lon_e7"][st], cx, cy)
    gs, crs = c["gs_mps"][st], c["course_deg"][st]
    we, wn, tas = fit_wind(gs, crs)
    wmag = math.hypot(we, wn)
    nav_roll = c["nav_roll_deg"][st]
    per = c["l1_period_s"][st] if "l1_period_s" in c else np.full(len(st), base_period)
    per = np.where(np.isfinite(per), per, base_period)
    sat = np.abs(nav_roll) >= bank - 0.5
    # e sesaat sebelum fase stabil (untuk menandai kejadian sisa putaran masuk)
    pre_i = idx_loit[idx_loit < st[0]]
    e_before = float(c["wp_dist_m"][pre_i[-1]] - R) if len(pre_i) else 0.0
    seg_pulses = [p for p in pulses if ts[0] - PULSE_WINDOW_S <= p[0] <= ts[-1]]
    n_pre = sum(p[0] < ts[0] for p in seg_pulses)  # P1 = pulsa pertama di fase stabil; sebelumnya P0, P-1, ...
    events = []
    for k, (a, b) in enumerate(find_events(ts, e, tol, merge_s)):
        sl = slice(a, b)
        ipk = a + int(np.argmax(np.abs(e[sl])))
        pk = float(e[ipk])
        jenis = "overshoot" if pk > 0 else "undershoot"
        bearing = math.degrees(math.atan2(X[ipk], Y[ipk])) % 360
        c_rad = math.radians(crs[ipk])
        along = we * math.sin(c_rad) + wn * math.cos(c_rad)
        arah_angin = ("searah angin" if along > 0.5 * wmag else "lawan angin" if along < -0.5 * wmag else "menyamping")
        t0, t1 = float(ts[a]), float(ts[b - 1])
        tpk = float(ts[ipk])
        cand = [(i, p) for i, p in enumerate(seg_pulses) if t0 - PULSE_WINDOW_S <= p[0] <= tpk]
        if not cand:  # pulsa mulai sesudah puncak tetapi masih di dalam kejadian
            cand = [(i, p) for i, p in enumerate(seg_pulses) if tpk < p[0] <= t1][:1]
        pulsa = cand[-1] if cand else None
        events.append(dict(
            segmen=label, no=k + 1, jenis=jenis, arah="luar" if pk > 0 else "dalam",
            t_mulai_s=round(t0, 1), t_selesai_s=round(t1, 1),
            durasi_s=round(float(ts[min(b, len(ts) - 1)] - ts[a]) if b < len(ts) else t1 - t0, 1),
            berakhir_di_luar="ya" if b >= len(ts) else "",
            puncak_m=round(pk, 1), t_puncak_s=round(float(ts[ipk]), 1),
            awal_stabil="ya" if a == 0 and abs(e_before) > tol else "",
            sudut_dari_pusat_deg=round(bearing), sisi_orbit=sector(bearing),
            gs_puncak_mps=round(float(gs[ipk]), 1), gs_maks_mps=round(float(gs[sl].max()), 1),
            arah_terbang=arah_angin,
            nav_roll_puncak_deg=round(float(nav_roll[ipk]), 1), roll_puncak_deg=round(float(c["roll_deg"][st][ipk]), 1),
            roll_jenuh_saat_puncak="ya" if sat[ipk] else "tidak",
            roll_jenuh_pct=round(100 * float(sat[sl].mean())),
            period_puncak_s=round(float(per[ipk]), 2), dT_puncak_s=round(float(per[ipk] - base_period), 2),
            period_min_s=round(float(per[sl].min()), 2), period_maks_s=round(float(per[sl].max()), 2),
            pulsa_no=(pulsa[0] + 1 - n_pre) if pulsa else "", pulsa_us=pulsa[1][1] if pulsa else "",
            t_sejak_pulsa_s=round(float(ts[ipk]) - pulsa[1][0], 1) if pulsa else "",
            pulsa_sesudah_mulai="ya" if pulsa and pulsa[1][0] > t0 else "",
            x_timur_m=round(float(X[ipk]), 1), y_utara_m=round(float(Y[ipk]), 1),
        ))
    info = dict(ts=ts, e=e, X=X, Y=Y, nav_roll=nav_roll, roll=c["roll_deg"][st], per=per, gs=gs, sat=sat,
                wind=(we, wn), tas=tas, pulses=seg_pulses, n_pre=n_pre, ev_slices=find_events(ts, e, tol, merge_s))
    return events, info


def plot_segment(out, label, info, events, R, tol, bank, base_period, metode):
    ts, e = info["ts"], info["e"]
    fig, ax = plt.subplots(3, 1, figsize=(15, 9.5), sharex=True, gridspec_kw=dict(height_ratios=[2.2, 1.2, 1]))
    t0 = ts[0]
    tr = ts - t0
    a = ax[0]
    a.plot(tr, e, color="k", lw=1.1, label="galat radius e")
    a.axhline(0, color="red", lw=1.4, label="setpoint (R = %.0f m)" % R)
    for s in (tol, -tol):
        a.axhline(s, color="0.3", ls="--", lw=1)
    a.plot([], [], color="0.3", ls="--", label="toleransi ±%.0f m" % tol)
    for (i0, i1), ev in zip(info["ev_slices"], events):
        col = JENIS_COLOR[ev["jenis"]]
        for aa in ax:
            aa.axvspan(tr[i0], tr[min(i1, len(tr) - 1)], color=col, alpha=0.15, lw=0)
        a.annotate("%d" % ev["no"], (ev["t_puncak_s"] - t0, ev["puncak_m"]), xytext=(0, 6 if ev["puncak_m"] > 0 else -14),
                   textcoords="offset points", ha="center", fontsize=9, color=col, weight="bold")
    for i, (tp, us, dur) in enumerate(info["pulses"]):
        if tp < ts[0]:
            continue
        i -= info["n_pre"]
        for aa in ax:
            aa.axvspan(tp - t0, tp - t0 + dur, color="orange", alpha=0.35, lw=0)
        a.annotate("P%d" % (i + 1), (tp - t0, 1.0), xycoords=("data", "axes fraction"), xytext=(1, -10),
                   textcoords="offset points", fontsize=8, color="darkorange")
    a.fill_between([], [], color=JENIS_COLOR["overshoot"], alpha=0.15, label="overshoot (keluar, radius melebar)")
    a.fill_between([], [], color=JENIS_COLOR["undershoot"], alpha=0.15, label="undershoot (masuk ke dalam orbit)")
    if info["pulses"]:
        a.fill_between([], [], color="orange", alpha=0.35, label="pulsa gangguan")
    lo, hi = min(float(e.min()), -tol), max(float(e.max()), tol)
    a.set_ylim(lo - 4, hi + 4)
    a.set_ylabel("galat radius [m]\n(+ luar, − dalam)")
    a.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    a.grid(alpha=0.3)
    n_o = sum(ev["jenis"] == "overshoot" for ev in events)
    a.set_title("%s — %s, loiter fase stabil %.0f s, NAVL1_PERIOD %.0f s: %s" % (
        label, metode, ts[-1] - ts[0], base_period,
        "tidak ada kejadian keluar ±%.0f m" % tol if not events else
        "%d kejadian keluar ±%.0f m (%d overshoot, %d undershoot)" % (len(events), tol, n_o, len(events) - n_o)),
        fontsize=10)
    a = ax[1]
    a.plot(tr, info["nav_roll"], color="tab:purple", lw=1, label="perintah roll (nav_roll)")
    a.plot(tr, info["roll"], color="0.5", lw=0.8, label="roll")
    for s in (bank, -bank):
        a.axhline(s, color="0.3", ls="--", lw=1)
    a.plot([], [], color="0.3", ls="--", label="batas bank ±%.0f°" % bank)
    a.set_ylabel("roll [deg]")
    a.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    a.grid(alpha=0.3)
    a = ax[2]
    a.plot(tr, info["per"], color="tab:green", lw=1.2, label="period L1")
    a.axhline(base_period, color="red", lw=1, label="NAVL1_PERIOD %.0f s" % base_period)
    a.set_ylabel("period [s]")
    a.set_ylim(min(11.5, info["per"].min() - 0.5), max(16.5, info["per"].max() + 0.5))
    b = a.twinx()
    b.plot(tr, info["gs"], color="tab:brown", lw=0.8, alpha=0.7, label="kecepatan tanah")
    b.set_ylabel("GS [m/s]", color="tab:brown")
    h1, l1 = a.get_legend_handles_labels()
    h2, l2 = b.get_legend_handles_labels()
    a.legend(h1 + h2, l1 + l2, loc="upper left", bbox_to_anchor=(1.06, 1.0), fontsize=8)
    a.grid(alpha=0.3)
    a.set_xlim(-1, tr[-1] + 1)
    a.set_xlabel("waktu sejak awal fase stabil [s] (waktu log %.1f s)" % t0)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "%s_galat_waktu.png" % label), dpi=130)
    plt.close(fig)

    fig, a = plt.subplots(figsize=(9, 9))
    th = np.linspace(0, 2 * np.pi, 361)
    a.plot(R * np.sin(th), R * np.cos(th), color="red", lw=1.4, label="orbit perintah R = %.0f m" % R)
    for r in (R - tol, R + tol):
        a.plot(r * np.sin(th), r * np.cos(th), color="0.3", ls="--", lw=1)
    a.plot([], [], color="0.3", ls="--", label="toleransi ±%.0f m" % tol)
    a.plot(info["X"], info["Y"], color="0.55", lw=1, label="jejak fase stabil")
    for (i0, i1), ev in zip(info["ev_slices"], events):
        col = JENIS_COLOR[ev["jenis"]]
        a.plot(info["X"][i0:i1], info["Y"][i0:i1], color=col, lw=2.6)
        a.annotate("%d" % ev["no"], (ev["x_timur_m"], ev["y_utara_m"]), xytext=(6, 6), textcoords="offset points",
                   color=col, weight="bold")
    a.plot([], [], color=JENIS_COLOR["overshoot"], lw=2.6, label="overshoot")
    a.plot([], [], color=JENIS_COLOR["undershoot"], lw=2.6, label="undershoot")
    for i, (tp, us, dur) in enumerate(info["pulses"]):
        i -= info["n_pre"]
        k = int(np.searchsorted(ts, tp))
        if tp >= ts[0] and k < len(ts):
            a.plot(info["X"][k], info["Y"][k], "o", color="orange", ms=8, mec="k")
            a.annotate("P%d" % (i + 1), (info["X"][k], info["Y"][k]), xytext=(-14, -12), textcoords="offset points",
                       fontsize=8, color="darkorange")
    if info["pulses"]:
        a.plot([], [], "o", color="orange", mec="k", label="awal pulsa gangguan")
    we, wn = info["wind"]
    wm = math.hypot(we, wn)
    if wm > 0.3:
        s = 0.55 * R / max(wm, 1.0)
        a.annotate("", xy=(we * s * 0.5, wn * s * 0.5), xytext=(-we * s * 0.5, -wn * s * 0.5),
                   arrowprops=dict(arrowstyle="->", lw=2, color="tab:cyan"))
        a.text(0, -0.12 * R, "angin %.1f m/s dari %.0f°" % (wm, (math.degrees(math.atan2(we, wn)) + 180) % 360),
               ha="center", color="tab:cyan", fontsize=9)
    a.set_aspect("equal")
    a.set_xlabel("timur dari pusat orbit [m]")
    a.set_ylabel("utara dari pusat orbit [m]")
    a.grid(alpha=0.3)
    a.legend(loc="upper right", fontsize=8)
    a.set_title("%s — %s: jejak loiter fase stabil dan kejadian keluar toleransi" % (label, metode), fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "%s_orbit.png" % label), dpi=130)
    plt.close(fig)


def write_csv(path, rows, fields=None):
    if not rows and not fields:
        open(path, "w").close()
        return
    fields = fields or list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("spec", nargs="+", help='"LABEL|LOG.CSV|R[|t0|t1]"')
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--tol", type=float, default=10.0, help="toleransi galat radius [m]")
    ap.add_argument("--bank", type=float, default=45.0, help="LOIT_BANK_DEG [deg] untuk deteksi roll jenuh")
    ap.add_argument("--gabung", type=float, default=1.0, help="gabungkan kejadian yang berjarak < s [s]")
    ap.add_argument("--min-stabil", type=float, default=10.0, help="abaikan segmen dengan fase stabil < s [s]")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    all_ev, rekap = [], []
    for spec in args.spec:
        p = spec.split("|")
        label, path, R = p[0], p[1], float(p[2])
        win = (float(p[3]) if len(p) >= 5 and p[3] else -np.inf, float(p[4]) if len(p) >= 5 and p[4] else np.inf)
        keep = p[5] if len(p) >= 6 else ""  # huruf segmen yang dipakai, mis. "b"; kosong = semua
        clip = (-np.inf, np.inf)
        if len(p) >= 7 and p[6]:
            a_, b_ = p[6].split("-")
            clip = (float(a_) if a_ else -np.inf, float(b_) if b_ else np.inf)
        log = read_sdlog(path)
        c = log.col
        t = c["time_boot_ms"] / 1000.0
        snaps = [s for s in log.snapshots if s.waypoints]
        loiter_wp = max(w["seq"] for w in snaps[-1].waypoints) if snaps else int(np.nanmax(c["wp_next"]))
        cwp = next((w for w in snaps[-1].waypoints if w["seq"] == loiter_wp), None) if snaps else None
        pulses = []
        for ev in log.events_of("DIST_ON"):
            off = next((x.t_s for x in log.events_of("DIST_OFF") if x.t_s > ev.t_s), ev.t_s + 2.0)
            pulses.append((ev.t_s, ev.args[0] if ev.args else "", off - ev.t_s))
        auto = np.array([m == "AUTO" for m in c["mode"]]) & (c["armed"] > 0) & (t >= win[0]) & (t <= win[1])
        segs = []
        for a, b in runs(auto):
            idx = np.arange(a, b)
            idx = idx[c["wp_next"][idx] == loiter_wp]
            if len(idx) < 100:
                continue
            turn = loiter_turn_index(c["lat_e7"][idx], c["lon_e7"][idx])
            st = idx[turn >= 1]
            st = st[: np.flatnonzero(np.diff(st) > 1)[0] + 1] if np.any(np.diff(st) > 1) else st
            if len(st) < 2 or t[st[-1]] - t[st[0]] < args.min_stabil:
                print("%s: segmen AUTO t_boot %.0f-%.0f s dilewati (fase stabil %.0f s)"
                      % (label, t[a], t[b - 1], (t[st[-1]] - t[st[0]]) if len(st) > 1 else 0))
                continue
            segs.append((idx, st))
        for j, (idx, st) in enumerate(segs):
            sl = label + ("abcdefgh"[j] if len(segs) > 1 else "")
            if keep and "abcdefgh"[j] not in keep:
                print("%s: dilewati (tidak dipilih)" % sl)
                continue
            st = st[(t[st] >= clip[0]) & (t[st] <= clip[1])]
            if len(st) < 2:
                print("%s: fase stabil kosong sesudah dipotong %s" % (sl, p[6]))
                continue
            eff = log.effective_at(float(c["time_boot_ms"][st[0]]))
            base = eff.get("fuzzy_base_period", eff.get("l1_period_now", 14.0))
            fz = bool(np.nanmax(c["fuzzy_on"][st]) > 0) if "fuzzy_on" in c else eff.get("fuzzy_enable", 0.0) > 0
            pid = eff.get("nav_pid_enable", 0.0) > 0
            metode = "PID" if pid else ("L1 + fuzzy" if fz else "L1 tetap")
            center = (cwp["lat_e7"], cwp["lon_e7"]) if cwp else (float(c["lat_e7"][st].mean()), float(c["lon_e7"][st].mean()))
            events, info = analyse_segment(sl, c, t, st, idx, R, args.tol, args.bank, args.gabung, base, center, pulses)
            for ev in events:
                ev["metode"] = metode
            plot_segment(args.out, sl, info, events, R, args.tol, args.bank, base, metode)
            all_ev += events
            ts = info["ts"]
            ang = np.unwrap(np.arctan2(info["Y"], info["X"]))
            we, wn = info["wind"]
            ov = [ev for ev in events if ev["jenis"] == "overshoot"]
            un = [ev for ev in events if ev["jenis"] == "undershoot"]
            rekap.append(dict(
                segmen=sl, log=os.path.basename(path), metode=metode, R_m=R,
                t_boot_stabil=f"{ts[0]:.1f}-{ts[-1]:.1f}", stabil_s=round(float(ts[-1] - ts[0])),
                putaran_stabil=round(abs(ang[-1] - ang[0]) / (2 * np.pi), 1),
                angin_mps=round(math.hypot(we, wn), 1),
                angin_dari_deg=round((math.degrees(math.atan2(we, wn)) + 180) % 360),
                pulsa_di_stabil=sum(ts[0] <= p[0] <= ts[-1] for p in info["pulses"]),
                kejadian=len(events), overshoot=len(ov), undershoot=len(un),
                puncak_luar_m=max((ev["puncak_m"] for ev in ov), default=""),
                puncak_dalam_m=min((ev["puncak_m"] for ev in un), default=""),
                durasi_terlama_s=max((ev["durasi_s"] for ev in events), default=""),
                terkait_pulsa=sum(ev["pulsa_no"] != "" and not ev["pulsa_sesudah_mulai"] for ev in events),
                roll_jenuh_saat_puncak=sum(ev["roll_jenuh_saat_puncak"] == "ya" for ev in events),
                roll_jenuh_selama=sum(ev["roll_jenuh_pct"] > 0 for ev in events),
                searah_angin=sum(ev["arah_terbang"] == "searah angin" for ev in events),
                sisa_putaran_masuk=sum(ev["awal_stabil"] == "ya" for ev in events),
            ))
            print("%-6s %-11s stabil %3.0f s  %d kejadian (%d over, %d under)" % (
                sl, metode, ts[-1] - ts[0], len(events), len(ov), len(un)))

    ev_fields = ["segmen", "metode", "no", "jenis", "arah", "t_mulai_s", "t_selesai_s", "durasi_s", "berakhir_di_luar",
                 "puncak_m", "t_puncak_s", "awal_stabil", "sudut_dari_pusat_deg", "sisi_orbit", "gs_puncak_mps",
                 "gs_maks_mps", "arah_terbang", "nav_roll_puncak_deg", "roll_puncak_deg", "roll_jenuh_saat_puncak",
                 "roll_jenuh_pct", "period_puncak_s", "dT_puncak_s", "period_min_s", "period_maks_s", "pulsa_no",
                 "pulsa_us", "t_sejak_pulsa_s", "pulsa_sesudah_mulai", "x_timur_m", "y_utara_m"]
    write_csv(os.path.join(args.out, "kejadian_toleransi.csv"), all_ev, ev_fields)
    write_csv(os.path.join(args.out, "rekap_per_penerbangan.csv"), rekap)

    with open(os.path.join(args.out, "rekap_kejadian.md"), "w") as f:
        f.write("# Kejadian keluar toleransi ±%.0f m, loiter fase stabil\n\n" % args.tol)
        f.write("Dibuat oleh `analyze_kejadian_toleransi.py` (bank %.0f°, kejadian berjarak < %.1f s digabung). "
                "overshoot = keluar orbit (radius melebar), undershoot = masuk ke dalam orbit.\n\n" % (args.bank, args.gabung))
        f.write("## Per penerbangan\n\n| Segmen | Metode | Stabil | Putaran | Angin | Pulsa | Kejadian | Overshoot | "
                "Undershoot | Puncak luar | Puncak dalam | Terlama | Terkait pulsa | Roll jenuh (puncak / selama) | Searah angin |\n"
                "|" + "---|" * 15 + "\n")
        for r in rekap:
            f.write("| %s | %s | %s s | %s | %s m/s dari %s° | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |\n" % (
                r["segmen"], r["metode"], r["stabil_s"], r["putaran_stabil"], r["angin_mps"], r["angin_dari_deg"],
                r["pulsa_di_stabil"], r["kejadian"] if r["kejadian"] else "**tidak ada**", r["overshoot"],
                r["undershoot"], r["puncak_luar_m"] or "–", r["puncak_dalam_m"] or "–",
                (str(r["durasi_terlama_s"]) + " s") if r["durasi_terlama_s"] != "" else "–", r["terkait_pulsa"],
                "%d / %d" % (r["roll_jenuh_saat_puncak"], r["roll_jenuh_selama"]), r["searah_angin"]))
        f.write("\n## Per kejadian\n\n| Segmen | No | Jenis | Puncak | Durasi | Sisi orbit | Arah terbang | GS | "
                "nav_roll | Jenuh | Period (dT) | Pulsa | Catatan |\n" + "|" + "---|" * 13 + "\n")
        for ev in all_ev:
            cat = []
            if ev["awal_stabil"]:
                cat.append("sisa putaran masuk")
            if ev["pulsa_sesudah_mulai"]:
                cat.append("pulsa diberikan saat sudah di luar toleransi")
            if ev["berakhir_di_luar"]:
                cat.append("masih di luar saat fase stabil berakhir")
            f.write("| %s | %d | %s | %+.1f m | %.1f s | %s (%d°) | %s | %.1f m/s | %+.1f° | %s (%d%%) | %.1f s (%+.1f) | %s | %s |\n" % (
                ev["segmen"], ev["no"], ev["jenis"], ev["puncak_m"], ev["durasi_s"], ev["sisi_orbit"],
                ev["sudut_dari_pusat_deg"], ev["arah_terbang"], ev["gs_puncak_mps"], ev["nav_roll_puncak_deg"],
                ev["roll_jenuh_saat_puncak"], ev["roll_jenuh_pct"], ev["period_puncak_s"], ev["dT_puncak_s"],
                ("P%s (%s µs, puncak %+.1f s)" % (ev["pulsa_no"], ev["pulsa_us"], ev["t_sejak_pulsa_s"]))
                if ev["pulsa_no"] != "" else "–", ", ".join(cat)))


if __name__ == "__main__":
    main()
