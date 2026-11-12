#!/usr/bin/env python3
"""Analisis satu tlog Mission Planner: roll/pitch FBWA, kegagalan AUTO, dan plot pendukung.

Dipakai untuk uji terbang 2026-10-13 (FBWA Khageswara + AUTO). Membaca .tlog
langsung via pymavlink (tanpa CSV), lalu menulis PNG ke folder keluaran.

Run:
    python analyze_tlog_fbwa_auto.py "path/ke/2026-10-13 15-28-12.tlog" [-o out_dir]

Mode diambil dari HEARTBEAT kendaraan (type==1): custom_mode 0=MANUAL, 5=FBWA,
10=AUTO, 4=GUIDED; armed = bit 128 base_mode.
"""

import argparse
import collections
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

R2D = 180.0 / np.pi
MODE_NAME = {0: "MANUAL", 5: "FBWA", 10: "AUTO", 4: "GUIDED"}
MODE_COLOR = {"FBWA": "#cfe8ff", "AUTO": "#ffd9b3", "MANUAL": "#e6e6e6", "GUIDED": "#e6d4ff"}
ROLL_GAIN_UST_PER_DEG = 18.0  # hasil identifikasi dari log (lihat laporan)


def load(path):
    conn = mavutil.mavlink_connection(path, dialect="ardupilotmega")
    D = collections.defaultdict(list)
    while True:
        m = conn.recv_match()
        if m is None:
            break
        d = m.to_dict()
        d["_t"] = m._timestamp
        D[m.get_type()].append(d)
    return D


def series(D, typ, key, t0, scale=1.0):
    t = np.array([d["_t"] - t0 for d in D[typ]])
    v = np.array([d[key] * scale for d in D[typ]], dtype=float)
    return t, v


def arduino_map(x, a, b, c, d):
    return np.trunc((x - a) * (d - c) / (b - a)) + c


def mode_intervals(D, t0, t_end):
    """[(t_start, t_end, mode_name, armed)] dari HEARTBEAT type==1."""
    ev = []
    last = None
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


def shade(ax, intervals, labels=False):
    seen = set()
    for a, b, name, armed in intervals:
        if not armed:
            continue
        lab = name if (labels and name not in seen) else None
        seen.add(name)
        ax.axvspan(a, b, color=MODE_COLOR.get(name, "#eee"), alpha=0.7, lw=0, label=lab)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tlog")
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_2026-10-13")
    os.makedirs(out, exist_ok=True)

    D = load(args.tlog)
    t0 = D["ATTITUDE"][0]["_t"]
    ta, roll = series(D, "ATTITUDE", "roll", t0, R2D)
    _, pitch = series(D, "ATTITUDE", "pitch", t0, R2D)
    _, yaw = series(D, "ATTITUDE", "yaw", t0, R2D)
    t_end = ta[-1]
    iv = mode_intervals(D, t0, t_end)
    airborne = [(a, b, n) for a, b, n, arm in iv if arm]
    t_arm = airborne[0][0]
    t_dis = airborne[-1][1]
    auto_iv = [(a, b) for a, b, n in airborne if n == "AUTO"]

    ts, ail = series(D, "SERVO_OUTPUT_RAW", "servo1_raw", t0)
    _, ele = series(D, "SERVO_OUTPUT_RAW", "servo2_raw", t0)
    _, thr_s = series(D, "SERVO_OUTPUT_RAW", "servo3_raw", t0)
    tr, ch1 = series(D, "RC_CHANNELS_RAW", "chan1_raw", t0)
    _, ch2 = series(D, "RC_CHANNELS_RAW", "chan2_raw", t0)
    _, ch3 = series(D, "RC_CHANNELS_RAW", "chan3_raw", t0)
    tn, nav_roll = series(D, "NAV_CONTROLLER_OUTPUT", "nav_roll", t0)
    _, nav_pitch = series(D, "NAV_CONTROLLER_OUTPUT", "nav_pitch", t0)
    _, wp_dist = series(D, "NAV_CONTROLLER_OUTPUT", "wp_dist", t0)
    _, xtrack = series(D, "NAV_CONTROLLER_OUTPUT", "xtrack_error", t0)
    _, alt_err = series(D, "NAV_CONTROLLER_OUTPUT", "alt_error", t0)
    _, aspd_err = series(D, "NAV_CONTROLLER_OUTPUT", "aspd_error", t0)
    tv, airspeed = series(D, "VFR_HUD", "airspeed", t0)
    _, gs = series(D, "VFR_HUD", "groundspeed", t0)
    _, thr = series(D, "VFR_HUD", "throttle", t0)
    _, alt = series(D, "VFR_HUD", "alt", t0)
    _, climb = series(D, "VFR_HUD", "climb", t0)
    tp, lat = series(D, "GLOBAL_POSITION_INT", "lat", t0, 1e-7)
    _, lon = series(D, "GLOBAL_POSITION_INT", "lon", t0, 1e-7)

    # perintah roll/pitch dari stick (rumus FBWA Khageswara, tanpa tanda aileron)
    roll_cmd = 1.3 * arduino_map(ch1 - 1500, -512, 512, -35, 35)
    pitch_cmd = 1.1 * arduino_map(ch2 - 1500, -512, 512, -25, 35)

    named = {}
    for n in ("MHN_ROLL", "MHN_PITCH", "MHN_YAW"):
        L = [d for d in D["NAMED_VALUE_FLOAT"] if d["name"] == n]
        named[n] = (np.array([d["_t"] - t0 for d in L]), np.array([d["value"] for d in L]))

    # ---------------------------------------------------------------- Fig 1
    fig, ax = plt.subplots(4, 1, figsize=(15, 11), sharex=True)
    for a in ax:
        shade(a, iv, labels=(a is ax[0]))
    ax[0].plot(ta, roll, "b", lw=1.1, label="roll (ATTITUDE)")
    ax[0].plot(tr, roll_cmd, "r--", lw=0.9, label="cmd stick 1.3*map(±35)")
    ax[0].axhline(40, color="gray", ls=":", lw=0.8)
    ax[0].axhline(-40, color="gray", ls=":", lw=0.8, label="ambang ALERT ±40°")
    ax[0].set_ylabel("roll [deg]")
    ax[0].legend(ncol=5, fontsize=8, loc="upper right")
    ax[0].set_title("Roll, pitch, dan keluaran servo selama penerbangan (latar: mode)")
    ax[1].plot(ta, pitch, "g", lw=1.1, label="pitch (ATTITUDE)")
    ax[1].plot(tr, pitch_cmd, "r--", lw=0.9, label="cmd stick 1.1*map(-25..35)")
    ax[1].set_ylabel("pitch [deg]")
    ax[1].legend(ncol=2, fontsize=8, loc="upper right")
    ax[2].plot(ts, ail, "b", lw=0.9, label="aileron PWM")
    ax[2].plot(tr, ch1, "k", lw=0.6, alpha=0.5, label="stick roll (ch1)")
    ax[2].axhline(1099, color="gray", ls=":", lw=0.8)
    ax[2].axhline(1900, color="gray", ls=":", lw=0.8)
    ax[2].set_ylabel("PWM [us]")
    ax[2].legend(ncol=2, fontsize=8, loc="upper right")
    ax[3].plot(ts, ele, "g", lw=0.9, label="elevator PWM")
    ax[3].plot(tr, ch2, "k", lw=0.6, alpha=0.5, label="stick pitch (ch2)")
    ax[3].set_ylabel("PWM [us]")
    ax[3].set_xlabel("waktu sejak log mulai [s]")
    ax[3].legend(ncol=2, fontsize=8, loc="upper right")
    ax[3].set_xlim(0, 140)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "01_roll_pitch_servo_semua_mode.png"), dpi=130)
    plt.close(fig)

    # ---------------------------------------------------------------- Fig 2 : FBWA saja
    fbwa_iv = [(a, b) for a, b, n in airborne if n == "FBWA"]
    fig, ax = plt.subplots(2, 1, figsize=(15, 7), sharex=True)
    for k, (sig, cmd, name, col) in enumerate(((roll, roll_cmd, "roll", "b"), (pitch, pitch_cmd, "pitch", "g"))):
        for a, b in fbwa_iv:
            m = (ta >= a) & (ta < b)
            ax[k].plot(ta[m], sig[m], col, lw=1.2)
            mc = (tr >= a) & (tr < b)
            ax[k].plot(tr[mc], cmd[mc], "r--", lw=0.9)
        for a, b in auto_iv:
            ax[k].axvspan(a, b, color=MODE_COLOR["AUTO"], alpha=0.8, lw=0)
        ax[k].set_ylabel("%s [deg]" % name)
        ax[k].grid(alpha=0.3)
    ax[0].set_title("Mode FBWA saja: respons (biru/hijau) vs perintah stick (merah putus-putus); oranye = AUTO (dipotong)")
    ax[1].set_xlabel("waktu [s]")
    ax[1].set_xlim(t_arm, t_dis)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "02_fbwa_roll_pitch_vs_stick.png"), dpi=130)
    plt.close(fig)

    # ---------------------------------------------------------------- Fig 3 : pelacakan FBWA (scatter)
    m_fb = np.zeros_like(ta, bool)
    for a, b in fbwa_iv:
        m_fb |= (ta >= a + 0.5) & (ta < b)
    rc_i = np.interp(ta, tr, roll_cmd)
    pc_i = np.interp(ta, tr, pitch_cmd)
    fig, ax = plt.subplots(1, 3, figsize=(16, 5))
    ax[0].scatter(rc_i[m_fb], roll[m_fb], s=6, alpha=0.5)
    lim = [-55, 55]
    ax[0].plot(lim, lim, "r--", label="roll = cmd")
    cc = np.polyfit(rc_i[m_fb], roll[m_fb], 1)
    ax[0].plot(lim, np.polyval(cc, lim), "k", label="fit: %.2f*cmd %+.1f" % tuple(cc))
    ax[0].set_xlabel("cmd roll dari stick [deg]")
    ax[0].set_ylabel("roll aktual [deg]")
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[0].set_title("FBWA: pelacakan roll")
    ax[1].scatter(pc_i[m_fb], pitch[m_fb], s=6, alpha=0.5, color="g")
    cp = np.polyfit(pc_i[m_fb], pitch[m_fb], 1)
    ax[1].plot([-30, 40], [-30, 40], "r--", label="pitch = cmd")
    ax[1].plot([-30, 40], np.polyval(cp, [-30, 40]), "k", label="fit: %.2f*cmd %+.1f" % tuple(cp))
    ax[1].set_xlabel("cmd pitch dari stick [deg]")
    ax[1].set_ylabel("pitch aktual [deg]")
    ax[1].legend()
    ax[1].grid(alpha=0.3)
    ax[1].set_title("FBWA: pelacakan pitch")
    # hukum kendali aileron teridentifikasi
    ail_i = np.interp(ta, ts, ail) - 1500
    pred = ROLL_GAIN_UST_PER_DEG * (roll - rc_i)
    ax[2].scatter(np.clip(pred, -420, 420)[m_fb], ail_i[m_fb], s=6, alpha=0.5, color="purple")
    ax[2].plot([-400, 400], [-400, 400], "r--")
    ax[2].set_xlabel("%.0f*(roll - cmd_roll)  [us]" % ROLL_GAIN_UST_PER_DEG)
    ax[2].set_ylabel("aileron PWM - 1500 [us]")
    ax[2].set_title("Hukum kendali aileron yang teridentifikasi (FBWA)")
    ax[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "03_fbwa_pelacakan_dan_hukum_kendali.png"), dpi=130)
    plt.close(fig)

    # ---------------------------------------------------------------- Fig 4 : AUTO zoom
    if auto_iv:
        fig, axes = plt.subplots(5, len(auto_iv), figsize=(5.5 * len(auto_iv) + 2, 13), squeeze=False)
        for j, (a, b) in enumerate(auto_iv):
            pa, pb = a - 3, b + 3
            ax = axes[:, j]
            for x in ax:
                x.axvspan(a, b, color=MODE_COLOR["AUTO"], alpha=0.8, lw=0)
                x.set_xlim(pa, pb)
                x.grid(alpha=0.3)
            mt = (ta >= pa) & (ta <= pb)
            ax[0].plot(ta[mt], roll[mt], "b.-", lw=1, ms=3, label="roll aktual")
            mn = (tn >= pa - 2) & (tn <= pb + 2)
            ax[0].step(tn[mn], nav_roll[mn], "r", where="post", lw=1.3, label="nav_roll (L1 -> setpoint)")
            ax[0].step(tn[mn], -nav_roll[mn], "m:", where="post", lw=1.0, label="-nav_roll")
            ax[0].set_ylabel("roll [deg]")
            ax[0].legend(fontsize=7, loc="upper right")
            ax[0].set_title("AUTO #%d  (%.1f-%.1f s)" % (j + 1, a, b))
            tt = np.arange(pa, pb, 0.125)
            r = np.interp(tt, ta, roll)
            A = np.interp(tt, ts, ail) - 1500
            ax[1].plot(tt, r - A / ROLL_GAIN_UST_PER_DEG, "k-", lw=1, label="setpoint roll yang DIKERJAKAN FC\n(roll - ail/18)")
            ax[1].step(tn[mn], nav_roll[mn], "r", where="post", lw=1.0, label="nav_roll")
            ax[1].set_ylabel("setpoint [deg]")
            ax[1].set_ylim(-80, 80)
            ax[1].legend(fontsize=7, loc="upper right")
            mm = (ts >= pa) & (ts <= pb)
            ax[2].plot(ts[mm], ail[mm], "b", lw=1, label="aileron PWM")
            ax[2].axhline(1099, color="gray", ls=":")
            ax[2].axhline(1900, color="gray", ls=":")
            ax[2].set_ylabel("PWM [us]")
            ax[2].legend(fontsize=7)
            mw = (tn >= pa) & (tn <= pb)
            ax[3].plot(tn[mw], wp_dist[mw], "o-", color="darkgreen", label="wp_dist [m]")
            ax[3].plot(tn[mw], xtrack[mw], "s-", color="orange", label="xtrack [m]")
            ax[3].set_ylabel("m")
            ax[3].legend(fontsize=7)
            mt2 = (tp >= pa) & (tp <= pb)
            ax[4].plot(ta[mt], pitch[mt], "g", lw=1, label="pitch")
            ax[4].step(tn[mn], nav_pitch[mn], "r", where="post", label="nav_pitch (TECS)")
            ax[4].set_ylabel("pitch [deg]")
            ax[4].set_xlabel("waktu [s]")
            ax[4].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "04_auto_zoom_tiap_segmen.png"), dpi=120)
        plt.close(fig)

    # ---------------------------------------------------------------- Fig 5 : track
    lat0, lon0 = lat[0], lon[0]
    kx = 111320.0 * np.cos(np.radians(lat0))
    X = (lon - lon0) * kx
    Y = (lat - lat0) * 111320.0
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(X, Y, color="lightgray", lw=1)
    mode_at = lambda t: next((n for a, b, n, arm in iv if a <= t < b and arm), None)
    colors = {"FBWA": "tab:blue", "AUTO": "tab:red"}
    for a, b, n in airborne:
        m = (tp >= a) & (tp <= b)
        ax.plot(X[m], Y[m], color=colors.get(n, "k"), lw=2, label=n if n not in ax.get_legend_handles_labels()[1] else None)
    ax.plot(0, 0, "k^", ms=10, label="posisi awal / HOME")
    hit = [(d["_t"] - t0, d["seq"]) for d in D.get("MISSION_ITEM_REACHED", [])]
    for t, seq in hit:
        i = np.argmin(np.abs(tp - t))
        ax.plot(X[i], Y[i], "g*", ms=14)
        ax.annotate("WP seq %d tercapai\n(t=%.1f s)" % (seq, t), (X[i], Y[i]), textcoords="offset points", xytext=(8, 8), fontsize=8)
    for a, b in auto_iv:
        i = np.argmin(np.abs(tp - a))
        ax.annotate("AUTO on %.0fs" % a, (X[i], Y[i]), textcoords="offset points", xytext=(-40, -15), fontsize=8, color="red")
    ax.set_aspect("equal")
    ax.set_xlabel("timur [m]")
    ax.set_ylabel("utara [m]")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title("Lintasan (GLOBAL_POSITION_INT) -- biru: FBWA, merah: AUTO")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "05_lintasan_gps.png"), dpi=130)
    plt.close(fig)

    # ---------------------------------------------------------------- Fig 6 : alt/speed
    fig, ax = plt.subplots(4, 1, figsize=(15, 10), sharex=True)
    for a in ax:
        shade(a, iv)
    ax[0].plot(tv, alt, "k", label="altitude VFR_HUD (baro/AGL)")
    ax[0].set_ylabel("alt [m]")
    ax[0].legend(loc="upper right")
    ax[1].plot(tv, gs, "b", label="groundspeed")
    ax[1].plot(tv, airspeed, "r", label="airspeed (selalu 0 -> sensor/TECS mati)")
    ax[1].axhline(15, color="gray", ls=":", label="AIRSPEED_CRUISE 15")
    ax[1].axhline(14, color="gray", ls="--", lw=0.7, label="MIN_AIRSPEED 14")
    ax[1].set_ylabel("m/s")
    ax[1].legend(loc="upper right", fontsize=8)
    ax[2].plot(tv, climb, "g", label="climb rate")
    ax[2].set_ylabel("m/s")
    ax[2].legend(loc="upper right")
    ax[3].plot(tv, thr, "k", label="throttle VFR_HUD [%]")
    ax[3].plot(tr, (ch3 - 988) / (2012 - 988) * 100, "orange", lw=0.7, label="stick throttle ch3 [%]")
    ax[3].set_ylabel("%")
    ax[3].set_xlabel("waktu [s]")
    ax[3].legend(loc="upper right")
    ax[3].set_xlim(0, 140)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "06_altitude_kecepatan_throttle.png"), dpi=130)
    plt.close(fig)

    # ---------------------------------------------------------------- Fig 7 : Mahony vs BNO
    fig, ax = plt.subplots(3, 1, figsize=(15, 9), sharex=True)
    pairs = (("MHN_ROLL", roll, "roll"), ("MHN_PITCH", pitch, "pitch"), ("MHN_YAW", yaw, "yaw"))
    for k, (n, ref, lab) in enumerate(pairs):
        shade(ax[k], iv)
        ax[k].plot(ta, ref, "b", lw=1, label="%s BNO055 (dipakai kontrol)" % lab)
        tm, vm = named[n]
        ax[k].plot(tm, vm, "r", lw=0.9, label="%s Mahony (paralel, tidak dipakai kontrol)" % lab)
        ax[k].set_ylabel("%s [deg]" % lab)
        ax[k].legend(loc="upper right", fontsize=8)
        ax[k].grid(alpha=0.3)
    ax[2].set_xlabel("waktu [s]")
    ax[2].set_xlim(0, 140)
    fig.suptitle("Perbandingan estimator attitude: BNO055 on-chip vs Mahony")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "07_bno055_vs_mahony.png"), dpi=130)
    plt.close(fig)

    # ---------------------------------------------------------------- Fig 8 : link health
    tR, rssi = series(D, "RADIO_STATUS", "rssi", t0)
    _, remrssi = series(D, "RADIO_STATUS", "remrssi", t0)
    _, rxerr = series(D, "RADIO_STATUS", "rxerrors", t0)
    tg, sats = series(D, "GPS_RAW_INT", "satellites_visible", t0)
    _, eph = series(D, "GPS_RAW_INT", "eph", t0, 0.01)
    tb, vbat = series(D, "BATTERY_STATUS", "battery_remaining", t0)
    vb = np.array([d["voltages"][0] for d in D["BATTERY_STATUS"]], float)
    fig, ax = plt.subplots(4, 1, figsize=(15, 10), sharex=True)
    ax[0].plot(tR, rssi, label="RSSI lokal")
    ax[0].plot(tR, remrssi, label="RSSI remote")
    ax[0].set_ylabel("RSSI (0-255)")
    ax[0].legend()
    ax[1].plot(tR, rxerr, "r", label="rxerrors kumulatif")
    ax[1].legend()
    ax[2].plot(tg, sats, label="satelit")
    ax2 = ax[2].twinx()
    ax2.plot(tg, eph, "r", label="HDOP (eph)")
    ax[2].legend(loc="upper left")
    ax2.legend(loc="upper right")
    ax[3].plot(tb, vb, "k", label="voltages[0] [mV] (nilai ~1.9 V -> bukan pack, kalibrasi ADC?)")
    ax[3].legend()
    ax[3].set_xlabel("waktu [s]")
    for a in ax:
        shade(a, iv)
        a.grid(alpha=0.3)
    dt_att = np.diff(ta)
    for i in np.where(dt_att > 2.0)[0]:
        for a in ax:
            a.axvspan(ta[i], ta[i + 1], color="red", alpha=0.15, lw=0)
    fig.suptitle("Kesehatan link & sensor (merah = ATTITUDE hilang > 2 s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "08_link_gps_baterai.png"), dpi=130)
    plt.close(fig)

    print("selesai ->", out)


if __name__ == "__main__":
    main()
