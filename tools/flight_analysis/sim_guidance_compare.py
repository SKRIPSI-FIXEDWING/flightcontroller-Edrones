#!/usr/bin/env python3
"""Simulasi offline: L1 tetap vs L1 + fuzzy vs PID, di bawah pulsa gangguan roll
seperti pemicu CH7 (DisturbanceInjector). Firmware TIDAK diubah; semua hukum
kendali di sini adalah salinan Python dari kode firmware:

  L1 tetap   L1Controller::updateWaypoint / updateLoiter, NAVL1_PERIOD 14 s,
             NAVL1_DAMPING 0.707, NAVL1_XTRACK_I 0.2
  L1 + fuzzy sama, periode = 14 + dT dari FuzzyL1Tuner (Tabel 3.6, MF dari
             fuzzy_mf_from_data.py), dibatasi 12-16 s
  PID        a_lat = Kp e + Kd de/dt + Ki int(e) (+ feedforward sentripetal di loiter),
             gain SETARA L1 pada periode 14 s (Park et al. 2004, linearisasi
             error kecil): Kp = w^2, Kd = 2 zeta w, w = 2 pi / T. Pada loiter ini
             persis suku PD lingkaran di updateLoiter; bedanya PID tidak punya
             cabang "capture" L1 dan tidak punya batas sudut intersep.

Model wahana (sama dengan sim_l1_mission.py): titik massa, airspeed konstan,
angin konstan, roll orde-1 (tau) menuju nav_roll yang dibatasi bank limit,
heading rate = g tan(roll) / V. Gangguan = offset roll dphi (derajat) yang
ditambahkan ke target roll selama DIST_MS -- padanan DIST_US (us) -> derajat
BELUM dikalibrasi; kalibrasi dari log run CH7 pertama (kolom roll vs dist_us).

Bukan pengganti uji terbang: tanpa dinamika pitch/altitude, gust, delay GPS,
atau osilasi loop attitude. Dipakai untuk membandingkan metode secara relatif.

Run:  python sim_guidance_compare.py
"""

import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fuzzy_mf_from_data as fz

G = 9.80665
HERE = os.path.dirname(os.path.abspath(__file__))

# ---- kondisi nominal (terbang 2026-10-26 loiter: airspeed ~22 m/s, angin 6-7 m/s) ----
V_AIR = 22.0
WIND_MPS = 6.5
WIND_FROM_DEG = 205.0
TAU = 0.4                 # konstanta waktu roll (kalibrasi sim_l1_mission.py)
DT = 0.02
PERIOD = 14.0
DAMPING = 0.707
XTRACK_I = 0.2
P_MIN, P_MAX = 12.0, 16.0
LOITER_BANK = 35.0        # LOIT_BANK_DEG
ROLL_LIMIT = 40.0         # ROLL_LIMIT_DEG (lurus)
DIST_MS = 2000.0          # DIST_MS default
RECOVERY_BAND = 5.0       # sama dengan analyze_sdlog_runs / analyze_tlog_loiter
RECOVERY_HOLD = 2.0
TOL = 10.0
GPS_HZ = 5.0              # posisi/kecepatan GPS 5 Hz ...
GPS_DELAY = 0.2           # ... dengan jeda 0,2 s (guidance memakai nilai terukur, metrik memakai nilai asli)

METHODS = ["L1 tetap", "L1 + fuzzy", "PID"]
COLORS = {"L1 tetap": "tab:blue", "L1 + fuzzy": "tab:red", "PID": "tab:green"}


# ---------------------------------------------------------------- fuzzy (LUT)
class FuzzyLUT:
    """dT(|e|, d|e|/dt) dari fuzzy_mf_from_data.infer (cermin eFLL, dicek sama
    dengan FuzzyL1Tuner::evaluate), ditabelkan lalu interpolasi bilinear."""

    def __init__(self):
        E, D, O = fz.build_sets(3.0, 10.0, 30.0, 1.25, 2.25)  # = FuzzyL1TunerConfig
        self.e = np.linspace(0.0, 60.0, 241)
        self.d = np.linspace(-4.5, 4.5, 181)
        self.z = np.array([[fz.infer(a, b, E, D, O) for b in self.d] for a in self.e])

    def __call__(self, e_abs, de):
        e_abs = min(max(e_abs, 0.0), 60.0)
        de = min(max(de, -4.5), 4.5)
        i = min(int(e_abs / 0.25), 239)
        j = min(int((de + 4.5) / 0.05), 179)
        fe = (e_abs - self.e[i]) / 0.25
        fd = (de - self.d[j]) / 0.05
        z = self.z
        return ((1 - fe) * (1 - fd) * z[i, j] + fe * (1 - fd) * z[i + 1, j]
                + (1 - fe) * fd * z[i, j + 1] + fe * fd * z[i + 1, j + 1])


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


# ------------------------------------------------------------- hukum kendali
def l1_straight(pos, vg, A, u, period, state):
    """L1Controller::updateWaypoint (cabang mengikuti garis). Return a_lat (+ = kanan)."""
    gs = max(np.linalg.norm(vg), 1.0)
    L1 = 0.3183099 * DAMPING * period * gs
    xt = cross(pos - A, u)  # = a_air % ab (firmware), + = kiri garis
    nu2 = math.atan2(cross(vg, u), float(vg @ u))
    nu1 = math.asin(max(-0.7071, min(0.7071, xt / max(L1, 0.1))))
    if abs(nu1) < math.radians(5.0):
        state["integ"] = max(-0.1, min(0.1, state["integ"] + nu1 * XTRACK_I * DT))
    nu1 += state["integ"]
    nu = max(-1.5708, min(1.5708, nu1 + nu2))
    return 4.0 * DAMPING ** 2 * gs * gs / L1 * math.sin(nu)


def l1_loiter(pos, vg, C, R, direction, period):
    """L1Controller::updateLoiter. Return a_lat (+ = kanan)."""
    gs = max(np.linalg.norm(vg), 1.0)
    omega = 2.0 * math.pi / period
    kx, kv, k_l1 = omega ** 2, 2.0 * DAMPING * omega, 4.0 * DAMPING ** 2
    L1 = 0.3183099 * DAMPING * period * gs
    a_air = pos - C
    a_u = a_air / max(np.linalg.norm(a_air), 0.1)
    xtrack_vel_cap = cross(a_u, vg)
    ltrack_vel_cap = -float(vg @ a_u)
    nu = max(-math.pi / 2, min(math.pi / 2, math.atan2(xtrack_vel_cap, ltrack_vel_cap)))
    cap = k_l1 * gs * gs / L1 * math.sin(nu)
    err = np.linalg.norm(a_air) - R
    vel = -ltrack_vel_cap
    pd = err * kx + vel * kv
    vt = xtrack_vel_cap * direction
    if ltrack_vel_cap < 0.0 and vt < 0.0:
        pd = max(pd, 0.0)
    ctr = vt * vt / max(0.5 * R, R + err)
    circ = direction * (pd + ctr)
    if err > 0.0 and direction * cap < direction * circ:
        return cap
    return circ


W0 = 2.0 * math.pi / PERIOD
KP, KD = W0 ** 2, 2.0 * DAMPING * W0
KI = KP / 20.0            # Ti = 20 s
I_LIM = G * math.tan(math.radians(10.0))   # anti-windup: integrator maks setara 10 deg bank


def pid_straight(pos, vg, A, u, state):
    e = cross(u, pos - A)                   # + = kanan garis
    de = cross(u, vg)
    state["i"] = max(-I_LIM, min(I_LIM, state["i"] + KI * e * DT))
    return -(KP * e + KD * de + state["i"])


def pid_loiter(pos, vg, C, R, direction, state):
    a_air = pos - C
    r = np.linalg.norm(a_air)
    a_u = a_air / max(r, 0.1)
    e = r - R                               # + = di luar lingkaran
    de = float(vg @ a_u)
    vt = cross(a_u, vg) * direction
    ff = vt * vt / max(0.5 * R, R + e)      # feedforward sentripetal (sama dengan L1)
    state["i"] = max(-I_LIM, min(I_LIM, state["i"] + KI * e * DT))
    return direction * (KP * e + KD * de + state["i"] + ff)


# ------------------------------------------------------------------- simulasi
def wind_vec():
    to = math.radians(WIND_FROM_DEG + 180.0)
    return np.array([math.cos(to), math.sin(to)]) * WIND_MPS


def run(method, scenario, pulses, dphi, t_end, R=112.0, direction=1, fuzzy=None, bank=None, bank_fn=None):
    """scenario 'loiter' (pusat 0,0, CW) atau 'lurus' (garis utara melalui 0,0).
    pulses = waktu mulai pulsa [s]; dphi = offset roll bertanda [deg].
    bank = batas bank tetap (default LOITER_BANK / ROLL_LIMIT); bank_fn(dT) ->
    batas bank per siklus untuk varian fuzzy yang juga menyetel bank."""
    bank_override = bank
    w = wind_vec()
    if scenario == "loiter":
        pos = np.array([0.0, -R])           # di lingkaran, sisi barat, menghadap utara (CW)
        psi = 0.0
        bank = LOITER_BANK if bank_override is None else bank_override
    else:
        pos = np.array([0.0, 0.0])
        psi = 0.0
        bank = ROLL_LIMIT if bank_override is None else bank_override
    A, u = np.array([0.0, 0.0]), np.array([1.0, 0.0])
    phi = 0.0
    st = {"integ": 0.0, "i": 0.0}
    period = PERIOD
    n = int(t_end / DT)
    log = np.zeros((n, 8))
    gps_every = max(int(round(1.0 / (GPS_HZ * DT))), 1)
    lag = int(round(GPS_DELAY / DT))
    hist = []
    for k in range(n):
        t = k * DT
        vg_true = np.array([math.cos(psi), math.sin(psi)]) * V_AIR + w
        e_true = (np.linalg.norm(pos) - R) if scenario == "loiter" else cross(u, pos - A)
        hist.append((pos.copy(), vg_true))
        if len(hist) > lag + 1:
            hist.pop(0)
        if k % gps_every == 0 or k == 0:
            gpos, gvg = hist[0]
        pos_true = pos
        pos, vg = gpos, gvg            # guidance melihat GPS terukur
        if scenario == "loiter":
            r = np.linalg.norm(pos)
            e = r - R
            de_signed = float(vg @ (pos / max(r, 0.1)))
        else:
            e = cross(u, pos - A)
            de_signed = cross(u, vg)
        d_abs = math.copysign(1.0, e) * de_signed if e != 0 else 0.0
        dT = 0.0
        if method == "L1 + fuzzy":
            dT = fuzzy(abs(e), d_abs)
            period = min(max(PERIOD + dT, P_MIN), P_MAX)
        if method == "PID":
            a = pid_loiter(pos, vg, np.zeros(2), R, direction, st) if scenario == "loiter" \
                else pid_straight(pos, vg, A, u, st)
        else:
            a = l1_loiter(pos, vg, np.zeros(2), R, direction, period) if scenario == "loiter" \
                else l1_straight(pos, vg, A, u, period, st)
        lim = bank_fn(dT) if bank_fn is not None else bank
        nav_roll = max(-lim, min(lim, math.degrees(math.atan(a / G))))
        dist = any(p <= t < p + DIST_MS / 1000.0 for p in pulses)
        target = nav_roll + (dphi if dist else 0.0)
        phi += (target - phi) * DT / TAU
        psi += G * math.tan(math.radians(phi)) / V_AIR * DT
        pos = pos_true + vg_true * DT
        log[k] = (t, pos_true[0], pos_true[1], e_true, nav_roll, phi, period, 1.0 if dist else 0.0)
    return log


def pulse_metrics(log, t0, window=40.0):
    t, e = log[:, 0], log[:, 3]
    t_end = t0 + DIST_MS / 1000.0
    m = (t >= t0) & (t < t0 + window)
    tt, ee = t[m], e[m]
    k_peak = int(np.argmax(np.abs(ee)))
    peak = float(ee[k_peak])
    side = math.copysign(1.0, peak)
    after = tt >= tt[k_peak]
    overshoot = float(max(0.0, np.max(-side * ee[after])))
    rec = float("nan")
    inside = np.abs(ee) <= RECOVERY_BAND
    for i in np.flatnonzero((tt >= t_end) & inside):
        j = (tt >= tt[i]) & (tt <= tt[i] + RECOVERY_HOLD)
        if tt[i] + RECOVERY_HOLD <= tt[-1] and np.all(inside[j]):
            rec = float(tt[i] - t_end)
            break
    sat = float(np.mean(np.abs(log[m, 4]) >= LOITER_BANK - 0.01))
    return dict(peak_abs=abs(peak), overshoot=overshoot, recovery_s=rec,
                rms=float(np.sqrt(np.mean(ee ** 2))), pct_out_tol=100.0 * float(np.mean(np.abs(ee) > TOL)),
                sat_frac=sat)


# ---------------------------------------------------------------------- utama
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "sim_guidance_2026-10-26"))
    ap.add_argument("--radius", type=float, nargs="+", default=[112.0, 75.0])
    ap.add_argument("--amp", type=float, nargs="+", default=[10.0, 20.0, 30.0], help="offset roll gangguan [deg]")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    fuzzy = FuzzyLUT()

    rows = []
    settle = 60.0
    gap = 45.0
    n_pulse = 8
    pulses = [settle + i * gap for i in range(n_pulse)]   # 45 s ~ 1,4 putaran -> fase berbeda-beda
    t_end = pulses[-1] + gap
    examples = {}

    # ---- loiter ----
    for R in args.radius:
        for amp in args.amp:
            for sgn, arah in [(+1, "ke dalam"), (-1, "ke luar")]:
                for mth in METHODS:
                    L = run(mth, "loiter", pulses, sgn * amp, t_end, R=R, fuzzy=fuzzy)
                    steady = L[(L[:, 0] > 20) & (L[:, 0] < settle)]
                    for i, p in enumerate(pulses):
                        r = pulse_metrics(L, p)
                        rows.append(dict(skenario="loiter", radius=R, amp_deg=amp, arah=arah, metode=mth,
                                         pulsa=i + 1, steady_rms=float(np.sqrt(np.mean(steady[:, 3] ** 2))), **r))
                    if amp == 20.0:
                        examples[(R, arah, mth)] = L
    # ---- lurus (angin silang) ----
    for amp in args.amp:
        for sgn, arah in [(+1, "kanan"), (-1, "kiri")]:
            for mth in METHODS:
                L = run(mth, "lurus", [30.0], sgn * amp, 90.0, fuzzy=fuzzy)
                r = pulse_metrics(L, 30.0, window=55.0)
                rows.append(dict(skenario="lurus", radius=0, amp_deg=amp, arah=arah, metode=mth, pulsa=1,
                                 steady_rms=float(np.sqrt(np.mean(L[(L[:, 0] > 15) & (L[:, 0] < 30), 3] ** 2))), **r))
                if amp == 20.0:
                    examples[("lurus", arah, mth)] = L

    keys = list(rows[0].keys())
    with open(os.path.join(args.out, "metrik_per_pulsa.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        wr.writerows(rows)

    # ---- ringkasan ----
    def agg(sel):
        out = {}
        for k in ("steady_rms", "peak_abs", "overshoot", "recovery_s", "rms", "pct_out_tol", "sat_frac"):
            v = np.array([r[k] for r in sel], dtype=float)
            out[k] = float(np.nanmean(v))
        out["gagal_pulih"] = int(sum(1 for r in sel if math.isnan(r["recovery_s"])))
        out["n"] = len(sel)
        return out

    summ = []
    for sk, R in [("loiter", R) for R in args.radius] + [("lurus", 0)]:
        for amp in args.amp:
            for mth in METHODS:
                sel = [r for r in rows if r["skenario"] == sk and r["radius"] == R and r["amp_deg"] == amp
                       and r["metode"] == mth]
                summ.append(dict(skenario=sk, radius=R, amp_deg=amp, metode=mth, **agg(sel)))
    with open(os.path.join(args.out, "ringkasan.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(summ[0].keys()))
        wr.writeheader()
        for s in summ:
            wr.writerow({k: (round(v, 2) if isinstance(v, float) else v) for k, v in s.items()})

    print(f"{'skenario':8s} {'R':>4s} {'amp':>4s} {'metode':11s} {'steadyRMS':>9s} {'puncak':>7s} "
          f"{'overshoot':>9s} {'pulih_s':>7s} {'gagal':>5s} {'RMS40s':>6s} {'%>tol':>6s} {'sat':>5s}")
    for s in summ:
        print(f"{s['skenario']:8s} {s['radius']:4.0f} {s['amp_deg']:4.0f} {s['metode']:11s} {s['steady_rms']:9.2f} "
              f"{s['peak_abs']:7.1f} {s['overshoot']:9.1f} {s['recovery_s']:7.1f} {s['gagal_pulih']:2d}/{s['n']:<2d} "
              f"{s['rms']:6.1f} {s['pct_out_tol']:6.1f} {s['sat_frac']:5.2f}")

    # ---- gambar 01: lintasan loiter, satu pulsa 20 deg ke luar ----
    for R in args.radius:
        fig, axs = plt.subplots(1, 2, figsize=(12, 5.4))
        for ax, arah in zip(axs, ["ke luar", "ke dalam"]):
            th = np.linspace(0, 2 * np.pi, 300)
            ax.plot(R * np.sin(th), R * np.cos(th), "k--", lw=1, label=f"setpoint R {R:.0f} m")
            for rr in (R - TOL, R + TOL):
                ax.plot(rr * np.sin(th), rr * np.cos(th), color="0.7", lw=0.8)
            p = pulses[2]
            for mth in METHODS:
                L = examples[(R, arah, mth)]
                m = (L[:, 0] >= p - 5) & (L[:, 0] < p + 35)
                ax.plot(L[m, 2], L[m, 1], color=COLORS[mth], lw=1.6, label=mth)
                md = m & (L[:, 7] > 0)
                ax.plot(L[md, 2], L[md, 1], color="k", lw=4, alpha=0.35)
            ax.set_aspect("equal")
            ax.set_title(f"Pulsa 20° {arah} (garis tebal abu = saat gangguan)")
            ax.set_xlabel("timur [m]"); ax.set_ylabel("utara [m]"); ax.grid(alpha=0.3)
            ax.legend(loc="lower left", fontsize=8)
        fig.suptitle(f"Simulasi loiter R {R:.0f} m, angin {WIND_MPS} m/s, airspeed {V_AIR} m/s")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, f"01_lintasan_loiter_R{R:.0f}.png"), dpi=150)
        plt.close(fig)

        # ---- gambar 02: galat & periode vs waktu ----
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        p = pulses[2]
        for i, arah in enumerate(["ke luar", "ke dalam"]):
            ax = axs[i]
            for mth in METHODS:
                L = examples[(R, arah, mth)]
                m = (L[:, 0] >= p - 5) & (L[:, 0] < p + 40)
                ax.plot(L[m, 0] - p, L[m, 3], color=COLORS[mth], label=mth)
            ax.axvspan(0, DIST_MS / 1000, color="k", alpha=0.12)
            ax.axhspan(-RECOVERY_BAND, RECOVERY_BAND, color="green", alpha=0.08)
            ax.axhline(TOL, color="0.6", ls=":"); ax.axhline(-TOL, color="0.6", ls=":")
            ax.set_ylabel("galat radius [m]\n(+ luar)"); ax.set_title(f"pulsa 20° {arah}"); ax.grid(alpha=0.3)
            ax.legend(fontsize=8, loc="upper right")
        for arah, ls in [("ke luar", "-"), ("ke dalam", "--")]:
            L = examples[(R, arah, "L1 + fuzzy")]
            m = (L[:, 0] >= p - 5) & (L[:, 0] < p + 40)
            axs[2].plot(L[m, 0] - p, L[m, 6], color=COLORS["L1 + fuzzy"], ls=ls, label=f"fuzzy, {arah}")
        axs[2].axhline(PERIOD, color=COLORS["L1 tetap"], label="L1 tetap (14 s)")
        axs[2].set_ylabel("periode L1 [s]"); axs[2].set_xlabel("waktu sejak pulsa mulai [s]")
        axs[2].set_ylim(11.5, 16.5); axs[2].grid(alpha=0.3); axs[2].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, f"02_galat_dan_periode_R{R:.0f}.png"), dpi=150)
        plt.close(fig)

    # ---- gambar 03: metrik vs besar gangguan ----
    for sk, R in [("loiter", R) for R in args.radius] + [("lurus", 0)]:
        fig, axs = plt.subplots(1, 4, figsize=(15, 3.8))
        for ax, (k, lab) in zip(axs, [("peak_abs", "galat puncak [m]"), ("overshoot", "overshoot [m]"),
                                      ("recovery_s", "waktu pulih [s]"), ("rms", "RMS galat 40 s [m]")]):
            x = np.arange(len(args.amp))
            for j, mth in enumerate(METHODS):
                v = [next(s[k] for s in summ if s["skenario"] == sk and s["radius"] == R and s["amp_deg"] == a
                          and s["metode"] == mth) for a in args.amp]
                ax.bar(x + (j - 1) * 0.27, v, 0.27, color=COLORS[mth], label=mth)
            ax.set_xticks(x); ax.set_xticklabels([f"{a:.0f}°" for a in args.amp])
            ax.set_xlabel("offset roll gangguan"); ax.set_title(lab); ax.grid(axis="y", alpha=0.3)
        axs[0].legend(fontsize=8)
        title = f"loiter R {R:.0f} m" if sk == "loiter" else "lintasan lurus (angin silang)"
        fig.suptitle(f"Rata-rata semua pulsa (fase & arah), {title}")
        fig.tight_layout()
        name = f"03_metrik_loiter_R{R:.0f}.png" if sk == "loiter" else "03_metrik_lurus.png"
        fig.savefig(os.path.join(args.out, name), dpi=150)
        plt.close(fig)

    # ---- gambar 04: lurus ----
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for ax, arah in zip(axs, ["kanan", "kiri"]):
        for mth in METHODS:
            L = examples[("lurus", arah, mth)]
            m = L[:, 0] >= 25
            ax.plot(L[m, 0] - 30, L[m, 3], color=COLORS[mth], label=mth)
        ax.axvspan(0, DIST_MS / 1000, color="k", alpha=0.12)
        ax.axhspan(-RECOVERY_BAND, RECOVERY_BAND, color="green", alpha=0.08)
        ax.set_ylabel("cross-track error [m]\n(+ kanan)"); ax.set_title(f"lintasan lurus, pulsa 20° roll {arah}")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    axs[1].set_xlabel("waktu sejak pulsa mulai [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "04_galat_lurus.png"), dpi=150)
    plt.close(fig)
    print(f"\nhasil -> {args.out}")


if __name__ == "__main__":
    main()
