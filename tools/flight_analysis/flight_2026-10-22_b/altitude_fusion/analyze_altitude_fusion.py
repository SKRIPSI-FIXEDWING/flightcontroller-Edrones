"""Analisis drift ALT_FUSED (fc::AltitudeFusion) dari tlog 2026-10-22.

Pakai:
    python analyze_altitude_fusion.py "~/Downloads/2026-10-22 13-35-40.tlog" \
        "~/Downloads/2026-10-22 12-35-20.tlog" [--sim DIR]

Catatan isi tlog (firmware saat itu):
  - ALT_GPS hanyalah flag 0/1 (gpsActive()), BUKAN tinggi GPS.
  - GPS_RAW_INT.alt dan GLOBAL_POSITION_INT.alt berisi altitude fused, bukan
    tinggi GNSS -> tinggi/kecepatan vertikal GPS mentah tidak terekam.
  - ATTITUDE = Euler BNO055, MHN_* = Mahony.
  - SD card gagal init -> tidak ada log akselerometer mentah.
Kebenaran ("truth") dipakai: baro di darat sebelum lepas landas dan sesudah
mendarat (pesawat diam di tempat yang sama).

--sim DIR: folder berisi tr_{old,new}_{f1,stuck}.txt dari sim.cpp (opsional).
"""
import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

OUT = os.path.dirname(os.path.abspath(__file__))
BLUE, ORANGE, AQUA, YELLOW, RED = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e34948"
INK2, GRID = "#52514e", "#e4e3df"

plt.rcParams.update({
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "axes.edgecolor": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 1.6, "font.size": 9, "axes.titlesize": 10,
    "axes.titleweight": "bold", "legend.frameon": False,
})


def load(path):
    m = mavutil.mavlink_connection(os.path.expanduser(path))
    d, t0 = {}, None
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        if msg.get_srcSystem() == 255:
            continue
        if t0 is None:
            t0 = msg._timestamp
        ts, typ = msg._timestamp - t0, msg.get_type()
        if typ == "NAMED_VALUE_FLOAT":
            d.setdefault(msg.name, []).append((ts, msg.value))
        elif typ == "ATTITUDE":
            d.setdefault("att", []).append((ts, msg.roll, msg.pitch))
        elif typ == "VFR_HUD":
            d.setdefault("hud", []).append((ts, msg.airspeed, msg.climb))
    return {k: np.array(v) for k, v in d.items()}


def grid(d, a, b, dt=0.2):
    t = np.arange(a, b, dt)
    g = {"t": t}
    for k in ["ALT_FUSED", "ALT_BARO", "ALT_BBIAS", "ALT_REJ", "ALT_GPS", "MHN_ROLL"]:
        g[k] = np.interp(t, d[k][:, 0], d[k][:, 1])
    g["roll"] = np.degrees(np.interp(t, d["att"][:, 0], d["att"][:, 1]))
    g["pitch"] = np.degrees(np.interp(t, d["att"][:, 0], d["att"][:, 2]))
    g["asp"] = np.interp(t, d["hud"][:, 0], d["hud"][:, 1])
    g["climb"] = np.interp(t, d["hud"][:, 0], d["hud"][:, 2])
    k = 10
    sm = np.convolve(np.pad(g["ALT_BARO"], k, mode="edge"), np.ones(k) / k, "same")[k:-k]
    g["baro_rate"] = np.gradient(sm, t)
    return g


def label_end(ax, x, y, text, color):
    ax.annotate(text, (x[-1], y[-1]), xytext=(4, 0), textcoords="offset points",
                color=INK2, fontsize=8, va="center")
    ax.plot([x[-1]], [y[-1]], "o", ms=4, color=color)


def flags(ax, g):
    ax.fill_between(g["t"], 0, g["ALT_REJ"], step="mid", color=ORANGE, alpha=0.35, lw=0,
                    label="ALT_REJ = 1 (baro ditolak)")
    ax.fill_between(g["t"], 1.2, 1.2 + g["ALT_GPS"], step="mid", color=AQUA, alpha=0.35, lw=0,
                    label="ALT_GPS = 1 (flag gpsActive, bukan tinggi)")
    ax.set_yticks([])
    ax.set_ylim(-0.1, 2.3)
    ax.legend(loc="upper left", ncol=2, fontsize=8)


def plot_overview(g, name, title, mark=None):
    fig, ax = plt.subplots(4, 1, figsize=(11, 8.5), sharex=True,
                           gridspec_kw={"height_ratios": [3, 1.4, 0.8, 1.4]})
    t = g["t"]
    ax[0].plot(t, g["ALT_BARO"], color=BLUE, label="ALT_BARO (baro saja)")
    ax[0].plot(t, g["ALT_FUSED"], color=ORANGE, label="ALT_FUSED (dipakai TECS)")
    label_end(ax[0], t, g["ALT_BARO"], "baro", BLUE)
    label_end(ax[0], t, g["ALT_FUSED"], "fused", ORANGE)
    ax[0].set_ylabel("altitude [m]")
    ax[0].legend(loc="upper left")
    ax[0].set_title(title)
    ax[1].plot(t, g["ALT_BBIAS"], color=ORANGE)
    ax[1].set_ylabel("ALT_BBIAS [m]")
    flags(ax[2], g)
    ax[3].plot(t, g["roll"], color=BLUE, label="roll BNO055")
    ax[3].plot(t, g["pitch"], color=YELLOW, label="pitch BNO055")
    ax[3].set_ylabel("[deg]")
    ax[3].legend(loc="upper left", ncol=2)
    ax[3].set_xlabel("waktu tlog [s]")
    if mark:
        for a in ax:
            a.axvspan(*mark, color=GRID, alpha=0.6, lw=0)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)


def plot_zoom(g, name):
    fig, ax = plt.subplots(4, 1, figsize=(11, 8.5), sharex=True,
                           gridspec_kw={"height_ratios": [2.4, 1.8, 0.8, 1.4]})
    t = g["t"]
    ax[0].plot(t, g["ALT_BARO"], color=BLUE, label="ALT_BARO")
    ax[0].plot(t, g["ALT_FUSED"], color=ORANGE, label="ALT_FUSED")
    ax[0].plot(t, g["ALT_FUSED"] + g["ALT_BBIAS"], color=ORANGE, ls=":", lw=1.2,
               label="ALT_FUSED + ALT_BBIAS (= baro jika konsisten)")
    ax[0].set_ylabel("altitude [m]")
    ax[0].legend(loc="lower left", fontsize=8)
    ax[0].set_title("Flight 3, loiter bank 35-43 deg: fused turun saat pesawat sebenarnya naik")
    ax[0].annotate("re-anchor: BBIAS -3 -> +11 m\n(GPS masih 'aktif' 2 s, padahal\nsudah tidak setuju)",
                   (873.5, 77), xytext=(862.5, 42), fontsize=8, color=INK2,
                   arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    ax[0].annotate("reset tanpa GPS (3 s):\n+29 m dalam 1 sampel", (878, 89), xytext=(881, 42),
                   fontsize=8, color=INK2, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    ax[1].plot(t, g["baro_rate"], color=BLUE, label="d(baro)/dt")
    ax[1].plot(t, g["climb"], color=ORANGE, label="laju naik fused (VFR_HUD.climb)")
    ax[1].plot(t, g["asp"] * np.sin(np.radians(g["pitch"] - 3.0)), color=YELLOW, lw=1.2,
               label="V_udara * sin(pitch - 3 deg)")
    ax[1].axhline(0, color=INK2, lw=0.6)
    ax[1].set_ylabel("laju naik [m/s]")
    ax[1].legend(loc="lower left", fontsize=8, ncol=3)
    flags(ax[2], g)
    ax[3].plot(t, g["roll"], color=BLUE, label="roll BNO055")
    ax[3].plot(t, g["MHN_ROLL"], color=BLUE, ls="--", lw=1.0, label="roll Mahony")
    ax[3].plot(t, g["pitch"], color=YELLOW, label="pitch BNO055")
    ax[3].set_ylabel("[deg]")
    ax[3].legend(loc="lower left", ncol=3, fontsize=8)
    ax[3].set_xlabel("waktu tlog [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)


def accel_error_windows(g):
    """Galat akselerasi tersirat: kelengkungan (fused - baro) di jendela 3 s
    saat baro ditolak (filter praktis hanya inersia), tanpa re-anchor/reset."""
    t, e = g["t"], g["ALT_FUSED"] - g["ALT_BARO"]
    fly, rows, w = g["asp"] > 15, [], 15
    for i in range(0, len(t) - w, w):
        s = slice(i, i + w)
        if not fly[s].all() or g["ALT_REJ"][s].mean() < 0.99:
            continue
        if np.any(np.abs(np.diff(g["ALT_BBIAS"][s])) > 0.5) or np.any(np.abs(np.diff(g["ALT_FUSED"][s])) > 3):
            continue
        p = np.polyfit(t[s] - t[i], e[s], 2)
        rows.append((2 * p[0], np.mean(np.abs(g["roll"][s]))))
    return np.array(rows).reshape(-1, 2)


def plot_accel(rows, name):
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.scatter(rows[:, 1], rows[:, 0], s=36, color=BLUE, edgecolor="#fcfcfb", lw=1.5)
    ax.axhline(0, color=INK2, lw=0.6)
    lines = []
    for lo, hi in [(0, 25), (25, 50)]:
        m = (rows[:, 1] >= lo) & (rows[:, 1] < hi)
        if m.any():
            a = np.abs(rows[m, 0])
            lines.append("|roll| %d-%d deg: n=%d, median |galat| %.2f, maks %.2f m/s^2"
                         % (lo, hi, m.sum(), np.median(a), a.max()))
    ax.text(0.01, 0.97, "\n".join(lines), transform=ax.transAxes, fontsize=8, color=INK2, va="top")
    ax.axhspan(-0.2, 0.2, color=GRID, alpha=0.8, lw=0)
    ax.annotate("asumsi lama accel_noise 0,2", (1, 0.22), fontsize=8, color=INK2)
    ax.set_xlabel("|roll| rata-rata jendela [deg]")
    ax.set_ylabel("galat akselerasi vertikal tersirat [m/s^2]")
    ax.set_title("Galat input akselerasi (BNO055 linear accel . gravity), jendela 3 s")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)


def plot_ground_truth(cases, name):
    fig, ax = plt.subplots(figsize=(7.5, 4))
    x = np.arange(len(cases))
    eb = [c[2] - c[1] for c in cases]
    ef = [c[3] - c[1] for c in cases]
    ax.bar(x - 0.18, eb, 0.34, color=BLUE, label="baro - tanah", edgecolor="#fcfcfb", lw=2)
    ax.bar(x + 0.18, ef, 0.34, color=ORANGE, label="fused - tanah", edgecolor="#fcfcfb", lw=2)
    for xi, v in zip(x - 0.18, eb):
        ax.annotate("%+.0f m" % v, (xi, v), xytext=(0, 3 if v >= 0 else -11), textcoords="offset points",
                    ha="center", fontsize=8, color=INK2)
    for xi, v in zip(x + 0.18, ef):
        ax.annotate("%+.0f m" % v, (xi, v), xytext=(0, 3 if v >= 0 else -11), textcoords="offset points",
                    ha="center", fontsize=8, color=INK2)
    ax.set_xticks(x, [c[0] for c in cases])
    ax.axhline(0, color=INK2, lw=0.6)
    ax.set_ylabel("galat setelah mendarat [m]")
    ax.set_title("Setelah mendarat: baro benar (<= 2 m), fused salah 17-38 m")
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)


def plot_sim(simdir, name):
    fig, ax = plt.subplots(2, 1, figsize=(11, 6.5))
    for a, key, title in [
        (ax[0], "f1", "Sim: lintasan flight 1, galat accel -1,5 m/s^2 x sin|roll|/sin40, GPS ada"),
        (ax[1], "stuck", "Sim: data IMU beku (accel -8 m/s^2, t=60-120 s), GPS ada"),
    ]:
        old = np.loadtxt(os.path.join(simdir, "tr_old_%s.txt" % key))
        new = np.loadtxt(os.path.join(simdir, "tr_new_%s.txt" % key))
        a.plot(old[:, 0], old[:, 1], color=INK2, lw=1.0, label="truth")
        a.plot(old[:, 0], old[:, 3], color=ORANGE, label="fusion lama")
        a.plot(new[:, 0], new[:, 3], color=BLUE, label="fusion baru")
        lo, hi = old[:, 1].min() - 25, old[:, 1].max() + 25
        a.set_ylim(lo, hi)
        a.set_title(title + ("  (lama keluar skala: divergen)" if key == "stuck" else ""))
        a.set_ylabel("altitude [m]")
        a.legend(loc="upper right", ncol=3)
    ax[1].set_xlabel("waktu sim [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tlog_133540")
    ap.add_argument("tlog_123520")
    ap.add_argument("--sim")
    args = ap.parse_args()

    d = load(args.tlog_133540)
    f3 = grid(d, 765, 960)
    plot_overview(f3, "01_flight3_overview.png",
                  "2026-10-22 13-35-40 flight 3: ALT_FUSED vs ALT_BARO", mark=(865, 925))
    plot_zoom(grid(d, 862, 925, 0.1), "02_flight3_zoom_loiter.png")
    f1 = grid(d, 415, 640)
    plot_overview(f1, "03_flight1_overview.png", "2026-10-22 13-35-40 flight 1: ALT_FUSED vs ALT_BARO")

    d0 = load(args.tlog_123520)
    e0 = grid(d0, 100, 345)
    plot_overview(e0, "04_log_123520_overview.png", "2026-10-22 12-35-20: ALT_FUSED vs ALT_BARO")

    rows = np.vstack([accel_error_windows(g) for g in (f1, f3, e0)])
    plot_accel(rows, "05_accel_error_vs_bank.png")

    def at(g, t0):
        i = np.argmin(np.abs(g["t"] - t0))
        return g["ALT_BARO"][i], g["ALT_FUSED"][i]

    # (nama, tanah sebelum lepas landas [baro], baro & fused setelah diam)
    cases = []
    for label, g, t_before, t_after in [("12-35 (t=338)", e0, 110, 338), ("13-35 fl.1 (t=634)", f1, 418, 634),
                                        ("13-35 fl.3 (t=955)", f3, 768, 955)]:
        ground = at(g, t_before)[0]
        b, f = at(g, t_after)
        cases.append((label, ground, b, f))
        print("%-20s tanah %.1f  baro %.1f  fused %.1f" % (label, ground, b, f))
    plot_ground_truth(cases, "06_landing_ground_truth.png")
    if args.sim:
        plot_sim(args.sim, "07_sim_old_vs_new.png")
    print("REJ fraction in flight (13-35 fl.3): %.2f" % f3["ALT_REJ"][f3["asp"] > 15].mean())


if __name__ == "__main__":
    main()
