#!/usr/bin/env python3
"""Simulasi kinematik L1 (ArduPilot-style, sama dengan L1Controller::updateWaypoint)
pada misi poligon, untuk memilih NAVL1_PERIOD / kecepatan / bank limit / radius klaim.

Model: titik massa, airspeed konstan, angin konstan, roll orde-1 (tau) menuju
nav_roll yang dibatasi, bias roll opsional (offset terukur di log). Sekuensing WP
= radius klaim ATAU melewati finish-line (sama dengan Navigation::navigate()).

Bukan pengganti uji terbang: tidak ada dinamika pitch/altitude, gust, atau
osilasi loop attitude. Dipakai untuk membandingkan skenario secara relatif,
setelah dikalibrasi ke penerbangan 2026-10-15.

Run:  python sim_l1_mission.py                      (skenario 2026-10-15)
      python sim_l1_mission.py --plan "x.waypoints"  (plan Mission Planner, damping tetap)
"""

import argparse
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

G = 9.80665
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_2026-10-15")

# Misi 2026-10-15 (N, E dari HOME, meter) -- WP1..WP9
MISSION_REAL = np.array([
    [-37.3, -8.4], [-106.5, -32.0], [-156.5, 5.9], [-176.9, 79.9], [-146.1, 144.7],
    [13.0, 190.2], [76.1, 159.0], [95.5, 79.9], [57.7, 21.0]])
WIND_FROM_DEG = 205.0   # estimasi dari GPS (lingkaran kecepatan), log 2026-10-15
WIND_MPS = 6.3


def scaled_mission(target_mean_leg=100.0):
    legs = np.linalg.norm(np.diff(MISSION_REAL, axis=0), axis=1)
    c = MISSION_REAL.mean(axis=0)
    return c + (MISSION_REAL - c) * (target_mean_leg / legs.mean())


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def simulate(wps, period, damping, v_air, roll_lim, claim, tau=0.4, roll_bias=0.0,
             wind=(WIND_FROM_DEG, WIND_MPS), dt=0.02, t_max=200.0, xtrack_i=0.2, laps=1):
    wto = math.radians(wind[0] + 180.0)
    w = np.array([math.cos(wto), math.sin(wto)]) * wind[1]
    # mulai 120 m sebelum WP1 searah leg WP9->WP1 (mirip masuk AUTO di lapangan)
    d0 = wps[0] - wps[-1]
    d0 /= np.linalg.norm(d0)
    pos = wps[0] - d0 * 120.0
    psi = math.atan2(d0[1], d0[0])
    phi = 0.0
    route = [wps[i % len(wps)] for i in range(len(wps) * laps)]
    k = 0
    A = pos.copy()
    B = route[0]
    integ = 0.0
    t = 0.0
    log = []
    claims = []
    while t < t_max and k < len(route):
        vg = np.array([math.cos(psi), math.sin(psi)]) * v_air + w
        gs = max(np.linalg.norm(vg), 1.0)
        L1 = max(0.3183099 * damping * period * gs, 0.0)
        ab = B - A
        lab = np.linalg.norm(ab)
        ab_u = ab / lab
        a_air = pos - A
        xt = cross(a_air, ab_u)                # <0 = kanan garis (konvensi ArduPilot)
        xv = cross(vg, ab_u)
        lv = float(vg @ ab_u)
        nu2 = math.atan2(xv, lv)
        s1 = max(-0.7071, min(0.7071, xt / max(L1, 0.1)))
        nu1 = math.asin(s1)
        if abs(nu1) < math.radians(5.0):
            integ = max(-0.1, min(0.1, integ + nu1 * xtrack_i * dt))
        nu1 += integ
        nu = max(-1.5708, min(1.5708, nu1 + nu2))
        acc = 4.0 * damping ** 2 * gs * gs / L1 * math.sin(nu)
        nav_roll = max(-roll_lim, min(roll_lim, math.degrees(math.atan(acc / G))))
        phi += ((nav_roll + roll_bias) - phi) * dt / tau
        psi += G * math.tan(math.radians(phi)) / v_air * dt
        pos = pos + vg * dt
        dist = np.linalg.norm(B - pos)
        along = float((pos - A) @ ab_u)
        log.append((t, pos[0], pos[1], -xt, k, gs, L1, nav_roll, phi, dist))
        if dist <= claim or along >= lab:
            claims.append((k, dist, "radius" if dist <= claim else "finish"))
            A = B.copy()
            k += 1
            if k < len(route):
                B = route[k]
            integ = 0.0
        t += dt
    L = np.array(log)
    return L, claims


def metrics(L, claims, skip_first_leg=True):
    m = L[:, 4] >= (1 if skip_first_leg else 0)
    cte = L[m, 3]
    # jarak terdekat ke tiap WP
    closest = []
    for k in sorted(set(L[:, 4].astype(int))):
        mk = L[:, 4] == k
        closest.append(L[mk, 9].min())
    closest = np.array(closest[1:] if skip_first_leg else closest)
    n_rad = sum(1 for c in claims[1:] if c[2] == "radius")
    return dict(cte_mean=np.mean(np.abs(cte)), cte_max=np.max(np.abs(cte)),
                p45=100 * np.mean(np.abs(cte) > 45), closest_mean=closest.mean(),
                wp_in_claim=f"{n_rad}/{len(claims) - 1}")


def load_plan(path):
    """File .waypoints (QGC WPL 110) -> (HOME NE, array NE WP1..WPn, alt, param2 radius)."""
    rows = [ln.split("	") for ln in open(path, encoding="utf-8").read().splitlines()[1:] if ln.strip()]
    lat0, lon0 = float(rows[0][8]), float(rows[0][9])
    kx = 111320.0 * math.cos(math.radians(lat0))
    ne = np.array([[(float(r[8]) - lat0) * 111320.0, (float(r[9]) - lon0) * kx] for r in rows])
    return ne[0], ne[1:], np.array([float(r[10]) for r in rows[1:]]), np.array([float(r[5]) for r in rows[1:]])


def run_plan(path, damping, claim):
    home, wps, alts, _ = load_plan(path)
    legs = np.linalg.norm(np.diff(wps, axis=0), axis=1)
    print("plan: %d WP, leg %.0f-%.0f m (rata2 %.0f), alt %.0f m, damping %.3f, klaim %.0f m"
          % (len(wps), legs.min(), legs.max(), legs.mean(), alts.mean(), damping, claim))
    name = os.path.splitext(os.path.basename(path))[0]

    def m(**kw):
        L, c = simulate(wps, damping=damping, claim=claim, **kw)
        return metrics(L, c), L

    # 1) grid periode x airspeed (bank 36, bias roll terukur -6 deg, angin terukur)
    periods = [8, 10, 12, 14, 16, 18, 22]
    speeds = [16, 18, 20, 22, 24]
    print("\n|CTE| rata2 / maks [m]  (bank 36, bias -6, angin %.1f m/s dari %.0f)" % (WIND_MPS, WIND_FROM_DEG))
    print("P / V   " + "".join("%12d" % v for v in speeds))
    grid = np.zeros((len(periods), len(speeds)))
    for i, P in enumerate(periods):
        line = "%5.0f   " % P
        for j, v in enumerate(speeds):
            r, _ = m(period=P, v_air=v, roll_lim=36, roll_bias=-6)
            grid[i, j] = r["cte_mean"]
            line += "%7.1f/%4.0f" % (r["cte_mean"], r["cte_max"])
        print(line)
    # 2) sensitivitas arah angin (8 arah) untuk beberapa setelan
    print("\nPengaruh arah angin 6.3 m/s (8 arah): |CTE| rata2 terburuk / maks terburuk / WP<=klaim terburuk")
    best = []
    for P, v, bank, bias in [(22, 24, 36, -6), (22, 18, 36, -6), (12, 18, 36, -6), (12, 18, 36, 0),
                             (14, 18, 36, 0), (12, 18, 45, 0), (10, 18, 45, 0), (12, 20, 36, 0)]:
        worst = (0, 0, 99)
        for wd in range(0, 360, 45):
            r, _ = m(period=P, v_air=v, roll_lim=bank, roll_bias=bias, wind=(wd, WIND_MPS))
            n_in = int(r["wp_in_claim"].split("/")[0])
            worst = (max(worst[0], r["cte_mean"]), max(worst[1], r["cte_max"]), min(worst[2], n_in))
        tot = r["wp_in_claim"].split("/")[1]
        print("  P%-3d V%-3d bank%-3d bias%-3d -> %5.1f / %5.1f m / %d/%s" % (P, v, bank, bias, *worst, tot))
        best.append((P, v, bank, bias))

    # 3) plot: 4 skenario
    pick = [("Sekarang: P22 V24 bank36 bias-6", dict(period=22, v_air=24, roll_lim=36, roll_bias=-6)),
            ("Turunkan speed: P22 V18 bank36 bias-6", dict(period=22, v_air=18, roll_lim=36, roll_bias=-6)),
            ("Saran: P12 V18 bank36 bias-6", dict(period=12, v_air=18, roll_lim=36, roll_bias=-6)),
            ("Saran + trim roll: P12 V18 bank36 bias0", dict(period=12, v_air=18, roll_lim=36))]
    fig, axes = plt.subplots(1, 5, figsize=(27, 6.5), gridspec_kw=dict(width_ratios=[1, 1, 1, 1, 1.1]))
    for ax, (title, kw) in zip(axes[:4], pick):
        r, L = m(**kw)
        ax.plot(wps[:, 1], wps[:, 0], "k--o", ms=4, lw=1.2)
        ax.plot(home[1], home[0], "k^", ms=9)
        for j, p in enumerate(wps):
            ax.add_patch(plt.Circle((p[1], p[0]), claim, fill=False, ls=":", color="k", lw=0.8))
            ax.annotate("WP%d" % (j + 1), (p[1], p[0]), fontsize=8, xytext=(4, 4), textcoords="offset points")
        sc = ax.scatter(L[:, 2], L[:, 1], c=L[:, 5], s=2, cmap="plasma", vmin=10, vmax=32)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_title("%s\n|CTE| %.1f m, maks %.1f m, WP<=%gm: %s" % (title, r["cte_mean"], r["cte_max"], claim, r["wp_in_claim"]), fontsize=9)
        ax.set_xlabel("timur [m]")
    axes[0].set_ylabel("utara [m]")
    plt.colorbar(sc, ax=axes[3], shrink=0.6, label="groundspeed [m/s]")
    im = axes[4].imshow(grid, origin="lower", cmap="RdYlGn_r", vmin=0, vmax=30, aspect="auto")
    axes[4].set_xticks(range(len(speeds)), [str(v) for v in speeds])
    axes[4].set_yticks(range(len(periods)), [str(p) for p in periods])
    for i in range(len(periods)):
        for j in range(len(speeds)):
            axes[4].text(j, i, "%.0f" % grid[i, j], ha="center", va="center", fontsize=9)
    axes[4].set_xlabel("airspeed [m/s]")
    axes[4].set_ylabel("NAVL1_PERIOD [s]")
    axes[4].set_title("|CTE| rata-rata [m]\n(damping %.3f, bank 36, bias -6)" % damping, fontsize=9)
    plt.colorbar(im, ax=axes[4], shrink=0.8)
    fig.suptitle("Simulasi L1 untuk plan '%s' -- angin %.1f m/s dari %.0f° (estimasi log 15-10)" % (name, WIND_MPS, WIND_FROM_DEG))
    fig.tight_layout()
    out = os.path.join(OUT, "10_simulasi_plan_%s.png" % name.replace(" ", "_").replace("#", ""))
    fig.savefig(out, dpi=115)
    print("plot ->", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", help="file .waypoints Mission Planner")
    ap.add_argument("--damping", type=float, default=0.707)
    ap.add_argument("--claim", type=float, default=15.0)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    if args.plan:
        run_plan(args.plan, args.damping, args.claim)
        return
    real = MISSION_REAL
    m100 = scaled_mission(100.0)
    legs100 = np.linalg.norm(np.diff(m100, axis=0), axis=1)
    print("misi 100 m: leg =", np.round(legs100, 0))

    rows = []

    def run(name, wps, **kw):
        L, c = simulate(wps, **kw)
        r = metrics(L, c)
        r.update(name=name)
        rows.append((r, L, wps))
        print("%-46s |CTE| mean %5.1f  max %5.1f  >45m %4.0f%%  WP terdekat %5.1f m  WP<=klaim %s"
              % (name, r["cte_mean"], r["cte_max"], r["p45"], r["closest_mean"], r["wp_in_claim"]))
        return L

    print("--- kalibrasi ke log 2026-10-15 (nyata: |CTE| 36/38 m, maks 77/75 m) ---")
    run("REAL T1: P22 V20.4 bank36 klaim10 bias-6", real, period=22, damping=0.73, v_air=20.4, roll_lim=36, claim=10, roll_bias=-6)
    run("REAL T2: P22 V24.5 bank36 klaim10 bias-9", real, period=22, damping=0.73, v_air=24.5, roll_lim=36, claim=10, roll_bias=-9)
    print("--- misi leg ~100 m, klaim 15 m ---")
    run("A  P22 V24 bank36 (setelan sekarang)", m100, period=22, damping=0.73, v_air=24, roll_lim=36, claim=15, roll_bias=-6)
    run("B  P22 V18 bank36 (hanya turunkan speed)", m100, period=22, damping=0.73, v_air=18, roll_lim=36, claim=15, roll_bias=-6)
    run("C  P15 V18 bank36", m100, period=15, damping=0.75, v_air=18, roll_lim=36, claim=15, roll_bias=-6)
    run("D  P12 V18 bank36", m100, period=12, damping=0.75, v_air=18, roll_lim=36, claim=15, roll_bias=-6)
    run("E  P12 V18 bank36 bias 0 (trim roll benar)", m100, period=12, damping=0.75, v_air=18, roll_lim=36, claim=15)
    run("F  P12 V18 bank45 bias 0", m100, period=12, damping=0.75, v_air=18, roll_lim=45, claim=15)
    run("G  P10 V18 bank45 bias 0", m100, period=10, damping=0.75, v_air=18, roll_lim=45, claim=15)
    run("H  P12 V24 bank45 bias 0 (speed tetap tinggi)", m100, period=12, damping=0.75, v_air=24, roll_lim=45, claim=15)
    run("I  P12 V18 bank45 bias 0 tanpa angin", m100, period=12, damping=0.75, v_air=18, roll_lim=45, claim=15, wind=(0, 0))

    # plot: kalibrasi + 4 skenario kunci
    pick = [0, 2, 3, 5, 7]
    fig, axes = plt.subplots(1, len(pick), figsize=(5.2 * len(pick), 6))
    for ax, i in zip(axes, pick):
        r, L, wps = rows[i]
        ax.plot(wps[:, 1], wps[:, 0], "k--o", ms=4, lw=1.2)
        for j, p in enumerate(wps):
            ax.add_patch(plt.Circle((p[1], p[0]), 15 if i else 10, fill=False, ls=":", color="k", lw=0.8))
            ax.annotate(str(j + 1), (p[1], p[0]), fontsize=8, xytext=(3, 3), textcoords="offset points")
        ax.plot(L[:, 2], L[:, 1], "r", lw=1.6)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_title("%s\n|CTE| %.0f m, maks %.0f m, WP<=klaim %s" % (r["name"], r["cte_mean"], r["cte_max"], r["wp_in_claim"]), fontsize=8)
        ax.set_xlabel("timur [m]")
    axes[0].set_ylabel("utara [m]")
    fig.suptitle("Simulasi L1 (angin %.1f m/s dari %.0f°) -- garis putus = rencana, merah = jalur sim" % (WIND_MPS, WIND_FROM_DEG))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "09_simulasi_skenario_L1.png"), dpi=120)
    print("plot ->", os.path.join(OUT, "09_simulasi_skenario_L1.png"))


if __name__ == "__main__":
    main()
