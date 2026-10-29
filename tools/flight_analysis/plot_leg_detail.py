#!/usr/bin/env python3
"""Zoom satu segmen AUTO: yaw vs course GPS, nav_bearing L1, roll, xtrack.

Dipakai untuk menjelaskan simpangan besar WP4->WP5 di uji 2026-10-22.
Firmware s.d. 22/10 memakai yaw (heading) x groundspeed sebagai vektor
kecepatan L1, bukan course GPS (sejak 26/10 course GPS). Bila yaw - course
besar (crab angin + error heading), L1 berbasis yaw mengira pesawat sudah
menuju garis padahal sejajar.

Run:
    python plot_leg_detail.py "path/ke/log.tlog" --t0 298 --t1 330 -o out.png
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil


def wrap_disp(a):
    """Sudut kompas ke rentang [-60, 300) agar tidak meloncat 0/360 saat terbang ke utara."""
    return (np.asarray(a) + 60.0) % 360.0 - 60.0


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tlog")
    ap.add_argument("--t0", type=float, required=True, help="waktu log awal [s]")
    ap.add_argument("--t1", type=float, required=True, help="waktu log akhir [s]")
    ap.add_argument("-o", "--out", default="12_detail_segmen.png")
    ap.add_argument("--title", default="Detail segmen AUTO")
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(args.tlog, dialect="ardupilotmega")
    gps, att, nav, hud = [], [], [], []
    while True:
        m = conn.recv_match()
        if m is None:
            break
        t = m.get_type()
        if t == "GPS_RAW_INT":
            gps.append((m._timestamp, m.vel / 100.0, m.cog / 100.0))
        elif t == "ATTITUDE":
            att.append((m._timestamp, np.degrees(m.yaw), np.degrees(m.roll)))
        elif t == "NAV_CONTROLLER_OUTPUT":
            nav.append((m._timestamp, m.nav_roll, m.nav_bearing, m.target_bearing, m.wp_dist, m.xtrack_error))
        elif t == "VFR_HUD":
            hud.append((m._timestamp,))
    gps, att, nav = np.array(gps), np.array(att), np.array(nav)
    t0 = hud[0][0]
    gps[:, 0] -= t0
    att[:, 0] -= t0
    nav[:, 0] -= t0

    sel = lambda a: (a[:, 0] >= args.t0) & (a[:, 0] <= args.t1)
    g, a, n = gps[sel(gps)], att[sel(att)], nav[sel(nav)]
    yaw_g = np.interp(g[:, 0], a[:, 0], np.degrees(np.unwrap(np.radians(a[:, 1])))) % 360

    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    ax[0].plot(a[:, 0], wrap_disp(a[:, 1]), "b", lw=1, label="yaw AHRS")
    ax[0].plot(g[:, 0], wrap_disp(g[:, 2]), "k", lw=1.6, label="course GPS (arah gerak)")
    ax[0].step(n[:, 0], wrap_disp(n[:, 2]), "r", where="post", label="nav_bearing L1")
    ax[0].step(n[:, 0], wrap_disp(n[:, 3]), "g--", where="post", lw=1, label="bearing ke WP aktif")
    ax[0].set_ylabel("derajat")
    ax[0].legend(fontsize=8, ncol=2)
    ax[1].plot(g[:, 0], wrap180(yaw_g - g[:, 2]), "m", lw=1.4, label="yaw - course GPS")
    ax[1].axhline(0, color="0.5", lw=0.8)
    ax[1].set_ylabel("derajat")
    ax[1].legend(fontsize=8)
    ax[2].plot(a[:, 0], a[:, 2], "b", lw=1, label="roll aktual")
    ax[2].step(n[:, 0], n[:, 1], "r", where="post", label="nav_roll L1")
    ax[2].set_ylabel("derajat")
    ax[2].legend(fontsize=8)
    ax[3].step(n[:, 0], np.abs(n[:, 5]), "r", where="post", label="|xtrack| telemetri")
    ax[3].step(n[:, 0], n[:, 4], "k", where="post", lw=1, label="jarak ke WP aktif")
    ax3b = ax[3].twinx()
    ax3b.plot(g[:, 0], g[:, 1], "c", lw=1, label="groundspeed")
    ax3b.set_ylabel("m/s", color="c")
    ax[3].set_ylabel("m")
    ax[3].set_xlabel("waktu log [s]")
    ax[3].legend(fontsize=8, loc="upper left")
    for x in ax:
        x.grid(alpha=0.3)
    fig.suptitle(args.title)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print("selesai ->", args.out)


if __name__ == "__main__":
    main()
