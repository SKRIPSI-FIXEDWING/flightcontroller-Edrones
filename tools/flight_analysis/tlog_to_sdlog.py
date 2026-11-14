#!/usr/bin/env python3
"""Buat CSV berformat log SD (fc::FlightLog v2) dari tlog Mission Planner.

Cadangan bila log SD tidak tercatat (mis. 14 Nov 2026: "[SD] 999 logs on card").
Hasilnya bisa langsung dibaca analyze_sdlog_gangguan_kecepatan.py dan
analyze_fuzzy_sdlog.py tanpa mengubah skrip itu.

Sumbu waktu: jam FC (ATTITUDE/RC_CHANNELS time_boot_ms) digeser ke waktu sejak
awal tlog dengan offset median per sesi boot (seperti fc_time() di
analyze_tlog_l1_auto.py), jadi sama dengan "waktu log" di skrip tlog tetapi
rapat dan monoton (timestamp terima tlog datang bergerombol).
Satu baris per ATTITUDE (~10 Hz, bukan 20 Hz seperti log SD); pesan lain diambil
nilai terakhir yang sudah diterima.

Kolom (sumber):
  mode, armed        HEARTBEAT; LOITER (12) dengan item misi aktif dicatat "AUTO"
                     seperti log SD (loiter adalah item misi)
  wp_next            MISSION_CURRENT.seq (hanya saat AUTO/LOITER, selain itu 0)
  leg_from_wp        1 bila wp_next >= 2 (leg pertama posisi -> WP1 tanpa garis)
  lat_e7, lon_e7     GLOBAL_POSITION_INT
  wp_dist_m          jarak horizontal ke wp_next dihitung dari posisi
  cte_m              NAV_CONTROLLER_OUTPUT.xtrack_error (di loiter = galat radius, + luar)
  roll_deg           ATTITUDE; nav_roll_deg NAV_CONTROLLER_OUTPUT
  alt_fused_m        NAMED_VALUE_FLOAT ALT_FUSED (cadangan: relative_alt)
  gs_mps, course_deg GPS_RAW_INT vel, cog
  airspeed_mps       VFR_HUD.airspeed
  thr_out_us         SERVO_OUTPUT_RAW.servo3_raw
  batt_v             SYS_STATUS.voltage_battery
  de_mps, fuzzy_out  NAMED_VALUE_FLOAT FZ_DE, FZ_DT
  l1_period_s        NAMED_VALUE_FLOAT L1_PER
  l1_dist_m          (1/pi) * NAVL1_DAMPING * L1_PER * gs
  ch7                RC_CHANNELS.chan7_raw

Event gangguan DIST_ON/DIST_OFF direkonstruksi dari CH7 (RC_CHANNELS ~10 Hz,
ketelitian +-0,1 s): tepi naik (> 1700 us) setelah RENDAH (< 1300 us) >= 300 ms,
saat ARM dan mode AUTO/LOITER; DIST_OFF = DIST_ON + DIST_MS. Pulsa yang datang
saat pulsa sebelumnya masih aktif diabaikan.

Pemakaian:
    python tlog_to_sdlog.py "2026-11-14 12-14-37.tlog" -o pseudo_sd.CSV
"""
import argparse
import math

from pymavlink import mavutil

MODE = {0: "MAN", 2: "STAB", 5: "STAB", 10: "AUTO", 4: "GUID", 11: "RTL", 12: "LOIT"}
COLS = ["time_boot_ms", "mode", "armed", "wp_next", "leg_from_wp", "lat_e7", "lon_e7", "wp_dist_m", "cte_m",
        "roll_deg", "nav_roll_deg", "alt_fused_m", "gs_mps", "course_deg", "airspeed_mps", "thr_out_us", "batt_v",
        "de_mps", "fuzzy_out", "l1_period_s", "l1_dist_m", "ch7"]


def dist_m(lat1, lon1, lat2, lon2):
    dy = (lat2 - lat1) * 1e-7 * 111320.0
    dx = (lon2 - lon1) * 1e-7 * 111320.0 * math.cos(math.radians(lat1 * 1e-7))
    return math.hypot(dx, dy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tlog")
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(args.tlog, dialect="ardupilotmega")
    t0 = None
    s = dict(mode="MAN", armed=0, seq=0, lat=0, lon=0, xt=0.0, nroll=0.0, alt_f=None, alt_r=0.0, gs=0.0, cog=0.0,
             asp=0.0, thr=0, bv=0.0, de=0.0, dt=0.0, per=14.0, ch7=0)
    params, wps, rows, ch7 = {}, {}, [], []
    while True:
        m = conn.recv_match()
        if m is None:
            break
        t = m._timestamp
        t0 = t0 or t
        t -= t0
        ty = m.get_type()
        if ty == "HEARTBEAT" and m.type == 1:
            s["mode"] = MODE.get(m.custom_mode, str(m.custom_mode))
            s["armed"] = 1 if m.base_mode & 128 else 0
        elif ty == "MISSION_CURRENT":
            s["seq"] = m.seq
        elif ty == "MISSION_ITEM_INT" and m.mission_type == 0:
            wps[m.seq] = dict(seq=m.seq, lat_e7=m.x, lon_e7=m.y, alt_cm=int(m.z * 100), cmd=m.command)
        elif ty == "PARAM_VALUE":
            params[m.param_id] = m.param_value
        elif ty == "GLOBAL_POSITION_INT":
            s["lat"], s["lon"], s["alt_r"] = m.lat, m.lon, m.relative_alt / 1000.0
        elif ty == "NAV_CONTROLLER_OUTPUT":
            s["xt"], s["nroll"] = m.xtrack_error, m.nav_roll
        elif ty == "GPS_RAW_INT":
            s["gs"], s["cog"] = m.vel / 100.0, m.cog / 100.0
        elif ty == "VFR_HUD":
            s["asp"] = m.airspeed
        elif ty == "SERVO_OUTPUT_RAW":
            s["thr"] = m.servo3_raw
        elif ty == "SYS_STATUS":
            s["bv"] = m.voltage_battery / 1000.0
        elif ty == "RC_CHANNELS":
            s["ch7"] = m.chan7_raw
            ch7.append((m.time_boot_ms / 1000.0, t, m.chan7_raw, s["mode"], s["armed"]))
        elif ty == "NAMED_VALUE_FLOAT":
            k = {"FZ_DE": "de", "FZ_DT": "dt", "L1_PER": "per", "ALT_FUSED": "alt_f"}.get(m.name)
            if k:
                s[k] = m.value
        elif ty == "ATTITUDE":
            nav = s["mode"] in ("AUTO", "LOIT") and s["armed"]
            seq = s["seq"] if nav else 0
            mode = "AUTO" if s["mode"] == "LOIT" and seq > 0 else s["mode"]
            w = wps.get(seq)
            wd = dist_m(s["lat"], s["lon"], w["lat_e7"], w["lon_e7"]) if (w and s["lat"]) else 0.0
            zeta = params.get("NAVL1_DAMPING", 0.707)
            rows.append([(m.time_boot_ms / 1000.0, t), mode, s["armed"], seq, 1 if seq >= 2 else 0, s["lat"], s["lon"],
                         round(wd, 2), round(s["xt"] if nav else 0.0, 2), round(math.degrees(m.roll), 2),
                         round(s["nroll"], 2), round(s["alt_f"] if s["alt_f"] is not None else s["alt_r"], 2),
                         round(s["gs"], 2), round(s["cog"], 1), round(s["asp"], 2), s["thr"], round(s["bv"], 2),
                         round(s["de"] if nav else 0.0, 3), round(s["dt"] if nav else 0.0, 3), round(s["per"], 2),
                         round(zeta * s["per"] * s["gs"] / math.pi if nav else 0.0, 1), s["ch7"]])

    # ---- jam FC -> waktu log (offset median per sesi boot)
    def remap(pairs):
        out, a = [], 0
        for b in range(1, len(pairs) + 1):
            if b == len(pairs) or pairs[b][0] < pairs[b - 1][0] - 1.0:
                offs = sorted(tr - tb for tb, tr in pairs[a:b])
                off = offs[len(offs) // 2]
                out += [tb + off for tb, _ in pairs[a:b]]
                a = b
        return out

    for r, tt in zip(rows, remap([r[0] for r in rows])):
        r[0] = int(round(tt * 1000))
    ch7 = [(tt,) + c[2:] for c, tt in zip(ch7, remap([c[:2] for c in ch7]))]

    # ---- gangguan dari CH7
    dist_us = int(round(params.get("DIST_US", 0)))
    dist_ms = int(round(params.get("DIST_MS", 2000)))
    events, state, low_since, busy_until = [], None, None, -1.0
    for t, v, mode, armed in ch7:
        if v < 1300:
            if state != "L":
                low_since = t
            state = "L"
        elif v > 1700:
            if state == "L" and t - low_since >= 0.3 and armed and mode in ("AUTO", "LOIT") and t >= busy_until:
                events.append(int(round(t * 1000)))
                busy_until = t + dist_ms / 1000.0
            state = "H"

    home = wps.get(0)
    with open(args.out, "w") as f:
        f.write("# tlog_to_sdlog sumber=%s (BUKAN log SD asli; 10 Hz dari telemetri)\n" % args.tlog.split("/")[-1])
        f.write("#SNAPSHOT,BOOT,0\n")
        f.write("# EFFECTIVE roll_limit_deg=%g l1_damping=%g fuzzy_base_period=%g fuzzy_enable=%g dist_us=%d dist_ms=%d\n"
                % (params.get("ROLL_LIMIT_DEG", 40), params.get("NAVL1_DAMPING", 0.707), params.get("NAVL1_PERIOD", 14),
                   params.get("FUZZY_ENABLE", 0), dist_us, dist_ms))
        if home:
            f.write("# HOME,%d,%d,%d\n" % (home["lat_e7"], home["lon_e7"], home["alt_cm"]))
        for seq in sorted(k for k in wps if k > 0):
            w = wps[seq]
            rad = params.get("WP_LOITER_RAD", 0) if w["cmd"] in (17, 18, 19) else 0
            f.write("# WP,%d,%d,%d,%d,%g\n" % (seq, w["lat_e7"], w["lon_e7"], w["alt_cm"], rad))
        for t_ms in events:
            f.write("#EVENT,%d,DIST_ON,%+d\n" % (t_ms, dist_us))
            f.write("#EVENT,%d,DIST_OFF\n" % (t_ms + dist_ms))
        f.write(",".join(COLS) + "\n")
        for r in rows:
            f.write(",".join(str(x) for x in r) + "\n")
    print("%d baris, %d WP, %d gangguan (DIST_US %+d, %d ms) -> %s" % (len(rows), len(wps) - 1, len(events), dist_us,
                                                                       dist_ms, args.out))


if __name__ == "__main__":
    main()
