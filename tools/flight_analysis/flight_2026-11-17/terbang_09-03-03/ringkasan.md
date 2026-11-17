# Terbang 17 Nov 2026 (09-03-03) — L1 tetap, **R = 80 m**, gangguan CH7 −300 µs (4 loiter)

Sumber: `2026-11-17 08-48-08.tlog` (dipotong t = 893–1512 s) dan log SD `2026-11-17_08-51-16.CSV` (dipotong t_boot =
718–1336 s, membuang ARM 3 s di darat). Cara membuat ulang: `../buat_ulang.sh`. Ringkasan dan perbandingan: `../ringkasan_hari.md`.

Setelan: `FUZZY_ENABLE` = 0, `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14, damping 0,707, `WP_LOITER_RAD` = 80,
`LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40, `DIST_ENABLE` = 1 (−300 µs, 2 s, 26 pulsa).

| AUTO (folder) | t_boot | Loiter mulai | Fase stabil | Pulsa pertama | Angin | Airspeed | Baterai min |
|---|---|---|---|---|---|---|---|
| 1 (`auto1`) | 736,5–866,8 | 760,6 | 794,0 | 800,5 | 1,4 m/s | 15,9 m/s | 15,02 V |
| 2 (`auto2`) | 867,4–977,7 | 892,6 | 931,0 | 921,7 (sebelum fase stabil) | 0,4 m/s | 13,4 m/s | 14,86 V |
| 3 (`auto3`) | 978,4–1187,4 | 1003,0 | 1046,2 | **1140,5** | 1,1 m/s | 12,7 m/s | 14,42 V |
| 4 (`auto4`) | 1188,1–1295,0 | 1205,8 | 1243,0 | 1235,3 (sebelum fase stabil) | 1,7 m/s | 13,1 m/s | 14,19 V |

- **AUTO 3 punya ±2,8 putaran fase stabil tanpa gangguan** (1046,2–1140,5 s). Putaran pertamanya (1046,2–1080,4 s)
  dipakai sebagai pembanding tanpa gangguan: galat −1,8 … +0,7 m.
- Pesawat makin lambat sepanjang terbang (baterai turun, throttle stick tetap 1447 µs): airspeed 15,9 → 12,7 m/s,
  pitch +5°. Ketinggian stabil (std galat 1,4–1,7 m).
- Dengan gangguan: setiap pulsa menghasilkan overshoot 11–19 m selama 1,4–3,8 s; roll tidak jenuh.
