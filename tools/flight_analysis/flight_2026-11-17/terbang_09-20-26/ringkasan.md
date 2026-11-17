# Terbang 17 Nov 2026 (09-20-26) — L1 + fuzzy, **R = 80 m**, gangguan CH7 −300 µs (5 loiter)

Sumber: `2026-11-17 09-18-02.tlog` (dipotong t = 141–920 s) dan log SD `2026-11-17_09-17-42.CSV` (dipotong t_boot =
174–951 s, membuang ARM 3 s di darat). Cara membuat ulang: `../buat_ulang.sh`. Ringkasan dan perbandingan: `../ringkasan_hari.md`.

Setelan: **`FUZZY_ENABLE` = 1** (dinyalakan t_boot 62,5 s, sebelum ARM), `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14
(dasar), `FUZZY_MIN/MAX_PER` = 12/16, damping 0,707, `WP_LOITER_RAD` = 80, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40,
`DIST_ENABLE` = 1 (−300 µs, 2 s, 35 pulsa).

| AUTO (folder) | t_boot | Loiter mulai | Fase stabil | Pulsa pertama | Angin | Airspeed | Baterai min |
|---|---|---|---|---|---|---|---|
| 1 (`auto1`) | 194,5–354,3 | 221,3 | 252,8 | **296,4** | 1,8 m/s | 18,0 m/s | 16,05 V |
| 2 (`auto2`) | 355,0–462,5 | 379,3 | 411,1 | 411,5 | 1,3 m/s | 18,1 m/s | 15,90 V |
| 3 (`auto3`) | 463,2–616,5 | 486,3 | 519,5 | **543,5** | 1,2 m/s | 17,1 m/s | 15,61 V |
| 4 (`auto4`) | 617,1–758,0 | 639,1 | 673,9 | 683,8 | 0,9 m/s | 15,3 m/s | 15,45 V |
| 5 (`auto5`) | 758,5–903,0 | 784,8 | 821,3 | 824,0 | 0,1 m/s | 14,9 m/s | 15,26 V |

- **AUTO 1 punya ±1,6 putaran fase stabil tanpa gangguan** (252,8–296,4 s). Putaran pertamanya (252,8–278,9 s) dipakai
  sebagai pembanding tanpa gangguan: galat −9,5 … +6,3 m, period 12,7–16,0 s. AUTO 3 hanya 0,9 putaran bebas (tambahan).
- Baterai baru (16,2 V) dengan throttle stick 1451 µs: pesawat jauh lebih cepat daripada terbang L1 (airspeed 18 m/s
  di AUTO 1–2) dan ketinggian naik-turun (std galat 5–8 m, ±7 m tiap putaran).
- Fuzzy di loiter stabil: Turun 34%, Tetap 40%, Naik 26%, period di batas 12/16 s 15% waktu (`fuzzy/ringkasan_fuzzy.csv`).
- Dengan gangguan: 18 kejadian dari 34 pulsa, puncak 13–31 m, sampai 7,2 s; puncak besar di sisi barat daya–selatan
  dengan `nav_roll` 40–45° dan period 12–12,9 s.
