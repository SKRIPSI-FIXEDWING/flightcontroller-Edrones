# Terbang 16 Nov 2026 (11-17-00) — L1 tetap, **R = 80 m**, gangguan CH7 −300 µs

Sumber: `2026-11-16 11-13-33.tlog` (dipotong t = 205–430,3 s) dan log SD `2026-11-16_11-15-54.CSV`
(dipotong t_boot = 82,9–299,1 s, membuang ARM 3 s di darat).

Setelan:
- **`FUZZY_ENABLE` = 0**, `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14 tetap, damping 0,707.
- **`WP_LOITER_RAD` = 80**, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40.
- `DIST_ENABLE` = 1, gangguan **−300 µs selama 2 s**, 6 pulsa semuanya di loiter stabil.
- Misi sama dengan R100: WP1 → WP2 → WP3 → WP4 loiter CW (bukan "loiter saja 8 putaran" seperti rencana grup LK).

Cara membuat ulang: sama dengan `terbang_10-44-59/ringkasan.md`, tetapi `--radius 80`.

`fuzzy/` hanya menunjukkan aturan yang **akan** aktif bila fuzzy menyala (periode tetap 14 s).

## Kondisi

- Angin **6,2 m/s dari 200°**, di atas batas checklist untuk grup R80 (≤ 4 m/s).
- TAS perkiraan 16,9 m/s, sensor 14,6 m/s. Baterai minimum 14,89 V.

## AUTO (tlog, toleransi ±45 m)

| Leg | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|
| masuk → WP1 | 93,1 m | 149,7 m | leg tanpa garis, putaran besar dari arah masuk |
| WP1 → WP2 | 5,4 m | 7,0 m | |
| WP2 → WP3 | 14,6 m | 20,1 m | |
| WP3 → WP4 | 33,9 m | 45,6 m | `nav_roll` jenuh 100% |

## Hasil loiter (tlog)

| Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|---|
| 1 (masuk) | 33,8 s | −8,0 m | 15,6 m | 45,0 m | ya | 10% |
| 2 | 33,8 s | +1,9 m | 6,2 m | 12,7 m | ya | 1% |
| 3 (parsial) | 30,5 s | +4,3 m | 7,1 m | 14,3 m | ya | 4% |

Loiter stabil (64 s, termasuk gangguan): galat rata-rata **+3,1 m** (radius aktual 83,1 m), RMS **6,7 m**, maks 14,3 m,
**19%** keluar ±10 m. Jendela tanpa gangguan hanya ±8 s (bias ≈ −3,6 m, tidak cukup untuk disimpulkan).

## Gangguan: lokasi dan respons (log SD)

| # | t_boot (s) | Lokasi (timur, utara) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|
| 1 | 190,6 | (38, 36) | 24,3 | 3,5 → 11,0 m | 5,9 s | buruk |
| 2 | 196,7 | (136, 67) | 12,3 | 9,2 → 12,4 m | 4,6 s | buruk |
| 3 | 205,5 | (175, −15) | 11,2 | 1,3 → 12,4 m | 5,6 s | buruk |
| 4 | 220,4 | (53, −74) | 15,9 | 2,2 → 13,2 m | 6,3 s | buruk |
| 5 | 227,7 | (30, 36) | 21,7 | 6,4 → 13,8 m | 5,8 s | buruk |
| 6 | 238,4 | (171, 25) | 14,4 | 1,4 → 14,3 m | 5,5 s | buruk |

- Semua pulsa keluar ±10 m dengan puncak 11–14 m (geser ke luar median 13 m). Penilaian "buruk" di sini karena
  **keluar toleransi**, bukan karena tidak pulih: semua pulih dalam 4,6–6,3 s.
- Hanya 1 terbang R80 dan tanpa pembanding fuzzy R80; lihat `../ringkasan_hari.md`.
