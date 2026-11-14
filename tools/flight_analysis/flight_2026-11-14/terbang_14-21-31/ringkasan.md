# Terbang 14 Nov 2026 (14-21-31) — L1 tetap (tanpa fuzzy) + gangguan CH7 −300 µs

Sumber: `2026-11-14 14-21-31.tlog` dan log SD `2026-11-14_14-19-44.CSV`.

Setelan (snapshot ARM di log SD):
- **`fuzzy_enable` = 0**, `nav_pid_enable` = 0, periode L1 14 s tetap, damping 0,707.
- `DIST_ENABLE` = 1: gangguan roll **−300 µs selama 2 s** tiap CH7 ditekan (5 kali, semua di loiter stabil).
- `WP_LOITER_RAD` = 100, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40. Misi WP1 → WP2 → WP3 `LOITER_UNLIM` CW.

Cara membuat ulang:
- tlog: `analyze_tlog_l1_auto.py` (root), `analyze_tlog_loiter.py --bagian loiter` (`loiter/`) dan `--bagian full` (`full/`).
- Log SD: `analyze_fuzzy_sdlog.py` (`fuzzy/`) dan `analyze_sdlog_gangguan_kecepatan.py --radius 100 --bank 45`
  (`gangguan/`, `kecepatan/`).

`fuzzy/` hanya menunjukkan aturan yang **akan** aktif bila fuzzy menyala (periode tetap 14 s).

## Kondisi

- Telemetri dan log SD utuh. Waktu di `gangguan/`, `kecepatan/`, `fuzzy/` = `time_boot_ms` log SD; waktu di
  root/`loiter/`/`full/` = waktu tlog (selisih sekitar 165 s).
- Angin sekitar **7,5 m/s dari 191°**. Baterai minimum 15,2 V, normal.

## AUTO (tlog, toleransi ±45 m)

| Leg | Durasi | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|---|
| masuk → WP1 | 12,8 s | 29,3 m | 37,6 m | leg tanpa garis |
| WP1 → WP2 | 8,2 s | 14,8 m | 20,5 m | overshoot 20,5 m setelah belok di WP1 |
| WP2 → WP3 | 2,8 s | 11,6 m | 18,0 m | langsung masuk loiter |

## Hasil loiter (tlog)

| Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|---|
| 1 (masuk) | 29,2 s | −14,6 m | 21,8 m | 60,7 m | ya | 13% |
| 2 (gangguan #1–#3) | 30,0 s | −1,2 m | 7,4 m | 22,4 m | ya | 14% |
| 3 (parsial, gangguan #4–#5) | 29,2 s | +2,9 m | 12,1 m | 30,7 m | ya | 8% |

Log SD, loiter stabil (t 310–369 s, 58 s): galat +0,7 ± 10,0 m, RMS 10,0 m, maks 30,5 m, 18% keluar ±10 m.
Sebelum pulsa pertama hanya ada 10,7 s loiter stabil: galat **−7,2 ± 2,9 m** (cocok dengan bias −6,7 m di 12-14-37,
tetapi jendelanya terlalu pendek untuk angka yang andal).

## Gangguan: lokasi dan respons

Lokasi di `gangguan/01_lokasi_gangguan.png` dan `05_lokasi_pulsa_stabil.png`, respons di
`gangguan/02_respons_per_gangguan.png`, angka di `gangguan/respons_gangguan.csv`. Galat terhadap R = 100 m.

| # | t SD (s) | Fase | Lokasi (timur, utara dari HOME) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|---|
| 1 | 321,1 | loiter stabil | (202, 49) m, sisi timur | 15,2 | 9,4 → 8,8 m | 0 s | baik |
| 2 | 330,0 | loiter stabil | (122, −52) m, sisi selatan | 18,9 | 5,4 → 9,6 m | – | terpotong |
| 3 | 336,5 | loiter stabil | (6, 20) m, sisi barat | 32,2 | 8,0 → 30,5 m | 8,0 s | tercampur (`nav_roll` jenuh 54%) |
| 4 | 347,0 | loiter stabil | (148, 123) m, sisi timur laut | 20,1 | 4,6 → 11,7 m | 2,2 s | buruk (keluar ±10 m) |
| 5 | 360,9 | loiter stabil | (130, −53) m, sisi selatan | 17,7 | 4,8 → 12,0 m | 6,7 s | buruk (keluar ±10 m) |

Penilaian sama dengan 29 Okt (baik: naik ≤ 5 m dan pulih ≤ 10 s; cukup: naik ≤ 10 m dan pulih ≤ 20 s; tanpa keluar
toleransi; tercampur: galat > 15 m sebelumnya atau `nav_roll` jenuh > 40%).

- #4 dan #5 "buruk" karena menembus +10 m (puncak 11,7 dan 12,0 m); kenaikannya sendiri 7 m dan pulih ≤ 7 s.
- #3 di sisi barat dengan angin dari belakang (GS 32 m/s): bank mentok, galat +30 m. Pola sama dengan 12-22-15 #3
  dan 29 Okt c #5.
- Jarak antar-pulsa 6,5–14 s, masih terlalu rapat.

## Kecepatan GS / ASP (log SD)

| Fase | GS rata-rata (min–maks) | ASP rata-rata (min–maks) | ASP − GS |
|---|---|---|---|
| STABILIZE | 17,7 (8,1–33,3) | 17,1 (6,9–25,1) | −0,6 |
| AUTO | 16,0 (10,6–21,6) | 18,7 (14,0–21,4) | +2,7 |
| Loiter masuk | 19,2 (12,9–28,7) | 19,9 (15,1–22,8) | +0,8 |
| Loiter stabil | 20,5 (14,6–33,2) | 21,7 (14,8–25,2) | +1,2 |

- TAS dari fit loiter sekitar **22,8 m/s**, sensor 21,7 m/s: sensor ≈ 5% terlalu rendah.
