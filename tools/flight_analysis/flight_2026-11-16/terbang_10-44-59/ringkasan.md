# Terbang 16 Nov 2026 (10-44-59) — L1 tetap (tanpa fuzzy) + gangguan CH7 −300 µs

Sumber: `2026-11-16 10-21-27.tlog` (dipotong t = 1400–1615 s) dan log SD `2026-11-16_10-40-05.CSV`
(dipotong t_boot = 311–502 s; terbang pertama di file yang sama, ARM 64 s, tidak dipakai karena `DIST_US` masih +300).
Pengganti terbang 14 Nov 12-14-37 (+300) yang dihapus.

Setelan:
- **`FUZZY_ENABLE` = 0**, `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14 tetap, damping 0,707.
- `DIST_ENABLE` = 1, gangguan **−300 µs selama 2 s** (7 pulsa: 2 di putaran masuk, 5 di loiter stabil).
- `WP_LOITER_RAD` = 100, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40.
- Misi: WP1 → WP2 → WP3 → WP4 loiter CW.

Cara membuat ulang (dari `tools/flight_analysis`, venv `.venv` berisi pymavlink):
- Potong: `flight_2026-11-16/skrip/potong_tlog.py <tlog> F2.tlog 1400 1615` dan
  `flight_2026-11-16/skrip/potong_sdlog.py <CSV> F2.CSV 311 502`.
- tlog: `analyze_tlog_l1_auto.py F2.tlog -o <folder>`, `analyze_tlog_loiter.py F2.tlog -o <folder>/loiter --bagian loiter`
  dan `--bagian full` (`full/`).
- Log SD: `analyze_fuzzy_sdlog.py F2.CSV -o <folder>/fuzzy`,
  `analyze_sdlog_gangguan_kecepatan.py F2.CSV -o <folder> --radius 100 --bank 45`.

`fuzzy/` hanya menunjukkan aturan yang **akan** aktif bila fuzzy menyala (periode tetap 14 s).

## Kondisi

- Angin **3,3 m/s dari 176°** (fit loiter stabil). TAS perkiraan 16,4 m/s, sensor 14,4 m/s (≈ 12% terlalu rendah).
- Baterai minimum 15,32 V, normal.
- **Mode di tlog terlambat:** heartbeat baru menunjukkan AUTO pada t_boot 346 s, padahal log SD mencatat AUTO 332,9 s
  dan WP1 tercapai 350,3 s. Akibatnya analisis tlog melewatkan leg ke WP1 (tabel AUTO di bawah mulai dari WP2).
  Urutan dari log SD: WP1 350,3 s → WP2 353,7 s → WP3 358,8 s → loiter.

## AUTO (tlog, toleransi ±45 m)

| Leg | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|
| (masuk) → WP2 | 15,1 m | 22,0 m | tlog terlambat, lihat Kondisi |
| WP2 → WP3 | 37,8 m | 58,2 m | leg 62 m, lebih pendek dari jari-jari belok; `nav_roll` jenuh 92% |

## Hasil loiter (tlog)

| Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|---|
| 1 (masuk) | 32,7 s | −5,0 m | 14,6 m | 33,5 m | ya | 14% |
| 2 | 41,6 s | +3,0 m | 8,6 m | 21,3 m | ya | 4% |
| 3 (parsial) | 15,7 s | +3,5 m | 8,5 m | 16,9 m | ya | 7% |

Loiter stabil (putaran ke-2 dst, 57 s, termasuk gangguan): galat rata-rata **+3,3 m**, RMS **8,6 m**, maks 21,3 m,
**20%** keluar ±10 m. Tidak ada jendela tanpa gangguan yang cukup panjang untuk mengukur bias.

## Gangguan: lokasi dan respons (log SD)

| # | t_boot (s) | Fase | Lokasi (timur, utara) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|---|
| 1 | 372,5 | masuk | (28, 21) | 23,4 | 10,4 → 21,7 m | 7,0 s | buruk |
| 2 | 398,6 | masuk | (105, −94) | 18,6 | 7,6 → 9,4 m | 3,9 s | baik |
| 3 | 405,6 | stabil | (19, −17) | 19,3 | 4,7 → 21,3 m | – | **buruk** |
| 4 | 413,0 | stabil | (65, 94) | 16,4 | 15,7 → 15,1 m | 3,7 s | tercampur |
| 5 | 422,6 | stabil | (187, 63) | 13,9 | 5,5 → 7,8 m | 1,0 s | baik |
| 6 | 428,2 | stabil | (221, 0) | 13,6 | 4,5 → 7,1 m | 8,7 s | baik |
| 7 | 443,6 | stabil | (51, −69) | 18,8 | 4,5 → 17,0 m | 8,3 s | **buruk** |

- Arah sesuai teori: −300 µs = roll kiri, di loiter CW mendorong **ke luar** lingkaran.
- Pulsa di sisi barat (GS tinggi, angin dari belakang) paling besar efeknya (#3, #7), sisi timur (GS rendah) kecil (#5, #6).
- Jarak antar-pulsa 6–15 s, masih di bawah 15 s yang diminta checklist.
