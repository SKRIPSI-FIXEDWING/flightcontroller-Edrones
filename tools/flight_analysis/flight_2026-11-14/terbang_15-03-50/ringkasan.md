# Terbang 14 Nov 2026 (15-03-50) — L1 tetap (tanpa fuzzy) + gangguan CH7 −300 µs

Sumber: `2026-11-14 15-03-50.tlog` dan log SD `2026-11-14_14-48-03.CSV` (log SD dibuka saat sesi darat 14-43-14,
FC tidak di-reboot sebelum terbang, jadi `time_boot_ms` sudah sekitar 990 s saat ARM).

Setelan (snapshot ARM di log SD):
- **`fuzzy_enable` = 0**, `nav_pid_enable` = 0, periode L1 14 s tetap, damping 0,707.
- `DIST_ENABLE` = 1: gangguan roll **−300 µs selama 2 s** (7 kali: #1 di loiter masuk, #2–#7 di loiter stabil).
- `WP_LOITER_RAD` = 100, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40. Misi WP1 → WP2 → WP3 `LOITER_UNLIM` CW.

Cara membuat ulang: seperti `../terbang_14-21-31/ringkasan.md`, ditambah `--plan ../misi_loiter_wp3.waypoints`
(tlog ini tidak memuat daftar WP).

## Kondisi

- Telemetri dan log SD utuh. Angin lemah, sekitar **3,6 m/s dari 197°**. Baterai minimum 15,7 V, normal.

## AUTO (tlog, toleransi ±45 m)

| Leg | Durasi | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|---|
| masuk → WP1 | 13,7 s | 61,4 m | 85,5 m | leg tanpa garis, belok dari arah datang |
| WP1 → WP2 | 6,6 s | 24,6 m | 32,9 m | overshoot 32,9 m setelah belok di WP1 |
| WP2 → WP3 | 2,0 s | 20,5 m | 23,4 m | langsung masuk loiter |

## Hasil loiter (tlog)

| Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|---|
| 1 (masuk, gangguan #1) | 27,8 s | −11,6 m | 17,4 m | 43,7 m | ya | 14% |
| 2 (gangguan #2–#5) | 30,6 s | +0,4 m | 7,9 m | 25,3 m | ya | 11% |
| 3 (parsial, gangguan #6–#7) | 14,9 s | +2,5 m | 14,0 m | 27,4 m | ya | 8% |

Log SD, loiter stabil (t 1065–1110 s, 45 s): galat +0,8 ± 9,9 m, RMS 9,9 m, maks 26,9 m, 25% keluar ±10 m.
Sebelum pulsa #2 hanya ada 7,3 s loiter stabil (galat +1,3 ± 5,7 m), dan itu masih sisa respons pulsa #1.

## Gangguan: lokasi dan respons

Angka di `gangguan/respons_gangguan.csv`, gambar `gangguan/01`–`05`. Galat terhadap R = 100 m.

| # | t SD (s) | Fase | Lokasi (timur, utara) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|---|
| 1 | 1054,6 | loiter masuk | (130, −43) selatan | 18,0 | 11,7 → 11,4 m | 0 s | baik |
| 2 | 1072,6 | loiter stabil | (170, 113) timur laut | 19,8 | 6,6 → 12,0 m | 2,3 s | buruk (keluar ±10 m) |
| 3 | 1077,4 | loiter stabil | (205, 30) timur | 17,8 | 3,9 → 7,7 m | 1,5 s | **baik** |
| 4 | 1084,1 | loiter stabil | (131, −48) selatan | 18,6 | 4,4 → 7,2 m | 1,0 s | **baik** |
| 5 | 1091,4 | loiter stabil | (12, 19) barat | 25,4 | 3,4 → 26,9 m | – | tercampur (`nav_roll` jenuh 54%) |
| 6 | 1098,8 | loiter stabil | (71, 150) utara | 18,8 | 22,6 → 14,8 m | 0 s | tercampur (sisa #5) |
| 7 | 1107,2 | loiter stabil | (197, 61) timur | 18,0 | 11,4 → 14,1 m | 1,2 s | buruk (sudah > 10 m) |

- Pulsa di timur/selatan: kenaikan 3–5 m, pulih ≤ 2,3 s. #2 dan #7 dinilai "buruk" hanya karena menembus +10 m
  (puncak 12–14 m).
- Pulsa di **sisi barat (#5)** sekali lagi jenuh bank dan galat sampai +27 m. Sisi barat dengan angin dari belakang
  muncul di keempat terbang −300 µs.
- Jarak antar-pulsa 4,8–8,4 s (kecuali #1→#2 18 s).

## Kecepatan GS / ASP (log SD)

| Fase | GS rata-rata (min–maks) | ASP rata-rata (min–maks) | ASP − GS |
|---|---|---|---|
| STABILIZE | 19,0 (8,0–32,1) | 18,4 (6,5–27,7) | −0,5 |
| AUTO | 20,2 (18,0–24,5) | 21,6 (15,9–24,7) | +1,3 |
| Loiter masuk | 20,9 (15,4–29,9) | 20,9 (17,2–24,0) | 0,0 |
| Loiter stabil | 20,6 (17,0–27,7) | 20,1 (14,0–25,0) | −0,5 |

- TAS dari fit loiter sekitar 21,2 m/s, sensor 20,1 m/s (≈ 5% rendah).
