# Terbang 14 Nov 2026 (14-29-40) — L1 tetap (tanpa fuzzy) + gangguan CH7 −300 µs, ada arus naik

Sumber: `2026-11-14 14-29-40.tlog` dan log SD `2026-11-14_14-30-17.CSV`.

Setelan (snapshot ARM di log SD):
- **`fuzzy_enable` = 0**, `nav_pid_enable` = 0, periode L1 14 s tetap, damping 0,707.
- `DIST_ENABLE` = 1: gangguan roll **−300 µs selama 2 s** (11 kali, semua di loiter stabil).
- `WP_LOITER_RAD` = 100, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40. Misi WP1 → WP2 → WP3 `LOITER_UNLIM` CW.

Cara membuat ulang: seperti `../terbang_14-21-31/ringkasan.md`, ditambah `--plan ../misi_loiter_wp3.waypoints`
untuk `analyze_tlog_l1_auto.py` dan `analyze_tlog_loiter.py` (tlog ini tidak memuat daftar WP, hanya HOME).

## Kondisi

- FC reboot di awal tlog (t = 17 s), lalu `[SD] no card/mount fail err=0x04` sekali sebelum log terbuka normal.
  Log SD utuh selama terbang.
- **AUTO pertama dihentikan pilot** (STABILIZE di t SD 463 s), lalu AUTO lagi di t SD 482 s. Pindah ke STABILIZE
  me-reset misi ke WP1, jadi WP1 tercapai dua kali. Ketinggian sempat 162 m di sekitar AUTO pertama.
- **Arus naik (kemungkinan termal) saat loiter.** Dua kali pesawat naik 30–40 m (sampai 139 m di t SD 562–578 dan
  602–610 s) walau `nav_pitch` mentok −10° dan airspeed turun ke 13–14 m/s. Galat ketinggian RMS di loiter 13,1 m
  (terbang lain 1,7–2,8 m). Groundspeed di loiter juga turun (min 8,5 m/s).
- Angin sekitar **6,0 m/s dari 227°**. Baterai minimum 14,6 V, normal.

## AUTO (tlog, toleransi ±45 m)

| Bagian | Leg | Durasi | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|---|---|
| AUTO 1 | masuk → WP1 | 17,1 s | 22,7 m | 27,3 m | leg tanpa garis |
| AUTO 1 | WP1 → WP2 | 6,4 s | 10,8 m | 15,0 m | dihentikan pilot sebelum WP2 |
| AUTO 2 | masuk → WP1 | 16,5 s | 21,1 m | 24,3 m | leg tanpa garis |

## Hasil loiter (tlog)

| Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|---|
| 1 (masuk, gangguan #1) | 31,2 s | −4,5 m | 7,1 m | 16,0 m | ya | 0% |
| 2 (gangguan #2–#5, arus naik) | 38,9 s | +7,8 m | 12,4 m | 25,6 m | ya | 5% |
| 3 (gangguan #6–#9, arus naik) | 39,3 s | +6,4 m | 9,3 m | 20,6 m | ya | 0% |
| 4 (parsial, gangguan #10–#11) | 12,9 s | +9,2 m | 10,6 m | 17,7 m | ya | 0% |

Log SD, loiter stabil (t 540–643 s, 104 s): galat +5,4 ± 9,6 m, RMS 11,0 m, maks 26,1 m, 40% keluar ±10 m.
Pulsa #1 sudah datang 10 s setelah loiter stabil dimulai, jadi tidak ada data loiter stabil tanpa gangguan yang berarti.

- Putaran lebih lama (39 s, terbang lain 28–31 s) dan radius rata-rata melebar ke 106–109 m: groundspeed rendah
  saat arus naik, ditambah 11 pulsa −300 µs yang semuanya mendorong ke luar.
- **Terbang ini sebaiknya tidak dipakai sebagai data utama**: arus naik dan pulsa yang sangat rapat bercampur.

## Gangguan: lokasi dan respons

Angka di `gangguan/respons_gangguan.csv`, gambar `gangguan/01`–`05`. Galat terhadap R = 100 m.

| # | t SD (s) | Lokasi (timur, utara) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|
| 1 | 550,1 | (205, 27) timur | 15,4 | 7,5 → 20,4 m | – | buruk |
| 2 | 561,2 | (133, −66) selatan | 14,6 | 9,9 → 26,1 m | – | buruk |
| 3 | 568,7 | (27, −31) barat daya | 17,6 | 17,0 → 17,8 m | 4,9 s | tercampur |
| 4 | 577,3 | (49, 115) barat laut | 25,5 | 3,3 → 9,4 m | 2,3 s | cukup |
| 5 | 588,0 | (209, 36) timur | 13,7 | 3,6 → 15,0 m | 7,8 s | buruk |
| 6 | 598,7 | (134, −62) selatan | 14,2 | 6,1 → 21,1 m | 7,6 s | buruk |
| 7 | 607,5 | (19, −8) barat daya | 21,6 | 8,1 → 9,5 m | 3,9 s | baik |
| 8 | 614,5 | (64, 125) utara | 24,0 | 2,6 → 8,3 m | 1,9 s | cukup |
| 9 | 622,7 | (202, 79) timur | 13,8 | 1,7 → 13,9 m | 7,8 s | buruk |
| 10 | 635,9 | (152, −54) tenggara | 14,1 | 4,4 → 17,2 m | – | buruk |
| 11 | 640,5 | (87, −72) selatan | 18,4 | 16,9 → 16,9 m | 2,6 s | tercampur |

- Pola menurut sisi: pulsa di **timur/selatan (GS 14–15 m/s, angin dari depan)** menaikkan galat 11–16 m;
  di **barat laut/utara (GS 24–26 m/s)** hanya 6 m. Kebalikan dari sisi barat di terbang lain (bank jenuh), karena
  di sini pada GS rendah pesawat lebih lambat kembali ke lingkaran.
- Jarak antar-pulsa 4,6–13 s.

## Kecepatan GS / ASP (log SD)

| Fase | GS rata-rata (min–maks) | ASP rata-rata (min–maks) | ASP − GS |
|---|---|---|---|
| STABILIZE | 19,7 (8,0–32,4) | 18,1 (7,8–25,7) | −1,6 |
| AUTO | 13,5 (4,2–24,7) | 18,3 (10,5–23,1) | +4,8 |
| Loiter masuk | 19,3 (14,8–27,0) | 19,5 (14,1–22,3) | +0,2 |
| Loiter stabil | 17,4 (8,5–26,1) | 18,4 (13,0–24,0) | +1,0 |

- TAS dari fit loiter sekitar 19,3 m/s, sensor 18,4 m/s (≈ 5% rendah). Fit angin kurang andal karena arus naik
  mengubah kecepatan udara.
