# Terbang 29 Okt 2026 (d) — L1 + fuzzy + gangguan CH7

Sumber: `2026-10-29 11-39-42.tlog` dan log SD `2026-10-29_11-40-20.CSV`.

Setelan:
- `FUZZY_ENABLE` = 1, `NAVL1_PERIOD` = 14 (periode fuzzy 12–16 s).
- `DIST_ENABLE` = 1: gangguan roll +150 µs selama 1 s tiap CH7 ditekan (12 kali).
- `WP_LOITER_RAD` = 100; radius efektif 111,2 m. `LOIT_BANK_DEG` = 35.
- Pasangannya: terbang c (L1 tetap + gangguan, misi dan gangguan sama, selisih 8 menit).

Cara membuat ulang:
- tlog: `analyze_tlog_l1_auto.py` (root), `analyze_tlog_loiter.py --bagian loiter` (`loiter/`) dan `--bagian full` (`full/`).
- Log SD: `analyze_fuzzy_sdlog.py` (`fuzzy/`) dan `analyze_sdlog_gangguan_kecepatan.py --radius 111.2` (`gangguan/`, `kecepatan/`).

## Kondisi

- Loiter dihentikan pilot (ganti ke FBWA) di t = 269,6 s (log SD), saat pesawat melebar keluar lingkaran di sisi barat.
- Angin sekitar 4,9 m/s dari 212°. Sensor airspeed hidup.

## Hasil loiter (tlog)

| Putaran | Durasi | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|
| 1 (masuk) | 40,0 s | 23,1 m | 44,1 m | ya | 44% |
| 2 | 33,6 s | **4,1 m** | 9,9 m | tidak | 16% |
| 3 (parsial) | 18,7 s | 9,3 m | 25,9 m | ya | 34% |

Putaran 3 keluar toleransi di sisi barat. Penyebabnya batas bank saat angin dari belakang (lihat bawah), lalu pilot keluar ke FBWA.

## Fuzzy (log SD)

| Fase | Durasi | \|e\| rata-rata / RMS / maks | Turun / Tetap / Naik | Periode rata-rata (min–maks) |
|---|---|---|---|---|
| AUTO | 23 s | 6,9 / 14,4 / 44,2 m | 28 / 30 / 43 % | 14,3 s (12–16) |
| Loiter masuk | 46 s | 20,6 / 26,0 / 55,5 m | 68 / 23 / 9 % | 13,1 s (12–16) |
| Loiter stabil | 49 s | 4,4 / 6,0 / 24,1 m | 16 / 68 / 16 % | 14,0 s (12–16) |

Dibanding terbang b (tanpa gangguan, 95% Tetap), di loiter stabil fuzzy lebih sering bergerak (16% Turun, 16% Naik). Rata-ratanya tetap 14 s, jadi fuzzy bereaksi tapi tidak menggeser periode secara permanen.

## Gangguan: lokasi dan respons

Lokasi di `gangguan/01_lokasi_gangguan.png`, respons per event di `gangguan/02_respons_per_gangguan.png`, angka di `gangguan/respons_gangguan.csv`.

| # | t (s) | Fase | Lokasi (timur, utara dari HOME) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|---|
| 1 | 153,1 | AUTO leg ke WP1 | (104, 78) m | 16,6 | – | – | tak terukur |
| 2 | 155,4 | AUTO leg ke WP1 | (80, 52) m | 15,8 | – | – | tak terukur |
| 3 | 162,7 | AUTO leg ke WP1 | (18, −46) m | 15,7 | – | – | tak terukur |
| 4 | 167,6 | AUTO leg WP1→WP2 | (−12, −114) m | 17,6 | 6,1 → 12,3 m | WP2 tercapai dulu | terpotong |
| 5 | 202,2 | loiter masuk | (162, 109) m, sisi timur laut | 22,0 | 9,5 → 16,1 m | > 13 s | **buruk** |
| 6 | 215,3 | loiter masuk | (182, −103) m, sisi tenggara | 20,1 | 13,8 → 12,5 m | langsung | **baik** |
| 7 | 222,3 | loiter stabil | (42, −86) m, sisi barat daya | 22,1 | 3,1 → 4,4 m | 3,4 s | **baik** |
| 8 | 226,2 | loiter stabil | (9, −3) m, sisi barat | 25,1 | 3,5 → 6,5 m | – | tercampur |
| 9 | 234,8 | loiter stabil | (156, 112) m, sisi timur laut | 23,0 | 5,0 → 7,5 m | 1,9 s | **baik** |
| 10 | 250,4 | loiter stabil | (153, −105) m, sisi tenggara | 18,4 | 1,7 → 2,3 m | 4,7 s | **baik** |
| 11 | 255,2 | loiter stabil | (66, −101) m, sisi selatan | 19,7 | 1,0 → 2,4 m | 0,1 s | **baik** |
| 12 | 261,0 | loiter stabil | (5, 10) m, sisi barat | 26,0 | 0,6 → 23,9 m | 8,6 s* | tercampur |

\*Galat turun ke 0 karena loiter dihentikan (FBWA), bukan karena pulih.

Kriteria penilaian sama dengan terbang c (`ringkasan.md` terbang c).

- **Di loiter stabil, 5 dari 5 gangguan yang bisa dinilai hasilnya baik:** naik ≤ 2,5 m dan pulih ≤ 5 s.
- **#8 dan #12 di sisi barat lingkaran** (GS 25–26 m/s, angin dari belakang). Setelah gangguan, `nav_roll` mentok 35° pada 62–72% waktu dan pesawat melebar sampai 134 m dari pusat. Polanya sama persis dengan #5 di terbang c: penyebabnya batas bank, bukan gangguan.
- **Saat masuk loiter**, gangguan #5 menambah 6,6 m dan galat keluar toleransi; #6 tidak berpengaruh.

## Perbandingan dengan terbang c (L1 tetap)

| | Terbang c (L1 tetap) | Terbang d (fuzzy) |
|---|---|---|
| Gangguan di loiter stabil yang bisa dinilai | 2 | 5 |
| Kenaikan galat | −0,1 dan 2,4 m | 0,6–2,5 m |
| Waktu pulih | 2,3–4,0 s | 0,1–4,7 s |
| Gangguan di loiter masuk | +5,8 m (cukup), +6,5 m (buruk) | +6,6 m (buruk), −1,3 m (baik) |
| Loiter stabil (log SD), \|e\| rata-rata / RMS | 6,4 / 9,8 m | 4,4 / 6,0 m |

- **Gangguan +150 µs × 1 s terlalu kecil untuk membedakan keduanya.** L1 tetap dan fuzzy sama-sama meredamnya dalam beberapa meter dan beberapa detik.
- Loiter stabil fuzzy sedikit lebih rapi (RMS 6,0 vs 9,8 m), tapi selisihnya terutama datang dari pelebaran di sisi barat, yaitu batas bank, bukan dari respons gangguan.
- Untuk menguji manfaat fuzzy, gangguan perlu lebih besar (mis. 300 µs atau 2 s) dan diberikan **di luar sisi barat**, supaya `nav_roll` punya ruang untuk koreksi.

## Kecepatan GS / ASP (log SD)

| Fase | GS rata-rata (min–maks) | ASP rata-rata (min–maks) | ASP − GS |
|---|---|---|---|
| FBWA | 18,9 (9,3–28,5) | 24,5 (17,1–33,8) | +5,6 |
| AUTO | 17,2 (14,9–25,3) | 26,2 (24,7–27,9) | +9,1 |
| Loiter masuk | 21,7 (14,6–28,6) | 26,2 (22,8–28,7) | +4,5 |
| Loiter stabil | 21,8 (16,2–28,0) | 25,8 (22,9–27,9) | +4,0 |

- TAS dari fit loiter sekitar **21,6 m/s**, sedangkan sensor 25,8 m/s: **sensor membaca sekitar 20% terlalu tinggi**.
- Terbang ini lebih cepat daripada c (GS loiter 21,8 vs 18,5 m/s). Itu membuat bank yang dibutuhkan di sisi barat makin dekat batas 35°.
- Baterai minimum 14,9 V, normal.
