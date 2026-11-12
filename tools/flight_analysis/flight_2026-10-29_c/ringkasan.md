# Terbang 29 Okt 2026 (c) — L1 tetap (tanpa fuzzy) + gangguan CH7

Sumber: `2026-10-29 11-31-10.tlog` dan log SD `2026-10-29_11-31-11.CSV`.

Setelan:
- **`FUZZY_ENABLE` = 0** (snapshot ARM di log SD: `fuzzy_enable=0`), `NAVL1_PERIOD` = 14 tetap.
- `DIST_ENABLE` = 1: gangguan roll +150 µs selama 1 s tiap CH7 ditekan (7 kali).
- `WP_LOITER_RAD` = 100; radius efektif 110,6 m. `LOIT_BANK_DEG` = 35.
- Pasangannya: terbang d (fuzzy + gangguan, misi dan gangguan sama).

Cara membuat ulang:
- tlog: `analyze_tlog_l1_auto.py` (root), `analyze_tlog_loiter.py --bagian loiter` (`loiter/`) dan `--bagian full` (`full/`).
- Log SD: `analyze_fuzzy_sdlog.py` (`fuzzy/`) dan `analyze_sdlog_gangguan_kecepatan.py --radius 110.6` (`gangguan/`, `kecepatan/`).

`fuzzy/` di terbang ini hanya menunjukkan aturan yang **akan** aktif bila fuzzy menyala. Keluarannya tidak diterapkan (periode tetap 14 s).

## Kondisi

- **Telemetri putus 21 s di akhir loiter** (tlog t = 426–447 s). Analisis loiter dari tlog berhenti di 2,2 putaran. Log SD utuh sampai keluar AUTO.
- Angin sekitar 5,2 m/s dari 190°. Sensor airspeed hidup.

## Hasil loiter (tlog)

| Putaran | Durasi | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|
| 1 (masuk) | 50,9 s | 21,5 m | 66,5 m | ya | 21% |
| 2 | 39,4 s | 12,3 m | 28,5 m | ya | 22% |
| 3 (parsial, terpotong telemetri) | 8,2 s | 3,4 m | 4,1 m | tidak | 0% |

Dari log SD (loiter stabil 63 s): \|e\| rata-rata 6,4 m, RMS 9,8 m, maks 28,3 m.

Putaran 2 keluar toleransi di sisi barat lingkaran. Penyebabnya dibahas di bawah: batas bank, bukan gangguan.

## Gangguan: lokasi dan respons

Lokasi di `gangguan/01_lokasi_gangguan.png`, respons per event di `gangguan/02_respons_per_gangguan.png`, angka di `gangguan/respons_gangguan.csv`.

| # | t (s) | Fase | Lokasi (timur, utara dari HOME) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|---|
| 1 | 316,6 | AUTO leg ke WP1 | (75, 35) m | 15,2 | – | – | tak terukur |
| 2 | 323,6 | AUTO leg ke WP1 | (15, −43) m | 13,3 | – | – | tak terukur |
| 3 | 362,6 | loiter masuk | (17, 29) m, sisi barat | 25,7 | 3,9 → 9,7 m | 6,7 s | cukup |
| 4 | 390,4 | loiter masuk | (124, −108) m, sisi selatan | 16,0 | 4,6 → 11,1 m | > 9 s | **buruk** |
| 5 | 399,4 | loiter stabil | (−3, 15) m, sisi barat | 26,8 | 7,3 → 28,3 m | 10,6 s | tercampur |
| 6 | 417,5 | loiter stabil | (224, 19) m, sisi timur | 15,5 | 2,7 → 5,1 m | 2,3 s | **baik** |
| 7 | 430,9 | loiter stabil | (109, −117) m, sisi selatan | 15,7 | 4,0 → 3,9 m | 4,0 s | **baik** |

Penilaian: **baik** = naik ≤ 5 m dan pulih ≤ 10 s; **cukup** = naik ≤ 10 m dan pulih ≤ 20 s; keduanya tanpa keluar toleransi. **Buruk** = selain itu. **Tercampur** = `nav_roll` jenuh > 40% atau galat sudah > 15 m sebelum gangguan. **Tak terukur** = leg pertama AUTO tanpa garis (`cte_m` = 0).

- **#1 dan #2 tidak bisa dinilai.** Leg dari posisi pesawat ke WP1 tidak punya garis lintasan, jadi tidak ada CTE.
- **#5 bukan efek gangguan.** Sebelum gangguan, pesawat sudah melebar (jarak ke pusat 109 → 121 m) dengan `nav_roll` mentok 35°, di sisi barat saat angin dari belakang (GS 27 m/s). Bank yang dibutuhkan untuk R 111 m pada GS 27 m/s sekitar 34°, jadi tidak ada sisa untuk koreksi. Gangguan +150 µs (roll kanan) di loiter CW justru mendorong ke dalam lingkaran.
- **Di loiter stabil, gangguan hampir tidak terasa** (#6, #7: naik ≤ 2,4 m, pulih 2–4 s).
- Saat masuk loiter, gangguan menambah sekitar 6 m (#3, #4); #4 membuat galat melewati 10 m.

## Kecepatan GS / ASP (log SD)

| Fase | GS rata-rata (min–maks) | ASP rata-rata (min–maks) | ASP − GS |
|---|---|---|---|
| STABILIZE | 19,9 (9,0–28,0) | 25,7 (13,6–35,6) | +5,7 |
| AUTO | 16,7 (13,2–24,9) | 25,6 (23,1–28,7) | +8,9 |
| Loiter masuk | 18,1 (9,7–26,2) | 24,7 (18,3–29,1) | +6,7 |
| Loiter stabil | 18,5 (13,2–26,8) | 23,4 (17,2–28,0) | +4,9 |

- TAS dari fit loiter sekitar **18,8 m/s**, sedangkan sensor 23,4 m/s: **sensor membaca sekitar 24% terlalu tinggi**.
- GS tertinggi (26–27 m/s) selalu di sisi barat lingkaran (angin dari belakang). Di titik inilah `nav_roll` jenuh dan galat loiter terbesar muncul.
- Baterai minimum 15,2 V, normal.
