# Terbang 14 Nov 2026 (12-14-37) — L1 tetap (tanpa fuzzy) + gangguan CH7 +300 µs

Sumber: `2026-11-14 12-14-37.tlog`. **Log SD tidak ada** (lihat Kondisi).

Setelan (PARAM_VALUE di tlog):
- **`FUZZY_ENABLE` = 0**, `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14 tetap, damping 0,707.
- `DIST_ENABLE` = 1: gangguan roll **+300 µs selama 2 s** tiap CH7 ditekan (4 kali, semua di loiter stabil).
- `WP_LOITER_RAD` = 100 (radius tetap, `LOIT_RAD` = 100 di telemetri), `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40.
- Misi: WP1 → WP2 → WP3 `LOITER_UNLIM` CW.
- Pasangannya: terbang 12-22-15 (gangguan −300 µs, misi sama).

Cara membuat ulang:
- tlog: `analyze_tlog_l1_auto.py` (root), `analyze_tlog_loiter.py --bagian loiter` (`loiter/`) dan `--bagian full` (`full/`).
- Pengganti log SD: `tlog_to_sdlog.py <tlog> -o pseudo.CSV`, lalu `analyze_fuzzy_sdlog.py pseudo.CSV` (`fuzzy/`) dan
  `analyze_sdlog_gangguan_kecepatan.py pseudo.CSV --radius 100 --bank 45` (`gangguan/`, `kecepatan/`).

`fuzzy/` hanya menunjukkan aturan yang **akan** aktif bila fuzzy menyala (periode tetap 14 s).

## Kondisi

- **Log SD tidak tercatat**: status `[SD] 999 logs on card, delete old ones` (`SdLogger::openNextFile`).
  `gangguan/`, `kecepatan/`, `fuzzy/` dibuat dari telemetri lewat `tlog_to_sdlog.py`: 10 Hz (bukan 20 Hz),
  waktu gangguan direkonstruksi dari tepi naik CH7 di `RC_CHANNELS` (ketelitian ±0,1 s).
- Angin sekitar **6,4 m/s dari 219°** (fit loiter stabil). Sensor airspeed hidup.
- t = 113–151 s: `ROLL LIMIT` sampai 80° saat belum ARM (pesawat dipegang di darat), bukan kejadian terbang.
- Baterai minimum 14,75 V di loiter, normal.

## AUTO (tlog, toleransi ±45 m)

| Leg | Durasi | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|---|
| masuk → WP1 | 13,1 s | 63,4 m | 89,5 m | leg tanpa garis, belok dari arah datang; `nav_roll` jenuh 56% |
| WP1 → WP2 | 5,3 s | 6,7 m | 8,0 m | baik |
| WP2 → WP3 | 2,4 s | 9,1 m | 13,6 m | langsung masuk loiter |

## Hasil loiter (tlog)

| Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|---|
| 1 (masuk) | 29,3 s | −12,5 m | 17,5 m | 47,7 m | ya | 9% |
| 2 | 27,9 s | −7,2 m | 7,3 m | 11,7 m | ya | 0% |
| 3 (gangguan #1 di akhir) | 28,6 s | −8,0 m | 9,5 m | 24,7 m | ya | 0% |
| 4 (parsial, gangguan #2–#4) | 26,6 s | −18,5 m | 20,7 m | 38,8 m | ya | 6% |

Loiter stabil **sebelum gangguan** (t 256–308 s, 52 s): galat **−6,7 ± 1,5 m**, RMS 6,8 m, maks 11,7 m, 5% keluar ±10 m.
Loiter stabil seluruhnya (t 256–339 s, termasuk gangguan): RMS 13,6 m, maks 38,8 m, 32% keluar ±10 m.

- **Bias tetap ke dalam ≈ −6,7 m** (radius aktual ≈ 93 m). Roll aktual rata-rata 19,0° vs `nav_roll` 17,2°.
  Hukum lingkaran PD tanpa integral tidak menghapus offset tetap; ini yang diuji suku Ki di terbang PID.
- Okt: bias ke **luar** 10–34 m. Setelah radius tetap (commit `0f4c961`), bias ke dalam dan jauh lebih kecil.

## Gangguan: lokasi dan respons

Lokasi di `gangguan/01_lokasi_gangguan.png` dan `05_lokasi_pulsa_stabil.png`, respons per event di
`gangguan/02_respons_per_gangguan.png`, angka di `gangguan/respons_gangguan.csv`.
Galat diukur terhadap **R = 100 m** (setpoint), jadi bias −6,7 m ikut terhitung di `e_sebelum`.

| # | t (s) | Fase | Lokasi (timur, utara dari HOME) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|---|
| 1 | 308,1 | loiter stabil | (16, 1) m, sisi barat daya | 21,1 | 7,9 → 24,6 m | terpotong #2 | **buruk** |
| 2 | 314,2 | loiter stabil | (54, 140) m, sisi utara | 28,9 | 20,9 → 39,3 m | 6,5 s | tercampur |
| 3 | 322,8 | loiter stabil | (177, 49) m, sisi timur | 15,9 | 13,0 → 24,9 m | 6,9 s | **buruk** |
| 4 | 331,6 | loiter stabil | (95, −29) m, sisi selatan | 15,6 | 11,8 → 28,2 m | 7,1 s | **buruk** |

Penilaian sama dengan 29 Okt: **baik** = naik ≤ 5 m dan pulih ≤ 10 s; **cukup** = naik ≤ 10 m dan pulih ≤ 20 s;
keduanya tanpa keluar toleransi. **Tercampur** = galat > 15 m sebelum gangguan atau `nav_roll` jenuh > 40%.

- Arah sesuai teori: +300 µs = roll kanan, di loiter CW mendorong **ke dalam** lingkaran.
- Tiap pulsa menambah galat 12–18 m ke dalam, puncak 3,5–4,5 s setelah pulsa mulai. Roll terukur sampai 54–67°
  (melewati `LOIT_BANK_DEG` 45) karena roll kanan menumpuk di atas bank belok ~19°.
- Respons berosilasi (periode ~9–10 s), lihat #1: puncak kedua −39 m di t+10 s.
- **Jarak antar-pulsa terlalu rapat (6–9 s).** Pulsa berikutnya datang sebelum galat kembali ke bias awal, jadi #2–#4
  berangkat dari galat 12–21 m. Untuk data final: jarak ≥ 20 s (≈ ¾ putaran).
- Dibanding 29 Okt c (+150 µs, 1 s), pulsa di sini 4× lebih besar energinya (2× amplitudo, 2× durasi).

## Kecepatan GS / ASP

| Fase | GS rata-rata (min–maks) | ASP rata-rata (min–maks) | ASP − GS |
|---|---|---|---|
| STABILIZE | 19,2 (8,8–29,9) | 17,6 (8,0–21,7) | −1,7 |
| AUTO | 19,5 (16,7–24,2) | 21,8 (18,3–25,3) | +2,3 |
| Loiter masuk | 19,5 (14,5–30,0) | 20,2 (15,3–23,5) | +0,7 |
| Loiter stabil | 19,9 (13,2–29,9) | 19,7 (15,3–23,9) | −0,2 |

- TAS dari fit loiter sekitar **21,5 m/s**, sensor 19,7 m/s: sensor membaca **≈ 9% terlalu rendah**
  (29 Okt: 24% terlalu tinggi).
- GS tertinggi (≈ 29–30 m/s) di sisi barat laut–utara lingkaran (angin dari belakang); gangguan #2 jatuh di sini.
