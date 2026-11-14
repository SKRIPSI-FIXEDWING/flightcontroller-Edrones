# Terbang 14 Nov 2026 (12-22-15) — L1 tetap (tanpa fuzzy) + gangguan CH7 −300 µs

Sumber: `2026-11-14 12-22-15.tlog`. **Log SD tidak ada** (lihat Kondisi).

Setelan (PARAM_VALUE di tlog):
- **`FUZZY_ENABLE` = 0**, `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14 tetap, damping 0,707.
- `DIST_ENABLE` = 1: gangguan roll **−300 µs selama 2 s** tiap CH7 ditekan (5 kali, semua di loiter stabil).
- `WP_LOITER_RAD` = 100 (`LOIT_RAD` = 100 di telemetri), `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40.
- Misi: WP1 → WP2 → WP3 `LOITER_UNLIM` CW (sama dengan 12-14-37).
- Pasangannya: terbang 12-14-37 (gangguan +300 µs).

Cara membuat ulang: sama dengan `../terbang_12-14-37/ringkasan.md`
(`tlog_to_sdlog.py` → `analyze_sdlog_gangguan_kecepatan.py --radius 100 --bank 45`, `analyze_fuzzy_sdlog.py`).

`fuzzy/` hanya menunjukkan aturan yang **akan** aktif bila fuzzy menyala (periode tetap 14 s).

## Kondisi

- **Log SD tidak tercatat**: `[SD] 999 logs on card, delete old ones`. Data SD diganti telemetri 10 Hz
  (`tlog_to_sdlog.py`); waktu gangguan dari tepi naik CH7 (±0,1 s).
- Angin sekitar **6,9 m/s dari 208°** (fit loiter stabil).
- **Baterai**: `BATTERY LOW` sejak t = 209 s, **`BATTERY CRITICAL` 13,54 V (t = 300 s) dan 13,28 V (t = 320 s)**
  di akhir loiter; minimum 12,66 V saat STABILIZE menjelang mendarat. Terbang ini dimulai dengan baterai yang sudah
  dipakai (15,8 V saat di darat).
- Loiter berakhir dengan `AUTO: LOITER selesai` (t = 320 s), lalu STABILIZE.

## AUTO (tlog, toleransi ±45 m)

| Leg | Durasi | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|---|
| masuk → WP1 | 13,3 s | 36,5 m | 48,1 m | leg tanpa garis; `nav_roll` jenuh 19% |
| WP1 → WP2 | 7,2 s | 25,6 m | 35,8 m | overshoot 35,8 m setelah belok di WP1; `nav_roll` jenuh 58% |
| WP2 → WP3 | 2,0 s | 20,9 m | 23,9 m | langsung masuk loiter |

## Hasil loiter (tlog)

| Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|---|
| 1 (masuk) | 28,5 s | −12,8 m | 19,0 m | 52,2 m | ya | 9% |
| 2 (gangguan #1, #2) | 31,2 s | −3,0 m | 5,3 m | 8,6 m | tidak | 6% |
| 3 (parsial, gangguan #3–#5) | 14,7 s | +2,9 m | 6,8 m | 13,3 m | ya | 10% |

Loiter stabil **sebelum gangguan** (t 274–287 s, hanya 12,5 s): galat **−6,6 ± 0,5 m**, RMS 6,6 m, maks 7,3 m, 0% keluar.
Loiter stabil seluruhnya (t 274–320 s, termasuk gangguan): RMS 5,8 m, maks 13,3 m, 5% keluar ±10 m.

- Bias ke dalam **−6,6 m**, sama dengan 12-14-37 (−6,7 m): sifat sistematis, bukan kebetulan satu terbang.
- RMS keseluruhan lebih kecil dari 12-14-37 karena gangguan −300 µs mendorong ke **luar**, berlawanan dengan bias,
  sehingga sebagian saling meniadakan.

## Gangguan: lokasi dan respons

Lokasi di `gangguan/01_lokasi_gangguan.png` dan `05_lokasi_pulsa_stabil.png`, respons per event di
`gangguan/02_respons_per_gangguan.png`, angka di `gangguan/respons_gangguan.csv`. Galat terhadap R = 100 m.

| # | t (s) | Fase | Lokasi (timur, utara dari HOME) | GS | Galat sebelum → puncak | Pulih | Penilaian |
|---|---|---|---|---|---|---|---|
| 1 | 286,8 | loiter stabil | (178, 51) m, sisi timur | 15,7 | 5,5 → 7,2 m | 3,8 s | **baik** |
| 2 | 295,8 | loiter stabil | (91, −30) m, sisi selatan | 17,0 | 4,8 → 7,1 m | 1,0 s | **baik** |
| 3 | 302,5 | loiter stabil | (−11, 38) m, sisi barat | 27,4 | 1,0 → 13,0 m | – | tercampur |
| 4 | 308,0 | loiter stabil | (51, 168) m, sisi utara | 26,2 | 12,3 → 10,7 m | 0,1 s | baik* |
| 5 | 317,8 | loiter stabil | (179, 65) m, sisi timur | 15,5 | 4,7 → 5,2 m | 0,3 s | **baik** |

\* #4 dinilai "baik" oleh aturan otomatis karena galat tidak naik, tetapi pesawat masih di luar toleransi akibat #3
(e sebelum 12,3 m). Secara jujur #4 **tercampur**, tidak dipakai sebagai bukti respons baik.

- Arah sesuai teori: −300 µs = roll kiri, di loiter CW mendorong **ke luar** (galat bertanda berubah dari −6 ke +7…+13 m).
- #1, #2, #5 (sisi timur/selatan, GS 15–17 m/s, angin dari depan): kenaikan ≤ 2,3 m, pulih ≤ 4 s.
- #3 (sisi barat, GS 27 m/s, angin dari belakang): `nav_roll` jenuh 58% di batas bank, galat naik 11,9 m dan keluar
  toleransi. Pola yang sama dengan 29 Okt c #5: di sisi angin belakang tidak ada sisa bank untuk koreksi.
- Jarak antar-pulsa 6–10 s, sama rapatnya dengan 12-14-37.

## Perbandingan dengan 12-14-37 (+300 µs)

| | 12-14-37 (+300, ke dalam) | 12-22-15 (−300, ke luar) |
|---|---|---|
| Galat stabil tanpa gangguan | −6,7 ± 1,5 m | −6,6 ± 0,5 m |
| Kenaikan galat per pulsa (tidak tercampur) | 12–17 m | 0,5–2,3 m |
| Pulsa baik / cukup / buruk / tercampur | 0 / 0 / 3 / 1 | 3 / 0 / 0 / 2 |
| RMS loiter stabil (termasuk gangguan) | 13,6 m | 5,8 m |

- **Respons sangat asimetris.** Roll kanan (+300) menumpuk di atas bank belok ~19° (roll sampai 54–67°) dan pesawat
  memotong ke dalam lingkaran; roll kiri (−300) hanya mendatarkan pesawat sesaat, dan bias −6,6 m memberi "ruang"
  sebelum menembus +10 m.
- Membandingkan metode (fuzzy vs L1 tetap) harus memakai **tanda gangguan yang sama**; data kedua tanda tidak boleh
  dicampur dalam satu rata-rata.

## Kecepatan GS / ASP

| Fase | GS rata-rata (min–maks) | ASP rata-rata (min–maks) | ASP − GS |
|---|---|---|---|
| STABILIZE | 15,9 (8,0–31,3) | 15,6 (2,9–24,9) | −0,3 |
| AUTO | 16,7 (14,2–19,8) | 18,2 (13,9–21,2) | +1,5 |
| Loiter masuk | 20,3 (14,4–28,3) | 19,0 (14,6–22,3) | −1,3 |
| Loiter stabil | 20,0 (14,4–29,5) | 19,7 (14,5–23,9) | −0,3 |

- TAS dari fit loiter sekitar **21,7 m/s**, sensor 19,7 m/s: sensor **≈ 10% terlalu rendah** (12-14-37: 9%).
