# Terbang 16 Nov 2026 (11-03-36) — L1 + fuzzy, 2 loiter dengan gangguan −300 µs + 1 loiter tanpa gangguan

Sumber: `2026-11-16 11-02-20.tlog` (dipotong t = 75–584,3 s) dan log SD `2026-11-16_11-02-12.CSV`
(dipotong t_boot = 101–594 s, membuang ARM 3 s di darat).

Setelan:
- **`FUZZY_ENABLE` = 1**, `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14 (dasar), `FUZZY_MIN/MAX_PER` = 12/16, damping 0,707.
- `DIST_ENABLE` = 1, gangguan **−300 µs selama 2 s**: 9 pulsa di segmen 1, 9 di segmen 2, **0 di segmen 3**.
- `WP_LOITER_RAD` = 100, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40. Misi WP1 → WP2 → WP3 → WP4 loiter CW.
- 3 segmen AUTO + loiter (keluar ke STAB 1–2 s di antaranya). Di grafik tlog ditulis "terbang 1/2/3".
- **Segmen 3 = data tambahan L1 + fuzzy tanpa gangguan.**

Cara membuat ulang: sama dengan `terbang_10-44-59/ringkasan.md` (potong tlog 75–584,3, log SD 101–594 untuk `fuzzy/`),
kecuali gangguan/kecepatan yang dijalankan per segmen (lihat bagian Gangguan).

## Kondisi

- Angin **4,1 m/s dari 184°** (fit loiter stabil). TAS perkiraan 17,8 m/s, sensor 16,4 m/s.
- Baterai minimum 15,60 V.

## AUTO (tlog, toleransi ±45 m)

| Segmen | Leg | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|---|
| 1 | masuk → WP1 | 102,9 m | 154,6 m | leg tanpa garis, putaran besar seperti 10-51-03 segmen 1 |
| 1 | WP1 → WP2 | 8,3 m | 10,7 m | |
| 1 | WP2 → WP3 | 34,3 m | 46,7 m | `nav_roll` jenuh 100% |
| 1 | WP3 → WP4 | 55,1 m | 67,0 m | `nav_roll` jenuh 100% |
| 2 | masuk → WP1 | 12,8 m | 25,1 m | |
| 2 | WP1 → WP2 | 10,1 m | 13,9 m | |
| 2 | WP2 → WP3 | 21,3 m | 28,6 m | |
| 2 | WP3 → WP4 | 22,5 m | 28,4 m | `nav_roll` jenuh 80% |
| 3 | masuk → WP1 | 10,8 m | 18,7 m | |
| 3 | WP1 → WP2 | 9,5 m | 11,9 m | |
| 3 | WP2 → WP3 | 17,4 m | 24,5 m | |
| 3 | WP3 → WP4 | 25,0 m | 31,8 m | `nav_roll` jenuh 100% |

Leg WP1→WP2→WP3 hanya 59–62 m, lebih pendek dari jari-jari belok, jadi galat di leg ini terutama dari belokan.

## Hasil loiter (tlog)

| Segmen | Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m |
|---|---|---|---|---|---|---|
| 1 | 1 (masuk) | 38,8 s | −6,4 m | 17,8 m | 66,3 m | ya |
| 1 | 2 | 34,5 s | −1,3 m | 4,2 m | 9,8 m | tidak |
| 1 | 3 | 38,0 s | +2,5 m | 6,7 m | 18,3 m | ya |
| 1 | 4 (parsial) | 18,1 s | +3,0 m | 5,4 m | 13,2 m | ya |
| 2 | 1 (masuk) | 40,8 s | −2,1 m | 8,3 m | 23,8 m | ya |
| 2 | 2 | 40,2 s | +0,3 m | 6,2 m | 16,8 m | ya |
| 2 | 3 (parsial) | 17,1 s | +4,8 m | 7,6 m | 17,5 m | ya |
| 3 | 1 (masuk) | 38,2 s | −7,3 m | 12,8 m | 42,5 m | ya |
| 3 | 2 | 36,2 s | −4,5 m | 6,1 m | 9,4 m | **tidak** |
| 3 | 3 (parsial) | 15,1 s | −1,1 m | 3,7 m | 8,0 m | **tidak** |

Loiter stabil (putaran ke-2 dst):

| Segmen | Gangguan | Durasi | Galat rata-rata | RMS | Maks | Keluar ±10 m |
|---|---|---|---|---|---|---|
| 1 | 9 pulsa | 90,4 s | +1,2 m | 5,6 m | 18,3 m | 5% |
| 2 | 6 pulsa | 57,0 s | +1,7 m | 6,7 m | 17,5 m | 11% |
| **3** | **tanpa** | 51,2 s | **−3,5 m** | **5,5 m** | **9,4 m** | **0%** |

- Segmen 3 (tanpa gangguan): bias ke dalam **−3,5 m** (radius aktual rata-rata 96,5 m), tidak pernah keluar ±10 m.
  Bandingkan L1 tetap 14 Nov 12-14-37 sebelum gangguan: −6,7 ± 1,5 m (lain hari, angin 6,4 m/s).

## Fuzzy (log SD, `fuzzy/`)

- Loiter stabil seluruhnya (192 s, termasuk segmen 3): periode rata-rata 14,2 s, rentang **12,0–16,0 s**, di batas 10% waktu.
  Keluaran Turun 16%, Tetap 56%, Naik 27%.
- Segmen 3 saja (tanpa gangguan): periode **13,4–15,6 s**, tidak pernah di batas; Tetap 83%, Naik 15%, Turun 2%.

## Gangguan: lokasi dan respons (log SD, per masuk AUTO)

Gangguan dianalisis **per masuk AUTO** (`auto1/`, `auto2/`, `auto3/`, masing-masing berisi `gangguan/` dan `kecepatan/`;
`auto3/` hanya `kecepatan/` karena tanpa gangguan). Nomor pulsa mulai dari #1 di tiap folder.
Log SD dipotong per segmen (t_boot): auto1 101–289,9, auto2 289,95–421,2, auto3 421,25–594.

Fase stabil saja:

| Segmen | Folder | Angin | Pulsa stabil | baik | cukup | buruk | terpotong | Kenaikan median |
|---|---|---|---|---|---|---|---|---|
| 1 | `auto1/` | 4,3 m/s dari 193° | 9 (#1–#9) | 2 | 3 | 3 | 1 | 5,5 m |
| 2 | `auto2/` | 3,7 m/s dari 177° | 6 (#4–#9) | 2 | 2 | 2 | 0 | 7,5 m |
| 3 | `auto3/` | 4,3 m/s dari 174° | – | | | | | tanpa gangguan |

- Semua pulsa **buruk** di fase stabil (auto1 #4, #5, #9; auto2 #4, #9) jatuh di sisi **barat, barat laut, dan utara**
  lingkaran (pusat di (108, 42) m) dengan GS 18–22 m/s (angin dari belakang), sama seperti temuan 14 Nov.
  Pulsa di sisi timur/tenggara (GS 12–15 m/s) umumnya baik/cukup.
- Jarak antar-pulsa 7–17 s (median ±8 s).
