# Terbang 16 Nov 2026 (10-51-03) — L1 + fuzzy + gangguan CH7 −300 µs (3 loiter)

Sumber: `2026-11-16 10-21-27.tlog` (dipotong t = 1775,5–2270 s) dan log SD `2026-11-16_10-50-34.CSV`
(dipotong t_boot = 49–532,3 s, membuang ARM 3 s di darat).

Setelan:
- **`FUZZY_ENABLE` = 1**, `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14 (dasar), `FUZZY_MIN/MAX_PER` = 12/16, damping 0,707.
- `DIST_ENABLE` = 1, gangguan **−300 µs selama 2 s** (23 pulsa).
- `WP_LOITER_RAD` = 100, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40. Misi WP1 → WP2 → WP3 → WP4 loiter CW.
- Pilot keluar ke STAB 1–2 s lalu masuk AUTO lagi dua kali, jadi ada **3 segmen AUTO + loiter** (misi di-reset ke WP1 tiap kali).
  Di grafik tlog segmen ini ditulis "terbang 1/2/3".

Cara membuat ulang: sama dengan `terbang_10-44-59/ringkasan.md` (potong tlog 1775,5–2270, log SD 49–532,3 untuk `fuzzy/`),
kecuali gangguan/kecepatan yang dijalankan per segmen (lihat bagian Gangguan).

## Kondisi

- Angin **3,5 m/s dari 203°** (fit loiter stabil). TAS perkiraan 15,7 m/s, sensor 14,1 m/s.
- Baterai minimum 14,81 V.

## AUTO (tlog, toleransi ±45 m)

| Segmen | Leg | CTE RMS | CTE maks | Keterangan |
|---|---|---|---|---|
| 1 | masuk → WP1 | 120,3 m | 178,7 m | leg tanpa garis (CTE diukur ke garis dari titik masuk), lihat catatan |
| 1 | WP1 → WP2 → WP3 → WP4 | 50,3 m | 110,9 m | tlog melewatkan WP2/WP3; log SD: WP2 101,6 s, WP3 106,6 s |
| 2 | masuk → WP1 | 7,7 m | 11,8 m | |
| 2 | WP1 → WP2 | 20,5 m | 26,4 m | |
| 2 | WP2 → WP3 | 15,6 m | 21,9 m | |
| 2 | WP3 → WP4 | 20,6 m | 26,3 m | `nav_roll` jenuh 80% |
| 3 | masuk → WP1 | 39,5 m | 62,8 m | |
| 3 | WP1 → WP2 | 8,8 m | 11,9 m | |
| 3 | WP2 → WP3 | 17,2 m | 27,8 m | `nav_roll` jenuh 86% |

**Kenapa jalur AUTO segmen 1 berbentuk lingkaran besar ke utara** (`11_jalur_rencana_vs_aktual_terbang1.png`):
- AUTO dimasukkan di (147, 75) m saat pesawat terbang ke **utara-timur laut (course 34°) dengan GS 25,7 m/s**,
  sedangkan WP1 ada ±205 m di **belakang** (barat daya).
- L1 langsung meminta bank maksimum **−40°** (belok kiri) selama ±11 s (t_boot 72–83 s). Pada GS 25 m/s, jari-jari belok
  dengan bank 40° ≈ v²/(g·tan 40°) ≈ 76 m, ditambah angin dari belakang (dari 203°), jadi pesawat terdorong sampai
  ±230 m ke utara sebelum menghadap WP1.
- Setelah itu pesawat terbang lurus ke selatan (course 185–190°, roll −10°) sampai WP1. Leg ini **tidak punya garis acuan**
  (leg posisi → WP1), jadi CTE 120 m di tabel bukan galat lintasan yang sebenarnya.
- Leg WP1→WP2 (59 m) dan WP2→WP3 (62 m) **lebih pendek dari jari-jari belok**, jadi pesawat memotong sudut dan jalurnya
  tampak membulat/mengotak di sekitar WP2–WP3. Ini sama di semua terbang hari ini.
- Segmen 2 dan 3 masuk AUTO dari posisi yang lebih dekat ke arah WP1, jadi tidak ada putaran besar.

## Hasil loiter (tlog)

| Segmen | Putaran | Durasi | Galat rata-rata | Galat RMS | Galat maks | Keluar ±10 m |
|---|---|---|---|---|---|---|
| 1 | 1 (masuk) | 33,9 s | −8,0 m | 12,4 m | 33,8 m | ya |
| 1 | 2 | 43,0 s | +0,7 m | 7,3 m | 19,2 m | ya |
| 1 | 3 (parsial) | 35,6 s | +2,1 m | 8,4 m | 19,3 m | ya |
| 2 | 1 (masuk) | 40,4 s | −2,5 m | 10,7 m | 21,1 m | ya |
| 2 | 2 | 40,8 s | +1,3 m | 4,6 m | 12,1 m | ya |
| 2 | 3 (parsial) | 12,2 s | +2,1 m | 6,5 m | 11,1 m | ya |
| 3 | 1 (masuk) | 41,0 s | −3,9 m | 10,3 m | 34,9 m | ya |
| 3 | 2 | 43,8 s | +3,6 m | 6,0 m | 12,9 m | ya |
| 3 | 3 (parsial) | 21,7 s | +2,2 m | 4,4 m | 10,5 m | ya |

Loiter stabil (putaran ke-2 dst, termasuk gangguan):

| Segmen | Durasi | Galat rata-rata | RMS | Maks | Keluar ±10 m |
|---|---|---|---|---|---|
| 1 | 78,2 s | +1,5 m | 8,0 m | 19,3 m | 23% |
| 2 | 52,6 s | +1,5 m | 5,1 m | 12,1 m | 5% |
| 3 | 65,1 s | +3,1 m | 5,5 m | 12,9 m | 11% |

## Fuzzy (log SD, `fuzzy/`)

- Loiter stabil (192 s): periode rata-rata 14,2 s, rentang **12,0–16,0 s**, di batas 14% waktu.
  Keluaran Turun 23%, Tetap 47%, Naik 30%.
- Aturan terbanyak di loiter stabil: Nol/Kecil → Tetap 29%, Neg/Kecil → Naik 20%, Pos/Kecil → Tetap 17%.
- Putaran masuk: lebih banyak Turun (39%), periode rata-rata 13,7 s.

## Gangguan: lokasi dan respons (log SD, per masuk AUTO)

Gangguan dianalisis **per masuk AUTO** (`auto1/`, `auto2/`, `auto3/`, masing-masing berisi `gangguan/` dan `kecepatan/`),
jadi tiap segmen dihitung sebagai satu penerbangan fuzzy + gangguan. Nomor pulsa mulai dari #1 di tiap folder.
Log SD dipotong per segmen (t_boot): auto1 49–224,7, auto2 224,75–345,1, auto3 345,15–532.

Fase stabil saja (pulsa di putaran masuk tidak dinilai untuk skripsi):

| Segmen | Folder | Angin | Pulsa stabil | baik | cukup | buruk | Kenaikan median |
|---|---|---|---|---|---|---|---|
| 1 | `auto1/` | 5,1 m/s dari 200° | 6 (#1–#6) | 0 | 0 | 6 | 10,9 m |
| 2 | `auto2/` | 2,5 m/s dari 196° | 5 (#3–#7) | 3 | 0 | 2 | 4,9 m |
| 3 | `auto3/` | 2,4 m/s dari 223° | 8 (#3–#10) | 2 | 2 | 4 | 8,5 m |

Detail per pulsa di `auto<N>/gangguan/respons_gangguan.csv` dan `auto<N>/gangguan/02_respons_per_gangguan.png`.

- Segmen 1 paling buruk: angin paling kencang (5,1 m/s), dan galat sebelum pulsa sering sudah 7–11 m.
- Jarak antar-pulsa 6–16 s (median ±8 s), banyak respons belum selesai saat pulsa berikutnya datang.
- Grafik lingkaran loiter stabil dari tlog (`loiter/13_..._terbang1.png`) bersegi karena telemetri putus (posisi 2,3 Hz,
  14 celah > 1 s). Pakai versi log SD `loiter/13_lingkaran_steady_terbang<N>_sd.png` (`plot_loiter_steady_sdlog.py`).
