# Terbang 16 Nov 2026 (14-07-25) — L1 tetap, **R = 80 m**, gangguan CH7 −300 µs (6 loiter)

Sumber: `2026-11-16 14-04-15.tlog` (dipotong t = 189–1042 s). **Log SD (`2026-11-16_14-06-34.CSV`) belum disalin**, jadi
`fuzzy/`, `auto<N>/` dan grafik `_sd` dibuat dari CSV pengganti `tlog_to_sdlog.py` (10 Hz, waktu = detik sejak awal tlog
potongan, bukan t_boot). Ulangi dengan log SD asli bila kartunya sudah dibaca.

Setelan:
- **`FUZZY_ENABLE` = 0**, `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14 tetap, damping 0,707.
- **`WP_LOITER_RAD` = 80**, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40.
- `DIST_ENABLE` = 1, gangguan **−300 µs selama 2 s**, 47 pulsa.
- Misi WP1 → WP2 → WP3 → WP4 loiter CW. **6 kali masuk AUTO**, semuanya sampai loiter (keluar ke STAB 1–2 s di antaranya).
  Di grafik tlog ditulis "terbang 1–6", di folder gangguan `auto1/`–`auto6/`.

Cara membuat ulang (dari `tools/flight_analysis`, venv `.venv`):
- `flight_2026-11-16/skrip/potong_tlog.py <tlog> F7.tlog 189 1042`, lalu `tlog_to_sdlog.py F7.tlog -o F7.CSV`.
- tlog: `analyze_tlog_l1_auto.py`, `analyze_tlog_loiter.py --bagian loiter|full`.
- CSV: `analyze_fuzzy_sdlog.py F7.CSV -o fuzzy`, `plot_loiter_steady_sdlog.py F7.CSV -o loiter --radius 80`.
- Gangguan per AUTO: `skrip/potong_sdlog.py F7.CSV auto<N>.CSV <t0> <t1>` lalu
  `analyze_sdlog_gangguan_kecepatan.py auto<N>.CSV -o auto<N> --radius 80 --bank 45`. Jendela (detik CSV):
  184,4–371,9 / 371,95–496,7 / 496,75–606,8 / 606,85–716,8 / 716,85–843,3 / 843,35–1030.

`fuzzy/` hanya menunjukkan aturan yang **akan** aktif bila fuzzy menyala (periode tetap 14 s).

## Kondisi

- **Angin kencang: 3,2–6,3 m/s dari 178–210°** (fit loiter stabil per AUTO), di atas batas checklist R80 (≤ 4 m/s)
  kecuali `auto2`. Dari fit tlog per loiter: 5,7–9,7 m/s.
- GS rata-rata di loiter stabil 15–21 m/s. Baterai minimum 14,80 V (AUTO terakhir).
- **Ketinggian mulai tidak stabil di AUTO 5 dan 6** (alt std per putaran 8–14 m, rata-rata sampai 113 m), lihat terbang 14-28-43.

## Hasil loiter (tlog), fase stabil (putaran ke-2 dst, termasuk gangguan)

| AUTO | Durasi stabil | Galat rata-rata | RMS | Maks | Keluar ±10 m | Radius aktual | Angin (fit tlog) |
|---|---|---|---|---|---|---|---|
| 1 | 86 s | +0,8 m | **5,4 m** | 18,1 m | 3% | 80,8 m | 8,2 m/s |
| 2 | 57 s | +3,3 m | 7,0 m | 17,0 m | 19% | 83,3 m | 7,9 m/s |
| 3 | 51 s | +6,6 m | 12,4 m | 28,6 m | 31% | 86,5 m | 5,7 m/s |
| 4 | 57 s | +6,0 m | 8,5 m | 20,2 m | 29% | 85,9 m | 7,7 m/s |
| 5 | 77 s | +7,0 m | 21,3 m | 58,6 m | 48% | 87,0 m | 8,4 m/s |
| 6 | 61 s | +4,2 m | 12,9 m | 32,6 m | 40% | 84,2 m | 9,7 m/s |

Detail per putaran di `loiter/metrik_per_putaran.csv`. Grafik lingkaran stabil versi CSV: `loiter/13_lingkaran_steady_terbang<N>_sd.png`.

- Galat rata-rata cenderung **ke luar** (+3…+7 m) dan makin besar ke AUTO akhir. Angin makin kencang dan pulsa −300 µs
  juga mendorong ke luar.

## Gangguan per masuk AUTO (fase stabil)

| AUTO | Pulsa stabil | baik | cukup | buruk | tercampur | Kenaikan median | `nav_roll` jenuh > 40% |
|---|---|---|---|---|---|---|---|
| 1 | 6 | 4 | 1 | 1 | 0 | 4,3 m | 0 |
| 2 | 6 | 2 | 0 | 4 | 0 | 7,8 m | 0 |
| 3 | 7 | 2 | 0 | 2 | 3 | 3,1 m | 1 |
| 4 | 6 | 0 | 0 | 4 | 2 | 6,7 m | 2 |
| 5 | 8 | 1 | 0 | 2 | 5 | 5,2 m | 3 |
| 6 | 8 | 1 | 0 | 3 | 4 | 5,3 m | 3 |

- AUTO 1 paling bersih (4 dari 6 pulsa baik). Mulai AUTO 3, banyak pulsa **tercampur**: galat sebelum pulsa sudah > 15 m
  atau bank jenuh di sisi angin belakang, jadi responsnya tidak bisa dipisahkan dari efek angin.
- Jarak antar-pulsa ±6–15 s.
