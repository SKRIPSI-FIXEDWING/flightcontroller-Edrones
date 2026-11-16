# Terbang 16 Nov 2026 (14-28-43) — L1 + fuzzy, **R = 80 m**, gangguan CH7 −300 µs

Sumber: `2026-11-16 14-27-05.tlog` (dipotong t = 97–690 s). **Log SD (`2026-11-16_14-26-33.CSV`) belum disalin**, jadi
`fuzzy/`, `auto<N>/` dan grafik `_sd` dibuat dari CSV pengganti `tlog_to_sdlog.py` (10 Hz, waktu = detik sejak awal tlog
potongan, bukan t_boot). Ulangi dengan log SD asli bila kartunya sudah dibaca.

Setelan:
- **`FUZZY_ENABLE` = 1** (dinyalakan t = 36 s, sebelum ARM), `NAV_PID_ENABLE` = 0, `NAVL1_PERIOD` = 14 (dasar),
  `FUZZY_MIN/MAX_PER` = 12/16, damping 0,707.
- **`WP_LOITER_RAD` = 80**, `LOIT_BANK_DEG` = 45, `ROLL_LIMIT_DEG` = 40.
- `DIST_ENABLE` = 1, gangguan **−300 µs selama 2 s**, 16 pulsa (semua di AUTO 1–3).
- **5 kali masuk AUTO**: AUTO 1–3 sampai loiter, AUTO 4–5 **tidak** (pilot ambil alih saat ketinggian naik, lihat bawah).

Cara membuat ulang: sama dengan `terbang_14-07-25/ringkasan.md`, potong tlog 97–690 s. Jendela gangguan (detik CSV):
auto1 91–285,3, auto2 285,35–383,6, auto3 383,65–521,5.

## Kondisi

- **Angin kencang: 6,7–7,2 m/s dari 192–203°** (fit per AUTO), jauh di atas batas checklist R80 (≤ 4 m/s).
  GS rata-rata loiter stabil 16–25 m/s. Baterai minimum 15,07 V.
- **Ketinggian tidak terkendali mulai AUTO 2**: loiter AUTO 3 naik-turun antara ±86 dan ±146 m tiap putaran
  (alt std ≈ 20 m). AUTO 4 naik sampai ±171 m, AUTO 5 sampai ±193 m, lalu pilot pindah ke STAB.
  - Naiknya **nyata**: ketinggian GPS dan baro selisih < 3 m.
  - Saat naik, `nav_pitch` **mentok di −10°** (`pitch_limit_min_deg`, `include/navigation/Navigation.h`) dan pitch IMU
    −8…−12° (mengikuti perintah), tapi pesawat tetap naik **3–5 m/s** dengan airspeed 14–22 m/s. Di sisi angin belakang
    turun cepat (−10…−13 m/s), lalu TECS menarik hidung sampai +20…+28°. Ketinggian berosilasi ±30 m tiap putaran.
  - **Throttle di AUTO adalah stick pilot**, bukan TECS (`FC_AUTO_THROTTLE_ENABLE` = 0 di `include/FC_Config.h`).
    Di terbang ini stick lebih tinggi daripada terbang pagi: median 1512 µs, **1600–1740 µs saat AUTO 4–5**, sedangkan
    terbang pagi median 1432–1447 µs. Dengan pitch hanya boleh turun −10°, kelebihan tenaga ini tidak bisa dibuang.
  - Hidung −8° tetapi naik 3–5 m/s pada airspeed ±15 m/s berarti ada **udara naik ±5 m/s** (thermal siang/angin ±7 m/s),
    **atau** estimasi pitch BNO055 meleset saat belok terus-menerus. Dari tlog keduanya tidak bisa dipisahkan; IMU tidak
    membeku (tidak ada sampel attitude identik berturut-turut). Cek `imu_cal` dan pitch di log SD asli bila sudah disalin.
    14 Nov 14-29-40 (jam yang sama) juga mencatat arus naik +30–40 m.
  - Ada celah telemetri ±10 s di t ≈ 429–438 s (nilai tlog membeku).

## Hasil loiter (tlog), fase stabil (putaran ke-2 dst, termasuk gangguan)

| AUTO | Durasi stabil | Galat rata-rata | RMS | Maks | Keluar ±10 m | Radius aktual | Alt std (per putaran) |
|---|---|---|---|---|---|---|---|
| 1 | 94 s | +2,1 m | **9,6 m** | 24,9 m | 21% | 82,1 m | 3–6 m |
| 2 | 16 s | +8,4 m | 19,4 m | 39,7 m | 34% | 88,4 m | 8–9 m |
| 3 | 41 s | +13,0 m | 22,5 m | 70,4 m | 72% | 93,0 m | ±20 m |

Grafik lingkaran stabil versi CSV: `loiter/13_lingkaran_steady_terbang<N>_sd.png`.

- **Hanya AUTO 1 yang layak dipakai** sebagai data fuzzy R80. AUTO 2 loiter stabilnya cuma 16 s, AUTO 3 tercampur
  dengan naik-turun ketinggian ±60 m.

## Fuzzy (CSV, `fuzzy/`)

- Loiter stabil seluruhnya (137 s): periode rata-rata 13,7 s, rentang 12,0–16,0 s, **di batas 23% waktu**.
  Keluaran Turun 45%, Tetap 33%, Naik 22%. Lebih sering Turun daripada R100 (16–23%), sesuai galat yang lebih besar.

## Gangguan per masuk AUTO (fase stabil)

| AUTO | Pulsa stabil | baik | cukup | buruk | tercampur | Kenaikan median |
|---|---|---|---|---|---|---|
| 1 | 7 | 1 | 1 | 3 | 2 | 8,7 m |
| 2 | 1 | 0 | 0 | 0 | 1 | – |
| 3 | 5 | 0 | 0 | 0 | 5 | – |

AUTO 2 juga punya 3 pulsa di putaran masuk (tidak dinilai). Detail di `auto<N>/gangguan/respons_gangguan.csv`.
