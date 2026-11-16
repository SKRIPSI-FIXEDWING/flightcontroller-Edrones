# Rekap terbang 14 Nov 2026

Semua terbang firmware kita hari ini: **L1 tetap** (`FUZZY_ENABLE` = 0, `NAV_PID_ENABLE` = 0, period 14 s),
R = 100 m, `LOIT_BANK_DEG` 45, misi WP1 → WP2 → WP3 loiter CW (`misi_loiter_wp3.waypoints`), gangguan 2 s.
**Belum ada terbang fuzzy maupun PID.** Detail per terbang di `terbang_<jam>/ringkasan.md`.

**16 Nov:** terbang 12-14-37 (gangguan **+300**) dihapus dan tidak dipakai; diganti terbang L1 tetap 16 Nov
(`flight_2026-11-16/T2_L1`). Hasil lamanya masih ada di commit `d750be0` (bias tanpa gangguan −6,7 ± 1,5 m, 52 s).

| Terbang (tlog) | Log SD | Gangguan | Pulsa | Loiter stabil: galat rata-rata / RMS / % keluar ±10 m | Kenaikan per pulsa (median, tidak tercampur) | Catatan |
|---|---|---|---|---|---|---|
| 12-22-15 | – (kartu penuh) | −300 | 5 | −1,1 / 5,8 m / 5% | 1,1 m | baterai CRITICAL di akhir loiter |
| 14-21-31 | 14-19-44 | −300 | 5 | +0,7 / 9,9 m / 16% | 5,7 m | |
| 14-29-40 | 14-30-17 | −300 | 11 | +7,4 / 10,9 m / 39% | 12,2 m | **arus naik** (+30–40 m), AUTO dihentikan pilot sekali; jangan jadi data utama |
| 15-03-50 | 14-48-03 | −300 | 7 | +0,9 / 10,1 m / 24% | 3,3 m | angin lemah 3,6 m/s |

Angka loiter stabil dari tlog (`loiter/ringkasan_steady.csv`, termasuk gangguan); kenaikan dari
`gangguan/respons_gangguan.csv` (12-14-37 dan 12-22-15 lewat `tlog_to_sdlog.py`, 10 Hz).

## Temuan

- **Bias ke dalam ≈ −7 m tanpa gangguan** (12-14-37 [dihapus]: −6,7 ± 1,5 m selama 52 s; 12-22-15 −6,6 m dan 14-21-31 −7,2 m
  dari jendela 11–13 s). Di terbang lain pulsa datang terlalu cepat untuk mengukurnya.
- **Sisi barat lingkaran (angin dari belakang, GS 25–32 m/s) selalu jenuh bank** saat pulsa −300: 12-22-15 #3,
  14-21-31 #3, 15-03-50 #5 (puncak +13…+30 m, `nav_roll` jenuh 54–58%). Ini batas bank, bukan kegagalan L1.
- **Jarak antar-pulsa terlalu rapat di semua terbang (4,6–14 s)**, sehingga banyak respons tercampur atau terpotong.

## Tidak dianalisis

- ArduPlane (CUAV-X7), bukan firmware ini: 10-13-48, 12-55-52, 13-15-55, 13-19-33.
- Sesi darat: 09-45-14, 12-13-24, 13-29-57 (SD 13-28-20), 14-43-14, 15-02-39, 15-07-12; SD 13-06-54 dan 14-09-18.
- 10-38-52: firmware ini tapi setelan lama (period 11, bank 35, gangguan mati), loiter hanya 8 s.
- SD 15-11-30 (tidak ada tlog-nya di zip): ARM hanya 5 s, bukan terbang lengkap.

## Saran untuk terbang berikutnya

1. Beri jarak **≥ 20 s** antar-pulsa dan biarkan **1 putaran penuh tanpa gangguan** setelah masuk loiter
   (untuk mengukur bias).
2. Pakai tanda gangguan yang sama dengan pembanding (data L1 tetap hari ini kebanyakan −300 µs).
3. Kalau memungkinkan, hindari memberi pulsa di sisi barat lingkaran saat angin dari selatan/barat daya
   (atau catat sisi pulsa agar bisa dipisah saat analisis).
