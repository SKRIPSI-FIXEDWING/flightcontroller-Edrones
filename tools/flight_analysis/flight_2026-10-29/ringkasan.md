# Terbang 29 Okt 2026 — L1 + fuzzy (pertama kali)

Sumber: `2026-10-29 10-27-24.tlog` dan log SD `2026-10-29_10-26-20.CSV`.

Setelan:
- `FUZZY_ENABLE` = 1; `NAVL1_PERIOD` = 14, sehingga periode fuzzy berada di 12–16 s.
- `WP_LOITER_RAD` = 100; radius efektif 109,5 m.
- `LOIT_BANK_DEG` = 35.
- Misi sama dengan 26 Okt: WP1–3 lalu loiter di WP4.

Tidak ada gangguan (`dist_on` selalu 0). Log SD utuh: tidak ada restart, `ctrl_overrun` = 0, `rows_dropped` = 0.

Cara membuat ulang:
- tlog: `analyze_tlog_l1_auto.py` (root), `analyze_tlog_loiter.py --bagian loiter` (`loiter/`) dan `--bagian full` (`full/`).
- Log SD: `analyze_fuzzy_sdlog.py` (`fuzzy/`).

## Kondisi yang membuat run ini tidak setara dengan 26 Okt

- **Sensor airspeed mati sepanjang terbang.** Nilainya 0 di log SD dan di VFR_HUD, sedangkan pada 26 Okt sekitar 21 m/s. Akibatnya:
  - TECS dan radius loiter memakai airspeed target (18 m/s), bukan airspeed terukur.
  - Kecepatan pesawat tidak terpantau. Groundspeed sempat turun ke 8,5 m/s saat melawan angin.
- **Angin sekitar 9,4 m/s**, sedangkan pada 26 Okt 6–7 m/s (P14: 6,3 m/s).
- **AUTO dimulai rendah**, sekitar 44–72 m. Selama putaran pertama loiter pesawat masih naik ke 100 m (galat ketinggian RMS 14,5 m; P14: 2,7 m).
- **Loiter dihentikan setelah 1,7 putaran** (ganti ke FBWA), jadi fase stabilnya pendek.

## Apakah fuzzy bekerja?

**Ya, sesuai rancangan.** Periode yang dipakai L1, dihitung ulang dari `l1_dist_m`, sama dengan 14 + ΔT keluaran fuzzy. Kolom `l1_period_s` di log ini salah untuk loiter; sudah diperbaiki di firmware bf0cba7.

| Fase | Durasi | \|e\| rata-rata / RMS / maks | Turun / Tetap / Naik | Periode rata-rata (min–maks) |
|---|---|---|---|---|
| AUTO (leg lurus) | 32 s | 2,6 / 5,7 / 19,5 m | 15 / 42 / 43 % | 14,6 s (12–16) |
| Loiter putaran 1 (masuk) | 48 s | 29 / 40 / 97 m | 68 / 17 / 15 % | 13,0 s (12–16) |
| Loiter stabil | 55 s | 4,5 / 5,0 / 9,1 m | 4 / 91 / 6 % | 14,0 s (13,3–15,2) |

Aturan yang dominan (`fuzzy/02_aktivasi_aturan.png`):
- Saat masuk loiter: R9 (Positif/Besar → Turun) 25% dan R3 (Negatif/Besar → Turun) 22%.
- Saat loiter stabil: R4 (Nol/Kecil → Tetap) 69% dan R5 (Nol/Sedang → Tetap) 25%.

Jadi periode diturunkan saat galat besar dan dikembalikan ke 14 saat sudah di lintasan.

## Apakah fuzzy berguna? (sementara, satu terbang)

Perbandingan loiter dengan L1 periode tetap pada 26 Okt (galat radius dari tlog):

| Run | Angin | Putaran 1: RMS / maks | Putaran 2: RMS / maks | % keluar ±10 m |
|---|---|---|---|---|
| P12 | 6,1 | 37,5 / 101,5 m | 3,0 / 5,9 m | 20,2 |
| P13 | 7,3 | 45,2 / 90,3 m | 3,1 / 5,5 m | 37,8 |
| **P14 (pembanding)** | 6,3 | 39,7 / 113,0 m | 2,8 / 5,8 m | 26,1 |
| P16 | 8,8 | 25,1 / 73,1 m | 9,5 / 14,6 m | 47,6 |
| **Fuzzy** | **9,4** | **30,3 / 96,4 m** | **4,0 / 7,4 m** | **26,2** |

P15 (pagi, radius 98,8 m) tidak dimasukkan karena radiusnya beda.

- **Putaran 1 (masuk):** galat fuzzy lebih kecil daripada P12–P14, padahal angin lebih kencang. Tapi galat saat masuk sangat bergantung pada arah datang dan ketinggian, dan di run ini pesawat masih naik. Jadi ini belum bisa disebut bukti.
  - Selama masuk, `nav_roll` mentok di batas bank 35° pada 45% waktu. Saat roll sudah mentok, periode yang lebih kecil tidak bisa membuat belokan lebih tajam. Inilah batas utama manfaat fuzzy pada loiter dengan bank 35°, dan sama dengan hasil simulasi.
- **Loiter stabil:** fuzzy praktis sama dengan P14 (91% Tetap). Galatnya sedikit lebih besar (RMS 4,0 vs 2,8 m), wajar dengan angin 9,4 vs 6,3 m/s, dan tetap di dalam ±10 m. Hasil ini sesuai rancangan: aturan sengaja tidak mengubah apa pun bila galat kecil.
- **Kesimpulan sementara:** fuzzy aktif di saat yang benar dan tidak memperburuk loiter stabil. Manfaatnya belum terbukti dari satu terbang dengan angin dan kondisi berbeda. Manfaat terbesar seharusnya terlihat pada pemulihan setelah gangguan, yaitu ketika aturan Turun aktif. Itu perlu run gangguan CH7 berpasangan (L1 tetap vs fuzzy) di hari dan angin yang sama.

## Temuan pada rancangan fuzzy

**Leg pertama AUTO (langsung ke WP1) memicu Naik secara keliru.** Pada leg dari posisi pesawat ke WP1 tidak ada garis lintasan, jadi `cte_m` selalu 0. Namun firmware tetap mengisi d|e|/dt dengan kecepatan menyamping (sampai −18 m/s). Fuzzy membacanya sebagai Negatif/Kecil → Naik, sehingga periode 16 s selama ±9 s pertama AUTO. Belokan awal ke WP1 menjadi lebih landai. **Sudah diperbaiki di firmware:** bila tidak ada garis lintasan (`L1Controller::hasTrack()` = false), fuzzy menahan periode dasar (ΔT = 0), sehingga leg ini identik antara L1 tetap dan L1 + fuzzy.

## Sebelum terbang berikutnya

1. Periksa sensor airspeed: kabel I2C/Wire2, selang pitot, dan pesan saat boot.
2. Flash firmware terbaru: `l1_period_s` benar di loiter, telemetri `L1_PER`/`FZ_DT` di Mission Planner.
3. Masuk AUTO di ketinggian sekitar 100 m supaya putaran pertama tidak bercampur dengan menanjak.
4. Biarkan loiter selesai (≥ 3 putaran), lalu lakukan run gangguan berpasangan pada hari yang sama.
