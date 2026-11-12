# Terbang 29 Okt 2026 (b) — L1 + fuzzy, tanpa gangguan

Sumber: `2026-10-29 11-04-03.tlog` dan log SD `2026-10-29_11-18-09.CSV`.

Setelan:
- `FUZZY_ENABLE` = 1, `NAVL1_PERIOD` = 14 (periode fuzzy 12–16 s).
- `DIST_ENABLE` = 0, tidak ada gangguan.
- `WP_LOITER_RAD` = 100; radius efektif 109,9 m. `LOIT_BANK_DEG` = 35.
- Misi sama dengan 26 Okt dan terbang 29 Okt pertama: WP1–3 lalu loiter 3 putaran di WP4.

Cara membuat ulang:
- tlog: `analyze_tlog_l1_auto.py --plan flight_2026-10-29_b/misi_29okt.waypoints` (root), `analyze_tlog_loiter.py --bagian loiter --plan ...` (`loiter/`) dan `--bagian full` (`full/`).
  tlog ini hanya memuat HOME, tanpa item misi WP1–4. Misi diambil dari tlog 29 Okt lain (identik), disimpan di `misi_29okt.waypoints`.
- Log SD: `analyze_fuzzy_sdlog.py` (`fuzzy/`) dan `analyze_sdlog_gangguan_kecepatan.py --radius 109.9` (`kecepatan/`).

## Kondisi

- **Baterai drop berat saat menanjak.** Tercatat BATTERY LOW 14,05 V, lalu CRITICAL 12,54 V di tlog, dan minimum 11,63 V di log SD (fase STABILIZE, throttle penuh). Saat AUTO/loiter tegangan kembali 14,4–14,6 V. Ini tanda baterai lemah atau sel tidak seimbang. **Cek baterai sebelum dipakai lagi.**
- Sensor airspeed **hidup** (berbeda dengan terbang pertama 29 Okt).
- Angin sekitar 4,2 m/s dari 211° (fit lingkaran kecepatan di loiter steady, log SD).

## Hasil loiter (tlog)

| Putaran | Durasi | Galat RMS | Galat maks | Keluar ±10 m | nav_roll jenuh |
|---|---|---|---|---|---|
| 1 (masuk) | 43,9 s | 28,4 m | 56,6 m | ya | 38% |
| 2 | 38,8 s | **4,6 m** | 9,1 m | tidak | 2% |
| 3 (parsial) | 19,3 s | **4,2 m** | 9,9 m | tidak | 0% |

- Konvergen dalam 22,4 s. Galat altitude RMS 5,2 m (target 100 m).
- Loiter stabil rapi dan di dalam toleransi di semua putaran setelah entry.

## Fuzzy (log SD)

| Fase | Durasi | \|e\| rata-rata / RMS / maks | Turun / Tetap / Naik | Periode rata-rata (min–maks) |
|---|---|---|---|---|
| AUTO | 36 s | 9,0 / 18,4 / 51,6 m | 26 / 61 / 13 % | 13,7 s (12–16) |
| Loiter masuk | 49 s | 19,0 / 27,8 / 56,7 m | 48 / 43 / 9 % | 13,3 s (12–16) |
| Loiter stabil | 55 s | 3,7 / 4,5 / 9,1 m | 3 / 95 / 2 % | 14,0 s (12,5–14,4) |

Polanya sama dengan terbang 29 Okt pertama: periode diturunkan saat masuk loiter (R9/R3 → Turun), lalu ditahan 14 s saat loiter stabil (R4/R5 → Tetap, 98%).

## Kecepatan GS / ASP (log SD)

| Fase | GS rata-rata (min–maks) | ASP rata-rata (min–maks) | ASP − GS |
|---|---|---|---|
| STABILIZE | 17,7 (8,5–29,0) | 22,2 (7,9–33,3) | +4,5 |
| AUTO | 18,1 (13,6–25,7) | 25,0 (21,6–26,8) | +7,0 |
| Loiter masuk | 18,6 (14,3–24,3) | 23,7 (20,0–26,5) | +5,1 |
| Loiter stabil | 18,9 (13,3–23,8) | 23,0 (18,3–25,6) | +4,1 |

- Dari fit lingkaran di loiter stabil, TAS sebenarnya sekitar **18,8 m/s**, sedangkan sensor rata-rata 23,0 m/s. **Sensor airspeed membaca sekitar 23% terlalu tinggi.** Di loiter satu putaran penuh, ASP − GS seharusnya mendekati 0; di sini +4 m/s.
- GS naik-turun 13–24 m/s sepanjang lingkaran karena angin.
