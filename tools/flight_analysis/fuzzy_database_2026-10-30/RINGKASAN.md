# Database penyusunan fuzzy: uji L1 konvensional 30/10/2026

Dibuat oleh `tools/flight_analysis/build_fuzzy_database.py` dari lima terbang
L1 periode tetap (fuzzy sudah dihapus dari firmware). Sumber: tlog Mission
Planner (posisi GPS 5 Hz). CTE dihitung ulang terhadap garis WP_prev→WP_next;
leg masuk→WP1 dibuang (mode langsung ke WP).

## Data yang dipakai

| Periode L1 | Tlog | AUTO (s) | L1 distance rata-rata | Groundspeed | Angin |
|---|---|---|---|---|---|
| 11 s | 2026-10-30 10-01-14 | 849–946 | 44 m | 17,9 m/s | 3,1 m/s dari 210° |
| 13 s | 2026-10-30 10-18-10 | 171–263 | 49 m | 19,7 m/s | 1,9 m/s dari 207° |
| 15 s | 2026-10-30 10-28-55 | 211–313 | 51 m | 17,4 m/s | 2,4 m/s dari 201° |
| 17 s | 2026-10-30 10-35-28 | 150–247 | 62 m | 18,4 m/s | 2,1 m/s dari 185° |
| 19 s | 2026-10-30 10-40-54 | 121–215 | 74 m | 19,3 m/s | 3,2 m/s dari 216° |

Setting sama untuk semua: NAVL1_DAMPING 0,707, NAVL1_XTRACK_I 0,2,
WP_RADIUS 15 m, ROLL_LIMIT_DEG 45°, WP_TURN_ANT 1, WP_TURN_MAX 60 m, param
lain default (lihat catatan). Plan `COBA_SKRIPSI_FW_WAYPOINT` (6 WP, leg 350 m
dan 200 m bergantian, belokan ±90°), throttle manual 52–56 %.

## Isi folder

| File | Isi |
|---|---|
| `database_sampel.csv` | 1.894 sampel: `period_s, leg, t_leg_s, e_m, abs_e_m, de_mps` (d\|e\|/dt), `e_rate_mps` (de/dt), `fase` (transisi/tunak), `gs_mps, l1_dist_m, nav_roll_deg, roll_deg` |
| `metrik_transisi.csv` | per leg per periode: e awal leg, overshoot sisi seberang, waktu pulih, zero-crossing, \|e\| tunak, fraksi nav_roll jenuh |
| `ringkasan_per_period.csv` | ringkasan di bawah |
| `01`–`05_*.png` | boxplot \|e\|, bidang fase \|e\| vs d\|e\|/dt, tren metrik, histogram, respons per leg |

Definisi:
- **e** = cross-track error (m), + = kanan garis. **d\|e\|/dt** = laju \|e\|,
  + = menjauhi garis, dihitung dari kecepatan GPS yang diproyeksikan ke normal
  garis (sama dengan `xtrack_vel_mps` firmware; korelasi 0,98 dengan turunan
  numerik).
- **Fase transisi** = dari awal leg sampai \|e\| ≤ 5 m bertahan 2 s; sesudahnya **tunak**.
- **e awal leg** bukan error kendali: dengan WP_TURN_ANT=1 WP berganti
  ~25–40 m sebelum WP, sehingga pesawat langsung berjarak ≈ d·sin(Δψ) dari
  garis baru.
- **Overshoot** = simpangan maksimum di sisi seberang garis setelah pertama kali memotongnya.

## Ringkasan per periode

| Metrik | 11 s | 13 s | 15 s | 17 s | 19 s |
|---|---|---|---|---|---|
| \|e\| rata-rata (m) | 6,7 | 9,0 | **6,8** | 9,7 | 12,0 |
| e RMS (m) | 11,0 | 14,0 | **10,2** | 12,7 | 15,4 |
| \|e\| median (m) | **2,0** | 3,3 | 3,1 | 7,4 | 7,5 |
| \|e\| p95 (m) | 26,2 | 32,8 | **22,0** | 24,9 | 32,0 |
| \|e\| maks (m) | 40,5 | 42,5 | 37,7 | **31,0** | 37,8 |
| \|e\| fase tunak (m) | 1,3 | 1,7 | **1,2** | 3,2 | 5,0 |
| % waktu \|e\| ≤ 5 m | **65** | 59 | 58 | 39 | 24 |
| % waktu \|e\| > 45 m | 0 | 0 | 0 | 0 | 0 |
| % waktu fase transisi | **38** | 46 | 49 | 56 | 58 |
| Waktu pulih rata-rata / maks (s) | 6,4 / 8,1 | **6,2** / 8,7 | 7,3 / 8,3 | 8,2 / 9,3 | 7,9 / 9,1 |
| Leg tidak pulih sebelum WP berikut | 0 | 1 | 1 | 2 | 2 |
| Overshoot seberang rata-rata / maks (m) | 20,9 / 31,5 | 23,1 / 42,5 | **17,9 / 23,9** | 18,1 / 26,0 | 24,7 / 34,7 |
| Zero-crossing total (5 leg) | 16 | 10 | 15 | **7** | 8 |
| \|d\|e\|/dt\| median (m/s) | 1,1 | 1,8 | 1,8 | 2,5 | 3,3 |
| \|d\|e\|/dt\| p90 (m/s) | 16,7 | 16,9 | 16,4 | 15,1 | 14,6 |
| d\|e\|/dt min / maks (m/s) | −22,3 / 19,4 | −23,3 / 21,8 | −20,7 / 17,6 | −20,9 / 18,0 | −22,6 / 20,6 |

## Analisis per periode

**11 s (L1 ≈ 44 m): paling agresif.** Saat tunak paling rapat (median 2,0 m,
65 % waktu dalam ±5 m) dan cepat pulih (6,4 s), tetapi paling berosilasi
(16 zero-crossing) dan nav_roll paling sering jenuh di belokan (hingga 41 %
waktu leg WP4→WP5). Overshoot 21 m rata-rata.

**13 s (L1 ≈ 49 m): cepat, tetapi satu belokan buruk.** Waktu pulih rata-rata
tercepat (6,2 s), tunak 1,7 m. Namun leg WP4→WP5 overshoot 42,5 m (paling dekat
batas 45 m) dan tidak pulih sebelum WP5, sehingga rata-rata dan p95 lebih
buruk dari 15 s. Dengan satu terbang, belokan ini kemungkinan dipengaruhi
hembusan/throttle saat itu; perlu diulang sebelum disimpulkan.

**15 s (L1 ≈ 51 m): kompromi terbaik.** RMS (10,2 m), p95 (22,0 m), overshoot
rata-rata (17,9 m) dan maks (23,9 m), serta \|e\| tunak (1,2 m) terkecil dari
kelima periode, dengan waktu pulih 7,3 s. Kandidat periode dasar ("normal")
fuzzy.

**17 s (L1 ≈ 62 m): halus tetapi lambat.** Respons paling halus (7
zero-crossing, \|e\| maks terkecil 31 m, nav_roll hampir tidak jenuh), tetapi
waktu pulih rata-rata terlama (8,2 s), sisa \|e\| tunak naik ke 3,2 m dan
hanya 39 % waktu dalam ±5 m. Dua leg 200 m tidak sempat pulih.

**19 s (L1 ≈ 74 m): terlalu lunak.** \|e\| rata-rata (12,0 m) dan RMS
(15,4 m) terburuk, \|e\| tunak 5,0 m (koreksi terlalu lemah untuk
menghilangkan offset), hanya 24 % waktu dalam ±5 m, dan overshoot naik lagi
(24,7 m) karena koreksi yang lebar. Dua leg 200 m tidak pulih.

**Pola umum**
1. Periode besar → L1 distance besar → koreksi lebih lunak: \|e\| tunak naik
   (1,2–1,7 m pada 11–15 s menjadi 3,2–5,0 m pada 17–19 s) dan waktu pulih naik
   (≈6,3 s → ≈8 s).
2. Periode kecil → lebih banyak osilasi dan saturasi nav_roll saat transisi.
3. Overshoot paling kecil di 15–17 s; di bawahnya osilasi, di atasnya koreksi lebar.
4. Leg 200 m sering belum pulih sebelum belokan berikutnya (1–2 leg per
   terbang pada 13–19 s). Proposal mensyaratkan leg ≥ 400 m.
5. Tidak ada sampel di atas toleransi 45 m pada semua periode.

## Implikasi untuk himpunan fuzzy

Dari bidang fase (`02_bidang_fase_e_de.png`):
- **Semesta e:** 0–45 m (proposal) mencakup seluruh data (maks 42,5 m).
  Klaster tunak ada di \|e\| < 5–8 m; transisi menyapu 10–40 m. Usulan awal
  batas himpunan: Kecil ≈ 0–5 m, Sedang ≈ 5–20 m, Besar ≈ ≥ 15–20 m.
- **Semesta d\|e\|/dt:** ≈ −23 … +22 m/s (dibatasi groundspeed). Tunak
  \|d\|e\|/dt\| < ≈ 3 m/s; kembali ke garis ≈ −5 … −8 m/s; awal leg/awal belok
  sampai ≈ ±20 m/s. Bila fuzzy memakai \|d\|e\|/dt\| seperti versi lama, tanda
  "menjauh/mendekat" hilang; data menunjukkan tanda ini membedakan dua situasi
  yang butuh aksi berbeda.
- **Arah aturan yang didukung data** (indikatif, satu terbang per periode):
  - e kecil, laju kecil → periode ≤ 15 s (sisa offset terkecil).
  - e besar dan mendekat cepat (d\|e\|/dt negatif besar) → periode lebih besar
    (≈ 17 s) untuk menekan overshoot/osilasi.
  - e besar dan menjauh (d\|e\|/dt positif) → periode lebih kecil (≈ 11–13 s)
    untuk koreksi cepat.
  - Periode dasar ("normal") ≈ 15 s.
- **Sampel awal leg** (e ≈ 25–31 m dan d\|e\|/dt ≈ −15…−22 m/s) adalah geometri
  antisipasi belokan, bukan gangguan. Bila fuzzy diaktifkan di sana, ia akan
  bereaksi terhadap pergantian WP; perlu diputuskan apakah fuzzy dinonaktifkan
  sesaat setelah pergantian leg atau aturan "mendekat cepat" menanganinya.

## Keterbatasan

- Satu terbang per periode; perbedaan kecil (mis. 13 s vs 15 s) bisa karena
  angin/throttle. Ulangi minimal sekali per periode sebelum kesimpulan final.
- Data dari tlog (GPS 5 Hz). Log SD (`LOGxxx.CSV`, 20 Hz, kolom `cte_m`,
  `de_mps`, `l1_period_s`) lebih rapat dan membuktikan periode yang dipakai;
  belum disalin ke laptop.
- Param pada hari ini default (EEPROM ter-reset): IMU trim 0, TECS default,
  ROLL_LIMIT 45°. Uji L1 + fuzzy harus memakai setting yang sama.
- Throttle AUTO manual (52–56 %), memengaruhi kecepatan dan radius belok.
