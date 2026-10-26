# Altitude fusion (baro + akselerometer + GPS Here4)

Estimator altitude yang tahan terhadap gangguan barometer (cahaya, aliran
udara). `fc::AltitudeFusion` ([AltitudeFusion.h](../include/estimation/AltitudeFusion.h),
[AltitudeFusion.cpp](../src/estimation/AltitudeFusion.cpp)); dipakai semua
konsumen altitude (TECS, Navigation, MAVLink, log) lewat
`Barometer::applyFusedEstimate()` — tidak ada konsumen yang diubah.

Saklar: `FC_ALTITUDE_FUSION_ENABLE` di `FC_Config.h` (default `1`).

## Masalah

Log `2026-10-13 13-33-46.tlog`: tekanan MS5607 bergeser hingga +2,5 hPa
(≈ −20 m) mengikuti arah hadap FC, **suhu tetap**, kembali normal saat diputar
balik. Nilai itu langsung dari sensor (`SCALED_PRESSURE`), jadi bukan salah hitung
— filter Kalman baro-saja hanya meneruskannya.

## Cara kerja

State `x = [tinggi, laju naik, bias akselerometer]`.

1. **Prediksi** dari akselerasi vertikal BNO055 (`verticalAccelUpMss()`), 200 Hz.
   Dengan ini filter "tahu" apakah FC benar-benar bergerak naik/turun.
2. **Update baro** (`raw_altitude_m`, 1σ ≈ 0,8 m) dengan *gating*: sampel yang
   menyimpang > 3σ dari prediksi inersia ditolak. Gangguan cahaya muncul sebagai
   loncatan 10–25 m dalam 1–5 detik tanpa akselerasi yang cocok.
3. **Histeresis**: setelah penolakan, baro harus konsisten 1,5 detik sebelum
   dipercaya lagi (gangguan nyata berayun di tepi gerbang). Sampel marginal
   (> 1,5σ) diberi bobot Huber.
4. **GPS Here4** (jika 3D fix, ≥ 6 satelit, HDOP < 3): tinggi MSL dan **kecepatan
   vertikal** menjadi pengukuran absolut. Offset baro lambat (suhu/drift)
   dipelajari dengan τ = 40 s. Baro yang ditolak > 0,3 detik → **voting 2-dari-3**
   memakai sampel GPS *terkini* (umur ≤ 0,6 s, diterima maupun ditolak),
   ambang = 3·√(σ_gps² + σ_baro²) ≈ 6,5 m:
   - GPS setuju dengan state, tidak dengan baro → **baro yang menyimpang**:
     loncatan diserap ke offset baro (`ALT_BBIAS`), dibatasi |offset| ≤ 30 m.
   - GPS setuju dengan baro, tidak dengan state → **tinggi inersia yang
     menyimpang**: tinggi di-reset ke baro, laju naik ke laju naik GPS.
   - Tidak ada mayoritas → diperlakukan seperti tanpa GPS (langkah 5).
5. **Tanpa GPS / tanpa mayoritas**: baro ditolak maksimal 1 detik lalu diikuti
   lagi. Tanpa GPS offset baro meluruh ke 0 (τ = 40 s), jadi hasilnya kembali
   ≈ baro biasa.
6. Re-referensi GPS (GPS ditolak 20 s) hanya jika baro mendukung state, dan
   **tidak** dihitung sebagai "GPS diterima".

Referensi waktu hanya akumulasi `dt` di `predict()` — tidak ada ketergantungan
Arduino, sehingga bisa diuji ulang di PC.

## Diagnosis terbang 2026-10-22 (drift ALT_FUSED 20–45 m)

Plot dan skrip: [tools/flight_analysis/flight_2026-10-22_b/altitude_fusion/](../tools/flight_analysis/flight_2026-10-22_b/altitude_fusion/)
(`analyze_altitude_fusion.py`, harness `sim_altitude_fusion.cpp`).

**Yang benar adalah baro, bukan fused.** Setelah mendarat (pesawat diam di
tempat lepas landas) baro meleset ≤ 1 m dari bacaan tanah sebelum terbang,
sedangkan fused meleset −17 m (12-35-20), −31 m (13-35-40 flight 1) dan −38 m
(flight 3) — `06_landing_ground_truth.png`. Di loiter flight 3 (t = 872–877 s,
bank 40°, pitch +8…+12°, airspeed turun 31 → 26,5 m/s) pesawat jelas **naik**
(d(baro)/dt ≈ V·sin(pitch−α) ≈ +4 m/s) sementara laju naik fused −6 m/s —
`02_flight3_zoom_loiter.png`.

Rantai kejadian:

1. **Input akselerasi vertikal salah beberapa detik** (`linear_accel · gravity`
   BNO055): galat tersirat median 0,1–0,3 m/s², puncak ≈ 1,2–1,4 m/s²,
   terutama saat bank 25–40° (`05_accel_error_vs_bank.png`). Roll BNO055 dan
   Mahony berbeda ~10° saat bank; kesalahan kemiringan δ pada faktor beban n
   memberi galat ≈ n·g·sin φ·δ ≈ 0,14 m/s² per derajat pada bank 40°.
   `accel_noise_mss = 0,2` menganggap inersia jauh lebih akurat dari ini →
   P tinggi kecil → gerbang baro 3σ hanya ≈ 3 m → **baro ditolak 35–55 % waktu
   terbang**.
2. **Re-anchor memakai GPS basi.** `gpsActive()` = "GPS diterima ≤ 2 s lalu".
   Saat state mulai drift, GPS juga ditolak oleh gerbangnya sendiri, tetapi
   selama 2 s berikutnya `gpsActive()` masih true → `baro_bias_ += innovation`
   menempelkan baro ke tinggi inersia yang salah (BBIAS −3 → +11 m di t ≈ 873,
   ALT_GPS lalu turun ke 0 di t = 875).
3. **Re-referensi GPS memindahkan acuan GPS ke state yang salah** dan sekaligus
   menyetel `last_gps_accept_s_` → `gpsActive()` aktif lagi → re-anchor lagi
   (BBIAS +17 → +27 → +45 m, t = 893–904). Tanpa batas: setelah mendarat
   12-35-20, saat data IMU tampak beku, BBIAS naik ke +520 m dan fused −500 m.
4. Tanpa GPS: reset ke `baro − bias_basi` setelah 3 s, laju naik 0 → loncatan
   +29 m dalam satu sampel (t = 877 → 878) — bukan gerak fisik.

Yang difusikan dari GPS: tinggi MSL (relatif terhadap acuan saat fix pertama)
dan kecepatan vertikal (`-ned_velocity.z`), tetapi **kecepatan vertikal hanya
dipakai jika tinggi GPS lolos gerbang** — begitu state drift > ~8 m, GPS
berhenti dipakai sepenuhnya, padahal justru saat itu ia dibutuhkan.

Batasan analisis: tlog tidak berisi tinggi GPS mentah (`GPS_RAW_INT.alt` dan
`GLOBAL_POSITION_INT.alt` diisi altitude fused, `ALT_GPS` hanya flag) dan SD
card gagal init, jadi akselerasi mentah juga tidak ada; filter tidak bisa
di-replay persis. Perbaikan diverifikasi dengan harness PC di bawah (lintasan
nyata dari baro + galat akselerasi sintetis bergantung roll).

## Hasil uji setelah perbaikan 2026-10-23 (harness `sim_altitude_fusion.cpp`)

Galat rms / maks [m] terhadap truth; baro mentah rms 0,8 m (7,6 m dengan
gangguan cahaya 20 m).

| Skenario | Lama | Baru |
| --- | --- | --- |
| Gangguan cahaya 20 m, GPS | 0,25 / 0,6 | 0,56 / 1,1 |
| Gangguan cahaya 20 m, tanpa GPS | 7,6 / 21,6 | 7,6 / 20,9 |
| Naik 3 m/s / dive, tanpa GPS | 0,22 / 1,3 | 0,27 / 1,8 |
| GPS loncat +20 m 30 s | 0,40 / 1,1 | 0,26 / 1,4 |
| IMU beku (−8 m/s², 60 s), GPS | **divergen (10⁶ m)** | 0,72 / 10 |
| IMU beku, tanpa GPS | 10,5 / 84 | 2,2 / 18 |
| Lintasan flight 1, galat accel −1,5 m/s²·sin\|φ\|/sin40°, GPS | 5,3 / 38 (bias 12) | 0,45 / 1,5 |
| Lintasan flight 3, galat −1,0, tanpa GPS | 6,1 / 36 | 0,37 / 2,3 |
| Lintasan flight 1/3, galat ±2…3 m/s², GPS | divergen (10²–10⁸ m) | ≤ 1,9 / 5,2 |

Harga yang dibayar: penolakan gangguan cahaya dengan GPS sedikit kurang halus
(0,25 → 0,56 m rms, tetap 13× lebih baik dari baro). Tanpa GPS, gangguan
cahaya tetap tidak bisa ditolak (sama seperti sebelumnya).

## Hasil uji awal (simulasi PC, `g++`, data nyata + sintetis, sebelum 2026-10-23)

Gangguan nyata dari tlog (t > 210 s), kebenaran = diam di 0 m:

| Skenario | Galat rms | Galat maks |
| --- | --- | --- |
| Baro mentah | 10,8 m | 21,6 m |
| Fusion, **dengan GPS** | **1,6 m** | 2,9 m |
| Fusion, tanpa GPS | ≈ 11 m | ≈ 25 m (tidak membaik) |

Skenario sintetis (accel bias 0,4 m/s², noise baro 0,75 m):

| Skenario | rms |
| --- | --- |
| Diam tanpa gangguan, GPS / tanpa GPS | 0,21 / 0,17 m (baro mentah 0,75 m) |
| Loncatan baro 20 m selama 40 s, GPS | 0,54 m |
| Naik 3 m/s (60 m), GPS / tanpa GPS | 0,10 / 0,24 m |
| Naik + loncatan 20 m, GPS | 0,60 m |
| Dive 10 m/s, GPS / tanpa GPS | 0,10 / 0,17 m |

Perubahan altitude sungguhan tidak ditolak (tidak terkunci); tidak ada
"lock-out" karena ada timeout dan penyerapan ke bias.

## Batasan penting

- **Tanpa GPS 3D tidak ada keajaiban**: akselerometer tidak bisa membedakan
  kenaikan lambat sungguhan dari offset sensor. Perlindungan utamanya tetap
  **menutup sensor dari cahaya/aliran udara** (foam/selotip opak).
- Referensi GPS ditetapkan sekali, pada fix pertama. Jangan ada gangguan baro besar
  saat boot/ fix pertama; tekanan acuan baro sendiri juga dikunci saat `begin()`
  (kalibrasi ulang lewat "Calibrate Baro" me-reset fusion otomatis).
- Altitude tetap relatif terhadap tekanan saat boot (bukan MSL).
- Parameter (`AltitudeFusionConfig`) baru disesuaikan dari terbang
  2026-10-22 lewat simulasi (`accel_noise_mss` 0,2 → 0,7,
  `baro_no_gps_reset_s` 3 → 1, `baro_bias_max_m` = 30); belum diuji terbang.
  Saat uji terbang pertama, bandingkan `ALT_FUSED` dengan `ALT_BARO`: selisih
  > 10 m yang bertahan berarti masalah belum selesai — matikan
  `FC_ALTITUDE_FUSION_ENABLE` (kembali ke baro saja).

## Telemetri (Mission Planner, tab Status)

`ALT_FUSED`, `ALT_BARO` (baro-saja), `ALT_BBIAS` (offset baro yang sedang
ditolak — mestinya melonjak saat gangguan cahaya), `ALT_REJ`, `ALT_GPS`
(**flag 0/1** `gpsActive()`, bukan tinggi GPS). Tinggi dan kecepatan vertikal
GPS mentah belum dikirim — perlu ditambahkan agar drift bisa dianalisis
langsung (lihat Diagnosis 2026-10-22).
`BarometerData::baro_only_altitude_m` selalu menyimpan altitude baro-saja.
