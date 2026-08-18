# Opsi 2: Complementary filter altitude baro+accel (fc::AltitudeComplementaryFilter)

Dua opsi estimasi altitude tersedia berdampingan di firmware ini, untuk
perbandingan bench/skripsi -- **bukan** untuk operator memilih salah satu
saat terbang:

- **Opsi 1 -- Kalman filter baro-only** (`fc::Barometer`'s onboard 1D Kalman,
  lihat `docs/barometer-ms5611.md`). Ini yang **tetap** dikonsumsi
  TECS/Navigation untuk kontrol penerbangan -- tidak berubah oleh modul ini.
- **Opsi 2 -- Complementary filter baro+accel** (`fc::AltitudeComplementaryFilter`,
  modul baru). Berjalan paralel, di-log berdampingan dengan Opsi 1, **tidak**
  disambungkan ke TECS/Navigation.

## Kenapa tidak langsung menggantikan Opsi 1

Mengganti sumber altitude yang dipakai kontrol penerbangan (TECS/altitude-hold)
adalah perubahan safety-critical yang butuh validasi sebelum dipercaya
terbang. Modul ini sengaja dibuat sebagai estimator paralel yang di-log,
supaya Opsi 1 vs Opsi 2 bisa dibandingkan dulu dari data bench/terbang nyata
sebelum keputusan "pindah ke Opsi 2" diambil -- lihat
`tools/flight_analysis/compare_altitude_estimators.py`. Menyambungkannya ke
TECS/Navigation adalah langkah terpisah, lebih besar, di luar cakupan
penambahan modul ini.

## Referensi

Sabatini, A.M.; Genovese, V. **"A Sensor Fusion Method for Tracking Vertical
Velocity and Height Based on Inertial and Barometric Altimeter
Measurements."** *Sensors* 2014, 14(8), 13324-13347.
https://pmc.ncbi.nlm.nih.gov/articles/PMC4179067/ (open access, MDPI).

Paper ini mengusulkan complementary filter 2-state (posisi + kecepatan
vertikal) yang men-double-integrate akselerasi (gravity-compensated) lalu
menarik hasilnya kembali ke pembacaan barometer, dengan gain filter
ditentukan dari rasio deviasi standar noise: `tau = sigma_v/sigma_w` (sigma_v
= noise altitude barometer, sigma_w = noise akselerasi). Validasi eksperimental
mereka (kondisi diam 3 menit, free-fall, gerakan sirkular paksa, jongkok)
menunjukkan drift praktis hilang di domain kecepatan berkat kontribusi
barometer, dengan akurasi real-time height RMSE 5-68 cm dan velocity RMSE
0.04-0.24 m/s tergantung skenario gerakan.

Referensi kedua yang relevan (belum diimplementasikan di sini, dicatat untuk
kerja lanjutan): Wei, S.; Dan, G.; Chen, H. **"Altitude data fusion utilising
differential measurement and complementary filter."** *IET Science,
Measurement & Technology*, 2016, 10(9), 874-879. DOI: 10.1049/iet-smt.2016.0118
-- mengusulkan pengukuran altitude diferensial untuk lebih lanjut mengurangi
drift barometer sebelum masuk ke complementary filter.

## Struktur matematis yang diimplementasikan

`AltitudeComplementaryFilter::update()` ([AltitudeComplementaryFilter.cpp](../src/estimation/AltitudeComplementaryFilter.cpp))
mengimplementasikan bentuk diskrit disederhanakan dari struktur di atas --
complementary filter PI 2-state (posisi, kecepatan), bukan replikasi persis
matriks gain paper (yang menurunkan gain optimal dari kovariansi noise
eksplisit):

```text
// Predict: double-integrate akselerasi vertikal (gravity-compensated, positive-up)
velocity += accel_up * dt
altitude += velocity * dt

// Correct: tarik kedua state ke arah barometer, membatasi drift integrasi accel
error = baro_altitude - altitude
velocity += Ki * error * dt
altitude += Kp * error * dt
```

`Kp` dan `Ki` (`AltitudeComplementaryConfig`, default 1.0 dan 0.2) adalah
gain yang sepadan dengan `tau = sigma_v/sigma_w` pada paper -- semakin besar
relatif terhadap noise akselerasi, semakin cepat filter menarik kembali ke
barometer (kurang drift, tapi respons terhadap gerakan cepat sedikit
tertunda oleh koreksi barometer yang lebih agresif). Nilai ini belum di-tuning
secara empiris untuk MS5607+BNO055 spesifik proyek ini -- gunakan
`compare_altitude_estimators.py` terhadap data bench/terbang nyata untuk
menyetelnya.

## Sumber data input

- **Barometer**: `raw_altitude_m` (pra-Kalman, bukan `altitude_m` yang sudah
  difilter Opsi 1) -- supaya Opsi 1 dan Opsi 2 sama-sama berasal dari sinyal
  mentah yang sama, adil untuk dibandingkan, bukan Opsi 2 memfilter ulang
  hasil Opsi 1.
- **Akselerasi vertikal**: `AhrsData::acceleration_body_frame_mss.z` (sudah
  gravity-compensated dan dirotasi ke frame NED oleh `fc::Ahrs`, lihat
  `Ahrs.cpp`), dinegasikan (`-z`) karena konvensi NED "naik = Z negatif",
  disamakan dengan konvensi `climb_rate_mps` yang sudah ada ("positif =
  climbing"). Field ini sudah dipakai jalur lain di `Ahrs.cpp`
  (`updateDriftCorrectedVelocity()`), jadi tidak ada rotasi frame baru yang
  ditambahkan -- modul ini murni mengonsumsi ulang.

## Penjadwalan

Dipanggil dari `taskControl` (200 Hz, [main.cpp](../src/main.cpp)) setelah
`g_modeManager.update()` (yang memperbarui `g_ahrs` sebagai efek samping),
memakai `g_ctx->dt_s` yang sama dengan loop kontrol. **Tidak** ada task
FreeRTOS terpisah untuk modul ini.

## Format CSV (`fc::DataLogger`)

```text
seq,timestamp_us,pressure_pa,temperature_c,altitude_m,raw_altitude_m,climb_rate_mps,comp_altitude_m,comp_climb_rate_mps,comp_accel_up_mss
```

`altitude_m`/`climb_rate_mps` = Opsi 1 (Kalman). `comp_altitude_m`/
`comp_climb_rate_mps` = Opsi 2 (complementary). `comp_accel_up_mss` = nilai
akselerasi vertikal mentah yang benar-benar dipakai `update()` tiap siklus
(kolom diagnostik, lihat "Temuan bench" di bawah). Log lama (9 kolom, tanpa
`comp_accel_up_mss`; 7 kolom, tanpa Opsi 2; 6 kolom, tanpa `raw_altitude_m`)
tetap bisa dibaca `tools/flight_analysis/baro_log_io.py` -- kolom yang belum
ada diisi `NaN`.

## Membandingkan kedua opsi

```bash
cd tools/flight_analysis
.venv/Scripts/python.exe compare_altitude_estimators.py "path/ke/AKUSISI-BARO ....txt"
```

Mencetak statistik berdampingan (std, bias, selisih) dan menyimpan grafik
perbandingan. Skrip ini **tidak** memutuskan opsi mana yang "lebih baik" --
itu tergantung jenis pengujian (diam di meja: std lebih rendah lebih baik,
karena altitude sebenarnya konstan; gerakan vertikal nyata: lag dan noise
dua-duanya penting, jangan asal pilih yang paling halus).

## Temuan bench pertama (2026-08-15): offset besar, dugaan bug gravity-compensation lama

Pengujian bench pertama Opsi 2 (barometer diam di meja, ~1270 detik)
menunjukkan `comp_altitude_m` melesat dari 0 ke sekitar -94 m dalam ~50
detik, lalu menetap (plateau) di sekitar -94 sampai -97 m untuk sisa sesi --
**bukan** drift tak terbatas, tapi offset steady-state yang konsisten. Pola
ini persis perilaku teoretis complementary filter di bawah bias akselerasi
konstan `b`: pada steady state, `baro_altitude - comp_altitude` menetap di
`-b/Ki`. Balik-hitung dari offset yang teramati (Ki=0.2 default): bias
tersirat **≈ -18.97 m/s² ≈ -1.93 g** -- terlalu dekat ke -2g untuk kebetulan,
mengindikasikan bug tanda/dobel-hitung pada kompensasi gravitasi, bukan
sekadar bias sensor biasa (yang lazimnya hanya ±0.05-0.2 m/s²).

Filter complementary-nya sendiri **bekerja sesuai spesifikasi** -- offset
yang bounded (bukan divergen ke -infinity) justru bukti gain Kp/Ki-nya
menstabilkan sistem dengan benar terhadap gangguan konstan. Sumber
masalahnya kemungkinan besar bukan modul ini, tapi `AhrsData::acceleration_body_frame_mss`
di `Ahrs.cpp` -- field yang **sudah ada sebelum modul ini ditulis**, dan
komentar aslinya di kode sendiri sudah menandai jalur ini belum pernah
tervalidasi benar ("never exercised correctly ... validate it in SITL
before relying on it in flight", lihat `Ahrs.cpp`'s
`updateDriftCorrectedVelocity()`). Modul ini hanya mengonsumsi ulang field
tersebut, tidak menulis rotasi/kompensasi gravitasinya.

Kolom `comp_accel_up_mss` ditambahkan setelah temuan ini supaya bias bisa
dilihat langsung dari data mentah, bukan cuma disimpulkan tidak langsung dari
efeknya ke altitude. `compare_altitude_estimators.py` sekarang mencetak dan
memplot nilai ini, plus cross-check antara bias yang terukur langsung
(`comp_accel_up_mss`) vs bias yang tersirat dari offset steady-state Opsi 2
-- kalau dua-duanya kira-kira cocok, itu konfirmasi kuat masalahnya ada di
sinyal akselerasi (`Ahrs.cpp`), bukan di `AltitudeComplementaryFilter` atau
gain Kp/Ki-nya.

**Belum diperbaiki** -- perlu data bench baru (firmware dengan
`comp_accel_up_mss`) untuk memastikan diagnosis sebelum menyentuh `Ahrs.cpp`
(mengubah gravity-compensation di sana berisiko memengaruhi jalur
dead-reckoning groundspeed yang sudah memakai field yang sama).

## Keterbatasan yang diketahui

- `AltitudeComplementaryFilter` **tidak** otomatis me-reset saat
  `Barometer::recalibrateGroundPressure()` dipanggil (lihat
  `docs/barometer-ms5611.md`) -- filter akan konvergen kembali ke referensi
  baru secara alami dalam ~1/Kp detik (default Kp=1.0 → ~1 detik) lewat
  suku koreksi error-nya sendiri, bukan seketika seperti Opsi 1. Untuk
  pengujian bench yang sangat sensitif terhadap transient ini, tunggu
  beberapa detik setelah kalibrasi ulang sebelum mengambil data pembanding.
- Gain `Kp`/`Ki` masih nilai default belum di-tuning empiris untuk
  MS5607+BNO055 spesifik hardware ini.
- Belum divalidasi pada gerakan vertikal nyata (baru diuji compile + skrip
  analisis dengan data sintetis) -- validasi bench dengan gerakan naik-turun
  terukur adalah langkah selanjutnya sebelum data ini layak dipakai sebagai
  temuan skripsi yang kuat, sama seperti validasi Opsi 1 di
  `docs/barometer-ms5611.md`.
