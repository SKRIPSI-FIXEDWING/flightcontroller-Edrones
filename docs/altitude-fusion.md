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
   vertikal** menjadi pengukuran absolut. Baro yang ditolak > 0,3 detik saat GPS
   aktif → loncatannya diserap ke **offset baro** (`ALT_BBIAS`), bukan ke
   altitude. Offset baro lambat (suhu/drift) dipelajari dengan τ = 40 s.
5. **Tanpa GPS**: baro ditolak maksimal 3 detik lalu diikuti lagi. Tidak ada
   pihak yang bisa menentukan mana yang benar, jadi hasilnya ≈ baro biasa.

Referensi waktu hanya akumulasi `dt` di `predict()` — tidak ada ketergantungan
Arduino, sehingga bisa diuji ulang di PC.

## Hasil uji (simulasi PC, `g++`, data nyata + sintetis)

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
- Parameter (`AltitudeFusionConfig`) belum di-tuning di terbang nyata; nilai awal
  dari log bench 22 Agustus dan 13 Oktober.

## Telemetri (Mission Planner, tab Status)

`ALT_FUSED`, `ALT_BARO` (baro-saja), `ALT_BBIAS` (offset baro yang sedang
ditolak — mestinya melonjak saat gangguan cahaya), `ALT_REJ`, `ALT_GPS`.
`BarometerData::baro_only_altitude_m` selalu menyimpan altitude baro-saja.
