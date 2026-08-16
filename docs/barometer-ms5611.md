# Refactor Barometer MS5611

Dokumen ini menjelaskan batas modul, konvensi data, dan migrasi dari program
KHAGESWARA lama. Implementasi baru dikhususkan untuk wahana fixed-wing.

## Struktur modul

```text
include/drivers/Barometer.h       Kontrak data dan deklarasi class Barometer
src/drivers/Barometer.cpp         Implementasi sensor, kompensasi, dan transport I2C
lib/math/Filter/KalmanFilter1D.h  Kalman filter skalar generik (baru, dipakai altitude)
lib/math/Filter/DerivativeFilter.h  Derivative filter 7-titik (vendor ArduPilot, sudah ada)
```

`include/drivers/ms5611.h` dan `I2CDev.h` lama tidak digunakan. Transport I2C
langsung memakai `Wire1` di `Barometer.cpp`, mengikuti pola `Imu.cpp` (bus read/write
sendiri, bukan lewat wrapper `I2C`/`_I2C1`).

## Kepemilikan dan penjadwalan

Sama seperti `fc::Imu`: driver tidak membuat task sendiri. Pemilik utama flight
controller membuat satu object `fc::Barometer`, memanggil `begin()` saat inisialisasi
(blocking selama ± 2 detik untuk kalibrasi tekanan tanah, sama seperti perilaku lama),
lalu memanggil `update()` dari scheduler.

```cpp
#include "drivers/Barometer.h"

fc::Barometer baro(Wire1);

void setupBaro()
{
    if (!baro.begin()) {
        // Masuk ke penanganan pre-arm/failsafe.
    }
}

void updateBaro()
{
    if (baro.update()) {
        const fc::BarometerData baro_data = baro.data();
    }
}
```

## Data yang dihasilkan

| Data | Unit |
| --- | --- |
| `pressure_pa`, `temperature_c` | Pa, derajat Celsius |
| `altitude_m` | meter, relatif terhadap tekanan tanah saat `begin()`, sudah difilter Kalman |
| `raw_altitude_m` | sama seperti `altitude_m`, tapi **sebelum** filter Kalman -- hanya untuk logging/tuning filter, bukan konsumsi controller |
| `climb_rate_mps` | meter/detik, dari derivative filter 7-titik |
| `eas2tas` | rasio equivalent-to-true-airspeed (model ISA lapse-rate) |

## Varian chip: MS5611 vs MS5607 (`BarometerConfig::chip_variant`)

MS5611 dan MS5607 memakai command set dan layout PROM (C1-C6) yang identik,
tapi konstanta bit-shift rumus kompensasi OFF/SENS **berbeda** (MS5607 punya
rentang tekanan lebih lebar, jadi butuh satu bit shift berbeda pada tiap
suku). Modul breakout murah "GY-63 MS5611" kadang sebenarnya berisi chip
MS5607.

**Cara terdeteksi**: data akuisisi bench (barometer diam di meja, lihat
`docs/data-logger-usb.md`) menunjukkan `pressure_pa` stabil di ~49845 Pa
(noise std hanya ~1 Pa -- jelas bukan bacaan acak/rusak, tapi konsisten salah
skala) -- setara ketinggian ~5500 m, tidak masuk akal untuk sensor di atas
meja. Dikalikan 2, hasilnya ~99691 Pa (~997 hPa), angka yang wajar untuk
tekanan udara ambien. Pola "konsisten ~2x terlalu rendah, noise rendah"
persis yang terjadi kalau chip fisiknya MS5607 tapi firmware memakai rumus
MS5611 (`SENS=C1×2^15, OFF=C2×2^16` alih-alih `SENS=C1×2^16, OFF=C2×2^17`
milik MS5607).

`BarometerConfig::chip_variant` default ke `BarometerChipVariant::Ms5607`
berdasarkan bukti di atas. Kalau ternyata salah (chip aslinya benar MS5611),
tinggal balik ke `BarometerChipVariant::Ms5611` -- tidak ada perubahan lain
yang diperlukan.

**Verifikasi lebih lanjut**: `Barometer::promDump()` mengembalikan konstanta
kalibrasi C1-C6 mentah, dan `main.cpp`'s `setup()` mencetaknya sekali ke
`Serial` saat boot (`[SETUP] Baro PROM: C1=... C6=...`) -- berguna untuk
membandingkan besaran konstanta dengan datasheet chip yang sebenarnya
terpasang, atau untuk debug lapangan lain di masa depan.

## Kalibrasi ulang ground-pressure sebelum terbang (`recalibrateGroundPressure()`)

`begin()` men-sample referensi ground-pressure **sekali** saat boot. Data
bench (`docs/data-logger-usb.md`) menunjukkan tekanan ambien bisa hanyut
(drift) ~1 hPa/jam akibat cuaca/HVAC -- kalau ada jeda antara boot dan lepas
landas, altitude 0 akan mulai meleset dari posisi darat sesungguhnya.

`Barometer::recalibrateGroundPressure()` men-sample ulang ground-pressure
di posisi saat ini (tanpa reset chip / baca ulang PROM -- itu hanya perlu
sekali di `begin()`) dan me-reset filter Kalman + derivative filter supaya
altitude langsung terbaca 0, bukan konvergen pelan ke 0.

Diekspos lewat MAVLink sebagai `MAV_CMD_PREFLIGHT_CALIBRATION` dengan
`param3=1` -- ini persis perintah yang dikirim tombol **"Calibrate Baro"**
Mission Planner (klik-kanan pada HUD) dan QGroundControl. Lihat
`Mavlink.cpp`'s `handleCommandLong()`. Ditolak (`MAV_RESULT_DENIED`) kalau
`Radio::armed()` true -- harus disarm dan diam di darat dulu, sama seperti
filosofi keamanan arming di `docs/radio.md`. Kalibrasi sensor lain (gyro,
mag, accel, radio, ESC -- param1/2/4/5/6/7) tidak diimplementasikan dan akan
dibalas `MAV_RESULT_UNSUPPORTED`.

`altitudeMeters()`, `climbRateMetersPerSecond()`, `equivalentToTrueAirspeedRatio()`,
`pressurePascals()`, `temperatureCelsius()` disediakan untuk migrasi controller lama.
Kode baru dapat mengambil seluruh hasil sekaligus melalui `baro.data()`.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `Barometer baro;` (global di `gps.h`) | Object `fc::Barometer` dimiliki vehicle/core |
| `baro.init_baro()` | `baro.begin()` dan periksa nilai kembaliannya |
| `baro.update_baro()` | `baro.update()` dan periksa nilai kembaliannya |
| `baro.altitude` | `baro.data().altitude_m` atau `baro.altitudeMeters()` |
| `baro.get_climb_rate()` | `baro.data().climb_rate_mps` atau `baro.climbRateMetersPerSecond()` |
| `baro.getEAS2TAS()` | `baro.data().eas2tas` atau `baro.equivalentToTrueAirspeedRatio()` |
| `baro.get_pressure()` | `baro.data().pressure_pa` atau `baro.pressurePascals()` |
| `baro.get_temperature()` | `baro.data().temperature_c` atau `baro.temperatureCelsius()` |
| `baro.readPressure(bool)` / `readTemperature(bool)` | Digabung ke dalam `compensate()` privat, dipanggil sekali per `update()` |
| `baro.thread_baro()` | Dihapus; tidak pernah dipanggil di program lama |
| `baro.sea_level` / `calcSeaLevel()` | Dihapus; tidak pernah dibaca di luar `Barometer` pada program lama |
| `hypsometricEquation()` / `virtual_temp` | Dihapus; dead code, tidak pernah dipanggil (altitude memakai formula barometrik `calcAltitude`, bukan hypsometric) |
| `reached_approach_alt()` / `highest_altitude` | **Belum dimigrasikan** — dipakai oleh `takeoff.h` lama; perlu port ulang saat modul takeoff/landing direfactor |

## Perilaku kegagalan

- `begin()` gagal jika reset chip, pembacaan PROM, atau kalibrasi tekanan tanah gagal
  (setiap langkah I2C diperiksa nilai kembaliannya, berbeda dari program lama yang
  tidak pernah memeriksa hasil `I2CDev`).
- `update()` tidak menerbitkan data parsial. Sampel terakhir tetap dipertahankan jika
  pembacaan tekanan atau suhu mentah gagal.
- `lastError()` dan `consecutiveReadFailures()` dapat dipakai oleh pre-arm, health
  monitor, dan failsafe.
- `isFresh(maximum_age_us)` mencegah controller memakai data barometer yang basi.
- Guard NaN/Inf pada `altitude_m` dipertahankan dari program lama (`baro_safe_value`).

## File lama yang tidak dimigrasikan

```text
include/Barometer.h
include/drivers/ms5611.h
```

File-file tersebut tetap boleh berada di arsip program lama sampai seluruh pemakai
barometer selesai dimigrasikan (AHRS, TECS, L1, Navigation, takeoff, Mavlink), tetapi
tidak boleh dimasukkan ke build program baru.
