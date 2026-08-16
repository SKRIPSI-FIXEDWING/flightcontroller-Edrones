# Refactor Airspeed MS4525DO

Dokumen ini menjelaskan batas modul, konvensi data, dan migrasi dari program
KHAGESWARA lama. Implementasi baru dikhususkan untuk wahana fixed-wing.

## Struktur modul

```text
include/drivers/Airspeed.h       Kontrak data dan deklarasi class Airspeed
src/drivers/Airspeed.cpp         Implementasi sensor dan transport I2C (Wire2)
lib/math/Filter/KalmanFilter1D.h Kalman filter skalar generik (sama dipakai Barometer)
```

## Kepemilikan dan penjadwalan

Sama seperti `fc::Imu`/`fc::Barometer`: driver tidak membuat task sendiri. Pemilik
utama flight controller membuat satu object `fc::Airspeed`, memanggil `begin()` saat
inisialisasi (mengambil satu sampel offset nol-tekanan — **pastikan pitot tube
terlepas/tidak ada aliran udara saat begin() dipanggil**, sama seperti program lama),
lalu memanggil `update()` dari scheduler.

```cpp
#include "drivers/Airspeed.h"

fc::Airspeed arspd(Wire2);

void setupAirspeed()
{
    if (!arspd.begin()) {
        // Masuk ke penanganan pre-arm/failsafe.
    }
}

void updateAirspeed()
{
    if (arspd.update()) {
        const fc::AirspeedData airspeed_data = arspd.data();
    }
}
```

## Data yang dihasilkan

| Data | Unit |
| --- | --- |
| `velocity_mps`, `velocity_kmh` | m/s, km/h — **sekarang benar-benar difilter Kalman** |
| `differential_pressure_psi` | psi |
| `differential_pressure_pa` | Pa (lihat catatan skala di bawah) |

**Kalman filter kini benar-benar aktif** (`R=4, Q=0.3, P0=0.1`, sama dengan nilai yang
sudah ada di program lama tapi baris penerapannya dikomentari — `// v_ms =
kalmanFilter.kalmanFilter(v_ms);`). Perilaku ini eksplisit berbeda dari program lama.

**Catatan skala `differential_pressure_pa`**: program lama menghitung
`prspsi = press_psi * 13789.5144` — konstanta ini adalah 2× faktor konversi psi→Pa
(dipakai ulang dari rumus Bernoulli `v = sqrt(2*dP/rho)`), bukan psi→Pa yang sebenarnya
(psi→Pa sejati adalah ×6894.7572). Field ini tidak pernah dibaca di luar `Airspeed`
pada program lama, jadi nilainya dipertahankan apa adanya (bukan dikoreksi) untuk
migrasi yang setia; jangan pakai `differential_pressure_pa` sebagai Pa yang benar-benar
akurat tanpa membagi 2 dahulu.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `Airspeed arspd;` (global di `AHRS.h`) | Object `fc::Airspeed` dimiliki vehicle/core |
| `arspd.initAirspeed()` | `arspd.begin()` dan periksa nilai kembaliannya |
| `arspd.updateAirspeed()` | `arspd.update()` dan periksa nilai kembaliannya |
| `arspd.v_ms` | `arspd.data().velocity_mps` atau `arspd.velocityMetersPerSecond()` |
| `arspd.v_kmh` | `arspd.data().velocity_kmh` atau `arspd.velocityKilometersPerHour()` |
| `arspd.press_psi` | `arspd.data().differential_pressure_psi` atau `arspd.differentialPressurePsi()` |
| `arspd.prspsi` | `arspd.data().differential_pressure_pa` (lihat catatan skala di atas) |
| `arspd.calibrateAirspeed()` | **Dihapus**; tidak pernah dipanggil di program lama |
| `arspd.airspeedThread()` | **Dihapus**; deklarasi tanpa implementasi di program lama, tidak pernah dipanggil |
| `randomSeed(A15)` di `initAirspeed()` | **Dihapus**; tidak berkaitan dengan airspeed, tidak ada pemakai `random()` aktif di jalur fixed-wing |
| `delay(2)` sebelum/sesudah `requestFrom` di `getRawAirspeed()` | **Dihapus**; keduanya idle time murni (tidak ada transaksi bus yang berlangsung saat delay tsb — `beginTransmission`/`endTransmission` yang membungkusnya tidak pernah benar-benar mengirim apa pun), `requestFrom()` sudah blocking sampai transaksi I2C selesai |

## Perilaku kegagalan

- `begin()` gagal jika sampel offset awal gagal dibaca dari bus I2C.
- `update()` tidak menerbitkan data parsial. Sampel terakhir tetap dipertahankan jika
  pembacaan mentah gagal.
- `lastError()` dan `consecutiveReadFailures()` dapat dipakai oleh pre-arm, health
  monitor, dan failsafe.
- `isFresh(maximum_age_us)` mencegah controller memakai data airspeed yang basi.
- Deteksi "sensor airspeed masuk akal atau tidak" (NaN/Inf/rentang wajar) tetap menjadi
  tanggung jawab modul AHRS, bukan driver ini — mengikuti pola lama di `AHRS.h`.

## File lama yang tidak dimigrasikan

```text
include/airspeed.h
```

File ini tetap boleh berada di arsip program lama sampai seluruh pemakai airspeed
selesai dimigrasikan (AHRS, TECS, L1, FW_ControlModes, Mavlink), tetapi tidak boleh
dimasukkan ke build program baru.
