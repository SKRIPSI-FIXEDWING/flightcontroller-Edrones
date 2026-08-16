# Refactor AHRS (estimasi posisi/kecepatan)

Dokumen ini menjelaskan batas modul, konvensi data, dan migrasi dari program
KHAGESWARA lama. Ini adalah keputusan arsitektur yang **dikonfirmasi eksplisit dengan
pengguna** (bukan asumsi sepihak) — lihat bagian "Kenapa EKF dihapus" di bawah.

## Struktur modul

```text
include/estimation/Ahrs.h   Kontrak data dan deklarasi class Ahrs
src/estimation/Ahrs.cpp     Implementasi DCM, drift-corrected velocity, windspeed
```

## Kenapa EKF (NavEKF/TinyEKF) dihapus

Audit kode lama menemukan bahwa `nav_ekf` (di `NavEKF.h`, berbasis `TinyEKF.h`)
**dijalankan tiap loop tapi hasilnya tidak pernah benar-benar dipakai**:

- `AHRS::get_position(Locations &loc)` lama memanggil `ekf.get_pos_ned(loc)` lalu
  **langsung menimpa** `loc.lat/lng/alt` dengan data GPS mentah
  (`gepees.latitude/longitude`, `baro.altitude`) di baris berikutnya — hasil EKF
  dibuang, tidak pernah sampai ke pemanggil.
- `nav_ekf::get_vel_ned()` (kecepatan hasil EKF) **tidak punya pemanggil sama
  sekali** di seluruh codebase yang ditelusuri.
- `L1_Controller.h` (`ahrs.get_position()`) dan `Navigation.h` — konsumen posisi yang
  sebenarnya terbang — jadi memakai **GPS mentah + altitude barometer**, bukan hasil
  fusi EKF apa pun.

Pengguna diberi tahu temuan ini secara eksplisit dan **memilih port apa adanya**: GPS
mentah + baro, tanpa EKF, alih-alih memperbaiki EKF supaya benar-benar dipakai (opsi
itu ditawarkan tapi tidak dipilih, karena mengubah perilaku navigasi nyata yang belum
pernah divalidasi). `AOA`/`SSA` (angle-of-attack/sideslip) yang dihitung
`updateAOASSA()` juga dihapus dengan alasan serupa — tidak punya getter publik, tidak
ada pemakai di luar `AHRS` sama sekali.

`lib/math/Filter/TinyEKF/`, `NavEKF.h`, `TinyEKF.h` **tidak dimigrasikan**.

## Perbaikan bug: faktor `× 10000` pada dead-reckoning velocity

`AHRS::estimateGroundspeedVelocity()` lama menghitung
`groundspeed = (hp + lp) * 10000` — faktor ini adalah **bug skala, bukan pilihan
desain**. Bukti: komentar penulis asli sendiri persis di baris terdekat
(`driftCorrection()`, "ini dicek lagi, kalau terbang jangan (* 1000)") menandai
persis kelas masalah skala ini sebagai belum tuntas. Mengalikan kecepatan m/s dengan
10000 akan mengirim nilai kecepatan yang meledak ke L1/TECS.

Port baru **menghilangkan faktor ini** (`groundspeed = hp + lp`, tanpa perkalian).
Jalur ini (`updateDriftCorrectedVelocity()`'s fallback branch) hanya berjalan saat
ada fix GPS tapi basi/hanya 2D — **belum pernah teruji benar di program lama**
(kemungkinan besar jalur ini nyaris tidak pernah tereksekusi dalam praktik, karena GPS
NMEA rutin mengirim update kecepatan tiap detik). Validasi lewat SITL/simulasi
direkomendasikan sebelum mengandalkan jalur ini di penerbangan nyata.

## Kepemilikan dan penjadwalan

`Ahrs` adalah kelas komputasi murni — tidak memegang instance driver sensor
(`Imu`/`Barometer`/`Airspeed`/GNSS), hanya menerima snapshot data (`ImuData`,
`GnssFixData`, `BarometerData`, `AirspeedData`) tiap panggilan `update()`. Ini
konsisten dengan `GnssFixData` sebagai antarmuka bersama tiga backend GNSS —
`Ahrs` tidak perlu tahu backend GPS mana yang aktif.

```cpp
#include "estimation/Ahrs.h"

fc::Ahrs ahrs;

void updateAhrs()
{
    const fc::ImuData imu_data = imu.data();
    const fc::GnssFixData gnss_data = gnss.data();
    const fc::BarometerData baro_data = baro.data();
    const fc::AirspeedData airspeed_data = airspeed.data();

    if (!ahrs.homeIsSet()) {
        ahrs.setHome(gnss_data, baro_data);
    }
    ahrs.update(imu_data, gnss_data, baro_data, airspeed_data);
}

// Dipanggil terpisah (mengikuti call graph lama: estimate_windspeed() dipanggil
// dari Navigation::nav_gps(), bukan dari update_ahrs()) — akan diaktifkan saat
// Navigation di-port pada Fase 3.
void updateWindEstimate()
{
    ahrs.updateWindspeed();
}
```

## Pemetaan API lama

| Program lama (`AHRS`) | Program baru (`Ahrs`) |
| --- | --- |
| `ahrs.init_ahrs()` | Dihapus; masing-masing driver sensor punya `begin()` sendiri (`Imu`, `Barometer`, `Airspeed`, `GnssNmea`/`GnssUbx`/`Here4GnssReceiver`) — `Ahrs` tidak lagi jadi hub bring-up sensor |
| `ahrs.update_ahrs()` | `ahrs.update(imu_data, gnss_data, baro_data, airspeed_data)` |
| `ahrs.haveGPS()` | `ahrs.haveGps(gnss_data)` |
| `ahrs.get_position(loc)` | `ahrs.data().position` (diisi tiap `update()`, GPS mentah + baro, lihat catatan EKF di atas) |
| `ahrs.setHome(loc)` | `ahrs.setHome(gnss_data, baro_data)`, lalu baca `ahrs.homeIsSet()`/`ahrs.data().home_set` |
| `ahrs.get_velocity_ned()` | `ahrs.data().velocity_ned_mps` |
| `ahrs.get_rotation_body_to_ned()` | `ahrs.data().rotation_body_to_ned` |
| `ahrs.get_accel_ef()` | `ahrs.data().acceleration_earth_frame_mss` |
| `ahrs.get_accel_bf()` | `ahrs.data().acceleration_body_frame_mss` (dihitung tiap `update()`, tidak seperti versi lama yang hanya ter-update saat jalur dead-reckoning berjalan — lihat catatan di bawah) |
| `ahrs.is_airspeed_sensor_enabled` | `ahrs.data().airspeed_sensor_enabled` |
| `ahrs.airspeedEstimate(&v)` | `ahrs.data().estimated_airspeed_mps` (valid, cek `ahrs.haveGps()`) |
| `ahrs.estimate_windspeed()` | `ahrs.updateWindspeed()`, hasil di `ahrs.data().windspeed_mps`/`windspeed_horizontal_mps` |
| `ahrs.get_relative_position_D_home(posD)` | `-ahrs.data().position.alt / 100.0f` (posisi altitude sudah dalam `data().position`, tidak perlu method terpisah) |
| `ahrs.updateAOASSA()` / `AOA` / `SSA` | **Dihapus** — dihitung di program lama tapi tidak punya getter publik dan tidak ada pemakai di luar `AHRS` sama sekali |
| `nav_ekf` / `NavEKF.h` / `TinyEKF.h` | **Dihapus** — lihat "Kenapa EKF dihapus" di atas |

## File lama yang tidak dimigrasikan

```text
include/AHRS.h
include/NavEKF.h
include/TinyEKF.h
lib/math/Filter/TinyEKF/
```

`lib/math/Filter/TinyEKF/tiny_ekf.c`/`.h` dibiarkan ada di `lib/math` (tidak dihapus
fisik, karena berpotensi dipakai ulang untuk kebutuhan estimasi lain di masa depan),
tapi **tidak di-include oleh modul manapun di program baru**.
