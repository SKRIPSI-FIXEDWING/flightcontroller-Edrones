# Refactor GNSS NMEA (Radiolink SE100)

Dokumen ini menjelaskan batas modul, konvensi data, dan migrasi dari program
KHAGESWARA lama. Ini adalah salah satu dari tiga backend GNSS (lihat juga
`docs/gnss-ubx-m10.md` dan `docs/gnss-here4-dronecan.md`); ketiganya menerbitkan
`fc::GnssFixData` yang sama (`include/navigation/GnssFixData.h`) sehingga kode
navigasi tidak perlu tahu backend GPS mana yang sedang dipakai.

## Struktur modul

```text
include/navigation/GnssFixData.h  Struct data bersama untuk ketiga backend GNSS
include/drivers/GnssNmea.h        Kontrak data dan deklarasi class GnssNmea
src/drivers/GnssNmea.cpp          Implementasi parsing NMEA (TinyGPSPlus)
```

## Perubahan transport: SoftwareSerial → HardwareSerial

Program lama memakai `SoftwareSerial ss(0, 1, baudRate)` — serial *bit-banged* pada
pin 0/1. Pin 0/1 pada Teensy 4.1 sebenarnya adalah pin UART perangkat keras
(`Serial1`). Implementasi baru memakai `Serial1` langsung (pin fisik sama persis),
karena serial bit-banged secara umum kurang andal di bawah *preemption* FreeRTOS
dibanding UART perangkat keras. Ini perubahan yang disengaja, bukan migrasi 1:1 buta.

## Kepemilikan dan penjadwalan

Sama seperti `fc::Imu`/`fc::Barometer`/`fc::Airspeed`: driver tidak membuat task
sendiri.

```cpp
#include "drivers/GnssNmea.h"

fc::GnssNmea gnss(Serial1);

void setupGnss()
{
    gnss.begin();
}

void updateGnss()
{
    if (gnss.update()) {
        const fc::GnssFixData fix = gnss.data();
    }
}
```

## Perbaikan dibanding program lama

- **`altitude_m` kini benar-benar diisi** dari sentence GGA (`gps.altitude.meters()`
  milik TinyGPSPlus). Pada program lama, `Gepees::alt_m`/`getAltitudeM()` **tidak
  pernah diisi di mana pun** (selalu 0) — seluruh jalur navigasi lama memang memakai
  `baro.altitude`, bukan altitude dari GPS, jadi ini penambahan yang aman (mengisi
  field yang sebelumnya dead), bukan perubahan perilaku pada sesuatu yang sudah
  dipakai.
- **Kopling ke `Barometer` dihapus.** Program lama meng-instantiate `Barometer baro;`
  *di dalam* `gps.h` dan `calculateVelocity()` membaca `baro.altitude` langsung untuk
  menghitung `velocity.z` (NED down). Modul baru **tidak** menghitung komponen
  vertikal sama sekali (`velocity_ned_mps[2]` selalu 0) — kecepatan vertikal
  disintesis di lapisan AHRS (Fase 2) dari climb rate barometer, bukan diduplikasi di
  setiap driver GNSS. Lihat catatan di `GnssFixData.h`.

## Pemetaan API lama

| Program lama (`Gepees`) | Program baru (`GnssNmea`) |
| --- | --- |
| `gepees.initGPS()` | `gnss.begin()` |
| `gepees.updateGPS()` | `gnss.update()` |
| `gepees.getLatitude()` / `getLongitude()` | `gnss.data().latitude_deg` / `longitude_deg`, atau `gnss.latitude()`/`longitude()` |
| `gepees.getAltitudeM()` | `gnss.data().altitude_m` atau `gnss.altitudeMeters()` (lihat catatan perbaikan di atas) |
| `gepees.getHDOP()` | `gnss.data().hdop` atau `gnss.hdop()` |
| `gepees.getSatellites()` | `gnss.data().satellites` atau `gnss.satellites()` |
| `gepees.getSpeedMPS()` / `getSpeedKMPH()` | `gnss.data().ground_speed_mps` (km/h dapat dihitung dari ini bila perlu) |
| `gepees.getCourse()` | `gnss.data().course_deg` atau `gnss.courseDegrees()` |
| `gepees.getVelocity()` (Vector3f NED) | `gnss.data().velocity_ned_mps[0..2]` (indeks 2/down selalu 0, lihat catatan di atas) |
| `gepees.getStatus()` (`StatusGPS`) | `gnss.data().fix_type` (`fc::GnssFixType`) atau `gnss.status()` |
| `gepees.line` (hitungan sentence) | Dihapus; tidak pernah dibaca di luar `Gepees` pada program lama |

## File lama yang tidak dimigrasikan

```text
include/gps.h
```

`lib/SparkFun_u-blox_GNSS_Arduino_Library-main` yang sudah ter-vendor di program lama
**tidak dipakai oleh backend NMEA ini** — library tersebut dipakai oleh backend UBX
(`GnssUbx`, lihat `docs/gnss-ubx-m10.md`) untuk modul u-blox M10.
