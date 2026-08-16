# Refactor GNSS UBX (u-blox M10)

Dokumen ini menjelaskan modul GNSS baru untuk modul u-blox M10 — **modul baru, tidak
ada padanan langsung di program KHAGESWARA lama** (program lama hanya memakai NMEA
via `TinyGPSPlus`, lihat `docs/gnss-nmea-se100.md`). Backend ini menerbitkan
`fc::GnssFixData` yang sama dengan backend NMEA dan DroneCAN
(`include/navigation/GnssFixData.h`).

## Struktur modul

```text
include/drivers/GnssUbx.h   Kontrak data dan deklarasi class GnssUbx
src/drivers/GnssUbx.cpp     Implementasi memakai SparkFun u-blox GNSS Arduino Library
```

Library `lib/SparkFun_u-blox_GNSS_Arduino_Library-main` sudah ter-vendor di program
lama tetapi **tidak pernah dipakai** (vestigial) — backend ini adalah pemakai
pertamanya.

## Transport dan protokol

I2C (`Wire`, alamat default `0x42`), memakai protokol UBX (bukan NMEA) —
`setI2COutput(COM_TYPE_UBX)` dipanggil saat `begin()`. `update()` melakukan polling
eksplisit terhadap UBX-NAV-PVT (`getPVT()`) setiap dipanggil, **bukan** mengaktifkan
mode `autoPVT` background milik library — ini sengaja dipilih agar waktu baca
deterministik di bawah scheduler polling milik codebase ini, bukan callback
background.

## Kepemilikan dan penjadwalan

```cpp
#include "drivers/GnssUbx.h"

fc::GnssUbx gnss(Wire);

void setupGnss()
{
    if (!gnss.begin()) {
        // Masuk ke penanganan pre-arm/failsafe.
    }
}

void updateGnss()
{
    if (gnss.update()) {
        const fc::GnssFixData fix = gnss.data();
    }
}
```

## Data yang dihasilkan

Berbeda dari backend NMEA, backend ini **mengisi kecepatan vertikal
(`velocity_ned_mps[2]`) dengan nilai sungguhan** dari solusi NED milik modul U-blox
(`getNedDownVel()`), bukan 0 — modul u-blox memang menghitung kecepatan 3-sumbu NED
secara native, sehingga tidak perlu disintesis dari barometer seperti pada backend
NMEA.

`hdop` pada struct bersama diisi dengan **PDOP** (positional dilution of precision),
bukan HDOP murni — modul u-blox mengekspos PDOP melalui `getPDOP()`, bukan HDOP
terpisah melalui API dasar ini. Lihat catatan pada `GnssFixData.h`.

Pemetaan `fix_type` dari `getFixType()`: 0/1/5 (no-fix/dead-reckoning-only/time-only)
→ `NoFix`, 2 → `Fix2D`, 3 atau 4 (3D atau 3D+dead-reckoning) → `Fix3D`.

## File terkait

```text
lib/SparkFun_u-blox_GNSS_Arduino_Library-main/  Vendor library (sudah ada di program lama, sekarang dipakai)
```
