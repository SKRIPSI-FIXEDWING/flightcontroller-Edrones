# Refactor GNSS DroneCAN (CubePilot Here4)

Dokumen ini menjelaskan modul GNSS baru untuk CubePilot Here4 via DroneCAN/CAN bus —
**modul baru, tidak ada padanan di program KHAGESWARA lama** (program lama hanya NMEA
via `TinyGPSPlus`). Backend ini menerbitkan `fc::GnssFixData` yang sama dengan backend
NMEA dan UBX (`include/navigation/GnssFixData.h`).

## PERINGATAN — baca sebelum uji terbang

Dua hal **belum terverifikasi** dan wajib diuji di meja kerja (bench test) dengan CAN
sniffer sebelum dipercaya untuk terbang:

1. **Perilaku node-ID allocation Here4 belum diketahui.** Driver ini mengasumsikan
   kasus umum: Here4 langsung menyiarkan (*broadcast*) Fix2/Auxiliary begitu
   dinyalakan, tanpa menunggu proses *dynamic node-ID allocation* DroneCAN. Jika
   ternyata Here4 menunggu alokasi node-ID dahulu, driver ini **tidak akan menerima
   data apa pun** sampai ditambahkan node responder alokasi minimal — sebuah
   penambahan scope yang cukup besar. Uji dengan CAN sniffer (mis. adapter USB-CAN
   murah + `candump`/SavvyCAN) sebelum mengandalkan modul ini.
2. **Konstanta "data type signature" DSDL adalah PLACEHOLDER, bukan nilai asli.**
   `kFix2DataTypeSignature` dan `kAuxiliaryDataTypeSignature` di
   `include/drivers/GnssHere4Can.h` saat ini bernilai `0x0`. Payload Fix2 (>7 byte)
   pasti terpecah jadi beberapa frame CAN (*multi-frame transfer*), dan libcanard
   **menolak setiap multi-frame transfer yang signature-nya tidak cocok** — salah di
   sini berarti **tidak ada data fix yang pernah sampai ke aplikasi**, tanpa gejala
   lain yang jelas (bukan crash, bukan data yang salah — hanya diam saja). Nilai asli
   harus dihitung dengan tooling signature DSDL resmi (`show_data_type_info.py` dari
   libcanard, atau `dronecan_dsdlc`) terhadap definisi DSDL yang sudah diverifikasi di
   bawah, **sebelum** dipercaya untuk terbang.

Bit-offset field Fix2/Auxiliary di bawah **sudah diverifikasi** langsung terhadap teks
DSDL asli (`dronecan/DSDL/uavcan/equipment/gnss/1063.Fix2.uavcan` dan
`1061.Auxiliary.uavcan`, diambil saat implementasi) — bagian ini punya keyakinan
tinggi, berbeda dari signature constant di atas.

## Struktur modul

```text
include/drivers/GnssHere4Can.h   Kontrak data dan deklarasi class Here4GnssReceiver
src/drivers/GnssHere4Can.cpp     Implementasi decode Fix2/Auxiliary via libcanard
lib/third_party/libcanard/       Vendor libcanard (dronecan/libcanard, MIT license)
```

`tonton81/FlexCAN_T4` (driver CAN Teensy 4.x) tidak ter-vendor di repo — didaftarkan
lewat git URL di `platformio.ini` (tidak terdaftar di registry PlatformIO dengan nama
itu, jadi dipakai lewat git URL yang di-pin ke commit tertentu, bukan hand-vendor
seperti libcanard).

## Transport dan wiring

- **CAN3** dipakai secara sengaja (pin 30/31 di Teensy 4.1), **bukan CAN2** (pin
  0/1) — pin 0/1 sudah dipakai `GnssNmea` untuk `Serial1`. Jika wiring kalian
  berbeda, ubah parameter template `FlexCAN_T4<CAN3, ...>` di
  `include/drivers/GnssHere4Can.h`.
- Perlu **transceiver CAN 3.3V eksternal** (mis. SN65HVD230/TJA1051) di antara pin CAN
  Teensy dan CAN_H/CAN_L Here4 — Teensy tidak punya transceiver bawaan.
- Terminasi 120Ω di kedua ujung fisik bus — pastikan salah satu ujung (biasanya di
  sisi Here4/kabelnya) sudah terminasi, tambahkan satu lagi di sisi Teensy.
- Bitrate default `1,000,000` bps (standar DroneCAN) — verifikasi Here4 tidak
  dikonfigurasi lain sebelum mempercayai default ini.
- Filter data-type dilakukan di level software (`shouldAcceptTransfer`), bukan
  hardware CAN filter — cukup untuk laju pesan GPS yang rendah, tapi bisa
  dioptimasi belakangan jika perlu.

## Kepemilikan dan penjadwalan

```cpp
#include "drivers/GnssHere4Can.h"

fc::Here4GnssReceiver gnss;

void setupGnss()
{
    gnss.begin();
}

void updateGnss()
{
    gnss.poll();  // Menguras semua frame CAN yang tertunda
    if (gnss.isFresh(500000)) {
        const fc::GnssFixData fix = gnss.data();
    }
}
```

## Data yang dihasilkan

- `latitude_deg`/`longitude_deg`: dari field `longitude_deg_1e8`/`latitude_deg_1e8`
  Fix2 — **skala derajat × 1e8**, berbeda dari MAVLink (×1e7) maupun
  `lib/common/Locations.h` yang sudah ada di repo ini (×1e7) — konversi skala sudah
  ditangani di dalam driver, tapi perlu diingat kalau menulis kode baru yang
  membandingkan field mentah.
- `altitude_m`: dari `height_msl_mm` (mean sea level, bukan ellipsoid).
- `velocity_ned_mps[0..2]`: langsung dari `ned_velocity` Fix2 — **kecepatan vertikal
  (down) sungguhan**, sama seperti backend UBX, bukan disintesis dari barometer
  seperti backend NMEA.
- `satellites`: dari `sats_used` Fix2.
- `fix_type`: dari field `status` Fix2 (0=NoFix, 2=Fix2D, 3=Fix3D — nilai 1/time-only
  dipetakan ke NoFix).
- `hdop`: dari `hdop` pada pesan **Auxiliary** (bukan Fix2 — Fix2 hanya punya `pdop`,
  yang tidak didekode modul ini). Auxiliary berprioritas rendah di DroneCAN, jadi
  bisa datang lebih jarang dari Fix2 — `hdop` mungkin sedikit lebih basi dari
  field lain di `GnssFixData` pada saat tertentu.

**Field yang sengaja tidak didekode**: `covariance` dan `ecef_position_velocity` pada
Fix2 (array panjang-variabel, tidak byte-aligned, memerlukan penanganan aturan
*tail-array optimization* DroneCAN yang di luar cakupan kebutuhan navigasi/kendali di
skripsi ini), serta `gdop`/`vdop`/`tdop`/`ndop`/`edop`/`sats_visible` pada Auxiliary
(tidak ada field yang sesuai di `GnssFixData` bersama).

## File terkait

```text
lib/third_party/libcanard/canard.c, canard.h, canard_internals.h   Vendor libcanard (MIT)
lib/third_party/libcanard/LICENSE, README.md                       Lisensi dan dokumentasi vendor
```
