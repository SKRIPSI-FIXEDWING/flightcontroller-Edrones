# Refactor MAVLink Telemetry

Port `Mavlink.h` lama (1389 baris) menjadi `class Mavlink`, beroperasi di atas
`VehicleContext` (Fase 6) alih-alih singleton global. Tiga port dipertahankan
persis: USB/`Serial` (GCS), radio telemetry/`Serial2`, companion computer
(RPi)/`Serial7` -- termasuk gating upload-misi (pesan non-misi ditahan ke RPi
selagi upload berlangsung).

## Struktur modul

```text
include/communication/Mavlink.h   Kontrak class Mavlink
src/communication/Mavlink.cpp     Implementasi handler + sender
include/vehicle/Battery.h         Sensor baterai nyata (baru dipakai Mavlink)
src/vehicle/Battery.cpp
```

## Yang TIDAK di-port (keputusan keselamatan, bukan kelalaian)

1. **ELRS-over-MAVLink RC override** (`RC_CHANNELS_OVERRIDE`/`RC_CHANNELS`/
   `RC_CHANNELS_RAW` -> `applyRcToGlobals()`). Ini sumber RC kedua yang dulu
   race dengan SBUS tanpa arbitrasi (diidentifikasi saat audit awal). Sesuai
   keputusan Anda di Fase 5, refactor ini **SBUS-only** -- lihat
   `docs/radio.md`.
2. **`MAV_CMD_COMPONENT_ARM_DISARM` tidak benar-benar mengubah apa pun.**
   Selalu dibalas `MAV_RESULT_UNSUPPORTED` + status-text penjelasan. Program
   lama membiarkan GCS meng-arm pesawat murni lewat MAVLink, menulis global
   `arming` yang SAMA dengan yang ditulis switch RC -- tanpa arbitrasi, mirip
   persis race condition RC ganda. Menambahkan jalur tulis kedua ke flag
   sekritis "arming" tanpa diminta eksplisit bertentangan dengan arah refactor
   ini di tempat lain. Kewenangan arming **sepenuhnya** di switch SBUS
   (Radio, Fase 5).
3. **Baterai palsu (sine-wave osilasi)** yang dulu dikirim `BATTERY_STATUS`
   (komentar asli sendiri: `// Osilasi naik-turun 10.5V – 12.6V mengikuti
   waktu`) -- ini jelas stub uji coba, tidak pernah tersambung ke sensor
   ADC nyata (`Voltage.h`'s `read_voltage()`/`update_voltage()`, yang
   sendiri juga tidak dipanggil dari mana pun di program lama). Refactor ini
   membangun `Battery` (baru, membaca `A13` sungguhan) dan menyambungkannya
   -- pemilik kendaraan (Fase 8) yang memanggil `battery.update()` tiap
   loop.

## `MAV_CMD_PREFLIGHT_CALIBRATION`: baro saja, sisanya `UNSUPPORTED`

Ditambahkan setelah karakterisasi drift ground-pressure (`docs/barometer-ms5611.md`)
menunjukkan referensi tekanan tanah bisa hanyut ~1 hPa/jam kalau ada jeda
antara boot dan lepas landas. `handleCommandLong()` menangani perintah ini
HANYA untuk `param3 == 1` (kalibrasi baro) -- persis perintah yang dikirim
Mission Planner/QGroundControl saat operator memicu kalibrasi pra-terbang,
jadi tidak ada perubahan yang dibutuhkan di sisi GCS. Ditolak
(`MAV_RESULT_DENIED`) kalau `Radio::armed()` true. Parameter kalibrasi sensor
lain (gyro/mag/accel/radio/ESC) pada perintah yang sama dibalas
`MAV_RESULT_UNSUPPORTED` -- tidak diimplementasikan.

## `DO_SET_MODE`: lewat `ModeManager`, bukan tulis field langsung

Program lama menulis `mode_now`/`current` (global) langsung saat menerima
`DO_SET_MODE`, TIDAK memanggil hook `enter()`/`exit()` mode manapun secara
konsisten dengan jalur RC. Modul baru memanggil
`ctx.mode_manager.setMode(new_mode_id)` -- memastikan `_enter()`/`_exit()`
tiap mode selalu terpanggil, dari sumber mana pun (RC atau GCS).

## Debug logging dipangkas

Program lama mengirim `sendStatusText()` "DEBUG WP: ..." di HAMPIR SETIAP
langkah handler misi (permintaan waypoint, item diterima, dst) -- pola debug
logging berlebihan. Modul baru mempertahankan alur protokol (request/ack)
persis sama, tapi tanpa narasi debug per-langkah; status akhir (`"Mission
upload complete"`, dll.) tetap dikirim.

## Kepemilikan dan penjadwalan

```cpp
#include "communication/Mavlink.h"

fc::Mavlink mavlink;

void updateTelemetry(fc::VehicleContext& ctx)
{
    mavlink.handlePorts(ctx);  // parse semua port masuk
    mavlink.update(ctx);       // kirim telemetry periodik + pump param stream
}

// Didaftarkan ke Navigation (Fase 3) saat inisialisasi kendaraan (Fase 8):
navigation.setWaypointReachedCallback(
    [](uint16_t seq, void* user_data) {
        static_cast<fc::Mavlink*>(user_data)->notifyWaypointReached(seq);
    },
    &mavlink);
```

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `MavlinkHandler::handlePorts()` | `mavlink.handlePorts(ctx)` |
| `MavlinkHandler::mavlinkTask()` | `mavlink.update(ctx)` |
| `MavlinkHandler::notifyWaypointReached(seq)` | `mavlink.notifyWaypointReached(seq)` |
| `applyRcToGlobals(...)` + handler ID 70/65/35/109 | **Dihapus** -- lihat "Yang TIDAK di-port" #1 |
| `handleCommandLong()`'s ARM_DISARM branch | **Dibatasi** -- lihat "Yang TIDAK di-port" #2 |
| `sendBatteryStatus()` dengan nilai palsu | `Battery::update()` + `sendBatteryStatus()` dengan data nyata |

## File lama yang tidak dimigrasikan

```text
include/Mavlink.h
include/Mavlink_old.h              (sudah dikonfirmasi dead file saat audit awal)
include/RaspberryPi_Telemtry.h      (superseded MAVLink terstruktur ke port RPi yang sama)
include/Telemetry.h                (referensi gcs rusak, menyetel gain yang salah/tidak ada)
include/Voltage.h                  (nama file saja; implementasi asli sebenarnya di controlinfo.h, sekarang digantikan vehicle/Battery.h)
```
