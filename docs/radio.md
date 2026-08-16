# Refactor Radio (RC Input, SBUS-only)

Sesuai keputusan Anda: **SBUS saja**. Program lama punya **dua sumber RC yang
aktif bersamaan tanpa arbitrasi** — SBUS langsung (`Radio.h`) dan
ELRS-over-MAVLink (`Mavlink.h::applyRcToGlobals()`), keduanya menulis global
yang sama (`ch_roll`, `ch_throttle`, `arming`, `mode_now`, dst.) tanpa
prioritas — race condition nyata yang diidentifikasi saat audit awal. Modul
baru ini **tidak** mem-port jalur MAVLink RC-override; kalau nanti terbukti
dipakai (radio telemetry ELRS untuk RC passthrough), tambahkan sebagai sumber
kedua yang diarbitrase eksplisit (SBUS utama, override hanya saat sinyal SBUS
hilang) — jangan direplikasi race condition-nya.

## Struktur modul

```text
include/vehicle/Radio.h   Kontrak class Radio
src/vehicle/Radio.cpp     Implementasi SBUS + decode mode + failsafe
```

## Perilaku yang WAJIB dipertahankan (bukan bug, fitur keselamatan)

- **`begin()` blocking**: menolak lanjut selama transmitter armed atau sinyal
  hilang saat boot — interlock pre-flight yang disengaja. Jangan diubah jadi
  non-blocking tanpa pengecekan pre-arm pengganti di tempat lain.
- **Failsafe kehilangan sinyal RC**: jika armed & sinyal hilang, `mode_now`
  **dipaksa ke 3 (AUTO)** sehingga wahana lanjut mengikuti misi/RTL alih-alih
  jatuh ke mode manual tanpa stick input — perilaku keselamatan kritis,
  dipertahankan persis.
- **`Serial.println()` di `begin()`/failsafe**: satu-satunya modul di refactor
  ini yang sengaja melakukan I/O langsung — ini bukan debug logging, tapi
  umpan balik operasional ke pilot saat interlock keselamatan aktif (beda
  konteks dari driver sensor yang sengaja senyap).

## Kepemilikan dan penjadwalan

```cpp
#include "vehicle/Radio.h"

fc::Radio radio(Serial8);

void setupRadio()
{
    radio.begin();  // blocking pre-flight interlock
}

void updateRadio()
{
    radio.update();
    if (radio.armed()) { ... }
}
```

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `remote_setup()` | `radio.begin()` |
| `remote_loop()` | `radio.update()` |
| `ch_roll`/`ch_pitch`/`ch_throttle`/`ch_yaw` | `radio.channelRoll()`/`channelPitch()`/`channelThrottle()`/`channelYaw()` |
| `ch_mode`/`ch_mode_backup`/`ch_vehicle_mode` | `radio.channelMode()`/`channelModeBackup()`/`channelVehicleMode()` |
| `arming` | `radio.armed()` |
| `signal_lost` | `radio.signalLost()` |
| `failsafe_active` | `radio.failsafeActive()` |
| `mode_now` | `radio.modeNow()` |
| `outputScaler(ch)` | `fc::Radio::outputScaler(ch)` (static) |

## File lama yang tidak dimigrasikan

```text
include/Radio.h
```

Flag mode copter (`mode_hover`, `pos_hold`, `transition_phase1/2/3`, `mode_vtol`,
`mode_vtol_plane`) dari program lama **tidak ada padanannya** — modul ini hanya
mengekspos apa yang relevan untuk fixed-wing.
