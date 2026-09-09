# Refactor Params (parameter EEPROM)

Rebuild FW-only dari `Params.h` lama sebagai `class Params` (bukan static
global array + kumpulan `inline` function). Skema EEPROM baru sepenuhnya
(magic number `0xFC01`, beda dari `0xCDAC` lama) -- **data EEPROM lama tidak
kompatibel dan tidak akan ter-load**, dianggap tidak valid dan diganti default,
karena layout parameternya sudah beda total.

## Yang dihapus

- **Semua param copter** (`CP_STAB_*`, `CP_RATE_*`, `CP_ALT_P`, `CP_ZVEL_P`) --
  tidak relevan, tidak ada copter lagi.
- **`MODEL_UAV`** -- tidak ada lagi switch tipe wahana; build ini FW-only
  secara compile-time.
- **`FW_ROLL/PITCH/YAW/THR_P/I/D`** (gain PID lama) -- digantikan `K` LQR yang
  **tetap, dihitung offline** (lihat `docs/attitude-lqr.md`) -- **sengaja
  tidak diekspos** sebagai param EEPROM, karena keputusan Anda adalah `K`
  tetap/tidak berubah saat runtime.
- **`FW_ATHR_MIN/MAX`** (batas PWM auto-throttle `FW_CONTROL`) -- `FW_CONTROL`
  sendiri sudah tidak di-port.
- **Pin/channel servo** (`SERVO_AIL_L` dst, `MOTOR_1-4_PIN`) -- lihat
  `docs/actuator.md`, pin jarang perlu di-tuning tanpa reflash.

## Yang ditambahkan

- **`L1_PERIOD`, `L1_DAMPING`, `L1_XTRACK_I`** -- di program lama, `_L1_period`
  adalah `float` bebas file-scope, **tidak terdaftar sebagai param sama
  sekali** (tidak ada expose EEPROM). Sekarang bisa di-tuning tanpa reflash.
- **`FUZZY_MIN_PER`, `FUZZY_MAX_PER`** -- batas skala period fuzzy tuner (inti
  skripsi).

## Catatan desain: `AIRSPEED_CRUISE`/`MIN_AIRSPEED`/`MAX_AIRSPEED`

Param ini didaftarkan menunjuk ke field `TecsConfig` (satu-satunya sumber
otoritatif), **bukan** didaftarkan dua kali ke `NavigationConfig`/
`AttitudeControllerConfig` juga (yang masing-masing punya field serupa untuk
keperluan internal mereka sendiri). Ini untuk menghindari EEPROM menulis ke
banyak salinan yang bisa desinkron. Saat inisialisasi kendaraan (Fase 8),
pemilik vehicle bertanggung jawab menyalin nilai `TecsConfig` yang sudah
di-load ke config lain yang butuh nilai sama, sebelum konstruksi objek
tersebut selesai.

## Kepemilikan

```cpp
#include "storage/Params.h"

fc::L1ControllerConfig l1_config;
fc::FuzzyL1TunerConfig fuzzy_config;
fc::TecsConfig tecs_config;

fc::Params params;
params.initFixedWing(l1_config, fuzzy_config, tecs_config);
fc::ParamLoadResult result = params.load();  // isi l1_config/fuzzy_config/tecs_config dari EEPROM (atau default)

// Baru SEKARANG konstruksi objek yang memakai config ini:
fc::L1Controller l1(l1_config);
fc::FuzzyL1Tuner fuzzy_tuner(fuzzy_config);
fc::Tecs tecs(tecs_config);
```

Tidak ada `Serial.print`/`printf` internal (beda dari program lama yang
mencetak di hampir setiap operasi) -- `load()` mengembalikan
`ParamLoadResult` (`LoadedFromEeprom`/`NoValidEepromData`/`SchemaUpgraded`),
pemanggil yang mencetak/mencatat jika perlu.

## Alamat EEPROM (diperbaiki 2026-08-20)

`Params` sekarang menempati `[1000, 1132)`, bukan `[650, 782)` seperti
sebelumnya. Alamat lama tumpang tindih penuh dengan `storage/Waypoints.h`
(`[0, 808)` untuk `kMaxEepromWaypoints=50 * sizeof(Locations)=16` byte) —
setiap `Waypoints::save()` menimpa data `Params`, dan sebaliknya, tanpa ada
yang menyadarinya karena kedua modul independen dan tidak saling tahu
alamat masing-masing.

Peta EEPROM saat ini (Teensy 4.1, `kTeensy41EepromSize=4284`):

| Modul | Rentang alamat |
| --- | --- |
| `storage/Waypoints.h` | `[0, 808)` |
| `storage/Params.cpp` | `[1000, 1132)` |
| `storage/ImuCalibrationStorage.h` | `[2000, 2022)` |

Konsekuensi upgrade: begitu firmware ini pertama kali boot, magic number di
alamat 1000 tidak akan cocok (data lama di sana, kalaupun ada, adalah bekas
tulisan `Waypoints` di rentang lamanya yang kebetulan tumpang tindih dengan
alamat baru `Params`) — `Params::load()` sudah menangani ini secara aman
lewat jalur `NoValidEepromData` yang sudah ada (reset ke default lalu
simpan), tidak ada perubahan kode yang perlu dilakukan untuk migrasi ini,
tapi tuning param yang tersimpan sebelum perbaikan ini akan hilang sekali.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `initParams()` | `params.initFixedWing(l1_config, fuzzy_config, tecs_config)` |
| `loadParamsFromEEPROM()` | `params.load()` |
| `saveParamsToEEPROM()` | `params.save()` |
| `resetParamsToDefault()` | `params.resetToDefaults()` |
| `setParamValue(name, val)` | `params.setValue(name, val)` |
| `getParamValue(name, &val)` | `params.getValue(name, val)` |
| `printParams()` | Iterasi `params.count()`/`params.entryAt(i)`, dicetak pemanggil |

## File lama yang tidak dimigrasikan

```text
include/Params.h
```
