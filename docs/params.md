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

- **`NAVL1_PERIOD`, `NAVL1_DAMPING`, `NAVL1_XTRACK_I`** (dulu `L1_PERIOD`, `L1_DAMPING`, `L1_XTRACK_I`; lihat bagian "Nama param & deskripsi Mission Planner") -- di program lama, `_L1_period`
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

`Params` sekarang menempati `[1000, 1140)`, bukan `[650, 782)` seperti
sebelumnya. Alamat lama tumpang tindih penuh dengan `storage/Waypoints.h`
(`[0, 808)` untuk `kMaxEepromWaypoints=50 * sizeof(Locations)=16` byte) —
setiap `Waypoints::save()` menimpa data `Params`, dan sebaliknya, tanpa ada
yang menyadarinya karena kedua modul independen dan tidak saling tahu
alamat masing-masing.

Peta EEPROM saat ini (Teensy 4.1, `kTeensy41EepromSize=4284`):

| Modul | Rentang alamat |
| --- | --- |
| `storage/Waypoints.h` | `[0, 808)` |
| `storage/Params.cpp` | `[1000, 1140)` |
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

## `WP_RADIUS` -- radius klaim waypoint dari Mission Planner (2026-10-17)

`WP_RADIUS` (default 15 m, rentang 2-100 m) mengisi
`NavigationConfig::wp_radius_default_m`. Namanya sama dengan ArduPlane, jadi
kotak **WP Radius** di tab Flight Plan Mission Planner membaca dan
menulisnya langsung (MP hanya mengaktifkan kotak itu bila param `WP_RADIUS`
ada di daftar param). Berbeda dengan param lain, nilainya berlaku **langsung**:
`Mavlink::handleParamSet()` meneruskannya ke `Navigation::setWaypointRadius()`.
Radius per-WP (MAVLink `param2 > 0`) tetap diutamakan.

WP tetap dianggap tercapai bila wahana melewati *finish-line* (garis tegak
lurus leg di titik WP), walaupun di luar radius -- radius hanya salah satu
syarat klaim.

`WP_RADIUS` diletakkan di **akhir** tabel (`kMaxParams = 33`). `Params::load()`
membaca EEPROM berdasarkan **urutan**, bukan nama: menyisipkan param di tengah
menggeser nilai semua param sesudahnya (terjadi saat `AIL_OUT_SIGN` disisipkan;
log 2026-10-15 menunjukkan `ROLL_KP`..`YAW_KP` tergeser satu slot). Setelah
flashing firmware dengan param baru di akhir, EEPROM lama naik skema dengan
aman (`SchemaUpgraded`): param lama tidak bergeser, param baru memakai default.

## Nama param & deskripsi Mission Planner (2026-10-18)

Protokol MAVLink tidak membawa teks deskripsi param (`PARAM_VALUE` hanya
berisi nama, nilai, tipe). Kolom **Description**, **Units**, dan **Range** di
*Config > Full Parameter List* diisi Mission Planner dari file definisi
ArduPlane (`C:\ProgramData\Mission Planner\ArduPlane.apm.pdef.xml`), yang
diunduh ulang otomatis oleh Mission Planner. Karena dicocokkan berdasarkan
**nama**, hanya param bernama sama dengan ArduPlane yang punya keterangan;
mengedit file itu sendiri tidak awet (tertimpa saat MP memperbarui).

Karena itu param L1 memakai nama ArduPlane:

| Param | Dulu | Default | Keterangan |
| --- | --- | --- | --- |
| `NAVL1_PERIOD` | `L1_PERIOD` | 22 s | periode loop L1 -- penentu utama agresivitas belok di AUTO |
| `NAVL1_DAMPING` | `L1_DAMPING` | 0.707 | rasio redaman L1 (nilai tetap proposal) |
| `NAVL1_XTRACK_I` | `L1_XTRACK_I` | 0.2 | gain integrator cross-track |
| `WP_RADIUS` | -- | 15 m | radius klaim WP (lihat bagian `WP_RADIUS`) |
| `ROLL_LIMIT_DEG` | -- (tetap 36) | 45 deg | batas bank perintah navigasi AUTO, berlaku langsung |

Catatan:
- Nilai yang tersimpan di EEPROM ikut terbawa (EEPROM dimuat berdasarkan
  urutan, bukan nama). Default di atas hanya berlaku pada EEPROM kosong.
- Setelah flashing, tekan **Refresh Params** di MP agar nama lama hilang.
- Range MP untuk `NAVL1_XTRACK_I` adalah 0-0.1 (ArduPlane); nilai 0.2 di sini
  akan ditandai di luar rentang oleh MP, tetapi tetap diterima firmware
  (batas firmware 0-1; integrator sendiri dijepit +-0.1 rad di L1Controller).
- ~~Param L1 berlaku setelah reboot~~ (diubah 2026-10-25): `NAVL1_PERIOD`,
  `NAVL1_DAMPING`, `NAVL1_XTRACK_I`, `FUZZY_ENABLE`, `FUZZY_MIN_PER`,
  `FUZZY_MAX_PER`, `WP_RADIUS`, `ROLL_LIMIT_DEG` dan `DIST_ENABLE` berlaku
  LANGSUNG (lihat bagian di bawah). Param lain tetap butuh reboot.

## `ROLL_LIMIT_DEG` -- batas bank AUTO (2026-10-19)

Mengisi `NavigationConfig::roll_limit_deg`: `nav_roll` dari L1 dijepit ke
+-`ROLL_LIMIT_DEG` sebelum masuk kendali attitude (Navigation.cpp,
`updateAutoAttitudeTargets()`). Default 45 deg (dulu tetap 36 = HEAD_MAX
Khageswara), rentang 20-60 deg, berlaku langsung tanpa reboot. Hanya AUTO;
FBWA tetap memakai pemetaan stick. Ditambahkan di akhir tabel
(`kMaxParams = 34`), jadi EEPROM lama naik skema dan param ini mengambil
default 45 saat boot pertama.

Bank lebih besar = radius belok lebih kecil (R = v^2 / (g tan(bank))), tetapi
faktor beban naik (1/cos 45 = 1.41, kecepatan stall naik ~19%) dan wahana lebih
mudah kehilangan ketinggian saat belok.

## `FUZZY_ENABLE`, `DIST_ENABLE`, dan param L1 live (2026-10-25)

Dua param baru di **akhir** tabel (`kMaxParams` 34 → 40; EEPROM lama otomatis
memberi nilai default pada slot baru):

| Param | Default | Fungsi |
|---|---|---|
| `FUZZY_ENABLE` | `FC_FUZZY_L1_ENABLE` (0) | 1 = fuzzy self-tuning periode L1; 0 = L1 periode tetap = `NAVL1_PERIOD` |
| `DIST_ENABLE` | 0 | 1 = `DisturbanceInjector` boleh menyuntik gangguan roll di AUTO (`docs/disturbance-injection.md`) |

`Mavlink::handleParamSet()` meneruskan perubahan langsung (di bawah
`ControlLock`) ke objek yang memakainya, tanpa reboot:
`NAVL1_PERIOD` → `L1Controller::setPeriod()` + `FuzzyL1Tuner::setBasePeriod()`;
`NAVL1_DAMPING` / `NAVL1_XTRACK_I` → setter L1; `FUZZY_ENABLE` →
`FuzzyL1Tuner::applyEnabled()`; `FUZZY_MIN_PER` / `FUZZY_MAX_PER` →
`setPeriodLimits()`; `DIST_ENABLE` dibaca langsung oleh `DisturbanceInjector`.
Jika `ControlLock` tidak didapat dalam 30 ms, nilai tetap tersimpan di EEPROM
dan GCS mendapat peringatan "PARAM: busy, applies after reboot".

Hasil nyata tercatat di log SD (`# EFFECTIVE`, event `PARAM`, kolom
`l1_period_s`/`fuzzy_on`), jadi tidak perlu lagi memotret layar parameter.
