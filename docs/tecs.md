# Refactor TECS (Total Energy Control System)

Algoritma TECS (ArduPilot `AP_TECS`) **tidak diubah** — hanya direstrukturisasi dari
~80 variabel file-scope global + fungsi bebas menjadi `class Tecs`, dengan sensor
snapshot (`AhrsData`, `BarometerData`, `ImuData`, `AirspeedData`) diteruskan sebagai
parameter alih-alih singleton global. Termasuk seluruh "CRITICAL FIX" bawaan program
lama (feedforward height-rate yang dulu selalu nol, adaptive time constant, windup
inhibit saat overshoot altitude) — **dipertahankan persis**, ini adalah tambalan bug
yang sudah teruji, bukan bagian yang perlu di-refactor.

## Struktur modul

```text
include/navigation/Tecs.h   Kontrak class Tecs
src/navigation/Tecs.cpp     Implementasi algoritma TECS (identik dengan lama)
```

## Dua perbaikan bug unit (bukan perubahan algoritma)

1. **NaN/Inf pitch-demand fallback** (`_update_pitch()` dan `update_pitch_throttle()`
   lama): kode lama menetapkan `_pitch_dem = imu.pitch` — `imu.pitch` dalam **derajat**,
   padahal `_pitch_dem` didokumentasikan dan dipakai di seluruh fungsi lain sebagai
   **radian** (dibandingkan dengan `_PITCHminf`/`_PITCHmaxf` yang sudah dikonversi
   `radians()`). Port baru memakai `imu.pitch_rad`. Ini jalur *self-healing* yang hanya
   berjalan saat NaN/Inf terdeteksi (kondisi darurat), tapi tetap salah unit di kode
   asli.
2. **`get_pitch_demand()` mengembalikan derajat, bukan centidegree** — komentar
   program lama bilang "demanded pitch angle in centi-degrees", tapi rumusnya
   `_pitch_dem * 57.295781f` adalah radian→derajat biasa (57.29578 = 180/π, bukan
   5729.58 untuk centidegree), dan pemakainya (`FW_ControlModes.h::calc_nav_pitch()`)
   membandingkannya langsung dengan `pitch_limit_min`/`max` dalam derajat. Method baru
   diberi nama `pitchDemandDeg()` sesuai apa yang benar-benar dikembalikan, bukan
   nama lama yang menyesatkan.

## Kepemilikan dan penjadwalan

`Tecs` adalah kelas komputasi murni, sama seperti `Ahrs`/`L1Controller` — tidak
memegang instance driver, hanya menerima snapshot data tiap panggilan.

```cpp
#include "navigation/Tecs.h"

fc::Tecs tecs;

void update50HzLoop()
{
    tecs.update50Hz(ahrs_data, baro_data, imu_data, airspeed_data);
}

void updateNavigationLoop()
{
    tecs.updatePitchThrottle(hgt_dem_cm, eas_dem_mps, throttle_nudge, hgt_afe,
                              load_factor, ahrs_data, baro_data, imu_data, airspeed_data);
    int32_t throttle_pct100 = tecs.throttleDemand();
    float pitch_deg = tecs.pitchDemandDeg();
}
```

## Logging debug dipindah keluar dari kelas

Program lama mencetak langsung ke `Serial2` dari dalam TECS (`TECS_DEBUG` flag).
Modul baru **tidak melakukan I/O apa pun secara internal** (konsisten dengan
`Imu`/`Barometer`/`Ahrs`/`L1Controller`) — seluruh state yang dulu di-print
(`underspeed()`, `badDescent()`, `heightRateDemandMps()`, dll.) tersedia lewat
getter; lapisan telemetry (Fase 7) yang memutuskan apa dan kapan mencetak.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `update_50hz()` | `tecs.update50Hz(ahrs, baro, imu, airspeed)` |
| `update_pitch_throttle(hgt_cm, eas_cm, nudge, hgt_afe, load_factor)` | `tecs.updatePitchThrottle(hgt_cm, eas_mps, nudge, hgt_afe, load_factor, ahrs, baro, imu, airspeed)` |
| `get_throttle_demand()` | `tecs.throttleDemand()` |
| `get_pitch_demand()` | `tecs.pitchDemandDeg()` (lihat catatan unit di atas) |
| `get_target_airspeed()` | `tecs.targetAirspeedMps(eas2tas)` (menerima `eas2tas` sebagai parameter, bukan `baro.getEAS2TAS()` global) |
| `set_tecs_min_throttle(v)` / `set_tecs_max_throttle(v)` | `tecs.setMinThrottlePercent(v)` / `setMaxThrottlePercent(v)` |
| `use_synthetic_airspeed()` | `tecs.useSyntheticAirspeedOnce()` |
| `TECS_DEBUG` + `Serial2.println(...)` | Dihapus dari kelas; baca lewat getter (`underspeed()`, `badDescent()`, dll.) dari lapisan telemetry |

## File lama yang tidak dimigrasikan

```text
include/TECS.h
```
