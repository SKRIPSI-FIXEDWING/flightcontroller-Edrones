# Refactor L1 Controller

> **2026-10-29: FuzzyL1Tuner dihapus dari firmware** (beserta param `FUZZY_ENABLE`,
> `FUZZY_MIN_PER`, `FUZZY_MAX_PER`, library eFLL, dan kolom log `fuzzy_on`/`fuzzy_out`)
> supaya data L1 konvensional tidak terpengaruh. Kode lamanya ada di riwayat git
> (commit `dc16baa`). Bagian yang menyebut fuzzy di bawah bersifat historis.

Algoritma L1 (ArduPilot `AP_L1_Control`) tidak diubah kecuali sumber arah wahana
(lihat "Perubahan algoritma" di bawah) — selebihnya hanya
direstrukturisasi dari kumpulan fungsi bebas + variabel file-scope global menjadi
`class L1Controller` dengan state sebagai anggota kelas dan sensor snapshot
(`AhrsData`, `ImuData`) diteruskan sebagai parameter, bukan diakses lewat singleton
global (`ahrs`, `imu`) seperti program lama.

## Struktur modul

```text
include/navigation/L1Controller.h   Kontrak class L1Controller
src/navigation/L1Controller.cpp     Implementasi algoritma L1 (identik dengan lama kecuali arah wahana)
```

## `_L1_period` — parameter yang akan disetel fuzzy tuner

Ini adalah **inti skripsi**: `period()`/`setPeriod()` menggantikan `_L1_period` yang
dulunya `float` bebas file-scope, tidak terdaftar di `Params.h`, tidak punya
setter — sekarang jadi anggota `L1ControllerConfig` dengan accessor publik.
`FuzzyL1Tuner` (modul baru, lihat `docs/fuzzy-l1-tuner.md`) akan memanggil
`setPeriod()` tiap loop berdasarkan crosstrack error (`e`) dan laju perubahannya
(`Δe`) — inilah perbandingan "L1 konvensional" (period tetap) vs "L1 + fuzzy"
(period disetel daring) yang jadi topik skripsi.

## Perubahan algoritma: arah wahana dari course GPS (sejak 2026-10-22)

Satu-satunya penyimpangan dari program lama. `updateWaypoint()` lama membentuk
vektor kecepatan dari **yaw** × groundspeed, sehingga error sudut
η = β_L − β_h memakai heading, bukan course. Proposal (rumus 2.2, subbab 2.6.2)
dan `AP_L1_Control` ArduPilot memakai **arah gerak wahana (course χ)**, yaitu
arah vektor kecepatan GPS terhadap bumi.

Dengan angin samping, yaw dan course berbeda sebesar sudut crab. Pada uji
2026-10-22 selisihnya 45–50° di leg WP4→WP5. Akibatnya nav_bearing hampir sama
dengan yaw, nav_roll mendekati 0, padahal pesawat terbang sejajar garis, dan CTE
mencapai 112 m (lihat `tools/flight_analysis/flight_2026-10-22/12_detail_wp4_wp5.png`).

Sekarang `updateWaypoint()` memakai `ahrs.velocity_ned_mps` (NED dari GNSS) bila
`ahrs.gps_lock` dan |v| ≥ `course_min_speed_mps` (3 m/s). Di luar kondisi itu
(diam di darat, dead-reckoning) tetap memakai yaw seperti dulu. Besar vektor
tetap `ground_speed` yang dilantai `min_guidance_speed_mps`, jadi panjang L1 dan
rumus percepatan lateral tidak berubah. `updateLoiter()` sejak awal sudah memakai
vektor GPS; `updateHeadingHold()` tetap memakai yaw karena memang menahan heading.

Data uji sebelum perubahan ini (13–22/10/2026) diterbangkan dengan versi berbasis
yaw, jadi tidak bisa dibandingkan langsung dengan uji sesudahnya.

## Perubahan struktural (bukan perubahan algoritma)

- **Dependency injection, bukan global**: `ahrs.get_position()`/`get_velocity_ned()`/
  `groundspeed` lama → parameter `const AhrsData&`. `imu.yaw`/`rad_yaw`/`rad_pitch`
  lama → parameter `const ImuData&`. `baro.getEAS2TAS()` lama → parameter `eas2tas`
  eksplisit di `turnDistance()`/`loiterRadius()`. `get_target_airspeed()` (yang di
  program lama sebenarnya berasal dari `TECS.h`, bukan `L1_Controller.h` sendiri —
  bekerja hanya karena kebiasaan include-order file header lama) → parameter
  `target_airspeed_mps` eksplisit di `loiterRadius()`/`updateLoiter()`.
- **Guard yang tadinya dead jadi benar-benar berfungsi**: `ahrs.get_position()` di
  program lama **selalu** `return true` tanpa syarat (lihat `docs/ahrs.md`), jadi
  pengecekan `if (ahrs.get_position(...) == false)` di `update_waypoint()`/
  `update_loiter()` lama tidak pernah bercabang. Modul baru mengecek `ahrs.valid`
  (field asli, diisi `Ahrs::update()`) — pengecekan yang sama niatnya, sekarang
  benar-benar bisa bernilai false kalau AHRS belum pernah di-update.
- `grspd` (variabel global debug) → `groundspeedVectorAngle()`.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `_L1_period` (baca/tulis langsung) | `l1.period()` / `l1.setPeriod(v)` |
| `_L1_damping` | `l1.damping()` / `l1.setDamping(v)` |
| `_L1_xtrack_i_gain` | `l1.xtrackIntegratorGain()` / `l1.setXtrackIntegratorGain(v)` |
| `_loiter_bank_limit` | `l1.setLoiterBankLimitDeg(v)` |
| `set_reverse(bool)` | `l1.setReverse(bool)` |
| `update_waypoint(prev, next, dist_min)` | `l1.updateWaypoint(ahrs_data, imu_data, prev, next, dist_min)` |
| `update_loiter(center, radius, dir)` | `l1.updateLoiter(ahrs_data, imu_data, center, radius, dir, eas2tas, target_airspeed_mps)` |
| `update_heading_hold(heading_cd)` | `l1.updateHeadingHold(ahrs_data, imu_data, heading_cd)` |
| `update_level_flight()` | `l1.updateLevelFlight(imu_data)` |
| `nav_roll_cd()` | `l1.navRollCd(imu_data)` |
| `lateral_acceleration()` | `l1.lateralAcceleration()` |
| `nav_bearing_cd()` / `bearing_error_cd()` / `target_bearing_cd()` | `l1.navBearingCd()` / `bearingErrorCd()` / `targetBearingCd()` |
| `turn_distance(wp_radius)` / `(wp_radius, turn_angle)` | `l1.turnDistance(wp_radius, eas2tas)` / `(wp_radius, turn_angle, eas2tas)` |
| `loiter_radius(radius)` | `l1.loiterRadius(radius, eas2tas, target_airspeed_mps)` |
| `reached_loiter_target()` | `l1.reachedLoiterTarget()` |
| `data_is_stale()` / `set_data_is_stale()` | `l1.dataIsStale()` / `l1.setDataIsStale()` |
| `grspd` | `l1.groundspeedVectorAngle()` |

## File lama yang tidak dimigrasikan

```text
include/L1_Controller.h
include/LQR_PF.h   (nama menyesatkan — isinya salinan lama algoritma L1 yang di-comment total, bukan LQR; lihat catatan di Fase 4/docs/attitude-lqr.md)
```
