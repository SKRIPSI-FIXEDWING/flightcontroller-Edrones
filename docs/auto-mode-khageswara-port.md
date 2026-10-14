# Evaluasi mode AUTO: penyamaan dengan Khageswara (2026-10-14)

Dokumen ini mencatat keputusan hasil evaluasi AUTO dan di mana masing-masing
diterapkan. Semua fitur khusus Edrones dimatikan secara default lewat flag di
`include/FC_Config.h`; nyalakan dengan mengubah flag-nya.

## Ringkasan keputusan

| Hal | Keputusan | Lokasi |
|---|---|---|
| Throttle AUTO | Manual (stik pilot) | `ModeAuto::_update()` |
| Attitude AUTO | Sama dengan FBWA Khageswara (P/PD) | `FbwaKhageswaraController::updateAuto()` |
| Batas pitch | -10 / +30 derajat | `TecsConfig`, `NavigationConfig` |
| Throttle TECS min/max | 30 / 85 % | `TecsConfig`, `Params.cpp` (TECS_THR_MIN/MAX) |
| Acceptance WP | **Tetap** = radius per-WP (param2) atau 40 m; tidak bergantung L1 (adil untuk L1 vs L1+fuzzy) | `Navigation::navigate()` |
| Bobot kecepatan TECS | `TECS_SPD_WGT = 0`: pitch hanya menahan tinggi (throttle manual) | `TecsConfig::speed_weight`, `Params.cpp` |
| Arming MAVLink | Ada; RC switch edge-triggered | `Mavlink.cpp`, `Radio::setArmedFromGcs()` |
| Set WP aktif / Start mission | Ada (`MISSION_SET_CURRENT`, `MAV_CMD_MISSION_START`) | `Navigation::setCurrentWaypoint()` |
| DO_SET_MODE | Jujur: ACCEPTED hanya bila mode benar-benar berubah | `Mavlink.cpp` |
| RC override MAVLink | Tidak ada (sengaja) | - |
| Thread safety | `ControlLock` (mutex) antara task Control dan Mavlink | `include/vehicle/ControlLock.h` |
| Radius per-WP | dari MAVLink `param2`, tidak disimpan EEPROM | `MissionState::waypoint_radius_m` |
| HOME | tidak ditimpa saat masuk AUTO; di-latch saat ARMING bila Fix3D | `Navigation::updateHomeAndPosition()` |
| Akhir misi | tidak melakukan apa-apa (`mission_done_behavior = 0`) | `Navigation::startMissionDone()` |
| RTL | kode siap, **tidak terdaftar/tidak dapat dipilih** | `ModeRtl`, `FC_RTL_MODE_ENABLE = 0` |
| Kecepatan adaptif belokan | mati (target = AIRSPEED_CRUISE) | `FC_ADAPTIVE_WP_AIRSPEED_ENABLE = 0` |
| FuzzyL1Tuner | mati (periode L1 tetap = `L1_PERIOD`) | `FC_FUZZY_L1_ENABLE = 0` |
| Penolakan masuk AUTO | mati (tidak menolak) | `FC_AUTO_REJECT_INVALID_MISSION = 0` |
| Pengali groundspeed ground test | `FC_GROUNDSPEED_TEST_SCALE` (default 1) | `Ahrs.cpp` |

## Home

- `arm_edge` (disarm -> arm): `home_pending_on_arm_ = true`.
- Selama armed dan `fix_type == Fix3D` dan pending: `Ahrs::setHome()` sekali,
  lalu pending dibersihkan. Bila belum 3D saat arming, home di-latch pada fix 3D
  pertama setelah arming.
- Disarm lalu arming lagi: di-latch ulang pada posisi saat itu.
- Sebelum arming (disarmed) home tetap mengikuti posisi tiap 5 s (syarat:
  `baro <= 10 m`), supaya home sudah ada dan tampil di GCS.
- Tidak pernah berubah saat armed setelah di-latch, termasuk saat masuk AUTO.
- Catatan: ketinggian TECS tidak terikat ke home, melainkan ke nol barometer
  (atau fusi) saat boot. Memindahkan wahana ke tempat dengan elevasi berbeda
  tidak menggeser acuan ketinggian.

## Thread safety: ControlLock

Dua task berprioritas sama (5) memakai data yang sama: Control membaca
`waypoint[]`/`wp_sum`/`flag_wp` dan menjalankan `ModeManager::update()`;
Mavlink menulis daftar waypoint dan memanggil `ModeManager::setMode()`.
FreeRTOS membagi waktu di antara task prioritas sama, sehingga satu task bisa
terpotong di tengah operasi dan task lain melihat data setengah jadi.

`fc::ControlLock` adalah satu mutex. Control menahannya selama satu siklus
(`applyRcModeSwitch()` + `update()`); handler MAVLink menahannya hanya saat
menulis state/mode, dan tidak saat menulis EEPROM atau mengirim paket. Bila
lock tidak didapat dalam 30 ms, aksi dibatalkan dan GCS dijawab gagal.

## Mode switch RC: edge-triggered

Saklar RC hanya bertindak saat posisinya BERUBAH (dan pada pembacaan pertama
setelah boot). Mode yang di-set dari GCS bertahan sampai saklar digerakkan.
Hal yang sama berlaku untuk saklar arming (`Radio::update()`).

## Arming via MAVLink

- Arm ditolak bila belum ada fix GNSS (kecuali `param2 = 21196`) atau throttle
  stik > 1100 us.
- Disarm ditolak bila `ground_speed > 5 m/s` (kecuali `param2 = 21196`).
- Perintah GCS tidak ditimpa frame RC berikutnya; menggerakkan saklar RC
  selalu mengambil alih.

## Mengaktifkan RTL (hanya atas perintah)

1. Set `FC_RTL_MODE_ENABLE 1` (build flag atau `FC_Config.h`).
2. Mode terdaftar dan dapat dipilih lewat MAVLink DO_SET_MODE `custom_mode = 11`.
   Tombol RC belum dipetakan.
3. Masuk RTL ditolak (mode lama dipulihkan) bila home belum di-latch.

## Ground test dengan pengali groundspeed

`FC_GROUNDSPEED_TEST_SCALE` (bilangan bulat) mengalikan `groundspeed_mps` dan
komponen horizontal `velocity_ned_mps` di `Ahrs::updateDriftCorrectedVelocity()`.
Contoh: `-DFC_GROUNDSPEED_TEST_SCALE=1000`. Build memberi `#warning` bila
nilainya bukan 1. **Wajib 1 untuk terbang.**

Catatan: L1 sudah punya lantai kecepatan `min_guidance_speed_mps = 8`, jadi
arah aileron biasanya sudah terlihat di bangku tanpa pengali. Pengali berguna
bila ingin bank mendekati batas ±36 derajat di bangku.
