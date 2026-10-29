# Refactor Navigation (orkestrasi AUTO/GUIDED)

Menggabungkan `Navigation.h` dan `Auto_setup.h` lama menjadi satu `class Navigation`
yang memegang satu `MissionState` (menggantikan kumpulan variabel global lepas), dan
memanggil `L1Controller`/`Tecs` langsung (bukan `calc_nav_roll()`/`calc_nav_pitch()`
terpisah di `FW_ControlModes.h` yang menulis global `nav_roll_deg`/`nav_pitch_deg`).

## Struktur modul

```text
include/navigation/MissionState.h   Struct state misi (pengganti Auto_setup.h globals)
include/navigation/Navigation.h     Kontrak class Navigation
src/navigation/Navigation.cpp       Implementasi orkestrasi (identik dengan logika lama)
```

## Yang BELUM di-port: takeoff

`auto_takeoff_mode`/`auto_takeoff_state` (logika takeoff otomatis) **sengaja belum
dimasukkan** ke `MissionState`/`Navigation` — `takeoff.h` terikat erat dengan
`Actuator.h` dan `Mode_setup.h` yang belum di-refactor (Fase 5/6). Akan ditambahkan
bersamaan saat kedua modul itu di-port, bukan diasumsikan sekarang.

## Dependency injection, bukan global

- `ModeId::GUIDED` check lama → parameter `bool is_guided_mode` (`Mode` belum
  di-port; Fase 6 akan meneruskan hasil pengecekan mode aktifnya sebagai boolean ini).
- `notifyWaypointReached(seq)` (fungsi global, dulu diimplementasikan di `Mavlink.h`)
  → `WaypointReachedCallback` yang didaftarkan lewat `setWaypointReachedCallback()`,
  supaya `Navigation` tidak perlu tahu apa pun tentang MAVLink. Fase 7 akan
  mendaftarkan callback yang mengirim `MISSION_ITEM_REACHED`.
- `mode_fbwa = true` (global `Radio.h`, dipanggil saat misi selesai) → **tidak
  ditulis langsung**. Pemanggil (lapisan Mode, Fase 6) bertanggung jawab memeriksa
  `mission.auto_navigation_mode` beralih ke `false` dan memutuskan pindah mode
  sendiri — `Navigation` tidak boleh menulis state RC/mode yang bukan miliknya.
- `arming`/`ch_throttle`/`scaleToPercent()` (Radio.h/Actuator.h, Fase 5) →
  parameter `bool armed`, `int16_t throttle_stick_percent` eksplisit.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `nav_gps()` | `navigation.updateHomeAndPosition(ahrs, gnss_data, baro_data, armed, now_ms)` |
| `update_wp_nav()` | `navigation.updateWaypointNav(l1, ahrs_data, imu_data)` (dipanggil internal oleh `navigate()` juga) |
| `update_alt()` | `navigation.updateAltitude(tecs, ahrs_data, imu_data, baro_data, airspeed_data, throttle_stick_percent, enable_nudge)` |
| `update_speed_height()` | `navigation.updateSpeedHeight(tecs, ahrs_data, baro_data, imu_data, airspeed_data)` |
| `get_adaptive_wp_airspeed(...)` | `navigation.getAdaptiveWaypointAirspeed(...)` |
| `updateAuto_FW()` / `calc_nav_roll()` / `calc_nav_pitch()` | `navigation.updateAutoAttitudeTargets(l1, tecs, imu_data)`, hasil di `navigation.state().nav_roll_deg`/`nav_pitch_deg` |
| `navigate()` | `navigation.navigate(is_guided_mode, l1, ahrs_data, imu_data, eas2tas)` |
| `set_next_WP(loc)` | `navigation.setNextWaypoint(loc)` |
| `get_next_ground_course(default)` | `navigation.getNextGroundCourse(default)` |
| `waypoint[]` / `wp_sum` / `flag_wp` / `auto_state` / `target_altitude` | `navigation.state().waypoint[]` / `.wp_sum` / `.flag_wp` / `.auto_state` / `.target_altitude` |
| `rpi_external_setpoint_*` | `navigation.state().rpi_external_setpoint_*` (diisi Mavlink.h saat Fase 7) |
| `calc_groundspeed_undershoot()` | `navigation.groundspeedUndershoot()` (dihitung internal tiap `updateHomeAndPosition()`) |

## Antisipasi belokan (`WP_TURN_ANT`, 2026-10-26)

Opsional, **default mati**. Tanpa fitur ini WP diklaim bila jarak ≤ radius
tetap (`WP_RADIUS` atau radius per-WP param2) atau wahana melewati garis
finish. Pada belokan tajam pesawat baru mulai berbelok setelah WP, sehingga
melebar sekitar satu radius belok R. Uji 2026-10-26: v ≈ 20 m/s, bank 40°,
R ≈ 49 m, simpangan terukur 52–56 m di setiap belokan 90°.

Bila `WP_TURN_ANT` = 1 atau 2, `Navigation::navigate()` memperlebar jarak
klaim WP aktif menjadi

```text
R      = v_g^2 / (g · tan(ROLL_LIMIT_DEG))      v_g = ahrs.groundspeed_mps
d_full = R · tan(|Δψ| / 2)                      Δψ = next_turn_angle_deg (setupTurnAngle)
mode 2: d = d_full                              (fillet penuh)
mode 1: d = min(d_full, sqrt(r² + 2·r·R))       (lewat dalam radius r)
acc    = max(r, min(d, WP_TURN_MAX))            r = WP_RADIUS / radius per-WP
```

`d_full` adalah jarak awal busur *fillet* yang menyinggung kedua leg (Beard &
McLain, *Small Unmanned Aircraft*, bab path manager; sama dengan *turn
anticipation* WP fly-by pada RNAV). Busur itu memotong sudut dan lewat
R·(1/cos(Δψ/2) − 1) dari WP: sekitar 20 m untuk Δψ = 90°, R = 49 m. Jadi
**mode 2 tidak menjamin wahana masuk radius r**.

**Mode 1 (direkomendasikan bila wahana wajib masuk radius r):** busur yang
dimulai a meter sebelum WP dan menyinggung leg masuk berpusat di (−a, R)
relatif WP, sehingga jarak terdekatnya ke WP √(a² + R²) − R, untuk Δψ
berapa pun. Syarat ≤ r memberi a ≤ √(r² + 2rR). Sisa belokan menjadi
overshoot ≈ R − a (untuk Δψ = 90°). Contoh r = 15 m, bank 40°, Δψ = 90°:

| v_g | R | d (mode 1) | lewat dari WP | overshoot ≈ |
|---|---|---|---|---|
| 15 m/s | 27 m | 27 m (= d_full) | 11 m | 0 |
| 20 m/s | 49 m | 41 m | 15 m | 8 m |
| 25 m/s | 76 m | 50 m | 15 m | 26 m |

Tanpa antisipasi (mode 0) overshoot ≈ R (uji 26/10: 52–56 m). Keterlambatan
roll-in dan angin searah (R nyata > R perkiraan) membuat busur lebih dekat ke
WP, jadi syarat "lewat ≤ r" tetap terpenuhi; yang bertambah hanya overshoot.
Untuk Δψ < sekitar 34° (v = 20 m/s), d < 15 m dan radius tetap yang menentukan.

Catatan desain:
- **Tidak memakai `L1Controller::turnDistance()`** (versi ArduPlane:
  min(WP_RADIUS, L1) · |Δψ|/90°). Versi itu bergantung pada panjang L1, yang
  diubah `FuzzyL1Tuner`, sehingga titik pergantian WP ikut berbeda antara
  L1 konvensional dan L1 + fuzzy. `d` di atas hanya bergantung pada
  groundspeed dan `ROLL_LIMIT_DEG`, jadi aturannya sama untuk kedua metode.
- WP terakhir tidak diantisipasi (tidak ada leg berikutnya;
  `getNextGroundCourse()` = -1).
- |Δψ| dijepit ≤ 170° (tan(90°) tak hingga); belokan hampir balik arah
  dibatasi `WP_TURN_MAX` (default 80 m).
- v_g adalah groundspeed saat mendekati WP. Bila belokan berakhir searah
  angin, groundspeed setelah belok lebih besar sehingga R sebenarnya lebih
  besar dari perkiraan; antisipasi mengurangi, tetapi tidak menghapus,
  overshoot pada kasus itu.
- Event `WP_REACHED` di log SD memberi bit reason 8 bila klaim terjadi di
  luar radius tetap karena antisipasi (selalu bersama bit 1). Pada mode 1
  jarak terdekat ke WP baru dicapai SESUDAH klaim, saat busur belok;
  `analyze_tlog_l1_auto.py` menghitung `jarak_terdekat_wp_m` sampai akhir
  leg berikutnya dan menandai "lewat dalam/LUAR radius".
- Membandingkan uji dengan dan tanpa `WP_TURN_ANT` harus dicatat terpisah;
  fitur ini mengubah titik pergantian leg, jadi CTE di sekitar WP berubah.

## File lama yang tidak dimigrasikan

```text
include/Navigation.h
include/Auto_setup.h
include/AutoMode.h        (kemungkinan dead/duplikat Auto_setup.h, tidak pernah dipakai jalur aktif)
include/WP_Loiter.h       (dead code — dikonfirmasi audit awal, tidak ada pemakai jalur aktif)
```
