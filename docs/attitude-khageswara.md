# Kendali attitude Khageswara (FBWA & AUTO) dan alur AUTO

Dokumen ini menjelaskan kendali attitude P/PD ala program Khageswara yang
dipakai oleh mode **FBWA** dan **AUTO**, polaritas aileron
(`AIL_OUT_SIGN`), serta alur mode AUTO (misi, done-behaviour, penolakan masuk).

```
include/control/FbwaKhageswaraController.h   parameter + kelas kendali
src/control/FbwaKhageswaraController.cpp     update() [FBWA], updateAuto() [AUTO]
include/control/AttitudeController.h         updateFbwaKhageswara(), updateAutoKhageswara()
src/modes/FixedWingModes.cpp                 ModeFbwa / ModeAuto
src/navigation/Navigation.cpp                beginAuto(), navigate(), done-behaviour
src/navigation/L1Controller.cpp              guidance lateral (nav_roll)
src/navigation/Tecs.cpp                      guidance vertikal (nav_pitch)
```

GUIDED masih memakai LQR (`AttitudeController::update()`).

---

## 1. Rumus (identik di FBWA dan AUTO)

```
u_roll  = -Kp_r * (-roll + roll_int - (roll_cmd  + trim_roll)) - Kd_r * gyro_x
u_pitch =  Kp_p * (-pitch_arg + pitch_int - (-pitch_cmd + trim_pitch)) - Kd_p * gyro_y
          dengan pitch_arg = -imu.pitch_deg
u_yaw   = 0
keluaran = clamp(u, -400, +400) raw / 11.378  -> derajat
```

Gain: roll `P=10, I=0, D=0`; pitch `P=10, I=0, D=0.2`.

Dengan trim = 0 dan integral = 0, bentuk di atas menjadi:

| axis | u | titik setimbang |
|---|---|---|
| roll | `Kp (roll + roll_cmd)` | `roll = -roll_cmd` |
| pitch | `Kp (imu.pitch + pitch_cmd)` | `imu.pitch = -pitch_cmd` |

Artinya `*_cmd` di kendali ini adalah **negatif** dari sudut yang dicapai.

### Sumber `roll_cmd` / `pitch_cmd`

| mode | roll_cmd | pitch_cmd |
|---|---|---|
| FBWA | `1.3 * map(ch_roll-1500, -512..512 -> -35..35)` | `1.1 * map(ch_pitch-1500, -512..512 -> -25..35)` |
| AUTO | `auto_roll_cmd_sign * nav_roll_deg` (L1) | `auto_pitch_cmd_sign * nav_pitch_deg` (TECS) |

**Revisi 2026-10-14 (arah NAVIGASI AUTO):** kendali attitude tidak diubah
(FBWA dan AUTO memakai loop dan tanda yang sama; `AIL_OUT_SIGN` hanya untuk
polaritas aileron). Koreksi arah navigasi ada di lapisan navigasi:
`NavigationConfig::nav_roll_sign` (macro `FC_NAV_ROLL_SIGN`, default -1) yang
mengalikan hasil L1 di `Navigation::updateAutoAttitudeTargets()`. Setelah
itu `mission.nav_roll_deg` berkonvensi stick (positif = bank kanan).

**Uji arah navigasi (bangku):** `NAV_CONTROLLER_OUTPUT` membawa `nav_bearing`
dan `target_bearing` (derajat). Hadapkan hidung pesawat ke utara (kompas HP):
`yaw` harus mendekati 0 (heading yang salah 180 derajat membalik kiri/kanan
untuk target di depan). Lalu bandingkan `target_bearing - yaw`: negatif
(WP di kiri) harus memberi `nav_roll` negatif, positif harus positif.
Bila `yaw` benar tetapi `nav_roll` berlawanan, balik `FC_NAV_ROLL_SIGN`;
bila `yaw` salah 180 derajat, perbaiki `heading_offset_deg` di `ImuConfig`.

Sebelumnya: `auto_roll_cmd_sign = +1` dan `auto_pitch_cmd_sign = -1` (2026-10-14).

- Roll `+1`: hasil uji menunjukkan `-1` membuat aileron AUTO meminta arah
  **berlawanan** dengan stick FBWA untuk bank yang sama. Dengan `+1`,
  `nav_roll > 0` (kanan) memberi keluaran aileron yang sama dengan stick
  kanan di FBWA.
- Pitch `-1`: arah elevator "minta naik" sudah sesuai hasil uji.

> Catatan: tanda ini hanya mengatur **arah perintah**. Apakah loop roll
> berumpan-balik negatif (menahan sayap) ditentukan oleh uji pada
> section 2 (miringkan pesawat dengan tangan di FBWA).

### Kenapa elevator terlihat perlahan saat masuk AUTO dari Manual

TECS hanya dijalankan di mode auto-throttle. Bila TECS tidak dipanggil > 1 s,
state-nya di-reset saat masuk (`initialiseStates`), dan ArduPilot mengisi
filter tuntutan ketinggian dengan ketinggian **saat ini**. Tuntutan lalu
mendekati target lewat limit climb-rate (5 m/s) dan lag orde-1
(`time_const_s = 5`), sehingga pitch membangun diri selama 1-2 detik atau
lebih. Pindah AUTO -> FBWA -> AUTO dalam waktu < 1 s tidak mereset, jadi
tuntutan yang sudah di target langsung terlihat sebagai permintaan naik.
Perbedaan "dari Manual" vs "dari FBWA" sebenarnya adalah selang waktu sejak
AUTO terakhir (> 1 s atau tidak), bukan jenis modenya.

`TecsConfig::snap_height_demand_on_entry = true` (default) mengisi filter itu
dengan ketinggian tujuan, sehingga elevator langsung meminta naik (dibatasi
pitch limit +14 derajat dan `vertical_accel_limit_mps2`). Set `false` untuk
kembali ke perilaku ArduPilot.

---

## 2. `AIL_OUT_SIGN` -- polaritas aileron

### Latar belakang

Legacy Khageswara menulis servo seperti ini (`Actuator.h`, `fw_servos_out_fbwa()`):

```
pwm_ail = 1500 - servos[aileron]      <- aileron DIBALIK
pwm_ele = 1500 + servos[elevator]
```

Refactor ini menulis `PWM = center + gain * deg` untuk semua axis
(`Actuator::writeAttitude()`), dan mode Manual meneruskan `ch_roll` apa
adanya (PWM tinggi = roll kanan). Rumus roll di atas, `u = Kp (roll + cmd)`,
baru berumpan-balik **negatif** bila `PWM = center - u`. Dengan `+u`,
loop roll berpotensi **umpan balik positif** (aileron mendorong sayap makin
miring), baik di FBWA maupun AUTO.

### Parameter

| | |
|---|---|
| Nama MAVLink | `AIL_OUT_SIGN` |
| Field | `FbwaKhageswaraConfig::aileron_output_sign` |
| Nilai | `+1.0` (default, apa adanya) atau `-1.0` (polaritas legacy: `PWM = center - u`) |
| Berlaku | setelah reboot (param dibaca saat boot, sebelum `AttitudeController` dibuat) |
| Cakupan | aileron di FBWA **dan** AUTO; elevator & rudder tidak berubah |
| Aman | nilai `< 0` dibaca `-1`, selainnya `+1`; `0` tidak mematikan aileron |

Param ini disisipkan di tengah tabel param sehingga EEPROM lama tergeser satu
slot; param baru wajib ditambahkan di akhir (lihat `docs/params.md`, bagian
`WP_RADIUS`).

### Prosedur uji bangku (tanpa baling-baling, wahana ditahan)

1. Nyalakan, masuk **FBWA**, stick roll netral, throttle minimum.
2. Miringkan badan pesawat dengan tangan **ke kanan** ~20 derajat.
3. Amati aileron. Koreksi yang benar adalah aileron yang akan
   mengembalikan sayap **ke kiri**, yaitu PWM **turun** dari 1500
   (karena di Manual PWM tinggi = roll kanan).
   - Sesuai: biarkan `AIL_OUT_SIGN = 1`.
   - Berlawanan: set `AIL_OUT_SIGN = -1`, tulis param, reboot, ulangi.
4. Gerakkan stick roll kanan: bank yang diminta harus ke kanan.
   Jika langkah 3 benar tetapi stick terbalik, itu urusan `roll_cmd_multiplier`
   / pemetaan stick, bukan `AIL_OUT_SIGN`.
5. Ulangi 2-3 untuk elevator (hidung naik -> elevator harus menekan
   hidung turun). Elevator belum punya parameter polaritas; bila terbalik
   laporkan agar ditambahkan `ELE_OUT_SIGN` dengan cara yang sama.

> Jangan terbang sebelum langkah 3 lolos. Polaritas aileron yang salah
> membuat roll tidak stabil, bukan sekadar terbalik.

---

## 3. Alur AUTO

```
RC switch / MAVLink DO_SET_MODE=10
  -> ModeManager::setMode(Auto)
       -> ModeAuto::_enter() -> Navigation::beginAuto()   (boleh MENOLAK)
  -> ModeAuto::_update()  (tiap siklus control)
       1. ahrs.update(), navigation.updateHomeAndPosition()
       2. navigation.navigate()            L1: nav_roll  | sequencing WP
       3. navigation.updateSpeedHeight()   TECS 50 Hz
       4. navigation.updateAltitude()      TECS: nav_pitch
       5. navigation.updateAutoAttitudeTargets()  -> mission.nav_roll_deg / nav_pitch_deg
       6. attitude.updateAutoKhageswara()  (kendali FBWA, section 1)
       7. actuator.writeAttitude() + writeThrottleManual()   (throttle = stick)
```

### Penolakan masuk AUTO (`beginAuto()`)

`setMode(Auto)` mengembalikan `false` dan mode sebelumnya dipulihkan bila:

| alasan (STATUSTEXT) | penyebab |
|---|---|
| `AUTO: no mission (need HOME + >=1 WP)` | `wp_sum < 2` |
| `AUTO: no GPS position yet` | posisi (0,0) |
| `AUTO: home not set (needed for RTL)` | `mission_done_behavior = 1` dan home belum terkunci |
| `AUTO: WP at 0,0 is invalid` | ada item 1..N berkoordinat (0,0) |

Dari RC, penolakan tidak diulang tiap loop; geser switch menjauh lalu kembali.

### Urutan waypoint

Item 0 adalah HOME (seq 0 Mission Planner) dan **tidak** dijadikan target.
Navigasi mulai dari item 1, atau melanjutkan `flag_wp` bila masih valid.

### Done-behaviour (akhir misi)

Setelah WP terakhir tercapai, mode tetap AUTO dan navigasi berpindah:

| `NavigationConfig::mission_done_behavior` | perilaku |
|---|---|
| `0` (default) | tidak melakukan apa-apa: tetap AUTO dan terus mengikuti leg terakhir |
| `1` | loiter di WP terakhir, pada ketinggian WP tersebut |
| `2` | terbang ke home pada `rtl_altitude_m`, lalu loiter di atas home |

(Ringkasan keputusan evaluasi AUTO 2026-10-14: `docs/auto-mode-khageswara-port.md`.)

Loiter: `loiter_radius_m = 50`, `loiter_direction = -1` (CCW, default legacy).
Masuk ulang ke AUTO memulai misi dari item 1.

---

## 4. Aktuator "tidak mengunci" ke waypoint di AUTO

Gejala: GPS sudah dapat satelit, waypoint ada, tetapi aileron/elevator tidak
bergerak.

### Aileron

Percepatan lateral L1 sebanding dengan **groundspeed**
(`a = K * v^2 / L1`, `L1 ~ v`, jadi `a ~ v`). Saat wahana diam di bangku
`v ~ 0`, maka `nav_roll ~ 0` dan aileron netral **walaupun** arah ke
waypoint jauh berbeda. Pada `v = 0` persis, hasilnya `0/0 = NaN`.

Perbaikan: `L1ControllerConfig::min_guidance_speed_mps = 8` memberi lantai
kecepatan pada L1. Di penerbangan normal (>= 12 m/s) lantai ini tidak aktif.
Di bangku, `nav_roll` kini mengikuti galat arah antara heading dan bearing ke
waypoint, sehingga aileron mengunci ke kiri/kanan.

Bank yang diminta sebanding dengan galat heading: 90 derajat galat pada 8 m/s
sekitar 18 derajat bank.

### Elevator

Elevator mengunci ke atas ketika altitude WP lebih tinggi dari ketinggian
saat ini (TECS). Dua hal yang menutupinya sebelumnya:

1. **Tanda tinggi TECS** (`Tecs.cpp`, `Navigation.cpp`): TECS menerima
   `height = -altitude`. Sudah diperbaiki; TECS kini memakai `height = +altitude`.
2. Altitude WP `0` dari GCS (frame/ketinggian tidak diisi) -> tidak ada
   permintaan naik. Periksa `z` item misi di Mission Planner.

### Cara memeriksa dari Mission Planner

Telemetry `NAV_CONTROLLER_OUTPUT` (tab Status: `nav_roll`, `nav_pitch`,
`wp_dist`, `xtrack_error`) menampilkan permintaan L1/TECS **sebelum**
kendali attitude:

| `nav_roll` / `nav_pitch` | artinya |
|---|---|
| keduanya ~ 0 | masalahnya di guidance (misi, altitude WP, GPS, speed floor) |
| tidak ~ 0, servo diam | masalahnya di keluaran (`AIL_OUT_SIGN`, wiring, mode bukan AUTO) |
| `nav_roll = -36` persis & tetap | kemungkinan NaN terjepit; periksa groundspeed/IMU |

---

## 5. Hal yang belum dilakukan / perlu diuji

- Polaritas aileron dan elevator **belum** diuji di bangku (section 2).
- `auto_roll_cmd_sign` / `auto_pitch_cmd_sign` diturunkan dari analisis kode;
  konfirmasi di bangku: di AUTO dengan waypoint di kanan, aileron harus
  meminta bank ke kanan.
- Throttle AUTO manual: TECS tetap menghitung demand throttle tetapi tidak
  dipakai, sehingga pengendalian kecepatan sepenuhnya oleh pilot.
- Parameter done-behaviour belum didaftarkan sebagai MAVLink param.
