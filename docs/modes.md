# Refactor Modes (FW-only)

Menggabungkan `Mode_setup.h` + `Mode_Manager.h` + `Mode_Fixedwing.h` lama menjadi
struktur FW-only: `ModeId` hanya punya 4 nilai (`Manual`, `Stabilize`, `Auto`, `Guided`
-- `COPT`/`QHOV`/`TRNS` dihapus), dan **tidak ada lagi cabang runtime
`model_uav`** -- program lama meng-include `Mode_Flywing.h`/`Mode_Copter.h`/
`Mode_Transisi.h` TANPA SYARAT (dikompilasi selalu, dipilih saat runtime);
build baru ini FW-only secara compile-time, kode copter/VTOL tidak pernah ada
dalam biner sama sekali.

## Nama mode STABILIZE (dulu FBWA)

Sejak 12 Nov 2026 mode stabil berbasis stick bernama **STABILIZE** (kelas `ModeStabilize`,
`ModeId::Stabilize`, kode log SD `STAB`). Sebelumnya mode yang sama bernama FBWA. Perilaku tidak
berubah: stick tetap memberi setpoint sudut roll/pitch. Nomor MAVLink `custom_mode` sekarang **2**
(Mission Planner menampilkan "Stabilize"); nomor lama **5** tetap diterima dari GCS dan dari
`FLTMODE1..6`, supaya saklar mode yang masih tersimpan 5 di EEPROM tetap berfungsi. Log dan tlog
sebelum 12 Nov tetap mencatat FBWA / 5; skrip analisis memetakan keduanya ke STABILIZE.

## Struktur modul

```text
include/modes/VehicleContext.h    Bag-of-references ke semua subsistem + snapshot per-loop
include/modes/Mode.h              ModeId, ModeBase, ModeManager (port persis dari Mode_setup.h)
include/modes/FixedWingModes.h    4 kelas mode: Manual, Stabilize, Auto, Guided
src/modes/FixedWingModes.cpp
```

## Perubahan arsitektur signifikan: satu attitude controller, bukan dua

Kode lama punya **dua implementasi attitude control paralel yang redundan**
(ditemukan saat audit awal): `FW_CONTROL::updateFBWA_FW()` (P+D sederhana
berbasis stick, dipakai STABILIZE) dan `Attitude.h::stabilize()` (rate-PID penuh,
dipakai AUTO/GUIDED lewat `FW_roll/pitch/yaw_controller.h`). Refactor ini
**hanya punya satu**: `AttitudeController` (LQR, Fase 4), dipakai oleh
**ketiga** mode (STABILIZE, AUTO, GUIDED) -- STABILIZE menghitung setpoint roll/pitch
dari stick (`roll_cmd = 1.3 * map(stick)`, `pitch_cmd = 1.1 * map(stick)`,
formula sama seperti dijelaskan `PROMPT_SIMULASI_FIXEDWING.md`), sementara
AUTO/GUIDED menghitung setpoint dari L1/TECS -- keduanya diteruskan ke
`AttitudeController` LQR yang sama.

**Konsekuensi**: axis yaw di STABILIZE kini **sepenuhnya otomatis**
(coordinated-turn feedforward dari setpoint roll, sama seperti AUTO), bukan
campuran manual-stick + auto-mix seperti lama. Ini penyederhanaan yang
disengaja begitu yaw dikendalikan LQR, bukan rudder-mix manual.

## Dependency injection: `VehicleContext`

Program lama mengakses semua subsistem lewat singleton global
(`imu`, `ahrs`, `fw_control`, dst). Setiap kelas mode di refactor ini
menyimpan `VehicleContext&` (referensi ke seluruh subsistem + snapshot GNSS/dt
yang di-refresh scheduler tiap loop) -- bukan mengakses global. `VehicleContext`
dibangun dan dimiliki lapisan integrasi (Fase 8, `main.cpp`), bukan oleh mode
itu sendiri.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `ModeId` (COPT/QHOV/GUIDED/MANU/FBWA/AUTO/TRNS) | `fc::ModeId` (Manual/Stabilize/Auto/Guided saja) |
| `ModeBase`/`ModeManager` | `fc::ModeBase`/`fc::ModeManager` (interface identik, port langsung) |
| `ModeManual_FW` | `fc::ModeManual` |
| `ModeFBWA_FW` | `fc::ModeStabilize` (sekarang lewat `AttitudeController`, bukan `FW_CONTROL`) |
| `ModeAuto_FW` | `fc::ModeAuto` |
| `ModeGuided_FW` | `fc::ModeGuided` |
| `init_auto()`/`exit_auto()` | Diganti `ModeAuto::_enter()`/`_exit()`, menyetel `navigation.state().auto_navigation_mode`/`auto_throttle_mode` langsung (bukan global terpisah) |
| `enterauto` | Tidak ada lagi -- state resume/first-entry waypoint ditangani `Navigation::navigate()` sendiri lewat `mission.flag_wp` |

## File lama yang tidak dimigrasikan

```text
include/Mode.h            (legacy scaffolding lama, sudah superseded Mode_setup.h di kode asli)
include/Mode_setup.h
include/Mode_Manager.h
include/Mode_Fixedwing.h
include/Mode_Copter.h, Mode_Flywing.h, Mode_Transisi.h, Transition.h   (copter/VTOL, di luar cakupan)
```

## STABILIZE me-reset misi AUTO (2026-11-07)

`ModeStabilize::_enter()` memanggil `Navigation::resetMissionProgress()`: `flag_wp`
kembali -1, status misi selesai dan loiter (di WP / akhir misi) dihapus. Jadi
setiap kali pilot pindah ke STABILIZE lalu kembali ke AUTO, misi selalu dimulai
lagi dari WP1. Daftar WP tidak berubah. Pindah ke mode lain (MANUAL, LOITER,
GUIDED) tidak me-reset; AUTO tetap melanjutkan dari WP terakhir.
