# Refactor Actuator (servo/motor output)

Port subset FW-relevan dari `Actuator.h` lama, dilepas dari dependensi
copter/flywing (`Copter_Control.h`, `flywing_control.h` tidak lagi ter-include
sama sekali — sebelumnya keduanya ikut ter-include walau hanya beberapa fungsi
di dalamnya yang benar-benar dipakai jalur FW).

## Struktur modul

```text
include/vehicle/Actuator.h   Kontrak class Actuator
src/vehicle/Actuator.cpp     Implementasi servo output
```

## Perubahan penting: sumber attitude output

`fw_servos_out_fbwa()` lama membaca `fw_control.servos[0..3]` — keluaran
`FW_CONTROL` (PID sederhana di `FW_control.h`, salah satu dari **dua**
implementasi kontrol attitude paralel yang saling redundan di kode lama, lihat
`docs/attitude-lqr.md`). Karena `FW_control.h` **tidak di-port** (digantikan
`AttitudeController` LQR, Fase 4), `Actuator::writeAttitude()` yang baru
menerima `AttitudeController::Output` (derajat aileron/elevator/rudder)
langsung, dan mengonversi ke PWM lewat `angleToPwm()` yang sama seperti
`servos_out_fbwa()` lama — bukan `fw_servos_out_fbwa()`'s `+1500` langsung yang
mengasumsikan skala PWM-delta dari `FW_CONTROL`.

## Fungsi yang tidak dimigrasikan (mati atau khusus copter/flywing)

```text
copter_calcOutput, copter_nyt_calcOutput      copter-only
motor_loop, motor_loop_nyt                     copter-only (VTOL motor 1-4)
servos_out_manual_flywing, servos_out_fbwa_flywing,
flywing_calcout, flywing_servos_out            flying-wing-only
motorCopterOff                                 copter-only
servos_out_fbwa(), servos_out_fbwa_auto()      ditandai "//GA KEPAKE" di kode lama — tidak dipakai
dual_motor_throttle_out(), _out_yaw()          ditandai "//GA KEPAKE" — tidak dipakai
calc_throttle()                                 badan fungsi kosong, dead code
payload_loop(), autopayload_loop()              digantikan payload_control_unified() yang lebih baru/lengkap
```

Inisialisasi motor VTOL (`MOTOR_1-4_PIN`, `model_uav==1` branch di
`init_actuator()`) juga tidak di-port — fixed-wing only, tidak ada cabang
model kendaraan runtime lagi (lihat Fase 6, `docs/modes.md`).

## Kepemilikan dan penjadwalan

```cpp
#include "vehicle/Actuator.h"
#include "control/AttitudeController.h"

fc::Actuator actuator;

void setupActuator() { actuator.begin(); }

// MANU
actuator.writeManual(radio.channelRoll(), radio.channelPitch(), radio.channelYaw());

// FBWA/GUIDED/AUTO
fc::AttitudeController::Output cmd = attitude.update(...);
actuator.writeAttitude(cmd);

// Payload (prioritas: MAVLink DROP > switch RC manual > hold)
actuator.updatePayload(radio.armed(), payload_drop_command, radio.channelVehicleMode() > 1500);
```

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `init_actuator()` | `actuator.begin()` |
| `servos_out_manual()` | `actuator.writeManual(ch_roll, ch_pitch, ch_yaw)` |
| `fw_servos_out_fbwa()` | `actuator.writeAttitude(attitude_output)` (lihat catatan sumber output di atas) |
| `plane_motors_out()` | `actuator.writeThrottleManual(ch_throttle, mode_fbwa_plane, armed)` |
| `plane_motors_out_auto(pwm)` | `actuator.writeThrottleAuto(pwm, armed)` |
| `payload_control_unified()` | `actuator.updatePayload(armed, payload_drop_command, manual_switch_active)` |
| `angleToPwm(angle)` | `fc::Actuator::angleToPwm(angle, gain, center)` (static) |

## File lama yang tidak dimigrasikan

```text
include/Actuator.h
include/Servos_FW.h   (sudah dikonfirmasi dead file penuh — di-comment total — saat audit awal)
```
