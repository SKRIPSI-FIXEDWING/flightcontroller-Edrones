#include "control/AttitudeController.h"

#include <AP_Math.h>
#include <definitions.h>
#include <math.h>

namespace fc {

// Constructor: bentuk 3 LqrAxisController (roll_, pitch_, yaw_) dari
// config.roll/pitch/yaw masing-masing. Dipanggil sekali saat VehicleContext
// dibuat (lihat pemakaian ctx_.attitude di FixedWingModes.cpp).
AttitudeController::AttitudeController(const AttitudeControllerConfig& config)
    : config_(config), roll_(config.roll), pitch_(config.pitch), yaw_(config.yaw)
{
}

// LANGKAH 4 dalam update(): hitung setpoint RATE yaw (deg/s) secara
// kinematik dari sudut roll saat ini, bukan dari LQR:
//   r_cmd = g * tan(roll) / V   (rumus coordinated turn standar)
// Ini menggantikan "rudder mixing" manual pada implementasi lama — makin
// besar bank angle (roll), makin besar rate yaw yang diminta supaya
// belokannya coordinated (tidak slip/skid). min_airspeed_mps mencegah
// pembagian dengan angka mendekati nol saat airspeed sangat rendah.
// Dipanggil dari update(), hasilnya diteruskan sebagai `setpoint` ke
// yaw_.update() (LANGKAH 5).
float AttitudeController::computeCoordinatedTurnRateDps(float roll_deg, float airspeed_mps,
                                                        float min_airspeed_mps)
{
    const float safe_airspeed = MAX(airspeed_mps, min_airspeed_mps);
    const float rate_rad_s = (GRAVITY_MSS * tanf(radians(roll_deg))) / safe_airspeed;
    return degrees(rate_rad_s);
}

// LANGKAH 1 dalam update(): hitung faktor skala output berdasarkan rasio
// airspeed trim vs airspeed terukur, dikuadratkan lalu di-clamp ke
// [scaler_min, scaler_max]. Alasan (lihat docs/attitude-lqr.md): gain K LQR
// dihitung offline untuk SATU titik trim airspeed saja (18 m/s); di
// kecepatan lain, efektivitas permukaan kendali berubah (~V^2), jadi
// output-nya di-scale sebagai kompensasi murah tanpa perlu gain-scheduling
// K yang penuh. Dipanggil di awal update(); hasilnya (scaler) dipakai untuk
// mengalikan roll_raw/pitch_raw/yaw_raw sebelum di-clamp di clampToOutputLimit().
float AttitudeController::computeSpeedScaler(float airspeed_mps) const
{
    const float safe_airspeed = MAX(airspeed_mps, config_.min_airspeed_for_scaling_mps);
    const float ratio = config_.trim_airspeed_mps / safe_airspeed;
    float scaler = ratio * ratio;
    scaler = constrain_float(scaler, config_.scaler_min, config_.scaler_max);
    return scaler;
}

// LANGKAH 6 dalam update(): batasi nilai akhir (raw * scaler) ke rentang
// [-limit_deg, +limit_deg]. Dipanggil 3x di akhir update() (satu per axis)
// sebagai langkah paling terakhir sebelum Output dikembalikan ke caller.
float AttitudeController::clampToOutputLimit(float value_deg, float limit_deg)
{
    return constrain_float(value_deg, -limit_deg, limit_deg);
}

// =============================================================================
// FUNGSI INTI — dipanggil sekali per loop control dari mode aktif
// (ModeFbwa/ModeAuto/ModeGuided::_update() di src/modes/FixedWingModes.cpp
// via `ctx_.attitude.update(...)`).
//
// URUTAN LOGIC DI DALAM FUNGSI INI:
//   1. computeSpeedScaler(airspeed_mps)                       -> scaler
//   2. roll_.update(nav_roll_deg, imu.roll_deg, gyro_x, dt)   -> roll_raw
//      (LqrAxisController::update() di LqrAxisController.cpp — hukum LQR
//       u = k_primary*error + rate_term + k_integral*integral)
//   3. pitch_.update(nav_pitch_deg, imu.pitch_deg, gyro_y, dt)-> pitch_raw
//      (LqrAxisController::update(), sama seperti roll)
//   4. computeCoordinatedTurnRateDps(nav_roll_deg, airspeed)  -> yaw setpoint (rate, deg/s)
//   5. yaw_.update(yaw_setpoint, gyro_z, 0, dt)                -> yaw_raw
//      (LqrAxisController::update(), mode RateOnly: tanpa integral, tanpa rate_term)
//   6. clampToOutputLimit(raw * scaler, limit) untuk roll/pitch/yaw -> Output
//
// Hasil Output{aileron_deg, elevator_deg, rudder_deg} lalu dikembalikan ke
// mode aktif di FixedWingModes.cpp, yang meneruskannya ke
// Actuator::writeAttitude(output) untuk menggerakkan servo fisik.
// =============================================================================
AttitudeController::Output AttitudeController::update(float nav_roll_deg, float nav_pitch_deg,
                                                       const ImuData& imu, float airspeed_mps,
                                                       float dt)
{
    // 1) Faktor skala berbasis airspeed — dihitung sekali, dipakai untuk semua axis.
    //    TETAP dihitung & dicatat ke last_speed_scaler_ walau Params
    //    "SPD_SCALE_EN" mati, supaya masih kelihatan di telemetry/log nilai
    //    scaler yang SEHARUSNYA dipakai -- yang di-bypass cuma penerapannya
    //    ke output di langkah 6.
    const float computed_scaler = computeSpeedScaler(airspeed_mps);
    last_speed_scaler_ = computed_scaler;
    const float scaler = (config_.speed_scaler_enabled != 0.0f) ? computed_scaler : 1.0f;

    // 2) & 3) Axis roll & pitch: setpoint sudut (nav_*_deg) datang dari caller
    //    (stick di FBWA, atau L1/TECS di AUTO/GUIDED). measured_primary =
    //    sudut aktual dari IMU, measured_rate = gyro rate dari IMU.
    //    -> lompat ke LqrAxisController::update() untuk lihat rumus LQR-nya.
    const float roll_raw = roll_.update(nav_roll_deg, imu.roll_deg, imu.angular_rate_dps.x, dt);
    const float pitch_raw = pitch_.update(nav_pitch_deg, imu.pitch_deg, imu.angular_rate_dps.y, dt);

    // 4) Axis yaw: TIDAK dapat setpoint dari caller. Setpoint-nya dihitung
    //    sendiri di sini secara kinematik dari roll saat ini (coordinated turn).
    const float yaw_rate_setpoint_dps = computeCoordinatedTurnRateDps(nav_roll_deg, airspeed_mps, config_.min_airspeed_for_turn_rate_mps);
    last_yaw_rate_setpoint_dps_ = yaw_rate_setpoint_dps;
    // 5) Axis yaw mode RateOnly: setpoint & measured_primary sama-sama rate
    //    (deg/s), measured_rate diisi 0 karena tidak dipakai (lihat
    //    LqrAxisController::update(): rate_term hanya aktif utk AngleAndRate).
    const float yaw_raw = yaw_.update(yaw_rate_setpoint_dps, imu.angular_rate_dps.z, 0.0f, dt);

    // 6) Skalakan tiap axis lalu clamp ke limit masing-masing -> hasil akhir.
    Output output{};
    output.aileron_deg = clampToOutputLimit(roll_raw * scaler, roll_.config().output_limit_deg);
    output.elevator_deg = clampToOutputLimit(pitch_raw * scaler, pitch_.config().output_limit_deg);
    // MAVLink/GCS-settable kill switch (Params "YAW_CORR_EN"). yaw_.update()
    // above still runs every call regardless (keeps its internal state
    // consistent), this just gates whether the computed value reaches the
    // rudder servo.
    output.rudder_deg = (config_.yaw_correction_enabled != 0.0f)
                             ? clampToOutputLimit(yaw_raw * scaler, yaw_.config().output_limit_deg)
                             : 0.0f;
    return output;
}

void AttitudeController::resetIntegrators()
{
    roll_.resetIntegrator();
    pitch_.resetIntegrator();
    yaw_.resetIntegrator();
}

void AttitudeController::setIntegralEnabled(bool enabled)
{
    roll_.setIntegralEnabled(enabled);
    pitch_.setIntegralEnabled(enabled);
    yaw_.setIntegralEnabled(enabled);
}

float AttitudeController::rollIntegratorState() const
{
    return roll_.integratorState();
}

float AttitudeController::pitchIntegratorState() const
{
    return pitch_.integratorState();
}

float AttitudeController::lastSpeedScaler() const
{
    return last_speed_scaler_;
}

float AttitudeController::lastYawRateSetpointDps() const
{
    return last_yaw_rate_setpoint_dps_;
}

}  // namespace fc
