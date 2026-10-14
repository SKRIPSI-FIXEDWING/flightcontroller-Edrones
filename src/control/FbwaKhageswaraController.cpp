#include "control/FbwaKhageswaraController.h"

#include <AP_Math.h>

namespace fc {

namespace {

// map() integer Arduino (pembagian integer memotong ke nol). Legacy memakainya
// untuk setpoint stick, jadi kuantisasi derajatnya ikut direplikasi.
long arduinoMap(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

}  // namespace

FbwaKhageswaraController::FbwaKhageswaraController(const FbwaKhageswaraConfig& config)
    : config_(config)
{
}

// =============================================================================
// FUNGSI UTAMA -- urutan hitungan sama dengan FW_control.h::updateFBWA_FW()
// =============================================================================
FbwaKhageswaraController::Output FbwaKhageswaraController::update(uint16_t ch_roll,
                                                                  uint16_t ch_pitch,
                                                                  const ImuData& imu, bool armed,
                                                                  uint16_t ch_throttle, float dt)
{
    // 1) Stick -> sudut perintah (derajat).
    const float roll_cmd_deg = rollCommandDeg(ch_roll);
    const float pitch_cmd_deg = pitchCommandDeg(ch_pitch);

    // 2) Integral (sudut, bukan error); reset bila disarm / throttle rendah.
    updateIntegrals(imu, armed, ch_throttle, dt);

    // 3-6) Loop P/PD roll & pitch, u_yaw = 0, saturasi.
    return computeOutput(roll_cmd_deg, pitch_cmd_deg, imu);
}

// =============================================================================
// AUTO -- loop yang sama dengan FBWA; hanya sumber setpoint-nya berbeda
// (nav_roll/nav_pitch dari L1/TECS, bukan stick). Lihat auto_*_cmd_sign.
// =============================================================================
FbwaKhageswaraController::Output FbwaKhageswaraController::updateAuto(float nav_roll_deg,
                                                                      float nav_pitch_deg,
                                                                      const ImuData& imu, bool armed,
                                                                      float dt)
{
    // Kendali attitude TIDAK menyentuh tanda navigasi: nav_roll_deg di sini
    // sudah dalam konvensi stick (positif = bank kanan, sama dengan stick
    // kanan di FBWA). Koreksi arah navigasi ada di
    // Navigation::updateAutoAttitudeTargets() (NavigationConfig::nav_roll_sign).
    const float roll_cmd_deg = config_.auto_roll_cmd_sign * nav_roll_deg;
    const float pitch_cmd_deg = config_.auto_pitch_cmd_sign * nav_pitch_deg;

    // Throttle di AUTO manual dan tidak boleh me-reset integral: kirim nilai
    // throttle "tinggi" agar hanya kondisi !armed yang mereset.
    updateIntegrals(imu, armed, UINT16_MAX, dt);

    return computeOutput(roll_cmd_deg, pitch_cmd_deg, imu);
}

// --- Langkah 3-6 bersama -----------------------------------------------------
//   u_roll  = -Kp(-roll + int - (roll_cmd + trim)) - Kd * gyro_x
//   u_pitch =  Kp(-pitch + int - (-pitch_cmd + trim)) - Kd * gyro_y
//   u_yaw   = 0 ; saturasi +-400 raw lalu konversi ke derajat.
FbwaKhageswaraController::Output FbwaKhageswaraController::computeOutput(float roll_cmd_deg,
                                                                         float pitch_cmd_deg,
                                                                         const ImuData& imu) const
{
    const float u_roll = rollControl(imu.roll_deg, roll_cmd_deg, imu.angular_rate_dps.x);
    const float u_pitch = pitchControl(-imu.pitch_deg, pitch_cmd_deg, imu.angular_rate_dps.y);

    const float aileron_sign = (config_.aileron_output_sign < 0.0f) ? -1.0f : 1.0f;

    Output output{};
    output.aileron_deg = aileron_sign * saturateToDeg(u_roll);
    output.elevator_deg = saturateToDeg(u_pitch);
    output.rudder_deg = 0.0f;
    return output;
}

// --- Langkah 1: stick PWM -> sudut perintah --------------------------------
//   roll_cmd  = 1.3 * map(ch_r - 1500, -512, 512, -35, 35)
//   pitch_cmd = 1.1 * map(ch_p - 1500, -512, 512, -25, 35)
float FbwaKhageswaraController::rollCommandDeg(uint16_t ch_roll) const
{
    const long mapped = arduinoMap(static_cast<long>(ch_roll) - 1500, -512, 512, config_.roll_cmd_min_deg,
                                   config_.roll_cmd_max_deg);
    return config_.roll_cmd_multiplier * static_cast<float>(mapped);
}

float FbwaKhageswaraController::pitchCommandDeg(uint16_t ch_pitch) const
{
    const long mapped = arduinoMap(static_cast<long>(ch_pitch) - 1500, -512, 512, config_.pitch_cmd_min_deg,
                                   config_.pitch_cmd_max_deg);
    return config_.pitch_cmd_multiplier * static_cast<float>(mapped);
}

// --- Langkah 2: integral (getIntegral() legacy) ----------------------------
//   int += I * sudut * dt ; clamp +-15 ; reset bila !armed atau ch_thr < 1100
// FBWA bukan auto-throttle, jadi syarat "throttle manual" selalu berlaku.
void FbwaKhageswaraController::updateIntegrals(const ImuData& imu, bool armed, uint16_t ch_throttle,
                                               float dt)
{
    roll_int_ += config_.roll_i * imu.roll_deg * dt;
    pitch_int_ += config_.pitch_i * imu.pitch_deg * dt;

    roll_int_ = constrain_float(roll_int_, -config_.integral_limit, config_.integral_limit);
    pitch_int_ = constrain_float(pitch_int_, -config_.integral_limit, config_.integral_limit);

    if (!armed || ch_throttle < config_.throttle_reset_pwm) {
        roll_int_ = 0.0f;
        pitch_int_ = 0.0f;
    }
}

// --- Langkah 3: loop roll (baris 110 legacy) --------------------------------
// Bentuk penjumlahan "roll + roll_cmd" disalin apa adanya; tanda terhadap
// sensor harus diuji di bangku (lihat dokumentasi, temuan "Kritis").
float FbwaKhageswaraController::rollControl(float roll_deg, float roll_cmd_deg, float gyro_dps) const
{
    return -config_.roll_p * (-roll_deg + roll_int_ - (roll_cmd_deg + config_.trim_roll)) -
           config_.roll_d * gyro_dps;
}

// --- Langkah 4: loop pitch (baris 113 legacy) -------------------------------
float FbwaKhageswaraController::pitchControl(float pitch_deg, float pitch_cmd_deg, float gyro_dps) const
{
    return config_.pitch_p * (-pitch_deg + pitch_int_ - (-pitch_cmd_deg + config_.trim_pitch)) -
           config_.pitch_d * gyro_dps;
}

// --- Langkah 6: saturasi servos[i] +-400 raw, lalu raw -> derajat ----------
float FbwaKhageswaraController::saturateToDeg(float u_raw) const
{
    return constrain_float(u_raw, -config_.output_limit_raw, config_.output_limit_raw) / config_.raw_per_deg;
}

void FbwaKhageswaraController::resetIntegrators()
{
    roll_int_ = 0.0f;
    pitch_int_ = 0.0f;
}

float FbwaKhageswaraController::rollIntegrator() const
{
    return roll_int_;
}

float FbwaKhageswaraController::pitchIntegrator() const
{
    return pitch_int_;
}

}  // namespace fc
