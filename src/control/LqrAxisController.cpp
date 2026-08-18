#include "control/LqrAxisController.h"

namespace fc {

// Constructor: hanya menyimpan config gain (k_primary, k_rate, k_integral,
// limits, mode) untuk axis ini. Dipanggil dari AttitudeController's
// constructor (roll_(config.roll), pitch_(config.pitch), yaw_(config.yaw)).
LqrAxisController::LqrAxisController(const LqrAxisConfig& config)
    : config_(config)
{
}

// =============================================================================
// HUKUM KENDALI LQR untuk SATU axis (bentuk servomechanism/type-1 tracking:
// u = K_p*(setpoint - x) + K_rate*rate_term + K_i*integral(error) ).
// Dipanggil dari AttitudeController::update() — 3x per siklus, sekali per
// axis (roll_, pitch_, yaw_), masing-masing dengan config_ gain berbeda.
//
// URUTAN LOGIC DI DALAM FUNGSI INI:
//   1. Hitung primary_error = setpoint - measured_primary
//      (untuk roll/pitch: error sudut dalam derajat;
//       untuk yaw (RateOnly): error rate dalam deg/s)
//   2. Kalau k_integral aktif & integral_enabled_: akumulasikan error*dt ke
//      integral_state_, lalu clamp ke +-integral_limit (anti-windup).
//      Untuk axis yaw, k_integral = 0 sehingga blok ini efeknya nol.
//   3. Hitung rate_term:
//      - AngleAndRate (roll, pitch): -k_rate * measured_rate  (redaman gyro)
//      - RateOnly (yaw): selalu 0 (rate sudah jadi primary_error di atas)
//   4. Jumlahkan ketiga komponen: k_primary*error + rate_term + k_integral*integral
//      -> INI ADALAH u = -K*x versi LQR untuk axis ini, dikembalikan sebagai
//      nilai "raw" (belum di-scale kecepatan, belum di-clamp limit output).
//
// Nilai kembalian diterima oleh AttitudeController::update() sebagai
// roll_raw / pitch_raw / yaw_raw, yang kemudian dikalikan speed scaler dan
// di-clamp oleh AttitudeController::clampToOutputLimit() sebelum jadi
// Output.aileron_deg / elevator_deg / rudder_deg.
// =============================================================================
float LqrAxisController::update(float setpoint, float measured_primary, float measured_rate, float dt)
{
    // 1) Error terhadap setpoint (sudut untuk roll/pitch, rate untuk yaw).
    const float primary_error = setpoint - measured_primary;

    // 2) Aksi integral (anti-windup via clamp). k_integral = 0 (yaw) -> no-op.
    if (config_.k_integral != 0.0f && integral_enabled_) {
        integral_state_ += primary_error * dt;
        if (integral_state_ > config_.integral_limit) {
            integral_state_ = config_.integral_limit;
        } else if (integral_state_ < -config_.integral_limit) {
            integral_state_ = -config_.integral_limit;
        }
    }

    // 3) Rate feedback: hanya axis AngleAndRate (roll, pitch) yang punya
    //    state rate terpisah dari primary. Axis RateOnly (yaw) sudah
    //    memasukkan rate sebagai primary_error di langkah 1, jadi di sini 0.
    const float rate_term = (config_.mode == LqrStateMode::AngleAndRate) ? -config_.k_rate * measured_rate : 0.0f;

    // 4) Kombinasi linear ketiga term -> hasil akhir hukum kendali LQR axis ini.
    return config_.k_primary * primary_error + rate_term + config_.k_integral * integral_state_;
}

// Dipanggil (tidak langsung) dari AttitudeController::resetIntegrators(),
// yang dipanggil dari ModeFbwa/ModeAuto/ModeGuided::_enter().
void LqrAxisController::resetIntegrator()
{
    integral_state_ = 0.0f;
}

// Dipanggil (tidak langsung) dari AttitudeController::setIntegralEnabled().
// Mematikan integral juga langsung mereset state-nya ke 0, supaya begitu
// diaktifkan lagi tidak membawa nilai integral lama.
void LqrAxisController::setIntegralEnabled(bool enabled)
{
    integral_enabled_ = enabled;
    if (!enabled) {
        integral_state_ = 0.0f;
    }
}

// Untuk telemetry/logging (dipanggil via AttitudeController::rollIntegratorState()
// / pitchIntegratorState()) — tidak dipakai axis yaw karena k_integral = 0.
float LqrAxisController::integratorState() const
{
    return integral_state_;
}

// Dibaca oleh AttitudeController::update() untuk mengambil output_limit_deg
// axis ini saat melakukan clampToOutputLimit().
const LqrAxisConfig& LqrAxisController::config() const
{
    return config_;
}

}  // namespace fc
