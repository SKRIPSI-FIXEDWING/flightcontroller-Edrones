#pragma once

#include <stdint.h>

namespace fc {

enum class LqrStateMode : uint8_t {
    /** Primary state is an angle (deg); secondary state is its rate (deg/s). Roll, pitch. */
    AngleAndRate,
    /** Primary state IS a rate (deg/s); no secondary state. Yaw. */
    RateOnly,
};

struct LqrAxisConfig {
    LqrStateMode mode = LqrStateMode::AngleAndRate;

    // Gain row of the offline-computed LQR K matrix for this axis (see
    // tools/lqr_gain_design.py and docs/attitude-lqr.md). Applied to the
    // *error* against setpoint (servomechanism/type-1 tracking form), not to
    // the absolute measured state — see docs/attitude-lqr.md for why.
    float k_primary = 0.0f;   // gain on (setpoint - measured_primary)
    float k_rate = 0.0f;      // gain on measured_rate (0 for RateOnly axes)
    float k_integral = 0.0f;  // gain on the integral of (setpoint - measured_primary); 0 disables integral action

    float integral_limit = 0.0f;      // anti-windup clamp on the integral state, deg*s
    float output_limit_deg = 25.0f;   // final surface-deflection-equivalent clamp, degrees
};

/**
 * ===========================================================================
 * PERUNTUKAN FILE INI
 * ===========================================================================
 * One reusable LQR-tuned axis regulator, instantiated per-axis (roll, pitch,
 * yaw) by AttitudeController rather than three separate classes — only the
 * config (gains, mode, limits) differs between axes.
 *
 * State feedback comes directly from fc::Imu (BNO055 already fuses angle and
 * body rate onboard) — no additional state estimator. K gains are FIXED,
 * computed offline (see tools/lqr_gain_design.py); this class does not
 * adapt them online. Speed-scaling and yaw's coordinated-turn setpoint are
 * AttitudeController's responsibility, not this class's.
 *
 * INI ADALAH INTI PERHITUNGAN LQR-NYA (u = -K*x dalam bentuk error tracking).
 * Class ini TIDAK tahu apa-apa soal roll/pitch/yaw secara spesifik — hanya
 * "satu axis generik" yang dipanggil 3x oleh AttitudeController dengan
 * config gain berbeda-beda. Semua hal yang axis-spesifik (setpoint dari mana,
 * speed scaling, output limiting lintas-axis) ada di AttitudeController,
 * BUKAN di sini.
 *
 * DIPANGGIL DARI:
 *   AttitudeController::update()  di src/control/AttitudeController.cpp
 *   — dipanggil 3x per siklus: sekali untuk roll_, sekali untuk pitch_,
 *   sekali untuk yaw_ (lihat AttitudeController.h/.cpp untuk urutan lengkapnya).
 *
 * TIDAK memanggil fungsi lain (leaf/ujung dari rantai panggilan) — hanya
 * aritmatika murni. Hasilnya (float, "raw") dikembalikan ke
 * AttitudeController::update(), yang kemudian mengalikannya dengan speed
 * scaler dan meng-clamp-nya ke output_limit_deg.
 * ===========================================================================
 */
class LqrAxisController final {
public:
    explicit LqrAxisController(const LqrAxisConfig& config = LqrAxisConfig{});

    /**
     * FUNGSI INTI PERHITUNGAN LQR untuk SATU axis. Dipanggil dari
     * AttitudeController::update() — lihat LqrAxisController.cpp untuk
     * detail rumusnya (error, integral, rate_term, kombinasi ketiganya).
     *
     * setpoint/measured_primary: degrees (AngleAndRate) or deg/s (RateOnly).
     * measured_rate: deg/s (ignored in RateOnly mode). dt: seconds.
     * Returns a surface-deflection-equivalent command in degrees, NOT yet
     * clamped to output_limit_deg or speed-scaled — AttitudeController does
     * both, in that order, after combining all three axes.
     *
     * Nilai kembalian ("raw") INI BUKAN nilai akhir servo — masih harus
     * dikali speed scaler dan di-clamp oleh AttitudeController::update()
     * sebelum jadi Output.aileron_deg / elevator_deg / rudder_deg.
     */
    float update(float setpoint, float measured_primary, float measured_rate, float dt);

    // Reset integral_state_ ke 0. Dipanggil (tidak langsung) lewat
    // AttitudeController::resetIntegrators(), yang dipanggil dari
    // ModeFbwa/ModeAuto/ModeGuided::_enter() di FixedWingModes.cpp.
    void resetIntegrator();

    // Nyalakan/matikan akumulasi integral. Saat dimatikan, integral_state_
    // juga langsung direset ke 0 (lihat .cpp). Dipanggil (tidak langsung)
    // lewat AttitudeController::setIntegralEnabled().
    void setIntegralEnabled(bool enabled);

    // Nilai integral_state_ saat ini — untuk telemetry/logging saja, tidak
    // dipakai dalam perhitungan kendali lain.
    float integratorState() const;

    // Getter config, dipakai AttitudeController::update() untuk membaca
    // output_limit_deg saat meng-clamp hasil axis ini.
    const LqrAxisConfig& config() const;

private:
    LqrAxisConfig config_{};
    float integral_state_ = 0.0f;
    bool integral_enabled_ = true;
};

}  // namespace fc
