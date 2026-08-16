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
 * One reusable LQR-tuned axis regulator, instantiated per-axis (roll, pitch,
 * yaw) by AttitudeController rather than three separate classes — only the
 * config (gains, mode, limits) differs between axes.
 *
 * State feedback comes directly from fc::Imu (BNO055 already fuses angle and
 * body rate onboard) — no additional state estimator. K gains are FIXED,
 * computed offline (see tools/lqr_gain_design.py); this class does not
 * adapt them online. Speed-scaling and yaw's coordinated-turn setpoint are
 * AttitudeController's responsibility, not this class's.
 */
class LqrAxisController final {
public:
    explicit LqrAxisController(const LqrAxisConfig& config = LqrAxisConfig{});

    /**
     * setpoint/measured_primary: degrees (AngleAndRate) or deg/s (RateOnly).
     * measured_rate: deg/s (ignored in RateOnly mode). dt: seconds.
     * Returns a surface-deflection-equivalent command in degrees, NOT yet
     * clamped to output_limit_deg or speed-scaled — AttitudeController does
     * both, in that order, after combining all three axes.
     */
    float update(float setpoint, float measured_primary, float measured_rate, float dt);

    void resetIntegrator();
    void setIntegralEnabled(bool enabled);
    float integratorState() const;

    const LqrAxisConfig& config() const;

private:
    LqrAxisConfig config_{};
    float integral_state_ = 0.0f;
    bool integral_enabled_ = true;
};

}  // namespace fc
