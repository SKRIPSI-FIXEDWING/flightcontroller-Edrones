#pragma once

#include "drivers/Imu.h"

namespace fc {

<<<<<<< HEAD
/**
 * TD / V10-trainer2 axis gains.
 *
 * These gains operate in the same legacy raw servo-offset units used by
 * KHAGESWARA's FW_control.h. The final raw command is converted to surface
 * degrees before it reaches Actuator::writeAttitude().
 */
struct TdAxisConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_limit_deg = 15.0f;
    float output_limit_deg = 35.156f;  // +/-400 raw units / 11.378 raw-units-per-degree
};

struct AttitudeControllerConfig {
    // Flight-proven TD/V10-trainer2 baseline.
    TdAxisConfig roll{10.0f, 0.0f, 0.0f, 15.0f, 35.156f};
    TdAxisConfig pitch{10.0f, 0.0f, 0.20f, 15.0f, 35.156f};

    // Compatibility/experiment hook. TD itself does not use this V^-2 output
    // multiplier, so the default remains OFF. Keep it OFF when reproducing TD.
=======
struct AttitudeControllerConfig {
    // Defaults computed by tools/lqr_gain_design/lqr_gain_design.py (cruise
    // trim @ 18 m/s, 200 Hz zero-order-hold discretization, Bryson's-rule
    // Q/R against the assumed stability/control derivatives) — see
    // docs/attitude-lqr.md. Field order: {mode, k_primary, k_rate,
    // k_integral, integral_limit, output_limit_deg}. Re-run the script and
    // update these if the airframe's mass/derivatives/limits change; these
    // are NOT flight-validated, only pole/damping-checked in the script
    // across the 14-22 m/s envelope.
    // output_limit_deg = 35.156 (2026-08-25, was 45.0 -- see git history):
    // reverted on user's explicit request to match legacy FW_control.h's
    // servo-output clamp EXACTLY instead of ArduPilot's "same as Manual's
    // full throw" convention. Legacy's updateFBWA_FW()/updateAUTO_FW()
    // clamp servos[] (its final aileron/elevator/rudder command) to
    // +-400 raw units: "servos[i] = constrain(servos[i], -400, 400);".
    // Those raw units and this codebase's angle_to_pwm_gain (11.378,
    // Actuator.h) are the SAME conversion constant (Actuator.h's own doc
    // comment: "from legacy Actuator.h's angleToPwm()") -- so 400 raw units
    // == 400/11.378 == ~35.156 deg is the exact equivalent limit here.
    // Narrower than Manual's full +-512 raw-PWM throw (~45 deg): legacy
    // capped stabilized-mode output BELOW Manual's max on purpose, unlike
    // ArduPilot's convention -- match that behavior, not ArduPilot's.
    LqrAxisConfig roll{LqrStateMode::AngleAndRate, 3.449792f, 0.409788f, 1.393631f, 17.2f, 35.156f};
    LqrAxisConfig pitch{LqrStateMode::AngleAndRate, 5.205471f, 0.661225f, 1.630465f, 14.9f, 35.156f};
    LqrAxisConfig yaw{LqrStateMode::RateOnly, 1.021439f, 0.0f, 0.0f, 0.0f, 35.156f};

    // Speed scaler: (trim_airspeed_mps / measured_airspeed_mps)^2, clamped to
    // [scaler_min, scaler_max], multiplies each axis's raw LQR output before
    // the final output_limit_deg clamp. See docs/attitude-lqr.md for why a
    // single fixed K plus this post-multiplier is used instead of a
    // gain-scheduled K.
>>>>>>> parent of 0a2e1e2 (Tuningan baru hasil simulasi)
    float trim_airspeed_mps = 18.0f;
    float scaler_min = 0.6f;
    float scaler_max = 1.8f;
    float min_airspeed_for_scaling_mps = 3.0f;
    float speed_scaler_enabled = 0.0f;
};

/**
 * Fixed-wing inner-loop attitude controller ported from TD/V10-trainer2.
 *
 * Unified desired-attitude convention used by this refactor:
 *   roll desired  = nav_roll_deg
 *   pitch desired = nav_pitch_deg
 *
 * The equations reproduce the active TD FW_control.h path:
 *   u_roll_raw  = Kp_r * (roll - roll_des - I_r) - Kd_r * roll_rate
 *   u_pitch_raw = Kp_p * (pitch_des - pitch + I_p) - Kd_p * pitch_rate
 *
 * TD's final aileron PWM direction is reversed (1500 - u_roll). That reversal
 * is kept in Actuator::writeAttitude(), not hidden inside this controller.
 * Rudder is neutral, matching the active TD fixed-wing path (u_yaw = 0).
 *
 * The legacy TD integrator is also reproduced for completeness:
 *   I_r += Ki_r * roll * dt
 *   I_p += Ki_p * pitch * dt
 * but the flight-proven TD defaults use Ki = 0 for both axes.
 */
class AttitudeController final {
public:
    struct Output {
        float aileron_deg = 0.0f;
        float elevator_deg = 0.0f;
        float rudder_deg = 0.0f;
    };

    explicit AttitudeController(const AttitudeControllerConfig& config = AttitudeControllerConfig{});

    Output update(float nav_roll_deg, float nav_pitch_deg, const ImuData& imu,
                  float airspeed_mps, float dt_s);

    void resetIntegrators();
    void setIntegralEnabled(bool enabled);

    float rollIntegratorState() const;
    float pitchIntegratorState() const;
    float lastSpeedScaler() const;
    float lastYawRateSetpointDps() const;

private:
    float computeSpeedScaler(float airspeed_mps) const;
    static float clampToOutputLimit(float value_deg, float limit_deg);

    AttitudeControllerConfig config_{};
    float roll_integral_ = 0.0f;
    float pitch_integral_ = 0.0f;
    bool integral_enabled_ = true;

    float last_speed_scaler_ = 1.0f;
};

}  // namespace fc
