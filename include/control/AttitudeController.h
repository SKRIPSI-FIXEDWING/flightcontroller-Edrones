#pragma once

#include "control/LqrAxisController.h"
#include "drivers/Imu.h"

namespace fc {

struct AttitudeControllerConfig {
    // Defaults computed by tools/lqr_gain_design/lqr_gain_design.py (cruise
    // trim @ 18 m/s, 200 Hz zero-order-hold discretization, Bryson's-rule
    // Q/R against the assumed stability/control derivatives) — see
    // docs/attitude-lqr.md. Field order: {mode, k_primary, k_rate,
    // k_integral, integral_limit, output_limit_deg}. Re-run the script and
    // update these if the airframe's mass/derivatives/limits change; these
    // are NOT flight-validated, only pole/damping-checked in the script
    // across the 14-22 m/s envelope.
    LqrAxisConfig roll{LqrStateMode::AngleAndRate, 3.449792f, 0.409788f, 1.393631f, 17.2f, 25.0f};
    LqrAxisConfig pitch{LqrStateMode::AngleAndRate, 5.205471f, 0.661225f, 1.630465f, 14.9f, 25.0f};
    LqrAxisConfig yaw{LqrStateMode::RateOnly, 1.021439f, 0.0f, 0.0f, 0.0f, 25.0f};

    // Speed scaler: (trim_airspeed_mps / measured_airspeed_mps)^2, clamped to
    // [scaler_min, scaler_max], multiplies each axis's raw LQR output before
    // the final output_limit_deg clamp. See docs/attitude-lqr.md for why a
    // single fixed K plus this post-multiplier is used instead of a
    // gain-scheduled K.
    float trim_airspeed_mps = 18.0f;
    float scaler_min = 0.6f;
    float scaler_max = 1.8f;
    float min_airspeed_for_scaling_mps = 3.0f;  // guards the scaler's divide-by-airspeed

    // Minimum airspeed used in the yaw axis's coordinated-turn feedforward
    // (r_cmd = g*tan(roll)/V) — guards against divide-by-near-zero at
    // very low speed.
    float min_airspeed_for_turn_rate_mps = 5.0f;
};

/**
 * Fixed-wing 3-axis LQR attitude controller: replaces the legacy per-axis
 * rate-PID stack (FW_roll/pitch/yaw_controller.h) with one
 * LqrAxisController instance per axis (roll and pitch integral-augmented
 * for disturbance rejection — the thesis's external-roll-disturbance
 * scenario; yaw rate-only, commanded to a kinematic coordinated-turn rate
 * computed from the roll setpoint, replacing the legacy rudder-mixing
 * role only — L1/TECS's own load-factor/bank-angle logic is untouched).
 *
 * State feedback is read directly from fc::Imu — no new estimator.
 */
class AttitudeController final {
public:
    struct Output {
        float aileron_deg = 0.0f;
        float elevator_deg = 0.0f;
        float rudder_deg = 0.0f;
    };

    explicit AttitudeController(const AttitudeControllerConfig& config = AttitudeControllerConfig{});

    /**
     * nav_roll_deg/nav_pitch_deg: setpoints from Navigation (L1/TECS demand).
     * imu: latest attitude/rate snapshot. airspeed_mps: for the speed
     * scaler and yaw feedforward. dt: seconds since the last call.
     */
    Output update(float nav_roll_deg, float nav_pitch_deg, const ImuData& imu,
                 float airspeed_mps, float dt);

    void resetIntegrators();
    void setIntegralEnabled(bool enabled);

    float rollIntegratorState() const;
    float pitchIntegratorState() const;
    float lastSpeedScaler() const;
    float lastYawRateSetpointDps() const;

private:
    static float computeCoordinatedTurnRateDps(float roll_deg, float airspeed_mps,
                                               float min_airspeed_mps);
    float computeSpeedScaler(float airspeed_mps) const;
    static float clampToOutputLimit(float value_deg, float limit_deg);

    AttitudeControllerConfig config_{};
    LqrAxisController roll_;
    LqrAxisController pitch_;
    LqrAxisController yaw_;

    float last_speed_scaler_ = 1.0f;
    float last_yaw_rate_setpoint_dps_ = 0.0f;
};

}  // namespace fc
