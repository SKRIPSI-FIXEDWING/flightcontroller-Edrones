#include "control/AttitudeController.h"

#include <AP_Math.h>
#include <definitions.h>
#include <math.h>

namespace fc {

AttitudeController::AttitudeController(const AttitudeControllerConfig& config)
    : config_(config), roll_(config.roll), pitch_(config.pitch), yaw_(config.yaw)
{
}

float AttitudeController::computeCoordinatedTurnRateDps(float roll_deg, float airspeed_mps,
                                                        float min_airspeed_mps)
{
    const float safe_airspeed = MAX(airspeed_mps, min_airspeed_mps);
    const float rate_rad_s = (GRAVITY_MSS * tanf(radians(roll_deg))) / safe_airspeed;
    return degrees(rate_rad_s);
}

float AttitudeController::computeSpeedScaler(float airspeed_mps) const
{
    const float safe_airspeed = MAX(airspeed_mps, config_.min_airspeed_for_scaling_mps);
    const float ratio = config_.trim_airspeed_mps / safe_airspeed;
    float scaler = ratio * ratio;
    scaler = constrain_float(scaler, config_.scaler_min, config_.scaler_max);
    return scaler;
}

float AttitudeController::clampToOutputLimit(float value_deg, float limit_deg)
{
    return constrain_float(value_deg, -limit_deg, limit_deg);
}

AttitudeController::Output AttitudeController::update(float nav_roll_deg, float nav_pitch_deg,
                                                       const ImuData& imu, float airspeed_mps,
                                                       float dt)
{
    const float scaler = computeSpeedScaler(airspeed_mps);
    last_speed_scaler_ = scaler;

    const float roll_raw = roll_.update(nav_roll_deg, imu.roll_deg, imu.angular_rate_dps.x, dt);
    const float pitch_raw = pitch_.update(nav_pitch_deg, imu.pitch_deg, imu.angular_rate_dps.y, dt);

    const float yaw_rate_setpoint_dps = computeCoordinatedTurnRateDps(
        nav_roll_deg, airspeed_mps, config_.min_airspeed_for_turn_rate_mps);
    last_yaw_rate_setpoint_dps_ = yaw_rate_setpoint_dps;
    const float yaw_raw = yaw_.update(yaw_rate_setpoint_dps, imu.angular_rate_dps.z, 0.0f, dt);

    Output output{};
    output.aileron_deg = clampToOutputLimit(roll_raw * scaler, roll_.config().output_limit_deg);
    output.elevator_deg = clampToOutputLimit(pitch_raw * scaler, pitch_.config().output_limit_deg);
    output.rudder_deg = clampToOutputLimit(yaw_raw * scaler, yaw_.config().output_limit_deg);
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
