#include "control/LqrAxisController.h"

namespace fc {

LqrAxisController::LqrAxisController(const LqrAxisConfig& config)
    : config_(config)
{
}

float LqrAxisController::update(float setpoint, float measured_primary, float measured_rate, float dt)
{
    const float primary_error = setpoint - measured_primary;

    if (config_.k_integral != 0.0f && integral_enabled_) {
        integral_state_ += primary_error * dt;
        if (integral_state_ > config_.integral_limit) {
            integral_state_ = config_.integral_limit;
        } else if (integral_state_ < -config_.integral_limit) {
            integral_state_ = -config_.integral_limit;
        }
    }

    const float rate_term =
        (config_.mode == LqrStateMode::AngleAndRate) ? -config_.k_rate * measured_rate : 0.0f;

    return config_.k_primary * primary_error + rate_term + config_.k_integral * integral_state_;
}

void LqrAxisController::resetIntegrator()
{
    integral_state_ = 0.0f;
}

void LqrAxisController::setIntegralEnabled(bool enabled)
{
    integral_enabled_ = enabled;
    if (!enabled) {
        integral_state_ = 0.0f;
    }
}

float LqrAxisController::integratorState() const
{
    return integral_state_;
}

const LqrAxisConfig& LqrAxisController::config() const
{
    return config_;
}

}  // namespace fc
