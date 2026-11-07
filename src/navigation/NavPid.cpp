#include "navigation/NavPid.h"

#include <math.h>

namespace fc {

void NavPid::setEnabled(bool on)
{
    if (on != enabled()) {
        integrator_mss_ = 0.0f;
    }
    config_.enable = on ? 1.0f : 0.0f;
}

void NavPid::setGains(float kp, float kd, float ki)
{
    config_.kp = kp;
    config_.kd = kd;
    if (ki != config_.ki) {
        integrator_mss_ = 0.0f;
    }
    config_.ki = ki;
}

float NavPid::interceptErrorLimit(float groundspeed_mps, float max_intercept_rad) const
{
    if (config_.kp <= 0.0f) {
        return 0.0f;
    }
    return config_.kd * fabsf(groundspeed_mps) * sinf(max_intercept_rad) / config_.kp;
}

float NavPid::update(float error_m, float error_rate_mps, float dt_s, float error_limit_m)
{
    if (!isfinite(error_m) || !isfinite(error_rate_mps)) {
        return 0.0f;
    }
    bool beyond_limit = false;
    if (error_limit_m > 0.0f && fabsf(error_m) > error_limit_m) {
        error_m = (error_m > 0.0f) ? error_limit_m : -error_limit_m;
        beyond_limit = true;
    }
    if (dt_s > 0.0f && dt_s <= 1.0f && config_.ki > 0.0f && !beyond_limit) {
        integrator_mss_ += config_.ki * error_m * dt_s;
        const float lim = fabsf(config_.i_limit_mss);
        if (integrator_mss_ > lim) integrator_mss_ = lim;
        if (integrator_mss_ < -lim) integrator_mss_ = -lim;
    } else if (config_.ki <= 0.0f) {
        integrator_mss_ = 0.0f;
    }
    return config_.kp * error_m + config_.kd * error_rate_mps + integrator_mss_;
}

}  // namespace fc
