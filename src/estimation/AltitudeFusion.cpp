#include "estimation/AltitudeFusion.h"

#include <math.h>

namespace fc {

AltitudeFusion::AltitudeFusion(const AltitudeFusionConfig& config)
    : config_(config)
{
}

void AltitudeFusion::reset()
{
    initialized_ = false;
    height_ = climb_ = accel_bias_ = 0.0f;
    baro_bias_ = 0.0f;
    baro_reject_since_s_ = -1.0f;
    baro_last_reject_s_ = -1.0e9f;
    baro_in_gate_since_s_ = -1.0f;
    baro_rejected_ = false;
    gps_ref_set_ = false;
    gps_ref_ = 0.0f;
    last_gps_accept_s_ = -1.0e9f;
    gps_reject_since_s_ = -1.0f;
    for (auto& row : p_) {
        for (float& v : row) {
            v = 0.0f;
        }
    }
}

bool AltitudeFusion::gpsActive() const
{
    return (time_s_ - last_gps_accept_s_) <= config_.gps_timeout_s;
}

void AltitudeFusion::predict(float accel_up_mss, float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.5f) {
        return;
    }
    time_s_ += dt_s;
    if (!initialized_) {
        return;
    }

    const float a = accel_up_mss - accel_bias_;
    height_ += climb_ * dt_s + 0.5f * a * dt_s * dt_s;
    climb_ += a * dt_s;

    // P <- F P F^T + Q,  F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
    const float dt2 = dt_s * dt_s;
    float fp[3][3];
    for (int j = 0; j < 3; ++j) {
        fp[0][j] = p_[0][j] + dt_s * p_[1][j] - 0.5f * dt2 * p_[2][j];
        fp[1][j] = p_[1][j] - dt_s * p_[2][j];
        fp[2][j] = p_[2][j];
    }
    float np[3][3];
    for (int i = 0; i < 3; ++i) {
        np[i][0] = fp[i][0] + dt_s * fp[i][1] - 0.5f * dt2 * fp[i][2];
        np[i][1] = fp[i][1] - dt_s * fp[i][2];
        np[i][2] = fp[i][2];
    }

    const float q = config_.accel_noise_mss * config_.accel_noise_mss;
    np[0][0] += q * dt2 * dt_s / 3.0f;
    np[0][1] += q * dt2 / 2.0f;
    np[1][0] += q * dt2 / 2.0f;
    np[1][1] += q * dt_s;
    np[2][2] += config_.accel_bias_walk * config_.accel_bias_walk * dt_s;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            p_[i][j] = np[i][j];
        }
    }
}

// Scalar measurement of state `index` (H = e_index).
void AltitudeFusion::scalarUpdate(int index, float innovation, float measurement_variance)
{
    const float s = p_[index][index] + measurement_variance;
    float k[3];
    for (int i = 0; i < 3; ++i) {
        k[i] = p_[i][index] / s;
    }

    height_ += k[0] * innovation;
    climb_ += k[1] * innovation;
    accel_bias_ += k[2] * innovation;

    float row[3];
    for (int j = 0; j < 3; ++j) {
        row[j] = p_[index][j];
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            p_[i][j] -= k[i] * row[j];
        }
    }
    for (int i = 0; i < 3; ++i) {  // keep P symmetric against rounding
        for (int j = i + 1; j < 3; ++j) {
            const float m = 0.5f * (p_[i][j] + p_[j][i]);
            p_[i][j] = p_[j][i] = m;
        }
    }
}

void AltitudeFusion::updateBaro(float baro_altitude_m)
{
    const float dt_since_last = time_s_ - last_baro_time_s_;
    last_baro_time_s_ = time_s_;

    if (!initialized_) {
        height_ = baro_altitude_m;
        climb_ = 0.0f;
        accel_bias_ = 0.0f;
        baro_bias_ = 0.0f;
        for (auto& row : p_) {
            for (float& v : row) {
                v = 0.0f;
            }
        }
        p_[0][0] = config_.baro_noise_m * config_.baro_noise_m;
        p_[1][1] = 1.0f;
        p_[2][2] = config_.initial_accel_bias_sigma_mss * config_.initial_accel_bias_sigma_mss;
        initialized_ = true;
        return;
    }

    const float r_var = config_.baro_noise_m * config_.baro_noise_m;
    const float innovation = baro_altitude_m - baro_bias_ - height_;
    const float s = p_[0][0] + r_var;
    const float gate = config_.baro_gate_sigma;

    if (innovation * innovation > gate * gate * s) {
        // Inconsistent with accel-propagated height: disturbance candidate.
        baro_rejected_ = true;
        baro_reject_count_ += 1U;
        baro_last_reject_s_ = time_s_;
        baro_in_gate_since_s_ = -1.0f;
        if (baro_reject_since_s_ < 0.0f) {
            baro_reject_since_s_ = time_s_;
        }
        const float rejected_for_s = time_s_ - baro_reject_since_s_;

        if (gpsActive() && rejected_for_s >= config_.baro_reanchor_s) {
            // GPS (height + velocity) still agrees with our state, the baro
            // does not: it is the baro that moved. Absorb the step into its
            // offset instead of the altitude.
            baro_bias_ += innovation;
            baro_reanchor_count_ += 1U;
            baro_reject_since_s_ = -1.0f;
        } else if (!gpsActive() && rejected_for_s >= config_.baro_no_gps_reset_s) {
            // Nobody to arbitrate: follow the baro rather than ignore it.
            height_ = baro_altitude_m - baro_bias_;
            climb_ = 0.0f;
            p_[0][0] = r_var;
            baro_reject_since_s_ = -1.0f;
            baro_rejected_ = false;
        }
        return;
    }

    baro_reject_since_s_ = -1.0f;

    // Recovery hysteresis: a disturbed baro keeps wandering across the gate
    // edge; do not trust it again until it has been consistent for a while.
    if ((time_s_ - baro_last_reject_s_) < 30.0f) {
        if (baro_in_gate_since_s_ < 0.0f) {
            baro_in_gate_since_s_ = time_s_;
        }
        if ((time_s_ - baro_in_gate_since_s_) < config_.baro_recovery_s) {
            baro_rejected_ = true;  // still on hold
            return;
        }
    }
    baro_rejected_ = false;

    // Huber-style soft weighting of marginal samples.
    float effective_var = r_var;
    const float soft = config_.baro_soft_sigma;
    if (innovation * innovation > soft * soft * s) {
        const float scale = fabsf(innovation) / (soft * sqrtf(s));
        effective_var = r_var * scale * scale;
    }

    if (gpsActive() && dt_since_last > 0.0f) {
        // Slowly learn the baro offset against the GPS-anchored height.
        float gain = dt_since_last / config_.baro_bias_tau_s;
        if (gain > 1.0f) {
            gain = 1.0f;
        }
        baro_bias_ += gain * innovation;
        scalarUpdate(0, baro_altitude_m - baro_bias_ - height_, effective_var);
        return;
    }
    scalarUpdate(0, innovation, effective_var);
}

void AltitudeFusion::updateGps(float gps_msl_m, float gps_climb_rate_mps, bool gps_good)
{
    if (!initialized_ || !gps_good) {
        return;
    }

    if (!gps_ref_set_) {
        // Tie GPS MSL to the baro-relative height once, at first good fix.
        gps_ref_ = gps_msl_m - height_;
        gps_ref_set_ = true;
        last_gps_accept_s_ = time_s_;
        return;
    }

    const float r_var = config_.gps_noise_m * config_.gps_noise_m;
    const float innovation = (gps_msl_m - gps_ref_) - height_;
    const float s = p_[0][0] + r_var;
    const float gate = config_.gps_gate_sigma;

    if (innovation * innovation > gate * gate * s) {
        if (gps_reject_since_s_ < 0.0f) {
            gps_reject_since_s_ = time_s_;
        }
        if (time_s_ - gps_reject_since_s_ >= config_.gps_rereference_s) {
            gps_ref_ = gps_msl_m - height_;
            gps_reject_since_s_ = -1.0f;
            last_gps_accept_s_ = time_s_;
        }
        return;
    }

    gps_reject_since_s_ = -1.0f;
    last_gps_accept_s_ = time_s_;
    scalarUpdate(0, innovation, r_var);

    // Vertical velocity: constrains climb rate directly (and, via the
    // height/climb/bias correlations, the accelerometer bias).
    const float v_var = config_.gps_climb_noise_mps * config_.gps_climb_noise_mps;
    const float v_innovation = gps_climb_rate_mps - climb_;
    const float v_s = p_[1][1] + v_var;
    if (v_innovation * v_innovation <= 16.0f * v_s) {
        scalarUpdate(1, v_innovation, v_var);
    }
}

AltitudeFusionData AltitudeFusion::data() const
{
    AltitudeFusionData d{};
    d.altitude_m = height_;
    d.climb_rate_mps = climb_;
    d.accel_bias_mss = accel_bias_;
    d.baro_bias_m = baro_bias_;
    d.altitude_sigma_m = p_[0][0] > 0.0f ? sqrtf(p_[0][0]) : 0.0f;
    d.baro_rejected = baro_rejected_;
    d.gps_active = gpsActive();
    d.valid = initialized_;
    d.baro_reject_count = baro_reject_count_;
    d.baro_reanchor_count = baro_reanchor_count_;
    return d;
}

}  // namespace fc
