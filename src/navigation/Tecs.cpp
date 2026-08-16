#include "navigation/Tecs.h"

#include <definitions.h>
#include <math.h>

namespace fc {

Tecs::Tecs(const TecsConfig& config)
    : config_(config)
{
}

float Tecs::timeConstant() const
{
    if (config_.time_const_s < 0.1f) {
        return 0.1f;
    }
    return config_.time_const_s;
}

void Tecs::updateSpeed(float dt, const AhrsData& ahrs, const ImuData& imu,
                       const BarometerData& baro, const AirspeedData& airspeed)
{
    if (flags_.reset) {
        vdot_filter_.reset();
        vel_dot_lpf_ = vel_dot_;
    } else {
        const Matrix3f& rot_mat = ahrs.rotation_body_to_ned;
        const float temp = rot_mat.c.x * GRAVITY_MSS + imu.acceleration_mss.x;
        vel_dot_ = vdot_filter_.apply(temp);
        const float alpha = dt / (dt + timeConstant());
        vel_dot_lpf_ = vel_dot_lpf_ * (1.0f - alpha) + vel_dot_ * alpha;
    }

    const bool use_airspeed = use_synthetic_airspeed_once_ || use_synthetic_airspeed_ ||
                              ahrs.airspeed_sensor_enabled ||
                              (ahrs.gps_lock && ahrs.groundspeed_mps > 3.0f);

    const float eas2tas = baro.eas2tas;
    tas_dem_ = eas_dem_ * eas2tas;
    if (flags_.reset || !use_airspeed) {
        tas_max_ = config_.max_airspeed_mps * eas2tas;
    } else if (thr_clip_status_ == ClipStatus::kMax) {
        const float vel_rate_min = 0.5f * stedot_min_ / MAX(tas_state_, config_.min_airspeed_mps * eas2tas);
        tas_max_ += dt_ * vel_rate_min;
        tas_max_ = MAX(tas_max_, config_.cruise_airspeed_mps * eas2tas);
    } else {
        const float vel_rate_max = 0.5f * stedot_max_ / MAX(tas_state_, config_.min_airspeed_mps * eas2tas);
        tas_max_ += dt_ * vel_rate_max;
    }
    tas_max_ = MIN(tas_max_, config_.max_airspeed_mps * eas2tas);
    tas_min_ = config_.min_airspeed_mps * eas2tas;

    if (tas_max_ < tas_min_) {
        tas_max_ = tas_min_;
    }

    // Which airspeed source is in effect: 1=sensor, 2=GPS-synthetic, 3=cruise fallback.
    if (ahrs.airspeed_sensor_enabled) {
        eas_ = constrain_float(airspeed.velocity_mps, config_.min_airspeed_mps, config_.max_airspeed_mps);
    } else if (ahrs.gps_lock && ahrs.groundspeed_mps > 3.0f) {
        eas_ = constrain_float(ahrs.groundspeed_mps, config_.min_airspeed_mps, config_.max_airspeed_mps);
    } else {
        eas_ = constrain_float(config_.cruise_airspeed_mps, config_.min_airspeed_mps, config_.max_airspeed_mps);
    }

    constexpr float kMinAirspeed = 3.0f;

    if (flags_.reset) {
        tas_state_ = eas_ * eas2tas;
        tas_state_ = MAX(tas_state_, config_.min_airspeed_mps);
        integ_dtas_state_ = 0.0f;
        return;
    }

    const float aspd_err = (eas_ * eas2tas) - tas_state_;

    // Fast convergence: snap directly on a large error (cold start / mode
    // switch) instead of waiting for the filter to converge over dozens of
    // iterations.
    if (fabsf(aspd_err) > 5.0f) {
        tas_state_ = eas_ * eas2tas;
        tas_state_ = MAX(tas_state_, kMinAirspeed);
        integ_dtas_state_ = 0.0f;
        return;
    }

    float integ_dtas_input = aspd_err * config_.spd_comp_filter_omega * config_.spd_comp_filter_omega;
    if (tas_state_ < 3.1f) {
        integ_dtas_input = MAX(integ_dtas_input, 0.0f);
    }
    integ_dtas_state_ = integ_dtas_state_ + integ_dtas_input * dt;
    const float tas_input =
        integ_dtas_state_ + vel_dot_ + aspd_err * config_.spd_comp_filter_omega * 1.4142f;
    tas_state_ = tas_state_ + tas_input * dt;
    tas_state_ = MAX(tas_state_, kMinAirspeed);
}

void Tecs::update50Hz(const AhrsData& ahrs, const BarometerData& baro, const ImuData& imu,
                      const AirspeedData& airspeed)
{
    // NED down is positive; height-above-home for TECS wants up-positive.
    height_ = -(static_cast<float>(ahrs.position.alt) / 100.0f);

    const uint64_t now = micros();
    float dt = (now - update_50hz_last_usec_) * 1.0e-6f;
    flags_.reset = dt > 1.0f;
    if (flags_.reset) {
        climb_rate_ = 0.0f;
        height_filter_.dd_height = 0.0f;
        dt = 0.02f;
        vdot_filter_.reset();
    }
    update_50hz_last_usec_ = now;

    // Use inertial-nav vertical velocity (matches legacy's `if (true)` branch
    // — the complementary-filter alternative below it was dead code, never
    // taken; kept here as a documented fallback would require the branch
    // legacy never exercised, so it is not ported).
    const Vector3f velned = ahrs.velocity_ned_mps;
    climb_rate_ = -velned.z;

    updateSpeed(dt, ahrs, imu, baro, airspeed);
}

void Tecs::updateSpeedDemand()
{
    if (flags_.bad_descent || flags_.underspeed) {
        tas_dem_ = tas_min_;
    }

    tas_dem_ = constrain_float(tas_dem_, tas_min_, tas_max_);

    const float vel_rate_max = 0.5f * stedot_max_ / MAX(tas_state_, 3.0f);
    const float vel_rate_min = 0.5f * stedot_min_ / MAX(tas_state_, 3.0f);
    const float tas_dem_previous = tas_dem_adj_;

    if ((tas_dem_ - tas_dem_previous) > (vel_rate_max * dt_)) {
        tas_dem_adj_ = tas_dem_previous + vel_rate_max * dt_;
        tas_rate_dem_ = vel_rate_max;
    } else if ((tas_dem_ - tas_dem_previous) < (vel_rate_min * dt_)) {
        tas_dem_adj_ = tas_dem_previous + vel_rate_min * dt_;
        tas_rate_dem_ = vel_rate_min;
    } else {
        tas_rate_dem_ = (tas_dem_ - tas_dem_previous) / dt_;
        tas_dem_adj_ = tas_dem_;
    }

    if (flags_.reset) {
        tas_dem_adj_ = tas_state_;
        tas_rate_dem_lpf_ = tas_rate_dem_;
    } else {
        const float alpha = dt_ / (dt_ + timeConstant());
        tas_rate_dem_lpf_ = tas_rate_dem_lpf_ * (1.0f - alpha) + tas_rate_dem_ * alpha;
    }

    tas_dem_adj_ = constrain_float(tas_dem_adj_, tas_min_, tas_max_);
}

void Tecs::updateHeightDemand()
{
    hgt_dem_ = 0.5f * (hgt_dem_ + hgt_dem_in_old_);
    hgt_dem_in_old_ = hgt_dem_;

    const float max_sink_rate = config_.max_sink_rate_mps;

    if ((hgt_dem_ - hgt_dem_prev_) > (config_.max_climb_rate_mps * dt_)) {
        hgt_dem_ = hgt_dem_prev_ + config_.max_climb_rate_mps * dt_;
    } else if ((hgt_dem_ - hgt_dem_prev_) < (-max_sink_rate * dt_)) {
        hgt_dem_ = hgt_dem_prev_ - max_sink_rate * dt_;
    }
    hgt_dem_prev_ = hgt_dem_;

    const float coef = MIN(dt_ / (dt_ + MAX(timeConstant(), dt_)), 1.0f);
    hgt_dem_adj_ = coef * hgt_dem_ + (1.0f - coef) * hgt_dem_adj_last_;

    const float new_hgt_dem = hgt_dem_adj_;
    hgt_dem_adj_last_ = hgt_dem_adj_;
    hgt_dem_adj_ = new_hgt_dem;

    // CRITICAL FIX (preserved from legacy): height-rate feedforward used to
    // always be zero, causing elevator overshoot. Adaptive time constant
    // gives a faster response for small altitude errors to prevent
    // integrator windup.
    const float hgt_error = hgt_dem_adj_ - height_;

    float adaptive_time_const = config_.time_const_s;
    if (fabsf(hgt_error) < 3.0f) {
        adaptive_time_const = MAX(1.5f, fabsf(hgt_error) * 0.5f + 1.0f);
    }

    hgt_rate_dem_ = hgt_error / adaptive_time_const;
    hgt_rate_dem_ = constrain_float(hgt_rate_dem_, -config_.min_sink_rate_mps, config_.max_climb_rate_mps);
}

void Tecs::detectUnderspeed()
{
    if (flags_.underspeed && tas_state_ >= tas_min_ * 1.15f &&
        millis() - underspeed_start_ms_ > 3000U) {
        flags_.underspeed = false;
    }

    if (((tas_state_ < tas_min_ * 0.9f) && (throttle_dem_ >= thr_maxf_ * 0.95f)) ||
        ((height_ < hgt_dem_adj_) && flags_.underspeed)) {
        flags_.underspeed = true;
        if (tas_state_ < tas_min_ * 0.9f) {
            underspeed_start_ms_ = millis();
        }
    } else {
        flags_.underspeed = false;
    }
}

void Tecs::updateEnergies()
{
    spe_dem_ = hgt_dem_adj_ * GRAVITY_MSS;
    ske_dem_ = 0.5f * tas_dem_adj_ * tas_dem_adj_;

    skedot_dem_ = tas_state_ * (tas_rate_dem_ - tas_rate_dem_lpf_);

    spe_est_ = height_ * GRAVITY_MSS;
    ske_est_ = 0.5f * tas_state_ * tas_state_;

    spedot_ = climb_rate_ * GRAVITY_MSS;
    skedot_ = tas_state_ * (vel_dot_ - vel_dot_lpf_);
}

float Tecs::integratorGain() const
{
    return config_.integrator_gain;
}

void Tecs::updateThrottleWithAirspeed(const AhrsData& ahrs)
{
    const float spe_err_max = MAX(ske_est_ - 0.5f * tas_min_ * tas_min_, 0.0f);
    const float spe_err_min = MIN(ske_est_ - 0.5f * tas_max_ * tas_max_, 0.0f);

    spedot_dem_ = (spe_dem_ - spe_est_) / timeConstant();

    ste_error_ = constrain_float((spe_dem_ - spe_est_), spe_err_min, spe_err_max) + ske_dem_ - ske_est_;
    const float stedot_dem_raw = spedot_dem_ + skedot_dem_;
    float stedot_dem = constrain_float(stedot_dem_raw, stedot_min_, stedot_max_);
    float stedot_error = stedot_dem - spedot_ - skedot_;

    const float filt_coef = 2.0f * dt_;
    stedot_error = filt_coef * stedot_error + (1.0f - filt_coef) * stedot_err_last_;
    stedot_err_last_ = stedot_error;

    if (flags_.underspeed) {
        throttle_dem_ = 1.0f;
    } else {
        const float k_thr2ste = (stedot_max_ - stedot_min_) / (thr_maxf_ - thr_minf_);
        const float k_ste2thr = 1.0f / (timeConstant() * k_thr2ste);

        const float nom_thr = config_.cruise_throttle_percent * 0.01f;
        const Matrix3f& rot_mat = ahrs.rotation_body_to_ned;
        const float cos_phi = sqrtf((rot_mat.a.y * rot_mat.a.y) + (rot_mat.b.y * rot_mat.b.y));
        stedot_dem = stedot_dem +
                    config_.roll_compensation_deg * (1.0f / constrain_float(cos_phi * cos_phi, 0.1f, 1.0f) - 1.0f);
        const float ff_throttle = nom_thr + stedot_dem / k_thr2ste;

        const float throttle_damp = config_.throttle_damp;
        throttle_dem_ = (ste_error_ + stedot_error * throttle_damp) * k_ste2thr + ff_throttle;

        const float thr_minf_clipped_to_zero = constrain_float(thr_minf_, 0.0f, thr_maxf_);

        const float max_amp = 0.5f * (thr_maxf_ - thr_minf_clipped_to_zero);
        const float integ_max = constrain_float((thr_maxf_ - throttle_dem_ + 0.1f), -max_amp, max_amp);
        const float integ_min = constrain_float((thr_minf_ - throttle_dem_ - 0.1f), -max_amp, max_amp);

        integ_thr_state_ = integ_thr_state_ + (ste_error_ * integratorGain()) * dt_ * k_ste2thr;
        integ_thr_state_ = constrain_float(integ_thr_state_, integ_min, integ_max);

        if (config_.throttle_slew_rate_percent_per_s != 0.0f) {
            const float thr_rate_incr =
                dt_ * (thr_maxf_ - thr_minf_clipped_to_zero) * config_.throttle_slew_rate_percent_per_s * 0.01f;
            throttle_dem_ = constrain_float(throttle_dem_, last_throttle_dem_ - thr_rate_incr,
                                            last_throttle_dem_ + thr_rate_incr);
            last_throttle_dem_ = throttle_dem_;
        }

        throttle_dem_ = throttle_dem_ + integ_thr_state_;
    }

    throttle_dem_ = constrain_float(throttle_dem_, thr_minf_, thr_maxf_);
}

void Tecs::updateThrottleWithoutAirspeed(int16_t throttle_nudge, const AhrsData& ahrs)
{
    const float nom_thr = (config_.cruise_throttle_percent + throttle_nudge) * 0.01f;

    if (pitch_dem_ > 0.0f && pitch_maxf_ > 0.0f) {
        throttle_dem_ = nom_thr + (thr_maxf_ - nom_thr) * pitch_dem_ / pitch_maxf_;
    } else if (pitch_dem_ < 0.0f && pitch_minf_ < 0.0f) {
        throttle_dem_ = nom_thr + (thr_minf_ - nom_thr) * pitch_dem_ / pitch_minf_;
    } else {
        throttle_dem_ = nom_thr;
    }

    const Matrix3f& rot_mat = ahrs.rotation_body_to_ned;
    const float cos_phi = sqrtf((rot_mat.a.y * rot_mat.a.y) + (rot_mat.b.y * rot_mat.b.y));
    const float stedot_dem =
        config_.roll_compensation_deg * (1.0f / constrain_float(cos_phi * cos_phi, 0.1f, 1.0f) - 1.0f);
    throttle_dem_ = throttle_dem_ + stedot_dem / (stedot_max_ - stedot_min_) * (thr_maxf_ - thr_minf_);
}

void Tecs::detectBadDescent()
{
    const float stedot = spedot_ + skedot_;
    if ((!flags_.underspeed && (ste_error_ > 200.0f) && (stedot < 0.0f) && (throttle_dem_ >= thr_maxf_ * 0.9f)) ||
        (flags_.bad_descent && !flags_.underspeed && (ste_error_ > 0.0f))) {
        flags_.bad_descent = true;
    } else {
        flags_.bad_descent = false;
    }
}

void Tecs::updatePitch(const ImuData& imu)
{
    ske_weighting_ = constrain_float(config_.speed_weight, 0.0f, 2.0f);
    if (use_synthetic_airspeed_) {
        ske_weighting_ = 0.0f;
    }

    float spe_weighting = 2.0f - ske_weighting_;
    spe_weighting = MIN(spe_weighting, 1.0f);
    ske_weighting_ = MIN(ske_weighting_, 1.0f);

    const float seb_dem = spe_dem_ * spe_weighting - ske_dem_ * ske_weighting_;
    const float seb_est = spe_est_ * spe_weighting - ske_est_ * ske_weighting_;
    const float seb_error = seb_dem - seb_est;

    float sebdot_dem = hgt_rate_dem_ * GRAVITY_MSS * spe_weighting + seb_error / timeConstant();
    const float sebdot_dem_min = -config_.max_sink_rate_mps * GRAVITY_MSS;
    const float sebdot_dem_max = config_.max_climb_rate_mps * GRAVITY_MSS;
    if (sebdot_dem < sebdot_dem_min) {
        sebdot_dem = sebdot_dem_min;
        sebdot_dem_clip_ = ClipStatus::kMin;
    } else if (sebdot_dem > sebdot_dem_max) {
        sebdot_dem = sebdot_dem_max;
        sebdot_dem_clip_ = ClipStatus::kMax;
    } else {
        sebdot_dem_clip_ = ClipStatus::kNone;
    }

    const float sebdot_est = spedot_ * spe_weighting - skedot_ * ske_weighting_;
    const float sebdot_error = sebdot_dem - sebdot_est;

    const float pitch_damp = config_.pitch_damp;
    const float sebdot_dem_total = sebdot_dem + sebdot_error * pitch_damp;

    const float gain_inv = MAX(tas_state_, 3.0f) * GRAVITY_MSS;

    const float integ_sebdot_min = (gain_inv * (pitch_minf_ - radians(5.0f))) - sebdot_dem_total;
    const float integ_sebdot_max = (gain_inv * (pitch_maxf_ + radians(5.0f))) - sebdot_dem_total;

    const float integ_seb_range = integ_sebdot_max - integ_sebdot_min;
    const float integ_seb_delta = constrain_float(sebdot_error * integratorGain() * dt_,
                                                  -integ_seb_range * 0.1f, integ_seb_range * 0.1f);

    pitch_dem_unc_ = (sebdot_dem_total + integ_sebdot_ + integ_seb_delta + integ_ke_) / gain_inv;

    bool inhibit_integrator = ((pitch_dem_unc_ > pitch_maxf_) && integ_seb_delta > 0.0f) ||
                              ((pitch_dem_unc_ < pitch_minf_) && integ_seb_delta < 0.0f);

    // Anti-windup: inhibit when overshooting the height target upward.
    const float alt_error_signed = hgt_dem_adj_ - height_;
    if (alt_error_signed < -0.2f && integ_seb_delta > 0.0f) {
        inhibit_integrator = true;
    }

    if (!inhibit_integrator) {
        integ_sebdot_ += integ_seb_delta;
        integ_ke_ += (ske_est_ - ske_dem_) * ske_weighting_ * dt_ / timeConstant();
    } else {
        const float coef = 1.0f - dt_ / (dt_ + timeConstant());
        integ_sebdot_ *= coef;
        integ_ke_ *= coef;
    }
    integ_sebdot_ = constrain_float(integ_sebdot_, integ_sebdot_min, integ_sebdot_max);
    const float ke_integ_limit = 0.25f * (pitch_maxf_ - pitch_minf_) * gain_inv;
    integ_ke_ = constrain_float(integ_ke_, -ke_integ_limit, ke_integ_limit);

    pitch_dem_unc_ = (sebdot_dem_total + integ_sebdot_ + integ_ke_) / gain_inv;

    pitch_dem_ = constrain_float(pitch_dem_unc_, pitch_minf_, pitch_maxf_);

    const float ptch_rate_incr = dt_ * config_.vertical_accel_limit_mps2 / MAX(tas_state_, 3.0f);
    if ((pitch_dem_ - last_pitch_dem_) > ptch_rate_incr) {
        pitch_dem_ = last_pitch_dem_ + ptch_rate_incr;
    } else if ((pitch_dem_ - last_pitch_dem_) < -ptch_rate_incr) {
        pitch_dem_ = last_pitch_dem_ - ptch_rate_incr;
    }
    if (isnan(pitch_dem_) || isinf(pitch_dem_)) {
        // Fixed unit bug: legacy assigned `imu.pitch` (degrees) into a
        // radians field here. See docs/tecs.md.
        pitch_dem_ = imu.pitch_rad;
        integ_sebdot_ = 0.0f;
        integ_ke_ = 0.0f;
    }
    last_pitch_dem_ = pitch_dem_;
}

void Tecs::initialiseStates(float hgt_afe, const ImuData& imu)
{
    if (dt_ > 1.0f) {
        integ_thr_state_ = 0.0f;
        integ_seb_state_ = 0.0f;
        integ_sebdot_ = 0.0f;
        integ_ke_ = 0.0f;
        last_throttle_dem_ = config_.manual_cruise_throttle_percent * 0.01f;
        last_pitch_dem_ = imu.pitch_rad;
        hgt_dem_adj_last_ = hgt_afe;
        hgt_dem_adj_ = hgt_dem_adj_last_;
        hgt_dem_prev_ = hgt_dem_adj_last_;
        hgt_dem_in_old_ = hgt_dem_adj_last_;
        tas_dem_adj_ = tas_dem_;
        flags_.underspeed = false;
        flags_.bad_descent = false;
        dt_ = 0.2f;
    }
}

void Tecs::updateSteRateLimits()
{
    stedot_max_ = config_.max_climb_rate_mps * GRAVITY_MSS;
    stedot_min_ = -config_.min_sink_rate_mps * GRAVITY_MSS;
}

void Tecs::updatePitchThrottle(int32_t hgt_dem_cm, float eas_dem_mps, int16_t throttle_nudge,
                               float hgt_afe, float load_factor, const AhrsData& ahrs,
                               const BarometerData& baro, const ImuData& imu,
                               const AirspeedData& airspeed)
{
    (void)load_factor;  // Reserved: legacy's aparm.stall_prevention branch was already dead code (commented out).

    const uint64_t now = micros();
    dt_ = (now - update_pitch_throttle_last_usec_) * 1.0e-6f;
    update_pitch_throttle_last_usec_ = now;

    // Self-healing guards, preserved verbatim from legacy (with the pitch
    // fallback unit fixed — see docs/tecs.md).
    if (isnan(tas_state_) || isinf(tas_state_)) {
        tas_state_ = eas_dem_mps > 3.0f ? eas_dem_mps : config_.cruise_airspeed_mps;
        integ_dtas_state_ = 0.0f;
    }
    if (isnan(throttle_dem_) || isinf(throttle_dem_)) throttle_dem_ = config_.cruise_throttle_percent * 0.01f;
    if (isnan(last_throttle_dem_) || isinf(last_throttle_dem_)) last_throttle_dem_ = config_.cruise_throttle_percent * 0.01f;
    if (isnan(pitch_dem_) || isinf(pitch_dem_)) pitch_dem_ = imu.pitch_rad;
    if (isnan(last_pitch_dem_) || isinf(last_pitch_dem_)) last_pitch_dem_ = imu.pitch_rad;
    if (isnan(tas_dem_adj_) || isinf(tas_dem_adj_)) tas_dem_adj_ = tas_dem_;
    if (isnan(tas_rate_dem_) || isinf(tas_rate_dem_)) tas_rate_dem_ = 0.0f;
    if (isnan(integ_thr_state_) || isinf(integ_thr_state_)) integ_thr_state_ = 0.0f;
    if (isnan(integ_sebdot_) || isinf(integ_sebdot_)) integ_sebdot_ = 0.0f;
    if (isnan(integ_ke_) || isinf(integ_ke_)) integ_ke_ = 0.0f;

    hgt_dem_ = hgt_dem_cm * 0.01f;
    eas_dem_ = eas_dem_mps;

    updateSpeed(dt_, ahrs, imu, baro, airspeed);

    thr_maxf_ = config_.max_throttle_percent * 0.01f;
    thr_minf_ = config_.min_throttle_percent * 0.01f;
    thr_maxf_ = MAX(thr_maxf_, thr_minf_ + 0.01f);

    pitch_maxf_ = (config_.pitch_max_override_deg == 0)
                     ? config_.pitch_max_limit_deg
                     : MIN(config_.pitch_max_override_deg, config_.pitch_max_limit_deg);
    pitch_minf_ = (config_.pitch_min_override_deg >= 0)
                     ? config_.pitch_min_limit_deg
                     : MAX(config_.pitch_min_override_deg, config_.pitch_min_limit_deg);

    if (pitch_max_limit_ < 90) {
        pitch_maxf_ = constrain_float(pitch_maxf_, -90.0f, pitch_max_limit_);
        pitch_minf_ = constrain_float(pitch_minf_, -pitch_max_limit_, pitch_maxf_);
        pitch_max_limit_ = 90;
    }

    pitch_maxf_ = radians(pitch_maxf_);
    pitch_minf_ = radians(pitch_minf_);
    pitch_maxf_ = MAX(pitch_maxf_, pitch_minf_);

    initialiseStates(hgt_afe, imu);
    updateSteRateLimits();
    updateSpeedDemand();
    updateHeightDemand();
    detectUnderspeed();
    updateEnergies();

    if (ahrs.airspeed_sensor_enabled || use_synthetic_airspeed_once_ || use_synthetic_airspeed_ ||
        (ahrs.gps_lock && ahrs.groundspeed_mps > 3.0f)) {
        updateThrottleWithAirspeed(ahrs);
        use_synthetic_airspeed_once_ = false;
    } else {
        updateThrottleWithoutAirspeed(throttle_nudge, ahrs);
    }

    detectBadDescent();
    updatePitch(imu);
}

int32_t Tecs::throttleDemand() const
{
    return static_cast<int32_t>(throttle_dem_ * 100.0f);
}

float Tecs::pitchDemandDeg() const
{
    return pitch_dem_ * 57.295781f;
}

float Tecs::velocityXDot() const
{
    return vel_dot_;
}

float Tecs::targetAirspeedMps(float eas2tas) const
{
    return tas_dem_ / eas2tas;
}

float Tecs::maxClimbRateMps() const
{
    return config_.max_climb_rate_mps;
}

void Tecs::resetPitchIntegrator()
{
    integ_seb_state_ = 0.0f;
}

float Tecs::landSinkRateMps() const
{
    return 0.25f;
}

float Tecs::landAirspeedMps() const
{
    return -1.0f;
}

float Tecs::heightRateDemandMps() const
{
    return hgt_rate_dem_;
}

void Tecs::setPathProportion(float path_proportion)
{
    (void)constrain_float(path_proportion, 0.0f, 1.0f);
}

void Tecs::setPitchMaxLimitDeg(int8_t pitch_limit_deg)
{
    pitch_max_limit_ = pitch_limit_deg;
}

void Tecs::useSyntheticAirspeedOnce()
{
    use_synthetic_airspeed_once_ = true;
}

void Tecs::setMinThrottlePercent(float value)
{
    config_.min_throttle_percent = value;
}

void Tecs::setMaxThrottlePercent(float value)
{
    config_.max_throttle_percent = value;
}

float Tecs::minThrottlePercent() const
{
    return config_.min_throttle_percent;
}

float Tecs::maxThrottlePercent() const
{
    return config_.max_throttle_percent;
}

bool Tecs::underspeed() const
{
    return flags_.underspeed;
}

bool Tecs::badDescent() const
{
    return flags_.bad_descent;
}

}  // namespace fc
