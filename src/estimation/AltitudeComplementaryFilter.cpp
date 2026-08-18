#include "estimation/AltitudeComplementaryFilter.h"

#include <Arduino.h>

namespace fc {

AltitudeComplementaryFilter::AltitudeComplementaryFilter(const AltitudeComplementaryConfig& config)
    : config_(config)
{
}

void AltitudeComplementaryFilter::reset(float altitude_m)
{
    altitude_m_ = altitude_m;
    velocity_mps_ = 0.0f;
}

void AltitudeComplementaryFilter::update(float baro_altitude_m, float accel_up_mss, float dt_s)
{
    if (dt_s <= 0.0f) {
        return;
    }

    // Predict: double-integrate accelerometer-derived vertical motion.
    // Unconstrained, this alone would drift (accelerometer bias integrates
    // into velocity error, which integrates into unbounded altitude error).
    velocity_mps_ += accel_up_mss * dt_s;
    altitude_m_ += velocity_mps_ * dt_s;

    // Correct: pull both states toward the barometer, bounding that drift.
    // This is the complementary-filter step -- see AltitudeComplementaryConfig's
    // doc comment for the reference derivation.
    const float error_m = baro_altitude_m - altitude_m_;
    velocity_mps_ += config_.ki * error_m * dt_s;
    altitude_m_ += config_.kp * error_m * dt_s;

    AltitudeComplementaryData next{};
    next.altitude_m = altitude_m_;
    next.climb_rate_mps = velocity_mps_;
    next.last_accel_up_mss = accel_up_mss;
    next.timestamp_us = micros();
    next.sequence = data_.sequence + 1U;
    next.valid = true;
    data_ = next;
}

AltitudeComplementaryData AltitudeComplementaryFilter::data() const
{
    return data_;
}

}  // namespace fc
