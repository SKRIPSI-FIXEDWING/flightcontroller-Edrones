#pragma once

#include <stdint.h>

namespace fc {

/** One coherent complementary-filter altitude estimate. */
struct AltitudeComplementaryData {
    float altitude_m = 0.0f;
    float climb_rate_mps = 0.0f;
    uint32_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

struct AltitudeComplementaryConfig {
    // 2-state (position, velocity) complementary filter gains, following the
    // structure in Sabatini & Genovese (2014), "A Sensor Fusion Method for
    // Tracking Vertical Velocity and Height Based on Inertial and Barometric
    // Altimeter Measurements", Sensors 14(8):13324-13347 -- see
    // docs/altitude-complementary-filter.md for the full derivation, the
    // gain-tuning rule (tau = sigma_v/sigma_w), and how this compares to
    // fc::Barometer's onboard Kalman filter (kept as Option 1, unchanged).
    //
    // kp pulls the double-integrated altitude toward the barometer reading;
    // ki pulls the velocity state the same way, which is what keeps
    // accelerometer bias from integrating into unbounded altitude drift.
    float kp = 1.0f;
    float ki = 0.2f;
};

/**
 * Alternative altitude/climb-rate estimator (Option 2) fusing barometer
 * altitude with BNO055-derived vertical acceleration, for side-by-side
 * comparison against fc::Barometer's onboard Kalman filter (Option 1).
 *
 * NOT wired into TECS/Navigation by default -- this runs in parallel and is
 * logged alongside the Kalman estimate (see fc::DataLogger) purely for bench/
 * flight comparison. Promoting it to the active altitude source for flight
 * control is a separate, larger change once comparison data justifies it;
 * see docs/altitude-complementary-filter.md.
 */
class AltitudeComplementaryFilter final {
public:
    explicit AltitudeComplementaryFilter(const AltitudeComplementaryConfig& config = {});

    /**
     * Call once per control-loop tick with the freshest barometer altitude,
     * upward (positive-up, gravity-compensated) vertical acceleration, and
     * the loop's dt. Matches fc::Barometer's altitude_m sign convention
     * (positive = higher) and climb_rate_mps convention (positive = climbing).
     */
    void update(float baro_altitude_m, float accel_up_mss, float dt_s);

    /** Snap the estimate to a known altitude, e.g. after
     * Barometer::recalibrateGroundPressure() re-zeroes the baro reference. */
    void reset(float altitude_m = 0.0f);

    AltitudeComplementaryData data() const;

private:
    AltitudeComplementaryConfig config_{};
    float altitude_m_ = 0.0f;
    float velocity_mps_ = 0.0f;
    AltitudeComplementaryData data_{};
};

}  // namespace fc
