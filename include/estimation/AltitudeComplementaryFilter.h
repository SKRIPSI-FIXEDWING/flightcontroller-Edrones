#pragma once

#include <math.h>
#include <stdint.h>

namespace fc {

/**
 * Upward (positive = accelerating up) vertical acceleration, in m/s^2, from
 * the BNO055's on-chip gravity-free "linear acceleration" and gravity vector
 * (both in the sensor frame, same units/sign convention: gravity reads
 * +g along the axis that points UP when the board is at rest).
 *
 * a_up = linear_acceleration . (gravity / |gravity|)
 *
 * Why this instead of rotating raw accel with Euler angles and subtracting
 * g (what AhrsData::acceleration_body_frame_mss does): that path (a) adds
 * gravity instead of cancelling it for a Z-up sensor -- a constant ~-2g
 * (-19.6 m/s^2) bias, which this filter's Ki turns into a ~-98 m altitude
 * offset -- and (b) depends on the Euler-angle convention matching
 * Quaternion::from_euler(), so its error varies with orientation (seen in
 * the logs as comp_accel_up_mss swinging between -4 and -27 m/s^2 while the
 * board was only being rotated). Projecting onto the gravity direction
 * needs no Euler angles, no frame convention and no "+/- g" -- it is
 * independent of yaw by construction.
 *
 * Returns 0 if the gravity vector is not valid yet (|g| < 1 m/s^2).
 */
inline float verticalAccelUpMss(float linear_x, float linear_y, float linear_z,
                                float gravity_x, float gravity_y, float gravity_z)
{
    const float gravity_norm =
        sqrtf(gravity_x * gravity_x + gravity_y * gravity_y + gravity_z * gravity_z);
    if (gravity_norm < 1.0f) {
        return 0.0f;
    }
    return (linear_x * gravity_x + linear_y * gravity_y + linear_z * gravity_z) / gravity_norm;
}

/** One coherent complementary-filter altitude estimate. */
struct AltitudeComplementaryData {
    float altitude_m = 0.0f;
    float climb_rate_mps = 0.0f;

    /** The accel_up_mss input update() was called with -- diagnostic only,
     * so a stuck bias/sign bug upstream (AhrsData::acceleration_body_frame_mss)
     * is directly visible in logged data instead of only inferable from its
     * effect on altitude_m. See docs/altitude-complementary-filter.md. */
    float last_accel_up_mss = 0.0f;

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
