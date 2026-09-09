#pragma once

#include <stdint.h>

namespace fc {

/** One coherent Mahony-filter attitude estimate. */
struct AttitudeMahonyData {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;

    // Hamilton quaternion (w, x, y, z), body-to-earth.
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    uint32_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

struct AttitudeMahonyConfig {
    // Proportional + integral feedback gains, following Mahony, Hamel &
    // Pflimlin (2008), "Nonlinear Complementary Filters on the Special
    // Orthogonal Group", IEEE Trans. Automatic Control 53(5) -- see
    // docs/attitude-mahony-filter.md for the full derivation and how these
    // compare to BNO055's on-chip fusion gains (opaque, not user-tunable).
    // kp pulls the gyro-integrated attitude toward the accel/mag reference
    // each step; ki accumulates a bias estimate so gyro bias doesn't drift
    // the correction away over time.
    // Reduced from the previous aggressive values (2.0/0.005). The old Kp
    // made hand acceleration and magnetic disturbances immediately appear
    // as large attitude corrections.
    float kp = 0.8f;
    float ki = 0.001f;

    // Reject accelerometer tilt correction while the measured magnitude is
    // too far from 1 g. In that condition the accelerometer is measuring
    // aircraft/hand acceleration, not gravity alone.
    float gravity_mss = 9.80665f;
    float accel_rejection_mss = 2.5f;

    // Reject implausible or suddenly distorted magnetic-field magnitudes.
    // Typical Earth field is roughly 25..65 uT; the wider limits avoid false
    // rejection while still catching severe motor/steel interference.
    float mag_min_ut = 10.0f;
    float mag_max_ut = 100.0f;
    float mag_norm_change_ratio = 0.30f;

    // Project magnetic feedback onto the gravity axis. Magnetometer then
    // corrects heading only and cannot directly pull roll/pitch when yawing.
    bool magnetometer_yaw_only = true;
};

/**
 * 9-axis (gyro+accel+mag) Mahony AHRS attitude estimator (Option 2), for
 * side-by-side comparison against BNO055's on-chip NDOF fusion (Option 1,
 * fc::Imu's roll_deg/pitch_deg/yaw_deg) -- the comparison this thesis is
 * built around. NOT wired into AttitudeController/TECS/Navigation: runs in
 * parallel and is logged alongside the on-chip estimate purely for bench/
 * flight comparison, mirroring fc::AltitudeComplementaryFilter's role for
 * altitude. See docs/attitude-mahony-filter.md.
 *
 * Axis-convention-agnostic by design (like AltitudeComplementaryFilter takes
 * an already sign-corrected accel_up_mss): update()'s gyro/accel/mag inputs
 * are used exactly as given, in whatever body frame the caller passes. The
 * caller is responsible for feeding a frame that's actually comparable to
 * fc::Imu's roll_deg/pitch_deg/yaw_deg -- see the integration note in
 * docs/attitude-mahony-filter.md about why this is NOT assumed correct
 * out of the box for raw magnetometer/accelerometer axes.
 */
class AttitudeMahonyFilter final {
public:
    explicit AttitudeMahonyFilter(const AttitudeMahonyConfig& config = {});

    /**
     * Call once per IMU sample. gyro_x/y/z_rps are in rad/s; accel_x/y/z
     * and mag_x/y/z are each in any consistent unit (only direction matters
     * -- both are normalized internally). dt_s must be > 0 or the call is a
     * no-op.
     */
    void update(float gyro_x_rps, float gyro_y_rps, float gyro_z_rps, float accel_x, float accel_y,
               float accel_z, float mag_x, float mag_y, float mag_z, float dt_s);

    /** Resets to the identity quaternion and clears the integral bias estimate. */
    void reset();

    AttitudeMahonyData data() const;

private:
    AttitudeMahonyConfig config_{};

    float q0_ = 1.0f;
    float q1_ = 0.0f;
    float q2_ = 0.0f;
    float q3_ = 0.0f;

    float integral_fb_x_ = 0.0f;
    float integral_fb_y_ = 0.0f;
    float integral_fb_z_ = 0.0f;

    float mag_norm_reference_ = 0.0f;
    bool has_mag_norm_reference_ = false;

    AttitudeMahonyData data_{};
};

}  // namespace fc
