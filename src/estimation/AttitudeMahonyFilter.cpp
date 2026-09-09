#include "estimation/AttitudeMahonyFilter.h"

#include <Arduino.h>
#include <math.h>

namespace {

constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;

}  // namespace

namespace fc {

AttitudeMahonyFilter::AttitudeMahonyFilter(const AttitudeMahonyConfig& config)
    : config_(config)
{
}

void AttitudeMahonyFilter::reset()
{
    q0_ = 1.0f;
    q1_ = q2_ = q3_ = 0.0f;
    integral_fb_x_ = integral_fb_y_ = integral_fb_z_ = 0.0f;
    mag_norm_reference_ = 0.0f;
    has_mag_norm_reference_ = false;
}

void AttitudeMahonyFilter::update(float gyro_x_rps, float gyro_y_rps, float gyro_z_rps, float accel_x,
                                  float accel_y, float accel_z, float mag_x, float mag_y, float mag_z,
                                  float dt_s)
{
    if (dt_s <= 0.0f) {
        return;
    }

    // Normalize accelerometer. If it reads exactly zero (sensor not ready
    // yet), skip the accel/mag correction step entirely rather than
    // dividing by zero -- integrate gyro only for this sample.
    float norm = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    const float accel_norm = norm;
    const bool accel_magnitude_valid =
        config_.accel_rejection_mss <= 0.0f ||
        fabsf(accel_norm - config_.gravity_mss) <= config_.accel_rejection_mss;
    const bool have_accel = norm > 0.0f && isfinite(norm) && accel_magnitude_valid;
    if (have_accel) {
        accel_x /= norm;
        accel_y /= norm;
        accel_z /= norm;
    }

    norm = sqrtf(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
    const float mag_norm = norm;
    bool mag_magnitude_valid = isfinite(mag_norm) && mag_norm >= config_.mag_min_ut &&
                               mag_norm <= config_.mag_max_ut;
    if (mag_magnitude_valid && has_mag_norm_reference_) {
        const float relative_change = fabsf(mag_norm - mag_norm_reference_) /
                                      fmaxf(mag_norm_reference_, 1.0e-6f);
        mag_magnitude_valid = relative_change <= config_.mag_norm_change_ratio;
    }
    const bool have_mag = have_accel && norm > 0.0f && mag_magnitude_valid;
    if (have_mag) {
        mag_x /= norm;
        mag_y /= norm;
        mag_z /= norm;

        if (!has_mag_norm_reference_) {
            mag_norm_reference_ = mag_norm;
            has_mag_norm_reference_ = true;
        } else {
            // Slow baseline adaptation follows genuine environmental change
            // without accepting a one-sample magnetic spike as the new norm.
            mag_norm_reference_ += 0.002f * (mag_norm - mag_norm_reference_);
        }
    }

    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

    if (have_accel) {
        // Estimated direction of gravity in the body frame, from the
        // current quaternion (this is the third column of the rotation
        // matrix it represents).
        const float halfvx = q1_ * q3_ - q0_ * q2_;
        const float halfvy = q0_ * q1_ + q2_ * q3_;
        const float halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;

        if (have_mag) {
            // Reference magnetic field, rotated into the earth frame using
            // the current quaternion, then reduced to its horizontal (bx)
            // and vertical (bz) components -- this is what keeps yaw
            // observable (gyro-only integration has no absolute yaw
            // reference and drifts freely).
            const float hx =
                2.0f * (mag_x * (0.5f - q2_ * q2_ - q3_ * q3_) + mag_y * (q1_ * q2_ - q0_ * q3_) +
                       mag_z * (q1_ * q3_ + q0_ * q2_));
            const float hy =
                2.0f * (mag_x * (q1_ * q2_ + q0_ * q3_) + mag_y * (0.5f - q1_ * q1_ - q3_ * q3_) +
                       mag_z * (q2_ * q3_ - q0_ * q1_));
            const float hz =
                2.0f * (mag_x * (q1_ * q3_ - q0_ * q2_) + mag_y * (q2_ * q3_ + q0_ * q1_) +
                       mag_z * (0.5f - q1_ * q1_ - q2_ * q2_));
            const float bx = sqrtf(hx * hx + hy * hy);
            const float bz = hz;

            const float halfwx = bx * (0.5f - q2_ * q2_ - q3_ * q3_) + bz * (q1_ * q3_ - q0_ * q2_);
            const float halfwy = bx * (q1_ * q2_ - q0_ * q3_) + bz * (q0_ * q1_ + q2_ * q3_);
            const float halfwz = bx * (q0_ * q2_ + q1_ * q3_) + bz * (0.5f - q1_ * q1_ - q2_ * q2_);

            // Error = sum of the cross products between each measured
            // reference vector (accel, mag) and its currently-estimated
            // direction -- this is the feedback torque that pulls the
            // gyro-integrated quaternion back toward the accel/mag
            // reference.
            const float accel_ex = accel_y * halfvz - accel_z * halfvy;
            const float accel_ey = accel_z * halfvx - accel_x * halfvz;
            const float accel_ez = accel_x * halfvy - accel_y * halfvx;
            const float mag_ex = mag_y * halfwz - mag_z * halfwy;
            const float mag_ey = mag_z * halfwx - mag_x * halfwz;
            const float mag_ez = mag_x * halfwy - mag_y * halfwx;

            halfex = accel_ex;
            halfey = accel_ey;
            halfez = accel_ez;

            if (config_.magnetometer_yaw_only) {
                // 2*halfv is the unit gravity direction in body coordinates.
                // Only the component of magnetic error around gravity is a
                // heading error; perpendicular components would corrupt tilt.
                const float gravity_x = 2.0f * halfvx;
                const float gravity_y = 2.0f * halfvy;
                const float gravity_z = 2.0f * halfvz;
                const float yaw_error = mag_ex * gravity_x + mag_ey * gravity_y + mag_ez * gravity_z;
                halfex += yaw_error * gravity_x;
                halfey += yaw_error * gravity_y;
                halfez += yaw_error * gravity_z;
            } else {
                halfex += mag_ex;
                halfey += mag_ey;
                halfez += mag_ez;
            }
        } else {
            // Accel-only (6-axis) fallback: corrects roll/pitch, yaw is
            // free-running gyro integration with no absolute reference.
            halfex = (accel_y * halfvz - accel_z * halfvy);
            halfey = (accel_z * halfvx - accel_x * halfvz);
            halfez = (accel_x * halfvy - accel_y * halfvx);
        }
    }

    if (config_.ki > 0.0f) {
        integral_fb_x_ += config_.ki * halfex * dt_s;
        integral_fb_y_ += config_.ki * halfey * dt_s;
        integral_fb_z_ += config_.ki * halfez * dt_s;
        gyro_x_rps += integral_fb_x_;
        gyro_y_rps += integral_fb_y_;
        gyro_z_rps += integral_fb_z_;
    } else {
        integral_fb_x_ = integral_fb_y_ = integral_fb_z_ = 0.0f;
    }

    gyro_x_rps += config_.kp * halfex;
    gyro_y_rps += config_.kp * halfey;
    gyro_z_rps += config_.kp * halfez;

    // Integrate the quaternion derivative: dq/dt = 0.5 * q (x) [0, gyro].
    gyro_x_rps *= 0.5f * dt_s;
    gyro_y_rps *= 0.5f * dt_s;
    gyro_z_rps *= 0.5f * dt_s;
    const float qa = q0_;
    const float qb = q1_;
    const float qc = q2_;
    q0_ += (-qb * gyro_x_rps - qc * gyro_y_rps - q3_ * gyro_z_rps);
    q1_ += (qa * gyro_x_rps + qc * gyro_z_rps - q3_ * gyro_y_rps);
    q2_ += (qa * gyro_y_rps - qb * gyro_z_rps + q3_ * gyro_x_rps);
    q3_ += (qa * gyro_z_rps + qb * gyro_y_rps - qc * gyro_x_rps);

    norm = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (norm > 0.0f) {
        q0_ /= norm;
        q1_ /= norm;
        q2_ /= norm;
        q3_ /= norm;
    }

    // Aerospace Z-Y-X Euler decomposition -- the same convention BNO055's
    // own Euler registers use, so this is directly comparable to
    // fc::ImuData::roll_deg/pitch_deg/yaw_deg AS LONG AS the gyro/accel/mag
    // inputs were already in that same body frame (see this class's header
    // doc comment; not this filter's responsibility to guarantee).
    const float roll_rad = atan2f(2.0f * (q0_ * q1_ + q2_ * q3_), 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_));
    float pitch_sin_arg = 2.0f * (q0_ * q2_ - q3_ * q1_);
    pitch_sin_arg = fmaxf(-1.0f, fminf(1.0f, pitch_sin_arg));
    const float pitch_rad = asinf(pitch_sin_arg);
    const float yaw_rad = atan2f(2.0f * (q0_ * q3_ + q1_ * q2_), 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_));

    AttitudeMahonyData next{};
    next.q0 = q0_;
    next.q1 = q1_;
    next.q2 = q2_;
    next.q3 = q3_;
    next.roll_deg = roll_rad * kRadToDeg;
    next.pitch_deg = pitch_rad * kRadToDeg;
    next.yaw_deg = yaw_rad * kRadToDeg;
    next.timestamp_us = micros();
    next.sequence = data_.sequence + 1U;
    next.valid = true;
    data_ = next;
}

AttitudeMahonyData AttitudeMahonyFilter::data() const
{
    return data_;
}

}  // namespace fc
