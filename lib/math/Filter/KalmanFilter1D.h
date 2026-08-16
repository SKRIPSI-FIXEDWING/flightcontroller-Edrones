#pragma once

namespace fc {

/**
 * Minimal scalar recursive Kalman filter for smoothing a single noisy
 * measurement (e.g. barometric altitude, airspeed). Assumes the true value
 * is constant between updates plus process noise; each measurement carries
 * independent measurement noise. Ported from KHAGESWARA's legacy
 * `include/Filter.h::KalmanFilter`, namespaced and renamed to avoid clashing
 * with the many other "KalmanFilter" libraries in the Arduino ecosystem.
 */
class KalmanFilter1D {
public:
    void configure(float measurement_noise_r, float process_noise_q,
                   float initial_estimate_error_p);

    /** Snaps the estimate to a known value (e.g. re-zeroing after a ground-pressure
     * recalibration), bypassing the normal gradual convergence through update(). */
    void reset(float initial_estimate);

    float update(float measurement);

    float estimate() const;

private:
    float estimate_ = 0.0f;
    float estimate_error_ = 1.0f;
    float measurement_noise_r_ = 1.0f;
    float process_noise_q_ = 0.0f;
};

}  // namespace fc
