#include "KalmanFilter1D.h"

namespace fc {

void KalmanFilter1D::configure(float measurement_noise_r, float process_noise_q,
                                float initial_estimate_error_p)
{
    measurement_noise_r_ = measurement_noise_r;
    process_noise_q_ = process_noise_q;
    estimate_error_ = initial_estimate_error_p;
}

void KalmanFilter1D::reset(float initial_estimate)
{
    estimate_ = initial_estimate;
}

float KalmanFilter1D::update(float measurement)
{
    // Predict.
    const float predicted_estimate = estimate_;
    const float predicted_error = estimate_error_ + process_noise_q_;

    // Update.
    const float gain = predicted_error / (predicted_error + measurement_noise_r_);
    estimate_ = predicted_estimate + gain * (measurement - predicted_estimate);
    estimate_error_ = (1.0f - gain) * predicted_error;

    return estimate_;
}

float KalmanFilter1D::estimate() const
{
    return estimate_;
}

}  // namespace fc
