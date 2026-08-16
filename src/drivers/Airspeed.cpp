#include "drivers/Airspeed.h"

#include <math.h>

namespace {

// Pa per PSI, doubled: matches Bernoulli's v = sqrt(2 * dP_pa / rho) with
// dP_pa = differential_pressure_psi * 6894.7572f.
constexpr float kPsiToPaTimesTwo = 13789.5144f;
constexpr float kKmhPerMps = 3.6f;

}  // namespace

namespace fc {

Airspeed::Airspeed(TwoWire& wire)
    : wire_(wire)
{
}

Airspeed::Airspeed(TwoWire& wire, const AirspeedConfig& config)
    : wire_(wire), config_(config)
{
}

bool Airspeed::begin()
{
    initialized_ = false;
    data_ = AirspeedData{};
    consecutive_read_failures_ = 0;

    span_counts_ = config_.full_scale_counts - config_.min_scale_counts;
    zero_counts_ = (config_.min_scale_counts - config_.full_scale_counts) / 2;

    velocity_kalman_.configure(config_.kalman_measurement_noise_r,
                               config_.kalman_process_noise_q,
                               config_.kalman_initial_estimate_error_p);

    wire_.begin();

    // Single-sample zero-pressure offset, matching legacy initAirspeed().
    // Take this reading with the pitot tube disconnected/stationary.
    uint16_t pressure_counts = 0;
    uint16_t temperature_counts = 0;
    uint8_t status = 0;
    if (!readRawCounts(pressure_counts, temperature_counts, status)) {
        setError(AirspeedError::ReadFailed);
        return false;
    }
    offset_psi_ = fabsf(countsToDifferentialPressurePsi(pressure_counts));

    initialized_ = true;
    setError(AirspeedError::None);
    return true;
}

bool Airspeed::update()
{
    if (!initialized_) {
        setError(AirspeedError::NotInitialized);
        return false;
    }

    uint16_t pressure_counts = 0;
    uint16_t temperature_counts = 0;
    uint8_t status = 0;
    if (!readRawCounts(pressure_counts, temperature_counts, status)) {
        ++consecutive_read_failures_;
        setError(AirspeedError::ReadFailed);
        return false;
    }

    const float differential_pressure_psi =
        fabsf(countsToDifferentialPressurePsi(pressure_counts) - offset_psi_);
    const float raw_velocity_mps =
        sqrtf((differential_pressure_psi * kPsiToPaTimesTwo) / config_.air_density_kg_per_m3);

    AirspeedData next{};
    next.differential_pressure_psi = differential_pressure_psi;
    next.differential_pressure_pa = differential_pressure_psi * kPsiToPaTimesTwo;
    next.velocity_mps = velocity_kalman_.update(raw_velocity_mps);
    next.velocity_kmh = next.velocity_mps * kKmhPerMps;
    next.timestamp_us = micros();
    next.sequence = data_.sequence + 1U;
    next.valid = true;

    data_ = next;
    consecutive_read_failures_ = 0;
    setError(AirspeedError::None);
    return true;
}

bool Airspeed::readRawCounts(uint16_t& pressure_counts, uint16_t& temperature_counts,
                             uint8_t& status)
{
    // MS4525DO is a "measurement command" device: any read request returns
    // its latest continuously-converted sample, no register address or
    // conversion-trigger command is needed.
    if (wire_.requestFrom(kDeviceAddress, static_cast<uint8_t>(4U)) != 4U) {
        return false;
    }

    const uint8_t pressure_high = static_cast<uint8_t>(wire_.read());
    const uint8_t pressure_low = static_cast<uint8_t>(wire_.read());
    const uint8_t temperature_high = static_cast<uint8_t>(wire_.read());
    const uint8_t temperature_low = static_cast<uint8_t>(wire_.read());

    status = (pressure_high >> 6) & 0x03U;
    const uint8_t pressure_high_masked = pressure_high & 0x3FU;
    pressure_counts = static_cast<uint16_t>((static_cast<uint16_t>(pressure_high_masked) << 8) |
                                             pressure_low);
    temperature_counts = static_cast<uint16_t>((static_cast<uint16_t>(temperature_high) << 3) |
                                                (temperature_low >> 5));
    return true;
}

float Airspeed::countsToDifferentialPressurePsi(uint16_t pressure_counts) const
{
    return (static_cast<float>(static_cast<int16_t>(pressure_counts) - zero_counts_) /
            static_cast<float>(span_counts_)) *
           static_cast<float>(config_.full_scale_range_psi);
}

AirspeedData Airspeed::data() const
{
    return data_;
}

bool Airspeed::initialized() const
{
    return initialized_;
}

bool Airspeed::healthy() const
{
    return initialized_ &&
           last_error_ == AirspeedError::None &&
           consecutive_read_failures_ == 0U;
}

bool Airspeed::isFresh(uint32_t maximum_age_us) const
{
    return data_.valid &&
           static_cast<uint32_t>(micros() - data_.timestamp_us) <= maximum_age_us;
}

AirspeedError Airspeed::lastError() const
{
    return last_error_;
}

uint32_t Airspeed::consecutiveReadFailures() const
{
    return consecutive_read_failures_;
}

float Airspeed::velocityMetersPerSecond() const
{
    return data_.velocity_mps;
}

float Airspeed::velocityKilometersPerHour() const
{
    return data_.velocity_kmh;
}

float Airspeed::differentialPressurePsi() const
{
    return data_.differential_pressure_psi;
}

void Airspeed::setError(AirspeedError error)
{
    last_error_ = error;
}

}  // namespace fc
