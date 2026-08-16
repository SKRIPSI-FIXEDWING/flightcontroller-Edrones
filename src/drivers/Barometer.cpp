#include "drivers/Barometer.h"

#include <math.h>

#include "definitions.h"

namespace {

constexpr uint8_t kAddress = 0x77;
constexpr uint8_t kCmdReset = 0x1E;
constexpr uint8_t kCmdConvertD1 = 0x40;  // Pressure
constexpr uint8_t kCmdConvertD2 = 0x50;  // Temperature
constexpr uint8_t kCmdReadAdc = 0x00;
constexpr uint8_t kCmdReadPromBase = 0xA2;  // C1 at 0xA2, C6 at 0xAC

// EAS2TAS is only recalculated once altitude has moved this many meters.
constexpr float kEas2TasRecalcThresholdM = 25.0f;

}  // namespace

namespace fc {

Barometer::Barometer(TwoWire& wire)
    : wire_(wire)
{
}

Barometer::Barometer(TwoWire& wire, const BarometerConfig& config)
    : wire_(wire), config_(config)
{
}

bool Barometer::begin()
{
    initialized_ = false;
    data_ = BarometerData{};
    consecutive_read_failures_ = 0;
    cached_eas2tas_ = 0.0f;
    last_eas2tas_altitude_m_ = 0.0f;

    altitude_kalman_.configure(config_.kalman_measurement_noise_r,
                               config_.kalman_process_noise_q,
                               config_.kalman_initial_estimate_error_p);
    climb_rate_filter_.reset();

    wire_.begin();
    wire_.setClock(config_.i2c_clock_hz);

    conversion_delay_ms_ = conversionDelayMs(config_.oversampling);

    if (!writeCommand(kCmdReset)) {
        setError(BarometerError::DeviceNotFound);
        return false;
    }
    delay(config_.startup_settle_delay_ms);

    if (!readProm()) {
        setError(BarometerError::ReadFailed);
        return false;
    }

    if (!determineGroundPressure()) {
        setError(BarometerError::ReadFailed);
        return false;
    }

    initialized_ = true;
    setError(BarometerError::None);
    return true;
}

bool Barometer::recalibrateGroundPressure()
{
    if (!initialized_) {
        setError(BarometerError::NotInitialized);
        return false;
    }

    if (!determineGroundPressure()) {
        setError(BarometerError::ReadFailed);
        return false;
    }

    // Re-zero the filters against the new reference so the stale pre-recalibration
    // estimate doesn't linger and slowly converge -- see KalmanFilter1D::reset().
    altitude_kalman_.configure(config_.kalman_measurement_noise_r,
                               config_.kalman_process_noise_q,
                               config_.kalman_initial_estimate_error_p);
    altitude_kalman_.reset(0.0f);
    climb_rate_filter_.reset();
    cached_eas2tas_ = 0.0f;
    last_eas2tas_altitude_m_ = 0.0f;

    setError(BarometerError::None);
    return true;
}

bool Barometer::update()
{
    if (!initialized_) {
        setError(BarometerError::NotInitialized);
        return false;
    }

    uint32_t pressure_counts = 0;
    uint32_t temperature_counts = 0;
    if (!readRawPressureCounts(pressure_counts) ||
        !readRawTemperatureCounts(temperature_counts)) {
        ++consecutive_read_failures_;
        setError(BarometerError::ReadFailed);
        return false;
    }

    double pressure_pa = 0.0;
    double temperature_c = 0.0;
    compensate(pressure_counts, temperature_counts, /*apply_second_order_correction=*/true,
               pressure_pa, temperature_c);

    const float raw_altitude_m = computeAltitudeMeters(pressure_pa, ground_pressure_pa_);
    float altitude_m = altitude_kalman_.update(raw_altitude_m);
    if (isnan(altitude_m) || isinf(altitude_m)) {
        altitude_m = 0.0f;
    }

    climb_rate_filter_.update(altitude_m, millis());

    BarometerData next{};
    next.pressure_pa = static_cast<float>(pressure_pa);
    next.temperature_c = static_cast<float>(temperature_c);
    next.altitude_m = altitude_m;
    next.raw_altitude_m = raw_altitude_m;
    next.climb_rate_mps = climb_rate_filter_.slope() * 1.0e3f;
    next.eas2tas = updateEas2Tas(altitude_m, next.temperature_c, next.pressure_pa);
    next.timestamp_us = micros();
    next.sequence = data_.sequence + 1U;
    next.valid = true;

    data_ = next;
    consecutive_read_failures_ = 0;
    setError(BarometerError::None);
    return true;
}

bool Barometer::readProm()
{
    for (uint8_t index = 0; index < 6U; ++index) {
        const uint8_t command = kCmdReadPromBase + static_cast<uint8_t>(index * 2U);
        if (!readWordRegister(command, prom_[index])) {
            return false;
        }
    }
    return true;
}

bool Barometer::determineGroundPressure()
{
    uint32_t pressure_counts = 0;
    uint32_t temperature_counts = 0;

    // Flush a few readings to clear any stale data from the sensor.
    for (uint8_t i = 0; i < config_.ground_pressure_flush_samples; ++i) {
        if (!readRawPressureCounts(pressure_counts) ||
            !readRawTemperatureCounts(temperature_counts)) {
            return false;
        }
        delay(config_.ground_pressure_sample_delay_ms);
    }

    double sum_pressure_pa = 0.0;
    for (uint8_t i = 0; i < config_.ground_pressure_average_samples; ++i) {
        if (!readRawPressureCounts(pressure_counts) ||
            !readRawTemperatureCounts(temperature_counts)) {
            return false;
        }
        double pressure_pa = 0.0;
        double temperature_c = 0.0;
        compensate(pressure_counts, temperature_counts,
                   /*apply_second_order_correction=*/false, pressure_pa, temperature_c);
        sum_pressure_pa += pressure_pa;
        delay(config_.ground_pressure_sample_delay_ms);
    }

    ground_pressure_pa_ = sum_pressure_pa / config_.ground_pressure_average_samples;
    return true;
}

void Barometer::compensate(uint32_t raw_pressure_counts, uint32_t raw_temperature_counts,
                            bool apply_second_order_correction, double& pressure_pa,
                            double& temperature_c) const
{
    const int32_t dT = static_cast<int32_t>(raw_temperature_counts) -
                        static_cast<int32_t>(prom_[4]) * 256;

    // OFF/SENS bit-shift constants differ between MS5611 and MS5607 (wider
    // pressure range on the MS5607 costs it one bit of shift on each term).
    // See BarometerChipVariant's doc comment for why this is configurable.
    int64_t off;
    int64_t sens;
    if (config_.chip_variant == BarometerChipVariant::Ms5607) {
        off = static_cast<int64_t>(prom_[1]) * 131072 +
              (static_cast<int64_t>(prom_[3]) * dT) / 64;
        sens = static_cast<int64_t>(prom_[0]) * 65536 +
               (static_cast<int64_t>(prom_[2]) * dT) / 128;
    } else {
        off = static_cast<int64_t>(prom_[1]) * 65536 +
              (static_cast<int64_t>(prom_[3]) * dT) / 128;
        sens = static_cast<int64_t>(prom_[0]) * 32768 +
               (static_cast<int64_t>(prom_[2]) * dT) / 256;
    }
    int32_t temp = 2000 + static_cast<int32_t>((static_cast<int64_t>(dT) * prom_[5]) / 8388608);

    if (apply_second_order_correction && temp < 2000) {
        const int32_t temp2 = static_cast<int32_t>((static_cast<int64_t>(dT) * dT) / 2147483648LL);
        int64_t off2;
        int64_t sens2;
        if (config_.chip_variant == BarometerChipVariant::Ms5607) {
            off2 = 61 * (static_cast<int64_t>(temp) - 2000) * (temp - 2000) / 16;
            sens2 = 2 * (static_cast<int64_t>(temp) - 2000) * (temp - 2000);
            if (temp < -1500) {
                off2 += 15 * (static_cast<int64_t>(temp) + 1500) * (temp + 1500);
                sens2 += 8 * (static_cast<int64_t>(temp) + 1500) * (temp + 1500);
            }
        } else {
            off2 = 5 * (static_cast<int64_t>(temp) - 2000) * (temp - 2000) / 2;
            sens2 = 5 * (static_cast<int64_t>(temp) - 2000) * (temp - 2000) / 4;
            if (temp < -1500) {
                off2 += 7 * (static_cast<int64_t>(temp) + 1500) * (temp + 1500);
                sens2 += 11 * (static_cast<int64_t>(temp) + 1500) * (temp + 1500) / 2;
            }
        }

        off -= off2;
        sens -= sens2;
        temp -= temp2;
    }

    pressure_pa = static_cast<double>(
        (static_cast<int64_t>(raw_pressure_counts) * sens) / 2097152 - off) / 32768.0;
    temperature_c = static_cast<double>(temp) / 100.0;
}

float Barometer::updateEas2Tas(float altitude_m, float temperature_c, float pressure_pa)
{
    if (fabsf(altitude_m - last_eas2tas_altitude_m_) < kEas2TasRecalcThresholdM &&
        cached_eas2tas_ != 0.0f) {
        return cached_eas2tas_;
    }

    const float temp_k = temperature_c + C_TO_KELVIN - ISA_LAPSE_RATE * altitude_m;
    const float eas2tas_squared = SSL_AIR_DENSITY / (pressure_pa / (ISA_GAS_CONSTANT * temp_k));
    if (eas2tas_squared < 0.0f) {
        return 1.0f;
    }

    cached_eas2tas_ = sqrtf(eas2tas_squared);
    last_eas2tas_altitude_m_ = altitude_m;
    return cached_eas2tas_;
}

float Barometer::computeAltitudeMeters(double pressure_pa, double ground_pressure_pa)
{
    return static_cast<float>(
        44330.0 * (1.0 - pow(pressure_pa / ground_pressure_pa, 0.1902949)));
}

uint8_t Barometer::conversionDelayMs(BarometerOversampling oversampling)
{
    switch (oversampling) {
        case BarometerOversampling::UltraLow:
            return 1;
        case BarometerOversampling::Low:
            return 2;
        case BarometerOversampling::Standard:
            return 3;
        case BarometerOversampling::High:
            return 5;
        case BarometerOversampling::UltraHigh:
        default:
            return 10;
    }
}

bool Barometer::readRawPressureCounts(uint32_t& counts)
{
    if (!writeCommand(kCmdConvertD1 + static_cast<uint8_t>(config_.oversampling))) {
        return false;
    }
    delay(conversion_delay_ms_);
    return readAdcResult(counts);
}

bool Barometer::readRawTemperatureCounts(uint32_t& counts)
{
    if (!writeCommand(kCmdConvertD2 + static_cast<uint8_t>(config_.oversampling))) {
        return false;
    }
    delay(conversion_delay_ms_);
    return readAdcResult(counts);
}

bool Barometer::writeCommand(uint8_t command)
{
    wire_.beginTransmission(kAddress);
    wire_.write(command);
    return wire_.endTransmission() == 0U;
}

bool Barometer::readWordRegister(uint8_t command, uint16_t& out)
{
    wire_.beginTransmission(kAddress);
    wire_.write(command);
    if (wire_.endTransmission(false) != 0U) {
        return false;
    }

    if (wire_.requestFrom(kAddress, static_cast<uint8_t>(2U)) != 2U) {
        return false;
    }
    const uint8_t high_byte = static_cast<uint8_t>(wire_.read());
    const uint8_t low_byte = static_cast<uint8_t>(wire_.read());
    out = static_cast<uint16_t>((high_byte << 8) | low_byte);
    return true;
}

bool Barometer::readAdcResult(uint32_t& out)
{
    wire_.beginTransmission(kAddress);
    wire_.write(kCmdReadAdc);
    if (wire_.endTransmission(false) != 0U) {
        return false;
    }

    if (wire_.requestFrom(kAddress, static_cast<uint8_t>(3U)) != 3U) {
        return false;
    }
    const uint8_t byte2 = static_cast<uint8_t>(wire_.read());
    const uint8_t byte1 = static_cast<uint8_t>(wire_.read());
    const uint8_t byte0 = static_cast<uint8_t>(wire_.read());
    out = (static_cast<uint32_t>(byte2) << 16) | (static_cast<uint32_t>(byte1) << 8) | byte0;
    return true;
}

BarometerData Barometer::data() const
{
    return data_;
}

bool Barometer::initialized() const
{
    return initialized_;
}

bool Barometer::healthy() const
{
    return initialized_ &&
           last_error_ == BarometerError::None &&
           consecutive_read_failures_ == 0U;
}

bool Barometer::isFresh(uint32_t maximum_age_us) const
{
    return data_.valid &&
           static_cast<uint32_t>(micros() - data_.timestamp_us) <= maximum_age_us;
}

BarometerError Barometer::lastError() const
{
    return last_error_;
}

uint32_t Barometer::consecutiveReadFailures() const
{
    return consecutive_read_failures_;
}

BarometerPromDump Barometer::promDump() const
{
    BarometerPromDump dump{};
    dump.c1_sens_t1 = prom_[0];
    dump.c2_off_t1 = prom_[1];
    dump.c3_tcs = prom_[2];
    dump.c4_tco = prom_[3];
    dump.c5_tref = prom_[4];
    dump.c6_tempsense = prom_[5];
    return dump;
}

float Barometer::altitudeMeters() const
{
    return data_.altitude_m;
}

float Barometer::climbRateMetersPerSecond() const
{
    return data_.climb_rate_mps;
}

float Barometer::equivalentToTrueAirspeedRatio() const
{
    return data_.eas2tas;
}

float Barometer::pressurePascals() const
{
    return data_.pressure_pa;
}

float Barometer::temperatureCelsius() const
{
    return data_.temperature_c;
}

void Barometer::setError(BarometerError error)
{
    last_error_ = error;
}

}  // namespace fc
