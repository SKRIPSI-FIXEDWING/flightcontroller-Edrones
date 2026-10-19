#include "drivers/Airspeed.h"

#include <math.h>

namespace {

constexpr uint8_t kCmdReset = 0x1E;
constexpr uint8_t kCmdPromBase = 0xA0;
constexpr uint8_t kCmdConvertD1 = 0x48;  // pressure, OSR 4096 (max 9.04 ms)
constexpr uint8_t kCmdConvertD2 = 0x56;  // temperature, OSR 2048 (max 4.54 ms)
constexpr uint8_t kCmdAdcRead = 0x00;
constexpr uint16_t kConvertD1Ms = 10;
constexpr uint16_t kConvertD2Ms = 5;
constexpr uint32_t kConvertD1Us = 10000;
constexpr uint32_t kConvertD2Us = 5000;

// Q constants for MS5525DSO-pp001DS.
constexpr int kQ2 = 17;
constexpr int kQ1 = 15;
constexpr int kQ3 = 7;
constexpr int kQ4 = 5;
constexpr int kQ5 = 7;
constexpr int kQ6 = 21;

constexpr float kPsiToPa = 6894.757f;
constexpr float kAtmosphericPa = 101325.0f;
constexpr float kAirGasConstant = 287.058f;
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

bool Airspeed::sendCommand(uint8_t command)
{
    wire_.beginTransmission(config_.i2c_address);
    wire_.write(command);
    return wire_.endTransmission() == 0;
}

void Airspeed::recoverBus()
{
    wire_.end();
    delay(2);
    wire_.begin();
    wire_.setClock(config_.i2c_clock_hz);
    delay(2);
}

bool Airspeed::readPromWord(uint8_t index, uint16_t& value)
{
    if (!sendCommand(static_cast<uint8_t>(kCmdPromBase + index * 2U))) {
        return false;
    }
    if (wire_.requestFrom(config_.i2c_address, static_cast<uint8_t>(2U)) != 2U) {
        return false;
    }
    value = static_cast<uint16_t>(wire_.read()) << 8;
    value |= static_cast<uint16_t>(wire_.read());
    return true;
}

// Reads the 24 bit result of a conversion that was started earlier.
bool Airspeed::readAdcResult(uint32_t& raw)
{
    if (!sendCommand(kCmdAdcRead)) {
        return false;
    }
    if (wire_.requestFrom(config_.i2c_address, static_cast<uint8_t>(3U)) != 3U) {
        return false;
    }
    raw = static_cast<uint32_t>(wire_.read()) << 16;
    raw |= static_cast<uint32_t>(wire_.read()) << 8;
    raw |= static_cast<uint32_t>(wire_.read());
    return true;
}

// Blocking convert + wait + read with one retry. begin() only.
bool Airspeed::readAdcBlocking(uint8_t convert_command, uint16_t wait_ms, uint32_t& raw)
{
    for (uint8_t attempt = 0; attempt < 2U; ++attempt) {
        if (sendCommand(convert_command)) {
            delay(wait_ms);
            if (readAdcResult(raw)) {
                if (raw != 0U && raw != 0xFFFFFFU) {
                    return true;
                }
                ++invalid_data_;
            } else {
                ++i2c_errors_;
            }
        } else {
            ++i2c_errors_;
        }
        delay(2);
    }
    recoverBus();
    return false;
}

bool Airspeed::computeTemperature(uint32_t d2)
{
    const double dt = static_cast<double>(d2) - static_cast<double>(prom_[5]) * (1 << kQ5);
    const double temp = 2000.0 + dt * static_cast<double>(prom_[6]) / static_cast<double>(1UL << kQ6);
    const float temperature_c = static_cast<float>(temp / 100.0);
    if (temperature_c < -40.0f || temperature_c > 125.0f) {
        return false;
    }
    dt_last_ = dt;
    temperature_c_ = temperature_c;
    return true;
}

// Differential pressure in Pa, zero offset not yet removed.
bool Airspeed::computePressurePa(uint32_t d1, float& dp_pa)
{
    const double off = static_cast<double>(prom_[2]) * static_cast<double>(1UL << kQ2) +
                       static_cast<double>(prom_[4]) * dt_last_ / static_cast<double>(1UL << kQ4);
    const double sens = static_cast<double>(prom_[1]) * static_cast<double>(1UL << kQ1) +
                        static_cast<double>(prom_[3]) * dt_last_ / static_cast<double>(1UL << kQ3);
    const double p = ((static_cast<double>(d1) * sens / 2097152.0) - off) / 32768.0;  // 0.0001 psi

    const float dp = config_.pressure_sign * static_cast<float>(p * 0.0001 * kPsiToPa);
    if (fabsf(dp) > config_.dp_max_valid_pa) {
        return false;
    }
    dp_pa = dp;
    return true;
}

void Airspeed::calibrateZero()
{
    constexpr uint8_t kMaxSamples = 200;
    float buffer[kMaxSamples];
    const uint8_t requested =
        config_.zero_cal_samples > kMaxSamples ? kMaxSamples : config_.zero_cal_samples;

    delay(500);
    uint8_t n = 0;
    for (uint8_t i = 0; i < requested; ++i) {
        uint32_t d1 = 0;
        float dp = 0.0f;
        if (readAdcBlocking(kCmdConvertD1, kConvertD1Ms, d1) && computePressurePa(d1, dp)) {
            buffer[n++] = dp;
        }
        delay(5);
    }
    if (n < 20U) {
        offset_pa_ = 0.0f;
        return;
    }

    // Insertion sort, then mean of the middle half (immune to outliers).
    for (uint8_t i = 1; i < n; ++i) {
        const float key = buffer[i];
        int j = static_cast<int>(i) - 1;
        while (j >= 0 && buffer[j] > key) {
            buffer[j + 1] = buffer[j];
            --j;
        }
        buffer[j + 1] = key;
    }
    const uint8_t lo = n / 4U;
    const uint8_t hi = n - n / 4U;
    float sum = 0.0f;
    for (uint8_t i = lo; i < hi; ++i) {
        sum += buffer[i];
    }
    offset_pa_ = sum / static_cast<float>(hi - lo);
}

// Reset, PROM coefficients and first temperature reading.
bool Airspeed::probeSensor()
{
    if (!sendCommand(kCmdReset)) {
        return false;
    }
    delay(5);

    for (uint8_t i = 1; i <= 6U; ++i) {
        if (!readPromWord(i, prom_[i]) || prom_[i] == 0U || prom_[i] == 0xFFFFU) {
            return false;
        }
    }

    uint32_t d2 = 0;
    return readAdcBlocking(kCmdConvertD2, kConvertD2Ms, d2) && computeTemperature(d2);
}

bool Airspeed::begin()
{
    initialized_ = false;
    data_ = AirspeedData{};
    consecutive_read_failures_ = 0;
    i2c_errors_ = 0;
    invalid_data_ = 0;
    filter_initialized_ = false;
    dp_filtered_pa_ = 0.0f;
    offset_pa_ = 0.0f;
    pending_ = Pending::None;
    pressure_samples_since_temperature_ = 0;

    wire_.begin();
    wire_.setClock(config_.i2c_clock_hz);

    if (!probeSensor()) {
        last_error_ = AirspeedError::ReadFailed;
        return false;
    }

    calibrateZero();

    initialized_ = true;
    last_error_ = AirspeedError::None;
    return true;
}

void Airspeed::registerFailure(AirspeedError error)
{
    ++consecutive_read_failures_;
    last_error_ = error;
    pending_ = Pending::None;
    if (consecutive_read_failures_ >= config_.bus_recover_after_failures) {
        recoverBus();
    }
}

void Airspeed::publish(float filtered_dp_pa)
{
    float velocity_mps = 0.0f;
    if (filtered_dp_pa >= config_.deadband_pa) {
        const float rho = kAtmosphericPa / (kAirGasConstant * (temperature_c_ + 273.15f));
        velocity_mps = sqrtf(2.0f * filtered_dp_pa / rho);
    }

    AirspeedData next{};
    next.differential_pressure_pa = filtered_dp_pa;
    next.differential_pressure_psi = filtered_dp_pa / kPsiToPa;
    next.velocity_mps = velocity_mps;
    next.velocity_kmh = velocity_mps * kKmhPerMps;
    next.temperature_c = temperature_c_;
    next.timestamp_us = micros();
    next.sequence = data_.sequence + 1U;
    next.valid = true;
    data_ = next;
}

bool Airspeed::update()
{
    if (!initialized_) {
        last_error_ = AirspeedError::NotInitialized;
        return false;
    }

    bool published = false;

    // 1) Collect the conversion started by the previous call.
    if (pending_ != Pending::None) {
        const uint32_t needed_us = (pending_ == Pending::Pressure) ? kConvertD1Us : kConvertD2Us;
        if (static_cast<uint32_t>(micros() - pending_started_us_) < needed_us) {
            return false;  // called too soon; the conversion is still pending
        }

        const Pending finished = pending_;
        pending_ = Pending::None;

        uint32_t raw = 0;
        if (!readAdcResult(raw)) {
            ++i2c_errors_;
            registerFailure(AirspeedError::ReadFailed);
        } else if (raw == 0U || raw == 0xFFFFFFU) {
            ++invalid_data_;
            registerFailure(AirspeedError::InvalidData);
        } else if (finished == Pending::Temperature) {
            // On a bad temperature the previous one is kept.
            if (!computeTemperature(raw)) {
                ++invalid_data_;
            }
        } else {
            float dp = 0.0f;
            if (!computePressurePa(raw, dp)) {
                ++invalid_data_;
                registerFailure(AirspeedError::InvalidData);
            } else {
                dp -= offset_pa_;
                if (!filter_initialized_) {
                    dp_filtered_pa_ = dp;
                    filter_initialized_ = true;
                } else {
                    dp_filtered_pa_ += config_.filter_alpha * (dp - dp_filtered_pa_);
                }
                publish(dp_filtered_pa_);
                consecutive_read_failures_ = 0;
                last_error_ = AirspeedError::None;
                published = true;
            }
        }
    }

    // 2) Start the next conversion (temperature every N pressure samples).
    const bool want_temperature =
        pressure_samples_since_temperature_ >= config_.temperature_every_n_samples;
    if (!sendCommand(want_temperature ? kCmdConvertD2 : kCmdConvertD1)) {
        ++i2c_errors_;
        registerFailure(AirspeedError::ReadFailed);
        return false;
    }
    pending_ = want_temperature ? Pending::Temperature : Pending::Pressure;
    pending_started_us_ = micros();
    pressure_samples_since_temperature_ =
        want_temperature ? 0U : pressure_samples_since_temperature_ + 1U;

    return published;
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
    return initialized_ && last_error_ == AirspeedError::None && consecutive_read_failures_ == 0U;
}

bool Airspeed::isFresh(uint32_t maximum_age_us) const
{
    return data_.valid && static_cast<uint32_t>(micros() - data_.timestamp_us) <= maximum_age_us;
}

AirspeedError Airspeed::lastError() const
{
    return last_error_;
}

uint32_t Airspeed::consecutiveReadFailures() const
{
    return consecutive_read_failures_;
}

uint32_t Airspeed::i2cErrorCount() const
{
    return i2c_errors_;
}

uint32_t Airspeed::invalidDataCount() const
{
    return invalid_data_;
}

float Airspeed::zeroOffsetPa() const
{
    return offset_pa_;
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

}  // namespace fc
