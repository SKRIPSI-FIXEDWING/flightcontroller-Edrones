#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace fc {

/** One coherent MS5525DSO differential-pressure / airspeed measurement. */
struct AirspeedData {
    /** Filtered differential pressure, offset-corrected (signed). */
    float differential_pressure_pa = 0.0f;
    float differential_pressure_psi = 0.0f;

    /** Indicated airspeed from the filtered differential pressure. */
    float velocity_mps = 0.0f;
    float velocity_kmh = 0.0f;

    /** Sensor die temperature, used for the air density estimate. */
    float temperature_c = 25.0f;

    uint32_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

enum class AirspeedError : uint8_t {
    None = 0,
    NotInitialized,
    ReadFailed,
    InvalidData,
};

struct AirspeedConfig {
    uint8_t i2c_address = 0x76;
    uint32_t i2c_clock_hz = 100000;

    // Set to -1 if dP reads negative while air flows into the pitot.
    float pressure_sign = 1.0f;

    // Samples outside +-this are treated as corrupt and dropped.
    float dp_max_valid_pa = 7500.0f;
    // Filtered dP below this is reported as 0 m/s.
    float deadband_pa = 2.0f;
    // First-order low-pass on dP (per valid sample).
    float filter_alpha = 0.2f;

    // Temperature (D2) is re-read once every this many pressure samples.
    uint16_t temperature_every_n_samples = 40;

    // Zero-offset calibration done in begin() (pitot must see no airflow).
    uint8_t zero_cal_samples = 100;

    // Consecutive failures before the I2C bus is re-initialised.
    uint8_t bus_recover_after_failures = 2;
};

/**
 * Fixed-wing MS5525DSO (001DS, +-1 psi, I2C 0x76) airspeed driver.
 *
 * Scheduling stays outside this class, same as fc::Imu/fc::Barometer: call
 * begin() once (blocking, ~2 s because of the zero-offset calibration), then
 * update() periodically (>= 15 ms apart). update() never blocks: each call
 * reads the conversion started by the previous call and starts the next one.
 */
class Airspeed final {
public:
    explicit Airspeed(TwoWire& wire = Wire2);
    Airspeed(TwoWire& wire, const AirspeedConfig& config);

    Airspeed(const Airspeed&) = delete;
    Airspeed& operator=(const Airspeed&) = delete;

    /** Reset sensor, read PROM, read temperature, calibrate zero offset. */
    bool begin();

    /** Read one conversion result and start the next. */
    bool update();

    AirspeedData data() const;

    bool initialized() const;
    bool healthy() const;
    bool isFresh(uint32_t maximum_age_us) const;

    AirspeedError lastError() const;
    uint32_t consecutiveReadFailures() const;
    uint32_t i2cErrorCount() const;
    uint32_t invalidDataCount() const;
    float zeroOffsetPa() const;

    float velocityMetersPerSecond() const;
    float velocityKilometersPerHour() const;
    float differentialPressurePsi() const;

private:
    enum class Pending : uint8_t { None, Pressure, Temperature };

    bool sendCommand(uint8_t command);
    bool readPromWord(uint8_t index, uint16_t& value);
    bool readAdcResult(uint32_t& raw);
    bool readAdcBlocking(uint8_t convert_command, uint16_t wait_ms, uint32_t& raw);
    void recoverBus();

    bool probeSensor();
    bool computeTemperature(uint32_t d2);
    bool computePressurePa(uint32_t d1, float& dp_pa);
    void calibrateZero();
    void registerFailure(AirspeedError error);
    void publish(float filtered_dp_pa);

    TwoWire& wire_;
    AirspeedConfig config_{};

    uint16_t prom_[7] = {};
    double dt_last_ = 0.0;
    float temperature_c_ = 25.0f;

    float offset_pa_ = 0.0f;
    float dp_filtered_pa_ = 0.0f;
    bool filter_initialized_ = false;

    Pending pending_ = Pending::None;
    uint32_t pending_started_us_ = 0;
    uint32_t pressure_samples_since_temperature_ = 0;

    AirspeedData data_{};
    AirspeedError last_error_ = AirspeedError::NotInitialized;
    uint32_t consecutive_read_failures_ = 0;
    uint32_t i2c_errors_ = 0;
    uint32_t invalid_data_ = 0;
    bool initialized_ = false;
};

}  // namespace fc
