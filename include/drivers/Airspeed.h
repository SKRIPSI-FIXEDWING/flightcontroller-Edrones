#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "Filter/KalmanFilter1D.h"

namespace fc {

/** One coherent MS4525DO differential-pressure / airspeed measurement. */
struct AirspeedData {
    float differential_pressure_psi = 0.0f;
    float differential_pressure_pa = 0.0f;

    /** Kalman-filtered indicated airspeed. */
    float velocity_mps = 0.0f;
    float velocity_kmh = 0.0f;

    uint32_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

enum class AirspeedError : uint8_t {
    None = 0,
    NotInitialized,
    ReadFailed,
};

struct AirspeedConfig {
    // MS4525DO transfer-function constants (datasheet "A" type, 1 PSI span).
    int16_t full_scale_range_psi = 1;
    int16_t min_scale_counts = 1617;
    int16_t full_scale_counts = 14569;

    float air_density_kg_per_m3 = 1.225f;

    float kalman_measurement_noise_r = 4.0f;
    float kalman_process_noise_q = 0.3f;
    float kalman_initial_estimate_error_p = 0.1f;
};

/**
 * Fixed-wing MS4525DO airspeed driver.
 *
 * Scheduling stays outside this class, same as fc::Imu/fc::Barometer: call
 * begin() once, then update() from the flight scheduler and distribute
 * data() to estimation/navigation/TECS code.
 */
class Airspeed final {
public:
    explicit Airspeed(TwoWire& wire = Wire2);
    Airspeed(TwoWire& wire, const AirspeedConfig& config);

    Airspeed(const Airspeed&) = delete;
    Airspeed& operator=(const Airspeed&) = delete;

    /** Start the bus and take a single zero-pressure offset sample. */
    bool begin();

    /** Read and publish one complete sample. */
    bool update();

    AirspeedData data() const;

    bool initialized() const;
    bool healthy() const;
    bool isFresh(uint32_t maximum_age_us) const;

    AirspeedError lastError() const;
    uint32_t consecutiveReadFailures() const;

    // Migration helpers matching the values exposed by legacy airspeed.h.
    float velocityMetersPerSecond() const;
    float velocityKilometersPerHour() const;
    float differentialPressurePsi() const;

private:
    bool readRawCounts(uint16_t& pressure_counts, uint16_t& temperature_counts,
                        uint8_t& status);
    float countsToDifferentialPressurePsi(uint16_t pressure_counts) const;
    void setError(AirspeedError error);

    static constexpr uint8_t kDeviceAddress = 0x28;

    TwoWire& wire_;
    AirspeedConfig config_{};
    int16_t span_counts_ = 0;
    int16_t zero_counts_ = 0;

    float offset_psi_ = 0.0f;
    KalmanFilter1D velocity_kalman_{};

    AirspeedData data_{};
    AirspeedError last_error_ = AirspeedError::NotInitialized;
    uint32_t consecutive_read_failures_ = 0;
    bool initialized_ = false;
};

}  // namespace fc
