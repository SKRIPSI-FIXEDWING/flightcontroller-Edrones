#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "Filter/DerivativeFilter.h"
#include "Filter/KalmanFilter1D.h"

namespace fc {

/** One coherent MS5611 measurement, plus values derived from it. */
struct BarometerData {
    float pressure_pa = 0.0f;
    float temperature_c = 0.0f;

    /** Height above the pressure recorded at begin(), Kalman-filtered. */
    float altitude_m = 0.0f;

    /** Same height, before the Kalman filter -- for logging/filter-tuning only. */
    float raw_altitude_m = 0.0f;

    /** Positive climbing, from a 7-point derivative filter on altitude_m. */
    float climb_rate_mps = 0.0f;

    /** Equivalent-to-true-airspeed ratio (ISA lapse-rate model). */
    float eas2tas = 1.0f;

    uint32_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

enum class BarometerOversampling : uint8_t {
    UltraLow = 0x00,
    Low = 0x02,
    Standard = 0x04,
    High = 0x06,
    UltraHigh = 0x08,
};

enum class BarometerError : uint8_t {
    None = 0,
    NotInitialized,
    DeviceNotFound,
    ReadFailed,
};

/**
 * MS5611 and MS5607 share the same command set and PROM layout, but the
 * datasheet-mandated compensation formula uses different bit-shift constants
 * for OFF/SENS (MS5607's wider pressure range uses one more/fewer bit of
 * shift). Cheap "GY-63 MS5611" breakout boards are sometimes actually
 * MS5607 -- see docs/barometer-ms5611.md for how this was diagnosed (bench
 * pressure reading was a stable ~2x too low with the MS5611 formula).
 */
enum class BarometerChipVariant : uint8_t {
    Ms5611,
    Ms5607,
};

/** Raw PROM calibration words, exposed for one-time boot diagnostics only. */
struct BarometerPromDump {
    uint16_t c1_sens_t1 = 0;
    uint16_t c2_off_t1 = 0;
    uint16_t c3_tcs = 0;
    uint16_t c4_tco = 0;
    uint16_t c5_tref = 0;
    uint16_t c6_tempsense = 0;
};

struct BarometerConfig {
    uint32_t i2c_clock_hz = 400000;
    BarometerOversampling oversampling = BarometerOversampling::UltraHigh;
    BarometerChipVariant chip_variant = BarometerChipVariant::Ms5607;

    // Ground-pressure reference, sampled once at begin(). Widened from the
    // original 5/15 flush/average samples (2s total) to 15/40 (5.5s total):
    // bench data showed pressure noise std of only ~1 Pa, so more averaging
    // meaningfully tightens the ground reference (~sqrt(40/15) noise
    // reduction) at the cost of a longer one-time boot delay. This does NOT
    // fix the slower (tens-of-seconds) post-power-on thermal drift also seen
    // on the bench -- only re-zeroing after a longer warm-up would address
    // that, which is out of scope here.
    uint8_t ground_pressure_flush_samples = 15;
    uint8_t ground_pressure_average_samples = 40;
    uint16_t ground_pressure_sample_delay_ms = 100;

    // Settle time after chip reset, before the first PROM read.
    uint16_t startup_settle_delay_ms = 100;

    float kalman_measurement_noise_r = 5.0f;
    float kalman_process_noise_q = 0.1f;
    float kalman_initial_estimate_error_p = 1.0f;
};

/**
 * Fixed-wing MS5611 barometer driver.
 *
 * Scheduling stays outside this class, same as fc::Imu: call begin() once,
 * then update() from the flight scheduler and distribute data() to
 * estimation/navigation/telemetry code.
 */
class Barometer final {
public:
    explicit Barometer(TwoWire& wire = Wire1);
    Barometer(TwoWire& wire, const BarometerConfig& config);

    Barometer(const Barometer&) = delete;
    Barometer& operator=(const Barometer&) = delete;

    /** Reset the chip, read PROM calibration, and sample ground pressure. */
    bool begin();

    /**
     * Re-samples the ground-pressure reference at the current position and
     * zeroes the altitude filter to match, WITHOUT resetting the chip or
     * re-reading PROM (those only need to happen once, in begin()). This is
     * the "level/zero the barometer before flight" operation -- exposed over
     * MAVLink as MAV_CMD_PREFLIGHT_CALIBRATION param3=1 (Mission Planner's
     * "Calibrate Baro" / HUD right-click), see Mavlink.cpp. Call this while
     * stationary on the ground, disarmed, immediately before flight -- it
     * corrects the slow ambient-pressure drift documented in
     * docs/barometer-ms5611.md, which begin()'s one-time boot calibration
     * cannot.
     */
    bool recalibrateGroundPressure();

    /** Read, compensate, filter, and publish one complete sample. */
    bool update();

    BarometerData data() const;

    bool initialized() const;
    bool healthy() const;
    bool isFresh(uint32_t maximum_age_us) const;

    BarometerError lastError() const;
    uint32_t consecutiveReadFailures() const;

    /** Raw PROM calibration words, for one-time boot diagnostics (see main.cpp setup()). */
    BarometerPromDump promDump() const;

    // Migration helpers matching the values exposed by legacy Barometer.h.
    float altitudeMeters() const;
    float climbRateMetersPerSecond() const;
    float equivalentToTrueAirspeedRatio() const;
    float pressurePascals() const;
    float temperatureCelsius() const;

private:
    bool writeCommand(uint8_t command);
    bool readWordRegister(uint8_t command, uint16_t& out);
    bool readAdcResult(uint32_t& out);

    bool readProm();
    bool readRawPressureCounts(uint32_t& counts);
    bool readRawTemperatureCounts(uint32_t& counts);

    void compensate(uint32_t raw_pressure_counts, uint32_t raw_temperature_counts,
                     bool apply_second_order_correction, double& pressure_pa,
                     double& temperature_c) const;

    bool determineGroundPressure();
    float updateEas2Tas(float altitude_m, float temperature_c, float pressure_pa);

    static float computeAltitudeMeters(double pressure_pa, double ground_pressure_pa);
    static uint8_t conversionDelayMs(BarometerOversampling oversampling);

    void setError(BarometerError error);

    TwoWire& wire_;
    BarometerConfig config_{};

    // Calibration coefficients C1..C6, read from PROM addresses 0xA2..0xAC.
    uint16_t prom_[6] = {};
    uint8_t conversion_delay_ms_ = 10;
    double ground_pressure_pa_ = 0.0;

    DerivativeFilterFloat_Size7 climb_rate_filter_{};
    KalmanFilter1D altitude_kalman_{};

    float cached_eas2tas_ = 0.0f;
    float last_eas2tas_altitude_m_ = 0.0f;

    BarometerData data_{};
    BarometerError last_error_ = BarometerError::NotInitialized;
    uint32_t consecutive_read_failures_ = 0;
    bool initialized_ = false;
};

}  // namespace fc
