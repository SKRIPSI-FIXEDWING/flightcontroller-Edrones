#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace fc {

struct BatteryData {
    float voltage_v = 0.0f;
    int32_t percent = 0;
    uint32_t timestamp_us = 0;
    bool valid = false;
};

struct BatteryConfig {
    uint8_t analog_pin = A13;
    uint32_t sample_interval_ms = 50;

    // ADC-counts -> millivolts linear fit (voltage-divider specific to the
    // board), then an empirical offset, matching legacy Voltage.h exactly.
    float adc_max_millivolts = 16800.0f;
    float voltage_offset_v = -0.4f;

    // Percent curve: voltage at 0% and 100%.
    float percent_curve_min_v = 14.4f;
    float percent_curve_max_v = 16.4f;
};

/**
 * Battery voltage monitor (legacy Voltage.h's actual implementation, which
 * despite the filename lived in controlinfo.h in the legacy tree -- see
 * docs/battery.md). 10-sample moving average on an analog pin.
 *
 * This replaces the fake oscillating sine-wave battery value that
 * Mavlink.h's mavlinkTask() sent for BATTERY_STATUS (a clearly-labeled test
 * stub -- legacy comment: "Osilasi naik-turun 10.5V-12.6V mengikuti waktu"
 * -- never wired to this real sensor). See docs/mavlink.md.
 */
class Battery final {
public:
    explicit Battery(const BatteryConfig& config = BatteryConfig{});

    void begin();

    /** Internally rate-limited to config_.sample_interval_ms; safe to call every loop. */
    void update();

    BatteryData data() const;

private:
    static constexpr uint8_t kMovingAverageSamples = 10;

    BatteryConfig config_{};
    int readings_[kMovingAverageSamples] = {};
    uint8_t read_index_ = 0;
    int32_t running_total_ = 0;
    uint32_t last_sample_ms_ = 0;

    BatteryData data_{};
};

}  // namespace fc
