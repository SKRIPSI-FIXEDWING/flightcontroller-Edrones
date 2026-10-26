#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace fc {

enum class BatteryState : uint8_t { Ok, Low, Critical, AdcSaturated };

struct BatteryData {
    float voltage_v = 0.0f;
    float cell_voltage_v = 0.0f;
    float adc_raw = 0.0f;
    int32_t percent = 0;
    BatteryState state = BatteryState::Ok;
    uint32_t timestamp_us = 0;
    bool valid = false;
};

struct BatteryConfig {
    // Modul voltage sensor 0-25V: S -> pin 26 (A12), - -> GND, + -> 5V.
    uint8_t analog_pin = A12;
    uint32_t sample_interval_ms = 50;

    float adc_vref = 3.3f;     // ADC Teensy 4.1 0 - 3.3 V (pin TIDAK toleran 5 V)
    uint8_t adc_bits = 12;     // 0 - 4095
    float adc_max = 4095.0f;

    float divider_ratio = 5.0f;   // rasio modul sensor (Vin = Vout x 5)
    // Kalibrasi: tegangan_multimeter / tegangan_terbaca
    // (awal: battery checker 15.93 V vs terbaca 16.00 V -> 0.9956).
    float cal_factor = 0.9956f;

    uint8_t num_samples = 32;  // sampel per pembacaan (rata-rata)
    float ema_alpha = 0.20f;   // penghalus (0..1), makin kecil makin halus

    uint8_t cells = 4;             // baterai 4S
    float cell_warn_v = 3.60f;     // peringatan baterai rendah
    float cell_critical_v = 3.40f; // kritis
};

/**
 * Battery voltage monitor for the 4S LiPo (16.8 V full) on Teensy 4.1 pin 26
 * (A12), ported from the standalone voltage_sensor_teensy41.ino sketch:
 * 32-sample ADC average + EMA filter, percent from a resting-voltage LiPo
 * curve. See docs/battery.md.
 *
 * Note: with a 5:1 divider the ADC saturates at 3.3 V x 5 = 16.5 V, so a
 * freshly charged pack (16.8 V) reads clipped until it sags below 16.5 V;
 * state() reports AdcSaturated in that case.
 */
class Battery final {
public:
    explicit Battery(const BatteryConfig& config = BatteryConfig{});

    void begin();

    /** Internally rate-limited to config_.sample_interval_ms; safe to call every loop. */
    void update();

    BatteryData data() const;

private:
    float readAdcAverage() const;
    float percentFromCellVoltage(float cell_v) const;

    BatteryConfig config_{};
    float filtered_voltage_v_ = 0.0f;
    bool first_reading_ = true;
    uint32_t last_sample_ms_ = 0;

    BatteryData data_{};
};

}  // namespace fc
