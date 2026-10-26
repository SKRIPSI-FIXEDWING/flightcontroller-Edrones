#include "vehicle/Battery.h"

namespace fc {

namespace {

// Kurva LiPo (tegangan per sel saat diam -> persen), urut dari tinggi ke rendah.
constexpr int kCurvePoints = 21;
constexpr float kCurveV[kCurvePoints] = {
    4.20f, 4.15f, 4.11f, 4.08f, 4.02f, 3.98f, 3.95f, 3.91f, 3.87f, 3.85f, 3.84f,
    3.82f, 3.80f, 3.79f, 3.77f, 3.75f, 3.73f, 3.71f, 3.69f, 3.61f, 3.27f,
};
constexpr float kCurvePct[kCurvePoints] = {
    100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50,
    45,  40, 35, 30, 25, 20, 15, 10, 5,  0,
};

}  // namespace

Battery::Battery(const BatteryConfig& config)
    : config_(config)
{
}

void Battery::begin()
{
    // Matikan keeper internal pin agar tidak mengganggu pembacaan analog.
    pinMode(config_.analog_pin, INPUT_DISABLE);
    analogReadResolution(config_.adc_bits);
    analogReadAveraging(4);  // averaging bawaan hardware

    filtered_voltage_v_ = 0.0f;
    first_reading_ = true;
}

float Battery::readAdcAverage() const
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < config_.num_samples; ++i) {
        sum += analogRead(config_.analog_pin);
    }
    return static_cast<float>(sum) / config_.num_samples;
}

float Battery::percentFromCellVoltage(float cell_v) const
{
    if (cell_v >= kCurveV[0]) {
        return 100.0f;
    }
    if (cell_v <= kCurveV[kCurvePoints - 1]) {
        return 0.0f;
    }
    for (int i = 0; i < kCurvePoints - 1; ++i) {
        if (cell_v <= kCurveV[i] && cell_v > kCurveV[i + 1]) {
            const float span = kCurveV[i] - kCurveV[i + 1];
            const float frac = (cell_v - kCurveV[i + 1]) / span;
            return kCurvePct[i + 1] + frac * (kCurvePct[i] - kCurvePct[i + 1]);
        }
    }
    return 0.0f;
}

void Battery::update()
{
    const uint32_t now_ms = millis();
    if (now_ms - last_sample_ms_ < config_.sample_interval_ms) {
        return;
    }
    last_sample_ms_ = now_ms;

    const float adc = readAdcAverage();
    const float v_pin = (adc / config_.adc_max) * config_.adc_vref;
    const float v_batt = v_pin * config_.divider_ratio * config_.cal_factor;

    if (first_reading_) {
        filtered_voltage_v_ = v_batt;
        first_reading_ = false;
    } else {
        filtered_voltage_v_ = config_.ema_alpha * v_batt + (1.0f - config_.ema_alpha) * filtered_voltage_v_;
    }

    data_.adc_raw = adc;
    data_.voltage_v = filtered_voltage_v_;
    data_.cell_voltage_v = filtered_voltage_v_ / config_.cells;
    data_.percent = static_cast<int32_t>(percentFromCellVoltage(data_.cell_voltage_v) + 0.5f);

    if (adc >= config_.adc_max - 5.0f) {
        data_.state = BatteryState::AdcSaturated;
    } else if (data_.cell_voltage_v <= config_.cell_critical_v) {
        data_.state = BatteryState::Critical;
    } else if (data_.cell_voltage_v <= config_.cell_warn_v) {
        data_.state = BatteryState::Low;
    } else {
        data_.state = BatteryState::Ok;
    }

    data_.timestamp_us = micros();
    data_.valid = true;
}

BatteryData Battery::data() const
{
    return data_;
}

}  // namespace fc
