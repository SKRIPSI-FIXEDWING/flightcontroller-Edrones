#include "vehicle/Battery.h"

namespace fc {

Battery::Battery(const BatteryConfig& config)
    : config_(config)
{
}

void Battery::begin()
{
    pinMode(config_.analog_pin, INPUT);
    for (uint8_t i = 0; i < kMovingAverageSamples; ++i) {
        readings_[i] = 0;
    }
    read_index_ = 0;
    running_total_ = 0;
}

void Battery::update()
{
    const uint32_t now_ms = millis();
    if (now_ms - last_sample_ms_ < config_.sample_interval_ms) {
        return;
    }
    last_sample_ms_ = now_ms;

    running_total_ -= readings_[read_index_];
    readings_[read_index_] = analogRead(config_.analog_pin);
    running_total_ += readings_[read_index_];

    read_index_ = (read_index_ + 1) % kMovingAverageSamples;

    const int32_t average = running_total_ / kMovingAverageSamples;
    const float raw_millivolts =
        static_cast<float>(map(average, 0, 1023, 0, static_cast<long>(config_.adc_max_millivolts)));

    data_.voltage_v = (raw_millivolts / 1000.0f) + config_.voltage_offset_v;
    data_.percent = static_cast<int32_t>(
        map(static_cast<long>(data_.voltage_v * 100.0f), static_cast<long>(config_.percent_curve_min_v * 100.0f),
            static_cast<long>(config_.percent_curve_max_v * 100.0f), 0, 100));
    data_.timestamp_us = micros();
    data_.valid = true;
}

BatteryData Battery::data() const
{
    return data_;
}

}  // namespace fc
