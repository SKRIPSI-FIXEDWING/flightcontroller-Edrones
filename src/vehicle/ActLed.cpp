#include "vehicle/ActLed.h"

namespace {

constexpr uint32_t kFaultHalfPeriodMs = 100;        // 5 Hz
constexpr uint32_t kCalibratingHalfPeriodMs = 500;  // 1 Hz

}  // namespace

namespace fc {

ActLed::ActLed(const ActLedConfig& config)
    : config_(config)
{
}

void ActLed::begin()
{
    pinMode(config_.pin, OUTPUT);
    initialized_ = true;
    write(false);
}

void ActLed::setPattern(ActLedPattern pattern)
{
    pattern_ = pattern;
}

void ActLed::update()
{
    if (!initialized_) {
        return;
    }

    const uint32_t now_ms = millis();

    switch (pattern_) {
    case ActLedPattern::Off:
        write(false);
        break;

    case ActLedPattern::ImuReady:
        write(true);
        break;

    case ActLedPattern::ImuFault:
    case ActLedPattern::ImuCalibrating: {
        const uint32_t half_period_ms = (pattern_ == ActLedPattern::ImuFault)
                                            ? kFaultHalfPeriodMs
                                            : kCalibratingHalfPeriodMs;
        if (now_ms - last_toggle_ms_ >= half_period_ms) {
            last_toggle_ms_ = now_ms;
            write(!level_on_);
        }
        break;
    }
    }
}

void ActLed::write(bool on)
{
    level_on_ = on;
    digitalWrite(config_.pin, (on == config_.active_high) ? HIGH : LOW);
}

}  // namespace fc
