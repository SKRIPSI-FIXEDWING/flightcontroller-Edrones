#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace fc {

struct ActLedConfig {
    uint8_t pin = 22;  // Teensy 4.1 pin 22 (A8)
    bool active_high = true;
};

/**
 * Physical debug indicator, so subsystem state can be read without a
 * Serial/MAVLink connection. Currently reports IMU state only.
 */
enum class ActLedPattern : uint8_t {
    Off = 0,
    ImuFault,        // 5 Hz blink: init failed, read failing, or data stale
    ImuCalibrating,  // 1 Hz blink: IMU alive but not fully calibrated
    ImuReady,        // solid on: IMU healthy, fresh, fully calibrated
};

class ActLed final {
public:
    explicit ActLed(const ActLedConfig& config = ActLedConfig{});

    void begin();

    /** Select the pattern to show; cheap, safe to call every loop. */
    void setPattern(ActLedPattern pattern);

    /** Drive the blink state machine; call every ~10-50 ms. */
    void update();

private:
    void write(bool on);

    ActLedConfig config_{};
    ActLedPattern pattern_ = ActLedPattern::Off;
    bool initialized_ = false;
    bool level_on_ = false;
    uint32_t last_toggle_ms_ = 0;
};

}  // namespace fc
