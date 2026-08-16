#pragma once

#include <Arduino.h>
#include <sbus.h>
#include <stdint.h>

namespace fc {

struct RadioConfig {
    // Per-channel linear-fit calibration (remote-specific — see legacy
    // Radio.h's comment on why these constants exist: different remotes
    // have slightly different SBUS-to-PWM-equivalent scaling).
    float roll_scale = 0.6267f, roll_offset = 877.91f;
    float pitch_scale = 0.6251f, pitch_offset = 878.83f;
    float throttle_scale = 0.6274f, throttle_offset = 879.25f;
    float yaw_scale = 0.6304f, yaw_offset = 876.19f;
    float mode_scale = 0.6267f, mode_offset = 877.91f;

    uint16_t pwm_min = 988;
    uint16_t pwm_max = 2012;
    uint16_t deadband = 4;
    uint32_t signal_timeout_ms = 500;

    // ch[4] (0-indexed) above this = armed.
    uint16_t arm_threshold = 1500;
};

/**
 * SBUS-only RC input (legacy Radio.h). The legacy dual-source setup (SBUS +
 * MAVLink RC-override in Mavlink.h's applyRcToGlobals(), both writing the
 * same globals with no arbitration — a real race condition, flagged during
 * planning) is NOT reproduced: this module is SBUS-only, per your decision.
 * MAVLink RC-override can be added later as an explicitly-arbitrated second
 * source if confirmed needed.
 *
 * Scheduling stays outside this class: call begin() once at startup, then
 * update() from the flight scheduler.
 */
class Radio final {
public:
    explicit Radio(HardwareSerial& serial = Serial8);
    Radio(HardwareSerial& serial, const RadioConfig& config);

    Radio(const Radio&) = delete;
    Radio& operator=(const Radio&) = delete;

    /**
     * Pre-flight safety interlock: blocks until the transmitter is
     * disarmed and a signal is present. Intentional blocking behavior
     * carried over from legacy remote_setup() — do not make this
     * non-blocking without a compensating pre-arm check elsewhere.
     */
    void begin();

    /** Non-blocking: reads the latest SBUS frame and applies RC-loss failsafe. */
    void update();

    uint16_t channelRoll() const;
    uint16_t channelPitch() const;
    uint16_t channelThrottle() const;
    uint16_t channelYaw() const;
    uint16_t channelMode() const;
    uint16_t channelModeBackup() const;
    uint16_t channelVehicleMode() const;

    bool armed() const;
    bool signalLost() const;
    bool failsafeActive() const;

    /** 1-5, decoded from channelMode()/channelModeBackup() (matches legacy mode_now). */
    int modeNow() const;

    uint32_t timestampUs() const;
    uint32_t sequence() const;

    /** Scales a raw SBUS-derived PWM-equivalent channel value to roughly [-1, 1]. */
    static float outputScaler(uint16_t channel_value);

private:
    bool readArmSwitch() const;
    void decodeModeSwitch();
    void applyFailsafe();

    HardwareSerial& serial_;
    RadioConfig config_{};
    bfs::SbusRx sbus_rx_;
    bfs::SbusData sbus_data_{};

    uint16_t ch_roll_ = 1500;
    uint16_t ch_pitch_ = 1500;
    uint16_t ch_throttle_ = 988;
    uint16_t ch_yaw_ = 1500;
    uint16_t ch_mode_ = 0;
    uint16_t ch_mode_backup_ = 0;
    uint16_t ch_vehicle_mode_ = 0;

    bool arming_ = false;
    bool signal_lost_ = true;
    bool failsafe_active_ = false;
    int mode_now_ = 1;

    uint32_t last_valid_frame_ms_ = 0;
    uint32_t timestamp_us_ = 0;
    uint32_t sequence_ = 0;
};

}  // namespace fc
