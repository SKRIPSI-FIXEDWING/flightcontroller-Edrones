#pragma once

#include <Arduino.h>
#include <sbus.h>
#include <stdint.h>

#include "FC_Config.h"

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

    // Bench-confirmed 2026-08-18 on this airframe/TX: pitch stick (CH2)
    // reads backwards relative to elevator direction in Manual passthrough.
    // Inverted here (mirrored around the pwm_min/pwm_max midpoint) instead
    // of rewiring the transmitter/receiver, so BOTH Manual's direct
    // passthrough (Actuator::writeManual) AND FBWA's stick-derived pitch
    // setpoint (ModeFbwa::mapStickToDeg) get the corrected direction.
    bool invert_pitch = false;
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
#if FC_RC_PROTOCOL_CRSF
    explicit Radio(HardwareSerial& serial = Serial3);
#else
    explicit Radio(HardwareSerial& serial = Serial8);
#endif
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

    /** Raw, un-scaled SBUS count for ch[4] (the arm switch, CH5 on this
     * TX) -- for logging/diagnostics only, NOT what readArmSwitch()/armed()
     * compares against (that reads the same raw sbus_data_.ch[4] directly). */
    uint16_t channelArmRaw() const;

    /** CH5 (arm switch) scaled to PWM-equivalent us -- for GCS display. */
    uint16_t channelArm() const;

    bool armed() const;

    /**
     * Arm/disarm commanded from the GCS (MAVLink COMPONENT_ARM_DISARM).
     * The RC arm switch is edge-triggered (ArduPilot/Khageswara style): it
     * only changes the armed state when the switch MOVES, so a GCS command is
     * not overwritten by the next RC frame. Moving the switch always wins.
     */
    void setArmedFromGcs(bool armed);

    bool signalLost() const;
    bool failsafeActive() const;

    /** 1-5, decoded from channelMode()/channelModeBackup() (matches legacy mode_now). */
    int modeNow() const;

    uint32_t timestampUs() const;
    uint32_t sequence() const;

    // CRSF LINK_STATISTICS (uplink, as seen by the receiver). Always 0 / false
    // in SBUS builds.
    bool linkStatsValid() const { return link_stats_valid_; }
    uint8_t linkQuality() const { return link_quality_; }   // 0-100 %
    int8_t rssiDbm() const { return rssi_dbm_; }            // best antenna, dBm
    int8_t snrDb() const { return snr_db_; }

    /** Scales a raw SBUS-derived PWM-equivalent channel value to roughly [-1, 1]. */
    static float outputScaler(uint16_t channel_value);

private:
    bool readArmSwitch() const;
    void decodeModeSwitch();
    void applyFailsafe();

    /** Reads whatever the receiver has sent; returns true if a new RC frame
     * was decoded into sbus_data_ (SBUS or CRSF depending on build). */
    bool readFrame();

#if FC_RC_PROTOCOL_CRSF
    void crsfProcessBuffer();
    bool crsfHandleFrame(const uint8_t* frame);  // frame = [addr][len][type]...[crc]

    static constexpr size_t kCrsfBufSize = 64;
    uint8_t crsf_buf_[kCrsfBufSize] = {};
    size_t crsf_len_ = 0;
    bool crsf_rc_frame_ = false;
#endif

    bool link_stats_valid_ = false;
    uint8_t link_quality_ = 0;
    int8_t rssi_dbm_ = 0;
    int8_t snr_db_ = 0;
    uint32_t last_link_stats_ms_ = 0;

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
    bool arm_switch_seen_ = false;   // first valid RC frame after boot read yet
    bool last_arm_switch_ = false;   // arm switch state at the previous frame
    bool signal_lost_ = true;
    bool failsafe_active_ = false;
    int mode_now_ = 1;

    uint32_t last_valid_frame_ms_ = 0;
    uint32_t timestamp_us_ = 0;
    uint32_t sequence_ = 0;
};

}  // namespace fc
