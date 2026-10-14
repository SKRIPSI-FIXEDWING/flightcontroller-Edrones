#include "vehicle/Radio.h"

#include <math.h>
#include <string.h>

namespace fc {

#if FC_RC_PROTOCOL_CRSF
namespace {

constexpr uint8_t kCrsfAddrFlightController = 0xC8;
constexpr uint8_t kCrsfTypeRcChannelsPacked = 0x16;
constexpr uint8_t kCrsfTypeLinkStatistics = 0x14;
constexpr uint8_t kCrsfRcPayloadLen = 22;  // 16 channels x 11 bit

uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

}  // namespace
#endif

Radio::Radio(HardwareSerial& serial)
    : serial_(serial), sbus_rx_(&serial)
{
}

Radio::Radio(HardwareSerial& serial, const RadioConfig& config)
    : serial_(serial), config_(config), sbus_rx_(&serial)
{
}

bool Radio::readArmSwitch() const
{
    return sbus_data_.ch[4] > config_.arm_threshold;
}

bool Radio::readFrame()
{
#if FC_RC_PROTOCOL_CRSF
    crsf_rc_frame_ = false;
    while (serial_.available() > 0) {
        if (crsf_len_ >= kCrsfBufSize) {
            crsf_len_ = 0;  // cannot happen with the shift-on-error logic below, but never overrun
        }
        crsf_buf_[crsf_len_++] = static_cast<uint8_t>(serial_.read());
        crsfProcessBuffer();
    }
    return crsf_rc_frame_;
#else
    if (sbus_rx_.Read()) {
        sbus_data_ = sbus_rx_.data();
        return true;
    }
    return false;
#endif
}

#if FC_RC_PROTOCOL_CRSF
void Radio::crsfProcessBuffer()
{
    // Frame layout: [addr][len][type][payload...][crc8]; len counts
    // type+payload+crc, so a whole frame is len+2 bytes. Resync by dropping
    // one byte at a time until the head looks like a valid frame.
    for (;;) {
        if (crsf_len_ >= 1 && crsf_buf_[0] != kCrsfAddrFlightController) {
            memmove(crsf_buf_, crsf_buf_ + 1, --crsf_len_);
            continue;
        }
        if (crsf_len_ < 2) {
            return;
        }
        const uint8_t len = crsf_buf_[1];
        if (len < 2 || len > kCrsfBufSize - 2) {
            memmove(crsf_buf_, crsf_buf_ + 1, --crsf_len_);
            continue;
        }
        const size_t total = static_cast<size_t>(len) + 2;
        if (crsf_len_ < total) {
            return;
        }
        if (crc8(crsf_buf_ + 2, len - 1) == crsf_buf_[total - 1]) {
            if (crsfHandleFrame(crsf_buf_)) {
                crsf_rc_frame_ = true;
            }
            crsf_len_ -= total;
            memmove(crsf_buf_, crsf_buf_ + total, crsf_len_);
        } else {
            memmove(crsf_buf_, crsf_buf_ + 1, --crsf_len_);
        }
    }
}

bool Radio::crsfHandleFrame(const uint8_t* frame)
{
    const uint8_t len = frame[1];
    const uint8_t type = frame[2];
    const uint8_t* payload = frame + 3;

    if (type == kCrsfTypeRcChannelsPacked && len == kCrsfRcPayloadLen + 2) {
        uint32_t bits = 0;
        uint8_t nbits = 0;
        size_t idx = 0;
        for (size_t i = 0; i < kCrsfRcPayloadLen && idx < 16; ++i) {
            bits |= static_cast<uint32_t>(payload[i]) << nbits;
            nbits += 8;
            while (nbits >= 11 && idx < 16) {
                sbus_data_.ch[idx++] = static_cast<int16_t>(bits & 0x7FF);
                bits >>= 11;
                nbits -= 11;
            }
        }
        sbus_data_.lost_frame = false;
        sbus_data_.failsafe = false;
        return true;
    }

    if (type == kCrsfTypeLinkStatistics && len >= 12) {
        // payload: uplink RSSI ant1 (-dBm), RSSI ant2 (-dBm), uplink LQ,
        // uplink SNR (dB), active antenna, rf mode, tx power, downlink ...
        // Smaller magnitude = stronger; with Gemini both antennas report.
        const uint8_t r1 = payload[0];
        const uint8_t r2 = payload[1];
        const uint8_t best = (r2 != 0 && r2 < r1) ? r2 : r1;
        rssi_dbm_ = -static_cast<int8_t>(best);
        link_quality_ = payload[2];
        snr_db_ = static_cast<int8_t>(payload[3]);
        link_stats_valid_ = true;
        last_link_stats_ms_ = millis();
    }
    return false;
}
#endif

void Radio::begin()
{
#if !FC_RC_PROTOCOL_CRSF
    // CRSF: the UART (420000 baud + extended RX buffer) is opened by
    // main.cpp's setup(), next to the other Serial.begin() calls.
    sbus_rx_.Begin();
#endif

    if (readFrame()) {
        arming_ = readArmSwitch();
        signal_lost_ = sbus_data_.lost_frame;

        // Pre-flight safety interlock: refuse to proceed while armed or
        // while the transmitter signal is missing. Deliberately blocking
        // and printing operator feedback (unlike this codebase's sensor
        // drivers, which stay silent) -- this is a safety handshake with a
        // human, not routine debug logging.
        if (arming_) {
            Serial.println("Please disarm the remote for safety");
            while (arming_) {
                if (readFrame()) {
                    arming_ = readArmSwitch();
                    signal_lost_ = sbus_data_.lost_frame;
                }
            }
        }

        if (signal_lost_) {
            Serial.println("Signal lost, please reconnect");
            while (signal_lost_) {
                if (readFrame()) {
                    arming_ = readArmSwitch();
                    signal_lost_ = sbus_data_.lost_frame;
                }
            }
        }
    }

    last_valid_frame_ms_ = millis();
}

void Radio::decodeModeSwitch()
{
    // Single 3-position switch on ch_mode (ch[5]/CH6): low=Manual(1),
    // mid=FBWA(2), high=Auto(3). ch_mode_backup_ (ch[6]/CH7) is READ but
    // deliberately NOT used here.
    //
    // The previous version combined ch_mode_ AND ch_mode_backup_ to reach
    // modes 1-5 (a legacy 2-switch scheme for a UAV type with more modes).
    // On a transmitter that only wires ONE switch to CH6 and leaves CH7
    // unassigned/floating (this airframe's actual setup, confirmed
    // 2026-08-18), the floating backup channel sits near the old scheme's
    // 1512 threshold: switch-LOW decoded to mode 2 instead of 1, and
    // switch-MID/HIGH matched no branch at all, leaving mode_now_ frozen on
    // whatever it last was -- symptomatically identical to what looked like
    // "mode 1/2 lag" and "FBWA never engaging" during bench testing.
    //
    // If a second physical switch is wired to CH7 later (e.g. to add a 4th/
    // 5th mode), reintroduce a ch_mode_backup_-based branch here instead of
    // reverting this whole function.
    if (ch_mode_ < 1000) {
        mode_now_ = 1;
    } else if (ch_mode_ <= 1512) {
        mode_now_ = 2;
    } else {
        mode_now_ = 3;
    }
}

void Radio::applyFailsafe()
{
    if (signal_lost_) {
        if (!failsafe_active_) {
            Serial.println("!!! FAILSAFE TRIGGERED: RC SIGNAL LOST !!!");
            failsafe_active_ = true;
        }

        // RC-loss failsafe tidak lagi memaksa AUTO (mode_now_ tidak diubah);
        // hanya menetralkan stick. Mode terakhir tetap berjalan.
        ch_roll_ = 1500;
        ch_pitch_ = 1500;
        ch_yaw_ = 1500;
        if (!arming_) {
            ch_throttle_ = 988;
        }
    } else if (failsafe_active_) {
        Serial.println(">>> RC SIGNAL RESTORED: FAILSAFE CLEARED <<<");
        failsafe_active_ = false;
    }
}

void Radio::update()
{
    if (readFrame()) {
        if (sbus_data_.lost_frame || sbus_data_.failsafe) {
            signal_lost_ = true;
        } else {
            signal_lost_ = false;
            last_valid_frame_ms_ = millis();
        }

        if (!signal_lost_) {
            uint16_t new_roll = static_cast<uint16_t>(sbus_data_.ch[0] * config_.roll_scale + config_.roll_offset);
            uint16_t new_pitch = static_cast<uint16_t>(sbus_data_.ch[1] * config_.pitch_scale + config_.pitch_offset);
            uint16_t new_throttle = static_cast<uint16_t>(sbus_data_.ch[2] * config_.throttle_scale + config_.throttle_offset);
            uint16_t new_yaw = static_cast<uint16_t>(sbus_data_.ch[3] * config_.yaw_scale + config_.yaw_offset);

            if (config_.invert_pitch) {
                new_pitch = config_.pwm_min + config_.pwm_max - new_pitch;
            }

            new_roll = constrain(new_roll, config_.pwm_min, config_.pwm_max);
            new_pitch = constrain(new_pitch, config_.pwm_min, config_.pwm_max);
            new_throttle = constrain(new_throttle, config_.pwm_min, config_.pwm_max);
            new_yaw = constrain(new_yaw, config_.pwm_min, config_.pwm_max);

            // Deadband to prevent receiver/gimbal jitter when sticks are stationary.
            if (abs(static_cast<int>(new_roll) - static_cast<int>(ch_roll_)) > config_.deadband) {
                ch_roll_ = new_roll;
            }
            if (abs(static_cast<int>(new_pitch) - static_cast<int>(ch_pitch_)) > config_.deadband) {
                ch_pitch_ = new_pitch;
            }
            if (abs(static_cast<int>(new_throttle) - static_cast<int>(ch_throttle_)) > config_.deadband) {
                ch_throttle_ = new_throttle;
            }
            if (abs(static_cast<int>(new_yaw) - static_cast<int>(ch_yaw_)) > config_.deadband) {
                ch_yaw_ = new_yaw;
            }

            // Edge-triggered: the RC switch changes the armed state only when
            // it moves (the first frame after boot is authoritative).
            const bool arm_switch = readArmSwitch();
            if (!arm_switch_seen_) {
                arm_switch_seen_ = true;
                last_arm_switch_ = arm_switch;
                arming_ = arm_switch;
            } else if (arm_switch != last_arm_switch_) {
                last_arm_switch_ = arm_switch;
                arming_ = arm_switch;
            }

            ch_mode_ = static_cast<uint16_t>(sbus_data_.ch[5] * config_.mode_scale + config_.mode_offset);
            ch_mode_backup_ = static_cast<uint16_t>(sbus_data_.ch[6] * config_.mode_scale + config_.mode_offset);
            ch_vehicle_mode_ = static_cast<uint16_t>(sbus_data_.ch[7] * config_.mode_scale + config_.mode_offset);

            decodeModeSwitch();
        }
    }

    if (millis() - last_valid_frame_ms_ > config_.signal_timeout_ms) {
        signal_lost_ = true;
    }

#if FC_RC_PROTOCOL_CRSF
    // Receiver reports 0% uplink LQ (fresh stats only): link is gone even if
    // a stale RC frame was still being repeated.
    if (link_stats_valid_ && link_quality_ == 0 && millis() - last_link_stats_ms_ < 1000) {
        signal_lost_ = true;
    }
#endif

    // applyFailsafe();

    timestamp_us_ = micros();
    sequence_ += 1U;
}

uint16_t Radio::channelRoll() const { return ch_roll_; }
uint16_t Radio::channelPitch() const { return ch_pitch_; }
uint16_t Radio::channelThrottle() const { return ch_throttle_; }
uint16_t Radio::channelYaw() const { return ch_yaw_; }
uint16_t Radio::channelMode() const { return ch_mode_; }
uint16_t Radio::channelModeBackup() const { return ch_mode_backup_; }
uint16_t Radio::channelVehicleMode() const { return ch_vehicle_mode_; }
uint16_t Radio::channelArmRaw() const { return sbus_data_.ch[4]; }
uint16_t Radio::channelArm() const
{
    return static_cast<uint16_t>(sbus_data_.ch[4] * config_.mode_scale + config_.mode_offset);
}

bool Radio::armed() const { return arming_; }
void Radio::setArmedFromGcs(bool armed) { arming_ = armed; }
bool Radio::signalLost() const { return signal_lost_; }
bool Radio::failsafeActive() const { return failsafe_active_; }
int Radio::modeNow() const { return mode_now_; }

uint32_t Radio::timestampUs() const { return timestamp_us_; }
uint32_t Radio::sequence() const { return sequence_; }

float Radio::outputScaler(uint16_t channel_value)
{
    return 0.002f * static_cast<float>(channel_value) - 3.0f;
}

}  // namespace fc
