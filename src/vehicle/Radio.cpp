#include "vehicle/Radio.h"

#include <math.h>

namespace fc {

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

void Radio::begin()
{
    sbus_rx_.Begin();

    if (sbus_rx_.Read()) {
        sbus_data_ = sbus_rx_.data();
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
                if (sbus_rx_.Read()) {
                    sbus_data_ = sbus_rx_.data();
                    arming_ = readArmSwitch();
                    signal_lost_ = sbus_data_.lost_frame;
                }
            }
        }

        if (signal_lost_) {
            Serial.println("Signal lost, please reconnect");
            while (signal_lost_) {
                if (sbus_rx_.Read()) {
                    sbus_data_ = sbus_rx_.data();
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
    // Mode selection uses ch_mode (ch[5]) / ch_mode_backup (ch[6]): low/mid/high
    // combinations map to modes 1-5, matching legacy remote_loop().
    if (ch_mode_ < 1000) {
        if (ch_mode_backup_ <= 1000) {
            mode_now_ = 1;
        } else if (ch_mode_backup_ > 1000 && ch_mode_backup_ <= 1512) {
            mode_now_ = 2;
        } else if (ch_mode_backup_ >= 1512) {
            mode_now_ = 3;
        }
    } else {
        if (ch_mode_ > 1000 && ch_mode_ <= 1512 && ch_mode_backup_ >= 1512) {
            mode_now_ = 4;
        } else if (ch_mode_ >= 1512 && ch_mode_backup_ >= 1512) {
            mode_now_ = 5;
        }
    }
}

void Radio::applyFailsafe()
{
    if (signal_lost_) {
        if (!failsafe_active_) {
            Serial.println("!!! FAILSAFE TRIGGERED: RC SIGNAL LOST !!!");
            failsafe_active_ = true;
        }

        ch_roll_ = 1500;
        ch_pitch_ = 1500;
        ch_yaw_ = 1500;

        if (!arming_) {
            ch_throttle_ = 988;
        } else {
            // Airborne with RC lost: force AUTO so the aircraft continues
            // navigating the mission/RTL instead of falling back to a
            // manual mode with no stick input. Safety-critical behavior,
            // preserved exactly from legacy remote_loop().
            mode_now_ = 3;
            ch_throttle_ = 1500;
        }
    } else if (failsafe_active_) {
        Serial.println(">>> RC SIGNAL RESTORED: FAILSAFE CLEARED <<<");
        failsafe_active_ = false;
    }
}

void Radio::update()
{
    if (sbus_rx_.Read()) {
        sbus_data_ = sbus_rx_.data();

        if (sbus_data_.lost_frame || sbus_data_.failsafe) {
            signal_lost_ = true;
        } else {
            signal_lost_ = false;
            last_valid_frame_ms_ = millis();
        }

        if (!signal_lost_) {
            uint16_t new_roll = static_cast<uint16_t>(sbus_data_.ch[0] * config_.roll_scale + config_.roll_offset);
            uint16_t new_pitch = static_cast<uint16_t>(sbus_data_.ch[1] * config_.pitch_scale + config_.pitch_offset);
            uint16_t new_throttle =
                static_cast<uint16_t>(sbus_data_.ch[2] * config_.throttle_scale + config_.throttle_offset);
            uint16_t new_yaw = static_cast<uint16_t>(sbus_data_.ch[3] * config_.yaw_scale + config_.yaw_offset);

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

            arming_ = readArmSwitch();

            ch_mode_ = static_cast<uint16_t>(sbus_data_.ch[5] * config_.mode_scale + config_.mode_offset);
            ch_mode_backup_ = static_cast<uint16_t>(sbus_data_.ch[6] * config_.mode_scale + config_.mode_offset);
            ch_vehicle_mode_ = static_cast<uint16_t>(sbus_data_.ch[7] * config_.mode_scale + config_.mode_offset);

            decodeModeSwitch();
        }
    }

    if (millis() - last_valid_frame_ms_ > config_.signal_timeout_ms) {
        signal_lost_ = true;
    }

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

bool Radio::armed() const { return arming_; }
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
