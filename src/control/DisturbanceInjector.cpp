#include "control/DisturbanceInjector.h"

#include "FC_Config.h"

namespace fc {

DisturbanceInjector::DisturbanceInjector(const float& enable_flag, const float& amplitude_us,
                                         const float& duration_ms)
    : enable_flag_(enable_flag), amplitude_us_(amplitude_us), duration_ms_(duration_ms)
{
}

void DisturbanceInjector::reset()
{
    state_ = State::WaitLow;
    offset_us_ = 0;
    low_seen_ = false;
}

int16_t DisturbanceInjector::update(bool mode_allowed, bool armed, bool rc_ok, uint16_t trigger_ch_us,
                                    uint32_t now_ms)
{
    if (enable_flag_ < 0.5f || !mode_allowed || !armed || !rc_ok) {
        reset();
        return 0;
    }

    const bool switch_low = trigger_ch_us < kDistSwitchLowUs;
    const bool switch_high = trigger_ch_us > kDistSwitchHighUs;

    switch (state_) {
    case State::WaitLow:
        offset_us_ = 0;
        // Restart the LOW-hold timer whenever the switch leaves LOW.
        if (!switch_low) {
            low_seen_ = false;
        } else if (!low_seen_) {
            low_seen_ = true;
            low_since_ms_ = now_ms;
        } else if (now_ms - low_since_ms_ >= kDistSwitchLowHoldMs) {
            state_ = State::Ready;
        }
        break;

    case State::Ready:
        if (switch_high) {
            // Latch amplitude and duration so one pulse never changes mid-way.
            float amplitude = amplitude_us_;
            if (amplitude > kDistMaxAmplitudeUs) {
                amplitude = kDistMaxAmplitudeUs;
            } else if (amplitude < -kDistMaxAmplitudeUs) {
                amplitude = -kDistMaxAmplitudeUs;
            }
            float duration = duration_ms_;
            if (duration > static_cast<float>(kDistMaxDurationMs)) {
                duration = static_cast<float>(kDistMaxDurationMs);
            } else if (duration < 0.0f) {
                duration = 0.0f;
            }
            offset_us_ = static_cast<int16_t>(amplitude);
            active_duration_ms_ = static_cast<uint32_t>(duration);
            active_since_ms_ = now_ms;
            if (offset_us_ != 0 && active_duration_ms_ > 0) {
                state_ = State::Active;
            } else {
                reset();  // zero amplitude/duration: nothing to inject, wait for a fresh LOW
            }
        }
        break;

    case State::Active:
        // Runs the full duration regardless of the switch, so every pulse is identical.
        if (now_ms - active_since_ms_ >= active_duration_ms_) {
            reset();
        }
        break;
    }
    return offset_us_;
}

}  // namespace fc
