#include "control/DisturbanceInjector.h"

#include "FC_Config.h"

namespace fc {

namespace {

// Stick deviation -> signed level. 0 inside the deadband.
int16_t levelForStick(uint16_t roll_stick_us)
{
    const int32_t deviation = static_cast<int32_t>(roll_stick_us) - 1500;
    const uint32_t magnitude = static_cast<uint32_t>(deviation < 0 ? -deviation : deviation);
    if (magnitude < kDistDeadbandUs) {
        return 0;
    }
    const int16_t level = magnitude < kDistLevelSplitUs ? kDistLowUs : kDistHighUs;
    return deviation < 0 ? static_cast<int16_t>(-level) : level;
}

}  // namespace

DisturbanceInjector::DisturbanceInjector(const float& enable_flag)
    : enable_flag_(enable_flag)
{
}

void DisturbanceInjector::reset()
{
    state_ = State::WaitCenter;
    offset_us_ = 0;
    center_seen_ = false;
}

int16_t DisturbanceInjector::update(bool in_auto, bool armed, bool rc_ok, uint16_t roll_stick_us,
                                    uint32_t now_ms)
{
    if (enable_flag_ < 0.5f || !in_auto || !armed || !rc_ok) {
        reset();
        return 0;
    }

    const int16_t level = levelForStick(roll_stick_us);

    switch (state_) {
    case State::WaitCenter:
    case State::Latched:
        offset_us_ = 0;
        // Back to the start of the centered-hold timer whenever the stick leaves the deadband.
        if (level != 0) {
            center_seen_ = false;
        } else if (!center_seen_) {
            center_seen_ = true;
            center_since_ms_ = now_ms;
        } else if (now_ms - center_since_ms_ >= kDistCenterHoldMs) {
            state_ = State::Ready;
        }
        break;

    case State::Ready:
        if (level != 0) {
            state_ = State::Active;
            active_since_ms_ = now_ms;
            offset_us_ = level;
        }
        break;

    case State::Active:
        if (level == 0) {
            // Released: back to waiting for a fresh centered period.
            state_ = State::WaitCenter;
            offset_us_ = 0;
            center_seen_ = false;
        } else if (now_ms - active_since_ms_ >= kDistMaxDurationMs) {
            state_ = State::Latched;  // timed out: off until the stick has been centered again
            offset_us_ = 0;
            center_seen_ = false;
        } else {
            offset_us_ = level;  // stick moved between levels/sides while held
        }
        break;
    }
    return offset_us_;
}

}  // namespace fc
