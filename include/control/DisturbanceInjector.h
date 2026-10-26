#pragma once

#include <stdint.h>

namespace fc {

/**
 * External roll-disturbance injection for the thesis experiments ("path
 * following under external roll disturbance"). Produces a signed PWM offset
 * (us, roll-command domain, + = roll right = same direction as a right roll
 * stick) that fc::Actuator adds to the aileron command AFTER the attitude
 * controller. The pilot triggers it with the roll stick while in AUTO; two
 * discrete levels (FC_Config.h kDistLowUs/kDistHighUs, 125/250 us).
 *
 * Safety rules, all enforced here (never trust the caller to remember):
 *  - Off unless the DIST_ENABLE param is 1 (default 0). `enable_flag` is read
 *    live every call, so setting it back to 0 from the GCS stops it at once.
 *  - Only while in AUTO, armed, with a live RC signal; any of these false
 *    clears it immediately and returns to "not yet centered".
 *  - After entering AUTO or after an activation ends, the stick must sit near
 *    center for kDistCenterHoldMs before a new activation is allowed -- a
 *    stick already deflected when AUTO engages does nothing.
 *  - One activation lasts at most kDistMaxDurationMs, then switches off and
 *    stays off until the stick has been centered again.
 *
 * Called from the control task only (ModeAuto::_update); not thread-safe.
 */
class DisturbanceInjector {
public:
    explicit DisturbanceInjector(const float& enable_flag);

    /** Drop any active/latched state (mode exit, AUTO entry). */
    void reset();

    /**
     * @param in_auto       the caller is ModeAuto (always true there; false
     *                      is for completeness/tests)
     * @param roll_stick_us calibrated roll channel, ~988..2012, center ~1500
     * @return signed offset in us to add to the roll command; 0 = none
     */
    int16_t update(bool in_auto, bool armed, bool rc_ok, uint16_t roll_stick_us, uint32_t now_ms);

    bool active() const { return offset_us_ != 0; }
    int16_t offsetUs() const { return offset_us_; }

private:
    enum class State : uint8_t { WaitCenter, Ready, Active, Latched };

    const float& enable_flag_;
    State state_ = State::WaitCenter;
    int16_t offset_us_ = 0;
    uint32_t center_since_ms_ = 0;
    bool center_seen_ = false;
    uint32_t active_since_ms_ = 0;
};

}  // namespace fc
