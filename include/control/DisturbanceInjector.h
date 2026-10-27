#pragma once

#include <stdint.h>

namespace fc {

/**
 * External roll-disturbance injection for the thesis experiments (loiter /
 * path following under an external disturbance). Produces a signed PWM
 * offset (us, roll-command domain, + = roll right = same direction as a
 * right roll stick) that fc::Actuator adds to the aileron command AFTER the
 * attitude controller.
 *
 * Triggered by the CH7 switch: every LOW -> HIGH flip injects ONE pulse of
 * DIST_US for DIST_MS (params, latched when the pulse starts), so every run
 * gets the identical disturbance regardless of the pilot's hand. The pulse
 * runs its full duration even if the switch is flipped back; only the
 * safety conditions below end it early.
 *
 * Safety rules, all enforced here (never trust the caller to remember):
 *  - Off unless the DIST_ENABLE param is 1 (default 0). `enable_flag` is read
 *    live every call, so setting it back to 0 from the GCS stops it at once.
 *  - Only in an allowed mode (AUTO/LOITER), armed, with a live RC signal;
 *    any of these false clears it immediately and returns to "wait for LOW".
 *  - After entering the mode or after a pulse ends, CH7 must sit LOW for
 *    kDistSwitchLowHoldMs before a LOW -> HIGH flip counts -- a switch that
 *    is already HIGH when the mode engages does nothing, and an unassigned
 *    channel floating near 1500 is never LOW nor HIGH.
 *  - Amplitude and duration are clamped to kDistMaxAmplitudeUs /
 *    kDistMaxDurationMs whatever the params say.
 *
 * Called from the control task only (ModeAuto/ModeLoiter::_update); not
 * thread-safe.
 */
class DisturbanceInjector {
public:
    DisturbanceInjector(const float& enable_flag, const float& amplitude_us, const float& duration_ms);

    /** Drop any active/latched state (mode entry/exit). */
    void reset();

    /**
     * @param mode_allowed  the caller is ModeAuto or ModeLoiter (false is for
     *                      completeness/tests)
     * @param trigger_ch_us calibrated CH7, ~988..2012
     * @return signed offset in us to add to the roll command; 0 = none
     */
    int16_t update(bool mode_allowed, bool armed, bool rc_ok, uint16_t trigger_ch_us, uint32_t now_ms);

    bool active() const { return offset_us_ != 0; }
    int16_t offsetUs() const { return offset_us_; }

private:
    enum class State : uint8_t { WaitLow, Ready, Active };

    const float& enable_flag_;
    const float& amplitude_us_;
    const float& duration_ms_;
    State state_ = State::WaitLow;
    int16_t offset_us_ = 0;
    bool low_seen_ = false;
    uint32_t low_since_ms_ = 0;
    uint32_t active_since_ms_ = 0;
    uint32_t active_duration_ms_ = 0;
};

}  // namespace fc
