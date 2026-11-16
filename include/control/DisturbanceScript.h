#pragma once

#include <stdint.h>

namespace fc {

/**
 * Per-cycle inputs of the scripted disturbance (everything the script needs
 * from navigation, so it can be unit-tested on a PC without Navigation).
 */
struct DisturbanceScriptInput {
    int flag_wp = -1;          // index of the waypoint being flown TO (log: wp_next)
    int wp_sum = 0;            // mission items incl. item 0 (HOME): 4 = Scenario 1, 7 = Scenario 2
    float along_m = -1.0f;     // distance flown along the current leg from wp_prev; < 0 = not on a WP->WP leg
    float cte_m = 0.0f;        // cross-track error, + = left of the line
    float turn_angle_deg = 0.0f;  // turn angle at the NEXT waypoint (+ = right turn), Navigation::next_turn_angle_deg
    uint32_t now_ms = 0;
};

/**
 * Scripted roll-disturbance sequence for the thesis comparison (L1 fixed vs
 * L1 + fuzzy). Replaces the pilot's hand with a fixed table of events: each
 * event fires ONE pulse (DIST_MS, amplitude from the table) at a fixed place
 * on the route, so both methods receive identical disturbances.
 *
 * The table depends on the scenario (found from the mission size) and on the
 * flight number DIST_FLT (1..5). See docs/disturbance-injection.md.
 *
 *  - Straight event: fires when the aircraft is on the event's leg, has flown
 *    at least pos_m along it, and has been calm (|e| <= kDistCalmBandM for
 *    kDistCalmHoldMs). If the calm condition is not met before end_m, or the
 *    leg is left, the event is SKIPPED (never chased).
 *  - Turn event: fires the moment the waypoint is claimed (leg change), only
 *    if the aircraft was calm on the previous leg. Amplitude sign is relative
 *    to the turn: + = roll toward the turn ("searah"), - = away from it.
 *  - A running pulse is cut when |e| exceeds the abort limit.
 *
 * Pure logic; the caller (DisturbanceInjector) enforces mode/armed/RC/param.
 */
class DisturbanceScript {
public:
    /** Back to the first event (every AUTO entry starts the sequence anew). */
    void reset();

    /** @return signed roll offset in us (0 = none). */
    int16_t update(const DisturbanceScriptInput& in, int flight, uint32_t duration_ms);

    bool active() const { return state_ == State::Active; }
    /** 1-based id of the event currently running or fired last; 0 = none yet. */
    uint8_t eventId() const { return last_event_id_; }
    /** Bumped on every fire; FlightLog uses it to detect a new pulse. */
    uint16_t fireCount() const { return fire_count_; }
    uint16_t skipCount() const { return skip_count_; }
    uint8_t lastSkipId() const { return last_skip_id_; }
    /** 1 = not calm, 2 = slot passed, 3 = aborted (|e| too large), 4 = no leg change seen, 5 = turn sign unknown. */
    uint8_t lastSkipReason() const { return last_skip_reason_; }

private:
    enum class State : uint8_t { Waiting, Active, Done };

    void skip(uint8_t reason);
    void advance();

    State state_ = State::Waiting;
    uint8_t idx_ = 0;
    int16_t offset_us_ = 0;
    uint32_t active_since_ms_ = 0;
    uint32_t active_duration_ms_ = 0;
    bool active_is_turn_ = false;

    int prev_flag_ = -1;
    float turn_angle_last_ = 0.0f;
    bool calm_run_ = false;
    uint32_t calm_since_ms_ = 0;
    bool calm_ok_ = false;

    uint8_t last_event_id_ = 0;
    uint16_t fire_count_ = 0;
    uint16_t skip_count_ = 0;
    uint8_t last_skip_id_ = 0;
    uint8_t last_skip_reason_ = 0;
};

}  // namespace fc
