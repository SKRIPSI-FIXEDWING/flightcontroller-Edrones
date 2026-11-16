#include "control/DisturbanceScript.h"

#include <math.h>

#include "FC_Config.h"

namespace fc {

namespace {

enum class Kind : uint8_t { Straight, Turn };

// flag    : Straight -> leg being flown (wp_next); Turn -> wp_next AFTER the claim
// pos_m   : Straight only, earliest distance along the leg
// end_m   : Straight only, latest distance at which the event may still start
// amp     : index into kAmp
struct Event {
    Kind kind;
    uint8_t flag;
    int16_t pos_m;
    int16_t end_m;
    uint8_t amp;
};

// A1 = +150 us, A2 = -150 us, A3 = +300 us, A4 = -300 us, A5 = +300 us.
// Straight: sign is absolute (+ = roll right, - = roll left).
// Turn: only |A| is used and the roll is ALWAYS opposite to the turn direction
// (never toward the turn).
constexpr int16_t kAmp[5] = {150, -150, 300, -300, 300};

// Scenario 1 (wp_sum 4): L1 = leg WP1->WP2 (flag 2), B2 = claim of WP2 (flag 3).
constexpr Event kS1[5][2] = {
    {{Kind::Straight, 2, 90, 150, 0}, {Kind::Turn, 3, 0, 0, 0}},
    {{Kind::Straight, 2, 90, 150, 1}, {Kind::Turn, 3, 0, 0, 1}},
    {{Kind::Straight, 2, 90, 150, 2}, {Kind::Turn, 3, 0, 0, 2}},
    {{Kind::Straight, 2, 90, 150, 3}, {Kind::Turn, 3, 0, 0, 3}},
    {{Kind::Straight, 2, 90, 150, 4}, {Kind::Turn, 3, 0, 0, 4}},
};

// Scenario 2 (wp_sum 7): L1 = leg WP1->WP2 (flag 2), B3 = claim of WP3 (flag 4),
// B5 = claim of WP5 (flag 6), L5 = leg WP5->WP6 (flag 6).
constexpr Event kS2F1[3] = {{Kind::Straight, 2, 70, 150, 0}, {Kind::Turn, 4, 0, 0, 0}, {Kind::Turn, 6, 0, 0, 1}};
constexpr Event kS2F2[3] = {{Kind::Straight, 2, 70, 150, 1}, {Kind::Turn, 4, 0, 0, 2}, {Kind::Turn, 6, 0, 0, 3}};
constexpr Event kS2F3[3] = {{Kind::Straight, 2, 70, 150, 2}, {Kind::Turn, 4, 0, 0, 4}, {Kind::Straight, 6, 135, 190, 3}};
constexpr Event kS2F4[1] = {{Kind::Straight, 2, 70, 150, 4}};

const Event* eventsFor(int wp_sum, int flight, uint8_t& count)
{
    count = 0;
    if (wp_sum == 4 && flight >= 1 && flight <= 5) {
        count = 2;
        return kS1[flight - 1];
    }
    if (wp_sum == 7) {
        switch (flight) {
        case 1: count = 3; return kS2F1;
        case 2: count = 3; return kS2F2;
        case 3: count = 3; return kS2F3;
        case 4: count = 1; return kS2F4;
        default: break;
        }
    }
    return nullptr;
}

}  // namespace

void DisturbanceScript::reset()
{
    state_ = State::Waiting;
    idx_ = 0;
    offset_us_ = 0;
    prev_flag_ = -1;
    turn_angle_last_ = 0.0f;
    calm_run_ = false;
    calm_ok_ = false;
    last_event_id_ = 0;
}

void DisturbanceScript::advance()
{
    ++idx_;
    state_ = State::Waiting;
    offset_us_ = 0;
}

void DisturbanceScript::skip(uint8_t reason)
{
    ++skip_count_;
    last_skip_id_ = static_cast<uint8_t>(idx_ + 1);
    last_skip_reason_ = reason;
    advance();
}

int16_t DisturbanceScript::update(const DisturbanceScriptInput& in, int flight, uint32_t duration_ms)
{
    // Calm tracking: |e| inside the band continuously for kDistCalmHoldMs.
    const bool in_band = fabsf(in.cte_m) <= kDistCalmBandM;
    if (!in_band) {
        calm_run_ = false;
    } else if (!calm_run_) {
        calm_run_ = true;
        calm_since_ms_ = in.now_ms;
    }
    const bool calm_before = calm_ok_;  // state at the previous sample (before any leg change)
    calm_ok_ = calm_run_ && (in.now_ms - calm_since_ms_ >= kDistCalmHoldMs);

    const bool flag_changed = in.flag_wp != prev_flag_;
    const int prev_flag = prev_flag_;
    const float turn_before = turn_angle_last_;
    prev_flag_ = in.flag_wp;
    turn_angle_last_ = in.turn_angle_deg;

    uint8_t count = 0;
    const Event* events = eventsFor(in.wp_sum, flight, count);
    if (events == nullptr || idx_ >= count) {
        state_ = State::Done;
        offset_us_ = 0;
        return 0;
    }
    const Event& e = events[idx_];

    if (state_ == State::Active) {
        const float limit = active_is_turn_ ? kDistAbortTurnM : kDistAbortStraightM;
        if (fabsf(in.cte_m) > limit) {
            skip(3);
        } else if (in.now_ms - active_since_ms_ >= active_duration_ms_) {
            advance();
        }
        return offset_us_;
    }

    bool fire = false;
    int16_t amplitude = kAmp[e.amp];
    if (e.kind == Kind::Straight) {
        if (in.flag_wp > e.flag || (in.flag_wp == e.flag && in.along_m > static_cast<float>(e.end_m))) {
            skip(in.flag_wp == e.flag ? 1 : 2);
        } else if (in.flag_wp == e.flag && in.along_m >= static_cast<float>(e.pos_m) && calm_ok_) {
            fire = true;
        }
    } else {
        if (in.flag_wp > e.flag) {
            skip(4);
        } else if (flag_changed && in.flag_wp == e.flag) {
            if (prev_flag != e.flag - 1) {
                skip(4);
            } else if (!calm_before) {
                skip(1);
            } else if (fabsf(turn_before) < 1.0f) {
                skip(5);
            } else {
                // Always opposite to the turn: right turn (turn_before > 0) -> roll left (-),
                // left turn -> roll right (+).
                const int16_t magnitude = static_cast<int16_t>(amplitude < 0 ? -amplitude : amplitude);
                amplitude = static_cast<int16_t>(turn_before > 0.0f ? -magnitude : magnitude);
                fire = true;
            }
        }
    }

    if (fire) {
        uint32_t duration = duration_ms;
        if (duration > kDistMaxDurationMs) {
            duration = kDistMaxDurationMs;
        }
        if (duration == 0) {
            skip(2);
            return 0;
        }
        offset_us_ = amplitude;
        active_since_ms_ = in.now_ms;
        active_duration_ms_ = duration;
        active_is_turn_ = (e.kind == Kind::Turn);
        state_ = State::Active;
        last_event_id_ = static_cast<uint8_t>(idx_ + 1);
        ++fire_count_;
    }
    return offset_us_;
}

}  // namespace fc
