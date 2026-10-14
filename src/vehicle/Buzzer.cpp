#include "vehicle/Buzzer.h"

namespace {

// "What If I Call" (Alex Crichton), right-hand melody of the Atlantic Notes
// easy arrangement, key of D, 105 BPM, bars 1-8. Written pitch is D4-A4; it is
// transposed up two octaves because a piezo buzzer is far louder there.
// Ties are merged into one note; the final hold is shortened for boot time.
constexpr int kNoteA3 = 880;    // A3 -> A5
constexpr int kNoteD4 = 1175;   // D4 -> D6
constexpr int kNoteE4 = 1319;   // E4 -> E6
constexpr int kNoteFs4 = 1480;  // F#4 -> F#6
constexpr int kNoteG4 = 1568;   // G4 -> G6
constexpr int kNoteA4 = 1760;   // A4 -> A6
constexpr int kRest = 0;

// Duration unit = one sixteenth note (143 ms at 105 BPM).
constexpr unsigned long kUnitMs = 143;

const int kStartupNotes[] = {
    // bar 1-2: "So what if I call?"
    kNoteD4, kNoteA4, kNoteA4, kNoteA4, kNoteD4,
    // bar 3-4: "And you pick up the phone?"
    kRest, kNoteD4, kNoteD4, kNoteA4, kNoteA4, kNoteA4, kNoteD4,
    // bar 5-6: "And I use this holiday"
    kRest, kNoteD4, kNoteFs4, kNoteFs4, kNoteFs4, kNoteG4,
    kNoteFs4, kNoteD4, kNoteA3, kNoteG4,
    // bar 7-8: "To make my way to your ghost"
    kNoteFs4, kNoteD4, kNoteA3, kNoteFs4, kNoteE4
};
const uint8_t kStartupDurationsUnits[] = {
    2, 1, 1, 4, 18,
    6, 1, 1, 2, 2, 2, 18,
    6, 2, 2, 2, 2, 6,
    2, 6, 2, 4,
    4, 4, 2, 2, 12
};
constexpr uint8_t kStartupNoteCount = sizeof(kStartupNotes) / sizeof(kStartupNotes[0]);
static_assert(sizeof(kStartupDurationsUnits) / sizeof(kStartupDurationsUnits[0]) == kStartupNoteCount,
              "note/duration tables must match");
constexpr unsigned long kNoteGapMs = 30;  // silence at the end of each note

}  // namespace

namespace fc {

Buzzer::Buzzer(const BuzzerConfig& config)
    : config_(config)
{
}

void Buzzer::begin()
{
    if (initialized_) {
        return;
    }
    pinMode(config_.pin, OUTPUT);
    initialized_ = true;

    playing_startup_tune_ = true;
    current_note_index_ = 0;
    last_note_time_ms_ = millis();
}

void Buzzer::updateStartupTune()
{
    const unsigned long now_ms = millis();

    // Each note is held until the previous note's duration has elapsed.
    const unsigned long prev_duration_ms =
        current_note_index_ == 0
            ? 0
            : kStartupDurationsUnits[current_note_index_ - 1] * kUnitMs;
    if (now_ms - last_note_time_ms_ < prev_duration_ms) {
        return;
    }

    if (current_note_index_ < kStartupNoteCount) {
        const int note = kStartupNotes[current_note_index_];
        const unsigned long duration_ms = kStartupDurationsUnits[current_note_index_] * kUnitMs;
        if (note == kRest) {
            noTone(config_.pin);
        } else {
            tone(config_.pin, note, duration_ms > kNoteGapMs ? duration_ms - kNoteGapMs : duration_ms);
        }
        last_note_time_ms_ = now_ms;
        current_note_index_++;
    } else {
        noTone(config_.pin);
        playing_startup_tune_ = false;
        in_post_tune_transition_ = true;
        transition_start_ms_ = now_ms;
        is_buzzing_ = false;
        waiting_long_pause_ = false;
        beep_count_ = 0;
        beep_timer_ms_ = now_ms;
        long_pause_start_ms_ = now_ms;
    }
}

void Buzzer::updateDisarmedBeepPattern()
{
    const unsigned long now_ms = millis();

    if (waiting_long_pause_) {
        if (now_ms - long_pause_start_ms_ >= config_.long_pause_ms) {
            waiting_long_pause_ = false;
            beep_count_ = 0;
        }
        return;
    }

    if (!is_buzzing_) {
        if (now_ms - beep_timer_ms_ >= config_.beep_duration_ms) {
            tone(config_.pin, config_.beep_frequency_hz, config_.beep_duration_ms);
            beep_timer_ms_ = now_ms;
            beep_count_++;
            is_buzzing_ = true;

            if (beep_count_ >= config_.beeps_before_pause) {
                waiting_long_pause_ = true;
                long_pause_start_ms_ = now_ms;
            }
        }
    } else {
        if (now_ms - beep_timer_ms_ >= config_.beep_duration_ms) {
            noTone(config_.pin);
            is_buzzing_ = false;
        }
    }
}

void Buzzer::playPattern(uint8_t count, uint16_t beep_ms, uint16_t gap_ms, uint16_t frequency_hz)
{
    if (!initialized_) {
        return;
    }
    for (uint8_t i = 0; i < count; ++i) {
        tone(config_.pin, frequency_hz, beep_ms);
        delay(beep_ms + gap_ms);
    }
    noTone(config_.pin);
}

void Buzzer::update(bool armed)
{
    if (playing_startup_tune_) {
        updateStartupTune();
        return;
    }

    if (in_post_tune_transition_) {
        if (millis() - transition_start_ms_ >= config_.startup_tune_transition_ms) {
            in_post_tune_transition_ = false;
        }
        return;
    }

    if (!armed) {
        updateDisarmedBeepPattern();
    } else {
        noTone(config_.pin);
        is_buzzing_ = false;
        beep_count_ = 0;
        waiting_long_pause_ = false;
    }
}

}  // namespace fc
