#include "vehicle/Buzzer.h"

namespace {

// Mario Bros theme, matching legacy buzzer.h's initWithMusic().
constexpr int kNoteE7 = 2637;
constexpr int kNoteC7 = 2093;
constexpr int kNoteG7 = 3136;
constexpr int kNoteG6 = 1568;

const int kStartupNotes[] = {kNoteE7, kNoteE7, kNoteE7, kNoteC7, kNoteE7, kNoteG7, kNoteG6, kNoteG6};
const int kStartupDurationsMs[] = {175, 175, 175, 175, 175, 350, 350, 350};
constexpr uint8_t kStartupNoteCount = sizeof(kStartupNotes) / sizeof(kStartupNotes[0]);
constexpr float kStaccatoFactor = 0.7f;

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

    if (current_note_index_ < kStartupNoteCount) {
        if (now_ms - last_note_time_ms_ >= static_cast<unsigned long>(kStartupDurationsMs[current_note_index_])) {
            const int play_duration_ms =
                static_cast<int>(kStartupDurationsMs[current_note_index_] * kStaccatoFactor);
            tone(config_.pin, kStartupNotes[current_note_index_], play_duration_ms);
            last_note_time_ms_ = now_ms;
            current_note_index_++;
        }
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
