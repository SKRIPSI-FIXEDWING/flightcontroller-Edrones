#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace fc {

struct BuzzerConfig {
    uint8_t pin = 35;
    uint16_t beep_frequency_hz = 4000;
    uint16_t beep_duration_ms = 100;
    unsigned long long_pause_ms = 2000;
    uint8_t beeps_before_pause = 3;
    unsigned long startup_tune_transition_ms = 2000;
};

/**
 * Arming-state beep pattern + startup tune (legacy buzzer.h). Only the
 * methods actually called from the legacy buzzerTask are ported
 * (init()/beep()/soundBuzzer()/stop()/playHappyBirthday()/playNote() were
 * dead code -- playHappyBirthday() didn't even have a definition in the
 * legacy source, just a declaration).
 */
class Buzzer final {
public:
    explicit Buzzer(const BuzzerConfig& config = BuzzerConfig{});

    void begin();

    /** Call every loop: plays the startup tune once, then a 3-beep-then-pause
     * pattern while disarmed, silent while armed. */
    void update(bool armed);

private:
    void updateStartupTune();
    void updateDisarmedBeepPattern();

    BuzzerConfig config_{};
    bool initialized_ = false;

    bool playing_startup_tune_ = false;
    uint8_t current_note_index_ = 0;
    unsigned long last_note_time_ms_ = 0;

    bool in_post_tune_transition_ = false;
    unsigned long transition_start_ms_ = 0;

    bool is_buzzing_ = false;
    unsigned long beep_timer_ms_ = 0;
    uint8_t beep_count_ = 0;
    bool waiting_long_pause_ = false;
    unsigned long long_pause_start_ms_ = 0;
};

}  // namespace fc
