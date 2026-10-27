#pragma once

#include <Fuzzy.h>
#include <stdint.h>

#include "navigation/L1Controller.h"

namespace fc {

/**
 * Trapezoidal membership function breakpoints (a <= b <= c <= d), matching
 * eFLL's `FuzzySet(a, b, c, d)` convention.
 */
struct FuzzyTrapezoid {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;
};

struct FuzzyL1TunerConfig {
    // Baseline L1 period the tuner offsets. main.cpp sets it from NAVL1_PERIOD
    // before construction and the GCS param handler keeps it equal to
    // NAVL1_PERIOD afterwards (setBasePeriod()), so "Tetap" (dT = 0) is the
    // same period the fixed-period "conventional L1" run uses.
    float base_period_s = 14.0f;
    // Period clamp (params FUZZY_MIN_PER / FUZZY_MAX_PER). 12-16 s = the range
    // flown on 2026-10-26 (NAVL1_PERIOD sweep).
    float min_period_s = 12.0f;
    float max_period_s = 16.0f;

    // Membership functions derived from the 2026-10-26 flights (NAVL1_PERIOD
    // 12-16 s) by tools/flight_analysis/fuzzy_mf_from_data.py; see
    // docs/fuzzy-l1-tuner.md for how each breakpoint was chosen.
    //
    // Input 1: |e|, crosstrack error magnitude, meters (Kecil/Sedang/Besar).
    // Sedang peaks at the 10 m tolerance band.
    FuzzyTrapezoid e_small{0.0f, 0.0f, 3.0f, 10.0f};
    FuzzyTrapezoid e_medium{3.0f, 10.0f, 10.0f, 30.0f};
    FuzzyTrapezoid e_large{10.0f, 30.0f, 60.0f, 60.0f};

    // Input 2: d|e|/dt, meters/second, SIGNED (Negatif = closing on the path,
    // Positif = moving away).
    FuzzyTrapezoid de_negative{-4.5f, -4.5f, -2.25f, -1.25f};
    FuzzyTrapezoid de_zero{-2.25f, -1.25f, 1.25f, 2.25f};
    FuzzyTrapezoid de_positive{1.25f, 2.25f, 4.5f, 4.5f};

    // Output: dT, seconds added to base_period_s (Turun/Tetap/Naik). Smaller
    // period = more aggressive L1 tracking (shorter L1 distance, tighter
    // turns); larger period = gentler tracking.
    FuzzyTrapezoid out_down{-4.0f, -2.0f, -2.0f, 0.0f};
    FuzzyTrapezoid out_hold{-2.0f, 0.0f, 0.0f, 2.0f};
    FuzzyTrapezoid out_up{0.0f, 2.0f, 2.0f, 4.0f};
};

/**
 * Fuzzy self-tuner for L1Controller's period, the thesis's core contribution:
 * a 2-input (|e|, signed d|e|/dt) x 3-set eFLL fuzzy system with the 3x3
 * rule base of proposal Tabel 3.6, whose output dT offsets
 * L1Controller::period() at runtime, versus the fixed-period
 * "conventional L1" baseline.
 *
 * Repurposes the same eFLL pattern as the legacy (dormant) fuzzy roll/pitch
 * gain scheduler in Fuzzy_FW_Roll.h/Fuzzy_FW_Pitch.h — same library API, new
 * inputs/output targeting L1 guidance instead of attitude PID gain.
 *
 * setEnabled(false) freezes the tuner and leaves L1Controller's period
 * untouched, making "L1 conventional" vs. "L1 + fuzzy" a runtime A/B toggle
 * rather than two separate firmware builds — this is the thesis's
 * comparison axis.
 */
class FuzzyL1Tuner final {
public:
    explicit FuzzyL1Tuner(const FuzzyL1TunerConfig& config = FuzzyL1TunerConfig{});
    ~FuzzyL1Tuner();

    FuzzyL1Tuner(const FuzzyL1Tuner&) = delete;
    FuzzyL1Tuner& operator=(const FuzzyL1Tuner&) = delete;

    /** Builds the fuzzy rule base. Call once during initialization. */
    void begin();

    void setEnabled(bool enabled);
    bool enabled() const;

    /**
     * Live enable/disable (param FUZZY_ENABLE). Disabling hands the period
     * back to the fixed base period immediately, so the "conventional L1"
     * run never starts from whatever period the fuzzy tuner left behind.
     */
    void applyEnabled(bool enabled, L1Controller& l1);

    /** Live NAVL1_PERIOD / FUZZY_MIN_PER / FUZZY_MAX_PER (param handler). */
    void setBasePeriod(float period_s);
    void setPeriodLimits(float min_period_s, float max_period_s);

    /** Back to the base period with a clean state; called on AUTO/GUIDED entry. */
    void resetToBase(L1Controller& l1);

    /** Pure inference (no L1): |e| [m], signed d|e|/dt [m/s] -> dT [s]. 0 before begin(). */
    float evaluate(float abs_error_m, float error_rate_mps);

    /**
     * Fuzzifies (|e|, d|e|/dt) -> dT and applies base_period_s + dT
     * (clamped to [min,max]) to `l1` via setPeriod(). When disabled, l1's
     * period is left untouched.
     *
     * d|e|/dt is NOT differenced here any more: L1Controller already holds the
     * cross-track velocity from the GPS velocity vector (= d(cte)/dt), so
     * d|e|/dt = sign(cte) * that. The old per-cycle |e| difference ran at the
     * 200 Hz control rate on a 5 Hz GPS position -- zero most cycles, then a
     * hundreds-of-m/s spike when a new position arrived.
     */
    void update(L1Controller& l1, float crosstrack_error_m, float dt_s);

    float lastAppliedPeriod() const;
    float lastCrosstrackErrorRateMps() const;
    /** Hasil defuzzifikasi (dT periode, detik) siklus terakhir; 0 bila tuner nonaktif. */
    float lastDeltaPeriod() const;
    /** Config yang benar-benar dipakai (disalin saat konstruksi; param FUZZY_* baru berlaku setelah reboot). */
    const FuzzyL1TunerConfig& config() const;

private:
    void buildRuleBase();

    FuzzyL1TunerConfig config_{};
    Fuzzy* fuzzy_ = nullptr;
    bool enabled_ = true;

    float crosstrack_error_rate_mps_ = 0.0f;
    float last_applied_period_s_ = 14.0f;
    float last_delta_period_s_ = 0.0f;
};

}  // namespace fc
