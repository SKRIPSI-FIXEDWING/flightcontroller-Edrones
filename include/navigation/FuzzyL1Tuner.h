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
    // Baseline L1 period this tuner scales. main.cpp sets it from NAVL1_PERIOD
    // before construction and the GCS param handler keeps it equal to
    // NAVL1_PERIOD afterwards (setBasePeriod()), so the "Normal" set sits on
    // the same period the fixed-period "conventional L1" run uses.
    float base_period_s = 22.0f;
    float min_period_s = 10.0f;
    float max_period_s = 30.0f;

    // Crosstrack error `e` membership functions, meters. Starting points —
    // refine against simulation/flight-log crosstrack error magnitudes
    // before trusting in flight; see docs/fuzzy-l1-tuner.md.
    FuzzyTrapezoid e_small{0.0f, 5.0f, 5.0f, 15.0f};
    FuzzyTrapezoid e_medium{5.0f, 15.0f, 15.0f, 30.0f};
    FuzzyTrapezoid e_large{15.0f, 30.0f, 30.0f, 60.0f};

    // Crosstrack error rate `de/dt` membership functions, meters/second.
    FuzzyTrapezoid de_small{0.0f, 1.0f, 1.0f, 3.0f};
    FuzzyTrapezoid de_medium{1.0f, 3.0f, 3.0f, 6.0f};
    FuzzyTrapezoid de_large{3.0f, 6.0f, 6.0f, 12.0f};

    // Output: multiplier applied to base_period_s. Smaller period = more
    // aggressive L1 tracking (shorter L1 distance, tighter turns); larger
    // period = gentler tracking.
    FuzzyTrapezoid scale_aggressive{0.6f, 0.7f, 0.7f, 0.85f};
    FuzzyTrapezoid scale_normal{0.75f, 1.0f, 1.0f, 1.25f};
    FuzzyTrapezoid scale_gentle{1.15f, 1.3f, 1.3f, 1.5f};
};

/**
 * Fuzzy self-tuner for L1Controller's period, the thesis's core contribution:
 * a 2-input (crosstrack error `e`, its rate `de/dt`) x 3-set eFLL fuzzy
 * system that scales L1Controller::period() at runtime, versus the fixed-
 * period "conventional L1" baseline.
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

    /**
     * Fuzzifies (|e|, d|e|/dt) -> period scale and applies base_period_s *
     * scale (clamped to [min,max]) to `l1` via setPeriod(). When disabled,
     * l1's period is left untouched.
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
    /** Hasil defuzzifikasi (pengali periode) siklus terakhir; 0 bila tuner nonaktif. */
    float lastScale() const;
    /** Config yang benar-benar dipakai (disalin saat konstruksi; param FUZZY_* baru berlaku setelah reboot). */
    const FuzzyL1TunerConfig& config() const;

private:
    void buildRuleBase();

    FuzzyL1TunerConfig config_{};
    Fuzzy* fuzzy_ = nullptr;
    bool enabled_ = true;

    float crosstrack_error_rate_mps_ = 0.0f;
    float last_applied_period_s_ = 22.0f;
    float last_scale_ = 0.0f;
};

}  // namespace fc
