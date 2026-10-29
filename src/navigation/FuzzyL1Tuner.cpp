#include "navigation/FuzzyL1Tuner.h"

#include <math.h>

namespace {

constexpr uint8_t kInputCrosstrackError = 1;
constexpr uint8_t kInputCrosstrackErrorRate = 2;
constexpr uint8_t kOutputDeltaPeriod = 1;

FuzzySet* makeSet(const fc::FuzzyTrapezoid& t)
{
    return new FuzzySet(t.a, t.b, t.c, t.d);
}

}  // namespace

namespace fc {

FuzzyL1Tuner::FuzzyL1Tuner(const FuzzyL1TunerConfig& config)
    : config_(config), last_applied_period_s_(config.base_period_s)
{
}

FuzzyL1Tuner::~FuzzyL1Tuner()
{
    delete fuzzy_;
}

void FuzzyL1Tuner::begin()
{
    if (fuzzy_ != nullptr) {
        return;
    }
    fuzzy_ = new Fuzzy();
    buildRuleBase();
}

void FuzzyL1Tuner::buildRuleBase()
{
    FuzzyInput* crosstrack_error = new FuzzyInput(kInputCrosstrackError);
    FuzzySet* e_small = makeSet(config_.e_small);
    FuzzySet* e_medium = makeSet(config_.e_medium);
    FuzzySet* e_large = makeSet(config_.e_large);
    crosstrack_error->addFuzzySet(e_small);
    crosstrack_error->addFuzzySet(e_medium);
    crosstrack_error->addFuzzySet(e_large);
    fuzzy_->addFuzzyInput(crosstrack_error);

    FuzzyInput* crosstrack_error_rate = new FuzzyInput(kInputCrosstrackErrorRate);
    FuzzySet* de_negative = makeSet(config_.de_negative);
    FuzzySet* de_zero = makeSet(config_.de_zero);
    FuzzySet* de_positive = makeSet(config_.de_positive);
    crosstrack_error_rate->addFuzzySet(de_negative);
    crosstrack_error_rate->addFuzzySet(de_zero);
    crosstrack_error_rate->addFuzzySet(de_positive);
    fuzzy_->addFuzzyInput(crosstrack_error_rate);

    FuzzyOutput* delta_period = new FuzzyOutput(kOutputDeltaPeriod);
    FuzzySet* down = makeSet(config_.out_down);
    FuzzySet* hold = makeSet(config_.out_hold);
    FuzzySet* up = makeSet(config_.out_up);
    delta_period->addFuzzySet(down);
    delta_period->addFuzzySet(hold);
    delta_period->addFuzzySet(up);
    fuzzy_->addFuzzyOutput(delta_period);

    // Rule table = proposal Tabel 3.6. Large error, or error still growing
    // away from the path -> Turun (smaller period, more aggressive); small
    // error already closing on the path -> Naik (larger period, avoid
    // overshooting across it).
    struct RuleSpec {
        uint8_t id;
        FuzzySet* error_set;
        FuzzySet* rate_set;
        FuzzySet* output_set;
    };

    const RuleSpec rules[] = {
        {1, e_small, de_negative, up},
        {2, e_medium, de_negative, hold},
        {3, e_large, de_negative, down},
        {4, e_small, de_zero, hold},
        {5, e_medium, de_zero, hold},
        {6, e_large, de_zero, down},
        {7, e_small, de_positive, hold},
        {8, e_medium, de_positive, down},
        {9, e_large, de_positive, down},
    };

    for (const RuleSpec& rule : rules) {
        FuzzyRuleAntecedent* antecedent = new FuzzyRuleAntecedent();
        antecedent->joinWithAND(rule.error_set, rule.rate_set);
        FuzzyRuleConsequent* consequent = new FuzzyRuleConsequent();
        consequent->addOutput(rule.output_set);
        fuzzy_->addFuzzyRule(new FuzzyRule(rule.id, antecedent, consequent));
    }
}

void FuzzyL1Tuner::setEnabled(bool enabled)
{
    enabled_ = enabled;
}

bool FuzzyL1Tuner::enabled() const
{
    return enabled_;
}

void FuzzyL1Tuner::applyEnabled(bool enabled, L1Controller& l1)
{
    enabled_ = enabled;
    if (!enabled) {
        l1.setPeriod(config_.base_period_s);  // conventional L1 = fixed base period
        last_applied_period_s_ = config_.base_period_s;
        last_delta_period_s_ = 0.0f;
    }
}

void FuzzyL1Tuner::setBasePeriod(float period_s)
{
    config_.base_period_s = period_s;
}

void FuzzyL1Tuner::setPeriodLimits(float min_period_s, float max_period_s)
{
    if (min_period_s > 0.0f && max_period_s >= min_period_s) {
        config_.min_period_s = min_period_s;
        config_.max_period_s = max_period_s;
    }
}

void FuzzyL1Tuner::resetToBase(L1Controller& l1)
{
    l1.setPeriod(config_.base_period_s);
    last_applied_period_s_ = config_.base_period_s;
    last_delta_period_s_ = 0.0f;
    crosstrack_error_rate_mps_ = 0.0f;
}

float FuzzyL1Tuner::evaluate(float abs_error_m, float error_rate_mps)
{
    if (fuzzy_ == nullptr) {
        return 0.0f;
    }
    // Keep the inputs inside the outermost breakpoints: the shoulder sets
    // saturate there anyway, and eFLL never sees an out-of-universe value.
    abs_error_m = constrain_float(abs_error_m, config_.e_small.a, config_.e_large.d);
    error_rate_mps = constrain_float(error_rate_mps, config_.de_negative.a, config_.de_positive.d);
    fuzzy_->setInput(kInputCrosstrackError, abs_error_m);
    fuzzy_->setInput(kInputCrosstrackErrorRate, error_rate_mps);
    fuzzy_->fuzzify();
    return fuzzy_->defuzzify(kOutputDeltaPeriod);
}

void FuzzyL1Tuner::update(L1Controller& l1, float crosstrack_error_m, float /*dt_s*/)
{
    const float abs_error_m = fabsf(crosstrack_error_m);

    // d|e|/dt = sign(e) * d(e)/dt, with d(e)/dt = cross-track velocity (GPS).
    float rate = l1.crosstrackVelocityMps();
    if (!isfinite(rate)) {
        rate = 0.0f;
    }
    crosstrack_error_rate_mps_ = (crosstrack_error_m < 0.0f) ? -rate : rate;
    if (!l1.hasTrack()) {
        crosstrack_error_rate_mps_ = 0.0f;  // no line: see below
    }

    if (!enabled_ || fuzzy_ == nullptr) {
        // Conventional L1: leave whatever period is already configured
        // (typically the fixed base_period_s) untouched.
        last_applied_period_s_ = l1.period();
        last_delta_period_s_ = 0.0f;
        return;
    }

    // No path to track (direct-to-waypoint leg): e is 0 by construction and
    // the "rate" is just the sideways velocity relative to the bearing, which
    // the rules read as Kecil/Negatif -> Naik (flight 2026-10-29: period 16 s
    // for the first ~9 s of AUTO). Hold the base period instead.
    if (!l1.hasTrack()) {
        l1.setPeriod(config_.base_period_s);
        last_applied_period_s_ = config_.base_period_s;
        last_delta_period_s_ = 0.0f;
        return;
    }

    const float delta_period_s = evaluate(abs_error_m, crosstrack_error_rate_mps_);
    const float period =
        constrain_float(config_.base_period_s + delta_period_s, config_.min_period_s, config_.max_period_s);

    l1.setPeriod(period);
    last_applied_period_s_ = period;
    last_delta_period_s_ = delta_period_s;
}

float FuzzyL1Tuner::lastAppliedPeriod() const
{
    return last_applied_period_s_;
}

float FuzzyL1Tuner::lastCrosstrackErrorRateMps() const
{
    return crosstrack_error_rate_mps_;
}

float FuzzyL1Tuner::lastDeltaPeriod() const
{
    return last_delta_period_s_;
}

const FuzzyL1TunerConfig& FuzzyL1Tuner::config() const
{
    return config_;
}

}  // namespace fc
