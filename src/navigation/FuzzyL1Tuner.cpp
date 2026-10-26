#include "navigation/FuzzyL1Tuner.h"

#include <math.h>

namespace {

constexpr uint8_t kInputCrosstrackError = 1;
constexpr uint8_t kInputCrosstrackErrorRate = 2;
constexpr uint8_t kOutputPeriodScale = 1;

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
    FuzzySet* de_small = makeSet(config_.de_small);
    FuzzySet* de_medium = makeSet(config_.de_medium);
    FuzzySet* de_large = makeSet(config_.de_large);
    crosstrack_error_rate->addFuzzySet(de_small);
    crosstrack_error_rate->addFuzzySet(de_medium);
    crosstrack_error_rate->addFuzzySet(de_large);
    fuzzy_->addFuzzyInput(crosstrack_error_rate);

    FuzzyOutput* period_scale = new FuzzyOutput(kOutputPeriodScale);
    FuzzySet* gentle = makeSet(config_.scale_gentle);
    FuzzySet* normal = makeSet(config_.scale_normal);
    FuzzySet* aggressive = makeSet(config_.scale_aggressive);
    period_scale->addFuzzySet(gentle);
    period_scale->addFuzzySet(normal);
    period_scale->addFuzzySet(aggressive);
    fuzzy_->addFuzzyOutput(period_scale);

    // Rule table: small tracking error + low error rate -> gentle (large)
    // period; large error and/or fast-growing error -> aggressive (small)
    // period. Starting point per docs/fuzzy-l1-tuner.md, tune from
    // simulation/flight data.
    struct RuleSpec {
        uint8_t id;
        FuzzySet* error_set;
        FuzzySet* rate_set;
        FuzzySet* output_set;
    };

    const RuleSpec rules[] = {
        {1, e_small, de_small, gentle},
        {2, e_small, de_medium, normal},
        {3, e_small, de_large, normal},
        {4, e_medium, de_small, normal},
        {5, e_medium, de_medium, normal},
        {6, e_medium, de_large, aggressive},
        {7, e_large, de_small, aggressive},
        {8, e_large, de_medium, aggressive},
        {9, e_large, de_large, aggressive},
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
        last_scale_ = 0.0f;
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
    last_scale_ = 0.0f;
    crosstrack_error_rate_mps_ = 0.0f;
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

    if (!enabled_ || fuzzy_ == nullptr) {
        // Conventional L1: leave whatever period is already configured
        // (typically the fixed base_period_s) untouched.
        last_applied_period_s_ = l1.period();
        last_scale_ = 0.0f;
        return;
    }

    fuzzy_->setInput(kInputCrosstrackError, abs_error_m);
    fuzzy_->setInput(kInputCrosstrackErrorRate, fabsf(crosstrack_error_rate_mps_));
    fuzzy_->fuzzify();
    const float scale = fuzzy_->defuzzify(kOutputPeriodScale);

    float period = config_.base_period_s * scale;
    period = constrain_float(period, config_.min_period_s, config_.max_period_s);

    l1.setPeriod(period);
    last_applied_period_s_ = period;
    last_scale_ = scale;
}

float FuzzyL1Tuner::lastAppliedPeriod() const
{
    return last_applied_period_s_;
}

float FuzzyL1Tuner::lastCrosstrackErrorRateMps() const
{
    return crosstrack_error_rate_mps_;
}

float FuzzyL1Tuner::lastScale() const
{
    return last_scale_;
}

const FuzzyL1TunerConfig& FuzzyL1Tuner::config() const
{
    return config_;
}

}  // namespace fc
