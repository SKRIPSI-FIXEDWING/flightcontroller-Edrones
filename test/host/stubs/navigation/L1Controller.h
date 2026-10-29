#pragma once
// Host-test stand-in for the real L1Controller (which pulls in Arduino/AP_Math):
// only what FuzzyL1Tuner touches. Put test/host/stubs BEFORE include on the
// -I path so FuzzyL1Tuner.h picks this one up.

#include <cmath>

// Same as lib/math/AP_Math.h constrain_value(): NaN -> low.
static inline float constrain_float(float v, float lo, float hi)
{
    if (std::isnan(v)) {
        return lo;
    }
    return v < lo ? lo : (v > hi ? hi : v);
}

namespace fc {

class L1Controller {
public:
    void setPeriod(float period_s) { period_s_ = period_s; }
    float period() const { return period_s_; }
    float crosstrackVelocityMps() const { return xtrack_velocity_mps_; }
    bool hasTrack() const { return has_track_; }

    float period_s_ = 14.0f;
    float xtrack_velocity_mps_ = 0.0f;  // set by the test = d(cte)/dt
    bool has_track_ = true;             // false = direct-to-waypoint leg
};

}  // namespace fc
