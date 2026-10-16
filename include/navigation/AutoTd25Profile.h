#pragma once
#include <math.h>
#include "navigation/Tecs.h"

namespace fc {
// AUTO-only outer-loop profile from fcuy25-wahanaTDputih(1).zip.
// Keep the skripsi sensor/servo conventions and inner attitude controller.
struct AutoTd25Profile {
    static constexpr float l1_period_s = 22.0f;
    static constexpr float l1_damping = 0.73f;
    static constexpr float cruise_airspeed_mps = 12.0f;
    static float pitchTarget(float demand_deg) {
        if (!isfinite(demand_deg) || demand_deg < -20.0f) return 0.0f;
        return constrain_float(demand_deg, -40.0f, 40.0f);
    }
    static TecsConfig tecsConfig() {
        TecsConfig cfg;
        cfg.min_throttle_percent = 30.0f;
        cfg.max_throttle_percent = 85.0f;
        cfg.cruise_throttle_percent = 30.0f;
        cfg.manual_cruise_throttle_percent = 25.0f;
        cfg.pitch_min_limit_deg = -40.0f;
        cfg.pitch_max_limit_deg = 40.0f;
        cfg.min_airspeed_mps = 9.0f;
        cfg.cruise_airspeed_mps = cruise_airspeed_mps;
        cfg.max_airspeed_mps = 22.0f;
        return cfg;
    }
};
} // namespace fc
