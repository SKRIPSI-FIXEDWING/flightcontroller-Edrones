#pragma once

#include <AP_Math.h>
#include <Filter/AverageFilter.h>
#include <Filter/LowPassFilter.h>

#include "drivers/Airspeed.h"
#include "drivers/Barometer.h"
#include "drivers/Imu.h"
#include "estimation/Ahrs.h"

namespace fc {

struct TecsConfig {
    // Throttle limits (percent, 0-100). Defaults match the legacy TECS.h
    // compile-time defaults, which are what actually ran before any
    // Params/EEPROM override (FW_config.h's TECS_MAX_THROTTLE=85 was never
    // referenced by TECS.h — its own 95.0f default is what was live).
    float min_throttle_percent = 60.0f;
    float max_throttle_percent = 95.0f;
    float cruise_throttle_percent = 70.0f;         // TECS_CRUISE_THROTTLE
    float manual_cruise_throttle_percent = 70.0f;  // THROTTLE_CRUISE
    float throttle_slew_rate_percent_per_s = 100.0f;  // THR_SLEW_RATE

    float hgt_comp_filter_omega = 5.0f;
    float spd_comp_filter_omega = 2.0f;
    float max_climb_rate_mps = 5.0f;
    float min_sink_rate_mps = 2.0f;
    float max_sink_rate_mps = 5.0f;
    float time_const_s = 5.0f;
    float pitch_damp = 0.0f;
    float throttle_damp = 0.5f;
    float integrator_gain = 0.1f;
    float vertical_accel_limit_mps2 = 7.0f;
    float roll_compensation_deg = 10.0f;
    float speed_weight = 1.0f;

    // 0 = use pitch_max_limit_deg/pitch_min_limit_deg (ArduPilot TECS_PITCH_MAX/MIN==0 convention).
    int8_t pitch_max_override_deg = 0;
    int8_t pitch_min_override_deg = 0;
    float pitch_max_limit_deg = 14.0f;  // PITCH_MAX
    float pitch_min_limit_deg = -5.0f;  // PITCH_MIN

    float min_airspeed_mps = 14.0f;
    float max_airspeed_mps = 22.0f;
    float cruise_airspeed_mps = 18.0f;
};

/**
 * Total Energy Control System (ArduPilot's AP_TECS), ported to a class with
 * injected sensor snapshots instead of file-scope globals. The control
 * algorithm — including the legacy code's bespoke NaN/Inf self-healing
 * guards and the "CRITICAL FIX" adaptive height-demand time constant — is
 * unchanged; only two unit-scale bugs are fixed (see docs/tecs.md): the NaN/
 * Inf pitch-demand fallback assigned `imu.pitch` (degrees) directly into
 * `_pitch_dem`, a field documented and used everywhere else as radians.
 *
 * This class does not log to Serial internally (unlike legacy TECS_DEBUG) —
 * all internal state needed for debugging is exposed via getters; the
 * telemetry layer decides whether/where to print it.
 *
 * Scheduling stays outside this class: call update50Hz() at 50 Hz and
 * updatePitchThrottle() from the navigation loop, both with the latest
 * sensor snapshots.
 */
class Tecs final {
public:
    explicit Tecs(const TecsConfig& config = TecsConfig{});

    void update50Hz(const AhrsData& ahrs, const BarometerData& baro, const ImuData& imu,
                    const AirspeedData& airspeed);

    void updatePitchThrottle(int32_t hgt_dem_cm, float eas_dem_mps, int16_t throttle_nudge,
                             float hgt_afe, float load_factor, const AhrsData& ahrs,
                             const BarometerData& baro, const ImuData& imu,
                             const AirspeedData& airspeed);

    int32_t throttleDemand() const;
    /** Pitch demand in degrees (legacy's "get_pitch_demand()" comment said
     * centidegrees, but its formula (rad * 180/pi) and every consumer
     * treated the result as plain degrees — see docs/tecs.md). */
    float pitchDemandDeg() const;
    float velocityXDot() const;
    float targetAirspeedMps(float eas2tas) const;
    float maxClimbRateMps() const;
    void resetPitchIntegrator();
    float landSinkRateMps() const;
    float landAirspeedMps() const;
    float heightRateDemandMps() const;
    void setPathProportion(float path_proportion);
    void setPitchMaxLimitDeg(int8_t pitch_limit_deg);
    void useSyntheticAirspeedOnce();

    void setMinThrottlePercent(float value);
    void setMaxThrottlePercent(float value);
    float minThrottlePercent() const;
    float maxThrottlePercent() const;

    bool underspeed() const;
    bool badDescent() const;

private:
    void updateSpeed(float dt, const AhrsData& ahrs, const ImuData& imu,
                     const BarometerData& baro, const AirspeedData& airspeed);
    void updateSpeedDemand();
    void updateHeightDemand();
    void detectUnderspeed();
    void updateEnergies();
    float integratorGain() const;
    void updateThrottleWithAirspeed(const AhrsData& ahrs);
    void updateThrottleWithoutAirspeed(int16_t throttle_nudge, const AhrsData& ahrs);
    void detectBadDescent();
    void updatePitch(const ImuData& imu);
    void initialiseStates(float hgt_afe, const ImuData& imu);
    void updateSteRateLimits();
    float timeConstant() const;

    TecsConfig config_{};

    uint64_t update_50hz_last_usec_ = 0;
    uint64_t update_pitch_throttle_last_usec_ = 0;

    float height_ = 0.0f;
    float throttle_dem_ = 0.0f;
    float pitch_dem_ = 0.0f;
    float climb_rate_ = 0.0f;

    struct HeightFilter {
        float dd_height = 0.0f;
        float height = 0.0f;
    } height_filter_{};

    float integ_dtas_state_ = 0.0f;
    float tas_state_ = 0.0f;
    float integ_thr_state_ = 0.0f;
    float integ_seb_state_ = 0.0f;

    float last_throttle_dem_ = 0.0f;
    float last_pitch_dem_ = 0.0f;
    float vel_dot_ = 0.0f;

    float eas_ = 0.0f;
    float tas_max_ = 0.0f;
    float tas_min_ = 0.0f;
    float tas_dem_ = 0.0f;
    float eas_dem_ = 0.0f;

    float hgt_dem_ = 0.0f;
    float hgt_dem_in_old_ = 0.0f;
    float hgt_dem_adj_ = 0.0f;
    float hgt_dem_adj_last_ = 0.0f;
    float hgt_rate_dem_ = 0.0f;
    float hgt_dem_prev_ = 0.0f;

    float tas_dem_adj_ = 0.0f;
    float tas_rate_dem_ = 0.0f;
    float stedot_err_last_ = 0.0f;

    struct Flags {
        bool underspeed = false;
        bool bad_descent = false;
        bool reset = false;
    } flags_{};

    enum class ClipStatus : int8_t { kMin = -1, kNone = 0, kMax = 1 };
    ClipStatus thr_clip_status_ = ClipStatus::kNone;

    float integ_ke_ = 0.0f;
    float integ_sebdot_ = 0.0f;
    ClipStatus sebdot_dem_clip_ = ClipStatus::kNone;

    uint32_t underspeed_start_ms_ = 0;
    float pitch_dem_unc_ = 0.0f;

    float stedot_max_ = 0.0f;
    float stedot_min_ = 0.0f;
    float thr_maxf_ = 0.0f;
    float thr_minf_ = 0.0f;
    float pitch_maxf_ = 0.0f;
    float pitch_minf_ = 0.0f;

    float spe_dem_ = 0.0f;
    float ske_dem_ = 0.0f;
    float spedot_dem_ = 0.0f;
    float skedot_dem_ = 0.0f;
    float spe_est_ = 0.0f;
    float ske_est_ = 0.0f;
    float spedot_ = 0.0f;
    float skedot_ = 0.0f;

    float ste_error_ = 0.0f;

    float dt_ = 0.0f;

    float ske_weighting_ = 0.0f;

    float tas_rate_dem_lpf_ = 0.0f;
    float vel_dot_lpf_ = 0.0f;

    int8_t pitch_max_limit_ = 90;

    bool use_synthetic_airspeed_ = false;
    bool use_synthetic_airspeed_once_ = false;

    LowPassFilterFloat pitch_demand_lpf_{};
    LowPassFilterFloat pitch_measured_lpf_{};
    AverageFilterFloat_Size5 vdot_filter_{};
};

}  // namespace fc
