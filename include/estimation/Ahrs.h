#pragma once

#include <AP_Math.h>
#include <Locations.h>

#include "drivers/Airspeed.h"
#include "drivers/Barometer.h"
#include "drivers/Imu.h"
#include "navigation/GnssFixData.h"

namespace fc {

/**
 * One coherent AHRS snapshot: attitude-derived rotation/acceleration frames,
 * drift-corrected groundspeed/velocity, wind estimate, and the navigation
 * position consumed by L1/TECS.
 */
struct AhrsData {
    Matrix3f rotation_body_to_ned{};
    Vector3f acceleration_earth_frame_mss{};
    Vector3f acceleration_body_frame_mss{};

    bool gps_lock = false;
    float groundspeed_mps = 0.0f;
    Vector3f velocity_ned_mps{};

    bool airspeed_sensor_enabled = false;
    float estimated_airspeed_mps = 0.0f;
    Vector3f windspeed_mps{};
    float windspeed_horizontal_mps = 0.0f;

    /**
     * Navigation position: raw GNSS lat/lng + barometer altitude, matching
     * the legacy AHRS::get_position() behavior — see docs/ahrs.md for why
     * this is GPS-direct rather than a filtered/EKF position.
     */
    Locations position{};
    bool home_set = false;

    uint32_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

/**
 * Fixed-wing AHRS: attitude/acceleration frames from fc::Imu, drift-corrected
 * groundspeed/velocity from fc::GnssFixData (GPS-direct when fresh, dead-
 * reckoned fallback otherwise), windspeed estimate, and the GPS+baro position
 * consumed by navigation.
 *
 * This does NOT run a position/velocity Kalman filter. The legacy AHRS.h
 * instantiated one (NavEKF/TinyEKF) but never actually used its output —
 * AHRS::get_position() called the EKF getter and then unconditionally
 * overwrote the result with raw GPS before returning, and the EKF's
 * get_vel_ned() had no caller anywhere in the codebase. That EKF is
 * intentionally not ported; see docs/ahrs.md.
 *
 * Scheduling stays outside this class: call update() once per control loop
 * with the latest sensor snapshots, then distribute data() to
 * navigation/control/telemetry code. updateWindspeed() is called separately
 * (matching the legacy call graph, where estimate_windspeed() was invoked
 * from Navigation's nav_gps(), not from update_ahrs()).
 */
class Ahrs final {
public:
    Ahrs();

    /** Latch the navigation origin once a good 3D GPS fix is available. */
    void setHome(const GnssFixData& gnss, const BarometerData& baro);

    /** DCM, acceleration frames, drift-corrected velocity, and position. */
    void update(const ImuData& imu, const GnssFixData& gnss, const BarometerData& baro,
                const AirspeedData& airspeed);

    /** Wind estimate from fuselage-direction vs. velocity-vector drift. */
    void updateWindspeed();

    AhrsData data() const;

    bool haveGps(const GnssFixData& gnss) const;
    bool homeIsSet() const;

private:
    static Matrix3f rotationFromEuler(float roll_rad, float pitch_rad, float yaw_rad);
    static bool gpsFixIsFresh(const GnssFixData& gnss);

    void updateDriftCorrectedVelocity(const ImuData& imu, const GnssFixData& gnss,
                                       const BarometerData& baro, const AirspeedData& airspeed);

    Locations home_{};
    bool home_set_ = false;

    // Shared between updateDriftCorrectedVelocity()'s dead-reckoning fallback
    // and updateWindspeed(), matching the legacy AHRS::last_velocity member
    // (both functions read/write the same "last known velocity" state).
    Vector3f last_velocity_{};
    Vector3f last_fuselage_direction_{};
    uint32_t last_wind_update_ms_ = 0;

    float low_pass_speed_mps_ = 0.0f;
    float high_pass_speed_mps_ = 0.0f;
    float last_groundspeed_mps_ = 0.0f;

    AhrsData data_{};
};

}  // namespace fc
