#include "navigation/Navigation.h"

#include <math.h>

namespace {
constexpr float kGravityMss = 9.80665f;
}

namespace fc {

Navigation::Navigation(const NavigationConfig& config)
    : config_(config)
{
    mission_.target_airspeed_mps = config_.airspeed_cruise_mps;
}

void Navigation::setWaypointReachedCallback(WaypointReachedCallback callback, void* user_data)
{
    waypoint_reached_callback_ = callback;
    waypoint_reached_user_data_ = user_data;
}

MissionState& Navigation::state()
{
    return mission_;
}

const MissionState& Navigation::state() const
{
    return mission_;
}

void Navigation::calcGroundspeedUndershoot(const AhrsData& ahrs, bool have_gps)
{
    if (have_gps) {
        const float gnd_spd_fwd = ahrs.groundspeed_mps;
        groundspeed_undershoot_mps_ =
            (config_.min_groundspeed_mps > 0.0f) ? (config_.min_groundspeed_mps - gnd_spd_fwd) : 0.0f;
    } else {
        groundspeed_undershoot_mps_ = 0.0f;
    }
}

float Navigation::groundspeedUndershoot() const
{
    return groundspeed_undershoot_mps_;
}

void Navigation::updateHomeAndPosition(Ahrs& ahrs, const GnssFixData& gnss,
                                       const BarometerData& baro, bool armed, uint32_t now_ms)
{
    // Home at arming (ArduPilot style): every disarmed->armed transition
    // re-latches home at the vehicle's current position as soon as a 3D fix
    // is available (immediately if it already is, otherwise on the first 3D
    // fix while armed). After that home stays fixed for the whole flight.
    const bool arm_edge = armed && !was_armed_;
    was_armed_ = armed;
    if (arm_edge) {
        home_pending_on_arm_ = true;
    }
    if (home_pending_on_arm_ && armed && gnss.fix_type == GnssFixType::Fix3D) {
        ahrs.setHome(gnss, baro);
        home_pending_on_arm_ = false;
    }

    // Pre-arm: keep home tracking the vehicle while disarmed so a home exists
    // (and is shown on the GCS) before arming. Never runs while armed.
    if (!armed && now_ms - last_home_set_ms_ >= config_.home_set_interval_ms &&
        baro.altitude_m <= config_.home_altitude_settle_m) {
        last_home_set_ms_ = now_ms;
        ahrs.setHome(gnss, baro);  // Ahrs::setHome() itself no-ops unless gnss.fix_type is Fix3D.
    }

    const AhrsData& ahrs_data = ahrs.data();
    mission_.current_loc = ahrs_data.position;
    // position.alt = baro altitude * 100 (UP-positive, cm). Legacy computed
    // D = -alt and then negated it again for TECS; the port kept only the
    // first negation, so TECS saw height = -altitude. TECS wants up-positive.
    mission_.relative_alt_m = static_cast<float>(ahrs_data.position.alt) / 100.0f;
    mission_.home_set = ahrs.homeIsSet();
    if (mission_.home_set) {
        mission_.home_wp_loc = ahrs.homeLocation();
    }
    ahrs.updateWindspeed();

    calcGroundspeedUndershoot(ahrs_data, ahrs_data.gps_lock);
}

void Navigation::updateWaypointNav(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu)
{
    if (mission_.auto_state.crosstrack) {
        l1.updateWaypoint(ahrs, imu, mission_.prev_wp_loc, mission_.next_wp_loc, 0.0f);
    } else {
        l1.updateWaypoint(ahrs, imu, mission_.current_loc, mission_.next_wp_loc, 0.0f);
    }
}

float Navigation::getNextGroundCourse(float default_angle_deg) const
{
    if (mission_.flag_wp < 0 || mission_.flag_wp + 1 >= mission_.wp_sum) {
        return default_angle_deg;
    }
    const Locations& next_wp_after = mission_.waypoint[mission_.flag_wp + 1];
    if (next_wp_after.lat == 0 && next_wp_after.lng == 0) {
        return default_angle_deg;
    }
    return mission_.next_wp_loc.get_bearing_to(next_wp_after) / 100.0f;
}

void Navigation::setupTurnAngle()
{
    const float next_ground_course = getNextGroundCourse(-1.0f);
    if (next_ground_course == -1.0f) {
        mission_.auto_state.next_turn_angle_deg = 90.0f;
    } else {
        const float ground_course = mission_.prev_wp_loc.get_bearing_to(mission_.next_wp_loc) / 100.0f;
        mission_.auto_state.next_turn_angle_deg = wrap_180(next_ground_course - ground_course);
    }
}

void Navigation::setNextWaypoint(const Locations& loc)
{
    if (mission_.auto_state.next_wp_crosstrack) {
        mission_.prev_wp_loc = mission_.next_wp_loc;
        mission_.auto_state.crosstrack = true;
    } else {
        mission_.prev_wp_loc = mission_.current_loc;
        mission_.auto_state.crosstrack = false;
        mission_.auto_state.next_wp_crosstrack = true;
    }
    mission_.next_wp_loc = loc;
    setupTurnAngle();
}

void Navigation::updateAltitude(Tecs& tecs, const AhrsData& ahrs, const ImuData& imu,
                                const BarometerData& baro, const AirspeedData& airspeed,
                                int16_t throttle_stick_percent, bool enable_throttle_nudge)
{
    const int16_t throttle_nudge = enable_throttle_nudge
                                       ? static_cast<int16_t>(0.5f * throttle_stick_percent)
                                       : 0;

    if (mission_.auto_throttle_mode) {
        constexpr float kAerodynamicLoadFactor = 1.0f;  // load-factor compensation is dormant in legacy too
        tecs.updatePitchThrottle(static_cast<int32_t>(mission_.target_altitude.amsl_cm),
                                 mission_.target_airspeed_mps, throttle_nudge,
                                 mission_.relative_alt_m, kAerodynamicLoadFactor, ahrs, baro, imu,
                                 airspeed);
    }
}

void Navigation::updateSpeedHeight(Tecs& tecs, const AhrsData& ahrs, const BarometerData& baro,
                                   const ImuData& imu, const AirspeedData& airspeed)
{
    if (mission_.auto_throttle_mode) {
        tecs.update50Hz(ahrs, baro, imu, airspeed);
    }
}

float Navigation::getAdaptiveWaypointAirspeed(float dist_to_wp, float acceptance_dist,
                                              float next_turn_angle_deg,
                                              float current_roll_deg) const
{
    float target_spd = config_.airspeed_cruise_mps;

    if (fabsf(next_turn_angle_deg) > config_.adaptive_turn_angle_thresh_deg) {
        const float max_bank_rad = radians(config_.roll_limit_deg);
        const float optimal_turn_spd = sqrtf(config_.wp_radius_default_m * kGravityMss * tanf(max_bank_rad));
        const float safe_turn_spd =
            constrain_float(optimal_turn_spd, config_.airspeed_min_mps, config_.airspeed_cruise_mps);

        const float approach_zone = acceptance_dist * config_.adaptive_approach_mult;
        if (approach_zone > 0.1f && dist_to_wp <= approach_zone) {
            const float blend = constrain_float(dist_to_wp / approach_zone, 0.0f, 1.0f);
            target_spd = safe_turn_spd + blend * (config_.airspeed_cruise_mps - safe_turn_spd);
        }
    }

    if (fabsf(current_roll_deg) > config_.adaptive_wings_level_deg) {
        target_spd = constrain_float(target_spd, config_.airspeed_min_mps, target_spd);
    }

    if (isnan(target_spd) || isinf(target_spd) || target_spd < config_.airspeed_min_mps) {
        target_spd = config_.airspeed_cruise_mps;
    }
    return target_spd;
}

void Navigation::updateAutoAttitudeTargets(L1Controller& l1, Tecs& tecs, const ImuData& imu)
{
    // L1 raw bank demand (positive = waypoint to the right if yaw is clockwise)
    // times nav_roll_sign -> stick convention (positive = bank right).
    const float nav_sign = (config_.nav_roll_sign < 0.0f) ? -1.0f : 1.0f;
    mission_.nav_roll_deg = constrain_float(nav_sign * l1.navRollCd(imu) / 100.0f, -config_.roll_limit_deg,
                                            config_.roll_limit_deg);
    mission_.nav_pitch_deg =
        constrain_float(tecs.pitchDemandDeg(), config_.pitch_limit_min_deg, config_.pitch_limit_max_deg);
}

bool Navigation::beginAuto()
{
    auto_reject_reason_ = nullptr;

    // Mission validity. With reject_invalid_mission == false (default, same as
    // Khageswara) none of these refuse AUTO; they are only checked/refused when
    // the switch is on.
    const char* problem = nullptr;
    if (mission_.wp_sum < 2) {
        problem = "AUTO: no mission (need HOME + >=1 WP)";
    } else if (mission_.current_loc.lat == 0 && mission_.current_loc.lng == 0) {
        problem = "AUTO: no GPS position yet";
    } else if (config_.mission_done_behavior == 2 && !mission_.home_set) {
        // Home is only needed when the mission ends with RTL.
        problem = "AUTO: home not set (needed for RTL)";
    } else {
        const int last = (mission_.wp_sum < MissionState::kMaxWaypoints) ? mission_.wp_sum
                                                                         : MissionState::kMaxWaypoints;
        for (int i = 1; i < last; ++i) {
            if (mission_.waypoint[i].lat == 0 && mission_.waypoint[i].lng == 0) {
                problem = "AUTO: WP at 0,0 is invalid";
                break;
            }
        }
    }
    if (problem != nullptr && config_.reject_invalid_mission) {
        auto_reject_reason_ = problem;
        return false;
    }

    // Misi yang sudah selesai dimulai ulang dari item 1.
    if (mission_.mission_complete) {
        mission_.mission_complete = false;
        mission_.done_returning_home = false;
        mission_.flag_wp = -1;
    }

    mission_.auto_navigation_mode = true;
    mission_.auto_throttle_mode = true;
    mission_.auto_state.next_wp_crosstrack = false;

    if (mission_.flag_wp >= 1 && mission_.flag_wp < mission_.wp_sum) {
        // Lanjutkan dari WP terakhir (mis. pilot sempat pindah ke FBWA).
        mission_.target_altitude.amsl_cm = static_cast<float>(mission_.waypoint[mission_.flag_wp].alt);
        setNextWaypoint(mission_.waypoint[mission_.flag_wp]);
    } else {
        mission_.flag_wp = -1;  // navigate() akan memulai dari item 1
    }
    return true;
}

const char* Navigation::autoRejectReason() const
{
    return auto_reject_reason_;
}

bool Navigation::setCurrentWaypoint(int seq)
{
    if (seq < 1 || seq >= mission_.wp_sum || seq >= MissionState::kMaxWaypoints) {
        return false;
    }
    mission_.mission_complete = false;
    mission_.done_returning_home = false;
    mission_.flag_wp = seq;
    mission_.auto_state.next_wp_crosstrack = false;
    mission_.target_altitude.amsl_cm = static_cast<float>(mission_.waypoint[seq].alt);
    setNextWaypoint(mission_.waypoint[seq]);
    return true;
}

void Navigation::startMissionDone()
{
    mission_.mission_complete = true;
    mission_.target_airspeed_mps = config_.airspeed_cruise_mps;
    mission_.done_start_loc = mission_.current_loc;

    const bool have_last = mission_.wp_sum > 1;
    const Locations last_wp = have_last ? mission_.waypoint[mission_.wp_sum - 1] : mission_.current_loc;
    mission_.flag_wp = (mission_.wp_sum > 0) ? (mission_.wp_sum - 1) : 0;

    if (config_.mission_done_behavior == 0) {
        // Do nothing: stay in AUTO and keep following the last leg (prev/next
        // WP are left as they are, nothing is advanced, no more
        // "waypoint reached" notifications). With no valid leg, hold the
        // current heading instead of tracking toward a (0,0) placeholder.
        mission_.done_returning_home = false;
        if (!have_last) {
            mission_.prev_wp_loc = mission_.current_loc;
            mission_.next_wp_loc = mission_.current_loc;
            mission_.auto_state.next_wp_crosstrack = false;
        }
        return;
    }

    mission_.auto_state.next_wp_crosstrack = false;
    if (config_.mission_done_behavior == 2 && mission_.home_set) {
        // RTL: terbang ke home lalu loiter di atasnya.
        mission_.done_center = mission_.home_wp_loc;
        mission_.done_returning_home = true;
        mission_.target_altitude.amsl_cm = config_.rtl_altitude_m * 100.0f;
    } else {
        // Loiter di WP terakhir pada ketinggian WP tersebut.
        mission_.done_center = last_wp;
        mission_.done_returning_home = false;
        mission_.target_altitude.amsl_cm = static_cast<float>(have_last ? last_wp.alt : mission_.current_loc.alt);
    }
}

void Navigation::runMissionDone(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu, float eas2tas)
{
    if (config_.mission_done_behavior == 0) {
        // Do nothing special: keep tracking the last leg.
        mission_.auto_state.distance_next_wp = mission_.current_loc.get_distance(mission_.next_wp_loc);
        updateWaypointNav(l1, ahrs, imu);
        return;
    }
    runReturnAndLoiter(l1, ahrs, imu, eas2tas);
}

bool Navigation::beginRtl()
{
    auto_reject_reason_ = nullptr;
    if (!mission_.home_set) {
        auto_reject_reason_ = "RTL: home not set";
        return false;
    }
    mission_.done_center = mission_.home_wp_loc;
    mission_.done_returning_home = true;
    mission_.done_start_loc = mission_.current_loc;
    mission_.target_altitude.amsl_cm = config_.rtl_altitude_m * 100.0f;
    mission_.target_airspeed_mps = config_.airspeed_cruise_mps;
    mission_.auto_navigation_mode = false;
    mission_.auto_throttle_mode = true;
    return true;
}

void Navigation::updateRtl(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu, float eas2tas)
{
    runReturnAndLoiter(l1, ahrs, imu, eas2tas);
}

void Navigation::runReturnAndLoiter(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu,
                                    float eas2tas)
{
    const float radius = fmaxf(fabsf(config_.loiter_radius_m), 20.0f);
    const int8_t direction = (config_.loiter_direction >= 0) ? 1 : -1;

    mission_.target_airspeed_mps = config_.airspeed_cruise_mps;
    mission_.next_wp_loc = mission_.done_center;
    mission_.auto_state.distance_next_wp = mission_.current_loc.get_distance(mission_.done_center);
    mission_.auto_state.bearing_deg = mission_.current_loc.get_bearing_to(mission_.done_center) / 100.0f;

    if (mission_.done_returning_home) {
        if (mission_.auto_state.distance_next_wp > radius + config_.rtl_loiter_capture_margin_m) {
            l1.updateWaypoint(ahrs, imu, mission_.done_start_loc, mission_.done_center, 0.0f);
            return;
        }
        mission_.done_returning_home = false;  // sudah di area home -> loiter
    }

    l1.updateLoiter(ahrs, imu, mission_.done_center, radius, direction, eas2tas,
                    mission_.target_airspeed_mps);
}

void Navigation::navigate(bool is_guided_mode, L1Controller& l1, const AhrsData& ahrs,
                          const ImuData& imu, float eas2tas)
{
    if (mission_.rpi_external_setpoint_active) {
        if (millis() - mission_.rpi_external_setpoint_last_ms > config_.rpi_setpoint_timeout_ms) {
            mission_.rpi_external_setpoint_active = false;
        } else {
            mission_.prev_wp_loc = mission_.current_loc;
            mission_.next_wp_loc = mission_.rpi_external_setpoint_target;
            mission_.auto_state.next_wp_crosstrack = false;
            mission_.target_altitude.amsl_cm = static_cast<float>(mission_.rpi_external_setpoint_target.alt);
            mission_.auto_navigation_mode = true;
            updateWaypointNav(l1, ahrs, imu);
            return;
        }
    }

    if (is_guided_mode) {
        mission_.prev_wp_loc = mission_.current_loc;
        mission_.next_wp_loc = mission_.current_loc;
        mission_.auto_state.next_wp_crosstrack = false;
        mission_.target_altitude.amsl_cm = static_cast<float>(mission_.current_loc.alt);
        mission_.auto_state.distance_next_wp = 0.0f;
        mission_.auto_state.bearing_deg = 0.0f;
        mission_.auto_state.wp_proportion = 0.0f;
        updateWaypointNav(l1, ahrs, imu);
        return;
    }

    // Misi sudah selesai: jalankan done behaviour (loiter / RTL), bukan
    // melanjutkan leg terakhir.
    if (mission_.mission_complete) {
        runMissionDone(l1, ahrs, imu, eas2tas);
        return;
    }

    if (mission_.auto_navigation_mode) {
        // Item 0 adalah HOME (seq 0 Mission Planner), bukan target terbang:
        // navigasi selalu mulai dari item 1.
        if (mission_.flag_wp < 1) {
            mission_.flag_wp = 1;
            mission_.auto_state.next_wp_crosstrack = false;
            if (mission_.flag_wp < mission_.wp_sum) {
                mission_.target_altitude.amsl_cm = static_cast<float>(mission_.waypoint[1].alt);
                setNextWaypoint(mission_.waypoint[1]);
            }
        }
        // Misi diperpendek/dihapus dari GCS saat terbang.
        if (mission_.flag_wp >= mission_.wp_sum) {
            startMissionDone();
            runMissionDone(l1, ahrs, imu, eas2tas);
            return;
        }
    }
    // NOTE: legacy's auto_takeoff_mode branch here is intentionally not
    // ported yet — see docs/navigation.md.

    mission_.auto_state.distance_next_wp = mission_.current_loc.get_distance(mission_.next_wp_loc);
    mission_.auto_state.bearing_deg = mission_.current_loc.get_bearing_to(mission_.next_wp_loc) / 100.0f;
    mission_.auto_state.wp_proportion =
        mission_.current_loc.line_path_proportion(mission_.prev_wp_loc, mission_.next_wp_loc);

    if (mission_.auto_navigation_mode) {
        // Fixed acceptance radius: per-WP radius (MAVLink param2) when set,
        // else the default. Deliberately NOT turn_distance(): that depends on
        // the L1 length and would differ between "L1 conventional" and
        // "L1 + fuzzy" runs.
        float effective_radius = config_.wp_radius_default_m;
        if (mission_.flag_wp >= 0 && mission_.flag_wp < mission_.wp_sum &&
            mission_.flag_wp < MissionState::kMaxWaypoints &&
            mission_.waypoint_radius_m[mission_.flag_wp] > 0.0f) {
            effective_radius = mission_.waypoint_radius_m[mission_.flag_wp];
        }
        mission_.acceptance_distance_m = effective_radius;

        mission_.target_airspeed_mps =
            config_.adaptive_waypoint_airspeed_enabled
                ? getAdaptiveWaypointAirspeed(mission_.auto_state.distance_next_wp,
                                              mission_.acceptance_distance_m,
                                              mission_.auto_state.next_turn_angle_deg, imu.roll_deg)
                : config_.airspeed_cruise_mps;
    }

    const bool hit_radius = mission_.auto_state.distance_next_wp <= mission_.acceptance_distance_m;
    const bool finish_line =
        mission_.current_loc.past_interval_finish_line(mission_.prev_wp_loc, mission_.next_wp_loc);
    // Optional wider claim for the last WP (disabled when last_wp_acceptance_m == 0).
    const bool is_last_wp = mission_.flag_wp >= (mission_.wp_sum - 1);
    const bool near_last_wp = config_.last_wp_acceptance_m > 0.0f && is_last_wp &&
                              mission_.auto_state.distance_next_wp <= config_.last_wp_acceptance_m;

    if (hit_radius || finish_line || near_last_wp) {
        if (mission_.auto_navigation_mode) {
            if (waypoint_reached_callback_ != nullptr) {
                waypoint_reached_callback_(static_cast<uint16_t>(mission_.flag_wp), waypoint_reached_user_data_);
            }

            mission_.flag_wp++;
            if (mission_.flag_wp >= mission_.wp_sum) {
                // Semua WP selesai: pindah ke done behaviour (loiter di WP
                // terakhir atau RTL). Mode tetap AUTO.
                startMissionDone();
                runMissionDone(l1, ahrs, imu, eas2tas);
                return;
            } else {
                mission_.auto_state.next_wp_crosstrack = true;
                mission_.target_altitude.amsl_cm = static_cast<float>(mission_.waypoint[mission_.flag_wp].alt);
                setNextWaypoint(mission_.waypoint[mission_.flag_wp]);
            }
        }
    }

    updateWaypointNav(l1, ahrs, imu);
}

}  // namespace fc
