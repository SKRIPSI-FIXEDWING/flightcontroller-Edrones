#pragma once

#include <Locations.h>
#include <stdint.h>

namespace fc {

/**
 * Consolidated auto-flight/mission state, replacing the loose file-scope
 * globals in legacy Auto_setup.h (auto_navigation_mode, flag_wp, waypoint[],
 * auto_state, target_altitude, rpi_external_setpoint_*, etc.) with one
 * struct owned by Navigation.
 *
 * Takeoff state (legacy `auto_takeoff_state`) is intentionally NOT included
 * yet — takeoff.h is tightly coupled to Actuator/Mode, neither of which is
 * ported yet (Phases 5/6). It will be added alongside that port.
 */
struct MissionState {
    static constexpr int kMaxWaypoints = 100;

    bool auto_navigation_mode = false;
    bool auto_throttle_mode = false;
    bool auto_loiter_mode = false;

    float target_airspeed_mps = 18.0f;
    float relative_alt_m = 0.0f;

    int flag_wp = -1;
    int wp_sum = 0;
    Locations waypoint[kMaxWaypoints]{};
    // Radius penerimaan per-WP dari MAVLink MISSION_ITEM param2 (m); 0 = pakai
    // radius default. Tidak disimpan di EEPROM (sama seperti Khageswara).
    float waypoint_radius_m[kMaxWaypoints]{};

    Locations prev_wp_loc{};
    Locations current_loc{};
    Locations next_wp_loc{};
    Locations home_wp_loc{};
    bool home_set = false;

    float acceptance_distance_m = 0.0f;

    // Perilaku akhir misi (MIS_DONE_BEHAVE). Setelah WP terakhir tercapai,
    // mode AUTO tetap aktif tetapi navigasinya berpindah ke "done behaviour":
    // loiter di WP terakhir, atau RTL ke home lalu loiter. Di-reset saat
    // masuk AUTO (Navigation::beginAuto()).
    bool mission_complete = false;
    bool done_returning_home = false;  // true selama fase terbang menuju home (RTL)
    Locations done_center{};           // pusat loiter
    Locations done_start_loc{};        // posisi saat misi selesai (awal garis ke home)

    struct AutoState {
        bool next_wp_crosstrack = false;
        bool crosstrack = false;
        float distance_next_wp = 0.0f;
        float next_wp_altitude = 0.0f;
        float bearing_deg = 0.0f;
        float wp_proportion = 0.0f;
        float next_turn_angle_deg = 0.0f;
    } auto_state{};

    struct TargetAltitude {
        float amsl_cm = 0.0f;
    } target_altitude{};

    // Companion-computer (RPi) guided-setpoint bridge, fed by MAVLink
    // SET_POSITION_TARGET_* (ported alongside Mavlink.h in Phase 7).
    bool rpi_external_setpoint_active = false;
    uint32_t rpi_external_setpoint_last_ms = 0;
    Locations rpi_external_setpoint_target{};

    // Pencatat klaim waypoint terakhir, dibaca fc::FlightLog untuk event
    // WP_REACHED di log SD. Bit reason: 1 = radius, 2 = garis finish,
    // 4 = zona WP terakhir (last_wp_acceptance_m). Hanya diisi Navigation.
    uint32_t wp_reached_count = 0;
    int16_t wp_reached_seq = -1;
    uint8_t wp_reached_reason = 0;

    // Attitude targets for the LQR attitude controller (Phase 4), written by
    // Navigation::updateAutoAttitudeTargets() — replaces legacy
    // FW_ControlModes.h's global nav_roll_deg/nav_pitch_deg.
    float nav_roll_deg = 0.0f;
    float nav_pitch_deg = 0.0f;
};

}  // namespace fc
