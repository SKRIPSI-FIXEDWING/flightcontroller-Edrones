#pragma once

#include <AP_Math.h>
#include <Locations.h>

#include "estimation/Ahrs.h"
#include "navigation/L1Controller.h"
#include "navigation/MissionState.h"
#include "navigation/Tecs.h"

namespace fc {

struct NavigationConfig {
    float wp_radius_default_m = 15.0f;  // radius klaim WP (m); param "WP_RADIUS", override per-WP lewat MAVLink param2
    float min_groundspeed_mps = 17.0f;  // 0 disables the undershoot check

    // Tanda arah NAVIGASI roll di AUTO. L1 menghasilkan nav_roll > 0 bila
    // waypoint ada di kanan (konvensi ArduPilot, asalkan yaw searah jarum
    // jam). Nilai ini dikalikan ke hasil L1 sebelum diteruskan ke kendali
    // attitude, jadi mission.nav_roll_deg selalu berkonvensi stick
    // (positif = bank kanan). -1 = membalik (setara `roll_cmd = -nav_roll`
    // Khageswara updateAUTO_FW). Kendali attitude TIDAK disentuh.
    // Diisi dari FC_NAV_ROLL_SIGN (FC_Config.h).
    float nav_roll_sign = 1.0f;

    float roll_limit_deg = 40.0f;   // param ROLL_LIMIT_DEG (dulu tetap 36 = HEAD_MAX Khageswara)
    float pitch_limit_min_deg = -10.0f;  // PITCH_MIN (disamakan dengan Khageswara)
    float pitch_limit_max_deg = 30.0f;   // PITCH_MAX (disamakan dengan Khageswara)
    float airspeed_min_mps = 14.0f;
    float airspeed_cruise_mps = 18.0f;

    float adaptive_turn_angle_thresh_deg = 20.0f;
    float adaptive_approach_mult = 2.0f;
    float adaptive_wings_level_deg = 10.0f;

    // Home is (re)latched while disarmed, at least this many ms apart, once
    // barometer altitude settles at/under this many meters (matches legacy
    // `baro.altitude <= 10`).
    uint32_t home_set_interval_ms = 5000;
    float home_altitude_settle_m = 10.0f;

    // RPi external-setpoint bridge: a setpoint older than this is dropped.
    uint32_t rpi_setpoint_timeout_ms = 1000;

    // Penerimaan waypoint: radius TETAP, tidak bergantung pada periode L1 atau
    // sudut belok (supaya perbandingan L1 konvensional vs L1 + fuzzy adil --
    // turn_distance() bergantung pada panjang L1, yang diubah FuzzyL1Tuner):
    //   acceptance = radius per-WP (MAVLink param2, bila > 0)
    //                selain itu wp_radius_default_m.
    // Tambahan: garis finish (melewati WP) tetap dipakai -- murni geometri.
    // last_wp_acceptance_m > 0 memperluas penerimaan WP terakhir (Khageswara:
    // 45 m); 0 = nonaktif, semua WP diperlakukan sama.
    float last_wp_acceptance_m = 0.0f;

    // Fitur khusus Edrones -- default mati agar sama dengan Khageswara
    // (lihat flag FC_* di FC_Config.h yang mengisi nilai ini di main.cpp).
    bool adaptive_waypoint_airspeed_enabled = false;
    bool reject_invalid_mission = false;

    // Perilaku saat semua WP selesai:
    //   0 = tidak melakukan apa-apa (tetap AUTO, terus mengikuti leg terakhir)
    //   1 = loiter di WP terakhir
    //   2 = RTL ke home lalu loiter di atas home
    uint8_t mission_done_behavior = 0;
    // Param WP_LOITER_RAD (Mission Planner: "Loiter Radius"). Tanda = arah,
    // seperti ArduPlane: negatif = CCW (default legacy -50), positif = CW.
    // Dipakai LOITER, RTL, dan loiter akhir misi.
    float loiter_radius_m = 100.0f;
    // Param FLTMODE1..6 (Mission Planner: Setup > Flight Modes). Nomor mode
    // ala ArduPlane (0 Manual, 4 Guided, 5 FBWA, 10 Auto, 12 Loiter) untuk 6
    // pita PWM saklar mode RC (CH6). Default cocok untuk saklar 3 posisi:
    // bawah = Manual, tengah = FBWA, atas = Auto. Dibaca main.cpp::applyRcModeSwitch.
    float flight_mode[6] = {0.0f, 0.0f, 5.0f, 5.0f, 10.0f, 10.0f};
    // Param LOIT_ROLL_SIGN: pengali KHUSUS roll navigasi saat loiter
    // (mode LOITER, loiter di WP, loiter akhir misi/RTL). +1 = apa adanya,
    // -1 = membalik respon roll loiter. Tidak mengubah roll AUTO ke WP.
    float loiter_roll_sign = 1.0f;
    // Param LOIT_ALT (m, relatif seperti altitude WP di Mission Planner):
    // ketinggian target mode LOITER. 0 = tahan ketinggian saat mode dimasuki
    // (perilaku ArduPlane); > 0 = TECS menaikkan/menurunkan ke ketinggian ini
    // sehingga ada pitch demand seperti di AUTO. Berlaku LIVE saat LOITER.
    float loiter_altitude_m = 0.0f;
    // Param LOIT_BANK_DEG: batas bank (derajat) khusus loiter, dipakai untuk
    // membatasi nav_roll saat berputar DAN oleh L1 untuk menghitung radius
    // minimum loiter. AUTO ke WP tetap memakai roll_limit_deg (ROLL_LIMIT_DEG).
    float loiter_bank_deg = 35.0f;
    float rtl_altitude_m = 60.0f;      // ketinggian RTL (m di atas home)
    float rtl_loiter_capture_margin_m = 15.0f;  // mulai loiter saat jarak < radius + margin
};

/**
 * Callback invoked when a waypoint is reached, so Navigation doesn't need to
 * know about MAVLink (ported alongside Mavlink.h in Phase 7, which will
 * register a callback that sends MISSION_ITEM_REACHED).
 */
using WaypointReachedCallback = void (*)(uint16_t seq, void* user_data);

/**
 * FW auto-navigation orchestration (legacy Navigation.h), consolidating
 * Auto_setup.h's loose globals into one owned MissionState and driving
 * L1Controller/Tecs instead of reaching into their globals.
 *
 * Takeoff sequencing (legacy auto_takeoff_mode/auto_takeoff_state) is not
 * ported here yet — see docs/navigation.md; it lands with Mode/Actuator
 * (Phases 5/6), which it's tightly coupled to.
 */
class Navigation final {
public:
    explicit Navigation(const NavigationConfig& config = NavigationConfig{});

    void setWaypointReachedCallback(WaypointReachedCallback callback, void* user_data);

    MissionState& state();
    const MissionState& state() const;

    /** Latch home (once, while disarmed and settled) and refresh position/windspeed. */
    void updateHomeAndPosition(Ahrs& ahrs, const GnssFixData& gnss, const BarometerData& baro,
                               bool armed, uint32_t now_ms);

    void updateWaypointNav(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu);

    void updateAltitude(Tecs& tecs, const AhrsData& ahrs, const ImuData& imu,
                        const BarometerData& baro, const AirspeedData& airspeed,
                        int16_t throttle_stick_percent, bool enable_throttle_nudge);
    void updateSpeedHeight(Tecs& tecs, const AhrsData& ahrs, const BarometerData& baro,
                           const ImuData& imu, const AirspeedData& airspeed);

    float getAdaptiveWaypointAirspeed(float dist_to_wp, float acceptance_dist,
                                      float next_turn_angle_deg, float current_roll_deg) const;

    /** Writes MissionState::nav_roll_deg/nav_pitch_deg from L1/TECS demand — the target the
     * Phase-4 LQR attitude controller tracks. */
    void updateAutoAttitudeTargets(L1Controller& l1, Tecs& tecs, const ImuData& imu);

    /**
     * Dipanggil dari ModeAuto::_enter(). Memvalidasi misi dan menyiapkan state:
     *  - butuh posisi GPS dan >= 2 item (item 0 = HOME, bukan target); home
     *    terkunci hanya wajib bila mission_done_behavior = RTL;
     *  - item target tidak boleh berkoordinat (0,0);
     *  - navigasi mulai dari item 1 (atau melanjutkan flag_wp bila masih valid);
     *  - misi yang sudah selesai dimulai ulang dari item 1.
     * Return false = tolak masuk AUTO; alasannya di autoRejectReason().
     */
    bool beginAuto();
    const char* autoRejectReason() const;

    /**
     * Make waypoint `seq` (1..wp_sum-1) the active target (MAVLink
     * MISSION_SET_CURRENT / MAV_CMD_MISSION_START). Works whether or not AUTO
     * is running: if not, beginAuto() resumes from it. A finished mission is
     * re-armed. Call under fc::ControlLock when not on the Control task.
     * Returns false if `seq` is outside the mission.
     */
    bool setCurrentWaypoint(int seq);

    /**
     * RTL (dipersiapkan, belum dipakai: mode RTL hanya terdaftar bila
     * FC_RTL_MODE_ENABLE = 1). beginRtl() menyiapkan terbang ke home pada
     * rtl_altitude_m lalu loiter di atasnya; updateRtl() dipanggil tiap siklus.
     */
    bool beginRtl();
    void updateRtl(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu, float eas2tas);

    /**
     * LOITER: beginLoiter() mengunci pusat putaran di posisi sekarang dan
     * ketinggian sekarang (false bila belum ada posisi GPS); updateLoiter()
     * dipanggil tiap siklus dan membaca radius/arah terbaru dari config.
     */
    bool beginLoiter();
    void updateLoiter(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu, float eas2tas);

    /** Radius loiter bertanda (param WP_LOITER_RAD), berlaku siklus berikutnya. */
    void setLoiterRadius(float radius_m);
    void setLoiterRollSign(float sign);
    void setLoiterAltitude(float altitude_m);
    void setLoiterBankDeg(float bank_deg);

    /** Main AUTO/GUIDED navigation state machine. */
    void navigate(bool is_guided_mode, L1Controller& l1, const AhrsData& ahrs, const ImuData& imu,
                 float eas2tas);

    void setNextWaypoint(const Locations& loc);

    /** Radius klaim WP default (param WP_RADIUS), dipakai mulai siklus navigasi berikutnya. */
    void setWaypointRadius(float radius_m);
    float waypointRadius() const;

    /** Batas |nav_roll| AUTO (param ROLL_LIMIT_DEG), dipakai mulai siklus navigasi berikutnya. */
    void setRollLimitDeg(float limit_deg);
    float rollLimitDeg() const;

    float getNextGroundCourse(float default_angle_deg) const;

    float groundspeedUndershoot() const;

private:
    void setupTurnAngle();
    void calcGroundspeedUndershoot(const AhrsData& ahrs, bool have_gps);
    void startMissionDone();
    void runMissionDone(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu, float eas2tas);
    void recordLoiterError(const Locations& center, float radius, int8_t direction);
    bool runWaypointLoiter(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu, float eas2tas,
                           bool& complete);
    void runReturnAndLoiter(L1Controller& l1, const AhrsData& ahrs, const ImuData& imu, float eas2tas);

    const char* auto_reject_reason_ = nullptr;
    bool was_armed_ = false;
    bool home_pending_on_arm_ = false;

    NavigationConfig config_{};
    MissionState mission_{};

    WaypointReachedCallback waypoint_reached_callback_ = nullptr;
    void* waypoint_reached_user_data_ = nullptr;

    uint32_t last_home_set_ms_ = 0;
    float groundspeed_undershoot_mps_ = 0.0f;
};

}  // namespace fc
