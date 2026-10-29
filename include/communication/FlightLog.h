#pragma once

#include <stdint.h>

#include "FC_Config.h"
#include "communication/SdLogger.h"
#include "modes/VehicleContext.h"
#include "storage/Params.h"

namespace fc {

/**
 * One coherent copy of everything the flight log records, captured by the
 * CONTROL task at the end of a control cycle -- so navigation/guidance values
 * (cte, L1 period, nav_roll, ...) are the very ones that cycle's control
 * laws used, not recomputed later by the logger. Plain data on purpose:
 * formatting into CSV text happens in the (slower, lower-priority) logger.
 */
struct FlightLogSnapshot {
    uint32_t time_boot_ms = 0;  // millis(): same clock as MAVLink time_boot_ms
    char mode[5] = {};
    bool armed = false;

    // GPS (GnssFixData as the control task saw it this cycle)
    uint32_t gps_seq = 0;
    int32_t lat_e7 = 0;
    int32_t lon_e7 = 0;
    float gps_alt_m = 0.0f;
    uint8_t gps_fix = 0;  // 0 = none, 1 = 2D, 2 = 3D
    uint8_t gps_sats = 0;
    float gps_hdop = 0.0f;
    uint16_t gps_age_ms = 0;  // saturates at 65535
    float gs_mps = 0.0f;
    float course_deg = 0.0f;
    uint64_t utc_us = 0;  // GPS UTC time at time_boot_ms, us since 1970; 0 = unknown

    float airspeed_mps = 0.0f;
    float alt_fused_m = 0.0f;
    float alt_baro_m = 0.0f;

    // Attitude actually flown on (BNO055 unless FC_ATTITUDE_CONTROL_SOURCE_MAHONY)
    float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
    float p_dps = 0.0f, q_dps = 0.0f, r_dps = 0.0f;
    float mhn_roll = 0.0f, mhn_pitch = 0.0f, mhn_yaw = 0.0f;

    // Navigation / guidance (zero outside AUTO/GUIDED, except l1_period_s)
    int16_t wp_prev = -1, wp_next = -1;
    bool leg_from_wp = false;
    float wp_dist_m = 0.0f;
    float cte_m = 0.0f;
    float xtrack_vel_mps = 0.0f;
    float de_mps = 0.0f;
    float l1_period_s = 0.0f;
    float l1_dist_m = 0.0f;
    float nav_roll_deg = 0.0f, nav_pitch_deg = 0.0f;
    bool fuzzy_on = false;
    float fuzzy_out = 0.0f;

    // Actuators (servo PWM read back from fc::Actuator)
    uint16_t ail_cmd_us = 0;  // left aileron servo PWM the controller alone asked for (no disturbance)
    uint16_t ail_out_us = 0;  // left aileron servo PWM actually sent (with disturbance)
    uint16_t ele_out_us = 0, thr_out_us = 0, rud_out_us = 0;
    bool dist_on = false;
    int16_t dist_us = 0;  // requested roll-command offset, + = roll right

    // BNO055 calibration status packed as sys*1000 + gyro*100 + accel*10 + mag (0..3 each; 3333 = full)
    uint16_t imu_cal = 0;

    uint16_t ch[8] = {};  // ch1..ch8, calibrated PWM-equivalent (988..2012)
    float batt_v = 0.0f;
    uint32_t ctrl_overrun = 0;
};

enum class FlightEventType : uint8_t { Arm, Disarm, Mode, WpReached, GpsLost, GpsOk, DistOn, DistOff };

struct FlightEvent {
    uint32_t time_ms = 0;
    FlightEventType type = FlightEventType::Arm;
    uint8_t reason = 0;  // WpReached: Navigation's wp_reached_reason bits
    int16_t seq = 0;     // WpReached: waypoint index; DistOn: signed offset in us
    char text[5] = {};   // Mode: 4-letter mode code
};

/**
 * Flight-data log content (docs/flight-log-csv.md): CSV rows, "#EVENT" rows,
 * the file header, and the parameter/mission snapshot at every ARM. Output
 * goes through fc::SdLogger, which never blocks.
 *
 * Two entry points, one per task, sharing only lock-free state:
 *
 * - capture(): control task, every control cycle. Fills the snapshot and
 *   detects event edges (arm, mode, waypoint reached, GPS lost/ok). Cheap,
 *   never blocks, never touches the card.
 * - step(): taskSdLog, every kSdLogPeriodMs. Writes whatever is due (file
 *   header on a new file, events, parameter-change events, one CSV row).
 */
class FlightLog final {
public:
    void capture(VehicleContext& ctx, float raw_dt_s);
    void step(SdLogger& sd, VehicleContext& ctx);

private:
    bool readSnapshot(FlightLogSnapshot& out) const;
    void pushEvent(const FlightEvent& event);
    bool popEvent(FlightEvent& out);

    void writeHeader(SdLogger& sd, VehicleContext& ctx, uint32_t now_ms);
    void writeSnapshotBlock(SdLogger& sd, VehicleContext& ctx);
    void writeEvent(SdLogger& sd, VehicleContext& ctx, const FlightEvent& event);
    void writeRow(SdLogger& sd, const FlightLogSnapshot& s);
    void checkParamChanges(SdLogger& sd, VehicleContext& ctx, uint32_t now_ms);

    // --- shared between capture() and step() ---
    static constexpr uint8_t kEventRing = 16;  // power of two
    FlightLogSnapshot snapshot_{};
    volatile uint32_t snapshot_seq_ = 0;  // seqlock: odd while capture() is writing
    FlightEvent events_[kEventRing]{};
    volatile uint8_t event_head_ = 0;  // capture() only
    volatile uint8_t event_tail_ = 0;  // step() only
    volatile uint32_t events_lost_ = 0;

    // --- capture()-only state ---
    bool capture_primed_ = false;
    bool prev_armed_ = false;
    char prev_mode_[5] = {};
    uint32_t prev_wp_reached_count_ = 0;
    uint32_t ctrl_overrun_ = 0;
    int16_t prev_dist_us_ = 0;
    bool gps_good_ = false;
    bool gps_candidate_ = false;
    uint32_t gps_candidate_since_ms_ = 0;

    // --- step()-only state ---
    uint32_t logged_generation_ = 0;
    uint32_t last_gps_seq_ = 0;
    bool utc_logged_ = false;   // #EVENT UTC written to the current file
    bool name_logged_ = false;  // #EVENT LOG_NAME written to the current file
    float param_cache_[Params::kMaxParams] = {};
    uint16_t param_cache_count_ = 0;
};

}  // namespace fc
