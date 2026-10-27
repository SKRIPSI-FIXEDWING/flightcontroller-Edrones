#include "communication/FlightLog.h"

#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "vehicle/ControlLock.h"

namespace fc {

namespace {

// Column names, in the exact order writeRow() prints them. Keep both in sync
// (docs/flight-log-csv.md lists every column).
constexpr const char* kColumns =
    "time_boot_ms,gps_seq,gps_new,mode,armed,lat_e7,lon_e7,gps_alt_m,gps_fix,gps_sats,gps_hdop,"
    "gps_age_ms,gs_mps,course_deg,airspeed_mps,alt_fused_m,alt_baro_m,"
    "roll_deg,pitch_deg,yaw_deg,p_dps,q_dps,r_dps,mhn_roll,mhn_pitch,mhn_yaw,"
    "wp_prev,wp_next,leg_from_wp,wp_dist_m,cte_m,xtrack_vel_mps,de_mps,l1_period_s,l1_dist_m,"
    "nav_roll_deg,nav_pitch_deg,fuzzy_on,fuzzy_out,"
    "ail_cmd_us,ail_out_us,ele_out_us,thr_out_us,rud_out_us,dist_on,dist_us,"
    "ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,batt_v,imu_cal,rows_dropped,ctrl_overrun\r\n";

// printf-style helper: formats one line and queues it (all-or-nothing).
void emit(SdLogger& sd, const char* fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n <= 0) {
        return;
    }
    if (n >= static_cast<int>(sizeof(buf))) {  // truncated: still terminate the line
        n = static_cast<int>(sizeof(buf)) - 1;
        buf[n - 2] = '\r';
        buf[n - 1] = '\n';
    }
    sd.writeLine(buf, static_cast<size_t>(n));
}

void reasonText(uint8_t reason, char* out, size_t out_len)
{
    out[0] = '\0';
    size_t used = 0;
    const struct {
        uint8_t bit;
        const char* name;
    } kReasons[] = {{1, "radius"}, {2, "finish_line"}, {4, "last_wp_zone"}};
    for (const auto& r : kReasons) {
        if ((reason & r.bit) == 0) {
            continue;
        }
        const int n = snprintf(out + used, out_len - used, "%s%s", used > 0 ? "+" : "", r.name);
        if (n <= 0 || static_cast<size_t>(n) >= out_len - used) {
            return;
        }
        used += static_cast<size_t>(n);
    }
    if (used == 0) {
        snprintf(out, out_len, "unknown");
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Control-task side
// ---------------------------------------------------------------------------

void FlightLog::pushEvent(const FlightEvent& event)
{
    const uint8_t head = event_head_;
    const uint8_t next = static_cast<uint8_t>((head + 1U) & (kEventRing - 1U));
    if (next == event_tail_) {
        events_lost_ = events_lost_ + 1;  // ring full: logger is not draining (no file open)
        return;
    }
    events_[head] = event;
    __sync_synchronize();
    event_head_ = next;
}

bool FlightLog::popEvent(FlightEvent& out)
{
    const uint8_t tail = event_tail_;
    if (tail == event_head_) {
        return false;
    }
    __sync_synchronize();
    out = events_[tail];
    event_tail_ = static_cast<uint8_t>((tail + 1U) & (kEventRing - 1U));
    return true;
}

void FlightLog::capture(VehicleContext& ctx, float raw_dt_s)
{
    const uint32_t now_ms = millis();
    FlightLogSnapshot s;

    if (raw_dt_s * 1000.0f > static_cast<float>(kCtrlOverrunThresholdMs)) {
        ctrl_overrun_++;
    }

    s.time_boot_ms = now_ms;
    strncpy(s.mode, ctx.mode_manager.code4(), sizeof(s.mode) - 1);
    s.armed = ctx.radio.armed();

    // --- GPS ---
    const GnssFixData& gnss = ctx.gnss;
    s.gps_seq = gnss.sequence;
    s.lat_e7 = static_cast<int32_t>(lround(gnss.latitude_deg * 1.0e7));
    s.lon_e7 = static_cast<int32_t>(lround(gnss.longitude_deg * 1.0e7));
    s.gps_alt_m = gnss.altitude_m;
    s.gps_fix = gnss.valid ? static_cast<uint8_t>(gnss.fix_type) : 0;
    s.gps_sats = gnss.satellites;
    s.gps_hdop = gnss.hdop;
    if (gnss.timestamp_us == 0) {
        s.gps_age_ms = 65535;
    } else {
        const uint32_t age_ms = (micros() - gnss.timestamp_us) / 1000U;
        s.gps_age_ms = static_cast<uint16_t>(age_ms > 65535U ? 65535U : age_ms);
    }
    s.gs_mps = gnss.ground_speed_mps;
    s.course_deg = gnss.course_deg;

    // --- speed / altitude ---
    const BarometerData baro = ctx.baro.data();
    s.airspeed_mps = ctx.airspeed.data().velocity_mps;
    s.alt_fused_m = baro.altitude_m;  // fused when FC_ALTITUDE_FUSION_ENABLE
    s.alt_baro_m = baro.baro_only_altitude_m;

    // --- attitude ---
    const ImuData imu = ctx.controlImu();
    s.roll_deg = imu.roll_deg;
    s.pitch_deg = imu.pitch_deg;
    s.yaw_deg = imu.yaw_deg;
    s.p_dps = imu.angular_rate_dps.x;
    s.q_dps = imu.angular_rate_dps.y;
    s.r_dps = imu.angular_rate_dps.z;
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
    const AttitudeMahonyData mahony = ctx.attitude_mahony.data();
    s.mhn_roll = mahony.roll_deg;
    s.mhn_pitch = mahony.pitch_deg;
    s.mhn_yaw = mahony.yaw_deg;
#endif

    // --- navigation / guidance ---
    const MissionState& mission = ctx.navigation.state();
    s.l1_period_s = ctx.l1.period();
    s.fuzzy_on = ctx.fuzzy_tuner.enabled();
    // auto_throttle_mode also covers LOITER/RTL (auto_navigation_mode is false
    // there), so the loiter radius error and the fuzzy output get logged too.
    if (mission.auto_navigation_mode || mission.auto_throttle_mode) {
        const int flag = mission.flag_wp;
        s.wp_next = static_cast<int16_t>(flag);
        s.wp_prev = static_cast<int16_t>(flag > 0 ? flag - 1 : 0);
        s.leg_from_wp = mission.auto_state.crosstrack;
        s.wp_dist_m = mission.auto_state.distance_next_wp;
        s.cte_m = ctx.l1.crosstrackError();
        s.xtrack_vel_mps = ctx.l1.crosstrackVelocityMps();
        s.de_mps = ctx.fuzzy_tuner.lastCrosstrackErrorRateMps();
        s.l1_period_s = ctx.l1.periodUsedS();
        s.l1_dist_m = ctx.l1.l1Distance();
        s.nav_roll_deg = mission.nav_roll_deg;
        s.nav_pitch_deg = mission.nav_pitch_deg;
        s.fuzzy_out = ctx.fuzzy_tuner.lastDeltaPeriod();
    }

    // --- actuators ---
    s.ail_out_us = ctx.actuator.aileronLeftPwm();
    s.ail_cmd_us = ctx.actuator.aileronCommandLeftPwm();
    s.ele_out_us = ctx.actuator.elevatorPwm();
    s.thr_out_us = ctx.actuator.throttlePwm();
    s.rud_out_us = ctx.actuator.rudderLeftPwm();
    s.dist_on = ctx.disturbance.active();
    s.dist_us = ctx.disturbance.offsetUs();

    const ImuCalibration cal = ctx.imu.calibration();
    s.imu_cal = static_cast<uint16_t>(cal.system * 1000U + cal.gyroscope * 100U +
                                      cal.accelerometer * 10U + cal.magnetometer);

    // --- RC ---
    s.ch[0] = ctx.radio.channelRoll();
    s.ch[1] = ctx.radio.channelPitch();
    s.ch[2] = ctx.radio.channelThrottle();
    s.ch[3] = ctx.radio.channelYaw();
    s.ch[4] = ctx.radio.channelArm();
    s.ch[5] = ctx.radio.channelMode();
    s.ch[6] = ctx.radio.channelModeBackup();
    s.ch[7] = ctx.radio.channelVehicleMode();

    s.batt_v = ctx.battery.data().voltage_v;
    s.ctrl_overrun = ctrl_overrun_;

    // --- event edges (first call only primes the previous-state trackers) ---
    if (!capture_primed_) {
        capture_primed_ = true;
        prev_armed_ = s.armed;
        memcpy(prev_mode_, s.mode, sizeof(prev_mode_));
        prev_wp_reached_count_ = mission.wp_reached_count;
    } else {
        FlightEvent event;
        event.time_ms = now_ms;

        if (s.armed != prev_armed_) {
            prev_armed_ = s.armed;
            event.type = s.armed ? FlightEventType::Arm : FlightEventType::Disarm;
            pushEvent(event);
        }
        if (strncmp(s.mode, prev_mode_, 4) != 0) {
            memcpy(prev_mode_, s.mode, sizeof(prev_mode_));
            event.type = FlightEventType::Mode;
            memcpy(event.text, s.mode, sizeof(event.text));
            pushEvent(event);
        }
        if (mission.wp_reached_count != prev_wp_reached_count_) {
            prev_wp_reached_count_ = mission.wp_reached_count;
            event.type = FlightEventType::WpReached;
            event.seq = mission.wp_reached_seq;
            event.reason = mission.wp_reached_reason;
            pushEvent(event);
        }
        // Disturbance: DIST_ON when it starts or changes level/side, DIST_OFF when it ends.
        if (s.dist_us != prev_dist_us_) {
            FlightEvent dist_event;
            dist_event.time_ms = now_ms;
            dist_event.type = (s.dist_us != 0) ? FlightEventType::DistOn : FlightEventType::DistOff;
            dist_event.seq = s.dist_us;
            pushEvent(dist_event);
        }
    }
    prev_dist_us_ = s.dist_us;

    // GPS_LOST / GPS_OK, debounced: the new state must hold for
    // kGpsEventDebounceMs. The event is stamped with when the change began.
    const bool gps_now = gnss.valid && gnss.fix_type == GnssFixType::Fix3D &&
                         s.gps_age_ms <= kGpsGoodMaxAgeMs;
    if (gps_now != gps_candidate_) {
        gps_candidate_ = gps_now;
        gps_candidate_since_ms_ = now_ms;
    }
    if (gps_candidate_ != gps_good_ && now_ms - gps_candidate_since_ms_ >= kGpsEventDebounceMs) {
        gps_good_ = gps_candidate_;
        FlightEvent event;
        event.time_ms = gps_candidate_since_ms_;
        event.type = gps_good_ ? FlightEventType::GpsOk : FlightEventType::GpsLost;
        pushEvent(event);
    }

    // --- publish (seqlock: logger retries if it read during this write) ---
    snapshot_seq_ = snapshot_seq_ + 1;
    __sync_synchronize();
    snapshot_ = s;
    __sync_synchronize();
    snapshot_seq_ = snapshot_seq_ + 1;
}

bool FlightLog::readSnapshot(FlightLogSnapshot& out) const
{
    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint32_t before = snapshot_seq_;
        if ((before & 1U) != 0U) {
            continue;
        }
        __sync_synchronize();
        out = snapshot_;
        __sync_synchronize();
        if (snapshot_seq_ == before) {
            return before != 0U;  // 0 = capture() has not run yet
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Logger-task side
// ---------------------------------------------------------------------------

void FlightLog::step(SdLogger& sd, VehicleContext& ctx)
{
    FlightLogSnapshot s;
    const bool have_snapshot = readSnapshot(s);
    if (have_snapshot) {
        sd.setArmed(s.armed);
    }

    FlightEvent event;
    if (!sd.isOpen()) {
        while (popEvent(event)) {
        }  // nothing to write them to; do not let stale events pile up
        return;
    }

    const uint32_t now_ms = millis();
    if (sd.fileGeneration() != logged_generation_) {
        logged_generation_ = sd.fileGeneration();
        last_gps_seq_ = 0;
        while (popEvent(event)) {
        }  // happened before this file existed; the header snapshot covers state
        writeHeader(sd, ctx, now_ms);
    }

    while (popEvent(event)) {
        writeEvent(sd, ctx, event);
    }
    checkParamChanges(sd, ctx, now_ms);

    if (have_snapshot) {
        writeRow(sd, s);
    }
}

void FlightLog::writeHeader(SdLogger& sd, VehicleContext& ctx, uint32_t now_ms)
{
    emit(sd, "# EDRONES_FLIGHT_LOG format=2 (docs/flight-log-csv.md)\r\n");
    emit(sd, "# fw_tag=%s build=%s %s\r\n", FC_BUILD_TAG, __DATE__, __TIME__);
    emit(sd, "# file=%s boot_ms=%lu log_period_ms=%lu\r\n", sd.fileName(),
         static_cast<unsigned long>(now_ms), static_cast<unsigned long>(kSdLogPeriodMs));
    emit(sd, "# time_boot_ms = millis(), same clock as MAVLink time_boot_ms (join with tlog on it)\r\n");
    emit(sd, "# gps_fix: 0=none 1=2D 2=3D. gps_new: 1 = fix changed since the previous row. "
             "gps_age_ms saturates at 65535\r\n");
    emit(sd, "# cte_m: + = aircraft LEFT of the line wp_prev->wp_next (looking along it), "
             "- = RIGHT. xtrack_vel_mps = d(cte)/dt from GPS velocity (same sign)\r\n");
    emit(sd, "# de_mps = d|cte|/dt exactly as fed to the fuzzy tuner (unfiltered). "
             "fuzzy_out = defuzzified L1 period offset dT [s], 0 when fuzzy is off\r\n");
    emit(sd, "# l1_period_s = L1 period actually used by that cycle's guidance update. "
             "leg_from_wp: 1 = line starts at wp_prev, 0 = starts at the aircraft (first leg)\r\n");
    emit(sd, "# wp_prev/wp_next = -1 and nav columns = 0 unless AUTO/GUIDED. "
             "wp_dist_m = distance to wp_next\r\n");
    emit(sd, "# ail_cmd_us = LEFT aileron servo PWM from the controller alone; ail_out_us = what was sent "
             "(right servo identical)\r\n");
    emit(sd, "# dist_on/dist_us = injected roll disturbance (us, roll-command domain, + = roll right = "
             "stick right; servo PWM moves the opposite way because of the aileron mirror). 0 unless "
             "DIST_ENABLE=1 in AUTO\r\n");
    emit(sd, "# imu_cal = BNO055 calibration sys*1000+gyro*100+accel*10+mag (each 0..3, 3333 = full)\r\n");
    emit(sd, "# ch1..ch8 = roll,pitch,throttle,yaw,arm,mode,mode_backup,vehicle_mode (calibrated PWM-equivalent). "
             "ctrl_overrun = control cycles later than %lu ms\r\n",
         static_cast<unsigned long>(kCtrlOverrunThresholdMs));
    emit(sd, "#SNAPSHOT,BOOT,%lu\r\n", static_cast<unsigned long>(now_ms));
    writeSnapshotBlock(sd, ctx);
    sd.writeLine(kColumns, strlen(kColumns));
}

void FlightLog::writeSnapshotBlock(SdLogger& sd, VehicleContext& ctx)
{
    // What the flight code is ACTUALLY using right now. Most params only take
    // effect after a reboot (their config structs are copied at construction),
    // so the Params table below can differ from what is flying.
    emit(sd, "# EFFECTIVE l1_damping=%.3f l1_xtrack_i=%.3f l1_period_now=%.2f wp_radius_m=%.1f "
             "roll_limit_deg=%.1f\r\n",
         static_cast<double>(ctx.l1.damping()), static_cast<double>(ctx.l1.xtrackIntegratorGain()),
         static_cast<double>(ctx.l1.period()), static_cast<double>(ctx.navigation.waypointRadius()),
         static_cast<double>(ctx.navigation.rollLimitDeg()));
    const FuzzyL1TunerConfig& fuzzy = ctx.fuzzy_tuner.config();
    emit(sd, "# EFFECTIVE fuzzy_enable=%u fuzzy_base_period=%.1f fuzzy_min_per=%.1f fuzzy_max_per=%.1f\r\n",
         ctx.fuzzy_tuner.enabled() ? 1U : 0U, static_cast<double>(fuzzy.base_period_s),
         static_cast<double>(fuzzy.min_period_s), static_cast<double>(fuzzy.max_period_s));

    emit(sd, "# PARAM_TABLE (value stored in the param table; only WP_RADIUS and ROLL_LIMIT_DEG apply "
             "live, all others after a reboot)\r\n");
    char line[200];
    size_t used = 0;
    int on_line = 0;
    line[0] = '\0';
    param_cache_count_ = ctx.params.count();
    for (uint16_t i = 0; i < param_cache_count_ && i < Params::kMaxParams; ++i) {
        const ParamEntry& entry = ctx.params.entryAt(i);
        param_cache_[i] = *entry.target;
        if (on_line == 0) {
            used = static_cast<size_t>(snprintf(line, sizeof(line), "# PARAM"));
        }
        const int n = snprintf(line + used, sizeof(line) - used, " %s=%.6g", entry.name,
                               static_cast<double>(*entry.target));
        if (n > 0 && used + static_cast<size_t>(n) < sizeof(line) - 3) {
            used += static_cast<size_t>(n);
        }
        if (++on_line == 5 || i + 1 == param_cache_count_) {
            emit(sd, "%s\r\n", line);
            on_line = 0;
        }
    }

    const MissionState& mission = ctx.navigation.state();
    if (mission.home_set) {
        emit(sd, "# HOME,%ld,%ld,%ld\r\n", static_cast<long>(mission.home_wp_loc.lat),
             static_cast<long>(mission.home_wp_loc.lng), static_cast<long>(mission.home_wp_loc.alt));
    } else {
        emit(sd, "# HOME,unset\r\n");
    }
    emit(sd, "# WP_FIELDS seq,lat_e7,lon_e7,alt_cm,radius_m(0=default WP_RADIUS)\r\n");
    int count = mission.wp_sum;
    if (count > MissionState::kMaxWaypoints) {
        count = MissionState::kMaxWaypoints;
    }
    for (int i = 0; i < count; ++i) {
        // Copy one item at a time under the control lock: a short hold keeps the
        // control task (which takes this lock every cycle) from ever waiting long.
        bool ok = false;
        int32_t lat = 0, lng = 0, alt = 0;
        float radius = 0.0f;
        {
            ControlLock lock(pdMS_TO_TICKS(5));
            if (lock.acquired()) {
                lat = mission.waypoint[i].lat;
                lng = mission.waypoint[i].lng;
                alt = mission.waypoint[i].alt;
                radius = mission.waypoint_radius_m[i];
                ok = true;
            }
        }
        if (ok) {
            emit(sd, "# WP,%d,%ld,%ld,%ld,%.1f\r\n", i, static_cast<long>(lat), static_cast<long>(lng),
                 static_cast<long>(alt), static_cast<double>(radius));
        } else {
            emit(sd, "# WP,%d,unavailable\r\n", i);
        }
    }
}

void FlightLog::writeEvent(SdLogger& sd, VehicleContext& ctx, const FlightEvent& event)
{
    const unsigned long t = static_cast<unsigned long>(event.time_ms);
    switch (event.type) {
    case FlightEventType::Arm: {
        emit(sd, "#EVENT,%lu,ARM\r\n", t);
        // Parameters can change between flights without a reboot, and the
        // tlog does not always carry the full mission -- so every ARM writes
        // the settings again. Skipped (and said so) if the RAM buffer lacks room.
        const size_t needed = 4000U + static_cast<size_t>(ctx.navigation.state().wp_sum) * 80U;
        if (sd.bufferFree() >= needed) {
            emit(sd, "#ARM_SNAPSHOT,%lu\r\n", t);
            writeSnapshotBlock(sd, ctx);
        } else {
            emit(sd, "#EVENT,%lu,ARM_SNAPSHOT_SKIPPED,buffer_full\r\n", t);
        }
        char message[48];
        snprintf(message, sizeof(message), "[SD] ARM log=%s", sd.fileName());
        sd.announce(message);
        break;
    }
    case FlightEventType::Disarm:
        emit(sd, "#EVENT,%lu,DISARM\r\n", t);
        break;
    case FlightEventType::Mode:
        emit(sd, "#EVENT,%lu,MODE,%s\r\n", t, event.text);
        break;
    case FlightEventType::WpReached: {
        char reason[40];
        reasonText(event.reason, reason, sizeof(reason));
        emit(sd, "#EVENT,%lu,WP_REACHED,%d,%s\r\n", t, static_cast<int>(event.seq), reason);
        break;
    }
    case FlightEventType::GpsLost:
        emit(sd, "#EVENT,%lu,GPS_LOST\r\n", t);
        break;
    case FlightEventType::GpsOk:
        emit(sd, "#EVENT,%lu,GPS_OK\r\n", t);
        break;
    case FlightEventType::DistOn:
        emit(sd, "#EVENT,%lu,DIST_ON,%+d\r\n", t, static_cast<int>(event.seq));
        break;
    case FlightEventType::DistOff:
        emit(sd, "#EVENT,%lu,DIST_OFF\r\n", t);
        break;
    }
    sd.requestFlush();  // important moments reach the card within one writer cycle
}

void FlightLog::checkParamChanges(SdLogger& sd, VehicleContext& ctx, uint32_t now_ms)
{
    // Polling the table (instead of hooking MAVLink PARAM_SET) catches a
    // change from any source and keeps this module independent of Mavlink.
    const uint16_t count = ctx.params.count();
    for (uint16_t i = 0; i < count && i < param_cache_count_; ++i) {
        const ParamEntry& entry = ctx.params.entryAt(i);
        const float value = *entry.target;
        if (value != param_cache_[i]) {
            param_cache_[i] = value;
            emit(sd, "#EVENT,%lu,PARAM,%s,%.6g\r\n", static_cast<unsigned long>(now_ms), entry.name,
                 static_cast<double>(value));
            sd.requestFlush();
        }
    }
}

void FlightLog::writeRow(SdLogger& sd, const FlightLogSnapshot& s)
{
    const bool gps_new = s.gps_seq != last_gps_seq_;
    last_gps_seq_ = s.gps_seq;

    char line[640];
    int n = snprintf(
        line, sizeof(line),
        "%lu,%lu,%u,%s,%u,%ld,%ld,%.1f,%u,%u,%.2f,%u,%.2f,%.1f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,"
        "%d,%d,%u,%.1f,%.2f,%.2f,%.3f,%.2f,%.1f,%.2f,%.2f,%u,%.3f,"
        "%u,%u,%u,%u,%u,%u,%d,"
        "%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%u,%lu,%lu\r\n",
        static_cast<unsigned long>(s.time_boot_ms), static_cast<unsigned long>(s.gps_seq),
        gps_new ? 1U : 0U, s.mode, s.armed ? 1U : 0U, static_cast<long>(s.lat_e7),
        static_cast<long>(s.lon_e7), static_cast<double>(s.gps_alt_m), static_cast<unsigned>(s.gps_fix),
        static_cast<unsigned>(s.gps_sats), static_cast<double>(s.gps_hdop),
        static_cast<unsigned>(s.gps_age_ms), static_cast<double>(s.gs_mps),
        static_cast<double>(s.course_deg), static_cast<double>(s.airspeed_mps),
        static_cast<double>(s.alt_fused_m), static_cast<double>(s.alt_baro_m),
        static_cast<double>(s.roll_deg), static_cast<double>(s.pitch_deg), static_cast<double>(s.yaw_deg),
        static_cast<double>(s.p_dps), static_cast<double>(s.q_dps), static_cast<double>(s.r_dps),
        static_cast<double>(s.mhn_roll), static_cast<double>(s.mhn_pitch), static_cast<double>(s.mhn_yaw),
        static_cast<int>(s.wp_prev), static_cast<int>(s.wp_next), s.leg_from_wp ? 1U : 0U,
        static_cast<double>(s.wp_dist_m), static_cast<double>(s.cte_m),
        static_cast<double>(s.xtrack_vel_mps), static_cast<double>(s.de_mps),
        static_cast<double>(s.l1_period_s), static_cast<double>(s.l1_dist_m),
        static_cast<double>(s.nav_roll_deg), static_cast<double>(s.nav_pitch_deg),
        s.fuzzy_on ? 1U : 0U, static_cast<double>(s.fuzzy_out),
        static_cast<unsigned>(s.ail_cmd_us), static_cast<unsigned>(s.ail_out_us),
        static_cast<unsigned>(s.ele_out_us), static_cast<unsigned>(s.thr_out_us),
        static_cast<unsigned>(s.rud_out_us), s.dist_on ? 1U : 0U, static_cast<int>(s.dist_us),
        static_cast<unsigned>(s.ch[0]), static_cast<unsigned>(s.ch[1]), static_cast<unsigned>(s.ch[2]),
        static_cast<unsigned>(s.ch[3]), static_cast<unsigned>(s.ch[4]), static_cast<unsigned>(s.ch[5]),
        static_cast<unsigned>(s.ch[6]), static_cast<unsigned>(s.ch[7]), static_cast<double>(s.batt_v),
        static_cast<unsigned>(s.imu_cal), static_cast<unsigned long>(sd.rowsDropped()), static_cast<unsigned long>(s.ctrl_overrun));
    if (n <= 0) {
        return;
    }
    if (n >= static_cast<int>(sizeof(line))) {  // absurd values (inf/huge floats): keep the line terminated
        n = static_cast<int>(sizeof(line)) - 1;
        line[n - 2] = '\r';
        line[n - 1] = '\n';
    }
    sd.writeLine(line, static_cast<size_t>(n));
}

}  // namespace fc
