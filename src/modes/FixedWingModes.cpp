#include "modes/FixedWingModes.h"

#include <AP_Math.h>

#include "vehicle/Actuator.h"

namespace fc {

void ModeManual::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    ctx_.actuator.writeManual(ctx_.radio.channelRoll(), ctx_.radio.channelPitch(), ctx_.radio.channelYaw());
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), false, ctx_.radio.armed());
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);
}

bool ModeFbwa::_enter()
{
    ctx_.attitude.resetIntegrators();
    return true;
}

float ModeFbwa::mapStickToDeg(uint16_t channel_pwm, float max_deg)
{
    const int16_t centered = constrain(static_cast<int16_t>(channel_pwm) - 1500, -512, 512);
    return (static_cast<float>(centered) / 512.0f) * max_deg;
}

void ModeFbwa::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(), ctx_.now_ms);

    // Stick-derived attitude setpoints, tracked by the SAME LQR
    // AttitudeController that AUTO/GUIDED use -- there's only one
    // attitude-control implementation now.
    //
    // Reverted 2026-08-25 to match legacy FW_control.h's updateFBWA_FW()
    // formula EXACTLY on user's explicit request (was briefly flattened to
    // a plain +-35 deg on 2026-08-22 -- see git history if that's ever
    // wanted back): roll_cmd = 1.3*map(...,+-35deg), pitch_cmd =
    // 1.1*map(...,+-35deg), i.e. effectively +-45.5 deg roll / +-38.5 deg
    // pitch bank-angle setpoint range, not a flat +-35. Still NOT the same
    // limit as AttitudeControllerConfig's output_limit_deg -- that one caps
    // servo/surface deflection, this one caps the commanded ATTITUDE
    // setpoint itself (see output_limit_deg's own doc comment, also
    // reverted to match legacy today).
    // 2026-10-13: FBWA kini memakai kendali P/PD PERSIS program Khageswara
    // (FW_control.h::updateFBWA_FW()), bukan LQR -- pemetaan stick, gain,
    // integral, dan saturasi semuanya ada di
    // AttitudeController::updateFbwaKhageswara(). AUTO/GUIDED tetap LQR.
    const AttitudeController::Output output = ctx_.attitude.updateFbwaKhageswara(
        ctx_.radio.channelRoll(), ctx_.radio.channelPitch(), ctx_.controlImu(), ctx_.radio.armed(),
        ctx_.radio.channelThrottle(), ctx_.dt_s);

    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command, ctx_.radio.channelVehicleMode() > 1500);
}

bool ModeAuto::_enter()
{
    // Tolak masuk AUTO bila misi tidak valid (tanpa misi / home belum
    // terkunci / WP di 0,0). ModeManager::setMode() memulihkan mode lama.
    if (!ctx_.navigation.beginAuto()) {
        return false;
    }
    ctx_.attitude.resetIntegrators();
    // The disturbance injector starts disarmed until CH7 has been seen LOW.
    ctx_.disturbance.reset();
    return true;
}

void ModeAuto::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    const AhrsData& ahrs_data = ctx_.ahrs.data();
    const ImuData& imu_data = ctx_.imu.data();
    const float eas2tas = ctx_.baro.data().eas2tas;

    ctx_.navigation.navigate(/*is_guided_mode=*/false, ctx_.l1, ahrs_data, imu_data, eas2tas);

    // Fuzzy Sugeno menetapkan periode L1 untuk siklus BERIKUTNYA dari |e| dan
    // d|e|/dt yang baru dihitung L1. Param FUZZY_ENABLE = 0 atau misi selesai:
    // periode tetap NAVL1_PERIOD. Lihat docs/fuzzy-sugeno-l1.md.
    const bool fuzzy_allowed = !ctx_.navigation.state().mission_complete;
    ctx_.l1.setPeriod(ctx_.fuzzy.update(fuzzy_allowed, ctx_.l1.crosstrackError(),
                                        ctx_.l1.crosstrackVelocityMps()));

    ctx_.navigation.updateSpeedHeight(ctx_.tecs, ahrs_data, ctx_.baro.data(), imu_data, ctx_.airspeed.data());

    const float throttle_stick_percent = Actuator::scaleToPercent(ctx_.radio.channelThrottle());
    ctx_.navigation.updateAltitude(ctx_.tecs, ahrs_data, imu_data, ctx_.baro.data(), ctx_.airspeed.data(),
                                   static_cast<int16_t>(throttle_stick_percent), ctx_.enableThrottleNudge);

    ctx_.navigation.updateAutoAttitudeTargets(ctx_.l1, ctx_.tecs, imu_data);

    // Kendali attitude AUTO = kendali FBWA Khageswara (P/PD) yang sama dengan
    // ModeFbwa; hanya setpoint-nya dari L1/TECS, bukan stick.
    const MissionState& mission = ctx_.navigation.state();
    const AttitudeController::Output output = ctx_.attitude.updateAutoKhageswara(
        mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(), ctx_.radio.armed(), ctx_.dt_s);

    // Roll disturbance for the thesis tests: added to the aileron command
    // AFTER the attitude controller, one fixed pulse per CH7 LOW -> HIGH
    // flip. Off unless param DIST_ENABLE = 1; see
    // control/DisturbanceInjector.h for the safety rules.
    // DIST_ENABLE = 2: scripted sequence instead of CH7 (inputs below).
    DisturbanceScriptInput script_in;
    script_in.flag_wp = mission.flag_wp;
    script_in.wp_sum = mission.wp_sum;
    // along_m dihitung dari posisi saat ini (bukan dari distance_next_wp yang basi
    // di siklus pertama setelah WP diklaim, yang membuat event lurus ter-skip).
    script_in.along_m = mission.auto_state.crosstrack
                            ? mission.prev_wp_loc.get_distance(mission.current_loc)
                            : -1.0f;
    script_in.cte_m = ctx_.l1.crosstrackError();
    script_in.turn_angle_deg = mission.auto_state.next_turn_angle_deg;
    script_in.now_ms = ctx_.now_ms;
    const int16_t disturbance_us =
        ctx_.disturbance.update(true, ctx_.radio.armed(), !ctx_.radio.signalLost(),
                                ctx_.radio.channelModeBackup(), ctx_.now_ms, &script_in);
    ctx_.actuator.writeAttitude(output, disturbance_us);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);

    // Throttle AUTO manual (pilot), sama seperti FBWA/GUIDED. TECS tetap
    // menghitung pitch demand; throttle demand-nya tidak dipakai.
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
}

void ModeAuto::_exit()
{
    // Leaving AUTO ends any injected disturbance (and the offset is an
    // argument of writeAttitude(), so no other mode can ever inherit it).
    ctx_.disturbance.reset();
    // Mode lain (GUIDED, RTL) selalu memakai periode tetap NAVL1_PERIOD.
    ctx_.l1.setPeriod(ctx_.fuzzy.update(/*allowed=*/false, 0.0f, 0.0f));
    ctx_.navigation.state().auto_navigation_mode = false;
    ctx_.navigation.state().auto_throttle_mode = false;
}

bool ModeGuided::_enter()
{
    ctx_.navigation.state().auto_navigation_mode = true;
    ctx_.navigation.state().auto_throttle_mode = true;
    ctx_.attitude.resetIntegrators();
    return true;
}

void ModeGuided::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    const AhrsData& ahrs_data = ctx_.ahrs.data();
    const ImuData& imu_data = ctx_.imu.data();
    const float eas2tas = ctx_.baro.data().eas2tas;

    ctx_.navigation.navigate(/*is_guided_mode=*/true, ctx_.l1, ahrs_data, imu_data, eas2tas);
    ctx_.navigation.updateSpeedHeight(ctx_.tecs, ahrs_data, ctx_.baro.data(), imu_data, ctx_.airspeed.data());

    const float throttle_stick_percent = Actuator::scaleToPercent(ctx_.radio.channelThrottle());
    ctx_.navigation.updateAltitude(ctx_.tecs, ahrs_data, imu_data, ctx_.baro.data(), ctx_.airspeed.data(),
                                   static_cast<int16_t>(throttle_stick_percent), ctx_.enableThrottleNudge);

    ctx_.navigation.updateAutoAttitudeTargets(ctx_.l1, ctx_.tecs, imu_data);

    const MissionState& mission = ctx_.navigation.state();
    const AttitudeController::Output output =
        ctx_.attitude.update(mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(),
                            ctx_.airspeed.data().velocity_mps, ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);

    // GUIDED uses manual throttle passthrough, not TECS auto-throttle,
    // matching legacy ModeGuided_FW's plane_motors_out() (not _auto()).
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
}

void ModeGuided::_exit()
{
    ctx_.navigation.state().auto_navigation_mode = false;
    ctx_.navigation.state().auto_throttle_mode = false;
}

// ---------------------------------------------------------------------------
// RTL (prepared, not registered unless FC_RTL_MODE_ENABLE = 1)
// ---------------------------------------------------------------------------
bool ModeRtl::_enter()
{
    if (!ctx_.navigation.beginRtl()) {
        return false;  // no home yet; ModeManager restores the previous mode
    }
    ctx_.attitude.resetIntegrators();
    return true;
}

void ModeRtl::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    const AhrsData& ahrs_data = ctx_.ahrs.data();
    const ImuData& imu_data = ctx_.imu.data();
    const float eas2tas = ctx_.baro.data().eas2tas;

    ctx_.navigation.updateRtl(ctx_.l1, ahrs_data, imu_data, eas2tas);
    ctx_.navigation.updateSpeedHeight(ctx_.tecs, ahrs_data, ctx_.baro.data(), imu_data, ctx_.airspeed.data());

    const float throttle_stick_percent = Actuator::scaleToPercent(ctx_.radio.channelThrottle());
    ctx_.navigation.updateAltitude(ctx_.tecs, ahrs_data, imu_data, ctx_.baro.data(), ctx_.airspeed.data(),
                                   static_cast<int16_t>(throttle_stick_percent), ctx_.enableThrottleNudge);
    ctx_.navigation.updateAutoAttitudeTargets(ctx_.l1, ctx_.tecs, imu_data);

    const MissionState& mission = ctx_.navigation.state();
    const AttitudeController::Output output = ctx_.attitude.updateAutoKhageswara(
        mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(), ctx_.radio.armed(), ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
}

void ModeRtl::_exit()
{
    ctx_.navigation.state().auto_throttle_mode = false;
}

}  // namespace fc
