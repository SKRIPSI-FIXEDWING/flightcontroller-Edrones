#include "modes/FixedWingModes.h"

#include <AP_Math.h>

#include "FC_Config.h"
#include "vehicle/Actuator.h"

namespace fc {

namespace {

#if FC_AUTO_THROTTLE_ENABLE
/** nullptr = auto throttle aman dipakai; selain itu alasan menolak. */
const char* autoThrottleBlockReason(const VehicleContext& ctx)
{
    if (!ctx.ahrs.data().airspeed_sensor_enabled) {
        return "AUTO THR OFF: airspeed tidak sehat";
    }
    const TecsConfig& c = ctx.tecs.config();
    // Contoh nyata (tlog 2026-10-19): TECS_THR_MIN 20 / TECS_THR_MAX 10 dan
    // MAX_AIRSPEED 15 < AIRSPEED_CRUISE 18 -- throttle akan terkunci ~21%.
    if (c.max_throttle_percent < c.min_throttle_percent + 10.0f) {
        return "AUTO THR OFF: TECS_THR_MAX <= MIN+10";
    }
    if (!(c.min_airspeed_mps < c.cruise_airspeed_mps && c.cruise_airspeed_mps < c.max_airspeed_mps)) {
        return "AUTO THR OFF: urutan MIN<CRUISE<MAX ARSPD";
    }
    return nullptr;
}
#endif

/**
 * Throttle untuk mode navigasi (AUTO/LOITER/RTL): TECS bila
 * FC_AUTO_THROTTLE_ENABLE dan aman, selain itu stick pilot seperti FBWA.
 */
void writeNavigationThrottle(VehicleContext& ctx)
{
    MissionState& mission = ctx.navigation.state();
#if FC_AUTO_THROTTLE_ENABLE
    mission.auto_throttle_block_reason = autoThrottleBlockReason(ctx);
    if (mission.auto_throttle_block_reason == nullptr && mission.auto_throttle_mode) {
        const ActuatorConfig& a = ctx.actuator.config();
        const float pct = constrain_float(static_cast<float>(ctx.tecs.throttleDemand()), 0.0f, 100.0f);
        const uint16_t pwm = static_cast<uint16_t>(
            a.min_throttle_pwm + pct * 0.01f * static_cast<float>(a.max_throttle_pwm - a.min_throttle_pwm));
        ctx.actuator.writeThrottleAuto(pwm, ctx.radio.armed());
        mission.auto_throttle_engaged = true;
        return;
    }
#endif
    mission.auto_throttle_engaged = false;
    ctx.actuator.writeThrottleManual(ctx.radio.channelThrottle(), true, ctx.radio.armed());
}

}  // namespace

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
    ctx_.navigation.updateSpeedHeight(ctx_.tecs, ahrs_data, ctx_.baro.data(), imu_data, ctx_.airspeed.data());

    const float throttle_stick_percent = Actuator::scaleToPercent(ctx_.radio.channelThrottle());
    ctx_.navigation.updateAltitude(ctx_.tecs, ahrs_data, imu_data, ctx_.baro.data(), ctx_.airspeed.data(),
                                   static_cast<int16_t>(throttle_stick_percent), ctx_.enableThrottleNudge);

    ctx_.navigation.updateAutoAttitudeTargets(ctx_.l1, ctx_.tecs, imu_data);

    // Thesis's self-tuning mechanism: adjusts L1's period from crosstrack
    // error/rate when enabled; no-ops (leaves period at its configured
    // fixed value) when disabled -- see docs/fuzzy-l1-tuner.md.
    ctx_.fuzzy_tuner.update(ctx_.l1, ctx_.l1.crosstrackError(), ctx_.dt_s);

    // Kendali attitude AUTO = kendali FBWA Khageswara (P/PD) yang sama dengan
    // ModeFbwa; hanya setpoint-nya dari L1/TECS, bukan stick.
    const MissionState& mission = ctx_.navigation.state();
    const AttitudeController::Output output = ctx_.attitude.updateAutoKhageswara(
        mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(), ctx_.radio.armed(), ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);

    // Throttle: TECS bila FC_AUTO_THROTTLE_ENABLE (dan aman), selain itu
    // manual dari stick pilot seperti FBWA/GUIDED.
    writeNavigationThrottle(ctx_);
}

void ModeAuto::_exit()
{
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
    ctx_.fuzzy_tuner.update(ctx_.l1, ctx_.l1.crosstrackError(), ctx_.dt_s);

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
    ctx_.fuzzy_tuner.update(ctx_.l1, ctx_.l1.crosstrackError(), ctx_.dt_s);

    const MissionState& mission = ctx_.navigation.state();
    const AttitudeController::Output output = ctx_.attitude.updateAutoKhageswara(
        mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(), ctx_.radio.armed(), ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);
    writeNavigationThrottle(ctx_);
}

void ModeRtl::_exit()
{
    ctx_.navigation.state().auto_throttle_mode = false;
}

// ---------------------------------------------------------------------------
// LOITER
// ---------------------------------------------------------------------------
bool ModeLoiter::_enter()
{
    if (!ctx_.navigation.beginLoiter()) {
        return false;  // belum ada posisi GPS; ModeManager memulihkan mode lama
    }
    ctx_.attitude.resetIntegrators();
    return true;
}

void ModeLoiter::_update()
{
    ctx_.ahrs.update(ctx_.imu.data(), ctx_.gnss, ctx_.baro.data(), ctx_.airspeed.data());
    ctx_.navigation.updateHomeAndPosition(ctx_.ahrs, ctx_.gnss, ctx_.baro.data(), ctx_.radio.armed(),
                                          ctx_.now_ms);

    const AhrsData& ahrs_data = ctx_.ahrs.data();
    const ImuData& imu_data = ctx_.imu.data();
    const float eas2tas = ctx_.baro.data().eas2tas;

    ctx_.navigation.updateLoiter(ctx_.l1, ahrs_data, imu_data, eas2tas);
    ctx_.navigation.updateSpeedHeight(ctx_.tecs, ahrs_data, ctx_.baro.data(), imu_data, ctx_.airspeed.data());

    const float throttle_stick_percent = Actuator::scaleToPercent(ctx_.radio.channelThrottle());
    ctx_.navigation.updateAltitude(ctx_.tecs, ahrs_data, imu_data, ctx_.baro.data(), ctx_.airspeed.data(),
                                   static_cast<int16_t>(throttle_stick_percent), ctx_.enableThrottleNudge);
    ctx_.navigation.updateAutoAttitudeTargets(ctx_.l1, ctx_.tecs, imu_data);
    ctx_.fuzzy_tuner.update(ctx_.l1, ctx_.l1.crosstrackError(), ctx_.dt_s);

    const MissionState& mission = ctx_.navigation.state();
    const AttitudeController::Output output = ctx_.attitude.updateAutoKhageswara(
        mission.nav_roll_deg, mission.nav_pitch_deg, ctx_.controlImu(), ctx_.radio.armed(), ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);
    writeNavigationThrottle(ctx_);
}

void ModeLoiter::_exit()
{
    ctx_.navigation.state().auto_throttle_mode = false;
}

}  // namespace fc
