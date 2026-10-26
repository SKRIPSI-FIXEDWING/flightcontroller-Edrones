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
    // Every AUTO run starts from the base L1 period (= NAVL1_PERIOD), so a
    // fuzzy run never inherits the previous run's last period, and a fixed-
    // period run starts exactly at the configured value. The disturbance
    // injector starts disarmed until the roll stick has been centered.
    ctx_.fuzzy_tuner.resetToBase(ctx_.l1);
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

    // Roll disturbance for the thesis tests: added to the aileron command
    // AFTER the attitude controller, from the roll stick (AUTO ignores the
    // stick otherwise). Off unless param DIST_ENABLE = 1; see
    // control/DisturbanceInjector.h for the safety rules.
    const int16_t disturbance_us = ctx_.disturbance.update(
        true, ctx_.radio.armed(), !ctx_.radio.signalLost(), ctx_.radio.channelRoll(), ctx_.now_ms);
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
    ctx_.navigation.state().auto_navigation_mode = false;
    ctx_.navigation.state().auto_throttle_mode = false;
}

bool ModeGuided::_enter()
{
    ctx_.navigation.state().auto_navigation_mode = true;
    ctx_.navigation.state().auto_throttle_mode = true;
    ctx_.attitude.resetIntegrators();
    ctx_.fuzzy_tuner.resetToBase(ctx_.l1);
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
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
}

void ModeRtl::_exit()
{
    ctx_.navigation.state().auto_throttle_mode = false;
}

}  // namespace fc
