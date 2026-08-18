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

    // Stick-derived attitude setpoints (matches legacy roll_cmd/pitch_cmd
    // scaling from PROMPT_SIMULASI_FIXEDWING.md's documented FBWA formula),
    // tracked by the SAME LQR AttitudeController that AUTO/GUIDED use  -- 
    // there's only one attitude-control implementation now.
    const float roll_cmd_deg = 1.3f * mapStickToDeg(ctx_.radio.channelRoll(), 35.0f);
    const float pitch_cmd_deg = 1.1f * mapStickToDeg(ctx_.radio.channelPitch(), 35.0f);

    const AttitudeController::Output output = ctx_.attitude.update(roll_cmd_deg, pitch_cmd_deg, ctx_.imu.data(), ctx_.airspeed.data().velocity_mps, ctx_.dt_s);

    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.writeThrottleManual(ctx_.radio.channelThrottle(), true, ctx_.radio.armed());
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command, ctx_.radio.channelVehicleMode() > 1500);
}

bool ModeAuto::_enter()
{
    ctx_.navigation.state().auto_navigation_mode = true;
    ctx_.navigation.state().auto_throttle_mode = true;
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

    const MissionState& mission = ctx_.navigation.state();
    const AttitudeController::Output output =
        ctx_.attitude.update(mission.nav_roll_deg, mission.nav_pitch_deg, imu_data,
                            ctx_.airspeed.data().velocity_mps, ctx_.dt_s);
    ctx_.actuator.writeAttitude(output);
    ctx_.actuator.updatePayload(ctx_.radio.armed(), ctx_.payload_drop_command,
                               ctx_.radio.channelVehicleMode() > 1500);

    const float throttle_percent = static_cast<float>(ctx_.tecs.throttleDemand()) / 100.0f;
    ctx_.actuator.writeThrottleAuto(Actuator::percentToPwm(throttle_percent), ctx_.radio.armed());
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
        ctx_.attitude.update(mission.nav_roll_deg, mission.nav_pitch_deg, imu_data,
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

}  // namespace fc
