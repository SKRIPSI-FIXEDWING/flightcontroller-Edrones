#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>

#include "control/AttitudeController.h"

namespace fc {

struct ActuatorConfig {
    // Servo pin assignments. Defaults match legacy Actuator.h's CH_MTR_*
    // mapping (SERVO_AIL_L=CH_MTR_3, SERVO_ELE=CH_MTR_2, SERVO_RUD_L=CH_MTR_1,
    // SERVO_AIL_R=CH_MTR_5, SERVO_RUD_R=CH_MTR_11, SERVO_PAYLOAD=CH_MTR_6,
    // THROTTLE=CH_MTR_4). VTOL motor pins (MOTOR_1-4_PIN) are dropped —
    // fixed-wing only.
    uint8_t pin_aileron_left = 6;
    uint8_t pin_aileron_right = 12;
    uint8_t pin_elevator = 4;
    uint8_t pin_rudder_left = 2;
    uint8_t pin_rudder_right = 13;
    uint8_t pin_payload = 14;
    uint8_t pin_throttle = 10;

    uint16_t pwm_min = 988;
    uint16_t pwm_max = 2012;
    uint16_t pwm_safe_throttle = 988;

    uint16_t min_throttle_pwm = 988;
    uint16_t max_throttle_pwm = 2012;

    uint16_t pwm_payload_hold = 1000;
    uint16_t pwm_payload_drop = 2000;
    uint32_t payload_drop_duration_ms = 500;

    // angle(deg) -> PWM(us) linear fit, from legacy Actuator.h's angleToPwm().
    float angle_to_pwm_gain = 11.378f;
    uint16_t angle_to_pwm_center = 1500;
};

/**
 * Fixed-wing servo/actuator output (legacy Actuator.h, FW-relevant subset
 * only). Copter/flywing/dual-motor-yaw functions and VTOL motor init are not
 * ported — see docs/actuator.md. FBWA/AUTO attitude output now comes from
 * AttitudeController (LQR, Phase 4) instead of the legacy FW_CONTROL PID
 * class, which this module no longer depends on.
 */
class Actuator final {
public:
    explicit Actuator(const ActuatorConfig& config = ActuatorConfig{});

    void begin();

    /** MANU mode: raw stick passthrough. */
    void writeManual(uint16_t ch_roll, uint16_t ch_pitch, uint16_t ch_yaw);

    /** FBWA/GUIDED/AUTO attitude output: AttitudeController's degrees -> servo PWM. */
    void writeAttitude(const AttitudeController::Output& attitude);

    /** FBWA/MANU throttle: stick-driven, clamped to [min,max] while in FBWA. */
    void writeThrottleManual(uint16_t ch_throttle, bool is_fbwa_plane, bool armed);

    /** AUTO mode: TECS-driven throttle PWM. */
    void writeThrottleAuto(uint16_t throttle_pwm, bool armed);

    /**
     * Priority-arbitrated payload servo: MAVLink DROP command (highest
     * priority) > manual RC switch > hold. payload_drop_command is
     * consumed (reset to false) once the drop starts, matching legacy
     * payload_control_unified().
     */
    void updatePayload(bool armed, bool& payload_drop_command, bool manual_drop_switch_active);

    /** Disarmed-safe state: throttle at minimum, payload held. */
    void writeSafe();

    static uint16_t angleToPwm(float angle_deg, float gain, uint16_t center);

    /** PWM-equivalent (988-2012) -> percent (0-100), legacy scaleToPercent(). */
    static float scaleToPercent(uint16_t pwm);
    /** percent (0-100) -> PWM (988-2012), legacy PercenttoPWM(). */
    static uint16_t percentToPwm(float percent);

    // Servo PWM readback, for telemetry (MAVLink SERVO_OUTPUT_RAW).
    uint16_t aileronLeftPwm() const;
    uint16_t aileronRightPwm() const;
    uint16_t elevatorPwm() const;
    uint16_t rudderLeftPwm() const;
    uint16_t rudderRightPwm() const;
    uint16_t throttlePwm() const;
    uint16_t payloadPwm() const;

private:
    ActuatorConfig config_{};

    Servo aileron_left_{};
    Servo aileron_right_{};
    Servo elevator_{};
    Servo rudder_left_{};
    Servo rudder_right_{};
    Servo throttle_{};
    Servo payload_{};

    bool payload_dropping_ = false;
    uint32_t payload_drop_start_ms_ = 0;
};

}  // namespace fc
