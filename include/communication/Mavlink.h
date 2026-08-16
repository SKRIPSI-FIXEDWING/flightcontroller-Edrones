#pragma once

#include <Arduino.h>
#include <common/mavlink.h>
#include <stdint.h>

#include "modes/VehicleContext.h"

namespace fc {

enum class MissionUploadState : uint8_t { Idle, Receiving };
enum class MavlinkPort : uint8_t { Usb = 0, Telemetry = 1, Companion = 2 };

struct MavlinkConfig {
    uint8_t system_id = 1;
    uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;

    unsigned long heartbeat_interval_ms = 1000;
    unsigned long attitude_interval_ms = 100;
    unsigned long altitude_interval_ms = 500;
    unsigned long gps_interval_ms = 200;
    unsigned long vfr_hud_interval_ms = 250;
    unsigned long battery_interval_ms = 5000;
    unsigned long nav_controller_interval_ms = 1000;
    unsigned long global_position_interval_ms = 200;
    unsigned long servo_output_interval_ms = 100;
    unsigned long rc_channels_interval_ms = 100;
    unsigned long scaled_pressure_interval_ms = 200;
    unsigned long mission_count_to_companion_interval_ms = 1000;

    float roll_alert_threshold_deg = 40.0f;
    unsigned long roll_alert_debounce_ms = 2000;

    static constexpr uint16_t kMaxMissionItems = MissionState::kMaxWaypoints;
};

/**
 * MAVLink telemetry + mission/param protocol handler (legacy Mavlink.h),
 * ported onto VehicleContext instead of file-scope globals. Three ports:
 * USB (Serial, GCS), Telemetry radio (Serial2), Companion computer/RPi
 * (Serial7) -- matching legacy wiring.
 *
 * Deliberately NOT ported: the ELRS-over-MAVLink RC-override path
 * (RC_CHANNELS_OVERRIDE/RC_CHANNELS/RC_CHANNELS_RAW -> applyRcToGlobals()).
 * That was the second, unarbitrated RC source flagged during planning as a
 * race condition with SBUS; per your decision (see docs/radio.md), this
 * refactor is SBUS-only.
 *
 * Also deliberately restricted: MAV_CMD_COMPONENT_ARM_DISARM is ACKed with
 * MAV_RESULT_UNSUPPORTED and a status-text explanation -- it does not arm or
 * disarm anything. Arming authority stays exclusively with the SBUS
 * transmitter switch (Radio, Phase 5). Legacy let a GCS arm the aircraft
 * directly over MAVLink with no RC involvement at all, writing the same
 * `arming` global RC did with no arbitration; adding a second write path to
 * a safety-critical flag without being asked for it is exactly the kind of
 * risk this refactor has been steering away from elsewhere (see
 * docs/mavlink.md).
 *
 * Fake test-stub battery telemetry (legacy sent an oscillating sine-wave
 * voltage, never wired to the real ADC sensor) is replaced with the real
 * Battery reading (Phase 7, vehicle/Battery.h).
 */
class Mavlink final {
public:
    explicit Mavlink(const MavlinkConfig& config = MavlinkConfig{});

    /** Parses incoming bytes from all three ports and dispatches handlers. */
    void handlePorts(VehicleContext& ctx);

    /** Periodic telemetry sender; call once per loop. */
    void update(VehicleContext& ctx);

    void notifyWaypointReached(uint16_t seq);

private:
    void mavWrite(HardwareSerial& port, const mavlink_message_t& msg);
    void mavWriteUsb(const mavlink_message_t& msg);
    void mavSendAll(const mavlink_message_t& msg);
    void replyToSender(const mavlink_message_t& msg);

    void processIncoming(VehicleContext& ctx, const mavlink_message_t& msg, MavlinkPort from);
    void forwardToOthers(const mavlink_message_t& msg, MavlinkPort from);

    void handleCommandLong(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleStatusText(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleMissionRequestList(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleMissionRequest(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleMissionRequestInt(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleMissionCount(const mavlink_message_t& msg);
    void handleMissionItemInt(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleMissionItem(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleMissionClearAll(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleSetPositionTargetLocalNed(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleSetPositionTargetGlobalInt(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleParamRequestList(VehicleContext& ctx);
    void handleParamRequestRead(VehicleContext& ctx, const mavlink_message_t& msg);
    void handleParamSet(VehicleContext& ctx, const mavlink_message_t& msg);

    void sendCommandAck(uint16_t command, uint8_t result);
    void sendStatusText(uint8_t severity, const char* text);
    void sendHeartbeat(const VehicleContext& ctx);
    void sendAlert(const char* text);
    void sendAltitude(float altitude_m);
    void sendAttitude(float roll_rad, float pitch_rad, float yaw_rad);
    void sendGpsRawInt(const VehicleContext& ctx);
    void sendBatteryStatus(const BatteryData& battery);
    void sendNavControllerOutput(const VehicleContext& ctx);
    void sendVfrHud(const VehicleContext& ctx);
    void sendScaledPressure(const VehicleContext& ctx);
    void sendMissionItemReached(uint16_t seq);
    void sendGlobalPositionInt(const VehicleContext& ctx);
    void sendServoOutputRaw(const VehicleContext& ctx);
    void sendRcChannelsRaw(const VehicleContext& ctx);
    void sendMissionCountToCompanion(const VehicleContext& ctx);
    void sendParamValue(const VehicleContext& ctx, uint16_t index);
    void pumpParamStream(VehicleContext& ctx);

    uint8_t baseModeFor(ModeId id, bool armed) const;
    uint32_t customModeFor(ModeId id) const;

    MavlinkConfig config_{};

    mavlink_status_t status_usb_{}, status_tlm_{}, status_rpi_{};
    mavlink_message_t rx_usb_{}, rx_tlm_{}, rx_rpi_{};

    MissionUploadState mission_upload_state_ = MissionUploadState::Idle;
    uint16_t expected_mission_count_ = 0;
    uint16_t current_upload_seq_ = 0;
    MavlinkPort current_port_ = MavlinkPort::Usb;

    uint16_t param_send_index_ = UINT16_MAX;

    unsigned long last_heartbeat_ms_ = 0;
    unsigned long last_attitude_ms_ = 0;
    unsigned long last_altitude_ms_ = 0;
    unsigned long last_gps_ms_ = 0;
    unsigned long last_vfr_hud_ms_ = 0;
    unsigned long last_battery_ms_ = 0;
    unsigned long last_nav_controller_ms_ = 0;
    unsigned long last_alert_ms_ = 0;
    unsigned long last_global_position_ms_ = 0;
    unsigned long last_servo_output_ms_ = 0;
    unsigned long last_rc_channels_ms_ = 0;
    unsigned long last_mission_count_to_companion_ms_ = 0;
    unsigned long last_scaled_pressure_ms_ = 0;
};

}  // namespace fc
