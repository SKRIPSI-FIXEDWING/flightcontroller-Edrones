#include "communication/Mavlink.h"

#include <math.h>
#include <string.h>

#include "FC_Config.h"
#include "storage/ImuCalibrationStorage.h"
#include "storage/Waypoints.h"
#include "vehicle/ControlLock.h"

namespace {

bool isMissionMessage(uint32_t msgid)
{
    switch (msgid) {
        case MAVLINK_MSG_ID_MISSION_COUNT:
        case MAVLINK_MSG_ID_MISSION_ITEM:
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        case MAVLINK_MSG_ID_MISSION_ACK:
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
            return true;
        default:
            return false;
    }
}

}  // namespace

namespace fc {

Mavlink::Mavlink(const MavlinkConfig& config)
    : config_(config)
{
}

void Mavlink::mavWrite(HardwareSerial& port, const mavlink_message_t& msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    port.write(buf, len);
}

void Mavlink::mavWriteUsb(const mavlink_message_t& msg)
{
#if MAVLINK_USB_ENABLE_TX
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
#else
    (void)msg;  // FC_DEBUG_SERIAL_ENABLE has USB Serial carrying plain text instead -- see FC_Config.h.
#endif
}

void Mavlink::mavSendAll(const mavlink_message_t& msg)
{
    mavWriteUsb(msg);
    mavWrite(Serial2, msg);
    if (mission_upload_state_ != MissionUploadState::Receiving || isMissionMessage(msg.msgid)) {
        mavWrite(Serial7, msg);
    }
}

void Mavlink::replyToSender(const mavlink_message_t& msg)
{
    switch (current_port_) {
        case MavlinkPort::Usb: mavWriteUsb(msg); break;
        case MavlinkPort::Telemetry: mavWrite(Serial2, msg); break;
        case MavlinkPort::Companion: mavWrite(Serial7, msg); break;
    }
}

void Mavlink::handlePorts(VehicleContext& ctx)
{
#if MAVLINK_USB_ENABLE_RX
    // FC_DEBUG_SERIAL_ENABLE==0 (FC_Config.h) required for this: otherwise
    // Serial carries plain-text debug output instead, and read()ing MAVLink
    // bytes here would just be reading and discarding that text stream.
    while (Serial.available()) {
        const uint8_t c = Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &rx_usb_, &status_usb_)) {
            processIncoming(ctx, rx_usb_, MavlinkPort::Usb);
            forwardToOthers(rx_usb_, MavlinkPort::Usb);
        }
    }
#endif

    while (Serial2.available()) {
        const uint8_t c = Serial2.read();
        if (mavlink_parse_char(MAVLINK_COMM_1, c, &rx_tlm_, &status_tlm_)) {
            processIncoming(ctx, rx_tlm_, MavlinkPort::Telemetry);
            forwardToOthers(rx_tlm_, MavlinkPort::Telemetry);
        }
    }

    while (Serial7.available()) {
        const uint8_t c = Serial7.read();
        if (mavlink_parse_char(MAVLINK_COMM_2, c, &rx_rpi_, &status_rpi_)) {
            processIncoming(ctx, rx_rpi_, MavlinkPort::Companion);
            forwardToOthers(rx_rpi_, MavlinkPort::Companion);
        }
    }
}

void Mavlink::processIncoming(VehicleContext& ctx, const mavlink_message_t& msg, MavlinkPort from)
{
    current_port_ = from;

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG: handleCommandLong(ctx, msg); break;
        case MAVLINK_MSG_ID_SET_MODE: handleSetMode(ctx, msg); break;
        case MAVLINK_MSG_ID_STATUSTEXT: handleStatusText(ctx, msg); break;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: handleSetPositionTargetLocalNed(ctx, msg); break;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: handleSetPositionTargetGlobalInt(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: handleMissionRequestList(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_REQUEST: handleMissionRequest(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: handleMissionRequestInt(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_COUNT: handleMissionCount(msg); break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: handleMissionItemInt(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_ITEM: handleMissionItem(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: handleMissionClearAll(ctx, msg); break;
        case MAVLINK_MSG_ID_MISSION_SET_CURRENT: handleMissionSetCurrent(ctx, msg); break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: handleParamRequestList(ctx); break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: handleParamRequestRead(ctx, msg); break;
        case MAVLINK_MSG_ID_PARAM_SET: handleParamSet(ctx, msg); break;
        default: break;
    }
}

void Mavlink::forwardToOthers(const mavlink_message_t& msg, MavlinkPort from)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_MISSION_COUNT:
        case MAVLINK_MSG_ID_MISSION_ITEM:
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        case MAVLINK_MSG_ID_MISSION_ACK:
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_SET:
        case MAVLINK_MSG_ID_PARAM_VALUE:
            return;
        default:
            break;
    }

    if (from != MavlinkPort::Telemetry) mavWrite(Serial2, msg);
    if (from != MavlinkPort::Companion) mavWrite(Serial7, msg);
}

void Mavlink::sendCommandAck(uint16_t command, uint8_t result)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(config_.system_id, config_.component_id, &msg, command, result, 0, 0, 0, 0);
    replyToSender(msg);
}

void Mavlink::sendStatusText(uint8_t severity, const char* text)
{
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(config_.system_id, config_.component_id, &msg, severity, text);
    mavSendAll(msg);
}

void Mavlink::sendNamedValueFloat(const char* name, float value)
{
    mavlink_message_t msg;
    mavlink_msg_named_value_float_pack(config_.system_id, config_.component_id, &msg, millis(), name, value);
    mavSendAll(msg);
}

void Mavlink::sendMahonyAttitude(float roll_deg, float pitch_deg, float yaw_deg)
{
    sendNamedValueFloat("MHN_ROLL", roll_deg);
    sendNamedValueFloat("MHN_PITCH", pitch_deg);
    sendNamedValueFloat("MHN_YAW", yaw_deg);
}

void Mavlink::sendAltitudeFusionDiagnostics(float fused_m, float baro_only_m, float baro_bias_m,
                                            bool baro_rejected, bool gps_active)
{
    sendNamedValueFloat("ALT_FUSED", fused_m);
    sendNamedValueFloat("ALT_BARO", baro_only_m);
    sendNamedValueFloat("ALT_BBIAS", baro_bias_m);
    sendNamedValueFloat("ALT_REJ", baro_rejected ? 1.0f : 0.0f);
    sendNamedValueFloat("ALT_GPS", gps_active ? 1.0f : 0.0f);
}

void Mavlink::handleStatusText(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_statustext_t status;
    mavlink_msg_statustext_decode(&msg, &status);

    if (msg.sysid == config_.system_id && msg.compid == config_.component_id) {
        return;
    }

    if (strncmp(status.text, "DROP", 4) == 0) {
        ctx.payload_drop_command = true;
        sendStatusText(MAV_SEVERITY_INFO, "PAYLOAD DROP TRIGGERED");
    }
}

uint8_t Mavlink::baseModeFor(ModeId id, bool armed) const
{
    uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    switch (id) {
        case ModeId::Manual:
            base |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            break;
        case ModeId::Fbwa:
            base |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
            break;
        case ModeId::Guided:
            base |= MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
            break;
        case ModeId::Auto:
        case ModeId::Rtl:
        case ModeId::Loiter:
            base |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
            break;
        default:
            base |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            break;
    }
    if (armed) {
        base |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    return base;
}

uint32_t Mavlink::customModeFor(ModeId id) const
{
    // ArduPlane-compatible custom_mode codes, so GCS mode dropdowns show
    // sensible names: 0=MANUAL, 5=FBWA, 10=AUTO, 4=GUIDED.
    switch (id) {
        case ModeId::Manual: return 0;
        case ModeId::Fbwa: return 5;
        case ModeId::Auto: return 10;
        case ModeId::Guided: return 4;
        case ModeId::Rtl: return 11;
        case ModeId::Loiter: return 12;
        default: return 0;
    }
}

uint8_t Mavlink::applyModeRequest(VehicleContext& ctx, uint32_t custom_mode)
{
    ModeId new_mode_id;
    const char* mode_name = "UNKNOWN";

    if (custom_mode == 0) { new_mode_id = ModeId::Manual; mode_name = "MANUAL"; }
    else if (custom_mode == 5) { new_mode_id = ModeId::Fbwa; mode_name = "FBWA"; }
    else if (custom_mode == 10) { new_mode_id = ModeId::Auto; mode_name = "AUTO"; }
    else if (custom_mode == 4) { new_mode_id = ModeId::Guided; mode_name = "GUIDED"; }
    else if (custom_mode == 12) { new_mode_id = ModeId::Loiter; mode_name = "LOITER"; }
#if FC_RTL_MODE_ENABLE
    else if (custom_mode == 11) { new_mode_id = ModeId::Rtl; mode_name = "RTL"; }
#endif
    else {
        sendStatusText(MAV_SEVERITY_WARNING, "MODE NOT SUPPORTED");
        return MAV_RESULT_UNSUPPORTED;
    }

    // Honest result: ACCEPTED only if the mode really changed. The
    // switch runs under the Control lock so it cannot interleave
    // with ModeManager::update() on the Control task.
    bool mode_changed = false;
    {
        ControlLock lock;
        if (lock.acquired()) {
            mode_changed = ctx.mode_manager.setMode(new_mode_id);
        }
    }

    if (!mode_changed) {
        const char* reason = (new_mode_id == ModeId::Auto || new_mode_id == ModeId::Rtl ||
                              new_mode_id == ModeId::Loiter)
                                 ? ctx.navigation.autoRejectReason()
                                 : nullptr;
        sendStatusText(MAV_SEVERITY_WARNING, (reason != nullptr) ? reason : "MODE SET FAILED");
        return MAV_RESULT_FAILED;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "MODE: %s", mode_name);
    sendStatusText(MAV_SEVERITY_INFO, buf);
    sendHeartbeat(ctx);  // Immediate heartbeat so GCS mode display updates without delay.
    last_heartbeat_ms_ = millis();
    return MAV_RESULT_ACCEPTED;
}

void Mavlink::handleSetMode(VehicleContext& ctx, const mavlink_message_t& msg)
{
    // Mission Planner (dropdown mode di HUD / tombol Set Mode / Actions)
    // mengirim pesan SET_MODE (#11), BUKAN COMMAND_LONG DO_SET_MODE. Tanpa
    // handler ini pilihan mode dari GCS diabaikan diam-diam (log 2026-10-19 C:
    // LOITER dipilih dari Mission Planner tetapi heartbeat tidak pernah
    // melaporkan mode 12). Tidak ada ACK untuk SET_MODE; hasilnya terlihat
    // dari HEARTBEAT dan STATUSTEXT.
    mavlink_set_mode_t sm;
    mavlink_msg_set_mode_decode(&msg, &sm);
    if (sm.target_system != config_.system_id) {
        return;
    }
    applyModeRequest(ctx, sm.custom_mode);
}

void Mavlink::handleCommandLong(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    if (cmd.target_system != config_.system_id && cmd.target_system != 0) {
        return;
    }

    uint8_t result = MAV_RESULT_UNSUPPORTED;

    switch (cmd.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            // param1: 1 = arm, 0 = disarm. param2 = 21196 forces (ArduPilot
            // convention). The RC arm switch is edge-triggered, so this
            // command is not overwritten by the next RC frame; moving the
            // RC switch always takes over.
            const bool arm = cmd.param1 >= 0.5f;
            const bool force = fabsf(cmd.param2 - 21196.0f) < 1.0f;

            if (arm) {
                if (!force && ctx.gnss.fix_type == GnssFixType::NoFix) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "ARM DENIED: NO GPS FIX (force: param2=21196)");
                } else if (!force && ctx.radio.channelThrottle() > 1100) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "ARM DENIED: THROTTLE NOT LOW");
                } else {
                    ctx.radio.setArmedFromGcs(true);
                    result = MAV_RESULT_ACCEPTED;
                    sendStatusText(MAV_SEVERITY_INFO, force ? "ARMED (FORCE)" : "ARMED");
                    sendHeartbeat(ctx);
                    last_heartbeat_ms_ = millis();
                }
            } else {
                // Refuse to cut the motor in flight unless forced.
                const bool airborne = ctx.gnss.ground_speed_mps > 5.0f;
                if (airborne && !force) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "DISARM DENIED: MOVING (force: param2=21196)");
                } else {
                    ctx.radio.setArmedFromGcs(false);
                    result = MAV_RESULT_ACCEPTED;
                    sendStatusText(MAV_SEVERITY_INFO, "DISARMED");
                    sendHeartbeat(ctx);
                    last_heartbeat_ms_ = millis();
                }
            }
            break;
        }

        case MAV_CMD_MISSION_START: {
            // param1 = first item (0 -> item 1: item 0 is HOME). Needs armed.
            if (!ctx.radio.armed()) {
                result = MAV_RESULT_DENIED;
                sendStatusText(MAV_SEVERITY_WARNING, "MISSION START DENIED: DISARMED");
                break;
            }
            int first_item = static_cast<int>(cmd.param1);
            if (first_item < 1) {
                first_item = 1;
            }

            bool ok = false;
            {
                ControlLock lock;
                if (lock.acquired()) {
                    ok = ctx.navigation.setCurrentWaypoint(first_item) ||
                         ctx.navigation.setCurrentWaypoint(1);
                    if (ok) {
                        ok = ctx.mode_manager.setMode(ModeId::Auto);
                    }
                }
            }
            if (ok) {
                result = MAV_RESULT_ACCEPTED;
                sendStatusText(MAV_SEVERITY_INFO, "MISSION START");
                sendHeartbeat(ctx);
                last_heartbeat_ms_ = millis();
            } else {
                result = MAV_RESULT_FAILED;
                const char* reason = ctx.navigation.autoRejectReason();
                sendStatusText(MAV_SEVERITY_WARNING, (reason != nullptr) ? reason : "MISSION START FAILED");
            }
            break;
        }

        case MAV_CMD_DO_SET_MODE:
            result = applyModeRequest(ctx, static_cast<uint32_t>(cmd.param2));
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION: {
            // param5 (accelerometer): 1=full 6-position, 2=board level/trim,
            // 4=simple 1-position -- per MAV_CMD_PREFLIGHT_CALIBRATION's
            // PREFLIGHT_CALIBRATION_ACCELEROMETER enum. Mission Planner's
            // "Calibrate Level" button sends param5=2. Only that one is
            // implemented so far: it needs just one MAVLink round trip and
            // maps directly onto Imu's existing roll_trim_deg/pitch_trim_deg
            // (see docs/imu-bno055.md). "Calibrate Accel" (param5=1, needs a
            // MAV_CMD_ACCELCAL_VEHICLE_POS handshake walking 6 positions) and
            // "Simple Accel Cal" (param5=4, writes BNO055's own accel offset
            // registers) are not implemented yet.
            if (cmd.param5 == 2.0f) {
                if (ctx.radio.armed()) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "LEVEL CAL: disarm first");
                    break;
                }

                const ImuData& imu_data = ctx.imu.data();
                if (!imu_data.valid) {
                    result = MAV_RESULT_TEMPORARILY_REJECTED;
                    sendStatusText(MAV_SEVERITY_WARNING, "LEVEL CAL: no IMU data yet");
                    break;
                }

                // Current roll_deg/pitch_deg already have the old trim
                // subtracted, so adding the residual to the old trim makes
                // the next reading land on zero. See Imu::readData().
                ctx.imu.rollTrimDegRef() += imu_data.roll_deg;
                ctx.imu.pitchTrimDegRef() += imu_data.pitch_deg;
                ctx.params.save();

                result = MAV_RESULT_ACCEPTED;
                char buf[50];
                snprintf(buf, sizeof(buf), "LEVEL CAL: trim R%.1f P%.1f", ctx.imu.rollTrimDegRef(),
                         ctx.imu.pitchTrimDegRef());
                sendStatusText(MAV_SEVERITY_INFO, buf);
                break;
            }

            if (cmd.param5 == 1.0f) {
                // TEMPORARY repurposing of the "Calibrate Accel" button,
                // pending the real 6-position flow (see docs/imu-bno055.md).
                // A bad accel/mag/gyro offset snapshot in
                // storage/ImuCalibrationStorage.h -- captured once, then
                // force-written into the chip on every boot -- can corrupt
                // BNO055's own on-chip fusion badly enough to produce
                // cross-axis coupling (roll/pitch moving during a pure yaw)
                // even through the Euler registers, not just our own
                // quaternion math. This clears the saved snapshot AND
                // zeroes the chip's live offset registers immediately, so
                // the next re-calibration starts clean. See
                // docs/imu-bno055.md's entry for this incident.
                if (ctx.radio.armed()) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "IMU CAL CLEAR: disarm first");
                    break;
                }

                ImuCalibrationStorage::clear();
                const ImuCalibrationOffsets zero_offsets{};
                if (ctx.imu.writeCalibrationOffsets(zero_offsets)) {
                    result = MAV_RESULT_ACCEPTED;
                    sendStatusText(MAV_SEVERITY_INFO, "IMU CAL CLEARED - power-cycle, re-test level");
                } else {
                    result = MAV_RESULT_FAILED;
                    sendStatusText(MAV_SEVERITY_ERROR, "IMU CAL CLEAR: chip write failed");
                }
                break;
            }

            if (cmd.param5 == 4.0f) {
                result = MAV_RESULT_UNSUPPORTED;
                sendStatusText(MAV_SEVERITY_WARNING, "ACCEL CAL: not implemented, use Calibrate Level");
                break;
            }

            // Ground-pressure (baro) calibration -- Mission Planner's
            // "Calibrate Baro" (HUD right-click) and the equivalent QGC
            // action send param3=1. Other sensors (gyro/mag/radio/ESC) have
            // no calibration routine in this build.
            if (cmd.param3 != 1.0f) {
                result = MAV_RESULT_UNSUPPORTED;
                sendStatusText(MAV_SEVERITY_WARNING, "CAL: only baro/level supported");
                break;
            }

            if (ctx.radio.armed()) {
                result = MAV_RESULT_DENIED;
                sendStatusText(MAV_SEVERITY_WARNING, "BARO CAL: disarm first");
                break;
            }

            if (ctx.baro.recalibrateGroundPressure()) {
                result = MAV_RESULT_ACCEPTED;
                sendStatusText(MAV_SEVERITY_INFO, "BARO CAL: ground pressure re-zeroed");
            } else {
                result = MAV_RESULT_FAILED;
                sendStatusText(MAV_SEVERITY_ERROR, "BARO CAL: sensor read failed");
            }
            break;
        }

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {
            // param1: 0=nothing, 1=reboot autopilot, 2=shutdown, 3=reboot to
            // bootloader. Teensy 4.1 cannot power itself off, so shutdown is
            // unsupported. Mission Planner's "Preflight_Reboot_Shutdown"
            // action sends param1=1.
            const int action = static_cast<int>(cmd.param1);
            if (action == 0) {
                result = MAV_RESULT_ACCEPTED;
                break;
            }
            if (action != 1 && action != 3) {
                result = MAV_RESULT_UNSUPPORTED;
                sendStatusText(MAV_SEVERITY_WARNING, "REBOOT: only reboot supported");
                break;
            }
            if (ctx.radio.armed()) {
                result = MAV_RESULT_DENIED;
                sendStatusText(MAV_SEVERITY_WARNING, "REBOOT DENIED: disarm first");
                break;
            }

            // ACK first so the GCS does not retry, let the UART/USB drain,
            // then reset the core. Never returns.
            sendStatusText(MAV_SEVERITY_INFO, (action == 3) ? "REBOOT TO BOOTLOADER" : "REBOOTING");
            sendCommandAck(cmd.command, MAV_RESULT_ACCEPTED);
            Serial2.flush();
            Serial7.flush();
            delay(200);
            if (action == 3) {
                _reboot_Teensyduino_();  // Teensy HalfKay bootloader
            }
            SCB_AIRCR = 0x05FA0004;      // SYSRESETREQ
            while (true) {
            }
        }

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
    }

    sendCommandAck(cmd.command, result);
}

void Mavlink::handleMissionRequestList(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_message_t reply;
    mavlink_msg_mission_count_pack(config_.system_id, config_.component_id, &reply, msg.sysid, msg.compid,
                                   ctx.navigation.state().wp_sum, MAV_MISSION_TYPE_MISSION);
    replyToSender(reply);
}

namespace {
// Simpan jenis perintah + parameternya supaya Read WP mengembalikan item yang
// sama. param2 hanya dipakai sebagai radius klaim untuk NAV_WAYPOINT (di
// LOITER_* param2 bukan radius klaim).
void storeItemCommand(MissionState& mission, uint16_t seq, uint16_t command, float p1, float p2, float p3)
{
    const bool is_loiter = (command == MAV_CMD_NAV_LOITER_UNLIM || command == MAV_CMD_NAV_LOITER_TURNS ||
                            command == MAV_CMD_NAV_LOITER_TIME);
    mission.waypoint_cmd[seq] = command;
    mission.waypoint_p1[seq] = (command == MAV_CMD_NAV_WAYPOINT || is_loiter) ? p1 : 0.0f;
    mission.waypoint_p3[seq] = is_loiter ? p3 : 0.0f;
    mission.waypoint_radius_m[seq] = (!is_loiter && p2 > 0.0f) ? p2 : 0.0f;
}
}  // namespace

void Mavlink::handleMissionRequest(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_mission_request_t request;
    mavlink_msg_mission_request_decode(&msg, &request);
    MissionState& mission = ctx.navigation.state();
    if (request.seq >= mission.wp_sum) {
        return;
    }

    mavlink_message_t reply;
    const uint16_t item_cmd = (mission.waypoint_cmd[request.seq] != 0) ? mission.waypoint_cmd[request.seq]
                                                                       : MAV_CMD_NAV_WAYPOINT;
    mavlink_msg_mission_item_int_pack(config_.system_id, config_.component_id, &reply, msg.sysid, msg.compid,
                                      request.seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, item_cmd, 0, 1,
                                      mission.waypoint_p1[request.seq], mission.waypoint_radius_m[request.seq],
                                      mission.waypoint_p3[request.seq], 0, mission.waypoint[request.seq].lat,
                                      mission.waypoint[request.seq].lng, mission.waypoint[request.seq].alt / 100.0f,
                                      MAV_MISSION_TYPE_MISSION);
    replyToSender(reply);

    if (request.seq == mission.wp_sum - 1) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
    }
}

void Mavlink::handleMissionRequestInt(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_mission_request_int_t request;
    mavlink_msg_mission_request_int_decode(&msg, &request);
    MissionState& mission = ctx.navigation.state();
    if (request.seq >= mission.wp_sum) {
        return;
    }

    mavlink_message_t reply;
    const uint16_t item_cmd = (mission.waypoint_cmd[request.seq] != 0) ? mission.waypoint_cmd[request.seq]
                                                                       : MAV_CMD_NAV_WAYPOINT;
    mavlink_msg_mission_item_int_pack(config_.system_id, config_.component_id, &reply, msg.sysid, msg.compid,
                                      request.seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, item_cmd, 0, 1,
                                      mission.waypoint_p1[request.seq], mission.waypoint_radius_m[request.seq],
                                      mission.waypoint_p3[request.seq], 0, mission.waypoint[request.seq].lat,
                                      mission.waypoint[request.seq].lng, mission.waypoint[request.seq].alt / 100.0f,
                                      MAV_MISSION_TYPE_MISSION);
    replyToSender(reply);

    if (request.seq == mission.wp_sum - 1) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
    }
}

void Mavlink::handleMissionCount(const mavlink_message_t& msg)
{
    mavlink_mission_count_t count_msg;
    mavlink_msg_mission_count_decode(&msg, &count_msg);
    expected_mission_count_ = count_msg.count;
    mission_upload_completed_ = false;

    if (expected_mission_count_ > MavlinkConfig::kMaxMissionItems) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ERROR, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        sendStatusText(MAV_SEVERITY_WARNING, "Mission count too large");
        return;
    }

    mission_upload_state_ = MissionUploadState::Receiving;
    current_upload_seq_ = 0;

    mavlink_message_t req;
    mavlink_msg_mission_request_int_pack(config_.system_id, config_.component_id, &req, msg.sysid, msg.compid, 0,
                                         MAV_MISSION_TYPE_MISSION);
    replyToSender(req);
}

bool Mavlink::resendFinalMissionAck(uint16_t seq, const mavlink_message_t& msg)
{
    // Sama seperti ArduPilot: jika ACK akhir hilang di radio, GCS mengirim
    // ulang item terakhir. Balas ACK lagi (tanpa menyimpan ulang) supaya
    // Mission Planner tidak timeout padahal misi sudah tersimpan. Hanya dalam
    // kMissionAckResendWindowMs setelah upload selesai.
    if (!mission_upload_completed_ || expected_mission_count_ == 0 || seq + 1 != expected_mission_count_) {
        return false;
    }
    if (millis() - mission_upload_completed_ms_ > MavlinkConfig::kMissionAckResendWindowMs) {
        // Sudah lewat jendela waktu: item basi, abaikan saja.
        mission_upload_completed_ = false;
        return false;
    }
    mavlink_message_t ack;
    mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                 MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    replyToSender(ack);
    return true;
}

void Mavlink::handleMissionItemInt(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&msg, &item);

    if (mission_upload_state_ != MissionUploadState::Receiving) {
        resendFinalMissionAck(item.seq, msg);
        return;
    }

    if (item.seq < current_upload_seq_) {
        // Item duplikat (GCS mengirim ulang karena request kita hilang di
        // radio): abaikan dan minta ulang seq yang ditunggu, seperti ArduPilot.
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(config_.system_id, config_.component_id, &req, msg.sysid, msg.compid,
                                             current_upload_seq_, MAV_MISSION_TYPE_MISSION);
        replyToSender(req);
        return;
    }
    if (item.seq > current_upload_seq_) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_INVALID_SEQUENCE, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        mission_upload_state_ = MissionUploadState::Idle;
        sendStatusText(MAV_SEVERITY_WARNING, "Invalid mission sequence");
        return;
    }

    MissionState& mission = ctx.navigation.state();
    {
        // Write under the Control lock: Control reads waypoint[]/wp_sum.
        ControlLock lock;
        if (!lock.acquired() || item.seq >= MissionState::kMaxWaypoints) {
            mavlink_message_t ack;
            mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                         MAV_MISSION_ERROR, MAV_MISSION_TYPE_MISSION);
            replyToSender(ack);
            mission_upload_state_ = MissionUploadState::Idle;
            sendStatusText(MAV_SEVERITY_WARNING, "Mission upload busy/invalid");
            return;
        }
        mission.waypoint[item.seq].lat = item.x;
        mission.waypoint[item.seq].lng = item.y;
        mission.waypoint[item.seq].alt = static_cast<int32_t>(item.z * 100.0f);
        storeItemCommand(mission, item.seq, item.command, item.param1, item.param2, item.param3);
    }
    current_upload_seq_++;

    if (current_upload_seq_ < expected_mission_count_) {
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(config_.system_id, config_.component_id, &req, msg.sysid, msg.compid,
                                             current_upload_seq_, MAV_MISSION_TYPE_MISSION);
        replyToSender(req);
    } else {
        {
            ControlLock lock;
            mission.wp_sum = expected_mission_count_;
            mission.flag_wp = -1;
            mission.mission_complete = false;
        }
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        mission_upload_state_ = MissionUploadState::Idle;
        mission_upload_completed_ = true;
        mission_upload_completed_ms_ = millis();
        sendStatusText(MAV_SEVERITY_INFO, "Mission upload complete");
        Waypoints::save(mission);
    }
}

void Mavlink::handleMissionItem(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&msg, &item);

    if (mission_upload_state_ != MissionUploadState::Receiving) {
        resendFinalMissionAck(item.seq, msg);
        return;
    }

    if (item.seq < current_upload_seq_) {
        // Item duplikat (GCS mengirim ulang karena request kita hilang di
        // radio): abaikan dan minta ulang seq yang ditunggu, seperti ArduPilot.
        mavlink_message_t req;
        mavlink_msg_mission_request_pack(config_.system_id, config_.component_id, &req, msg.sysid, msg.compid,
                                         current_upload_seq_, MAV_MISSION_TYPE_MISSION);
        replyToSender(req);
        return;
    }
    if (item.seq > current_upload_seq_) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_INVALID_SEQUENCE, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        mission_upload_state_ = MissionUploadState::Idle;
        sendStatusText(MAV_SEVERITY_WARNING, "Invalid mission sequence");
        return;
    }

    MissionState& mission = ctx.navigation.state();
    {
        ControlLock lock;
        if (!lock.acquired() || item.seq >= MissionState::kMaxWaypoints) {
            mavlink_message_t ack;
            mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                         MAV_MISSION_ERROR, MAV_MISSION_TYPE_MISSION);
            replyToSender(ack);
            mission_upload_state_ = MissionUploadState::Idle;
            sendStatusText(MAV_SEVERITY_WARNING, "Mission upload busy/invalid");
            return;
        }
        mission.waypoint[item.seq].lat = static_cast<int32_t>(item.x * 1e7);
        mission.waypoint[item.seq].lng = static_cast<int32_t>(item.y * 1e7);
        mission.waypoint[item.seq].alt = static_cast<int32_t>(item.z * 100.0f);
        storeItemCommand(mission, item.seq, item.command, item.param1, item.param2, item.param3);
    }
    current_upload_seq_++;

    if (current_upload_seq_ < expected_mission_count_) {
        mavlink_message_t req;
        mavlink_msg_mission_request_pack(config_.system_id, config_.component_id, &req, msg.sysid, msg.compid,
                                         current_upload_seq_, MAV_MISSION_TYPE_MISSION);
        replyToSender(req);
    } else {
        {
            ControlLock lock;
            mission.wp_sum = expected_mission_count_;
            mission.flag_wp = -1;
            mission.mission_complete = false;
        }
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        replyToSender(ack);
        mission_upload_state_ = MissionUploadState::Idle;
        mission_upload_completed_ = true;
        mission_upload_completed_ms_ = millis();
        sendStatusText(MAV_SEVERITY_INFO, "Mission upload complete");
        Waypoints::save(mission);
    }
}

void Mavlink::handleMissionClearAll(VehicleContext& ctx, const mavlink_message_t& msg)
{
    MissionState& mission = ctx.navigation.state();
    {
        ControlLock lock;
        mission.wp_sum = 0;
        mission.flag_wp = -1;
        mission.mission_complete = false;
        for (float& radius : mission.waypoint_radius_m) {
            radius = 0.0f;
        }
        for (uint16_t& c : mission.waypoint_cmd) {
            c = 0;
        }
        for (float& v : mission.waypoint_p1) {
            v = 0.0f;
        }
        for (float& v : mission.waypoint_p3) {
            v = 0.0f;
        }
    }
    Waypoints::clear();
    mission_upload_completed_ = false;

    mavlink_message_t ack;
    mavlink_msg_mission_ack_pack(config_.system_id, config_.component_id, &ack, msg.sysid, msg.compid,
                                 MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    replyToSender(ack);
    sendStatusText(MAV_SEVERITY_INFO, "Missions cleared");
}

void Mavlink::handleMissionSetCurrent(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_mission_set_current_t set_current;
    mavlink_msg_mission_set_current_decode(&msg, &set_current);

    bool ok = false;
    {
        ControlLock lock;
        if (lock.acquired()) {
            // Item 0 is HOME and never a flight target: 0 maps to item 1.
            const int seq = (set_current.seq < 1) ? 1 : static_cast<int>(set_current.seq);
            ok = ctx.navigation.setCurrentWaypoint(seq);
        }
    }

    if (ok) {
        mavlink_message_t reply;
        mavlink_msg_mission_current_pack(config_.system_id, config_.component_id, &reply,
                                         static_cast<uint16_t>(ctx.navigation.state().flag_wp));
        replyToSender(reply);
        sendStatusText(MAV_SEVERITY_INFO, "WP SET");
    } else {
        sendStatusText(MAV_SEVERITY_WARNING, "SET WP FAILED: OUT OF RANGE");
    }
}

void Mavlink::handleSetPositionTargetLocalNed(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_set_position_target_local_ned_t sp;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &sp);

    constexpr uint16_t kIgnoreX = 1U << 0;
    constexpr uint16_t kIgnoreY = 1U << 1;
    constexpr uint16_t kIgnoreZ = 1U << 2;
    if ((sp.type_mask & (kIgnoreX | kIgnoreY | kIgnoreZ)) != 0) {
        return;
    }

    MissionState& mission = ctx.navigation.state();
    Locations origin = mission.home_wp_loc;
    if (!origin.initialised()) {
        origin = mission.current_loc;
    }

    Locations target = origin;
    target.offset(sp.x, sp.y);
    target.alt = origin.alt - static_cast<int32_t>(lroundf(sp.z * 100.0f));

    mission.rpi_external_setpoint_target = target;
    mission.rpi_external_setpoint_last_ms = millis();
    mission.rpi_external_setpoint_active = true;
}

void Mavlink::handleSetPositionTargetGlobalInt(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_set_position_target_global_int_t sp;
    mavlink_msg_set_position_target_global_int_decode(&msg, &sp);

    constexpr uint16_t kIgnoreX = 1U << 0;
    constexpr uint16_t kIgnoreY = 1U << 1;
    constexpr uint16_t kIgnoreZ = 1U << 2;
    if ((sp.type_mask & (kIgnoreX | kIgnoreY | kIgnoreZ)) != 0) {
        return;
    }

    Locations target{};
    target.lat = sp.lat_int;
    target.lng = sp.lon_int;
    target.alt = static_cast<int32_t>(lroundf(sp.alt * 100.0f));

    MissionState& mission = ctx.navigation.state();
    mission.rpi_external_setpoint_target = target;
    mission.rpi_external_setpoint_last_ms = millis();
    mission.rpi_external_setpoint_active = true;
}

bool Mavlink::autoIsCircling(const VehicleContext& ctx) const
{
    // AUTO yang sedang berputar (item LOITER_* sesudah tertangkap, atau loiter
    // akhir misi): data loiter diperbarui < 500 ms lalu.
    if (ctx.mode_manager.current() == nullptr || ctx.mode_manager.current()->id() != ModeId::Auto) {
        return false;
    }
    const MissionState& mission = ctx.navigation.state();
    return mission.loiter_last_ms != 0 && millis() - mission.loiter_last_ms < 500;
}

void Mavlink::sendHeartbeat(const VehicleContext& ctx)
{
    ModeId mode_id = (ctx.mode_manager.current() != nullptr) ? ctx.mode_manager.current()->id() : ModeId::Manual;
    // Dilaporkan sebagai LOITER (custom_mode 12) selama AUTO menjalankan item
    // loiter, supaya Mission Planner/tlog menampilkan LOITER sesuai misi.
    // Mode internal TETAP AUTO: misi lanjut ke WP berikutnya setelah loiter
    // selesai, dan saklar RC tidak terpengaruh.
    if (autoIsCircling(ctx)) {
        mode_id = ModeId::Loiter;
    }
    const bool armed = ctx.radio.armed();

    mavlink_message_t msg;
    const uint8_t system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    mavlink_msg_heartbeat_pack(config_.system_id, config_.component_id, &msg, MAV_TYPE_FIXED_WING,
                              MAV_AUTOPILOT_ARDUPILOTMEGA, baseModeFor(mode_id, armed), customModeFor(mode_id),
                              system_status);
    mavSendAll(msg);
}

void Mavlink::sendAlert(const char* text)
{
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(config_.system_id, config_.component_id, &msg, MAV_SEVERITY_WARNING, text);
    mavSendAll(msg);
}

void Mavlink::sendAltitude(float altitude_m)
{
    mavlink_message_t msg;
    const uint64_t t_usec = static_cast<uint64_t>(millis()) * 1000ULL;
    mavlink_msg_altitude_pack(config_.system_id, config_.component_id, &msg, t_usec, altitude_m, altitude_m, 0,
                             altitude_m, 0, 0.0f);
    mavSendAll(msg);
}

void Mavlink::sendAttitude(float roll_rad, float pitch_rad, float yaw_rad)
{
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(config_.system_id, config_.component_id, &msg, millis(), roll_rad, pitch_rad,
                              yaw_rad, 0, 0, 0);
    mavSendAll(msg);
}

void Mavlink::sendGpsRawInt(const VehicleContext& ctx)
{
    const GnssFixData& gnss = ctx.gnss;
    mavlink_message_t msg;
    const uint64_t t_usec = static_cast<uint64_t>(millis()) * 1000ULL;
    // GnssFixType is 0/1/2 (None/2D/3D); MAVLink GPS_FIX_TYPE is 1/2/3
    // (NO_FIX/2D_FIX/3D_FIX, 0 = NO_GPS). Sending the enum raw made a real
    // 3D fix show up as "2D" in the GCS (seen in the 2026-10-13 tlog).
    uint8_t fix_type = GPS_FIX_TYPE_NO_FIX;
    switch (gnss.fix_type) {
        case GnssFixType::Fix3D:
            fix_type = GPS_FIX_TYPE_3D_FIX;
            break;
        case GnssFixType::Fix2D:
            fix_type = GPS_FIX_TYPE_2D_FIX;
            break;
        case GnssFixType::NoFix:
        default:
            break;
    }
    const uint16_t cog_centi =
        (gnss.fix_type != GnssFixType::NoFix) ? static_cast<uint16_t>(gnss.course_deg * 100.0f) : 65535;

    mavlink_msg_gps_raw_int_pack(config_.system_id, config_.component_id, &msg, t_usec, fix_type,
                                 static_cast<int32_t>(gnss.latitude_deg * 1e7),
                                 static_cast<int32_t>(gnss.longitude_deg * 1e7),
                                 static_cast<int32_t>(ctx.baro.data().altitude_m * 1000.0f),
                                 static_cast<uint16_t>(gnss.hdop * 100.0f), 0,
                                 static_cast<uint16_t>(gnss.ground_speed_mps * 100.0f), cog_centi, gnss.satellites,
                                 0, 0, 0, 0, 0);
    mavSendAll(msg);
}

void Mavlink::sendBatteryStatus(const BatteryData& battery)
{
    mavlink_message_t msg;
    uint16_t voltages[10];
    voltages[0] = static_cast<uint16_t>(battery.voltage_v * 1000.0f);
    for (uint8_t i = 1; i < 10; ++i) {
        voltages[i] = UINT16_MAX;
    }

    uint8_t chargeState = MAV_BATTERY_CHARGE_STATE_OK;
    if (battery.state == BatteryState::Low) {
        chargeState = MAV_BATTERY_CHARGE_STATE_LOW;
    } else if (battery.state == BatteryState::Critical) {
        chargeState = MAV_BATTERY_CHARGE_STATE_CRITICAL;
    }

    mavlink_msg_battery_status_pack(config_.system_id, config_.component_id, &msg, 0, MAV_BATTERY_FUNCTION_ALL,
                                    MAV_BATTERY_TYPE_LIPO, INT16_MAX, voltages, -1, -1, -1,
                                    static_cast<int8_t>(battery.percent), 0, chargeState);
    mavSendAll(msg);
}

// SYS_STATUS is what Mission Planner's HUD reads for "Bat1 <V> <A> <%>";
// BATTERY_STATUS alone only fills the battery details in the Status tab.
void Mavlink::sendSysStatus(const BatteryData& battery)
{
    mavlink_message_t msg;
    const uint16_t voltage_mv = battery.valid ? static_cast<uint16_t>(battery.voltage_v * 1000.0f) : UINT16_MAX;
    const int8_t remaining = battery.valid ? static_cast<int8_t>(battery.percent) : -1;

    mavlink_msg_sys_status_pack(config_.system_id, config_.component_id, &msg, MAV_SYS_STATUS_SENSOR_BATTERY,
                                MAV_SYS_STATUS_SENSOR_BATTERY,
                                battery.state == BatteryState::Ok ? MAV_SYS_STATUS_SENSOR_BATTERY : 0, 0,
                                voltage_mv, -1, remaining, 0, 0, 0, 0, 0, 0);
    mavSendAll(msg);
}

void Mavlink::sendNavControllerOutput(const VehicleContext& ctx)
{
    const MissionState& mission = ctx.navigation.state();
    const float aspd_error = mission.target_airspeed_mps - ctx.airspeed.data().velocity_mps;

    // Bearings (deg, 0..359): nav_bearing = L1 reference direction, target_bearing
    // = direction from the vehicle to the active waypoint. Compare with the
    // ATTITUDE yaw to verify the sign of nav_roll on the bench:
    //   (target_bearing - yaw) in (-180,0) => waypoint is to the LEFT  => nav_roll < 0
    //   (target_bearing - yaw) in (0,180)  => waypoint is to the RIGHT => nav_roll > 0
    auto toDeg360 = [](int32_t centi_deg) -> int16_t {
        int32_t d = centi_deg / 100;
        d %= 360;
        if (d < 0) {
            d += 360;
        }
        return static_cast<int16_t>(d);
    };

    // Saat berputar, xtrack_error = galat radius loiter (jarak ke pusat - radius,
    // m; + = di luar lingkaran), sama dengan LOIT_ERR, supaya bisa ditampilkan
    // langsung di Mission Planner ("Cross Track Error").
    const bool circling = mission.loiter_last_ms != 0 && millis() - mission.loiter_last_ms < 500;
    const float xtrack_error = circling ? mission.loiter_error_m : ctx.l1.crosstrackError();

    mavlink_message_t msg;
    mavlink_msg_nav_controller_output_pack(config_.system_id, config_.component_id, &msg, mission.nav_roll_deg,
                                           mission.nav_pitch_deg, toDeg360(ctx.l1.navBearingCd()),
                                           toDeg360(ctx.l1.targetBearingCd()),
                                           mission.auto_state.distance_next_wp, 0,
                                           aspd_error, xtrack_error);
    mavSendAll(msg);
}

void Mavlink::sendVfrHud(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    uint16_t throttle_pct =
        static_cast<uint16_t>(constrain(map(ctx.actuator.throttlePwm(), 988, 2012, 0, 100), 0, 100));
    if (!ctx.radio.armed()) {
        throttle_pct = 0;
    }

    const float heading_deg = ctx.imu.data().yaw_deg < 0.0f ? ctx.imu.data().yaw_deg + 360.0f : ctx.imu.data().yaw_deg;

    mavlink_msg_vfr_hud_pack(config_.system_id, config_.component_id, &msg, ctx.airspeed.data().velocity_mps,
                            ctx.gnss.ground_speed_mps, heading_deg, throttle_pct, ctx.baro.data().altitude_m,
                            ctx.baro.data().climb_rate_mps);
    mavSendAll(msg);
}

void Mavlink::sendScaledPressure(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    const float press_abs_hpa = ctx.baro.data().pressure_pa / 100.0f;
    const float press_diff_hpa = ctx.airspeed.data().differential_pressure_psi * 68.94757f;
    const int16_t temp_cdeg = static_cast<int16_t>(ctx.baro.data().temperature_c * 100.0f);

    mavlink_msg_scaled_pressure_pack(config_.system_id, config_.component_id, &msg, millis(), press_abs_hpa,
                                     press_diff_hpa, temp_cdeg);
    mavSendAll(msg);
}

void Mavlink::sendMissionItemReached(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_msg_mission_item_reached_pack(config_.system_id, config_.component_id, &msg, seq);
    mavSendAll(msg);
}

void Mavlink::notifyWaypointReached(uint16_t seq)
{
    sendMissionItemReached(seq);
    char text[32];
    snprintf(text, sizeof(text), "WP %u REACHED", static_cast<unsigned>(seq));
    sendStatusText(MAV_SEVERITY_INFO, text);
}

void Mavlink::sendGlobalPositionInt(const VehicleContext& ctx)
{
    const GnssFixData& gnss = ctx.gnss;
    const bool have_fix = gnss.fix_type != GnssFixType::NoFix;
    const int32_t lat = have_fix ? static_cast<int32_t>(gnss.latitude_deg * 1e7) : 0;
    const int32_t lon = have_fix ? static_cast<int32_t>(gnss.longitude_deg * 1e7) : 0;
    const int32_t alt_mm = static_cast<int32_t>(ctx.baro.data().altitude_m * 1000.0f);
    const float yaw_deg = ctx.imu.data().yaw_deg;
    const uint16_t hdg_cdeg = static_cast<uint16_t>((yaw_deg < 0.0f ? yaw_deg + 360.0f : yaw_deg) * 100.0f);

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(config_.system_id, config_.component_id, &msg, millis(), lat, lon, alt_mm,
                                         alt_mm, 0, 0, 0, hdg_cdeg);
    mavSendAll(msg);
}

void Mavlink::sendServoOutputRaw(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    mavlink_msg_servo_output_raw_pack(config_.system_id, config_.component_id, &msg,
                                      static_cast<uint32_t>(millis()), 0, ctx.actuator.aileronLeftPwm(),
                                      ctx.actuator.elevatorPwm(), ctx.actuator.throttlePwm(),
                                      ctx.actuator.rudderLeftPwm(), ctx.actuator.aileronRightPwm(),
                                      0, 0, ctx.actuator.payloadPwm(), 0, 0, 0, 0, 0, 0,
                                      0, 0);
    mavSendAll(msg);
}

void Mavlink::sendRcChannelsRaw(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    // RC_CHANNELS (not _RAW): the form Mission Planner's Radio Calibration
    // page reads. Slots follow the real transmitter channel numbers:
    // 1-4 sticks, 5 arm, 6 mode, 7-8 aux. RSSI field = ELRS link quality
    // scaled 0-255 (255 = unknown, e.g. SBUS builds).
    const uint8_t rssi = ctx.radio.linkStatsValid()
                             ? static_cast<uint8_t>(ctx.radio.linkQuality() * 255U / 100U)
                             : 255;
    mavlink_msg_rc_channels_pack(config_.system_id, config_.component_id, &msg, static_cast<uint32_t>(millis()), 8,
                                 ctx.radio.channelRoll(), ctx.radio.channelPitch(), ctx.radio.channelThrottle(),
                                 ctx.radio.channelYaw(), ctx.radio.channelArm(), ctx.radio.channelMode(),
                                 ctx.radio.channelModeBackup(), ctx.radio.channelVehicleMode(), 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, rssi);
    mavSendAll(msg);
}

void Mavlink::sendMissionCountToCompanion(const VehicleContext& ctx)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(config_.system_id, config_.component_id, &msg, 0, MAV_COMP_ID_ALL,
                                   ctx.navigation.state().wp_sum, MAV_MISSION_TYPE_MISSION);
    mavWrite(Serial7, msg);
}

void Mavlink::handleParamRequestList(VehicleContext& ctx)
{
    (void)ctx;
    param_send_index_ = 0;
}

void Mavlink::sendParamValue(const VehicleContext& ctx, uint16_t index)
{
    if (index >= ctx.params.count()) {
        return;
    }
    const ParamEntry& entry = ctx.params.entryAt(index);
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(config_.system_id, config_.component_id, &msg, entry.name, *entry.target,
                                 MAV_PARAM_TYPE_REAL32, ctx.params.count(), index);
    mavSendAll(msg);
}

void Mavlink::handleParamRequestRead(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_param_request_read_t req;
    mavlink_msg_param_request_read_decode(&msg, &req);

    int idx = -1;
    if (req.param_index >= 0) {
        idx = (req.param_index < ctx.params.count()) ? req.param_index : -1;
    } else {
        for (uint16_t i = 0; i < ctx.params.count(); ++i) {
            if (strncmp(ctx.params.entryAt(i).name, req.param_id, 16) == 0) {
                idx = static_cast<int>(i);
                break;
            }
        }
    }

    if (idx >= 0) {
        sendParamValue(ctx, static_cast<uint16_t>(idx));
    } else {
        sendStatusText(MAV_SEVERITY_WARNING, "PARAM NOT FOUND");
    }
}

void Mavlink::handleParamSet(VehicleContext& ctx, const mavlink_message_t& msg)
{
    mavlink_param_set_t ps;
    mavlink_msg_param_set_decode(&msg, &ps);

    if (ps.target_system != config_.system_id && ps.target_system != 0) {
        return;
    }

    if (!ctx.params.setValue(ps.param_id, ps.param_value, /*save_after_set=*/true)) {
        sendStatusText(MAV_SEVERITY_WARNING, "PARAM SET: NOT FOUND");
        return;
    }

    // Param lain baru berlaku setelah reboot (config disalin saat konstruksi);
    // param navigasi di bawah diteruskan langsung ke Navigation agar perubahan
    // dari Mission Planner langsung berpengaruh.
    struct LiveParam {
        const char* name;
        void (Navigation::*apply)(float);
    };
    static const LiveParam kLiveParams[] = {
        {"WP_RADIUS", &Navigation::setWaypointRadius},
        {"ROLL_LIMIT_DEG", &Navigation::setRollLimitDeg},
        {"WP_LOITER_RAD", &Navigation::setLoiterRadius},
        {"LOIT_ROLL_SIGN", &Navigation::setLoiterRollSign},
        {"LOIT_ALT", &Navigation::setLoiterAltitude},
        {"LOIT_BANK_DEG", &Navigation::setLoiterBankDeg},
    };
    for (const LiveParam& live : kLiveParams) {
        if (strncmp(ps.param_id, live.name, 16) != 0) {
            continue;
        }
        float value = 0.0f;
        if (ctx.params.getValue(live.name, value)) {
            ControlLock lock;
            if (lock.acquired()) {
                (ctx.navigation.*live.apply)(value);
            } else {
                // Sudah tersimpan di EEPROM; berlaku setelah reboot.
                sendStatusText(MAV_SEVERITY_WARNING, "PARAM: busy, applies after reboot");
            }
        }
        break;
    }

    // Param L1 & fuzzy (2026-10-25) juga berlaku LIVE; DIST_ENABLE/DIST_US/
    // DIST_MS dibaca langsung oleh DisturbanceInjector.
    const auto is = [&ps](const char* name) { return strncmp(ps.param_id, name, 16) == 0; };
    const bool is_l1_param = is("NAVL1_PERIOD") || is("NAVL1_DAMPING") || is("NAVL1_XTRACK_I");
    const bool is_fuzzy_param = is("FUZZY_ENABLE") || is("FUZZY_MIN_PER") || is("FUZZY_MAX_PER");
    if (is_l1_param || is_fuzzy_param) {
        ControlLock lock;
        if (lock.acquired()) {
            float value = 0.0f;
            if (is("NAVL1_PERIOD") && ctx.params.getValue("NAVL1_PERIOD", value)) {
                ctx.l1.setPeriod(value);
                ctx.fuzzy_tuner.setBasePeriod(value);  // "Normal" fuzzy = the fixed period
            } else if (is("NAVL1_DAMPING") && ctx.params.getValue("NAVL1_DAMPING", value)) {
                ctx.l1.setDamping(value);
            } else if (is("NAVL1_XTRACK_I") && ctx.params.getValue("NAVL1_XTRACK_I", value)) {
                ctx.l1.setXtrackIntegratorGain(value);
            } else if (is("FUZZY_ENABLE") && ctx.params.getValue("FUZZY_ENABLE", value)) {
                ctx.fuzzy_tuner.applyEnabled(value >= 0.5f, ctx.l1);
            } else if (is_fuzzy_param) {  // FUZZY_MIN_PER / FUZZY_MAX_PER
                float min_per = 0.0f, max_per = 0.0f;
                if (ctx.params.getValue("FUZZY_MIN_PER", min_per) &&
                    ctx.params.getValue("FUZZY_MAX_PER", max_per)) {
                    ctx.fuzzy_tuner.setPeriodLimits(min_per, max_per);
                }
            }
        } else {
            // Sudah tersimpan di EEPROM; berlaku setelah reboot.
            sendStatusText(MAV_SEVERITY_WARNING, "PARAM: busy, applies after reboot");
        }
    }

    int idx = -1;
    for (uint16_t i = 0; i < ctx.params.count(); ++i) {
        if (strncmp(ctx.params.entryAt(i).name, ps.param_id, 16) == 0) {
            idx = static_cast<int>(i);
            break;
        }
    }
    if (idx >= 0) {
        sendParamValue(ctx, static_cast<uint16_t>(idx));
    }
}

void Mavlink::pumpParamStream(VehicleContext& ctx)
{
    if (param_send_index_ == UINT16_MAX) {
        return;
    }
    if (param_send_index_ >= ctx.params.count()) {
        param_send_index_ = UINT16_MAX;
        return;
    }
    sendParamValue(ctx, param_send_index_++);
}

void Mavlink::update(VehicleContext& ctx)
{
    const unsigned long now = millis();

    if (now - last_heartbeat_ms_ >= config_.heartbeat_interval_ms) {
        last_heartbeat_ms_ = now;
        sendHeartbeat(ctx);
    }

    if (now - last_altitude_ms_ >= config_.altitude_interval_ms) {
        last_altitude_ms_ = now;
        sendAltitude(ctx.baro.data().altitude_m);
    }

    if (now - last_gps_ms_ >= config_.gps_interval_ms) {
        last_gps_ms_ = now;
        sendGpsRawInt(ctx);
    }

    if (now - last_attitude_ms_ >= config_.attitude_interval_ms) {
        last_attitude_ms_ = now;
        const ImuData& imu_data = ctx.imu.data();
        sendAttitude(imu_data.roll_rad, imu_data.pitch_rad, imu_data.yaw_rad);
    }
    

    const float roll_deg = ctx.imu.data().roll_deg;
    if (fabsf(roll_deg) > config_.roll_alert_threshold_deg && (now - last_alert_ms_ >= config_.roll_alert_debounce_ms)) {
        char buf[50];
        snprintf(buf, sizeof(buf), "ALERT: ROLL LIMIT (%.1f deg)", roll_deg);
        sendAlert(buf);
        last_alert_ms_ = now;
    }

    if (now - last_battery_ms_ >= config_.battery_interval_ms) {
        last_battery_ms_ = now;
        const BatteryData battery = ctx.battery.data();
        sendSysStatus(battery);
        sendBatteryStatus(battery);

        if (battery.valid && battery.state != last_battery_state_) {
            char buf[50];
            if (battery.state == BatteryState::Low) {
                snprintf(buf, sizeof(buf), "BATTERY LOW (%.2f V)", battery.voltage_v);
                sendAlert(buf);
            } else if (battery.state == BatteryState::Critical) {
                snprintf(buf, sizeof(buf), "BATTERY CRITICAL (%.2f V)", battery.voltage_v);
                sendAlert(buf);
            }
            last_battery_state_ = battery.state;
        }
    }

    if (now - last_nav_controller_ms_ >= config_.nav_controller_interval_ms) {
        last_nav_controller_ms_ = now;
        sendNavControllerOutput(ctx);
    }

    // Status auto throttle (FC_AUTO_THROTTLE_ENABLE): kabari GCS saat TECS
    // mulai/berhenti memegang throttle atau saat ditolak (dengan alasannya).
    {
        const MissionState& ms = ctx.navigation.state();
        const char* reason = ms.auto_throttle_mode ? ms.auto_throttle_block_reason : nullptr;
        if (ms.auto_throttle_engaged != auto_thr_engaged_reported_ || reason != auto_thr_reason_reported_) {
            auto_thr_engaged_reported_ = ms.auto_throttle_engaged;
            auto_thr_reason_reported_ = reason;
            if (reason != nullptr) {
                sendStatusText(MAV_SEVERITY_WARNING, reason);
            } else {
                sendStatusText(MAV_SEVERITY_INFO, ms.auto_throttle_engaged ? "AUTO THR: ON (TECS)" : "AUTO THR: manual");
            }
        }
    }

    // AUTO masuk/keluar loiter misi: kabari GCS dan kirim heartbeat segera
    // supaya pergantian AUTO <-> LOITER tercatat tepat waktu di tlog.
    const bool auto_circling = autoIsCircling(ctx);
    if (auto_circling != auto_circling_reported_) {
        auto_circling_reported_ = auto_circling;
        char buf[40];
        if (auto_circling) {
            snprintf(buf, sizeof(buf), "AUTO: LOITER WP %d", ctx.navigation.state().flag_wp);
        } else {
            snprintf(buf, sizeof(buf), "AUTO: LOITER selesai");
        }
        sendStatusText(MAV_SEVERITY_INFO, buf);
        sendHeartbeat(ctx);
        last_heartbeat_ms_ = now;
    }

    // Galat radius loiter (untuk analisis skripsi), 5 Hz: ikut tersimpan di tlog
    // Mission Planner sebagai NAMED_VALUE_FLOAT. Hanya dikirim saat wahana
    // benar-benar berputar (data loiter diperbarui < 500 ms lalu).
    if (now - last_loiter_ms_ >= 200) {
        last_loiter_ms_ = now;
        const MissionState& loiter = ctx.navigation.state();
        if (loiter.loiter_last_ms != 0 && now - loiter.loiter_last_ms < 500) {
            sendNamedValueFloat("LOIT_RAD", loiter.loiter_radius_used_m);
            sendNamedValueFloat("LOIT_DST", loiter.loiter_dist_m);
            sendNamedValueFloat("LOIT_ERR", loiter.loiter_error_m);
            sendNamedValueFloat("LOIT_DIR", static_cast<float>(loiter.loiter_direction));
            sendNamedValueFloat("LOIT_N", loiter.loiter_north_m);
            sendNamedValueFloat("LOIT_E", loiter.loiter_east_m);
            sendNamedValueFloat("LOIT_BNK", loiter.bank_limit_used_deg);
        }
    }

    // MISSION_CURRENT at 1 Hz: without it Mission Planner never shows which
    // waypoint is active, so claimed waypoints look "not claimed".
    if (now - last_mission_current_ms_ >= 1000) {
        last_mission_current_ms_ = now;
        const int current_wp = ctx.navigation.state().flag_wp;
        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(config_.system_id, config_.component_id, &msg,
                                         static_cast<uint16_t>(current_wp < 0 ? 0 : current_wp));
        mavSendAll(msg);
    }

    if (now - last_global_position_ms_ >= config_.global_position_interval_ms) {
        last_global_position_ms_ = now;
        sendGlobalPositionInt(ctx);
    }

    if (now - last_vfr_hud_ms_ >= config_.vfr_hud_interval_ms) {
        last_vfr_hud_ms_ = now;
        sendVfrHud(ctx);
    }

    if (now - last_scaled_pressure_ms_ >= config_.scaled_pressure_interval_ms) {
        last_scaled_pressure_ms_ = now;
        sendScaledPressure(ctx);
    }

    if (now - last_servo_output_ms_ >= config_.servo_output_interval_ms) {
        last_servo_output_ms_ = now;
        sendServoOutputRaw(ctx);
    }

    if (now - last_rc_channels_ms_ >= config_.rc_channels_interval_ms) {
        last_rc_channels_ms_ = now;
        sendRcChannelsRaw(ctx);
    }

    if (now - last_mission_count_to_companion_ms_ >= config_.mission_count_to_companion_interval_ms) {
        last_mission_count_to_companion_ms_ = now;
        sendMissionCountToCompanion(ctx);
    }

    pumpParamStream(ctx);
}

}  // namespace fc
