// KHAGESWARA-FC-2026 fixed-wing refactor -- integration entry point.
//
// This wires together every module ported across Phases 1-7 into one
// FreeRTOS-scheduled firmware. See docs/*.md for the per-module migration
// notes and docs/main-integration.md for this file's specific wiring
// decisions (task layout, GNSS backend choice, config-dependent object
// lazy-construction order).
//
// GNSS backend: selected by FC_GNSS_BACKEND_HERE4 in FC_Config.h --
// Here4GnssReceiver (DroneCAN over CAN3) when 1, GnssNmea (Radiolink SE100)
// when 0. GnssUbx (M10) is implemented but not wired here. See
// docs/gnss-here4-dronecan.md.

#include <Arduino.h>
#include <EEPROM.h>

#include "FreeRTOS.h"
#include "task.h"

#include "control/AttitudeController.h"
#include "drivers/Airspeed.h"
#include "drivers/Barometer.h"
#include "FC_Config.h"
#if FC_GNSS_BACKEND_HERE4
#include "drivers/GnssHere4Can.h"
#else
#include "drivers/GnssNmea.h"
#endif
#include "drivers/Imu.h"
#include "estimation/Ahrs.h"
#include "estimation/AltitudeComplementaryFilter.h"
#include "estimation/AltitudeFusion.h"
#include "estimation/AttitudeMahonyFilter.h"
#include "modes/FixedWingModes.h"
#include "modes/Mode.h"
#include "modes/VehicleContext.h"
#include "navigation/FuzzyL1Tuner.h"
#include "navigation/L1Controller.h"
#include "navigation/Navigation.h"
#include "navigation/Tecs.h"
#include "storage/Params.h"
#include "storage/ImuCalibrationStorage.h"
#include "storage/Waypoints.h"
#include "vehicle/Actuator.h"
#include "vehicle/ControlLock.h"
#include "vehicle/Battery.h"
#include "vehicle/ActLed.h"
#include "vehicle/Buzzer.h"
#include "vehicle/Radio.h"
#include "communication/Mavlink.h"
#include "communication/DataLogger.h"
#include "communication/FlightLog.h"
#include "communication/SdLogger.h"
#include "FC_Config.h"

#include <MTP_Teensy.h>

namespace {

// Task periods and serial baud rates live in FC_Config.h now -- one place
// to retune instead of hunting through this file. `using namespace` here
// (rather than qualifying every use as fc::kImuPeriodMs etc.) keeps every
// existing unqualified reference below working unchanged. FC_Config.h also
// defines FC_DEBUG_SERIAL_ENABLE (used directly below via #if, not through
// this using-directive -- it's a preprocessor macro, not a fc:: symbol).
using namespace fc;

DMAMEM uint8_t g_dmaRx2[4096];
DMAMEM uint8_t g_dmaTx2[4096];
DMAMEM uint8_t g_dmaRx7[2048];
DMAMEM uint8_t g_dmaTx7[4096];
// Serial3's default RX buffer is only 64 bytes; at 333 Hz an ELRS receiver
// sends ~26-byte RC frames plus link statistics, which would overflow it
// between 100 Hz Radio task runs.
DMAMEM uint8_t g_dmaRx3[512];

// ----------------------------------------------------------------------
// Config-independent subsystems: plain global objects (defaults are
// correct without any EEPROM override).
// ----------------------------------------------------------------------
fc::Imu g_imu(Wire);
fc::Barometer g_baro(Wire1);
fc::Airspeed g_airspeed(Wire2);
#if FC_GNSS_BACKEND_HERE4
// Here4 over DroneCAN, CAN3 (CRX3=pin 30, CTX3=pin 31). See FC_Config.h.
fc::Here4GnssReceiver g_gnss;
#else
fc::GnssNmea g_gnss(Serial1);
#endif
fc::Ahrs g_ahrs;
fc::Actuator g_actuator;
// Serial3 (CRSF/ELRS) or Serial8 (SBUS) per FC_RC_PROTOCOL_CRSF -- see FC_Config.h.
fc::Radio g_radio;
fc::Battery g_battery;
fc::Buzzer g_buzzer;
// Physical debug LED, Teensy 4.1 pin 22 (A8). Currently shows IMU state.
fc::ActLed g_actLed;
fc::ModeManager g_modeManager;
fc::Params g_params;
// Saklar eksperimen (param FUZZY_ENABLE / DIST_ENABLE), diisi Params::load().
fc::FlightExperimentConfig g_experiment;
fc::DisturbanceInjector g_disturbance(g_experiment.disturbance_enable, g_experiment.disturbance_us,
                                      g_experiment.disturbance_ms);
fc::Mavlink g_mavlink;
fc::SdLogger g_sdLogger;
fc::FlightLog g_flightLog;
// PuTTY/serial-terminal CSV logger over the primary USB port -- only
// meaningful when FC_DEBUG_SERIAL_ENABLE=1 (see FC_Config.h); harmless to
// construct either way since it just wraps Serial without touching it
// until begin()/logAttitudeAltitude() are called.
fc::DataLogger g_usbLogger(Serial);

// Option 2 altitude estimator (baro+accel complementary filter), run in
// parallel with fc::Barometer's onboard Kalman filter (Option 1) purely for
// bench/flight comparison -- NOT wired into TECS/Navigation. See
// docs/altitude-complementary-filter.md.
fc::AltitudeComplementaryFilter g_altComplementary;

#if FC_ALTITUDE_FUSION_ENABLE
// Disturbance-robust altitude (baro + accel + GPS), published into
// fc::Barometer so TECS/Navigation/MAVLink all consume it. See
// FC_Config.h and docs/altitude-fusion.md.
fc::AltitudeFusion g_altFusion;
#endif

#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
// Option 2 attitude estimator (Mahony, gyro+accel+mag), run in parallel
// with BNO055's on-chip NDOF fusion for bench/flight comparison. Excluded
// entirely (not just idle) when FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE=0 --
// see FC_Config.h. Drives AttitudeController too if
// FC_ATTITUDE_CONTROL_SOURCE_MAHONY is additionally set -- see
// VehicleContext::controlImu() and docs/attitude-mahony-filter.md.
fc::AttitudeMahonyFilter g_attitudeMahony;
#endif

// ----------------------------------------------------------------------
// Config-dependent subsystems: L1Controller/FuzzyL1Tuner/Tecs read their
// tunable fields from Params-owned config structs, and Navigation/
// AttitudeController both need the SAME airspeed envelope Tecs uses. All
// five are constructed in setup(), AFTER params.load() has populated the
// config structs below -- see docs/main-integration.md for why plain
// global objects don't work here (C++ static-init order would construct
// them with un-loaded defaults, before setup() ever runs).
// ----------------------------------------------------------------------
fc::L1ControllerConfig g_l1Config;
fc::FuzzyL1TunerConfig g_fuzzyConfig;
fc::TecsConfig g_tecsConfig;
fc::NavigationConfig g_navigationConfig;
fc::AttitudeControllerConfig g_attitudeConfig;

fc::L1Controller* g_l1 = nullptr;
fc::FuzzyL1Tuner* g_fuzzyTuner = nullptr;
fc::Tecs* g_tecs = nullptr;
fc::Navigation* g_navigation = nullptr;
fc::AttitudeController* g_attitude = nullptr;

fc::VehicleContext* g_ctx = nullptr;

fc::ModeManual* g_modeManual = nullptr;
fc::ModeFbwa* g_modeFbwa = nullptr;
fc::ModeAuto* g_modeAuto = nullptr;
fc::ModeGuided* g_modeGuided = nullptr;
fc::ModeLoiter* g_modeLoiter = nullptr;
#if FC_RTL_MODE_ENABLE
fc::ModeRtl* g_modeRtl = nullptr;
#endif

fc::ModeId g_activeModeId = fc::ModeId::Manual;

TaskHandle_t g_taskImu = nullptr;
TaskHandle_t g_taskBaro = nullptr;
TaskHandle_t g_taskGps = nullptr;
TaskHandle_t g_taskAirspeed = nullptr;
TaskHandle_t g_taskRadio = nullptr;
TaskHandle_t g_taskBuzzer = nullptr;
TaskHandle_t g_taskActLed = nullptr;
TaskHandle_t g_taskControl = nullptr;
TaskHandle_t g_taskMavlink = nullptr;
TaskHandle_t g_taskSdLog = nullptr;
TaskHandle_t g_taskSdWriter = nullptr;
TaskHandle_t g_taskMtp = nullptr;
#if FC_DEBUG_SERIAL_ENABLE
TaskHandle_t g_taskUsbLog = nullptr;
#endif

/** Routes one boot/status diagnostic line to whichever destination
 * FC_DEBUG_SERIAL_ENABLE selects (FC_Config.h) -- compile-time, matches
 * MAVLINK_USB_ENABLE_RX/TX in the same file. mavlink_severity
 * (MAV_SEVERITY_* from common/mavlink.h) is only used when
 * FC_DEBUG_SERIAL_ENABLE==0. */
void reportStatus(uint8_t mavlink_severity, const char* text)
{
#if FC_DEBUG_SERIAL_ENABLE
    (void)mavlink_severity;
    Serial.println(text);
#else
    g_mavlink.sendStatusText(mavlink_severity, text);
#endif
}

/**
 * Maps the CH6 mode-switch PWM to a ModeId through the FLTMODE1..6 params
 * (6 PWM bands, ArduPlane mode numbers), and switches ModeManager if the
 * resulting mode changed. An unknown mode number is a no-op.
 */
void applyRcModeSwitch()
{
    // CH6 dibagi 6 pita PWM (sama seperti ArduPlane); tiap pita memetakan ke
    // nomor mode lewat param FLTMODE1..6 (diatur di Mission Planner).
    const uint16_t pwm = g_radio.channelMode();
    const int band = (pwm < 1231) ? 0 : (pwm < 1361) ? 1 : (pwm < 1491) ? 2 : (pwm < 1621) ? 3 : (pwm < 1750) ? 4 : 5;
    fc::ModeId wanted;
    switch (static_cast<int>(g_navigationConfig.flight_mode[band] + 0.5f)) {
        case 0: wanted = fc::ModeId::Manual; break;
        case 5: wanted = fc::ModeId::Fbwa; break;
        case 10: wanted = fc::ModeId::Auto; break;
        case 4: wanted = fc::ModeId::Guided; break;
        case 12: wanted = fc::ModeId::Loiter; break;
        default: return;
    }

    // Edge-triggered (ArduPilot/Khageswara style): the RC switch only acts when
    // it MOVES (or on the first read after boot). A mode set from the GCS via
    // MAVLink therefore sticks until the pilot moves the switch, and a mode
    // that refused entry is not retried every loop.
    // Dibandingkan per MODE hasil pemetaan (bukan per pita), supaya pita
    // bersebelahan yang berisi mode sama tidak memicu pindah mode ulang
    // (mis. LOITER yang menggeser titik pusatnya).
    static int last_radio_mode = -1;
    const int radio_mode = static_cast<int>(wanted);
    if (radio_mode == last_radio_mode) {
        return;
    }
    last_radio_mode = radio_mode;

    if (g_modeManager.setMode(wanted)) {
        g_activeModeId = wanted;
    } else if (g_navigation != nullptr && g_navigation->autoRejectReason() != nullptr) {
        reportStatus(MAV_SEVERITY_WARNING, g_navigation->autoRejectReason());
    }
}

void waypointReachedTrampoline(uint16_t seq, void* user_data)
{
    static_cast<fc::Mavlink*>(user_data)->notifyWaypointReached(seq);
}

/** No mission in EEPROM: start EMPTY. The previous seed (HOME at 0,0 with
 * wp_sum = 1) made AUTO fly toward lat 0 / lon 0. An empty mission makes
 * Navigation::beginAuto() refuse AUTO until a real mission is uploaded. */
void seedDefaultMission(fc::MissionState& mission)
{
    mission.wp_sum = 0;
    mission.flag_wp = -1;
}

// ----------------------------------------------------------------------
// FreeRTOS tasks
// ----------------------------------------------------------------------

void taskImu(void*)
{
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
    uint32_t last_update_us = micros();
#endif
    for (;;) {
        if (g_imu.update()) {
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
            // Option 2 attitude estimator (Mahony, gyro+accel+mag), run
            // alongside BNO055's on-chip NDOF fusion (Option 1, g_imu's own
            // roll_deg/pitch_deg/yaw_deg). Logged/telemetered for
            // comparison either way; drives AttitudeController too if
            // FC_ATTITUDE_CONTROL_SOURCE_MAHONY is set (see
            // VehicleContext::controlImu()). See docs/attitude-mahony-filter.md
            // and FC_Config.h.
            const uint32_t now_us = micros();
            float dt_s = (now_us - last_update_us) * 1.0e-6f;
            if (dt_s <= 0.0f || dt_s > 0.1f) {
                dt_s = kImuPeriodMs * 1.0e-3f;  // clamp a startup/overrun gap to the nominal period
            }
            last_update_us = now_us;

            const fc::ImuData& imu_data = g_imu.data();
            g_attitudeMahony.update(
                imu_data.angular_rate_dps.x * kDegToRad, imu_data.angular_rate_dps.y * kDegToRad,
                imu_data.angular_rate_dps.z * kDegToRad, imu_data.acceleration_mss.x,
                imu_data.acceleration_mss.y, imu_data.acceleration_mss.z, imu_data.magnetic_field_ut.x,
                imu_data.magnetic_field_ut.y, imu_data.magnetic_field_ut.z, dt_s);
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(kImuPeriodMs));
    }
}

void taskBaro(void*)
{
    for (;;) {
        g_baro.update();
        vTaskDelay(pdMS_TO_TICKS(kBaroPeriodMs));
    }
}

#if FC_SD_LOG_ENABLE
// Producer: every kSdLogPeriodMs, FlightLog formats the latest control-task
// snapshot (plus header/events/ARM snapshot when due) into SdLogger's RAM
// buffer. Never touches the card and never waits -- see taskSdWriter.
// Stack is static in RAM2 like the writer's (RAM1's heap has no spare room);
// FlightLog formats ~340-char rows and 100-WP snapshots with float printf.
constexpr uint32_t kSdLogStackWords = 3072;
DMAMEM StackType_t g_sdLogStack[kSdLogStackWords];
StaticTask_t g_sdLogTcb;

void taskSdLog(void*)
{
    for (;;) {
        g_flightLog.step(g_sdLogger, *g_ctx);
        vTaskDelay(pdMS_TO_TICKS(kSdLogPeriodMs));
    }
}

// Consumer: the only task that touches the SD card (mount/retry, write,
// flush). Lowest application priority, so a stalled card write can delay
// nothing but itself, MTP and the activity LED. Stack is static in RAM2:
// xTaskCreate() stacks come from RAM1's heap, which has no room left for
// another task (see SdLogger.cpp's buffer comment).
constexpr uint32_t kSdWriterStackWords = 2048;
DMAMEM StackType_t g_sdWriterStack[kSdWriterStackWords];
StaticTask_t g_sdWriterTcb;

void taskSdWriter(void*)
{
    for (;;) {
        g_sdLogger.service();
        vTaskDelay(pdMS_TO_TICKS(kSdWriterPeriodMs));
    }
}
#endif

#if FC_DEBUG_SERIAL_ENABLE
// PuTTY-readable CSV twin of taskSdLog, over USB Serial instead of the SD
// card -- full raw IMU sample (not just roll/pitch/yaw) for axis-
// convention/interference diagnosis without pulling the SD card. See
// docs/data-logger-usb.md and docs/imu-bno055.md.
void taskUsbLog(void*)
{
    for (;;) {
        g_usbLogger.logAttitudeAltitude(g_imu.data(), g_imu.calibration(), g_baro.data(),
                                        g_altComplementary.data());
        vTaskDelay(pdMS_TO_TICKS(kUsbLogPeriodMs));
    }
}
#endif

#if FC_SD_MTP_ENABLE
// SD card in the PC's file manager. Holds SdLogger's SD mutex around every
// MTP.loop(): SdFat isn't thread-safe, and the PC asking for free space
// walks the whole FAT (seconds on a 32 GB card) -- concurrently with a log
// write that corrupted the logger's file. Paused entirely while ARMED.
void taskMtp(void*)
{
    bool sd_added = false;
    for (;;) {
        if (!g_radio.armed() && g_sdLogger.lockSd(0)) {
            // The writer mounts the card (possibly long after boot), so it's
            // handed to MTP here -- MTP_Teensy announces late-added storage
            // to the PC with a StoreAdded event.
            if (!sd_added && g_sdLogger.isMounted()) {
                MTP.addFilesystem(SD, "SD Card");
                sd_added = true;
            }
            MTP.loop();
            g_sdLogger.unlockSd();
        }
        vTaskDelay(pdMS_TO_TICKS(kMtpPeriodMs));
    }
}
#endif

void taskGps(void*)
{
    for (;;) {
        g_gnss.update();
        vTaskDelay(pdMS_TO_TICKS(kGpsPeriodMs));
    }
}

void taskAirspeed(void*)
{
    for (;;) {
        g_airspeed.update();
        vTaskDelay(pdMS_TO_TICKS(kAirspeedPeriodMs));
    }
}

void taskRadio(void*)
{
    for (;;) {
        g_radio.update();
        vTaskDelay(pdMS_TO_TICKS(kRadioPeriodMs));
    }
}

void taskBuzzer(void*)
{
    for (;;) {
        g_buzzer.update(g_radio.armed());
        vTaskDelay(pdMS_TO_TICKS(kBuzzerPeriodMs));
    }
}

// IMU state on the debug LED: fast blink = fault/stale, slow blink = alive
// but not fully calibrated, solid = ready to use.
void taskActLed(void*)
{
    constexpr uint32_t kImuMaxAgeUs = 100000U;  // 100 ms
    for (;;) {
        fc::ActLedPattern pattern = fc::ActLedPattern::ImuFault;
        if (g_imu.initialized() && g_imu.healthy() && g_imu.isFresh(kImuMaxAgeUs)) {
            pattern = g_imu.isFullyCalibrated() ? fc::ActLedPattern::ImuReady
                                                : fc::ActLedPattern::ImuCalibrating;
        }
        g_actLed.setPattern(pattern);
        g_actLed.update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void taskControl(void*)
{
    uint32_t last_update_us = micros();
    for (;;) {
        const uint32_t now_us = micros();
        const float raw_dt_s = (now_us - last_update_us) * 1.0e-6f;  // before the overrun clamp below
        g_ctx->dt_s = raw_dt_s;
        if (g_ctx->dt_s > 0.1f) {
            g_ctx->dt_s = kControlPeriodMs * 1.0e-3f;  // clamp a startup/overrun gap to the nominal period
        }
        last_update_us = now_us;

        g_ctx->gnss = g_gnss.data();
        g_ctx->now_ms = millis();

        // Vertical (up-positive, gravity-free) acceleration from BNO055's
        // linear acceleration projected on its gravity vector -- NOT from
        // AhrsData::acceleration_body_frame_mss, whose gravity compensation
        // has the wrong sign for this sensor (constant ~-19.6 m/s^2 bias)
        // and whose Euler-based rotation makes the error depend on
        // orientation. See docs/altitude-complementary-filter.md.
        const fc::ImuData imu_for_altitude = g_imu.data();
        const float accel_up_mss = fc::verticalAccelUpMss(
            imu_for_altitude.linear_acceleration_mss.x, imu_for_altitude.linear_acceleration_mss.y,
            imu_for_altitude.linear_acceleration_mss.z, imu_for_altitude.gravity_mss.x,
            imu_for_altitude.gravity_mss.y, imu_for_altitude.gravity_mss.z);

#if FC_ALTITUDE_FUSION_ENABLE
        // Runs BEFORE the modes so this tick's TECS/Navigation already see
        // the fused altitude through g_baro.data().
        {
            static uint32_t last_baro_sequence = 0;
            static uint32_t last_gnss_sequence = 0;
            static uint32_t last_calibration_count = 0;

            const uint32_t calibration_count = g_baro.groundCalibrationCount();
            if (calibration_count != last_calibration_count) {
                last_calibration_count = calibration_count;  // altitude re-zeroed
                g_altFusion.reset();
                last_baro_sequence = 0;
            }

            g_altFusion.predict(accel_up_mss, g_ctx->dt_s);

            const fc::BarometerData baro_sample = g_baro.data();
            if (baro_sample.valid && baro_sample.sequence != last_baro_sequence) {
                last_baro_sequence = baro_sample.sequence;
                g_altFusion.updateBaro(baro_sample.raw_altitude_m);
            }

            const fc::GnssFixData& gnss_sample = g_ctx->gnss;
            if (gnss_sample.sequence != last_gnss_sequence) {
                last_gnss_sequence = gnss_sample.sequence;
                // 3D fix, enough satellites, sane HDOP (0 = not reported).
                const bool gnss_good = gnss_sample.valid &&
                                       gnss_sample.fix_type == fc::GnssFixType::Fix3D &&
                                       gnss_sample.satellites >= 6 &&
                                       (gnss_sample.hdop == 0.0f || gnss_sample.hdop < 3.0f);
                g_altFusion.updateGps(gnss_sample.altitude_m, -gnss_sample.velocity_ned_mps[2],
                                      gnss_good);
            }

            const fc::AltitudeFusionData fused = g_altFusion.data();
            if (fused.valid) {
                g_baro.applyFusedEstimate(fused.altitude_m, fused.climb_rate_mps);
            }
        }
#endif

        {
            // Mode switch + mode update under the Control lock, so MAVLink
            // handlers (mission writes, DO_SET_MODE, SET_CURRENT) cannot
            // interleave with a running update(). See vehicle/ControlLock.h.
            fc::ControlLock lock(portMAX_DELAY);
            applyRcModeSwitch();
            g_modeManager.update();  // updates g_ahrs as a side effect (see mode update() bodies)
        }

        // Option 2 (baro+accel complementary) altitude, kept for logging.
        g_altComplementary.update(g_baro.data().raw_altitude_m, accel_up_mss, g_ctx->dt_s);

#if FC_SD_LOG_ENABLE
        // Flight log snapshot: the guidance/actuator values THIS cycle's
        // control laws just used, plus event edges (arm/mode/WP/GPS). A plain
        // memcpy + seqlock publish, never blocks. See communication/FlightLog.h.
        g_flightLog.capture(*g_ctx, raw_dt_s);
#endif

        vTaskDelay(pdMS_TO_TICKS(kControlPeriodMs));
    }
}

void taskMavlink(void*)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(kMavlinkPeriodMs);
    uint32_t last_battery_ms = 0;
    uint32_t last_imu_calib_ms = 0;
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
    uint32_t last_mahony_ms = 0;
#endif
    bool imu_calibration_saved = false;
#if FC_ALTITUDE_FUSION_ENABLE
    uint32_t last_altfusion_ms = 0;
#endif

    for (;;) {
        g_mavlink.handlePorts(*g_ctx);
        g_mavlink.update(*g_ctx);

        const uint32_t now_ms = millis();
        if (now_ms - last_battery_ms >= kBatteryPeriodMs) {
            last_battery_ms = now_ms;
            g_battery.update();
        }

        // 10 Hz, matching MavlinkConfig::attitude_interval_ms's default so
        // the two are comparable at a glance in Mission Planner's Status
        // tab (Option 2 vs the ATTITUDE message's Option 1). See
        // docs/attitude-mahony-filter.md.
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
        if (now_ms - last_mahony_ms >= 100) {
            last_mahony_ms = now_ms;
            const fc::AttitudeMahonyData& mahony = g_attitudeMahony.data();
            g_mavlink.sendMahonyAttitude(mahony.roll_deg, mahony.pitch_deg, mahony.yaw_deg);
        }
#endif

#if FC_ALTITUDE_FUSION_ENABLE
        // 5 Hz: see how much barometer disturbance is being rejected.
        if (now_ms - last_altfusion_ms >= 200) {
            last_altfusion_ms = now_ms;
            const fc::AltitudeFusionData fused = g_altFusion.data();
            g_mavlink.sendAltitudeFusionDiagnostics(fused.altitude_m,
                                                    g_baro.data().baro_only_altitude_m,
                                                    fused.baro_bias_m, fused.baro_rejected,
                                                    fused.gps_active);
        }
#endif

        // 1 Hz: poll BNO055 calibration status, and the first time it goes
        // fully calibrated this session, save the offset registers so next
        // boot can restore them instead of re-converging from scratch (see
        // ImuCalibrationStorage.h). Only ever saves once per power cycle.
        if (now_ms - last_imu_calib_ms >= 1000) {
            last_imu_calib_ms = now_ms;
#if FC_SD_LOG_ENABLE
            // SD mount/write/drop events, raised by taskSdWriter and
            // reported from here because this task owns the MAVLink/USB link.
            char sd_status[64];
            if (g_sdLogger.takeStatusMessage(sd_status, sizeof(sd_status))) {
                // "[SD] logging ..." and "[SD] ARM log=..." are the good-news
                // messages; anything else is a problem worth showing on the
                // GCS HUD.
                const bool ok = strncmp(sd_status, "[SD] logging", 12) == 0 ||
                                strncmp(sd_status, "[SD] ARM", 8) == 0;
                reportStatus(ok ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING, sd_status);
            }
#endif
            g_imu.updateCalibration();
            if (FC_IMU_AUTO_SAVE_CALIBRATION_ENABLE && !imu_calibration_saved &&
                g_imu.isFullyCalibrated() && !g_radio.armed()) {
                fc::ImuCalibrationOffsets offsets{};
                if (g_imu.readCalibrationOffsets(offsets)) {
                    fc::ImuCalibrationStorage::save(offsets);
                    imu_calibration_saved = true;
                    Serial.println("[IMU] Fully calibrated -- offsets saved to EEPROM");
                }
            }
        }

        if ((xTaskGetTickCount() - last_wake) >= period) {
            last_wake = xTaskGetTickCount();  // task overran; skip catch-up
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

}  // namespace

void setup()
{
    Serial.begin(kUsbBaud);
    Serial2.begin(kTelemetryBaud);
    Serial2.addMemoryForRead(g_dmaRx2, sizeof(g_dmaRx2));
    Serial2.addMemoryForWrite(g_dmaTx2, sizeof(g_dmaTx2));
    Serial7.begin(kCompanionBaud);
    Serial7.addMemoryForRead(g_dmaRx7, sizeof(g_dmaRx7));
    Serial7.addMemoryForWrite(g_dmaTx7, sizeof(g_dmaTx7));
#if FC_RC_PROTOCOL_CRSF
    Serial3.begin(kCrsfBaud);  // ELRS receiver: RX3=pin 15, TX3=pin 14
    Serial3.addMemoryForRead(g_dmaRx3, sizeof(g_dmaRx3));
#endif
    delay(100);
#if FC_DEBUG_SERIAL_ENABLE
    g_usbLogger.begin();
#endif

    // --- Parameters: register config-struct fields, then load from EEPROM
    // (or seed defaults if none present) BEFORE constructing anything that
    // reads these configs. ---
    fc::initControlMutex();  // before any task is created
    g_params.initFixedWing(g_l1Config, g_fuzzyConfig, g_tecsConfig, g_attitudeConfig, g_imu,
                          g_navigationConfig, g_experiment);
    g_params.load();

    // Keep Navigation/AttitudeController's airspeed envelope consistent
    // with the authoritative TecsConfig values Params just loaded -- see
    // docs/params.md for why these aren't registered as params twice.
    g_navigationConfig.airspeed_min_mps = g_tecsConfig.min_airspeed_mps;
    g_navigationConfig.airspeed_cruise_mps = g_tecsConfig.cruise_airspeed_mps;
    g_navigationConfig.nav_roll_sign = (FC_NAV_ROLL_SIGN < 0) ? -1.0f : 1.0f;
    g_navigationConfig.adaptive_waypoint_airspeed_enabled = (FC_ADAPTIVE_WP_AIRSPEED_ENABLE != 0);
    g_navigationConfig.reject_invalid_mission = (FC_AUTO_REJECT_INVALID_MISSION != 0);
    g_attitudeConfig.trim_airspeed_mps = g_tecsConfig.cruise_airspeed_mps;

    // The fuzzy tuner scales the SAME base period the fixed-period run uses, so
    // "Normal" = conventional L1 and the two methods compare fairly.
    g_fuzzyConfig.base_period_s = g_l1Config.period_s;

    g_l1 = new fc::L1Controller(g_l1Config);
    g_fuzzyTuner = new fc::FuzzyL1Tuner(g_fuzzyConfig);
    g_fuzzyTuner->begin();
    // Param FUZZY_ENABLE (live from the GCS afterwards). 0 = fixed L1 period.
    g_fuzzyTuner->setEnabled(g_experiment.fuzzy_enable >= 0.5f);
    g_tecs = new fc::Tecs(g_tecsConfig);
    g_navigation = new fc::Navigation(g_navigationConfig);
    g_attitude = new fc::AttitudeController(g_attitudeConfig);

    // --- Waypoints: load from EEPROM, or seed a minimal default mission. ---
    if (fc::Waypoints::load(g_navigation->state()) != fc::WaypointStorageResult::Success) {
        seedDefaultMission(g_navigation->state());
    }

    g_navigation->setWaypointReachedCallback(&waypointReachedTrampoline, &g_mavlink);

    // --- Vehicle context: bag of references every mode/Mavlink reads from. ---
    g_ctx = new fc::VehicleContext{
        g_imu, g_baro, g_airspeed, g_ahrs, *g_l1, *g_fuzzyTuner, *g_tecs, *g_navigation, *g_attitude,
        g_actuator, g_radio, g_battery, g_buzzer, g_modeManager, g_params, g_disturbance,
#if FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE
        g_attitudeMahony,
#endif
    };

    // --- Modes: construct and register. ---
    g_modeManual = new fc::ModeManual(*g_ctx);
    g_modeFbwa = new fc::ModeFbwa(*g_ctx);
    g_modeAuto = new fc::ModeAuto(*g_ctx);
    g_modeGuided = new fc::ModeGuided(*g_ctx);
    g_modeManager.registerMode(g_modeManual);
    g_modeManager.registerMode(g_modeFbwa);
    g_modeManager.registerMode(g_modeAuto);
    g_modeManager.registerMode(g_modeGuided);
    g_modeLoiter = new fc::ModeLoiter(*g_ctx);
    g_modeManager.registerMode(g_modeLoiter);
#if FC_RTL_MODE_ENABLE
    g_modeRtl = new fc::ModeRtl(*g_ctx);
    g_modeManager.registerMode(g_modeRtl);
#endif
    if (g_modeManager.setMode(fc::ModeId::Manual)) {
        g_activeModeId = fc::ModeId::Manual;
    }

    // --- Bring up hardware. ---
    g_buzzer.begin();  // Starts the startup tune; buzzerTask drives it forward.
    g_actLed.begin();
    g_actuator.begin();
    g_battery.begin();

    if (!g_imu.begin()) {
        reportStatus(MAV_SEVERITY_ERROR, "[SETUP] IMU init failed -- check wiring/I2C address");
    } else {
        // Restore last session's accel/mag/gyro offsets, if any -- BNO055
        // has no non-volatile calibration storage of its own and otherwise
        // re-converges from scratch every boot (see docs/imu-bno055.md).
        if (FC_IMU_RESTORE_CALIBRATION_ENABLE) {
            fc::ImuCalibrationOffsets saved_imu_calibration{};
            if (fc::ImuCalibrationStorage::load(saved_imu_calibration) ==
                fc::ImuCalibrationStorageResult::Success) {
                if (g_imu.writeCalibrationOffsets(saved_imu_calibration)) {
                    reportStatus(MAV_SEVERITY_INFO, "[SETUP] IMU calibration restored from EEPROM");
                } else {
                    reportStatus(MAV_SEVERITY_WARNING, "[SETUP] IMU calibration restore failed");
                }
            }
        }
    }
    if (g_imu.initialized()) {
        // Confirms what's actually configured on the chip (FC_Config.h's
        // FC_BNO055_AXIS_MAP_CONFIG/SIGN_X/Y/Z), so a captured log is
        // self-describing when comparing axis-convention behavior across
        // firmware builds -- see docs/imu-bno055.md.
        char axis_line[96];
        snprintf(axis_line, sizeof(axis_line),
                 "[IMU] BNO055 axis_map=0x%02X sign_x=%u sign_y=%u sign_z=%u",
                 static_cast<unsigned>(FC_BNO055_AXIS_MAP_CONFIG),
                 static_cast<unsigned>(FC_BNO055_AXIS_SIGN_X),
                 static_cast<unsigned>(FC_BNO055_AXIS_SIGN_Y),
                 static_cast<unsigned>(FC_BNO055_AXIS_SIGN_Z));
        reportStatus(MAV_SEVERITY_INFO, axis_line);
    }
    if (!g_baro.begin()) {
        reportStatus(MAV_SEVERITY_ERROR, "[SETUP] Barometer init failed -- check wiring/I2C address");
    } else {
        // One-time PROM dump for diagnosing MS5611-vs-MS5607 chip identity,
        // see docs/barometer-ms5611.md.
        const fc::BarometerPromDump prom = g_baro.promDump();
        char prom_line[96];
        snprintf(prom_line, sizeof(prom_line), "[SETUP] Baro PROM: C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u",
                 prom.c1_sens_t1, prom.c2_off_t1, prom.c3_tcs, prom.c4_tco, prom.c5_tref, prom.c6_tempsense);
        reportStatus(MAV_SEVERITY_INFO, prom_line);
    }
    if (!g_airspeed.begin()) {
        reportStatus(MAV_SEVERITY_ERROR, "[SETUP] Airspeed init failed -- check wiring/I2C address");
    }
    g_gnss.begin();

    // --- SD card logging + MTP. No card access here: setup() only creates
    // the RAM buffer + SD mutex; taskSdWriter mounts the card (and keeps
    // retrying if it's missing), taskMtp hands it to the PC once mounted.
    // So a missing/slow card can't delay boot or anything after it. Mount
    // result arrives as a "[SD] ..." status message ~1 s after the
    // scheduler starts (GCS Messages tab, or PuTTY with
    // FC_DEBUG_SERIAL_ENABLE=1). See docs/sd-logger-mtp.md. ---
#if FC_SD_LOG_ENABLE
    if (!g_sdLogger.init()) {
        reportStatus(MAV_SEVERITY_ERROR, "[SETUP] SD logger buffer alloc failed");
    }
#endif
#if FC_SD_MTP_ENABLE
    MTP.begin();
#endif

    // Pre-flight safety interlock: blocks until disarmed and a signal is
    // present. See docs/radio.md.
    g_radio.begin();

    // --- FreeRTOS tasks. Priorities match legacy main.cpp's relative
    // ordering (IMU highest, then control/mavlink/radio, then sensors,
    // buzzer lowest). ---
    xTaskCreate(taskImu, "IMU", 2048, nullptr, 6, &g_taskImu);
    xTaskCreate(taskControl, "Control", 8192, nullptr, 5, &g_taskControl);
    xTaskCreate(taskMavlink, "Mavlink", 8192, nullptr, 5, &g_taskMavlink);
    xTaskCreate(taskRadio, "Radio", 4096, nullptr, 5, &g_taskRadio);
    xTaskCreate(taskGps, "GPS", 4096, nullptr, 4, &g_taskGps);
    xTaskCreate(taskBaro, "BARO", 2048, nullptr, 3, &g_taskBaro);
    xTaskCreate(taskAirspeed, "Airspeed", 2048, nullptr, 3, &g_taskAirspeed);
#if FC_SD_LOG_ENABLE
    g_taskSdLog = xTaskCreateStatic(taskSdLog, "SdLog", kSdLogStackWords, nullptr, 2, g_sdLogStack,
                                    &g_sdLogTcb);
    // Priority 1 = below every flight task. configUSE_TIME_SLICING is 0, so
    // the card-touching tasks must NOT share a priority with anything that
    // matters: a busy-waiting SD write only holds back equal/lower tasks.
    g_taskSdWriter = xTaskCreateStatic(taskSdWriter, "SdWrite", kSdWriterStackWords, nullptr, 1,
                                       g_sdWriterStack, &g_sdWriterTcb);
#endif
#if FC_DEBUG_SERIAL_ENABLE
    xTaskCreate(taskUsbLog, "UsbLog", 4096, nullptr, 1, &g_taskUsbLog);
#endif
#if FC_SD_MTP_ENABLE
    xTaskCreate(taskMtp, "Mtp", 4096, nullptr, 1, &g_taskMtp);
#endif
    // xTaskCreate(taskBuzzer, "Buzzer", 2048, nullptr, 3, &g_taskBuzzer);
    xTaskCreate(taskActLed, "ActLed", 1024, nullptr, 1, &g_taskActLed);

    vTaskStartScheduler();
}

void loop()
{
    // Intentionally empty: FreeRTOS scheduler owns execution once
    // vTaskStartScheduler() is called, matching legacy main.cpp.
}
