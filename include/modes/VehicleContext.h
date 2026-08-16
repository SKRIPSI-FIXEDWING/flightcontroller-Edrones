#pragma once

#include <stdint.h>

#include "control/AttitudeController.h"
#include "drivers/Airspeed.h"
#include "drivers/Barometer.h"
#include "drivers/Imu.h"
#include "estimation/Ahrs.h"
#include "modes/Mode.h"
#include "navigation/FuzzyL1Tuner.h"
#include "navigation/GnssFixData.h"
#include "navigation/L1Controller.h"
#include "navigation/Navigation.h"
#include "navigation/Tecs.h"
#include "storage/Params.h"
#include "vehicle/Actuator.h"
#include "vehicle/Battery.h"
#include "vehicle/Buzzer.h"
#include "vehicle/Radio.h"

namespace fc {

/**
 * Bag of references to every subsystem a fixed-wing mode needs, plus the
 * per-loop snapshot data the scheduler refreshes before calling
 * ModeManager::update(). This replaces the legacy code's implicit global
 * access (imu, ahrs, gepees, fw_control, etc. as file-scope singletons) with
 * explicit dependency injection, while keeping ModeBase's virtual
 * enter/update/exit interface argument-free (each mode stores a
 * VehicleContext& instead).
 *
 * Owned and refreshed by the vehicle-integration layer (Phase 8's
 * main.cpp), not by the modes themselves.
 */
struct VehicleContext {
    Imu& imu;
    Barometer& baro;
    Airspeed& airspeed;
    Ahrs& ahrs;
    L1Controller& l1;
    FuzzyL1Tuner& fuzzy_tuner;
    Tecs& tecs;
    Navigation& navigation;
    AttitudeController& attitude;
    Actuator& actuator;
    Radio& radio;
    Battery& battery;
    Buzzer& buzzer;
    ModeManager& mode_manager;
    Params& params;

    // Refreshed once per loop by the scheduler, before ModeManager::update():
    GnssFixData gnss{};
    float dt_s = 0.0f;
    uint32_t now_ms = 0;

    // Set by the MAVLink handler (Phase 7) when a companion computer sends a
    // payload-drop command; consumed (reset to false) by Actuator::updatePayload().
    bool payload_drop_command = false;

    bool enableThrottleNudge = true;
};

}  // namespace fc
