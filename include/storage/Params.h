#pragma once

#include <stdint.h>

#include "navigation/FuzzyL1Tuner.h"
#include "navigation/L1Controller.h"
#include "navigation/Tecs.h"

namespace fc {

struct ParamEntry {
    char name[17] = {};
    float* target = nullptr;
    float default_val = 0.0f;
    float min_val = 0.0f;
    float max_val = 0.0f;
};

enum class ParamLoadResult : uint8_t {
    LoadedFromEeprom,
    NoValidEepromData,     // defaults applied and saved
    SchemaUpgraded,        // more params exist now than were saved; new ones defaulted and saved
};

/**
 * Runtime-tunable parameter table + EEPROM persistence (legacy Params.h),
 * rebuilt FW-only: every copter gain entry (CP_STAB_xxx, CP_RATE_xxx, CP_ALT_P,
 * CP_ZVEL_P) and the MODEL_UAV vehicle-type switch are dropped, since there
 * is only one vehicle type now and nothing to switch. The legacy FW_ROLL/
 * PITCH/YAW/THR_P/I/D PID gains are also dropped (superseded by the LQR's
 * offline-computed, fixed K; see docs/attitude-lqr.md). K is intentionally
 * NOT exposed here for runtime tuning, per the "K is fixed, computed
 * offline" decision.
 *
 * New entries this refactor adds: L1 period/damping/xtrack-integrator-gain
 * (not registered as params at all in the legacy code -- a bare compile-time
 * global with no EEPROM exposure) and the fuzzy tuner's period-scale bounds.
 *
 * Servo pin/channel mapping (legacy SERVO_AIL_L/ELE/RUD_L/AIL_R/RUD_R/
 * PAYLOAD/THROTTLE, MOTOR_1-4_PIN) is NOT ported into this table. Pin
 * numbers rarely need field-tuning without a reflash, unlike control-loop
 * gains, and ActuatorConfig's pins are plain hardware pin numbers rather
 * than EEPROM-addressable floats. Remap by editing ActuatorConfig's
 * defaults and reflashing.
 *
 * No Serial logging internally (unlike legacy, which printed on every
 * operation): callers get a result they can log themselves.
 */
class Params final {
public:
    static constexpr uint16_t kMaxParams = 32;

    /** Registers pointers into the caller-owned config structs. Call once at startup. */
    void initFixedWing(L1ControllerConfig& l1, FuzzyL1TunerConfig& fuzzy, TecsConfig& tecs);

    ParamLoadResult load();
    void save();
    void resetToDefaults();

    bool setValue(const char* name, float value, bool save_after_set = true);
    bool getValue(const char* name, float& value_out) const;

    uint16_t count() const;
    const ParamEntry& entryAt(uint16_t index) const;

private:
    void add(const char* name, float* target, float default_val, float min_val, float max_val);
    int findIndex(const char* name) const;
    void applyDefaults(uint16_t start_index);
    static float clampValue(float value, float min_val, float max_val);

    ParamEntry params_[kMaxParams]{};
    uint16_t count_ = 0;
};

}  // namespace fc
