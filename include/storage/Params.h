#pragma once

#include <stdint.h>

#include "FC_Config.h"
#include "control/AttitudeController.h"
#include "drivers/Imu.h"
#include "navigation/L1Controller.h"
#include "navigation/Navigation.h"
#include "navigation/Tecs.h"

namespace fc {

/**
 * Switches for the thesis experiment, kept as floats because the param table
 * only stores floats, read live by their users.
 *  - disturbance_enable: 1 = fc::DisturbanceInjector may inject the roll
 *    disturbance in AUTO. Default 0 (off) so it can never fire by accident.
 */
struct FlightExperimentConfig {
    float disturbance_enable = 0.0f;
};

struct ParamEntry {
    char name[17] = {};
    float* target = nullptr;
    float default_val = 0.0f;
    float min_val = 0.0f;
    float max_val = 0.0f;
};

enum class ParamLoadResult : uint8_t {
    LoadedFromEeprom,      // name-keyed block current; every param matched by name
    NoValidEepromData,     // defaults applied and saved
    SchemaUpgraded,        // params added/removed since the last save; matched by name, new ones defaulted
    StaleKeyedBlock,       // another firmware rewrote the positional block since our last save:
                           // this firmware's own last values (by name) were used instead
    LegacyPositional,      // no name-keyed block yet; positional load (same count), keyed block now written
    ForeignLayout,         // no name-keyed block and a LARGER positional table (other firmware):
                           // positions cannot be trusted, defaults applied
};

/**
 * Runtime-tunable parameter table + EEPROM persistence (legacy Params.h),
 * rebuilt FW-only: every copter gain entry (CP_STAB_xxx, CP_RATE_xxx, CP_ALT_P,
 * CP_ZVEL_P) and the MODEL_UAV vehicle-type switch are dropped, since there
 * is only one vehicle type now and nothing to switch. The legacy FW_ROLL/
 * PITCH/YAW/THR_P/I/D PID gains are also dropped (superseded by the LQR's
 * offline-computed, fixed K; see docs/attitude-lqr.md).
 *
 * K WAS intentionally not exposed here ("K is fixed, computed offline"),
 * until 2026-08-22: ground-test jitter diagnosis needed faster gain
 * iteration than reflashing per attempt, so ROLL/PITCH_KP/KRATE/KI and
 * YAW_KP are now registered too (see initFixedWing() below). IMPORTANT
 * caveat these gain params inherit from every param in this table: Tecs/
 * L1Controller/LqrAxisController all copy their Config struct BY VALUE at
 * construction (e.g. LqrAxisController's config_{}, Tecs's config_{}) --
 * changing a param here only updates the struct load() populates BEFORE
 * that one-time construction in main.cpp's setup(). A param write via
 * MAVLink SET_PARAM takes effect on the NEXT POWER CYCLE, not live in the
 * same session -- this was already true for every existing param
 * (TECS_TIME_CNST, NAVL1_PERIOD, etc.), the new gain params don't change that.
 * Set + "Write Params" in the GCS, then power-cycle (not necessarily
 * reflash) to apply.
 *
 * New entries this refactor adds: L1 period/damping/xtrack-integrator-gain
 * (not registered as params at all in the legacy code -- a bare compile-time
 * global with no EEPROM exposure).
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
    static constexpr uint16_t kMaxParams = 40;

    /** Registers pointers into the caller-owned config structs. Call once at startup. */
    void initFixedWing(L1ControllerConfig& l1, TecsConfig& tecs,
                       AttitudeControllerConfig& attitude, Imu& imu, NavigationConfig& nav,
                       FlightExperimentConfig& experiment);

    ParamLoadResult load();
    void save();
    void resetToDefaults();

    bool setValue(const char* name, float value, bool save_after_set = true);
    bool getValue(const char* name, float& value_out) const;

    uint16_t count() const;
    const ParamEntry& entryAt(uint16_t index) const;

    /** Result of the last load(). */
    ParamLoadResult lastLoadResult() const;
    /** Short GCS/log warning for the last load(), or nullptr when it needs no attention. */
    const char* loadWarning() const;

private:
    void add(const char* name, float* target, float default_val, float min_val, float max_val);
    int findIndex(const char* name) const;
    void applyDefaults(uint16_t start_index);
    static float clampValue(float value, float min_val, float max_val);
    static uint32_t nameHash(const char* name);
    static uint32_t positionalBlockHash(uint16_t saved_count);
    bool loadKeyedBlock(uint32_t& positional_hash_out, bool& all_matched_out);
    void saveKeyedBlock();

    ParamEntry params_[kMaxParams]{};
    uint16_t count_ = 0;
    ParamLoadResult last_load_result_ = ParamLoadResult::NoValidEepromData;
};

}  // namespace fc
