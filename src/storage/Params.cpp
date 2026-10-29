#include "storage/Params.h"

#include <EEPROM.h>
#include <math.h>
#include <string.h>

namespace {

constexpr uint16_t kEepromMagic = 0xFC01;  // New schema (FW-only) -- distinct from legacy 0xCDAC.

// 1000, not the old 650: storage/Waypoints.h's kAddrWpData(8) actually
// extends to 8 + kMaxEepromWaypoints(50)*sizeof(Locations)(16) = 808, which
// swallowed the old 650-782 range whole -- every waypoint save silently
// corrupted these params and vice versa. 1000 leaves a clear 192-byte gap
// past Waypoints' end; kAddrData+kMaxParams*4 = 1140 leaves plenty of
// margin before storage/ImuCalibrationStorage.h's block at 2000. See
// docs/params.md and docs/imu-bno055.md for the full EEPROM map.
constexpr uint16_t kAddrMagic = 1000;
constexpr uint16_t kAddrCount = 1002;
constexpr uint16_t kAddrData = 1004;
constexpr uint16_t kTeensy41EepromSize = 4284;

// Name-keyed copy of the table (2026-10-29). The positional block above is
// loaded by SLOT, so flashing a firmware with a different param order (e.g.
// the loiter branch, 47 params, then this one, 38) silently shifted values:
// WP_LOITER_RAD=100 landed in FUZZY_ENABLE (clamped to 1) and FLTMODE3=5 in
// WP_TURN_MAX. This block stores (FNV-1a hash of the name, value) pairs plus a
// hash of the positional block as it was when WE last saved. The positional
// block is still written unchanged so older firmware keeps reading it.
// Starts at 1300: past the loiter branch's positional end (1004 + 47*4 =
// 1192), and must end before ImuCalibrationStorage at 2000.
constexpr uint16_t kKeyedMagic = 0xFC02;
constexpr uint16_t kAddrKeyedMagic = 1300;
constexpr uint16_t kAddrKeyedCount = 1302;
constexpr uint16_t kAddrKeyedPosHash = 1304;
constexpr uint16_t kAddrKeyedData = 1308;
constexpr uint16_t kKeyedEntrySize = sizeof(uint32_t) + sizeof(float);
static_assert(kAddrKeyedData + fc::Params::kMaxParams * kKeyedEntrySize <= 2000,
              "name-keyed param block would overlap ImuCalibrationStorage at 2000");

// Slot order of positional tables written by earlier firmware that had no
// name-keyed block, recovered from tlog PARAM_VALUE dumps of 2026-10-26.
// Lets load() map an old positional block BY NAME instead of trusting that
// the current table is a prefix of it (it is not: the fuzzy params at slots
// 3, 4 and 34 were removed on 2026-10-29). A table that matches neither
// layout is treated as foreign (defaults).
const char* const kLegacyAutoLayout[] = {  // coba-auto-mode, 38 params (36 before WP_TURN_*)
    "NAVL1_PERIOD", "NAVL1_DAMPING", "NAVL1_XTRACK_I", "FUZZY_MIN_PER", "FUZZY_MAX_PER",
    "TECS_TIME_CNST", "TECS_THR_DAMP", "TECS_PTCH_DAMP", "TECS_INTEG_GN", "TECS_CLMB_MAX",
    "TECS_SINK_MIN", "TECS_SINK_MAX", "TECS_THR_MIN", "TECS_THR_MAX", "TECS_ROLL_CMP",
    "TECS_SPD_WGT", "TECS_VERT_ACC", "AIRSPEED_CRUISE", "MIN_AIRSPEED", "MAX_AIRSPEED",
    "YAW_CORR_EN", "SPD_SCALE_EN", "AIL_OUT_SIGN", "IMU_ROLL_TRIM", "IMU_PITCH_TRIM",
    "ROLL_KP", "ROLL_KRATE", "ROLL_KI", "PITCH_KP", "PITCH_KRATE",
    "PITCH_KI", "YAW_KP", "WP_RADIUS", "ROLL_LIMIT_DEG", "FUZZY_ENABLE",
    "DIST_ENABLE", "WP_TURN_ANT", "WP_TURN_MAX",
};
const char* const kLegacyLoiterLayout[] = {  // loiter branch, 47 params
    "NAVL1_PERIOD", "NAVL1_DAMPING", "NAVL1_XTRACK_I", "FUZZY_MIN_PER", "FUZZY_MAX_PER",
    "TECS_TIME_CNST", "TECS_THR_DAMP", "TECS_PTCH_DAMP", "TECS_INTEG_GN", "TECS_CLMB_MAX",
    "TECS_SINK_MIN", "TECS_SINK_MAX", "TECS_THR_MIN", "TECS_THR_MAX", "TECS_ROLL_CMP",
    "TECS_SPD_WGT", "TECS_VERT_ACC", "AIRSPEED_CRUISE", "MIN_AIRSPEED", "MAX_AIRSPEED",
    "YAW_CORR_EN", "SPD_SCALE_EN", "AIL_OUT_SIGN", "IMU_ROLL_TRIM", "IMU_PITCH_TRIM",
    "ROLL_KP", "ROLL_KRATE", "ROLL_KI", "PITCH_KP", "PITCH_KRATE",
    "PITCH_KI", "YAW_KP", "WP_RADIUS", "ROLL_LIMIT_DEG", "WP_LOITER_RAD",
    "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5",
    "FLTMODE6", "LOIT_ROLL_SIGN", "LOIT_ALT", "LOIT_BANK_DEG", "PARAM_REV",
    "FUZZY_ENABLE", "DIST_ENABLE",
};
constexpr uint16_t kLegacyAutoCount = sizeof(kLegacyAutoLayout) / sizeof(kLegacyAutoLayout[0]);
constexpr uint16_t kLegacyLoiterCount = sizeof(kLegacyLoiterLayout) / sizeof(kLegacyLoiterLayout[0]);

constexpr uint32_t kFnvOffset = 2166136261u;
constexpr uint32_t kFnvPrime = 16777619u;

uint32_t fnvAdd(uint32_t h, uint8_t byte)
{
    return (h ^ byte) * kFnvPrime;
}

}  // namespace

namespace fc {

void Params::add(const char* name, float* target, float default_val, float min_val, float max_val)
{
    if (count_ >= kMaxParams) {
        return;
    }
    ParamEntry& entry = params_[count_++];
    strncpy(entry.name, name, sizeof(entry.name) - 1);
    entry.name[sizeof(entry.name) - 1] = '\0';
    entry.target = target;
    entry.default_val = default_val;
    entry.min_val = min_val;
    entry.max_val = max_val;
}

void Params::initFixedWing(L1ControllerConfig& l1, TecsConfig& tecs,
                           AttitudeControllerConfig& attitude, Imu& imu, NavigationConfig& nav,
                           FlightExperimentConfig& experiment)
{
    count_ = 0;

    // L1 guidance -- NOT registered as a param at all in the legacy code
    // (bare compile-time global, no EEPROM exposure).
    //
    // Nama memakai awalan ArduPlane "NAVL1_" (dulu L1_PERIOD/L1_DAMPING/
    // L1_XTRACK_I): Mission Planner mengambil kolom Description/Units/Range
    // Full Parameter List dari definisi param ArduPlane (ArduPlane.apm.pdef.xml)
    // berdasarkan NAMA, jadi hanya nama ArduPlane yang punya keterangan.
    // Berlaku LIVE (Mavlink::handleParamSet -> L1Controller setter).
    add("NAVL1_PERIOD", &l1.period_s, 22.0f, 5.0f, 40.0f);
    add("NAVL1_DAMPING", &l1.damping, 0.707f, 0.6f, 1.0f);  // nilai tetap proposal
    add("NAVL1_XTRACK_I", &l1.xtrack_integrator_gain, 0.2f, 0.0f, 1.0f);

    // TECS.
    add("TECS_TIME_CNST", &tecs.time_const_s, 5.0f, 0.1f, 15.0f);
    add("TECS_THR_DAMP", &tecs.throttle_damp, 0.5f, 0.0f, 2.0f);
    add("TECS_PTCH_DAMP", &tecs.pitch_damp, 0.0f, 0.0f, 2.0f);
    add("TECS_INTEG_GN", &tecs.integrator_gain, 0.1f, 0.0f, 1.0f);
    add("TECS_CLMB_MAX", &tecs.max_climb_rate_mps, 5.0f, 0.5f, 20.0f);
    add("TECS_SINK_MIN", &tecs.min_sink_rate_mps, 2.0f, 0.5f, 10.0f);
    add("TECS_SINK_MAX", &tecs.max_sink_rate_mps, 5.0f, 0.5f, 20.0f);
    add("TECS_THR_MIN", &tecs.min_throttle_percent, 30.0f, 0.0f, 100.0f);
    add("TECS_THR_MAX", &tecs.max_throttle_percent, 85.0f, 0.0f, 100.0f);
    add("TECS_ROLL_CMP", &tecs.roll_compensation_deg, 10.0f, 0.0f, 20.0f);
    add("TECS_SPD_WGT", &tecs.speed_weight, 0.0f, 0.0f, 2.0f);  // 0 = pitch menahan tinggi saja (throttle manual)
    add("TECS_VERT_ACC", &tecs.vertical_accel_limit_mps2, 7.0f, 1.0f, 15.0f);

    // Airspeed envelope (shared by TECS and, at vehicle-init time, copied
    // into Navigation/AttitudeController's own airspeed fields  --  see
    // docs/params.md for why this is the single authoritative source).
    add("AIRSPEED_CRUISE", &tecs.cruise_airspeed_mps, 18.0f, 5.0f, 35.0f);
    add("MIN_AIRSPEED", &tecs.min_airspeed_mps, 14.0f, 5.0f, 30.0f);
    add("MAX_AIRSPEED", &tecs.max_airspeed_mps, 22.0f, 10.0f, 45.0f);

    // On/off switch for AttitudeController's yaw coordinated-turn
    // feedforward (rudder). 1=enabled, 0=rudder forced neutral. Defaults to
    // 0 (OFF) during the current bench-testing phase -- see
    // AttitudeControllerConfig::yaw_correction_enabled's doc comment.
    // K itself stays NOT exposed here (see class doc above) -- this is only
    // a kill switch, not a tuning knob.
    add("YAW_CORR_EN", &attitude.yaw_correction_enabled, 0.0f, 0.0f, 1.0f);

    // On/off switch for applying the airspeed-based speed scaler to the LQR
    // output. 1=applied as designed, 0=forced flat 1.0x (scaler is still
    // computed and shows up in telemetry either way). Defaults to 0 (OFF)
    // 2026-08-22: ground-testing reads near-zero airspeed, which clamps the
    // scaler to 1.8x and made bench behavior look far twitchier than actual
    // cruise-flight behavior -- see AttitudeControllerConfig::
    // speed_scaler_enabled's doc comment. MUST be set back to 1 before
    // flight -- the controller's gains were validated (tools/
    // lqr_gain_design.py) WITH this scaler compensating the 14-22 m/s
    // envelope active.
    add("SPD_SCALE_EN", &attitude.speed_scaler_enabled, 0.0f, 0.0f, 1.0f);

    // Polaritas aileron FBWA/AUTO Khageswara: +1 = apa adanya, -1 = polaritas
    // legacy (PWM = center - u). Berlaku setelah reboot. Lihat
    // docs/attitude-khageswara.md. CATATAN: param ini dulu disisipkan di
    // tengah tabel, sehingga EEPROM lama tergeser satu slot (log 2026-10-15:
    // ROLL_KP berisi nilai default ROLL_KRATE, dst.). Param baru selalu
    // ditambahkan di AKHIR tabel.
    add("AIL_OUT_SIGN", &attitude.fbwa.aileron_output_sign, 1.0f, -1.0f, 1.0f);

    // BNO055 bench-level trim, set interactively via Mission Planner's
    // "Calibrate Level" (MAV_CMD_PREFLIGHT_CALIBRATION param5=2, see
    // Mavlink::handleCommandLong) and persisted here so it survives reboot.
    add("IMU_ROLL_TRIM", &imu.rollTrimDegRef(), 0.0f, -45.0f, 45.0f);
    add("IMU_PITCH_TRIM", &imu.pitchTrimDegRef(), 0.0f, -45.0f, 45.0f);

    // LQR gains, roll/pitch/yaw -- exposed 2026-08-22 for ground-test jitter
    // tuning (see class doc comment above for the "takes effect on next
    // power-cycle, not live" caveat this inherits like every other param
    // here). Defaults kept in sync with AttitudeController.h's struct
    // literals -- roll/pitch updated 2026-09-11 to the new offline LQR
    // gain-design simulation result (see that file's comment). These
    // defaults only apply to a FRESH EEPROM (no magic) or a schema upgrade
    // (a param that didn't exist in the previously-saved count) -- an
    // EEPROM that already has ROLL_KP etc. saved from before this change
    // will keep loading the OLD values on boot regardless of this default,
    // since Params::load() prefers EEPROM over these when both exist. Set
    // the 6 values below explicitly in Mission Planner (Write Params) after
    // reflashing, or use resetToDefaults(), to actually pick up the retune.
    // integral_limit/output_limit_deg and yaw's k_rate/k_integral (fixed at
    // 0, RateOnly mode has no rate/integral term -- see LqrAxisController.h)
    // stay NOT exposed: those are saturation/mode structure, not tuning
    // knobs a jitter investigation needs to touch.
    add("ROLL_KP", &attitude.roll.k_primary, 2.569728f, 0.0f, 15.0f);
    add("ROLL_KRATE", &attitude.roll.k_rate, 0.232119f, 0.0f, 5.0f);
    add("ROLL_KI", &attitude.roll.k_integral, 1.059184f, 0.0f, 5.0f);
    add("PITCH_KP", &attitude.pitch.k_primary, 4.162670f, 0.0f, 15.0f);
    add("PITCH_KRATE", &attitude.pitch.k_rate, 0.482860f, 0.0f, 5.0f);
    add("PITCH_KI", &attitude.pitch.k_integral, 1.336095f, 0.0f, 5.0f);
    add("YAW_KP", &attitude.yaw.k_primary, 1.021439f, 0.0f, 10.0f);

    // Radius klaim waypoint (m). Nama sama dengan ArduPlane agar kotak
    // "WP Radius" di Flight Plan Mission Planner membaca/menulisnya langsung.
    // Berlaku LIVE (Mavlink::handleParamSet -> Navigation::setWaypointRadius),
    // bukan hanya setelah reboot. Radius per-WP (MAVLink param2 > 0) tetap
    // menang. WAJIB tetap di AKHIR tabel: EEPROM dimuat berdasarkan urutan,
    // menyisipkan param di tengah menggeser nilai semua param sesudahnya.
    add("WP_RADIUS", &nav.wp_radius_default_m, 15.0f, 2.0f, 100.0f);

    // Batas bank perintah navigasi AUTO (|nav_roll| maksimum dari L1), derajat.
    // Nama ArduPlane agar Mission Planner menampilkan keterangannya. Berlaku
    // LIVE seperti WP_RADIUS. Tidak membatasi FBWA (stick) -- FBWA memakai
    // pemetaan stick Khageswara sendiri. Tetap di AKHIR tabel (EEPROM posisional).
    add("ROLL_LIMIT_DEG", &nav.roll_limit_deg, 45.0f, 20.0f, 60.0f);

    // Saklar eksperimen skripsi (2026-10-25), berlaku LIVE (dibaca langsung
    // oleh DisturbanceInjector).
    //  DIST_ENABLE : 1 = injeksi gangguan roll di AUTO diizinkan (lihat
    //                docs/disturbance-injection.md). Default 0.
    add("DIST_ENABLE", &experiment.disturbance_enable, 0.0f, 0.0f, 1.0f);

    // Antisipasi belokan (2026-10-26), berlaku LIVE. Tetap di AKHIR tabel.
    //  WP_TURN_ANT: 0 = radius tetap saja; 1 = antisipasi dibatasi supaya busur
    //               tetap lewat <= WP_RADIUS dari WP; 2 = fillet penuh
    //               R tan(|dpsi|/2) (R dari groundspeed & ROLL_LIMIT_DEG).
    //  WP_TURN_MAX: batas atas jarak antisipasi (m).
    // Lihat docs/navigation.md "Antisipasi belokan".
    add("WP_TURN_ANT", &nav.turn_anticipation_enable, 0.0f, 0.0f, 2.0f);
    add("WP_TURN_MAX", &nav.turn_anticipation_max_m, 80.0f, 0.0f, 200.0f);
}

float Params::clampValue(float value, float min_val, float max_val)
{
    if (isnan(value) || isinf(value)) {
        return min_val;
    }
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void Params::applyDefaults(uint16_t start_index)
{
    for (uint16_t i = start_index; i < count_; ++i) {
        *params_[i].target = params_[i].default_val;
    }
}

int Params::findIndex(const char* name) const
{
    for (uint16_t i = 0; i < count_; ++i) {
        if (strncmp(params_[i].name, name, sizeof(params_[i].name) - 1) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

uint32_t Params::nameHash(const char* name)
{
    uint32_t h = kFnvOffset;
    for (size_t i = 0; i < sizeof(ParamEntry::name) - 1 && name[i] != '\0'; ++i) {
        h = fnvAdd(h, static_cast<uint8_t>(name[i]));
    }
    return h;
}

uint32_t Params::positionalBlockHash(uint16_t saved_count)
{
    // Covers magic, count and every saved value, so any rewrite of the
    // positional block by another firmware changes it.
    uint32_t h = kFnvOffset;
    const uint16_t end_addr = kAddrData + saved_count * sizeof(float);
    for (uint16_t addr = kAddrMagic; addr < end_addr && addr < kTeensy41EepromSize; ++addr) {
        h = fnvAdd(h, EEPROM.read(addr));
    }
    return h;
}

bool Params::loadKeyedBlock(uint32_t& positional_hash_out, bool& all_matched_out)
{
    uint16_t magic = 0;
    uint16_t saved_count = 0;
    EEPROM.get(kAddrKeyedMagic, magic);
    EEPROM.get(kAddrKeyedCount, saved_count);
    if (magic != kKeyedMagic || saved_count == 0 || saved_count > kMaxParams) {
        return false;
    }
    EEPROM.get(kAddrKeyedPosHash, positional_hash_out);

    uint32_t saved_hash[kMaxParams];
    float saved_value[kMaxParams];
    for (uint16_t k = 0; k < saved_count; ++k) {
        const uint16_t addr = kAddrKeyedData + k * kKeyedEntrySize;
        EEPROM.get(addr, saved_hash[k]);
        EEPROM.get(addr + sizeof(uint32_t), saved_value[k]);
    }

    all_matched_out = (saved_count == count_);
    for (uint16_t i = 0; i < count_; ++i) {
        const uint32_t h = nameHash(params_[i].name);
        bool found = false;
        for (uint16_t k = 0; k < saved_count; ++k) {
            if (saved_hash[k] == h) {
                *params_[i].target = clampValue(saved_value[k], params_[i].min_val, params_[i].max_val);
                found = true;
                break;
            }
        }
        if (!found) {
            *params_[i].target = params_[i].default_val;
            all_matched_out = false;
        }
    }
    return true;
}

void Params::saveKeyedBlock()
{
    EEPROM.put(kAddrKeyedMagic, kKeyedMagic);
    EEPROM.put(kAddrKeyedCount, count_);
    for (uint16_t i = 0; i < count_; ++i) {
        const uint16_t addr = kAddrKeyedData + i * kKeyedEntrySize;
        EEPROM.put(addr, nameHash(params_[i].name));
        EEPROM.put(addr + sizeof(uint32_t), *params_[i].target);
    }
    // Last, so an interrupted save leaves a hash mismatch (-> StaleKeyedBlock
    // path, still name-matched) rather than a falsely "current" block.
    EEPROM.put(kAddrKeyedPosHash, positionalBlockHash(count_));
}

ParamLoadResult Params::load()
{
    const uint16_t end_addr = kAddrData + (count_ * sizeof(float));
    if (end_addr > kTeensy41EepromSize) {
        applyDefaults(0);
        return last_load_result_ = ParamLoadResult::NoValidEepromData;
    }

    uint16_t magic = 0;
    uint16_t saved_count = 0;
    EEPROM.get(kAddrMagic, magic);
    EEPROM.get(kAddrCount, saved_count);

    // 1) Name-keyed block (this firmware or later): match every param by name.
    uint32_t keyed_pos_hash = 0;
    bool all_matched = false;
    if (loadKeyedBlock(keyed_pos_hash, all_matched)) {
        const bool positional_untouched =
            magic == kEepromMagic && positionalBlockHash(saved_count) == keyed_pos_hash;
        if (positional_untouched && all_matched) {
            return last_load_result_ = ParamLoadResult::LoadedFromEeprom;
        }
        save();  // rewrite both blocks in THIS firmware's layout
        return last_load_result_ = positional_untouched ? ParamLoadResult::SchemaUpgraded
                                                        : ParamLoadResult::StaleKeyedBlock;
    }

    // 2) No keyed block yet: only the positional block, which has no names.
    if (magic != kEepromMagic) {
        applyDefaults(0);
        save();
        return last_load_result_ = ParamLoadResult::NoValidEepromData;
    }
    // Known earlier layout: map each saved slot to its NAME, then to the
    // current table. Params not in the old layout keep their defaults.
    const char* const* layout = nullptr;
    if (saved_count <= kLegacyAutoCount) {
        layout = kLegacyAutoLayout;  // also covers its 36-param prefix (before WP_TURN_*)
    } else if (saved_count == kLegacyLoiterCount) {
        layout = kLegacyLoiterLayout;
    }
    if (layout == nullptr) {
        // Unknown layout: slot order cannot be trusted. Use defaults.
        applyDefaults(0);
        save();
        return last_load_result_ = ParamLoadResult::ForeignLayout;
    }

    applyDefaults(0);
    bool legacy_all_matched = true;
    for (uint16_t i = 0; i < count_; ++i) {
        bool found = false;
        for (uint16_t slot = 0; slot < saved_count; ++slot) {
            if (strncmp(layout[slot], params_[i].name, sizeof(params_[i].name) - 1) == 0) {
                float value = 0.0f;
                EEPROM.get(static_cast<uint16_t>(kAddrData + slot * sizeof(float)), value);
                *params_[i].target = clampValue(value, params_[i].min_val, params_[i].max_val);
                found = true;
                break;
            }
        }
        legacy_all_matched = legacy_all_matched && found;
    }
    save();  // also creates the keyed block, so this mapping runs only once
    return last_load_result_ = legacy_all_matched ? ParamLoadResult::LegacyPositional
                                           : ParamLoadResult::SchemaUpgraded;
}

void Params::save()
{
    const uint16_t end_addr = kAddrData + (count_ * sizeof(float));
    if (end_addr > kTeensy41EepromSize) {
        return;
    }

    EEPROM.put(kAddrMagic, kEepromMagic);
    EEPROM.put(kAddrCount, count_);

    uint16_t addr = kAddrData;
    for (uint16_t i = 0; i < count_; ++i) {
        const float value = clampValue(*params_[i].target, params_[i].min_val, params_[i].max_val);
        *params_[i].target = value;
        EEPROM.put(addr, value);
        addr += sizeof(float);
    }
    saveKeyedBlock();
}

ParamLoadResult Params::lastLoadResult() const
{
    return last_load_result_;
}

const char* Params::loadWarning() const
{
    // <= 50 chars: MAVLink STATUSTEXT limit.
    switch (last_load_result_) {
        case ParamLoadResult::StaleKeyedBlock:
            return "PARAM: EEPROM dari firmware lain, CEK PARAM";
        case ParamLoadResult::ForeignLayout:
            return "PARAM: tabel firmware lain, semua DEFAULT";
        case ParamLoadResult::NoValidEepromData:
            return "PARAM: EEPROM kosong, semua DEFAULT";
        case ParamLoadResult::SchemaUpgraded:
            return "PARAM: daftar berubah, param baru DEFAULT";
        default:
            return nullptr;
    }
}

void Params::resetToDefaults()
{
    applyDefaults(0);
    save();
}

bool Params::setValue(const char* name, float value, bool save_after_set)
{
    const int index = findIndex(name);
    if (index < 0) {
        return false;
    }
    *params_[index].target = clampValue(value, params_[index].min_val, params_[index].max_val);
    if (save_after_set) {
        save();
    }
    return true;
}

bool Params::getValue(const char* name, float& value_out) const
{
    const int index = findIndex(name);
    if (index < 0) {
        return false;
    }
    value_out = *params_[index].target;
    return true;
}

uint16_t Params::count() const
{
    return count_;
}

const ParamEntry& Params::entryAt(uint16_t index) const
{
    return params_[index];
}

}  // namespace fc
