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
// past Waypoints' end; kAddrData+kMaxParams*4 = 1144 leaves plenty of
// margin before storage/ImuCalibrationStorage.h's block at 2000. See
// docs/params.md and docs/imu-bno055.md for the full EEPROM map.
constexpr uint16_t kAddrMagic = 1000;
constexpr uint16_t kAddrCount = 1002;
constexpr uint16_t kAddrData = 1004;
constexpr uint16_t kTeensy41EepromSize = 4284;

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

void Params::initFixedWing(L1ControllerConfig& l1, FuzzyL1TunerConfig& fuzzy, NavigationConfig& navigation,
                           TecsConfig& tecs, AttitudeControllerConfig& attitude, Imu& imu)
{
    count_ = 0;

    // L1 guidance -- NOT registered as a param at all in the legacy code
    // (bare compile-time global, no EEPROM exposure). This is the parameter
    // the thesis's fuzzy tuner adjusts at runtime; the base value here is
    // what "L1 conventional" uses when the tuner is disabled.
    //
    // Nama memakai awalan ArduPlane "NAVL1_" (dulu L1_PERIOD/L1_DAMPING/
    // L1_XTRACK_I): Mission Planner mengambil kolom Description/Units/Range
    // Full Parameter List dari definisi param ArduPlane (ArduPlane.apm.pdef.xml)
    // berdasarkan NAMA, jadi hanya nama ArduPlane yang punya keterangan.
    // Ganti nama tidak menggeser EEPROM (dimuat berdasarkan urutan, bukan nama).
    // Berlaku setelah reboot (L1Controller menyalin config saat konstruksi).
    add("NAVL1_PERIOD", &l1.period_s, 22.0f, 5.0f, 40.0f);
    add("NAVL1_DAMPING", &l1.damping, 0.707f, 0.6f, 1.0f);  // nilai tetap proposal
    add("NAVL1_XTRACK_I", &l1.xtrack_integrator_gain, 0.2f, 0.0f, 1.0f);

    // Fuzzy L1-period tuner bounds (the thesis's self-tuning mechanism).
    add("FUZZY_MIN_PER", &fuzzy.min_period_s, 10.0f, 5.0f, 40.0f);
    add("FUZZY_MAX_PER", &fuzzy.max_period_s, 30.0f, 5.0f, 40.0f);
    add("FUZZY_L1_EN", &fuzzy.enabled, fuzzy.enabled, 0.0f, 1.0f);

    // Mission Planner-adjustable loiter and mission-completion behavior.
    // ArduPlane/Mission Planner name for the default loiter radius. Keep this
    // in the existing slot so EEPROM values after it do not shift.
    add("WP_LOITER_RAD", &navigation.loiter_radius_m, navigation.loiter_radius_m, 20.0f, 1000.0f);
    add("MIS_DONE", &navigation.mission_done_behavior, navigation.mission_done_behavior, 0.0f, 2.0f);

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
    add("WP_RADIUS", &navigation.wp_radius_default_m, 15.0f, 2.0f, 100.0f);

    // Batas bank perintah navigasi AUTO (|nav_roll| maksimum dari L1), derajat.
    // Nama ArduPlane agar Mission Planner menampilkan keterangannya. Berlaku
    // LIVE seperti WP_RADIUS. Tidak membatasi FBWA (stick) -- FBWA memakai
    // pemetaan stick Khageswara sendiri. Tetap di AKHIR tabel (EEPROM posisional).
    add("ROLL_LIMIT_DEG", &navigation.roll_limit_deg, 45.0f, 20.0f, 60.0f);
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

ParamLoadResult Params::load()
{
    const uint16_t end_addr = kAddrData + (count_ * sizeof(float));
    if (end_addr > kTeensy41EepromSize) {
        applyDefaults(0);
        return ParamLoadResult::NoValidEepromData;
    }

    uint16_t magic = 0;
    uint16_t saved_count = 0;
    EEPROM.get(kAddrMagic, magic);
    EEPROM.get(kAddrCount, saved_count);

    if (magic != kEepromMagic) {
        applyDefaults(0);
        save();
        return ParamLoadResult::NoValidEepromData;
    }

    const uint16_t load_count = (saved_count < count_) ? saved_count : count_;
    uint16_t addr = kAddrData;
    for (uint16_t i = 0; i < load_count; ++i) {
        float value = 0.0f;
        EEPROM.get(addr, value);
        addr += sizeof(float);
        *params_[i].target = clampValue(value, params_[i].min_val, params_[i].max_val);
    }

    if (saved_count < count_) {
        applyDefaults(saved_count);
        save();
        return ParamLoadResult::SchemaUpgraded;
    }

    return ParamLoadResult::LoadedFromEeprom;
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
