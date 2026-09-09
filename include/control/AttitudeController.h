#pragma once

#include "control/LqrAxisController.h"
#include "drivers/Imu.h"

namespace fc {

struct AttitudeControllerConfig {
    // Defaults computed by tools/lqr_gain_design/lqr_gain_design.py (cruise
    // trim @ 18 m/s, 200 Hz zero-order-hold discretization, Bryson's-rule
    // Q/R against the assumed stability/control derivatives) — see
    // docs/attitude-lqr.md. Field order: {mode, k_primary, k_rate,
    // k_integral, integral_limit, output_limit_deg}. Re-run the script and
    // update these if the airframe's mass/derivatives/limits change; these
    // are NOT flight-validated, only pole/damping-checked in the script
    // across the 14-22 m/s envelope.
    // output_limit_deg = 35.156 (2026-08-25, was 45.0 -- see git history):
    // reverted on user's explicit request to match legacy FW_control.h's
    // servo-output clamp EXACTLY instead of ArduPilot's "same as Manual's
    // full throw" convention. Legacy's updateFBWA_FW()/updateAUTO_FW()
    // clamp servos[] (its final aileron/elevator/rudder command) to
    // +-400 raw units: "servos[i] = constrain(servos[i], -400, 400);".
    // Those raw units and this codebase's angle_to_pwm_gain (11.378,
    // Actuator.h) are the SAME conversion constant (Actuator.h's own doc
    // comment: "from legacy Actuator.h's angleToPwm()") -- so 400 raw units
    // == 400/11.378 == ~35.156 deg is the exact equivalent limit here.
    // Narrower than Manual's full +-512 raw-PWM throw (~45 deg): legacy
    // capped stabilized-mode output BELOW Manual's max on purpose, unlike
    // ArduPilot's convention -- match that behavior, not ArduPilot's.
    LqrAxisConfig roll{LqrStateMode::AngleAndRate, 3.449792f, 0.409788f, 1.393631f, 17.2f, 35.156f};
    LqrAxisConfig pitch{LqrStateMode::AngleAndRate, 5.205471f, 0.661225f, 1.630465f, 14.9f, 35.156f};
    LqrAxisConfig yaw{LqrStateMode::RateOnly, 1.021439f, 0.0f, 0.0f, 0.0f, 35.156f};

    // Speed scaler: (trim_airspeed_mps / measured_airspeed_mps)^2, clamped to
    // [scaler_min, scaler_max], multiplies each axis's raw LQR output before
    // the final output_limit_deg clamp. See docs/attitude-lqr.md for why a
    // single fixed K plus this post-multiplier is used instead of a
    // gain-scheduled K.
    float trim_airspeed_mps = 18.0f;
    float scaler_min = 0.6f;
    float scaler_max = 1.8f;
    float min_airspeed_for_scaling_mps = 3.0f;  // guards the scaler's divide-by-airspeed

    // Minimum airspeed used in the yaw axis's coordinated-turn feedforward
    // (r_cmd = g*tan(roll)/V) — guards against divide-by-near-zero at
    // very low speed.
    float min_airspeed_for_turn_rate_mps = 5.0f;

    // MAVLink-settable (Params "YAW_CORR_EN") on/off switch for the yaw
    // axis's coordinated-turn feedforward. 1.0 = enabled (rudder fully
    // automatic from roll setpoint, in FBWA/AUTO/GUIDED alike). 0.0 =
    // disabled: rudder_deg forced to 0 (neutral) every update() call instead
    // of running the yaw LQR/feedforward. Stored as float (0.0/1.0) to fit
    // Params' float-only table — see storage/Params.h.
    //
    // Defaulted to 0.0 (OFF) starting 2026-08-18: bench-testing phase wants
    // roll/pitch FBWA validated in isolation first, without the automatic
    // rudder also moving. Flip back to 1.0 (either here or via
    // YAW_CORR_EN=1 in Mission Planner, no reflash needed) once that's
    // confirmed working and coordinated-turn rudder is ready to test too.
    float yaw_correction_enabled = 0.0f;

    // MAVLink-settable (Params "SPD_SCALE_EN") on/off switch for the speed
    // scaler (computeSpeedScaler()'s output) actually being APPLIED to the
    // three axes' raw LQR output. 1.0 = applied as designed (scaler ranges
    // scaler_min..scaler_max depending on airspeed_mps). 0.0 = scaler forced
    // to a flat 1.0x regardless of airspeed -- computeSpeedScaler() still
    // RUNS every update() call and its result still reaches
    // lastSpeedScaler() for telemetry/logging, only the actual multiplication
    // is bypassed.
    //
    // Defaulted to 0.0 (OFF) starting 2026-08-22: ground-testing on the
    // bench reads near-zero airspeed (no pitot flow while stationary), which
    // pushes the scaler to its scaler_max clamp (1.8x) -- amplifying every
    // axis's output ~1.8x more than the 1.0x it runs at during actual cruise
    // flight. That's expected/correct behavior once airborne (ArduPilot's
    // own AP_RollController/AP_PitchController do the same V^-2 scaling in
    // FBWA and every other non-MANUAL mode), but makes bench testing look
    // far twitchier/more sensitive than in-flight behavior actually is, and
    // was confusing ground-test diagnosis of an unrelated issue. Flip back
    // to 1.0 (either here or via SPD_SCALE_EN=1 in Mission Planner, no
    // reflash needed) before flight -- the controller was gain-tuned WITH
    // this scaler active (see docs/attitude-lqr.md), flying with it
    // permanently forced to 1.0x skips the compensation the 14-22 m/s
    // envelope validation in tools/lqr_gain_design.py relies on.
    float speed_scaler_enabled = 0.0f;
};

/**
 * ===========================================================================
 * PERUNTUKAN FILE INI
 * ===========================================================================
 * Fixed-wing 3-axis LQR attitude controller: replaces the legacy per-axis
 * rate-PID stack (FW_roll/pitch/yaw_controller.h) with one
 * LqrAxisController instance per axis (roll and pitch integral-augmented
 * for disturbance rejection — the thesis's external-roll-disturbance
 * scenario; yaw rate-only, commanded to a kinematic coordinated-turn rate
 * computed from the roll setpoint, replacing the legacy rudder-mixing
 * role only — L1/TECS's own load-factor/bank-angle logic is untouched).
 *
 * State feedback is read directly from fc::Imu — no new estimator.
 *
 * INI ADALAH "ORKESTRATOR" 3-AXIS. File ini SENDIRI TIDAK menghitung LQR-nya
 * (hukum kendali u = -K*x ada di LqrAxisController::update(), file lain).
 * Tugas AttitudeController hanya:
 *   1) menyiapkan setpoint per-axis (roll & pitch datang dari luar; yaw
 *      dihitung sendiri via coordinated-turn feedforward),
 *   2) memanggil LqrAxisController::update() tiga kali (roll_, pitch_, yaw_),
 *   3) menskalakan hasilnya terhadap airspeed (speed scaler),
 *   4) meng-clamp ke limit defleksi permukaan kendali,
 *   5) mengembalikan Output{aileron_deg, elevator_deg, rudder_deg} ke caller.
 *
 * DIPANGGIL DARI (siapa yang pakai class ini):
 *   - ModeFbwa::_update()   di src/modes/FixedWingModes.cpp
 *   - ModeAuto::_update()   di src/modes/FixedWingModes.cpp
 *   - ModeGuided::_update() di src/modes/FixedWingModes.cpp
 *   Semua lewat `ctx_.attitude.update(...)`, hasilnya (Output) lalu dikirim
 *   ke `ctx_.actuator.writeAttitude(output)` untuk digerakkan ke servo fisik.
 *
 * ALUR PANGGILAN LOGIC (urutan runtime per satu siklus loop):
 *   FixedWingModes.cpp (_update mode aktif)
 *     -> AttitudeController::update()                 [file ini, .cpp]
 *          -> computeSpeedScaler()                     (private, di bawah)
 *          -> LqrAxisController::update()  (roll_)      -> LqrAxisController.cpp
 *          -> LqrAxisController::update()  (pitch_)      -> LqrAxisController.cpp
 *          -> computeCoordinatedTurnRateDps()          (private, hitung setpoint yaw)
 *          -> LqrAxisController::update()  (yaw_)        -> LqrAxisController.cpp
 *          -> clampToOutputLimit() x3                  (private, batasi output)
 *     -> Actuator::writeAttitude(output)                [balik ke FixedWingModes.cpp]
 * ===========================================================================
 */
class AttitudeController final {
public:
    struct Output {
        float aileron_deg = 0.0f;   // hasil akhir kemudi aileron (roll), sudah di-scale & clamp
        float elevator_deg = 0.0f;  // hasil akhir kemudi elevator (pitch), sudah di-scale & clamp
        float rudder_deg = 0.0f;    // hasil akhir kemudi rudder (yaw), sudah di-scale & clamp
    };

    explicit AttitudeController(const AttitudeControllerConfig& config = AttitudeControllerConfig{});

    /**
     * FUNGSI UTAMA — dipanggil sekali per loop control dari mode aktif
     * (ModeFbwa/ModeAuto/ModeGuided di FixedWingModes.cpp).
     *
     * nav_roll_deg/nav_pitch_deg: setpoints from Navigation (L1/TECS demand).
     *   - Di FBWA: berasal dari stick radio yang sudah di-map jadi derajat
     *     (lihat ModeFbwa::mapStickToDeg() di FixedWingModes.cpp).
     *   - Di AUTO/GUIDED: berasal dari Navigation (L1/TECS), yaitu
     *     mission.nav_roll_deg / mission.nav_pitch_deg.
     * imu: latest attitude/rate snapshot (roll/pitch aktual + gyro rate),
     *   dipakai sebagai measured_primary & measured_rate ke LqrAxisController.
     * airspeed_mps: for the speed scaler and yaw feedforward.
     * dt: seconds since the last call — dipakai untuk integrasi (integral
     *   term) di dalam LqrAxisController.
     *
     * INTERNAL CALL ORDER (lihat implementasi di AttitudeController.cpp):
     *   1. computeSpeedScaler(airspeed_mps)              -> scaler
     *   2. roll_.update(nav_roll_deg, imu.roll, imu.gyro_x, dt)   -> roll_raw
     *   3. pitch_.update(nav_pitch_deg, imu.pitch, imu.gyro_y, dt)-> pitch_raw
     *   4. computeCoordinatedTurnRateDps(nav_roll_deg, airspeed) -> yaw setpoint (rate)
     *   5. yaw_.update(yaw_setpoint, imu.gyro_z, 0, dt)           -> yaw_raw
     *   6. clampToOutputLimit(raw * scaler, limit) untuk ketiga axis -> Output
     *
     * Return value ini lalu dikonsumsi caller (mode aktif) dan diteruskan ke
     * Actuator::writeAttitude() untuk benar-benar menggerakkan servo.
     */
    Output update(float nav_roll_deg, float nav_pitch_deg, const ImuData& imu,
                 float airspeed_mps, float dt);

    // Reset integral state ketiga axis (roll_, pitch_, yaw_) ke nol.
    // Dipanggil dari ModeFbwa::_enter(), ModeAuto::_enter(), ModeGuided::_enter()
    // di FixedWingModes.cpp — supaya integrator tidak "membawa" windup lama
    // saat baru masuk mode tersebut.
    void resetIntegrators();

    // Nyalakan/matikan aksi integral pada ketiga axis sekaligus (mis. saat
    // throttle rendah / belum armed, agar integrator tidak mengumpulkan error).
    void setIntegralEnabled(bool enabled);

    // Getter untuk logging/telemetry — nilai integral_state_ axis roll & pitch,
    // scaler airspeed terakhir, dan setpoint rate yaw terakhir. Tidak dipakai
    // dalam perhitungan kendali, hanya untuk observasi/debug.
    float rollIntegratorState() const;
    float pitchIntegratorState() const;
    float lastSpeedScaler() const;
    float lastYawRateSetpointDps() const;

private:
    // Menghitung setpoint RATE yaw (deg/s) dari coordinated-turn kinematik:
    //   r = g * tan(roll) / V
    // Dipanggil oleh update() sebelum memanggil yaw_.update(). Ini yang
    // menggantikan peran rudder-mixing manual pada implementasi lama.
    static float computeCoordinatedTurnRateDps(float roll_deg, float airspeed_mps,
                                               float min_airspeed_mps);

    // Menghitung faktor pengali output berbasis rasio airspeed trim vs
    // airspeed terukur (dikuadratkan, di-clamp). Dipanggil di awal update()
    // dan hasilnya dipakai untuk mengalikan roll_raw/pitch_raw/yaw_raw
    // sebelum di-clamp ke output_limit_deg.
    float computeSpeedScaler(float airspeed_mps) const;

    // Membatasi nilai akhir (setelah dikali scaler) ke rentang
    // [-limit_deg, +limit_deg]. Dipanggil tiga kali di akhir update(), satu
    // per axis, sebagai langkah terakhir sebelum Output dikembalikan.
    static float clampToOutputLimit(float value_deg, float limit_deg);

    AttitudeControllerConfig config_{};
    // Tiga instance LqrAxisController — satu per axis. Masing-masing punya
    // gain K sendiri (dari config_.roll/pitch/yaw) tapi logic perhitungannya
    // sama persis, didefinisikan satu kali di LqrAxisController::update().
    LqrAxisController roll_;
    LqrAxisController pitch_;
    LqrAxisController yaw_;

    float last_speed_scaler_ = 1.0f;          // untuk lastSpeedScaler() (telemetry)
    float last_yaw_rate_setpoint_dps_ = 0.0f; // untuk lastYawRateSetpointDps() (telemetry)
};

}  // namespace fc
