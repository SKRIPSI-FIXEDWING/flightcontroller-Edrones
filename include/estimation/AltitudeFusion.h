#pragma once

#include <stdint.h>

namespace fc {

struct AltitudeFusionConfig {
    // --- Process model: x = [height, climb rate, accel bias] -------------
    // Vertical acceleration (BNO055 gravity-free linear accel projected on
    // gravity, see verticalAccelUpMss()) is the *input*, so the filter keeps
    // tracking motion between barometer samples and can tell a genuine
    // climb (accel pulse) from a barometer disturbance (no accel).
    // Dulu 0,2: terlalu optimistis. Terbang 2026-10-22: galat akselerasi
    // vertikal (BNO055) median 0,1-0,3 m/s^2, puncak ~1,2-1,4 m/s^2 selama
    // beberapa detik (paling sering saat bank 25-40 deg) -> P tinggi terlalu
    // kecil, baro ditolak ~35-55% waktu terbang. Lihat docs/altitude-fusion.md.
    float accel_noise_mss = 0.7f;     // white accel noise density [(m/s^2)/sqrt(Hz)]
    float accel_bias_walk = 0.005f;   // accel bias random walk [(m/s^2)/sqrt(s)]
    float initial_accel_bias_sigma_mss = 0.5f;

    // --- Barometer ---------------------------------------------------------
    // 1-sigma noise of Barometer::raw_altitude_m (pre-Kalman); ~0.75 m on
    // this MS5607 in the 2026-08-22 logs.
    float baro_noise_m = 0.8f;
    // Reject a baro sample whose innovation exceeds this many sigma. Light /
    // airflow disturbances on the barometer show up as 10-25 m steps over
    // 1-5 s with no matching acceleration.
    float baro_gate_sigma = 3.0f;
    // GPS available and baro rejected continuously this long -> the baro is
    // the odd one out: re-anchor the baro offset (bias) instead of moving
    // the altitude. Short, so a one-sample I2C glitch is not absorbed but a
    // 10-20 m light step is, almost immediately.
    float baro_reanchor_s = 0.3f;
    // After a rejection the baro must stay inside the gate this long before
    // it is trusted again (disturbances wander across the gate edge).
    float baro_recovery_s = 1.5f;
    // Samples beyond this many sigma are down-weighted (Huber-style) instead
    // of used at full weight.
    float baro_soft_sigma = 1.5f;
    // No GPS to arbitrate: if the baro stays rejected this long, give up and
    // follow it (re-initialize height from baro, climb rate 0).
    // Dulu 3 s; tanpa GPS penolakan tidak memberi manfaat (uji bench: rms
    // sama dengan baro mentah) tetapi drift inersia 3 s saat belok = 10-20 m.
    float baro_no_gps_reset_s = 1.0f;
    // With GPS, how fast the slowly varying baro offset is tracked.
    // Tanpa GPS offset meluruh ke 0 dengan tau yang sama: tidak ada lagi yang
    // bisa memastikannya, jadi estimasi pelan-pelan kembali ke baro mentah.
    float baro_bias_tau_s = 40.0f;
    // Batas mutlak offset baro. Gangguan cahaya di bench 10-25 m; offset
    // lebih besar dari ini hampir pasti berasal dari tinggi inersia yang
    // drift (terbang 2026-10-22: +45 m, dan +500 m setelah mendarat),
    // bukan dari baro.
    float baro_bias_max_m = 30.0f;

    // --- GPS (Here4 / NEO-F9P) --------------------------------------------
    float gps_noise_m = 2.0f;
    float gps_gate_sigma = 4.0f;
    // NEO-F9P vertical velocity (Fix2 ned_velocity.z) is far more accurate
    // than height; it pins the climb-rate state and, through that, the
    // accelerometer bias.
    float gps_climb_noise_mps = 0.3f;
    // GPS rejected this long -> re-reference GPS to the current estimate.
    float gps_rereference_s = 20.0f;
    // GPS counts as "available" for this long after the last accepted fix.
    float gps_timeout_s = 2.0f;
    // Sampel GPS terakhir (diterima atau ditolak) dipakai sebagai "suara
    // ketiga" saat baro ditolak, asal umurnya tidak lebih dari ini.
    // Ambang setuju/tidak: baro_gate_sigma * sqrt(gps_noise^2 + baro_noise^2).
    float gps_vote_max_age_s = 0.6f;
};

struct AltitudeFusionData {
    float altitude_m = 0.0f;       // relative to the barometer's boot-time ground reference
    float climb_rate_mps = 0.0f;   // positive = climbing
    float accel_bias_mss = 0.0f;
    float baro_bias_m = 0.0f;      // estimated baro offset (disturbance / drift)
    float altitude_sigma_m = 0.0f;
    bool baro_rejected = false;    // baro currently gated out / on hold
    bool gps_active = false;       // GPS currently aiding
    bool valid = false;
    uint32_t baro_reject_count = 0;
    uint32_t baro_reanchor_count = 0;
    uint32_t inertial_reset_count = 0;  // tinggi inersia di-reset ke baro (baro & GPS sepakat)
};

/**
 * Barometer + vertical-accelerometer (+ optional GPS height/velocity)
 * altitude estimator that is robust to barometer disturbances.
 *
 * Why: MS5611/MS5607 readings are sensitive to light and local airflow
 * (the 2026-10-13 tlog shows a +2.5 hPa ~ -20 m swing that follows heading
 * and returns when rotated back, temperature unchanged). A plain baro
 * Kalman filter just follows it. This estimator instead:
 *   1. predicts height/climb from the accelerometer (with bias estimation),
 *   2. gates baro samples inconsistent with that prediction and holds off
 *      until the baro has been consistent again for a while,
 *   3. when GPS is available, lets the Here4 height and vertical velocity
 *      arbitrate -- voting 2-dari-3 dengan sampel GPS *terkini*: baro yang
 *      menyimpang -> gangguan diserap ke offset baro (dibatasi); tinggi
 *      inersia yang menyimpang (drift 0,5-1,5 m/s^2 saat belok miring
 *      lama, terbang 2026-10-22) -> tinggi di-reset ke baro; tidak ada
 *      mayoritas -> diperlakukan seperti tanpa GPS, and
 *   4. falls back to following the baro after a timeout, so a genuine
 *      change can never be ignored forever.
 *
 * No Arduino dependencies -- time is whatever predict()'s dt accumulates,
 * so it can be replayed offline.
 */
class AltitudeFusion final {
public:
    explicit AltitudeFusion(const AltitudeFusionConfig& config = {});

    /** Forget everything; the next baro sample re-initializes. */
    void reset();

    /** Propagate by dt_s with the vertical (up-positive, gravity-free) accel. */
    void predict(float accel_up_mss, float dt_s);

    /** New barometer sample: height above the boot ground reference. */
    void updateBaro(float baro_altitude_m);

    /**
     * New GNSS sample: MSL height and vertical velocity (positive = climbing,
     * i.e. -ned_velocity.z). gps_good = 3D fix with sane HDOP.
     */
    void updateGps(float gps_msl_m, float gps_climb_rate_mps, bool gps_good);

    AltitudeFusionData data() const;

private:
    // Scalar measurement of state `index` (0 height, 1 climb rate).
    void scalarUpdate(int index, float innovation, float measurement_variance);
    bool gpsActive() const;
    bool gpsVoteFresh() const;
    void resetHeightToBaro(float baro_corrected_m, float climb_mps);

    AltitudeFusionConfig config_{};

    bool initialized_ = false;
    float height_ = 0.0f;
    float climb_ = 0.0f;
    float accel_bias_ = 0.0f;
    float p_[3][3] = {};

    float baro_bias_ = 0.0f;
    float time_s_ = 0.0f;
    float last_baro_time_s_ = 0.0f;
    float baro_reject_since_s_ = -1.0f;  // <0: not rejecting
    float baro_last_reject_s_ = -1.0e9f;
    float baro_in_gate_since_s_ = -1.0f;
    bool baro_rejected_ = false;

    bool gps_ref_set_ = false;
    float gps_ref_ = 0.0f;
    float last_gps_accept_s_ = -1.0e9f;
    float gps_reject_since_s_ = -1.0f;
    // Sampel GPS terakhir apa adanya (relatif gps_ref_), untuk voting.
    float last_gps_sample_s_ = -1.0e9f;
    float last_gps_height_ = 0.0f;
    float last_gps_climb_ = 0.0f;

    uint32_t baro_reject_count_ = 0;
    uint32_t baro_reanchor_count_ = 0;
    uint32_t inertial_reset_count_ = 0;
};

}  // namespace fc
