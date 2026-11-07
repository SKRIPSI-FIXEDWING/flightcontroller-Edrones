#pragma once

namespace fc {

/**
 * Hukum navigasi lateral PID (2026-11-07), pembanding L1 untuk skripsi.
 * Dipakai L1Controller menggantikan rumus L1 HANYA bila NAV_PID_ENABLE = 1
 * (default 0 = mati). Murni aritmetika (tanpa Arduino), supaya bisa dites di
 * host: test/host/test_nav_pid.cpp.
 *
 *   a_lat = Kp e + Kd de/dt + Ki int(e)        (m/s^2)
 *
 * e = cross-track error (garis) atau galat radius (lingkaran), tanda sama
 * dengan L1Controller::crosstrackError(); de/dt = crosstrackVelocityMps().
 * Gain default SETARA L1 pada periode 14 s, damping 0.707 (linearisasi error
 * kecil, Park et al. 2004): Kp = w^2, Kd = 2 zeta w, w = 2 pi / 14; Ki = Kp/20
 * (Ti = 20 s). Sama dengan tools/flight_analysis/sim_guidance_compare.py.
 */
struct NavPidConfig {
    float enable = 0.0f;    // param NAV_PID_ENABLE: 1 = PID, 0 = L1 (default)
    float kp = 0.2014f;     // param NAV_PID_KP [1/s^2]
    float kd = 0.6347f;     // param NAV_PID_KD [1/s]
    float ki = 0.0101f;     // param NAV_PID_KI [1/s^3]
    // Anti-windup: |Ki int(e)| maksimum, setara bank 10 deg (g tan 10).
    float i_limit_mss = 1.729f;
};

class NavPid final {
public:
    explicit NavPid(const NavPidConfig& config = NavPidConfig{}) : config_(config) {}

    bool enabled() const { return config_.enable >= 0.5f; }
    void setEnabled(bool on);
    void setGains(float kp, float kd, float ki);
    const NavPidConfig& config() const { return config_; }

    /** Percepatan lateral (m/s^2), + = belok kanan. dt dalam detik.
     * error_limit_m > 0: |e| dijepit ke nilai ini untuk suku P, dan integrator
     * berhenti di luar batas (anti-windup saat menangkap garis). 0 = tanpa batas. */
    float update(float error_m, float error_rate_mps, float dt_s, float error_limit_m = 0.0f);

    /** Batas galat untuk sudut potong maksimum: di |e| >= batas, suku P
     * setimbang dengan suku D saat wahana mendekat dengan kecepatan
     * v sin(intercept). Setara batas sin(nu1) <= 0.7071 di L1. */
    float interceptErrorLimit(float groundspeed_mps, float max_intercept_rad) const;

    void resetIntegrator() { integrator_mss_ = 0.0f; }
    float integratorMss() const { return integrator_mss_; }

private:
    NavPidConfig config_{};
    float integrator_mss_ = 0.0f;  // = Ki int(e), sudah dijepit
};

}  // namespace fc
