#pragma once

#include <stdint.h>

#include "drivers/Imu.h"

namespace fc {

/**
 * Parameter kendali FBWA Khageswara -- nilai asli dari dokumentasi
 * (FW_control.h::updateFBWA_FW(), baris 93-155). Bukan LQR: ini P/PD
 * dengan integral yang dinonaktifkan (I = 0).
 */
struct FbwaKhageswaraConfig {
    // --- Pemetaan stick -> sudut perintah (baris 102-103 legacy) ---
    float roll_cmd_multiplier = 1.3f;
    float pitch_cmd_multiplier = 1.1f;
    int16_t roll_cmd_min_deg = -35;
    int16_t roll_cmd_max_deg = 35;
    int16_t pitch_cmd_min_deg = -25;  // asimetris: turun -25, naik +35
    int16_t pitch_cmd_max_deg = 35;

    // --- Gain (rate_*_P / _I / _D legacy) ---
    float roll_p = 10.0f;
    float roll_i = 0.0f;
    float roll_d = 0.0f;
    float pitch_p = 10.0f;
    float pitch_i = 0.0f;
    float pitch_d = 0.2f;

    // --- Trim, integral, saturasi ---
    float trim_roll = 0.0f;                // "trim nol"
    float trim_pitch = 0.0f;
    float integral_limit = 15.0f;          // constrain(roll_int/pitch_int, -15, 15)
    uint16_t throttle_reset_pwm = 1100;    // integral di-reset bila throttle manual < 1100
    float output_limit_raw = 400.0f;       // servos[i] = constrain(servos[i], -400, 400)
    float raw_per_deg = 11.378f;           // raw unit -> derajat (= angle_to_pwm_gain Actuator)

    // --- Mode AUTO memakai kendali yang SAMA dengan FBWA (updateAuto()) ---
    // Tanda perintah AUTO relatif terhadap perintah stick FBWA:
    //   roll_cmd  = auto_roll_cmd_sign  * nav_roll_deg   (nav_roll > 0 = ke kanan)
    //   pitch_cmd = auto_pitch_cmd_sign * nav_pitch_deg  (nav_pitch > 0 = naik)
    // Roll: +1 (2026-10-14, dari hasil uji bangku). Dengan -1 aileron AUTO
    // meminta arah yang BERLAWANAN dengan stick FBWA untuk bank yang sama
    // (L1 minta kanan -> PWM turun, padahal stick kanan di FBWA -> PWM naik).
    // +1 membuat nav_roll > 0 menghasilkan keluaran aileron yang sama dengan
    // stick kanan di FBWA.
    // Pitch: -1 (arah elevator "minta naik" sudah sesuai hasil uji).
    // Balik tanda di dua konstanta ini SAJA bila uji bangku menunjukkan
    // AUTO berbelok/menanjak ke arah yang salah padahal FBWA benar.
    // Roll +1: nav_roll_deg yang masuk ke sini sudah berkonvensi stick
    // (positif = bank kanan, sama dengan stick kanan di FBWA), jadi diteruskan
    // apa adanya. Koreksi arah navigasi BUKAN di sini, melainkan di
    // NavigationConfig::nav_roll_sign. Pitch -1 (arah "minta naik" sudah benar).
    float auto_roll_cmd_sign = 1.0f;
    float auto_pitch_cmd_sign = -1.0f;

    // --- Polaritas keluaran aileron (Params "AIL_OUT_SIGN") ---
    // Legacy menulis aileron sebagai  PWM = 1500 - u_roll  (Actuator.h:
    // fw_servos_out_fbwa()), sedangkan Actuator::writeAttitude() memakai
    //   PWM = center + gain * aileron_deg.
    // Rumus roll legacy (u = Kp*(roll + roll_cmd)) hanya berumpan-balik negatif
    // bila PWM = center - u. Faktor ini dikalikan ke aileron_deg SETELAH
    // saturasi, di FBWA dan AUTO sekaligus:
    //   +1.0 = apa adanya (perilaku sebelum parameter ini ada)
    //   -1.0 = polaritas legacy (PWM = center - u)
    // Nilai < 0 dibaca -1, selainnya +1 (0 tidak mematikan aileron).
    // Elevator dan rudder tidak terpengaruh. Lihat docs/attitude-khageswara.md
    // untuk prosedur uji bangku penentuan tandanya.
    float aileron_output_sign = 1.0f;
};

/**
 * Kendali FBWA persis program Khageswara, dipisah dari LQR.
 *
 * URUTAN HITUNGAN (satu siklus update(), lihat FbwaKhageswaraController.cpp):
 *   1. Stick PWM  -> sudut perintah roll_cmd / pitch_cmd
 *   2. Perbarui integral (clamp, reset saat disarm / throttle rendah)
 *   3. Hitung u_roll   (P + D gyro_x)
 *   4. Hitung u_pitch  (P + D gyro_y)
 *   5. u_yaw = 0
 *   6. Saturasi +-400 raw, ubah ke derajat
 */
class FbwaKhageswaraController final {
public:
    struct Output {
        float aileron_deg = 0.0f;
        float elevator_deg = 0.0f;
        float rudder_deg = 0.0f;  // selalu 0 (u_yaw = 0)
    };

    explicit FbwaKhageswaraController(const FbwaKhageswaraConfig& config = FbwaKhageswaraConfig{});

    // ch_*: PWM mentah (us) dari Radio. dt: detik sejak panggilan terakhir.
    Output update(uint16_t ch_roll, uint16_t ch_pitch, const ImuData& imu, bool armed,
                  uint16_t ch_throttle, float dt);

    // Mode AUTO: setpoint sudut dari L1/TECS (nav_roll_deg/nav_pitch_deg),
    // melewati loop P/PD, integral, dan saturasi yang SAMA dengan update().
    // Integral hanya di-reset saat disarm (throttle di AUTO manual, bukan
    // kondisi reset). Output dalam derajat, siap ke Actuator::writeAttitude().
    Output updateAuto(float nav_roll_deg, float nav_pitch_deg, const ImuData& imu, bool armed,
                      float dt);

    void resetIntegrators();

    // Untuk telemetry/logging.
    float rollIntegrator() const;
    float pitchIntegrator() const;

private:
    // Langkah 1
    float rollCommandDeg(uint16_t ch_roll) const;
    float pitchCommandDeg(uint16_t ch_pitch) const;
    // Langkah 2
    void updateIntegrals(const ImuData& imu, bool armed, uint16_t ch_throttle, float dt);
    // Langkah 3-6 (dipakai bersama oleh update() dan updateAuto())
    Output computeOutput(float roll_cmd_deg, float pitch_cmd_deg, const ImuData& imu) const;
    // Langkah 3 & 4
    float rollControl(float roll_deg, float roll_cmd_deg, float gyro_dps) const;
    float pitchControl(float pitch_deg, float pitch_cmd_deg, float gyro_dps) const;
    // Langkah 6
    float saturateToDeg(float u_raw) const;

    FbwaKhageswaraConfig config_{};
    float roll_int_ = 0.0f;
    float pitch_int_ = 0.0f;
};

}  // namespace fc
