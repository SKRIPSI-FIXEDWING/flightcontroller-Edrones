#include "navigation/FuzzySugenoL1.h"

#include <math.h>

namespace fc {

namespace {

// ---------------------------------------------------------------------------
// Parameter desain (DESAIN_FUZZY_SUGENO.md bagian 0). Ubah di sini saja.
// ---------------------------------------------------------------------------
constexpr float kEMax = 20.0f;   // E_MAX (m): |e| >= 20 m dianggap maksimum
constexpr float kDeMax = 13.0f;  // DE_MAX (m/s): batas normalisasi d|e|/dt
constexpr float kP0 = 15.0f;     // periode dasar (s), dipakai bila z = 0
constexpr float kS = 4.0f;       // faktor skala (s): z = -1..+1 -> 11..19 s

// Langkah 2, rule base. Isi = konstanta c (Sugeno orde-0); periode = 15 + 4c.
//   c = -1 -> 11 s (agresif), 0 -> 15 s (tetap), +0.5 -> 17 s, +1 -> 19 s (halus)
// Baris = |e| (Kecil, Sedang, Besar), kolom = d|e|/dt (Negatif, Zero, Positif).
constexpr float kRuleC[3][3] = {
    //  Negatif  Zero   Positif
    {-1.0f, -1.0f, -1.0f},  // Kecil  -> 11 s
    {0.0f, 0.0f, 0.0f},     // Sedang -> 15 s
    {0.5f, 0.0f, 0.0f},     // Besar  -> 17 s bila mendekat cepat, selain itu 15 s
};

float clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Sisi naik dari a (mu = 0) ke b (mu = 1), dipotong ke 0..1.
float rise(float x, float a, float b)
{
    return clamp((x - a) / (b - a), 0.0f, 1.0f);
}

}  // namespace

FuzzyL1Output fuzzySugenoL1(float abs_e_m, float de_mps)
{
    FuzzyL1Output out;

    // Langkah 0: normalisasi.
    out.e_fn = clamp(abs_e_m / kEMax, 0.0f, 1.0f);
    out.de_fn = clamp(de_mps / kDeMax, -1.0f, 1.0f);
    const float u = out.e_fn;
    const float y = out.de_fn;

    // Langkah 1: fuzzifikasi (titik 0,25 / 0,5 / 0,75 dan -0,5 / 0 / 0,5).
    out.mu_e[0] = 1.0f - rise(u, 0.25f, 0.5f);                          // Kecil
    out.mu_e[1] = fminf(rise(u, 0.25f, 0.5f), 1.0f - rise(u, 0.5f, 0.75f));  // Sedang
    out.mu_e[2] = rise(u, 0.5f, 0.75f);                                 // Besar

    out.mu_de[0] = 1.0f - rise(y, -0.5f, 0.0f);                         // Negatif (mendekat)
    out.mu_de[1] = fminf(rise(y, -0.5f, 0.0f), 1.0f - rise(y, 0.0f, 0.5f));  // Zero
    out.mu_de[2] = rise(y, 0.0f, 0.5f);                                 // Positif (menjauh)

    // Langkah 3: inferensi, alpha = min(mu_e, mu_de) untuk tiap aturan.
    // Langkah 4: defuzzifikasi, rata-rata berbobot z = sum(alpha c) / sum(alpha).
    float sum_alpha_c = 0.0f;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const float alpha = fminf(out.mu_e[i], out.mu_de[j]);
            out.alpha[i][j] = alpha;
            out.sum_alpha += alpha;
            sum_alpha_c += alpha * kRuleC[i][j];
        }
    }
    // Himpunan saling menyambung, jadi sum_alpha selalu > 0; pengaman saja.
    out.z = (out.sum_alpha > 0.0f) ? sum_alpha_c / out.sum_alpha : 0.0f;
    out.period_s = kP0 + kS * out.z;
    return out;
}

FuzzySugenoL1::FuzzySugenoL1(const float& enable_param, const float& fixed_period_param)
    : enable_param_(enable_param), fixed_period_param_(fixed_period_param)
{
}

float FuzzySugenoL1::update(bool allowed, float cte_m, float xtrack_vel_mps)
{
    active_ = allowed && enable_param_ >= 0.5f;
    if (!active_) {
        last_ = FuzzyL1Output{};
        last_.period_s = fixed_period_param_;
        return fixed_period_param_;  // L1 konvensional
    }

    // d|e|/dt = tanda(e) * de/dt: negatif = mendekati garis di sisi mana pun.
    const float de_mps = (cte_m < 0.0f) ? -xtrack_vel_mps : xtrack_vel_mps;
    last_ = fuzzySugenoL1(fabsf(cte_m), de_mps);
    return last_.period_s;
}

}  // namespace fc
