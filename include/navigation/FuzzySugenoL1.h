#pragma once

#include <stdint.h>

namespace fc {

/**
 * Fuzzy Sugeno orde-0 yang menyetel periode L1 dari |e| dan d|e|/dt.
 *
 * Desain (himpunan, rule base, E_MAX, DE_MAX) berasal dari data uji L1
 * konvensional 30/10/2026; lihat docs/fuzzy-sugeno-l1.md dan
 * tools/flight_analysis/fuzzy_sugeno_2026-10-30/DESAIN_FUZZY_SUGENO.md.
 *
 * Empat langkah, sama dengan dokumen desain:
 *   0. normalisasi    e_fn  = min(1, |e| / E_MAX)
 *                     de_fn = clip(d|e|/dt / DE_MAX, -1, 1)
 *   1. fuzzifikasi    mu Kecil/Sedang/Besar (e_fn), Negatif/Zero/Positif (de_fn)
 *   3. inferensi      alpha = min(mu_e, mu_de) untuk 9 aturan
 *   4. defuzzifikasi  z = sum(alpha * c) / sum(alpha)
 *                     periode = P0 + S * z
 * (langkah 2, rule base, adalah tabel kRuleC di FuzzySugenoL1.cpp)
 */

/** Hasil satu kali perhitungan fuzzy (semua nilai antara, untuk log dan uji). */
struct FuzzyL1Output {
    float e_fn = 0.0f;           // |e| ternormalisasi, 0..1
    float de_fn = 0.0f;          // d|e|/dt ternormalisasi, -1..1
    float mu_e[3] = {};          // Kecil, Sedang, Besar
    float mu_de[3] = {};         // Negatif, Zero, Positif
    float alpha[3][3] = {};      // firing strength [himpunan e][himpunan de]
    float sum_alpha = 0.0f;
    float z = 0.0f;              // keluaran ternormalisasi, -1..1
    float period_s = 15.0f;      // periode L1 hasil fuzzy (s)
};

/**
 * Fuzzy Sugeno murni: |e| (m) dan d|e|/dt (m/s, negatif = mendekati garis)
 * menjadi periode L1. Tidak menyimpan keadaan apa pun.
 */
FuzzyL1Output fuzzySugenoL1(float abs_e_m, float de_mps);

/**
 * Saklar dan penyimpan hasil terakhir untuk mode AUTO.
 *
 * Dipanggil sekali per siklus AUTO, SESUDAH Navigation::navigate() (jadi e dan
 * laju e dari siklus ini sudah dihitung L1). Periode yang dikembalikan dipakai
 * L1 pada siklus berikutnya.
 *
 *  - Param FUZZY_ENABLE = 0, atau `allowed` = false: kembalikan periode tetap
 *    (param NAVL1_PERIOD) = L1 konvensional.
 *  - FUZZY_ENABLE = 1: kembalikan periode hasil fuzzy (11..19 s).
 *
 * Kedua param dibaca langsung tiap panggilan (referensi ke tabel param), jadi
 * perubahan dari Mission Planner berlaku saat itu juga.
 */
class FuzzySugenoL1 {
public:
    FuzzySugenoL1(const float& enable_param, const float& fixed_period_param);

    /**
     * @param allowed          false saat fuzzy tidak boleh bekerja (misi selesai,
     *                         keluar AUTO)
     * @param cte_m            crosstrack error bertanda dari L1Controller
     * @param xtrack_vel_mps   d(cte)/dt dari L1Controller (tanda sama dengan cte)
     * @return periode L1 (s) untuk dipasang ke L1Controller::setPeriod()
     */
    float update(bool allowed, float cte_m, float xtrack_vel_mps);

    /** true bila siklus terakhir memakai periode hasil fuzzy. */
    bool active() const { return active_; }

    /** Hasil fuzzy terakhir (nol semua bila tidak aktif). */
    const FuzzyL1Output& last() const { return last_; }

private:
    const float& enable_param_;        // param FUZZY_ENABLE
    const float& fixed_period_param_;  // param NAVL1_PERIOD
    bool active_ = false;
    FuzzyL1Output last_{};
};

}  // namespace fc
