// Host-side unit test for fc::FuzzyL1Tuner: the rule base of proposal Tabel 3.6
// with the membership functions derived by tools/flight_analysis/fuzzy_mf_from_data.py.
// Uses the real eFLL library and a stub L1Controller. Build + run from the repo
// root after `pio pkg install` has fetched eFLL (one line):
//   g++ -std=c++17 -Itest/host/stubs -Iinclude -I.pio/libdeps/teensy41/eFLL test/host/test_fuzzy_l1_tuner.cpp src/navigation/FuzzyL1Tuner.cpp .pio/libdeps/teensy41/eFLL/*.cpp -o test_fuzzy && ./test_fuzzy
#include <cmath>
#include <cstdio>

#include "navigation/FuzzyL1Tuner.h"

static int g_failures = 0;
#define CHECK_NEAR(actual, expected, tol)                                                          \
    do {                                                                                           \
        const float a_ = (actual);                                                                 \
        if (!(std::fabs(a_ - (expected)) <= (tol))) {                                              \
            std::printf("FAIL line %d: %s = %.3f, expected %.3f\n", __LINE__, #actual, a_,         \
                        static_cast<double>(expected));                                            \
            ++g_failures;                                                                          \
        }                                                                                          \
    } while (0)

int main()
{
    fc::FuzzyL1Tuner tuner;  // default config = data-derived breakpoints, base 14 s, limits 12-16 s
    tuner.begin();

    // --- Tabel 3.6, one point well inside each cell (|e| m, d|e|/dt m/s) -> dT s ---
    // Negatif (closing on the path)
    CHECK_NEAR(tuner.evaluate(1.0f, -3.0f), +2.0f, 0.05f);   // Kecil  -> Naik
    CHECK_NEAR(tuner.evaluate(10.0f, -3.0f), 0.0f, 0.05f);   // Sedang -> Tetap
    CHECK_NEAR(tuner.evaluate(40.0f, -3.0f), -2.0f, 0.05f);  // Besar  -> Turun
    // Nol
    CHECK_NEAR(tuner.evaluate(1.0f, 0.0f), 0.0f, 0.05f);     // Kecil  -> Tetap
    CHECK_NEAR(tuner.evaluate(10.0f, 0.0f), 0.0f, 0.05f);    // Sedang -> Tetap
    CHECK_NEAR(tuner.evaluate(40.0f, 0.0f), -2.0f, 0.05f);   // Besar  -> Turun
    // Positif (moving away from the path)
    CHECK_NEAR(tuner.evaluate(1.0f, 3.0f), 0.0f, 0.05f);     // Kecil  -> Tetap
    CHECK_NEAR(tuner.evaluate(10.0f, 3.0f), -2.0f, 0.05f);   // Sedang -> Turun
    CHECK_NEAR(tuner.evaluate(40.0f, 3.0f), -2.0f, 0.05f);   // Besar  -> Turun

    // Out-of-universe inputs saturate instead of dropping to "no rule fired".
    CHECK_NEAR(tuner.evaluate(500.0f, 0.0f), -2.0f, 0.05f);
    CHECK_NEAR(tuner.evaluate(1.0f, -50.0f), +2.0f, 0.05f);
    CHECK_NEAR(tuner.evaluate(1.0f, 50.0f), 0.0f, 0.05f);

    // Between cells the output is graded, not a jump (|e| 5 m: Kecil 5/7, Sedang 2/7).
    const float mid = tuner.evaluate(5.0f, -3.0f);
    if (!(mid > 0.2f && mid < 1.9f)) {
        std::printf("FAIL: graded output expected between cells, got %.3f\n", mid);
        ++g_failures;
    }

    // --- update(): sign of d|e|/dt comes from sign(cte) * cross-track velocity ---
    fc::L1Controller l1;
    tuner.setEnabled(true);
    tuner.resetToBase(l1);
    CHECK_NEAR(l1.period(), 14.0f, 1e-4f);

    // cte = -1 m (left of path) with velocity +3 m/s (moving right = closing): Naik.
    l1.xtrack_velocity_mps_ = 3.0f;
    tuner.update(l1, -1.0f, 0.005f);
    CHECK_NEAR(tuner.lastCrosstrackErrorRateMps(), -3.0f, 1e-4f);
    CHECK_NEAR(l1.period(), 16.0f, 0.05f);
    CHECK_NEAR(tuner.lastDeltaPeriod(), 2.0f, 0.05f);

    // cte = +40 m and growing: Turun, period clamped at the 12 s floor.
    tuner.update(l1, 40.0f, 0.005f);
    CHECK_NEAR(l1.period(), 12.0f, 0.05f);

    // Tighter limits clamp the output.
    tuner.setPeriodLimits(13.0f, 15.0f);
    tuner.update(l1, 40.0f, 0.005f);
    CHECK_NEAR(l1.period(), 13.0f, 1e-4f);
    tuner.setPeriodLimits(12.0f, 16.0f);

    // Disabled: period goes back to base and is no longer touched.
    tuner.applyEnabled(false, l1);
    CHECK_NEAR(l1.period(), 14.0f, 1e-4f);
    tuner.update(l1, 40.0f, 0.005f);
    CHECK_NEAR(l1.period(), 14.0f, 1e-4f);
    CHECK_NEAR(tuner.lastDeltaPeriod(), 0.0f, 1e-4f);

    if (g_failures == 0) {
        std::printf("test_fuzzy_l1_tuner: ALL PASSED\n");
        return 0;
    }
    std::printf("test_fuzzy_l1_tuner: %d FAILED\n", g_failures);
    return 1;
}
