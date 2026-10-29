// Host-side stress test for fc::FuzzyL1Tuner: many hours' worth of control
// cycles with random, extreme and non-finite inputs. eFLL malloc()s and
// free()s its composition points on every defuzzify(), so on the Teensy a
// leak would end in vApplicationMallocFailedHook (FC halts) mid-flight.
// Checks: heap in use does not grow, output period always finite and inside
// [min, max], and the time per update. Build + run from the repo root (one line):
//   g++ -std=c++17 -O2 -Itest/host/stubs -Iinclude -I.pio/libdeps/teensy41/eFLL test/host/test_fuzzy_stress.cpp src/navigation/FuzzyL1Tuner.cpp .pio/libdeps/teensy41/eFLL/*.cpp -o test_fuzzy_stress && ./test_fuzzy_stress
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

#if defined(__APPLE__)
#include <malloc/malloc.h>
static size_t heapInUse()
{
    malloc_statistics_t st;
    malloc_zone_statistics(nullptr, &st);
    return st.size_in_use;
}
#else
#include <malloc.h>
static size_t heapInUse()
{
    return mallinfo2().uordblks;
}
#endif

#include "navigation/FuzzyL1Tuner.h"

static int g_failures = 0;
#define CHECK(cond)                                             \
    do {                                                        \
        if (!(cond)) {                                          \
            std::printf("FAIL line %d: %s\n", __LINE__, #cond); \
            ++g_failures;                                       \
        }                                                       \
    } while (0)

// 200 Hz control loop: 3 hours of flight.
constexpr long kCycles = 200L * 3600L * 3L;

int main()
{
    fc::FuzzyL1Tuner tuner;
    tuner.begin();
    fc::L1Controller l1;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float specials[] = {0.0f, -0.0f, nan, inf, -inf, 1e30f, -1e30f, 1e-30f, 60.0f, -4.5f, 4.5f, 3.0f, 10.0f};
    constexpr int kSpecials = sizeof(specials) / sizeof(specials[0]);

    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> e_dist(-200.0f, 200.0f);
    std::uniform_real_distribution<float> v_dist(-30.0f, 30.0f);
    std::uniform_int_distribution<int> pick(0, 99);

    // Warm-up so the allocator has its free lists in steady state.
    for (int i = 0; i < 10000; ++i) {
        l1.xtrack_velocity_mps_ = v_dist(rng);
        tuner.update(l1, e_dist(rng), 0.005f);
    }
    const size_t heap_start = heapInUse();
    size_t heap_max = heap_start;

    long bad_period = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (long i = 0; i < kCycles; ++i) {
        float e = e_dist(rng);
        float v = v_dist(rng);
        const int r = pick(rng);
        if (r < 3) {
            e = specials[i % kSpecials];
        } else if (r < 6) {
            v = specials[(i / 7) % kSpecials];
        }
        l1.xtrack_velocity_mps_ = v;
        tuner.update(l1, e, 0.005f);
        const float p = l1.period();
        if (!std::isfinite(p) || p < 12.0f || p > 16.0f || !std::isfinite(tuner.lastDeltaPeriod())) {
            ++bad_period;
        }
        // A/B toggling and mode re-entry, as in flight.
        if (i % 200000 == 0) {
            tuner.applyEnabled(false, l1);
            tuner.update(l1, e, 0.005f);
            tuner.applyEnabled(true, l1);
            tuner.resetToBase(l1);
        }
        if (i % 100000 == 0) {
            const size_t h = heapInUse();
            if (h > heap_max) {
                heap_max = h;
            }
        }
    }
    const double us_per_update =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / kCycles;
    const size_t heap_end = heapInUse();

    std::printf("cycles=%ld (%.1f h at 200 Hz)  heap start=%zu end=%zu max=%zu  bad_period=%ld  %.2f us/update (host)\n",
                kCycles, kCycles / 200.0 / 3600.0, heap_start, heap_end, heap_max, bad_period, us_per_update);
    CHECK(bad_period == 0);
    CHECK(heap_end <= heap_start + 1024);  // no growth beyond allocator noise
    CHECK(heap_max <= heap_start + 1024);

    if (g_failures == 0) {
        std::printf("test_fuzzy_stress: ALL PASSED\n");
        return 0;
    }
    std::printf("test_fuzzy_stress: %d FAILED\n", g_failures);
    return 1;
}
