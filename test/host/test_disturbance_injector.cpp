// Host-side unit test for fc::DisturbanceInjector (pure logic, no Arduino).
// Build + run from the repo root (one line):
//   g++ -std=c++17 -Iinclude test/host/test_disturbance_injector.cpp src/control/DisturbanceInjector.cpp -o test_dist && ./test_dist
#include <cstdio>
#include <cstdlib>

#include "FC_Config.h"
#include "control/DisturbanceInjector.h"

static int g_failures = 0;
#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            std::printf("FAIL line %d: %s\n", __LINE__, #cond);                       \
            ++g_failures;                                                             \
        }                                                                             \
    } while (0)

using fc::DisturbanceInjector;

constexpr uint16_t kLow = 1000;    // CH7 switch LOW
constexpr uint16_t kHigh = 2000;   // CH7 switch HIGH
constexpr uint16_t kFloat = 1500;  // unassigned channel floating mid-range

// Drives update() with CH7 held at `ch7` for `duration_ms` (1 ms steps) and
// returns the last offset.
static int run(DisturbanceInjector& d, uint32_t& now, uint16_t ch7, uint32_t duration_ms,
               bool mode_ok = true, bool armed = true, bool rc_ok = true)
{
    int16_t last = 0;
    for (uint32_t i = 0; i < duration_ms; ++i) {
        last = d.update(mode_ok, armed, rc_ok, ch7, ++now);
    }
    return last;
}

// Puts the injector in Ready (switch LOW long enough).
static void arm(DisturbanceInjector& d, uint32_t& now)
{
    run(d, now, kLow, fc::kDistSwitchLowHoldMs + 20);
}

int main()
{
    float enable = 1.0f;
    float amplitude = 250.0f;
    float duration = 2000.0f;
    DisturbanceInjector d(enable, amplitude, duration);
    uint32_t now = 1000;

    // 1. Disabled flag: flipping CH7 never injects.
    enable = 0.0f;
    CHECK(run(d, now, kLow, 1000) == 0);
    CHECK(run(d, now, kHigh, 500) == 0);
    enable = 1.0f;

    // 2. Switch already HIGH at mode entry does nothing until seen LOW for the hold time.
    d.reset();
    CHECK(run(d, now, kHigh, 1000) == 0);
    CHECK(run(d, now, kLow, fc::kDistSwitchLowHoldMs - 50) == 0);
    CHECK(run(d, now, kHigh, 5) == 0);  // not LOW long enough yet -> still nothing
    arm(d, now);
    CHECK(run(d, now, kHigh, 10) == 250);

    // 3. One pulse = exactly DIST_MS at DIST_US, independent of the switch afterwards.
    d.reset();
    arm(d, now);
    CHECK(run(d, now, kHigh, 1) == 250);
    CHECK(run(d, now, kLow, 1998) == 250);  // switched back LOW: pulse keeps going (t0+1998)
    CHECK(run(d, now, kLow, 1) == 250);     // t0+1999: last ms of the pulse
    CHECK(run(d, now, kLow, 1) == 0);       // 2000 ms after the start -> over
    CHECK(!d.active());

    // 4. Holding HIGH after a pulse does not re-trigger; needs LOW again.
    d.reset();
    arm(d, now);
    run(d, now, kHigh, 2000);
    CHECK(run(d, now, kHigh, 3000) == 0);
    arm(d, now);
    CHECK(run(d, now, kHigh, 10) == 250);

    // 5. A floating (unassigned) channel never arms nor triggers.
    d.reset();
    CHECK(run(d, now, kFloat, 2000) == 0);
    CHECK(run(d, now, kHigh, 100) == 0);
    // Mid-travel values are neither LOW nor HIGH.
    d.reset();
    CHECK(run(d, now, fc::kDistSwitchLowUs, 1000) == 0);  // exactly at the threshold is not LOW
    arm(d, now);
    CHECK(run(d, now, fc::kDistSwitchHighUs, 50) == 0);   // exactly at the threshold is not HIGH
    CHECK(run(d, now, fc::kDistSwitchHighUs + 1, 1) == 250);

    // 6. Amplitude sign and the hard clamps; params are latched at pulse start.
    d.reset();
    amplitude = -125.0f;
    arm(d, now);
    CHECK(run(d, now, kHigh, 1) == -125);
    amplitude = 300.0f;                     // changed mid-pulse: no effect on this pulse
    CHECK(run(d, now, kHigh, 10) == -125);
    d.reset();
    amplitude = 1000.0f;
    duration = 10000.0f;
    arm(d, now);
    CHECK(run(d, now, kHigh, 1) == fc::kDistMaxAmplitudeUs);
    CHECK(run(d, now, kHigh, fc::kDistMaxDurationMs - 1) == fc::kDistMaxAmplitudeUs);
    CHECK(run(d, now, kHigh, 1) == 0);      // clamped to kDistMaxDurationMs
    amplitude = 0.0f;                       // zero amplitude: nothing, ever
    duration = 2000.0f;
    d.reset();
    arm(d, now);
    CHECK(run(d, now, kHigh, 100) == 0);
    amplitude = 250.0f;

    // 7. Leaving the mode / disarm / RC loss clears at once and requires a new LOW period.
    d.reset();
    arm(d, now);
    CHECK(run(d, now, kHigh, 10) == 250);
    CHECK(run(d, now, kHigh, 1, /*mode_ok=*/false) == 0);
    CHECK(run(d, now, kHigh, 50) == 0);     // back in the mode, switch still HIGH -> nothing
    arm(d, now);
    CHECK(run(d, now, kHigh, 10) == 250);
    CHECK(run(d, now, kHigh, 1, true, /*armed=*/false) == 0);
    arm(d, now);
    CHECK(run(d, now, kHigh, 10) == 250);
    CHECK(run(d, now, kHigh, 1, true, true, /*rc_ok=*/false) == 0);
    CHECK(run(d, now, kHigh, 50) == 0);

    // 8. Turning the enable flag off while active stops it on the next call.
    d.reset();
    arm(d, now);
    CHECK(run(d, now, kHigh, 10) == 250);
    enable = 0.0f;
    CHECK(run(d, now, kHigh, 1) == 0);
    enable = 1.0f;
    CHECK(run(d, now, kHigh, 50) == 0);     // and does not resume by itself

    if (g_failures == 0) {
        std::printf("test_disturbance_injector: ALL PASSED\n");
        return 0;
    }
    std::printf("test_disturbance_injector: %d FAILED\n", g_failures);
    return 1;
}
