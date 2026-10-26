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

// Drives update() with the stick held at `stick` for `duration_ms` (1 ms steps)
// and returns the last offset.
static int run(DisturbanceInjector& d, uint32_t& now, uint16_t stick, uint32_t duration_ms,
               bool in_auto = true, bool armed = true, bool rc_ok = true)
{
    int16_t last = 0;
    for (uint32_t i = 0; i < duration_ms; ++i) {
        last = d.update(in_auto, armed, rc_ok, stick, ++now);
    }
    return last;
}

int main()
{
    float enable = 1.0f;
    DisturbanceInjector d(enable);
    uint32_t now = 1000;

    // 1. Disabled by default flag: a deflected stick never injects.
    enable = 0.0f;
    CHECK(run(d, now, 1500, 1000) == 0);
    CHECK(run(d, now, 2000, 500) == 0);
    enable = 1.0f;

    // 2. Stick deflected at AUTO entry does nothing until centered for the hold time.
    d.reset();
    CHECK(run(d, now, 2000, 1000) == 0);  // held off-center, never centered
    CHECK(run(d, now, 1500, fc::kDistCenterHoldMs - 50) == 0);
    CHECK(run(d, now, 2000, 5) == 0);     // not centered long enough yet -> still nothing
    CHECK(run(d, now, 1500, fc::kDistCenterHoldMs + 20) == 0);  // now Ready
    CHECK(run(d, now, 1500 + 200, 10) == fc::kDistLowUs);       // small deflection -> low level, right
    CHECK(run(d, now, 1500 - 200, 10) == -fc::kDistLowUs);      // other side while held
    CHECK(run(d, now, 1500 + 450, 10) == fc::kDistHighUs);      // big deflection -> high level
    CHECK(run(d, now, 1500 - 512, 10) == -fc::kDistHighUs);

    // 3. Deadband: just inside it is nothing, just outside is the low level.
    d.reset();
    run(d, now, 1500, fc::kDistCenterHoldMs + 20);
    CHECK(run(d, now, 1500 + fc::kDistDeadbandUs - 1, 5) == 0);
    CHECK(run(d, now, 1500 + fc::kDistDeadbandUs, 5) == fc::kDistLowUs);
    // Level split boundary.
    CHECK(run(d, now, 1500 + fc::kDistLevelSplitUs - 1, 5) == fc::kDistLowUs);
    CHECK(run(d, now, 1500 + fc::kDistLevelSplitUs, 5) == fc::kDistHighUs);

    // 4. Release ends it immediately.
    CHECK(run(d, now, 1500, 5) == 0);
    CHECK(!d.active());

    // 5. Max duration: switches off after kDistMaxDurationMs even if the stick is held,
    //    and does not come back until the stick has been centered again.
    d.reset();
    run(d, now, 1500, fc::kDistCenterHoldMs + 20);
    CHECK(run(d, now, 2000, fc::kDistMaxDurationMs - 100) == fc::kDistHighUs);
    CHECK(run(d, now, 2000, 200) == 0);        // timed out
    CHECK(run(d, now, 2000, 2000) == 0);       // still held -> stays off
    CHECK(run(d, now, 1500, fc::kDistCenterHoldMs + 20) == 0);  // centered again
    CHECK(run(d, now, 2000, 10) == fc::kDistHighUs);            // can trigger again

    // 6. Leaving AUTO / disarm / RC loss clears at once and requires a new centered period.
    d.reset();
    run(d, now, 1500, fc::kDistCenterHoldMs + 20);
    CHECK(run(d, now, 2000, 10) == fc::kDistHighUs);
    CHECK(run(d, now, 2000, 1, /*in_auto=*/false) == 0);
    CHECK(run(d, now, 2000, 50, true) == 0);   // back in AUTO, stick still deflected -> nothing
    run(d, now, 1500, fc::kDistCenterHoldMs + 20);
    CHECK(run(d, now, 2000, 10) == fc::kDistHighUs);
    CHECK(run(d, now, 2000, 1, true, /*armed=*/false) == 0);
    run(d, now, 1500, fc::kDistCenterHoldMs + 20);
    CHECK(run(d, now, 2000, 10) == fc::kDistHighUs);
    CHECK(run(d, now, 2000, 1, true, true, /*rc_ok=*/false) == 0);
    CHECK(run(d, now, 2000, 50) == 0);

    // 7. Turning the enable flag off while active stops it on the next call.
    d.reset();
    run(d, now, 1500, fc::kDistCenterHoldMs + 20);
    CHECK(run(d, now, 2000, 10) == fc::kDistHighUs);
    enable = 0.0f;
    CHECK(run(d, now, 2000, 1) == 0);
    enable = 1.0f;
    CHECK(run(d, now, 2000, 50) == 0);  // and does not resume by itself

    if (g_failures == 0) {
        std::printf("test_disturbance_injector: ALL PASSED\n");
        return 0;
    }
    std::printf("test_disturbance_injector: %d FAILED\n", g_failures);
    return 1;
}
