// Host-side unit test for fc::DisturbanceScript and the DIST_ENABLE = 2 path of
// fc::DisturbanceInjector (pure logic, no Arduino).
// Build + run from the repo root (one line):
//   g++ -std=c++17 -Iinclude test/host/test_disturbance_script.cpp src/control/DisturbanceScript.cpp src/control/DisturbanceInjector.cpp -o test_dscript && ./test_dscript
#include <cstdio>
#include <cstdlib>

#include "FC_Config.h"
#include "control/DisturbanceInjector.h"
#include "control/DisturbanceScript.h"

static int g_failures = 0;
#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            std::printf("FAIL line %d: %s\n", __LINE__, #cond);                       \
            ++g_failures;                                                             \
        }                                                                             \
    } while (0)

using fc::DisturbanceScript;
using fc::DisturbanceScriptInput;

struct LegResult {
    float first_fire_along = -1.0f;  // along_m when the first pulse began, -1 = none
    int16_t first_offset = 0;
};

// One leg (flag_wp = flag): along_m runs from..to at 18 m/s sampled at 20 Hz, cte constant.
static LegResult runLeg(DisturbanceScript& s, int wp_sum, int flight, int flag, float from, float to, float cte,
                        uint32_t& now, float turn_angle = 0.0f)
{
    LegResult r;
    for (float along = from; along <= to; along += 18.0f * 0.05f) {
        now += 50;
        DisturbanceScriptInput in;
        in.flag_wp = flag;
        in.wp_sum = wp_sum;
        in.along_m = along;
        in.cte_m = cte;
        in.turn_angle_deg = turn_angle;
        in.now_ms = now;
        const int16_t off = s.update(in, flight, 2000);
        if (off != 0 && r.first_fire_along < 0.0f) {
            r.first_fire_along = along;
            r.first_offset = off;
        }
    }
    return r;
}

int main()
{
    // ---- Scenario 1, flight 1: straight at 90 m with A1 = +150, then the turn event (always away from the turn).
    {
        DisturbanceScript s;
        uint32_t now = 1000;
        runLeg(s, 4, 1, 1, 0, 200, 0.0f, now, 180.0f);  // approach leg is ignored
        LegResult l = runLeg(s, 4, 1, 2, 0, 140, 0.0f, now, 46.6f);
        CHECK(l.first_fire_along >= 90.0f && l.first_fire_along < 92.0f);
        CHECK(l.first_offset == 150);
        CHECK(s.eventId() == 1);
        CHECK(s.fireCount() == 1);
        l = runLeg(s, 4, 1, 2, 141, 290, 0.0f, now, 46.6f);  // pulse over; turn event waits
        CHECK(l.first_fire_along < 0.0f);
        // WP2 claimed (flag 3), calm before, right turn (+46.6) -> away from the turn = roll left = -150
        l = runLeg(s, 4, 1, 3, 0, 20, 8.0f, now, -20.0f);
        CHECK(l.first_offset == -150);
        CHECK(s.eventId() == 2);
        CHECK(s.fireCount() == 2);
        CHECK(s.skipCount() == 0);
    }
    // ---- Flight 2 (A2 = -150): straight gives -150; turn is always away: right turn -150, left turn +150.
    {
        DisturbanceScript s;
        uint32_t now = 1000;
        runLeg(s, 4, 2, 2, 0, 290, 0.0f, now, 46.6f);
        CHECK(s.eventId() == 1);
        LegResult l = runLeg(s, 4, 2, 3, 0, 20, 8.0f, now, 0.0f);
        CHECK(l.first_offset == -150);
        DisturbanceScript s2;
        now = 1000;
        runLeg(s2, 4, 2, 2, 0, 290, 0.0f, now, -46.6f);  // left turn ahead
        l = runLeg(s2, 4, 2, 3, 0, 20, 8.0f, now, 0.0f);
        CHECK(l.first_offset == 150);
    }
    // ---- Flight 3 (A3 = +300) and flight 4 (A4 = -300): straight is absolute, left turn always +300 (roll right).
    {
        DisturbanceScript s3;
        uint32_t now = 1000;
        LegResult l = runLeg(s3, 4, 3, 2, 0, 140, 0.0f, now, -46.6f);
        CHECK(l.first_offset == 300);
        runLeg(s3, 4, 3, 2, 141, 290, 0.0f, now, -46.6f);
        l = runLeg(s3, 4, 3, 3, 0, 20, 8.0f, now, 0.0f);
        CHECK(l.first_offset == 300);
        DisturbanceScript s4;
        now = 1000;
        l = runLeg(s4, 4, 4, 2, 0, 140, 0.0f, now, -46.6f);
        CHECK(l.first_offset == -300);
        runLeg(s4, 4, 4, 2, 141, 290, 0.0f, now, -46.6f);
        l = runLeg(s4, 4, 4, 3, 0, 20, 8.0f, now, 0.0f);
        CHECK(l.first_offset == 300);  // left turn: away = roll right, even though A4 is negative
    }
    // ---- Stale first sample after the WP claim (along ~285 m) must NOT skip the straight event.
    {
        DisturbanceScript s;
        uint32_t now = 1000;
        runLeg(s, 4, 1, 1, 0, 200, 0.0f, now, 180.0f);
        DisturbanceScriptInput in;
        in.flag_wp = 2;
        in.wp_sum = 4;
        in.along_m = 285.0f;  // the old bug: distance_next_wp still refers to the previous WP
        in.cte_m = 0.0f;
        in.turn_angle_deg = -46.6f;
        in.now_ms = now + 50;
        now += 50;
        s.update(in, 1, 2000);
        CHECK(s.skipCount() == 1);  // documents the script's behaviour for such a sample;
                                    // the mode must therefore never pass one (along from position)
    }
    // ---- Not calm (|e| = 12 m all along): straight skipped (reason 1), turn skipped too.
    {
        DisturbanceScript s;
        uint32_t now = 1000;
        LegResult l = runLeg(s, 4, 1, 2, 0, 290, 12.0f, now, 46.6f);
        CHECK(l.first_fire_along < 0.0f);
        CHECK(s.skipCount() == 1);
        CHECK(s.lastSkipId() == 1);
        CHECK(s.lastSkipReason() == 1);
        l = runLeg(s, 4, 1, 3, 0, 20, 8.0f, now, 0.0f);
        CHECK(l.first_fire_along < 0.0f);
        CHECK(s.skipCount() == 2);
        CHECK(s.lastSkipReason() == 1);
    }
    // ---- Abort: pulse running, |e| jumps past 45 m -> offset 0 and skip reason 3.
    {
        DisturbanceScript s;
        uint32_t now = 1000;
        runLeg(s, 4, 1, 2, 0, 91, 0.0f, now, 46.6f);
        CHECK(s.active());
        DisturbanceScriptInput in;
        in.flag_wp = 2;
        in.wp_sum = 4;
        in.along_m = 95.0f;
        in.cte_m = 50.0f;
        in.turn_angle_deg = 46.6f;
        in.now_ms = now + 50;
        CHECK(s.update(in, 1, 2000) == 0);
        CHECK(s.lastSkipReason() == 3);
        CHECK(!s.active());
    }
    // ---- Scenario 2, flight 3: L1 (A3 +300) at 70 m, B3 (A5 300, away from the turn) at flag 4, L5 (A4 -300) at 135 m on flag 6.
    {
        DisturbanceScript s;
        uint32_t now = 1000;
        runLeg(s, 7, 3, 1, 0, 100, 0.0f, now, 180.0f);
        LegResult l = runLeg(s, 7, 3, 2, 0, 300, 0.0f, now, 90.0f);
        CHECK(l.first_offset == 300);
        CHECK(s.eventId() == 1);
        runLeg(s, 7, 3, 3, 0, 190, 0.0f, now, 90.0f);  // leg 2 (200 m): nothing
        l = runLeg(s, 7, 3, 4, 0, 20, 8.0f, now, -90.0f);  // WP3 claimed, turn +90 (right) -> away = -300
        CHECK(l.first_offset == -300);
        CHECK(s.eventId() == 2);
        runLeg(s, 7, 3, 4, 21, 300, 0.0f, now, 0.0f);
        runLeg(s, 7, 3, 5, 0, 190, 0.0f, now, 90.0f);  // WP4 claimed, 200 m leg: nothing
        l = runLeg(s, 7, 3, 6, 0, 300, 0.0f, now, 0.0f);
        CHECK(l.first_offset == -300);
        CHECK(l.first_fire_along >= 135.0f && l.first_fire_along < 137.0f);
        CHECK(s.eventId() == 3);
        CHECK(s.skipCount() == 0);
    }
    // ---- Unknown mission size or flight number -> nothing.
    {
        DisturbanceScript s;
        uint32_t now = 1000;
        LegResult l = runLeg(s, 5, 1, 2, 0, 290, 0.0f, now, 46.6f);
        CHECK(l.first_fire_along < 0.0f);
        DisturbanceScript s2;
        l = runLeg(s2, 4, 6, 2, 0, 290, 0.0f, now, 46.6f);
        CHECK(l.first_fire_along < 0.0f);
    }
    // ---- Injector wrapper: DIST_ENABLE = 2 uses the script and ignores CH7; 1 ignores the script.
    {
        float enable = 2.0f, amp = 250.0f, dur = 2000.0f, flight = 1.0f;
        fc::DisturbanceInjector d(enable, amp, dur, flight);
        uint32_t now = 1000;
        int16_t last = 0;
        for (float along = 0; along <= 120.0f; along += 0.9f) {
            DisturbanceScriptInput in;
            in.flag_wp = 2;
            in.wp_sum = 4;
            in.along_m = along;
            in.cte_m = 0.0f;
            in.turn_angle_deg = 46.6f;
            now += 50;
            in.now_ms = now;
            last = d.update(true, true, true, 2000, now, &in);  // CH7 HIGH all the time: irrelevant
            if (last != 0) {
                break;
            }
        }
        CHECK(last == 150);
        CHECK(d.active());
        CHECK(d.script().eventId() == 1);
        CHECK(d.update(false, true, true, 2000, now + 50, nullptr) == 0);  // leaving AUTO stops it
        CHECK(!d.active());
        CHECK(d.script().eventId() == 0);
        CHECK(d.update(true, true, true, 2000, now + 100, nullptr) == 0);  // scripted, no inputs: nothing
        enable = 1.0f;
        DisturbanceScriptInput in;
        in.flag_wp = 2;
        in.wp_sum = 4;
        in.along_m = 100.0f;
        in.cte_m = 0.0f;
        in.now_ms = now + 150;
        CHECK(d.update(true, true, true, 1500, now + 150, &in) == 0);  // manual mode ignores the script
        enable = 0.0f;
        CHECK(d.update(true, true, true, 2000, now + 200, &in) == 0);
    }

    if (g_failures == 0) {
        std::printf("test_disturbance_script: ALL PASSED\n");
        return 0;
    }
    std::printf("test_disturbance_script: %d FAILED\n", g_failures);
    return 1;
}
