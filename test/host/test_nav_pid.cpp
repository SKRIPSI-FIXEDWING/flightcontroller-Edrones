// Host-side unit test for fc::NavPid (navigasi PID pembanding L1, 2026-11-07).
// Build + run from the repo root (one line):
//   g++ -std=c++17 -Iinclude test/host/test_nav_pid.cpp src/navigation/NavPid.cpp -o test_nav_pid && ./test_nav_pid
#include <cmath>
#include <cstdio>

#include "navigation/NavPid.h"

static int g_failures = 0;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL line %d: %s\n", __LINE__, msg);              \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)
#define CHECK_NEAR(actual, expected, tol)                                                          \
    do {                                                                                           \
        const double a_ = (actual);                                                                \
        if (!(std::fabs(a_ - (expected)) <= (tol))) {                                              \
            std::printf("FAIL line %d: %s = %.4f, expected %.4f\n", __LINE__, #actual, a_,         \
                        static_cast<double>(expected));                                            \
            ++g_failures;                                                                          \
        }                                                                                          \
    } while (0)

namespace {
constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979;

struct LineResult {
    double max_abs_heading_deg;  // |sudut terhadap garis| terbesar
    double final_abs_cte;
    double max_overshoot;        // galat terbesar di sisi seberang
};

// Titik massa terbang di sepanjang garis utara (cte = -east, + = kiri garis,
// sama dengan L1Controller). a_lat + = belok kanan. Bank dijepit 40 deg.
LineResult flyLine(fc::NavPid& pid, double cte0, double v, double t_end, bool use_limit)
{
    double east = -cte0, psi = 0.0;  // psi = heading dari utara (rad)
    const double dt = 0.02, a_max = kG * std::tan(40.0 * kPi / 180.0);
    LineResult r{0.0, 0.0, 0.0};
    for (double t = 0.0; t < t_end; t += dt) {
        const double cte = -east;
        const double cte_dot = -v * std::sin(psi);
        const float lim = use_limit ? pid.interceptErrorLimit(static_cast<float>(v), static_cast<float>(kPi / 4)) : 0.0f;
        double a = pid.update(static_cast<float>(cte), static_cast<float>(cte_dot), static_cast<float>(dt), lim);
        a = std::fmax(-a_max, std::fmin(a_max, a));
        psi += a / v * dt;
        east += v * std::sin(psi) * dt;
        r.max_abs_heading_deg = std::fmax(r.max_abs_heading_deg, std::fabs(psi) * 180.0 / kPi);
        if (cte * cte0 < 0.0) r.max_overshoot = std::fmax(r.max_overshoot, std::fabs(cte));
    }
    r.final_abs_cte = std::fabs(east);
    return r;
}
}  // namespace

int main()
{
    // --- default: MATI ---
    {
        fc::NavPid pid;
        CHECK(!pid.enabled(), "default NavPid harus mati (NAV_PID_ENABLE = 0)");
    }

    // --- gain default = setara L1 periode 14 s, damping 0.707 ---
    {
        const double w = 2.0 * kPi / 14.0;
        fc::NavPid pid;
        CHECK_NEAR(pid.config().kp, w * w, 1e-3);
        CHECK_NEAR(pid.config().kd, 2.0 * 0.707 * w, 1e-3);
        CHECK_NEAR(pid.config().ki, w * w / 20.0, 1e-3);
    }

    // --- tanda: wahana di kiri garis (e > 0) -> belok kanan (a > 0) ---
    {
        fc::NavPid pid;
        CHECK(pid.update(5.0f, 0.0f, 0.0f) > 0.0f, "e > 0 harus memberi a > 0");
        CHECK(pid.update(0.0f, 2.0f, 0.0f) > 0.0f, "de > 0 harus memberi a > 0");
        CHECK(pid.update(-5.0f, 0.0f, 0.0f) < 0.0f, "e < 0 harus memberi a < 0");
    }

    // --- integrator: Ki*e*dt, dijepit, berhenti di luar batas galat ---
    {
        fc::NavPid pid;
        pid.update(10.0f, 0.0f, 1.0f);
        CHECK_NEAR(pid.integratorMss(), 0.0101 * 10.0, 1e-4);
        for (int i = 0; i < 10000; ++i) pid.update(50.0f, 0.0f, 0.1f);
        CHECK_NEAR(pid.integratorMss(), 1.729, 1e-3);  // anti-windup = g tan 10 deg
        pid.resetIntegrator();
        pid.update(100.0f, 0.0f, 0.1f, 40.0f);
        CHECK_NEAR(pid.integratorMss(), 0.0, 1e-6);  // di luar batas: tidak integrasi
        pid.update(5.0f, 0.0f, 2.0f);
        CHECK_NEAR(pid.integratorMss(), 0.0, 1e-6);  // dt > 1 s: tidak integrasi
    }

    // --- batas galat: suku P dijepit ---
    {
        fc::NavPid pid;
        const float a = pid.update(500.0f, 0.0f, 0.0f, 40.0f);
        CHECK_NEAR(a, 0.2014 * 40.0, 1e-3);
        // GS 18 m/s, 45 deg -> Kd*18*0.7071/Kp ~ 40 m
        CHECK_NEAR(pid.interceptErrorLimit(18.0f, static_cast<float>(kPi / 4)), 40.1, 0.5);
    }

    // --- lingkar tertutup: galat awal 100 m di garis lurus, GS 18 m/s ---
    {
        fc::NavPid pid;
        const LineResult r = flyLine(pid, 100.0, 18.0, 90.0, /*use_limit=*/true);
        std::printf("garis, e0=100 m, dengan batas 45 deg: heading maks %.1f deg, overshoot %.1f m, |e| akhir %.2f m\n",
                    r.max_abs_heading_deg, r.max_overshoot, r.final_abs_cte);
        CHECK(r.max_abs_heading_deg < 60.0, "dengan batas sudut potong, PID tidak boleh belok jauh lewat 45 deg");
        CHECK(r.final_abs_cte < 1.0, "harus konvergen ke garis");
    }
    {
        fc::NavPid pid;
        const LineResult r = flyLine(pid, 100.0, 18.0, 90.0, /*use_limit=*/false);
        std::printf("garis, e0=100 m, TANPA batas: heading maks %.1f deg (alasan batas dipasang)\n",
                    r.max_abs_heading_deg);
    }
    {
        // galat kecil (seperti setelah gangguan): batas tidak aktif, respons linear
        fc::NavPid pid;
        const LineResult r = flyLine(pid, 10.0, 18.0, 60.0, /*use_limit=*/true);
        std::printf("garis, e0=10 m: heading maks %.1f deg, overshoot %.2f m, |e| akhir %.3f m\n",
                    r.max_abs_heading_deg, r.max_overshoot, r.final_abs_cte);
        CHECK(r.final_abs_cte < 0.5, "galat kecil harus konvergen");
        CHECK(r.max_overshoot < 3.0, "overshoot kecil (redaman ~0.7)");
    }

    if (g_failures == 0) {
        std::printf("test_nav_pid: SEMUA LULUS\n");
        return 0;
    }
    std::printf("test_nav_pid: %d GAGAL\n", g_failures);
    return 1;
}
