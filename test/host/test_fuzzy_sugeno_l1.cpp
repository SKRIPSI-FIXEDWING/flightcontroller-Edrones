// Host-side unit test for fc::fuzzySugenoL1 / fc::FuzzySugenoL1 (pure logic, no Arduino).
// Build + run from the repo root (one line):
//   g++ -std=c++17 -Iinclude test/host/test_fuzzy_sugeno_l1.cpp src/navigation/FuzzySugenoL1.cpp -o test_fuzzy && ./test_fuzzy
//
// Selain contoh hitung tangan, uji ini memutar ulang seluruh sampel terbang
// 30/10/2026 dan membandingkannya dengan desain Python
// (tools/flight_analysis/fuzzy_sugeno_2026-10-30/putar_ulang_periode.csv).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "navigation/FuzzySugenoL1.h"

static int g_failures = 0;
#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            std::printf("FAIL line %d: %s\n", __LINE__, #cond);                       \
            ++g_failures;                                                             \
        }                                                                             \
    } while (0)

static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

using fc::FuzzyL1Output;
using fc::FuzzySugenoL1;
using fc::fuzzySugenoL1;

// Putar ulang semua sampel dari desain Python; kembalikan selisih periode maksimum.
static float replayPythonDesign(int& rows_out)
{
    const char* path = "tools/flight_analysis/fuzzy_sugeno_2026-10-30/putar_ulang_periode.csv";
    FILE* f = std::fopen(path, "r");
    rows_out = 0;
    if (f == nullptr) {
        std::printf("SKIP replay: %s tidak ditemukan (jalankan dari root repo)\n", path);
        return 0.0f;
    }
    char line[512];
    std::fgets(line, sizeof(line), f);  // judul kolom
    float worst = 0.0f;
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        // period_s,leg,t_leg_s,e_m,de_mps,e_fn,de_fn,z,periode_sugeno_s
        float cols[9] = {};
        int k = 0;
        for (char* tok = std::strtok(line, ","); tok != nullptr && k < 9; tok = std::strtok(nullptr, ","), ++k) {
            cols[k] = (k == 1) ? 0.0f : static_cast<float>(std::atof(tok));
        }
        const FuzzyL1Output out = fuzzySugenoL1(std::fabs(cols[3]), cols[4]);
        worst = std::fmax(worst, std::fabs(out.period_s - cols[8]));
        ++rows_out;
    }
    std::fclose(f);
    return worst;
}

int main()
{
    // 1. Contoh A (DESAIN bagian 6): tunak dekat garis -> 11 s.
    {
        const FuzzyL1Output o = fuzzySugenoL1(1.97f, -0.49f);
        CHECK(near(o.mu_e[0], 1.0f, 1e-4f));
        CHECK(near(o.alpha[0][1], 0.925f, 2e-3f));
        CHECK(near(o.period_s, 11.0f, 1e-3f));
    }

    // 2. Contoh B: error sedang kembali ke garis -> 15,49 s, sum_alpha 1,436.
    {
        const FuzzyL1Output o = fuzzySugenoL1(11.75f, -5.08f);
        CHECK(near(o.e_fn, 0.5875f, 1e-4f));
        CHECK(near(o.mu_e[1], 0.650f, 1e-3f));
        CHECK(near(o.mu_e[2], 0.350f, 1e-3f));
        CHECK(near(o.mu_de[0], 0.782f, 1e-3f));
        CHECK(near(o.sum_alpha, 1.436f, 2e-3f));
        CHECK(near(o.z, 0.1218f, 1e-3f));
        CHECK(near(o.period_s, 15.49f, 5e-3f));
    }

    // 3. Contoh C: awal leg, error besar mendekat cepat -> 17 s (masukan dipotong).
    {
        const FuzzyL1Output o = fuzzySugenoL1(31.01f, -17.83f);
        CHECK(near(o.e_fn, 1.0f, 1e-6f));
        CHECK(near(o.de_fn, -1.0f, 1e-6f));
        CHECK(near(o.period_s, 17.0f, 1e-4f));
    }

    // 4. Seluruh semesta: mu tiap variabel berjumlah 1, sum_alpha > 0,
    //    periode selalu di 11..17 s (rentang konstanta rule base).
    for (float e = 0.0f; e <= 50.0f; e += 0.25f) {
        for (float de = -25.0f; de <= 25.0f; de += 0.25f) {
            const FuzzyL1Output o = fuzzySugenoL1(e, de);
            CHECK(near(o.mu_e[0] + o.mu_e[1] + o.mu_e[2], 1.0f, 1e-5f));
            CHECK(near(o.mu_de[0] + o.mu_de[1] + o.mu_de[2], 1.0f, 1e-5f));
            CHECK(o.sum_alpha > 0.0f);
            CHECK(o.period_s >= 11.0f - 1e-4f && o.period_s <= 17.0f + 1e-4f);
        }
    }

    // 5. Saklar: FUZZY_ENABLE = 0 atau allowed = false -> periode tetap NAVL1_PERIOD.
    {
        float enable = 0.0f;
        float fixed_period = 15.0f;
        FuzzySugenoL1 fuzzy(enable, fixed_period);
        CHECK(near(fuzzy.update(true, 30.0f, -15.0f), 15.0f, 1e-6f));
        CHECK(!fuzzy.active());

        enable = 1.0f;  // berlaku langsung tanpa membuat objek baru
        CHECK(near(fuzzy.update(true, 30.0f, -15.0f), 17.0f, 1e-4f));
        CHECK(fuzzy.active());

        CHECK(near(fuzzy.update(false, 30.0f, -15.0f), 15.0f, 1e-6f));  // misi selesai / keluar AUTO
        CHECK(!fuzzy.active());

        fixed_period = 13.0f;  // NAVL1_PERIOD diubah dari GCS
        enable = 0.0f;
        CHECK(near(fuzzy.update(true, 0.0f, 0.0f), 13.0f, 1e-6f));
    }

    // 6. Tanda d|e|/dt: pesawat di KIRI garis (cte < 0) bergerak ke kanan
    //    (d(cte)/dt > 0) = mendekati garis, harus sama dengan sisi kanan yang mendekat.
    {
        float enable = 1.0f, fixed_period = 15.0f;
        FuzzySugenoL1 fuzzy(enable, fixed_period);
        const float left = fuzzy.update(true, -20.0f, +10.0f);
        const float right = fuzzy.update(true, +20.0f, -10.0f);
        CHECK(near(left, right, 1e-6f));
        CHECK(near(left, 17.0f, 1e-4f));  // Besar & Negatif
        CHECK(fuzzy.last().de_fn < 0.0f);
    }

    // 7. Sama dengan desain Python untuk seluruh sampel terbang 30/10/2026.
    int rows = 0;
    const float worst = replayPythonDesign(rows);
    if (rows > 0) {
        std::printf("replay %d sampel: selisih periode maksimum %.5f s\n", rows, static_cast<double>(worst));
        CHECK(rows == 1894);
        CHECK(worst < 2e-3f);  // CSV dibulatkan ke 0,001 s
    }

    if (g_failures == 0) {
        std::printf("OK: semua uji fuzzy Sugeno L1 lulus\n");
        return 0;
    }
    std::printf("%d uji GAGAL\n", g_failures);
    return 1;
}
