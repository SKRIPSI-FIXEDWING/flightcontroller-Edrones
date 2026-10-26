// Offline harness: same AltitudeFusion source as firmware, synthetic sensors.
#include "estimation/AltitudeFusion.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
#include <string>

struct Truth { std::vector<double> t, h, v, a, roll; };
static Truth loadTruth(const char* p) {
    Truth T; FILE* f = fopen(p, "r"); double a,b,c,d,e;
    while (fscanf(f, "%lf %lf %lf %lf %lf", &a,&b,&c,&d,&e) == 5) { T.t.push_back(a); T.h.push_back(b); T.v.push_back(c); T.a.push_back(d); T.roll.push_back(e);} fclose(f); return T;
}
static Truth synth(const std::string& kind) {
    Truth T; const double dt = 0.005; double h = 0, v = 0;
    for (int i = 0; i < 300 / dt; ++i) {
        double t = i * dt, a = 0;
        if (kind == "climb" || kind == "climblight") { if (t > 60 && t < 61) a = 3; if (t > 81 && t < 82) a = -3; }
        if (kind == "dive") { if (t > 60 && t < 62) a = -5; if (t > 66 && t < 68) a = 5; }
        T.t.push_back(t); T.h.push_back(h); T.v.push_back(v); T.a.push_back(a); T.roll.push_back(0);
        h += v * dt + 0.5 * a * dt * dt; v += a * dt;
    }
    return T;
}

int main(int argc, char** argv) {
    // usage: sim <scenario|truthfile> <accel_err_gain> <gps 0/1> <seed> [trace_out]
    std::string sc = argv[1]; double gain = atof(argv[2]); bool gps = atoi(argv[3]); int seed = atoi(argv[4]);
    const char* out = argc > 5 ? argv[5] : nullptr;
    Truth T = (sc.find(".txt") != std::string::npos) ? loadTruth(sc.c_str()) : synth(sc);
    std::mt19937 rng(seed); std::normal_distribution<double> N(0, 1);
    fc::AltitudeFusionConfig cfg;
    if (getenv("AN")) cfg.accel_noise_mss = atof(getenv("AN"));
    if (getenv("BW")) cfg.accel_bias_walk = atof(getenv("BW"));
    if (getenv("NG")) cfg.baro_no_gps_reset_s = atof(getenv("NG"));
    if (getenv("GS")) cfg.baro_gate_sigma = atof(getenv("GS"));
    fc::AltitudeFusion f(cfg);
    double gps_rw = 0, accel_rw = 0.05, se = 0, sb = 0, emax = 0, bmax = 0; int n = 0;
    FILE* fo = out ? fopen(out, "w") : nullptr;
    const double dt = 0.005; int delay = 40;  // GPS 0.2 s latency
    for (size_t i = 0; i < T.t.size(); ++i) {
        double t = T.t[i];
        // accel: truth + attitude-dependent error (sustained bank) + slow bias + noise
        accel_rw += 0.01 * sqrt(dt) * N(rng);
        double err = gain * sin(fabs(T.roll[i])) / sin(40 * M_PI / 180) + accel_rw + 0.3 * N(rng);
        double acc = T.a[i] + err;
        if (sc == "stuck" && t > 60 && t < 120) acc = -8.0;  // IMU data frozen (12-35-20 landing)
        f.predict((float)acc, (float)dt);
        // baro 25 Hz
        double dist = 0;
        if (sc == "light" || sc == "climblight") { double s = (t - 100) / 1.5; if (t > 100 && t < 140) dist = 20 * fmin(1.0, s); else if (t >= 140 && t < 141.5) dist = 20 * (1 - (t - 140) / 1.5); }
        double baro = T.h[i] + dist + 0.8 * N(rng);
        if (i % 8 == 0) f.updateBaro((float)baro);
        // GPS 5 Hz, MSL offset 50 m, slow wander
        gps_rw += -gps_rw * dt / 30.0 + 1.5 * sqrt(2 * dt / 30.0) * N(rng);
        if (gps && i % 40 == 0 && i >= (size_t)delay) {
            size_t j = i - delay; double g = T.h[j] + 50 + gps_rw + 0.7 * N(rng);
            if (sc == "gpsglitch" && t > 100 && t < 130) g += 20;
            f.updateGps((float)g, (float)(T.v[j] + 0.2 * N(rng)), true);
        }
        fc::AltitudeFusionData d = f.data();
        if (t > 20) { double e = d.altitude_m - T.h[i]; se += e * e; sb += (baro - T.h[i]) * (baro - T.h[i]); emax = fmax(emax, fabs(e)); bmax = fmax(bmax, fabs(d.baro_bias_m)); ++n; }
        if (fo && i % 40 == 0) fprintf(fo, "%.2f %.3f %.3f %.3f %.3f %d %d\n", t, T.h[i], baro, d.altitude_m, d.baro_bias_m, d.baro_rejected, d.gps_active);
    }
    if (fo) fclose(fo);
    printf("rms %6.2f max %6.2f |bias|max %6.1f  (baro rms %5.2f)\n", sqrt(se / n), emax, bmax, sqrt(sb / n));
}
