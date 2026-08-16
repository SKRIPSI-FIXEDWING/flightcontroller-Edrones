#pragma once

#include <stdint.h>

namespace fc {

enum class GnssFixType : uint8_t {
    NoFix = 0,
    Fix2D = 1,
    Fix3D = 2,
};

/**
 * Common position/velocity/status snapshot every GNSS backend (NMEA/SE100,
 * UBX/M10, DroneCAN/Here4) normalizes into, mirroring how ImuData is the one
 * shared IMU type multiple consumers read regardless of which physical
 * driver produced it.
 *
 * velocity_ned_mps[2] (down/vertical) is populated directly by backends
 * that receive a genuine GNSS-derived vertical velocity (UBX's NED
 * velocity solution, DroneCAN's Fix2 ned_velocity) and left at 0 by the
 * NMEA backend, which has no vertical-velocity sentence to parse. Callers
 * that need vertical velocity when the active backend is NMEA should use
 * the AHRS layer's synthesized estimate (from the barometer's climb rate)
 * instead — the legacy `gps.h` computed this locally by reaching into a
 * global `Barometer baro`, which coupled the GPS driver to the barometer
 * driver; this refactor removes that coupling from the GNSS drivers
 * themselves.
 */
struct GnssFixData {
    GnssFixType fix_type = GnssFixType::NoFix;

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    float altitude_m = 0.0f;

    // Horizontal dilution of precision where the backend reports one
    // (NMEA); the closest dilution-of-precision metric it exposes
    // otherwise (e.g. UBX backends report PDOP here) — see the owning
    // backend's docs/gnss-*.md for exactly which quantity this is.
    float hdop = 0.0f;
    uint8_t satellites = 0;

    float ground_speed_mps = 0.0f;
    float course_deg = 0.0f;

    /** North, East, Down, in m/s. */
    float velocity_ned_mps[3] = {0.0f, 0.0f, 0.0f};

    uint32_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

}  // namespace fc
