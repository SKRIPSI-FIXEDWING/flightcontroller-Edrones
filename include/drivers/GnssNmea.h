#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "navigation/GnssFixData.h"

namespace fc {

struct GnssNmeaConfig {
    uint32_t baud_rate = 38400;

    // Fix-quality heuristic thresholds, matching legacy gps.h::checkStatus().
    float fix_3d_max_hdop = 1.0f;
    uint8_t fix_3d_min_satellites = 6;
};

/**
 * NMEA GNSS driver (Radiolink SE100), via TinyGPSPlus over a hardware UART.
 *
 * Legacy `gps.h` used a bit-banged `SoftwareSerial` on pins 0/1, which are
 * Teensy 4.1's Serial1 hardware UART pins — this driver uses that hardware
 * UART directly instead (same physical pins, more reliable transport under
 * FreeRTOS preemption than a bit-banged port).
 *
 * Scheduling stays outside this class, same as fc::Imu/fc::Barometer: call
 * begin() once, then update() from the flight scheduler and distribute
 * data() to estimation/navigation/telemetry code.
 */
class GnssNmea final {
public:
    explicit GnssNmea(HardwareSerial& serial = Serial1);
    GnssNmea(HardwareSerial& serial, const GnssNmeaConfig& config);

    GnssNmea(const GnssNmea&) = delete;
    GnssNmea& operator=(const GnssNmea&) = delete;

    bool begin();

    /** Drain all available UART bytes; returns true if any NMEA sentence decoded. */
    bool update();

    GnssFixData data() const;
    bool isFresh(uint32_t maximum_age_us) const;

    // Migration helpers matching the values exposed by legacy gps.h::Gepees.
    float latitude() const;
    float longitude() const;
    float altitudeMeters() const;
    float hdop() const;
    uint8_t satellites() const;
    float speedMetersPerSecond() const;
    float courseDegrees() const;
    GnssFixType status() const;

private:
    void parseLocation();
    void parseSpeedAndCourse();
    void parseSatelliteInfo();
    void refreshFixType();

    HardwareSerial& serial_;
    GnssNmeaConfig config_{};
    TinyGPSPlus gps_{};

    GnssFixData data_{};
};

}  // namespace fc
