#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "drivers/Barometer.h"
#include "drivers/Imu.h"
#include "estimation/AltitudeComplementaryFilter.h"

namespace fc {

/**
 * Plain-text CSV logger for bench/ground data capture over USB Serial.
 * In FC_DEBUG_SERIAL_ENABLE builds, the primary `Serial` port is reserved for
 * PuTTY-readable text and MAVLink USB RX/TX is disabled in FC_Config.h --
 * see docs/data-logger-usb.md.
 */
class DataLogger final {
public:
    explicit DataLogger(Stream& port);

    /** Starts the USB CDC port and emits the CSV header row. */
    void begin();

    /**
     * Emits one CSV row with the full raw IMU sample (accel/gyro/mag/
     * linear-accel/gravity) alongside BNO055's on-chip roll/pitch/yaw,
     * calibration status, barometer altitude, and the complementary-filter
     * comparison estimate -- everything needed to diagnose axis-convention/
     * interference/calibration issues from a captured log without needing
     * the SD card. See docs/imu-bno055.md.
     *
     * `calibration` is a separate parameter (not part of ImuData) because
     * Imu::updateCalibration() is polled at a much slower rate (1 Hz, see
     * main.cpp) than update() -- pass whatever Imu::calibration() last
     * returned; it will repeat across several logged rows between polls,
     * that's expected.
     */
    void logAttitudeAltitude(const ImuData& imu, const ImuCalibration& calibration,
                             const BarometerData& baro, const AltitudeComplementaryData& complementary);

private:
    Stream& port_;
};

}  // namespace fc
