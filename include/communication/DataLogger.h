#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "drivers/Barometer.h"
#include "estimation/AltitudeComplementaryFilter.h"

namespace fc {

/**
 * Plain-text CSV logger for bench/ground data capture over a dedicated USB
 * CDC serial port (SerialUSB1, requires board_build.usb_type =
 * USB_DUAL_SERIAL -- see platformio.ini). Deliberately NOT sharing the
 * primary `Serial` port: that one carries the binary MAVLink stream to the
 * GCS (fc::Mavlink), and interleaving text on it would corrupt MAVLink
 * framing. See docs/data-logger-usb.md.
 */
class DataLogger final {
public:
    explicit DataLogger(Stream& port);

    /** Starts the USB CDC port and emits the CSV header row. */
    void begin();

    /**
     * Emits one CSV row combining the Kalman-filtered barometer estimate
     * (Option 1) and the complementary-filter baro+accel estimate (Option 2,
     * see docs/altitude-complementary-filter.md), for direct side-by-side
     * comparison. Skips the row if the barometer sample is invalid;
     * `complementary` may still be !valid() early after boot (its first
     * update hasn't run yet) -- its zeroed default is logged in that case
     * rather than skipping, so the CSV column count stays constant.
     */
    void logBarometer(const BarometerData& baro, const AltitudeComplementaryData& complementary);

private:
    Stream& port_;
};

}  // namespace fc
