#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>

#include "navigation/GnssFixData.h"

namespace fc {

struct GnssUbxConfig {
    uint32_t i2c_clock_hz = 400000;
    uint8_t i2c_address = 0x42;
    uint16_t max_wait_ms = 250;
};

/**
 * UBX GNSS driver (u-blox M10), via the SparkFun u-blox Arduino library over
 * I2C. Polls UBX-NAV-PVT explicitly once per update() rather than enabling
 * the library's autoPVT background mode, to keep timing deterministic under
 * this codebase's polling scheduler.
 *
 * Scheduling stays outside this class, same as the other GNSS backends:
 * call begin() once, then update() from the flight scheduler.
 */
class GnssUbx final {
public:
    explicit GnssUbx(TwoWire& wire = Wire);
    GnssUbx(TwoWire& wire, const GnssUbxConfig& config);

    GnssUbx(const GnssUbx&) = delete;
    GnssUbx& operator=(const GnssUbx&) = delete;

    bool begin();

    /** Poll UBX-NAV-PVT; returns true if a new fix was read. */
    bool update();

    GnssFixData data() const;
    bool isFresh(uint32_t maximum_age_us) const;

private:
    static GnssFixType mapFixType(uint8_t ublox_fix_type);

    TwoWire& wire_;
    GnssUbxConfig config_{};
    SFE_UBLOX_GNSS ublox_{};

    GnssFixData data_{};
};

}  // namespace fc
