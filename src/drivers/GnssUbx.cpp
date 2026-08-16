#include "drivers/GnssUbx.h"

namespace fc {

GnssUbx::GnssUbx(TwoWire& wire)
    : wire_(wire)
{
}

GnssUbx::GnssUbx(TwoWire& wire, const GnssUbxConfig& config)
    : wire_(wire), config_(config)
{
}

bool GnssUbx::begin()
{
    data_ = GnssFixData{};

    wire_.begin();
    wire_.setClock(config_.i2c_clock_hz);

    if (!ublox_.begin(wire_, config_.i2c_address, config_.max_wait_ms)) {
        return false;
    }

    ublox_.setI2COutput(COM_TYPE_UBX);
    return true;
}

bool GnssUbx::update()
{
    if (!ublox_.getPVT(config_.max_wait_ms)) {
        return false;
    }

    const uint8_t fix_type = ublox_.getFixType();

    data_.fix_type = mapFixType(fix_type);
    data_.latitude_deg = static_cast<double>(ublox_.getLatitude()) * 1e-7;
    data_.longitude_deg = static_cast<double>(ublox_.getLongitude()) * 1e-7;
    data_.altitude_m = static_cast<float>(ublox_.getAltitudeMSL()) / 1000.0f;
    data_.hdop = static_cast<float>(ublox_.getPDOP()) / 100.0f;
    data_.satellites = ublox_.getSIV();
    data_.ground_speed_mps = static_cast<float>(ublox_.getGroundSpeed()) / 1000.0f;
    data_.course_deg = static_cast<float>(ublox_.getHeading()) * 1e-5f;
    data_.velocity_ned_mps[0] = static_cast<float>(ublox_.getNedNorthVel()) / 1000.0f;
    data_.velocity_ned_mps[1] = static_cast<float>(ublox_.getNedEastVel()) / 1000.0f;
    data_.velocity_ned_mps[2] = static_cast<float>(ublox_.getNedDownVel()) / 1000.0f;
    data_.timestamp_us = micros();
    data_.sequence += 1U;
    data_.valid = fix_type >= 2U;  // 2 = 2D, 3 = 3D, 4 = GNSS+dead-reckoning

    return true;
}

GnssFixData GnssUbx::data() const
{
    return data_;
}

bool GnssUbx::isFresh(uint32_t maximum_age_us) const
{
    return data_.valid &&
           static_cast<uint32_t>(micros() - data_.timestamp_us) <= maximum_age_us;
}

GnssFixType GnssUbx::mapFixType(uint8_t ublox_fix_type)
{
    switch (ublox_fix_type) {
        case 3:  // 3D
        case 4:  // GNSS + dead reckoning combined
            return GnssFixType::Fix3D;
        case 2:  // 2D
            return GnssFixType::Fix2D;
        default:  // 0 = no fix, 1 = dead reckoning only, 5 = time only
            return GnssFixType::NoFix;
    }
}

}  // namespace fc
