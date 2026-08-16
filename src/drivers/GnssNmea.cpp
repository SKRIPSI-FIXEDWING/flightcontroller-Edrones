#include "drivers/GnssNmea.h"

#include <math.h>

namespace fc {

GnssNmea::GnssNmea(HardwareSerial& serial)
    : serial_(serial)
{
}

GnssNmea::GnssNmea(HardwareSerial& serial, const GnssNmeaConfig& config)
    : serial_(serial), config_(config)
{
}

bool GnssNmea::begin()
{
    data_ = GnssFixData{};
    serial_.begin(config_.baud_rate);
    return true;
}

bool GnssNmea::update()
{
    bool decoded_any_sentence = false;

    while (serial_.available() > 0) {
        if (gps_.encode(serial_.read())) {
            parseLocation();
            parseSpeedAndCourse();
            parseSatelliteInfo();
            refreshFixType();
            decoded_any_sentence = true;
        }
    }

    if (decoded_any_sentence) {
        data_.timestamp_us = micros();
        data_.sequence += 1U;
        data_.valid = gps_.location.isValid();
    }

    return decoded_any_sentence;
}

void GnssNmea::parseLocation()
{
    if (gps_.location.isValid() || gps_.location.isUpdated()) {
        data_.latitude_deg = gps_.location.lat();
        data_.longitude_deg = gps_.location.lng();
    }

    if (gps_.altitude.isValid() || gps_.altitude.isUpdated()) {
        data_.altitude_m = static_cast<float>(gps_.altitude.meters());
    }
}

void GnssNmea::parseSpeedAndCourse()
{
    if (gps_.course.isValid() || gps_.course.isUpdated()) {
        data_.course_deg = static_cast<float>(gps_.course.deg());
    }

    if (gps_.speed.isValid() || gps_.speed.isUpdated()) {
        data_.ground_speed_mps = static_cast<float>(gps_.speed.mps());
        data_.velocity_ned_mps[0] = data_.ground_speed_mps * cosf(radians(data_.course_deg));
        data_.velocity_ned_mps[1] = data_.ground_speed_mps * sinf(radians(data_.course_deg));
        // velocity_ned_mps[2] (down) is synthesized at the AHRS layer from
        // the barometer's climb rate, not duplicated here.
    }
}

void GnssNmea::parseSatelliteInfo()
{
    if (gps_.hdop.isValid() || gps_.hdop.isUpdated()) {
        data_.hdop = static_cast<float>(gps_.hdop.hdop());
    }

    if (gps_.satellites.isValid() || gps_.satellites.isUpdated()) {
        data_.satellites = static_cast<uint8_t>(gps_.satellites.value());
    }
}

void GnssNmea::refreshFixType()
{
    const bool good_hdop = data_.hdop > 0.0f && data_.hdop < config_.fix_3d_max_hdop;
    const bool enough_satellites = data_.satellites > config_.fix_3d_min_satellites;

    if (good_hdop && enough_satellites) {
        data_.fix_type = GnssFixType::Fix3D;
    } else if (good_hdop || enough_satellites) {
        data_.fix_type = GnssFixType::Fix2D;
    } else {
        data_.fix_type = GnssFixType::NoFix;
    }
}

GnssFixData GnssNmea::data() const
{
    return data_;
}

bool GnssNmea::isFresh(uint32_t maximum_age_us) const
{
    return data_.valid &&
           static_cast<uint32_t>(micros() - data_.timestamp_us) <= maximum_age_us;
}

float GnssNmea::latitude() const
{
    return static_cast<float>(data_.latitude_deg);
}

float GnssNmea::longitude() const
{
    return static_cast<float>(data_.longitude_deg);
}

float GnssNmea::altitudeMeters() const
{
    return data_.altitude_m;
}

float GnssNmea::hdop() const
{
    return data_.hdop;
}

uint8_t GnssNmea::satellites() const
{
    return data_.satellites;
}

float GnssNmea::speedMetersPerSecond() const
{
    return data_.ground_speed_mps;
}

float GnssNmea::courseDegrees() const
{
    return data_.course_deg;
}

GnssFixType GnssNmea::status() const
{
    return data_.fix_type;
}

}  // namespace fc
