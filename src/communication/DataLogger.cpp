#include "communication/DataLogger.h"

#include <stdio.h>

namespace fc {

DataLogger::DataLogger(Stream& port)
    : port_(port)
{
}

void DataLogger::begin()
{
    port_.println("seq,timestamp_us,pressure_pa,temperature_c,altitude_m,raw_altitude_m,climb_rate_mps,"
                   "comp_altitude_m,comp_climb_rate_mps,comp_accel_up_mss");
}

void DataLogger::logBarometer(const BarometerData& baro, const AltitudeComplementaryData& complementary)
{
    if (!baro.valid) {
        return;
    }

    char line[200];
    snprintf(line, sizeof(line), "%lu,%lu,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f",
              static_cast<unsigned long>(baro.sequence),
              static_cast<unsigned long>(baro.timestamp_us),
              static_cast<double>(baro.pressure_pa),
              static_cast<double>(baro.temperature_c),
              static_cast<double>(baro.altitude_m),
              static_cast<double>(baro.raw_altitude_m),
              static_cast<double>(baro.climb_rate_mps),
              static_cast<double>(complementary.altitude_m),
              static_cast<double>(complementary.climb_rate_mps),
              static_cast<double>(complementary.last_accel_up_mss));
    port_.println(line);
}

}  // namespace fc
