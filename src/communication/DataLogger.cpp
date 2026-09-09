#include "communication/DataLogger.h"

#include <stdio.h>

namespace fc {

DataLogger::DataLogger(Stream& port)
    : port_(port)
{
}

void DataLogger::begin()
{
    port_.println("imu_seq,baro_seq,timestamp_us,roll_deg,pitch_deg,yaw_deg,heading_deg,"
                  "altitude_m,raw_altitude_m,climb_rate_mps,pressure_pa,temperature_c,"
                  "comp_altitude_m,comp_climb_rate_mps,comp_accel_up_mss,"
                  "accel_x_mss,accel_y_mss,accel_z_mss,"
                  "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
                  "mag_x_ut,mag_y_ut,mag_z_ut,"
                  "linacc_x_mss,linacc_y_mss,linacc_z_mss,"
                  "grav_x_mss,grav_y_mss,grav_z_mss,"
                  "calib_sys,calib_gyro,calib_accel,calib_mag,"
                  "imu_valid,baro_valid");
}

void DataLogger::logAttitudeAltitude(const ImuData& imu, const ImuCalibration& calibration,
                                     const BarometerData& baro,
                                     const AltitudeComplementaryData& complementary)
{
    if (!imu.valid && !baro.valid) {
        return;
    }

    const uint32_t timestamp_us = imu.valid ? imu.timestamp_us : baro.timestamp_us;

    char line[600];
    snprintf(line, sizeof(line),
              "%lu,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.4f,"
              "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
              "%u,%u,%u,%u,%u,%u",
              static_cast<unsigned long>(imu.sequence),
              static_cast<unsigned long>(baro.sequence),
              static_cast<unsigned long>(timestamp_us),
              static_cast<double>(imu.roll_deg),
              static_cast<double>(imu.pitch_deg),
              static_cast<double>(imu.yaw_deg),
              static_cast<double>(imu.heading_deg),
              static_cast<double>(baro.altitude_m),
              static_cast<double>(baro.raw_altitude_m),
              static_cast<double>(baro.climb_rate_mps),
              static_cast<double>(baro.pressure_pa),
              static_cast<double>(baro.temperature_c),
              static_cast<double>(complementary.altitude_m),
              static_cast<double>(complementary.climb_rate_mps),
              static_cast<double>(complementary.last_accel_up_mss),
              static_cast<double>(imu.acceleration_mss.x),
              static_cast<double>(imu.acceleration_mss.y),
              static_cast<double>(imu.acceleration_mss.z),
              static_cast<double>(imu.angular_rate_dps.x),
              static_cast<double>(imu.angular_rate_dps.y),
              static_cast<double>(imu.angular_rate_dps.z),
              static_cast<double>(imu.magnetic_field_ut.x),
              static_cast<double>(imu.magnetic_field_ut.y),
              static_cast<double>(imu.magnetic_field_ut.z),
              static_cast<double>(imu.linear_acceleration_mss.x),
              static_cast<double>(imu.linear_acceleration_mss.y),
              static_cast<double>(imu.linear_acceleration_mss.z),
              static_cast<double>(imu.gravity_mss.x),
              static_cast<double>(imu.gravity_mss.y),
              static_cast<double>(imu.gravity_mss.z),
              static_cast<unsigned>(calibration.system),
              static_cast<unsigned>(calibration.gyroscope),
              static_cast<unsigned>(calibration.accelerometer),
              static_cast<unsigned>(calibration.magnetometer),
              imu.valid ? 1U : 0U,
              baro.valid ? 1U : 0U);
    port_.println(line);
}

}  // namespace fc
