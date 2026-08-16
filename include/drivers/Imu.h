#pragma once

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include <BNO055.h>
}

namespace fc {

/** A three-axis vector expressed in the unit stated by its field name. */
struct ImuVector3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * Normalized Hamilton quaternion (w, x, y, z), no unit conversion needed.
 * Polled separately via updateQuaternion(), not part of ImuData: kept for
 * logging/future AHRS use, and reading it costs an extra blocking I2C
 * transaction, so it must not add latency to the 200 Hz update() path that
 * roll_deg/pitch_deg/yaw_deg depend on. See docs/imu-bno055.md.
 */
struct ImuQuaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * One coherent BNO055 measurement.
 *
 * The field layout follows the information exposed by the legacy bno055.h,
 * while every name states its unit explicitly.
 */
struct ImuData {
    ImuVector3f acceleration_mss{};
    ImuVector3f linear_acceleration_mss{};
    ImuVector3f angular_rate_dps{};

    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float heading_deg = 0.0f;
    float yaw_deg = 0.0f;

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    float delta_roll_deg = 0.0f;
    float delta_pitch_deg = 0.0f;
    float delta_yaw_deg = 0.0f;

    uint32_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

struct ImuCalibration {
    uint8_t system = 0;
    uint8_t gyroscope = 0;
    uint8_t accelerometer = 0;
    uint8_t magnetometer = 0;
};

enum class ImuError : uint8_t {
    None = 0,
    NotInitialized,
    BusInUse,
    DeviceNotFound,
    ConfigurationFailed,
    ReadFailed,
};

struct ImuConfig {
    uint32_t i2c_clock_hz = 400000;
    uint16_t startup_delay_ms = 512;

    // Preserves the aircraft heading convention used by KHAGESWARA.
    float heading_offset_deg = -180.0f;

    // Bench-verified 2026-08-11 via MAVLink Inspector on this specific
    // airframe/mount: with the legacy default (true), pitch rate read
    // backwards (nose-up produced a negative rate). false is the confirmed-
    // correct value for this hardware -- see docs/imu-bno055.md.
    bool invert_gyro_y = false;

    // Same bench session: pitch_deg (from the BNO055 Euler register, see
    // Imu.cpp's readData()) also reads backwards in Mission Planner's HUD --
    // independent finding from invert_gyro_y above, since angle and rate
    // come from different BNO055 registers. true is confirmed-correct for
    // this hardware.
    bool invert_pitch = true;

    // Static bench-trim subtracted from roll_deg/pitch_deg, matching
    // fc-skripsi-main's roll_error/pitch_error. Determine by leveling the
    // airframe and recording the residual roll_deg/pitch_deg reading.
    float roll_trim_deg = 0.0f;
    float pitch_trim_deg = 0.0f;

    // Exponential moving average applied to yaw_deg only, matching
    // fc-skripsi-main's bno_euler.h (yaw = 0.95*last + 0.05*new, i.e.
    // alpha = 0.05). 0 disables filtering. Off by default here: this is
    // fixed-wing, and heavy smoothing adds yaw-control lag a multirotor
    // may tolerate better than a fixed-wing does.
    float yaw_filter_alpha = 0.0f;
};

/**
 * Fixed-wing BNO055 driver.
 *
 * Scheduling and locking intentionally remain outside this class. Call
 * update() from the flight scheduler, then distribute data() to control,
 * navigation, and telemetry code.
 *
 * The Bosch C API supports one active device, so only one initialized Imu
 * object may exist at a time.
 */
class Imu final {
public:
    explicit Imu(TwoWire& wire = Wire);
    Imu(TwoWire& wire, const ImuConfig& config);
    ~Imu();

    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;

    /** Initialize I2C, validate the chip ID, and enable NDOF fusion. */
    bool begin();

    /** Read and publish one complete sample without exposing partial data. */
    bool update();

    /** Read calibration status without blocking the flight loop. */
    bool updateCalibration();

    /**
     * Read the fusion quaternion. Call from a slow task (like
     * updateCalibration()), never from the 200 Hz IMU task: it is an extra
     * blocking I2C transaction on top of update()'s reads.
     */
    bool updateQuaternion();

    /** Return a copy of the last complete BNO055 measurement. */
    ImuData data() const;
    ImuCalibration calibration() const;
    ImuQuaternion quaternion() const;

    bool initialized() const;
    bool healthy() const;
    bool isFresh(uint32_t maximum_age_us) const;
    bool isFullyCalibrated() const;

    ImuError lastError() const;
    uint32_t consecutiveReadFailures() const;

    // Migration helpers matching the values exposed by legacy bno055.h.
    float rollDegrees() const;
    float pitchDegrees() const;
    float yawDegrees() const;
    float headingDegrees() const;
    ImuVector3f angularRateDps() const;
    float deltaRollDegrees() const;
    float deltaPitchDegrees() const;
    float deltaYawDegrees() const;

private:
    bool configureSensor();
    bool readData(ImuData& next);
    void setError(ImuError error);

    static int busRead(unsigned char device_address,
                       unsigned char register_address,
                       unsigned char* data,
                       unsigned char length);
    static int busWrite(unsigned char device_address,
                        unsigned char register_address,
                        unsigned char* data,
                        unsigned char length);
    static void delayMilliseconds(unsigned int duration_ms);

    static float degreesToRadians(float degrees);
    static float normalizePositiveDegrees(float angle_deg);
    static float normalizeSignedDegrees(float angle_deg);
    static float absoluteAngleDifferenceDegrees(float current_deg,
                                                float previous_deg);

    static Imu* active_instance_;

    TwoWire& wire_;
    ImuConfig config_{};
    bno055_t device_{};
    ImuData data_{};
    ImuCalibration calibration_{};
    ImuQuaternion quaternion_{};
    ImuError last_error_ = ImuError::NotInitialized;
    uint32_t consecutive_read_failures_ = 0;
    bool initialized_ = false;

    // State for the optional yaw_filter_alpha EMA.
    float last_yaw_deg_ = 0.0f;
    bool has_last_yaw_ = false;
};

}  // namespace fc
