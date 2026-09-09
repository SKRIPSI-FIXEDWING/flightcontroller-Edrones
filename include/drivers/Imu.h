#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "FC_Config.h"

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
    // BNO055's on-chip gravity-vector output (register 0x2E-0x33, fused from
    // accel+gyro+mag same as linear_acceleration_mss, just the complementary
    // half: acceleration_mss ~= linear_acceleration_mss + gravity_mss).
    // Logged mainly as a diagnostic cross-check on the on-chip fusion's own
    // internal consistency -- see tools/flight_analysis IMU log analysis.
    ImuVector3f gravity_mss{};
    ImuVector3f angular_rate_dps{};

    // Raw magnetometer, uncorrected chip axes/sign -- BNO055's raw data
    // registers are always in the factory axis frame regardless of any
    // fusion-output remap, unlike roll_deg/pitch_deg below (which come from
    // the Euler registers, already bench-corrected via invert_pitch). Not
    // used by AHRS/control; added for fc::AttitudeMahonyFilter's MARG input.
    // See docs/attitude-mahony-filter.md.
    ImuVector3f magnetic_field_ut{};

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

/**
 * BNO055's raw accel/mag/gyro offset registers (Bosch datasheet 0x55-0x66).
 * The chip has no non-volatile storage of its own: every power cycle it
 * re-runs calibration from scratch, converging differently depending on
 * incidental motion right after boot -- see docs/imu-bno055.md for why this
 * caused inconsistent attitude offsets between test runs. Read via
 * Imu::readCalibrationOffsets() once isFullyCalibrated() is true, persist
 * with storage/ImuCalibrationStorage.h, and restore with
 * Imu::writeCalibrationOffsets() at boot.
 */
struct ImuCalibrationOffsets {
    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    int16_t mag_x = 0;
    int16_t mag_y = 0;
    int16_t mag_z = 0;
    int16_t gyro_x = 0;
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;
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
    // FC_BNO055_I2C_CLOCK_HZ (FC_Config.h) -- drop to 100000 to test whether
    // sign-flipping magnetometer noise is an I2C signal-integrity issue.
    uint32_t i2c_clock_hz = FC_BNO055_I2C_CLOCK_HZ;
    uint16_t startup_delay_ms = 512;

    // NDOF_FMC_OFF keeps the magnetometer involved in heading estimation but
    // disables Bosch's fast magnetometer-calibration state changes. Those
    // state changes can make bench attitude jump while the airframe is being
    // yawed near magnetic interference. Set true only after clean flight-log
    // validation if the faster calibration behaviour is specifically needed.
    bool enable_fast_magnetometer_calibration = false;

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

    // Wrap-safe circular exponential moving average applied to yaw_deg only.
    // 0 disables filtering; 0.10 removes bench jitter without the +/-180 deg
    // discontinuity produced by a normal scalar EMA.
    float yaw_filter_alpha = 0.10f;
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

    /**
     * Read the chip's current accel/mag/gyro offset registers. Call from a
     * slow task once isFullyCalibrated() is true -- never from the 200 Hz
     * IMU task.
     */
    bool readCalibrationOffsets(ImuCalibrationOffsets& out) const;

    /**
     * Restore previously-saved offset registers. Switches to CONFIG mode and
     * back to NDOF, so fusion output is briefly stale/invalid while this
     * runs -- call only while disarmed, before the flight loop starts
     * relying on fresh attitude data (e.g. right after begin() in setup()).
     */
    bool writeCalibrationOffsets(const ImuCalibrationOffsets& offsets);

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

    // Stable references into config_ for Params to register directly (mirrors
    // the float* pattern every other Params entry uses), and for the MAVLink
    // "Calibrate Level" handler (MAV_CMD_PREFLIGHT_CALIBRATION param5=2) to
    // update in place.
    float& rollTrimDegRef();
    float& pitchTrimDegRef();

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
