#include "drivers/Imu.h"

#include <math.h>

#include "FC_Config.h"

namespace {

constexpr uint8_t kExpectedChipId = 0xA0;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadiansPerDegree = kPi / 180.0f;

constexpr float kAccelerationCountsPerMss = 100.0f;
constexpr float kGyroscopeCountsPerDps = 16.0f;
constexpr float kEulerCountsPerDegree = 16.0f;
constexpr float kMagnetometerCountsPerMicroTesla = 16.0f;  // fixed, no unit-select bit (Bosch datasheet)
// BNO055 quaternion registers report Q14 fixed point (2^14 LSB per unit).
constexpr float kQuaternionScale = 1.0f / 16384.0f;

// Accel(0x08-0x0D) + Mag(0x0E-0x13) + Gyro(0x14-0x19) + Euler(0x1A-0x1F) are
// contiguous in the BNO055 register map, so one burst read replaces what
// would otherwise be 4 separate I2C transactions -- see docs/imu-bno055.md
// for why extra per-cycle I2C transactions on the 200 Hz IMU task are
// worth avoiding (this is what added magnetometer without repeating that
// regression).
constexpr uint8_t kBurstRegisterStart = 0x08;
constexpr uint8_t kBurstRegisterLength = 24;

// LinearAccel(0x28-0x2D) + Gravity(0x2E-0x33) are contiguous too -- same
// one-transaction-instead-of-two reasoning as the burst above.
constexpr uint8_t kLinearAccelGravityRegisterStart = 0x28;
constexpr uint8_t kLinearAccelGravityRegisterLength = 12;

int16_t readLittleEndian16(const uint8_t* buffer, size_t offset)
{
    return static_cast<int16_t>(static_cast<uint16_t>(buffer[offset]) |
                                (static_cast<uint16_t>(buffer[offset + 1]) << 8));
}

bool succeeded(int status)
{
    return status == SUCCESS;
}

}  // namespace

namespace fc {

Imu* Imu::active_instance_ = nullptr;

Imu::Imu(TwoWire& wire)
    : wire_(wire)
{
}

Imu::Imu(TwoWire& wire, const ImuConfig& config)
    : wire_(wire), config_(config)
{
}

Imu::~Imu()
{
    if (active_instance_ == this) {
        active_instance_ = nullptr;
    }
}

bool Imu::begin()
{
    if (active_instance_ != nullptr && active_instance_ != this) {
        setError(ImuError::BusInUse);
        return false;
    }

    active_instance_ = this;
    initialized_ = false;
    data_ = ImuData{};
    calibration_ = ImuCalibration{};
    quaternion_ = ImuQuaternion{};
    consecutive_read_failures_ = 0;
    has_last_yaw_ = false;

    wire_.begin();
    wire_.setClock(config_.i2c_clock_hz);

    device_ = bno055_t{};
    device_.bus_read = &Imu::busRead;
    device_.bus_write = &Imu::busWrite;
    device_.delay_msec = &Imu::delayMilliseconds;

    if (!succeeded(bno055_init(&device_)) ||
        device_.chip_id != kExpectedChipId) {
        setError(ImuError::DeviceNotFound);
        active_instance_ = nullptr;
        return false;
    }

    if (!configureSensor()) {
        setError(ImuError::ConfigurationFailed);
        active_instance_ = nullptr;
        return false;
    }

    initialized_ = true;
    setError(ImuError::None);
    return true;
}

bool Imu::configureSensor()
{
    if (!succeeded(bno055_set_operation_mode(OPERATION_MODE_CONFIG))) {
        return false;
    }

    // Explicitly select m/s^2, degrees/s, degrees, and Windows orientation.
    const unsigned char fusion_mode = config_.enable_fast_magnetometer_calibration
                                          ? OPERATION_MODE_NDOF
                                          : OPERATION_MODE_NDOF_FMC_OFF;

    if (!succeeded(bno055_set_accel_unit(0)) ||
        !succeeded(bno055_set_gyro_unit(0)) ||
        !succeeded(bno055_set_euler_unit(0)) ||
        !succeeded(bno055_set_data_output_format(0)) ||
        !succeeded(bno055_set_axis_remap_value(FC_BNO055_AXIS_MAP_CONFIG)) ||
        !succeeded(bno055_set_x_remap_sign(FC_BNO055_AXIS_SIGN_X)) ||
        !succeeded(bno055_set_y_remap_sign(FC_BNO055_AXIS_SIGN_Y)) ||
        !succeeded(bno055_set_z_remap_sign(FC_BNO055_AXIS_SIGN_Z)) ||
        !succeeded(bno055_set_powermode(POWER_MODE_NORMAL)) ||
        !succeeded(bno055_set_operation_mode(fusion_mode))) {
        return false;
    }

    delay(config_.startup_delay_ms);
    return true;
}

bool Imu::update()
{
    if (!initialized_) {
        setError(ImuError::NotInitialized);
        return false;
    }

    ImuData next = data_;
    if (!readData(next)) {
        ++consecutive_read_failures_;
        setError(ImuError::ReadFailed);
        return false;
    }

    if (data_.valid) {
        next.delta_roll_deg = absoluteAngleDifferenceDegrees(
            next.roll_deg, data_.roll_deg);
        next.delta_pitch_deg = absoluteAngleDifferenceDegrees(
            next.pitch_deg, data_.pitch_deg);
        next.delta_yaw_deg = absoluteAngleDifferenceDegrees(
            next.yaw_deg, data_.yaw_deg);
    } else {
        next.delta_roll_deg = 0.0f;
        next.delta_pitch_deg = 0.0f;
        next.delta_yaw_deg = 0.0f;
    }

    next.timestamp_us = micros();
    next.sequence = data_.sequence + 1U;
    next.valid = true;

    // Publish only after every sensor read and conversion has succeeded.
    data_ = next;
    consecutive_read_failures_ = 0;
    setError(ImuError::None);
    return true;
}

bool Imu::readData(ImuData& next)
{
    uint8_t burst[kBurstRegisterLength] = {};
    uint8_t linear_accel_gravity_burst[kLinearAccelGravityRegisterLength] = {};

    if (busRead(device_.dev_addr, kBurstRegisterStart, burst, kBurstRegisterLength) != SUCCESS ||
        busRead(device_.dev_addr, kLinearAccelGravityRegisterStart, linear_accel_gravity_burst,
               kLinearAccelGravityRegisterLength) != SUCCESS) {
        return false;
    }

    const int16_t accel_x = readLittleEndian16(burst, 0);
    const int16_t accel_y = readLittleEndian16(burst, 2);
    const int16_t accel_z = readLittleEndian16(burst, 4);
    const int16_t mag_x = readLittleEndian16(burst, 6);
    const int16_t mag_y = readLittleEndian16(burst, 8);
    const int16_t mag_z = readLittleEndian16(burst, 10);
    const int16_t gyro_x = readLittleEndian16(burst, 12);
    const int16_t gyro_y = readLittleEndian16(burst, 14);
    const int16_t gyro_z = readLittleEndian16(burst, 16);
    const int16_t euler_h = readLittleEndian16(burst, 18);
    const int16_t euler_r = readLittleEndian16(burst, 20);
    const int16_t euler_p = readLittleEndian16(burst, 22);

    next.roll_deg =
        static_cast<float>(euler_r) / kEulerCountsPerDegree - config_.roll_trim_deg;

    // invert_pitch applied BEFORE trim: pitch_trim_deg is meant to be the
    // residual reading recorded with the airframe level, i.e. measured on
    // the final (already-inverted) pitch_deg -- see ImuConfig's doc comment.
    float pitch_deg = static_cast<float>(euler_p) / kEulerCountsPerDegree;
    if (config_.invert_pitch) {
        pitch_deg = -pitch_deg;
    }
    next.pitch_deg = pitch_deg - config_.pitch_trim_deg;

    next.heading_deg = normalizePositiveDegrees(
        static_cast<float>(euler_h) / kEulerCountsPerDegree +
        config_.heading_offset_deg);

    float yaw_deg = normalizeSignedDegrees(next.heading_deg);
    if (config_.yaw_filter_alpha > 0.0f && has_last_yaw_) {
        // Circular EMA: filter the shortest signed angular difference. A
        // normal scalar EMA jumps toward zero when heading crosses +/-180.
        const float alpha = fmaxf(0.0f, fminf(1.0f, config_.yaw_filter_alpha));
        const float yaw_error_deg = normalizeSignedDegrees(yaw_deg - last_yaw_deg_);
        yaw_deg = normalizeSignedDegrees(last_yaw_deg_ + alpha * yaw_error_deg);
    }
    last_yaw_deg_ = yaw_deg;
    has_last_yaw_ = true;
    next.yaw_deg = yaw_deg;

    next.roll_rad = degreesToRadians(next.roll_deg);
    next.pitch_rad = degreesToRadians(next.pitch_deg);
    next.yaw_rad = degreesToRadians(next.yaw_deg);

    next.angular_rate_dps.x =
        static_cast<float>(gyro_x) / kGyroscopeCountsPerDps;
    next.angular_rate_dps.y =
        static_cast<float>(gyro_y) / kGyroscopeCountsPerDps;
    next.angular_rate_dps.z =
        static_cast<float>(gyro_z) / kGyroscopeCountsPerDps;

    if (config_.invert_gyro_y) {
        next.angular_rate_dps.y = -next.angular_rate_dps.y;
    }

    next.acceleration_mss.x =
        static_cast<float>(accel_x) / kAccelerationCountsPerMss;
    next.acceleration_mss.y =
        static_cast<float>(accel_y) / kAccelerationCountsPerMss;
    next.acceleration_mss.z =
        static_cast<float>(accel_z) / kAccelerationCountsPerMss;

    next.magnetic_field_ut.x = static_cast<float>(mag_x) / kMagnetometerCountsPerMicroTesla;
    next.magnetic_field_ut.y = static_cast<float>(mag_y) / kMagnetometerCountsPerMicroTesla;
    next.magnetic_field_ut.z = static_cast<float>(mag_z) / kMagnetometerCountsPerMicroTesla;

    const int16_t linacc_x = readLittleEndian16(linear_accel_gravity_burst, 0);
    const int16_t linacc_y = readLittleEndian16(linear_accel_gravity_burst, 2);
    const int16_t linacc_z = readLittleEndian16(linear_accel_gravity_burst, 4);
    const int16_t gravity_x = readLittleEndian16(linear_accel_gravity_burst, 6);
    const int16_t gravity_y = readLittleEndian16(linear_accel_gravity_burst, 8);
    const int16_t gravity_z = readLittleEndian16(linear_accel_gravity_burst, 10);

    next.linear_acceleration_mss.x = static_cast<float>(linacc_x) / kAccelerationCountsPerMss;
    next.linear_acceleration_mss.y = static_cast<float>(linacc_y) / kAccelerationCountsPerMss;
    next.linear_acceleration_mss.z = static_cast<float>(linacc_z) / kAccelerationCountsPerMss;

    next.gravity_mss.x = static_cast<float>(gravity_x) / kAccelerationCountsPerMss;
    next.gravity_mss.y = static_cast<float>(gravity_y) / kAccelerationCountsPerMss;
    next.gravity_mss.z = static_cast<float>(gravity_z) / kAccelerationCountsPerMss;

    return true;
}

bool Imu::updateCalibration()
{
    if (!initialized_) {
        setError(ImuError::NotInitialized);
        return false;
    }

    unsigned char system = 0;
    unsigned char gyroscope = 0;
    unsigned char accelerometer = 0;
    unsigned char magnetometer = 0;

    if (!succeeded(bno055_get_syscalib_status(&system)) ||
        !succeeded(bno055_get_gyrocalib_status(&gyroscope)) ||
        !succeeded(bno055_get_accelcalib_status(&accelerometer)) ||
        !succeeded(bno055_get_magcalib_status(&magnetometer))) {
        setError(ImuError::ReadFailed);
        return false;
    }

    ImuCalibration next{};
    next.system = system;
    next.gyroscope = gyroscope;
    next.accelerometer = accelerometer;
    next.magnetometer = magnetometer;
    calibration_ = next;
    setError(ImuError::None);
    return true;
}

bool Imu::updateQuaternion()
{
    if (!initialized_) {
        setError(ImuError::NotInitialized);
        return false;
    }

    bno055_quaternion quaternion_raw{};
    if (!succeeded(bno055_read_quaternion_wxyz(&quaternion_raw))) {
        setError(ImuError::ReadFailed);
        return false;
    }

    float qw = static_cast<float>(quaternion_raw.w) * kQuaternionScale;
    float qx = static_cast<float>(quaternion_raw.x) * kQuaternionScale;
    float qy = static_cast<float>(quaternion_raw.y) * kQuaternionScale;
    float qz = static_cast<float>(quaternion_raw.z) * kQuaternionScale;

    const float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 0.0f) {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }

    quaternion_.w = qw;
    quaternion_.x = qx;
    quaternion_.y = qy;
    quaternion_.z = qz;
    setError(ImuError::None);
    return true;
}

bool Imu::readCalibrationOffsets(ImuCalibrationOffsets& out) const
{
    if (!initialized_) {
        return false;
    }

    BNO055_S16 accel_x = 0, accel_y = 0, accel_z = 0;
    BNO055_S16 mag_x = 0, mag_y = 0, mag_z = 0;
    BNO055_S16 gyro_x = 0, gyro_y = 0, gyro_z = 0;

    if (!succeeded(bno055_read_accel_offset_x_axis(&accel_x)) ||
        !succeeded(bno055_read_accel_offset_y_axis(&accel_y)) ||
        !succeeded(bno055_read_accel_offset_z_axis(&accel_z)) ||
        !succeeded(bno055_read_mag_offset_x_axis(&mag_x)) ||
        !succeeded(bno055_read_mag_offset_y_axis(&mag_y)) ||
        !succeeded(bno055_read_mag_offset_z_axis(&mag_z)) ||
        !succeeded(bno055_read_gyro_offset_x_axis(&gyro_x)) ||
        !succeeded(bno055_read_gyro_offset_y_axis(&gyro_y)) ||
        !succeeded(bno055_read_gyro_offset_z_axis(&gyro_z))) {
        return false;
    }

    out.accel_x = accel_x;
    out.accel_y = accel_y;
    out.accel_z = accel_z;
    out.mag_x = mag_x;
    out.mag_y = mag_y;
    out.mag_z = mag_z;
    out.gyro_x = gyro_x;
    out.gyro_y = gyro_y;
    out.gyro_z = gyro_z;
    return true;
}

bool Imu::writeCalibrationOffsets(const ImuCalibrationOffsets& offsets)
{
    if (!initialized_) {
        return false;
    }

    // Offset registers are only writable in CONFIG mode (Bosch datasheet).
    if (!succeeded(bno055_set_operation_mode(OPERATION_MODE_CONFIG))) {
        return false;
    }
    delay(25);

    const bool ok =
        succeeded(bno055_write_accel_offset_x_axis(offsets.accel_x)) &&
        succeeded(bno055_write_accel_offset_y_axis(offsets.accel_y)) &&
        succeeded(bno055_write_accel_offset_z_axis(offsets.accel_z)) &&
        succeeded(bno055_write_mag_offset_x_axis(offsets.mag_x)) &&
        succeeded(bno055_write_mag_offset_y_axis(offsets.mag_y)) &&
        succeeded(bno055_write_mag_offset_z_axis(offsets.mag_z)) &&
        succeeded(bno055_write_gyro_offset_x_axis(offsets.gyro_x)) &&
        succeeded(bno055_write_gyro_offset_y_axis(offsets.gyro_y)) &&
        succeeded(bno055_write_gyro_offset_z_axis(offsets.gyro_z));

    // Always try to return to the selected fusion mode, even if a write
    // above failed, so the sensor doesn't get stuck producing no output.
    const unsigned char fusion_mode = config_.enable_fast_magnetometer_calibration
                                          ? OPERATION_MODE_NDOF
                                          : OPERATION_MODE_NDOF_FMC_OFF;
    const bool resumed = succeeded(bno055_set_operation_mode(fusion_mode));
    delay(20);

    return ok && resumed;
}

ImuData Imu::data() const
{
    return data_;
}

ImuCalibration Imu::calibration() const
{
    return calibration_;
}

ImuQuaternion Imu::quaternion() const
{
    return quaternion_;
}

bool Imu::initialized() const
{
    return initialized_;
}

bool Imu::healthy() const
{
    return initialized_ &&
           last_error_ == ImuError::None &&
           consecutive_read_failures_ == 0U;
}

bool Imu::isFresh(uint32_t maximum_age_us) const
{
    return data_.valid &&
           static_cast<uint32_t>(micros() - data_.timestamp_us) <=
               maximum_age_us;
}

bool Imu::isFullyCalibrated() const
{
    return calibration_.system == 3 &&
           calibration_.gyroscope == 3 &&
           calibration_.accelerometer == 3 &&
           calibration_.magnetometer == 3;
}

ImuError Imu::lastError() const
{
    return last_error_;
}

uint32_t Imu::consecutiveReadFailures() const
{
    return consecutive_read_failures_;
}

float Imu::rollDegrees() const
{
    return data_.roll_deg;
}

float Imu::pitchDegrees() const
{
    return data_.pitch_deg;
}

float Imu::yawDegrees() const
{
    return data_.yaw_deg;
}

float Imu::headingDegrees() const
{
    return data_.heading_deg;
}

ImuVector3f Imu::angularRateDps() const
{
    return data_.angular_rate_dps;
}

float Imu::deltaRollDegrees() const
{
    return data_.delta_roll_deg;
}

float Imu::deltaPitchDegrees() const
{
    return data_.delta_pitch_deg;
}

float Imu::deltaYawDegrees() const
{
    return data_.delta_yaw_deg;
}

float& Imu::rollTrimDegRef()
{
    return config_.roll_trim_deg;
}

float& Imu::pitchTrimDegRef()
{
    return config_.pitch_trim_deg;
}

void Imu::setError(ImuError error)
{
    last_error_ = error;
}

int Imu::busRead(unsigned char device_address,
                 unsigned char register_address,
                 unsigned char* data,
                 unsigned char length)
{
    if (active_instance_ == nullptr || data == nullptr || length == 0U) {
        return ERROR1;
    }

    TwoWire& wire = active_instance_->wire_;
    wire.beginTransmission(device_address);
    if (wire.write(register_address) != 1U ||
        wire.endTransmission(false) != 0U) {
        return ERROR1;
    }

    const size_t received = wire.requestFrom(device_address, length);
    if (received != length) {
        while (wire.available() > 0) {
            (void)wire.read();
        }
        return ERROR1;
    }

    for (unsigned char index = 0; index < length; ++index) {
        if (wire.available() <= 0) {
            return ERROR1;
        }
        data[index] = static_cast<unsigned char>(wire.read());
    }

    return SUCCESS;
}

int Imu::busWrite(unsigned char device_address,
                  unsigned char register_address,
                  unsigned char* data,
                  unsigned char length)
{
    if (active_instance_ == nullptr || data == nullptr || length == 0U) {
        return ERROR1;
    }

    TwoWire& wire = active_instance_->wire_;
    wire.beginTransmission(device_address);
    if (wire.write(register_address) != 1U) {
        return ERROR1;
    }

    for (unsigned char index = 0; index < length; ++index) {
        if (wire.write(data[index]) != 1U) {
            return ERROR1;
        }
    }

    return wire.endTransmission() == 0U ? SUCCESS : ERROR1;
}

void Imu::delayMilliseconds(unsigned int duration_ms)
{
    delay(duration_ms);
}

float Imu::degreesToRadians(float degrees)
{
    return degrees * kRadiansPerDegree;
}

float Imu::normalizePositiveDegrees(float angle_deg)
{
    float result = fmodf(angle_deg, 360.0f);
    if (result < 0.0f) {
        result += 360.0f;
    }
    return result;
}

float Imu::normalizeSignedDegrees(float angle_deg)
{
    float result = normalizePositiveDegrees(angle_deg + 180.0f) - 180.0f;
    if (result >= 180.0f) {
        result -= 360.0f;
    }
    return result;
}

float Imu::absoluteAngleDifferenceDegrees(float current_deg,
                                          float previous_deg)
{
    return fabsf(normalizeSignedDegrees(current_deg - previous_deg));
}

}  // namespace fc
