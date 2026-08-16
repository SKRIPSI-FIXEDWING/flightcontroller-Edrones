#include "drivers/Imu.h"

#include <math.h>

namespace {

constexpr uint8_t kExpectedChipId = 0xA0;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadiansPerDegree = kPi / 180.0f;

constexpr float kAccelerationCountsPerMss = 100.0f;
constexpr float kGyroscopeCountsPerDps = 16.0f;
constexpr float kEulerCountsPerDegree = 16.0f;
// BNO055 quaternion registers report Q14 fixed point (2^14 LSB per unit).
constexpr float kQuaternionScale = 1.0f / 16384.0f;

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
    if (!succeeded(bno055_set_accel_unit(0)) ||
        !succeeded(bno055_set_gyro_unit(0)) ||
        !succeeded(bno055_set_euler_unit(0)) ||
        !succeeded(bno055_set_data_output_format(0)) ||
        !succeeded(bno055_set_powermode(POWER_MODE_NORMAL)) ||
        !succeeded(bno055_set_operation_mode(OPERATION_MODE_NDOF))) {
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
    bno055_euler euler{};
    bno055_gyro gyroscope{};
    bno055_accel acceleration{};
    bno055_linear_accel linear_acceleration{};

    if (!succeeded(bno055_read_euler_hrp(&euler)) ||
        !succeeded(bno055_read_gyro_xyz(&gyroscope)) ||
        !succeeded(bno055_read_accel_xyz(&acceleration)) ||
        !succeeded(bno055_read_linear_accel_xyz(&linear_acceleration))) {
        return false;
    }

    next.roll_deg =
        static_cast<float>(euler.r) / kEulerCountsPerDegree - config_.roll_trim_deg;

    // invert_pitch applied BEFORE trim: pitch_trim_deg is meant to be the
    // residual reading recorded with the airframe level, i.e. measured on
    // the final (already-inverted) pitch_deg -- see ImuConfig's doc comment.
    float pitch_deg = static_cast<float>(euler.p) / kEulerCountsPerDegree;
    if (config_.invert_pitch) {
        pitch_deg = -pitch_deg;
    }
    next.pitch_deg = pitch_deg - config_.pitch_trim_deg;

    next.heading_deg = normalizePositiveDegrees(
        static_cast<float>(euler.h) / kEulerCountsPerDegree +
        config_.heading_offset_deg);

    float yaw_deg = normalizeSignedDegrees(next.heading_deg);
    if (config_.yaw_filter_alpha > 0.0f && has_last_yaw_) {
        // Plain EMA, matching fc-skripsi-main's bno_euler.h. Does not
        // special-case the +-180 wrap, so a crossing glitches for one
        // sample -- same limitation as the reference implementation.
        yaw_deg = (1.0f - config_.yaw_filter_alpha) * last_yaw_deg_ +
                  config_.yaw_filter_alpha * yaw_deg;
    }
    last_yaw_deg_ = yaw_deg;
    has_last_yaw_ = true;
    next.yaw_deg = yaw_deg;

    next.roll_rad = degreesToRadians(next.roll_deg);
    next.pitch_rad = degreesToRadians(next.pitch_deg);
    next.yaw_rad = degreesToRadians(next.yaw_deg);

    next.angular_rate_dps.x =
        static_cast<float>(gyroscope.x) / kGyroscopeCountsPerDps;
    next.angular_rate_dps.y =
        static_cast<float>(gyroscope.y) / kGyroscopeCountsPerDps;
    next.angular_rate_dps.z =
        static_cast<float>(gyroscope.z) / kGyroscopeCountsPerDps;

    if (config_.invert_gyro_y) {
        next.angular_rate_dps.y = -next.angular_rate_dps.y;
    }

    next.acceleration_mss.x =
        static_cast<float>(acceleration.x) / kAccelerationCountsPerMss;
    next.acceleration_mss.y =
        static_cast<float>(acceleration.y) / kAccelerationCountsPerMss;
    next.acceleration_mss.z =
        static_cast<float>(acceleration.z) / kAccelerationCountsPerMss;

    next.linear_acceleration_mss.x =
        static_cast<float>(linear_acceleration.x) /
        kAccelerationCountsPerMss;
    next.linear_acceleration_mss.y =
        static_cast<float>(linear_acceleration.y) /
        kAccelerationCountsPerMss;
    next.linear_acceleration_mss.z =
        static_cast<float>(linear_acceleration.z) /
        kAccelerationCountsPerMss;

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
