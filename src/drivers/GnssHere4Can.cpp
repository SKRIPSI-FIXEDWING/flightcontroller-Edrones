#include "drivers/GnssHere4Can.h"

#include <math.h>
#include <string.h>

namespace {

// uavcan.equipment.gnss.Fix2 (data type 1063) bit offsets, decoded only
// through `sub_mode` — the dynamic-length `covariance`/`ecef_position_velocity`
// tail fields that follow are intentionally not decoded (not byte-aligned,
// and not needed for position/velocity/fix-status/sats). Verified against
// dronecan/DSDL/uavcan/equipment/gnss/1063.Fix2.uavcan field widths.
constexpr uint32_t kFix2OffsetLongitudeDeg1e8 = 136;  // int37
constexpr uint32_t kFix2OffsetLatitudeDeg1e8 = 173;   // int37
constexpr uint32_t kFix2OffsetHeightEllipsoidMm = 210;  // int27
constexpr uint32_t kFix2OffsetHeightMslMm = 237;        // int27
constexpr uint32_t kFix2OffsetNedVelocityN = 264;  // float32, byte-aligned
constexpr uint32_t kFix2OffsetNedVelocityE = 296;  // float32
constexpr uint32_t kFix2OffsetNedVelocityD = 328;  // float32
constexpr uint32_t kFix2OffsetSatsUsed = 360;  // uint6
constexpr uint32_t kFix2OffsetStatus = 366;    // uint2
constexpr uint32_t kFix2OffsetMode = 368;      // uint4 (unused, kept for reference)
constexpr uint32_t kFix2OffsetSubMode = 372;   // uint6 (unused, kept for reference)

// uavcan.equipment.gnss.Auxiliary (data type 1061) bit offsets. Verified
// against dronecan/DSDL/uavcan/equipment/gnss/1061.Auxiliary.uavcan.
constexpr uint32_t kAuxiliaryOffsetHdop = 32;  // float16, byte-aligned

constexpr uint8_t kFix2StatusNoFix = 0;
constexpr uint8_t kFix2Status2D = 2;
constexpr uint8_t kFix2Status3D = 3;

}  // namespace

namespace fc {

Here4GnssReceiver::Here4GnssReceiver(const Here4CanConfig& config)
    : config_(config)
{
}

bool Here4GnssReceiver::begin()
{
    data_ = GnssFixData{};
    detected_node_id_ = 0;

    can_.begin();
    can_.setBaudRate(config_.bitrate_hz);

    canardInit(&canard_, canard_memory_pool_, sizeof(canard_memory_pool_),
               &Here4GnssReceiver::onTransferReceived,
               &Here4GnssReceiver::shouldAcceptTransfer, this);

    initialized_ = true;
    setError(Here4CanError::None);
    return true;
}

void Here4GnssReceiver::poll()
{
    if (!initialized_) {
        setError(Here4CanError::NotInitialized);
        return;
    }

    CAN_message_t message;
    while (can_.read(message)) {
        CanardCANFrame frame{};
        frame.id = message.id;
        if (message.flags.extended) {
            frame.id |= CANARD_CAN_FRAME_EFF;
        }
        frame.data_len = message.len;
        memcpy(frame.data, message.buf, message.len);

        canardHandleRxFrame(&canard_, &frame, micros());
    }
}

bool Here4GnssReceiver::shouldAcceptTransfer(const CanardInstance* ins,
                                             uint64_t* out_data_type_signature,
                                             uint16_t data_type_id,
                                             CanardTransferType transfer_type,
                                             uint8_t source_node_id)
{
    (void)ins;
    (void)source_node_id;

    if (transfer_type != CanardTransferTypeBroadcast) {
        return false;
    }

    if (data_type_id == kFix2DataTypeId) {
        *out_data_type_signature = kFix2DataTypeSignature;
        return true;
    }
    if (data_type_id == kAuxiliaryDataTypeId) {
        *out_data_type_signature = kAuxiliaryDataTypeSignature;
        return true;
    }
    return false;
}

void Here4GnssReceiver::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    auto* self = static_cast<Here4GnssReceiver*>(canardGetUserReference(ins));
    if (self == nullptr) {
        return;
    }

    self->detected_node_id_ = transfer->source_node_id;

    if (transfer->data_type_id == kFix2DataTypeId) {
        self->handleFix2(*transfer);
    } else if (transfer->data_type_id == kAuxiliaryDataTypeId) {
        self->handleAuxiliary(*transfer);
    }
}

void Here4GnssReceiver::handleFix2(const CanardRxTransfer& transfer)
{
    int64_t longitude_1e8 = 0;
    int64_t latitude_1e8 = 0;
    int32_t height_ellipsoid_mm = 0;
    int32_t height_msl_mm = 0;
    float velocity_n = 0.0f;
    float velocity_e = 0.0f;
    float velocity_d = 0.0f;
    uint8_t sats_used = 0;
    uint8_t status = 0;

    const bool decoded_ok =
        canardDecodeScalar(&transfer, kFix2OffsetLongitudeDeg1e8, 37, true, &longitude_1e8) > 0 &&
        canardDecodeScalar(&transfer, kFix2OffsetLatitudeDeg1e8, 37, true, &latitude_1e8) > 0 &&
        canardDecodeScalar(&transfer, kFix2OffsetHeightEllipsoidMm, 27, true, &height_ellipsoid_mm) > 0 &&
        canardDecodeScalar(&transfer, kFix2OffsetHeightMslMm, 27, true, &height_msl_mm) > 0 &&
        canardDecodeScalar(&transfer, kFix2OffsetNedVelocityN, 32, true, &velocity_n) > 0 &&
        canardDecodeScalar(&transfer, kFix2OffsetNedVelocityE, 32, true, &velocity_e) > 0 &&
        canardDecodeScalar(&transfer, kFix2OffsetNedVelocityD, 32, true, &velocity_d) > 0 &&
        canardDecodeScalar(&transfer, kFix2OffsetSatsUsed, 6, false, &sats_used) > 0 &&
        canardDecodeScalar(&transfer, kFix2OffsetStatus, 2, false, &status) > 0;

    if (!decoded_ok) {
        setError(Here4CanError::DecodeFailed);
        return;
    }

    data_.latitude_deg = static_cast<double>(latitude_1e8) * 1e-8;
    data_.longitude_deg = static_cast<double>(longitude_1e8) * 1e-8;
    data_.altitude_m = static_cast<float>(height_msl_mm) / 1000.0f;
    data_.velocity_ned_mps[0] = velocity_n;
    data_.velocity_ned_mps[1] = velocity_e;
    data_.velocity_ned_mps[2] = velocity_d;
    data_.ground_speed_mps = sqrtf(velocity_n * velocity_n + velocity_e * velocity_e);
    data_.satellites = sats_used;

    switch (status) {
        case kFix2Status3D:
            data_.fix_type = GnssFixType::Fix3D;
            break;
        case kFix2Status2D:
            data_.fix_type = GnssFixType::Fix2D;
            break;
        case kFix2StatusNoFix:
        default:
            data_.fix_type = GnssFixType::NoFix;
            break;
    }

    data_.timestamp_us = static_cast<uint32_t>(transfer.timestamp_usec);
    data_.sequence += 1U;
    data_.valid = status >= kFix2Status2D;

    (void)height_ellipsoid_mm;  // Not exposed on the shared GnssFixData; MSL is.
    setError(Here4CanError::None);
}

void Here4GnssReceiver::handleAuxiliary(const CanardRxTransfer& transfer)
{
    uint16_t hdop_raw = 0;
    if (canardDecodeScalar(&transfer, kAuxiliaryOffsetHdop, 16, false, &hdop_raw) <= 0) {
        setError(Here4CanError::DecodeFailed);
        return;
    }

    data_.hdop = canardConvertFloat16ToNativeFloat(hdop_raw);
    setError(Here4CanError::None);
}

GnssFixData Here4GnssReceiver::data() const
{
    return data_;
}

bool Here4GnssReceiver::isFresh(uint32_t maximum_age_us) const
{
    return data_.valid &&
           static_cast<uint32_t>(micros() - data_.timestamp_us) <= maximum_age_us;
}

Here4CanError Here4GnssReceiver::lastError() const
{
    return last_error_;
}

uint8_t Here4GnssReceiver::detectedNodeId() const
{
    return detected_node_id_;
}

void Here4GnssReceiver::setError(Here4CanError error)
{
    last_error_ = error;
}

}  // namespace fc
