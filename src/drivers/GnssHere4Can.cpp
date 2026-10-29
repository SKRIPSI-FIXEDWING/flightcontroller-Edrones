#include "drivers/GnssHere4Can.h"

#include <math.h>
#include <string.h>

namespace {

// uavcan.equipment.gnss.Fix2 (data type 1063) bit offsets. Field order and
// widths per dronecan DSDL (decoded only through `status`; the variable-
// length `covariance` / `pdop` / `ecef_position_velocity` tail is not needed
// for navigation):
//   timestamp 56 | gnss_timestamp 56 | gnss_time_standard 3 | void13 |
//   num_leap_seconds 8 | longitude_deg_1e8 int37 | latitude_deg_1e8 int37 |
//   height_ellipsoid_mm int27 | height_msl_mm int27 | ned_velocity float32[3] |
//   sats_used 6 | status 2 | mode 4 | sub_mode 6 | covariance ...
constexpr uint32_t kFix2OffsetGnssTimestampUs = 56;     // uint56
constexpr uint32_t kFix2OffsetTimeStandard = 112;       // uint3
constexpr uint32_t kFix2OffsetNumLeapSeconds = 128;     // uint8
constexpr uint32_t kFix2OffsetLongitudeDeg1e8 = 136;    // int37
constexpr uint32_t kFix2OffsetLatitudeDeg1e8 = 173;     // int37
constexpr uint32_t kFix2OffsetHeightEllipsoidMm = 210;  // int27
constexpr uint32_t kFix2OffsetHeightMslMm = 237;        // int27
constexpr uint32_t kFix2OffsetNedVelocityN = 264;       // float32
constexpr uint32_t kFix2OffsetNedVelocityE = 296;       // float32
constexpr uint32_t kFix2OffsetNedVelocityD = 328;       // float32
constexpr uint32_t kFix2OffsetSatsUsed = 360;           // uint6
constexpr uint32_t kFix2OffsetStatus = 366;             // uint2

// uavcan.equipment.gnss.Auxiliary (1061): gdop, pdop, hdop, vdop, tdop,
// ndop, edop (float16 each), sats_visible uint7, sats_used uint6.
constexpr uint32_t kAuxiliaryOffsetHdop = 32;  // float16

// Fix2.status constants.
constexpr uint8_t kFix2StatusNoFix = 0;
constexpr uint8_t kFix2Status2D = 2;
constexpr uint8_t kFix2Status3D = 3;

// Fix2.gnss_time_standard constants (TAI is not handled: no receiver we use sends it).
constexpr uint8_t kFix2TimeStandardUtc = 2;
constexpr uint8_t kFix2TimeStandardGps = 3;
constexpr uint8_t kGpsUtcLeapSecondsDefault = 18;  // GPS - UTC since 2017, used when num_leap_seconds = 0
// Anything before 2024-01-01 is a receiver that has not got the time yet.
constexpr uint64_t kUtcSaneMinUs = 1704067200ULL * 1000000ULL;

// uavcan.protocol.dynamic_node_id.Allocation constants.
constexpr uint32_t kAllocationFollowupTimeoutMs = 500;  // FOLLOWUP_TIMEOUT_MS
constexpr uint8_t kMaxAllocatableNodeId = 125;          // 126/127 are reserved for tools

constexpr uint32_t kNodeStatusPeriodMs = 1000;
constexpr uint32_t kStaleCleanupPeriodMs = 1000;
constexpr uint32_t kGnssSourceTimeoutMs = 5000;
constexpr uint32_t kNoNodeWarnAfterMs = 5000;
constexpr uint8_t kMaxEventsPerPoll = 64;

fc::Here4GnssReceiver* g_instance = nullptr;

}  // namespace

namespace fc {

Here4GnssReceiver::Here4GnssReceiver(const Here4CanConfig& config)
    : config_(config)
{
}

Here4GnssReceiver::~Here4GnssReceiver()
{
    if (g_instance == this) {
        g_instance = nullptr;
    }
}

bool Here4GnssReceiver::begin()
{
    data_ = GnssFixData{};
    detected_node_id_ = 0;
    memset(allocated_, 0, sizeof(allocated_));
    partial_unique_id_length_ = 0;
    last_allocated_node_id_ = 0;
    rx_frame_count_ = 0;
    allocation_request_count_ = 0;
    fix2_count_ = 0;
    auxiliary_count_ = 0;
    last_micros_ = micros();
    micros_high_ = 0;

    g_instance = this;

    can_.begin();
    can_.setBaudRate(config_.bitrate_hz);
    can_.setMaxMB(16);
    // events() once up-front switches FlexCAN_T4 to deferred delivery, so
    // the receive callback always runs from poll() (task context), never
    // from the CAN interrupt.
    can_.events();
    can_.enableFIFO();
    can_.enableFIFOInterrupt();
    can_.setFIFOFilter(ACCEPT_ALL);
    can_.onReceive(&Here4GnssReceiver::onCanFrame);

    canardInit(&canard_, canard_memory_pool_, sizeof(canard_memory_pool_),
               &Here4GnssReceiver::onTransferReceived,
               &Here4GnssReceiver::shouldAcceptTransfer, this);
    // Non-anonymous: required to send the (multi-frame) allocation responses.
    canardSetLocalNodeID(&canard_, config_.local_node_id);

    last_node_status_ms_ = millis();
    last_cleanup_ms_ = last_node_status_ms_;

    initialized_ = true;
    setError(Here4CanError::NoNodeSeen);
    return true;
}

uint64_t Here4GnssReceiver::nowUs64()
{
    const uint32_t now = micros();
    if (now < last_micros_) {
        micros_high_ += 1U;  // micros() wrapped (every ~71.6 min)
    }
    last_micros_ = now;
    return (static_cast<uint64_t>(micros_high_) << 32) | now;
}

void Here4GnssReceiver::onCanFrame(const CAN_message_t& message)
{
    if (g_instance != nullptr) {
        g_instance->handleFrame(message);
    }
}

void Here4GnssReceiver::handleFrame(const CAN_message_t& message)
{
    rx_frame_count_ += 1U;

    // DroneCAN uses 29-bit extended identifiers only.
    if (!message.flags.extended || message.flags.remote || message.len == 0 || message.len > 8) {
        return;
    }

    CanardCANFrame frame{};
    frame.id = (message.id & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
    frame.data_len = message.len;
    memcpy(frame.data, message.buf, message.len);

    canardHandleRxFrame(&canard_, &frame, nowUs64());
}

void Here4GnssReceiver::poll()
{
    if (!initialized_) {
        setError(Here4CanError::NotInitialized);
        return;
    }

    // Each events() call delivers at most one frame; the return value's
    // upper bits carry the remaining RX queue depth.
    for (uint8_t i = 0; i < kMaxEventsPerPoll; ++i) {
        const uint64_t pending = can_.events();
        if ((pending >> 12) == 0U) {
            break;
        }
    }

    const uint32_t now_ms = millis();

    if (now_ms - last_cleanup_ms_ >= kStaleCleanupPeriodMs) {
        last_cleanup_ms_ = now_ms;
        canardCleanupStaleTransfers(&canard_, nowUs64());
    }

    if (now_ms - last_node_status_ms_ >= kNodeStatusPeriodMs) {
        last_node_status_ms_ = now_ms;
        sendNodeStatus();
    }

    pumpTxQueue();

    // Release the GNSS source lock if Here4 went silent, so a power-cycled
    // unit that comes back with a different node ID is picked up again.
    if (detected_node_id_ != 0 && (now_ms - last_gnss_seen_ms_) > kGnssSourceTimeoutMs) {
        detected_node_id_ = 0;
    }
    if (detected_node_id_ == 0 && now_ms > kNoNodeWarnAfterMs) {
        setError(Here4CanError::NoNodeSeen);
    }
}

bool Here4GnssReceiver::shouldAcceptTransfer(const CanardInstance* ins,
                                             uint64_t* out_data_type_signature,
                                             uint16_t data_type_id,
                                             CanardTransferType transfer_type,
                                             uint8_t source_node_id)
{
    (void)ins;

    if (transfer_type != CanardTransferTypeBroadcast) {
        return false;
    }

    switch (data_type_id) {
        case kFix2DataTypeId:
            *out_data_type_signature = kFix2DataTypeSignature;
            return true;
        case kAuxiliaryDataTypeId:
            *out_data_type_signature = kAuxiliaryDataTypeSignature;
            return true;
        case kAllocationDataTypeId:
            // Only anonymous requests are for us; allocation traffic from a
            // named node is another allocator's response.
            if (source_node_id == CANARD_BROADCAST_NODE_ID) {
                *out_data_type_signature = kAllocationDataTypeSignature;
                return true;
            }
            return false;
        default:
            return false;
    }
}

void Here4GnssReceiver::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    auto* self = static_cast<Here4GnssReceiver*>(canardGetUserReference(ins));
    if (self == nullptr) {
        return;
    }

    switch (transfer->data_type_id) {
        case kFix2DataTypeId:
            self->handleFix2(*transfer);
            break;
        case kAuxiliaryDataTypeId:
            self->handleAuxiliary(*transfer);
            break;
        case kAllocationDataTypeId:
            self->handleAllocationRequest(*transfer);
            break;
        default:
            break;
    }
}

bool Here4GnssReceiver::acceptGnssSource(uint8_t source_node_id)
{
    if (source_node_id == CANARD_BROADCAST_NODE_ID || source_node_id == config_.local_node_id) {
        return false;
    }
    if (detected_node_id_ == 0) {
        detected_node_id_ = source_node_id;  // first GNSS node wins
    }
    if (source_node_id != detected_node_id_) {
        return false;
    }
    last_gnss_seen_ms_ = millis();
    return true;
}

uint64_t Here4GnssReceiver::decodeUtcUs(const CanardRxTransfer& transfer)
{
    uint64_t gnss_us = 0;
    uint8_t standard = 0;
    uint8_t leap_s = 0;
    if (canardDecodeScalar(&transfer, kFix2OffsetGnssTimestampUs, 56, false, &gnss_us) <= 0 ||
        canardDecodeScalar(&transfer, kFix2OffsetTimeStandard, 3, false, &standard) <= 0 ||
        canardDecodeScalar(&transfer, kFix2OffsetNumLeapSeconds, 8, false, &leap_s) <= 0) {
        return 0;
    }
    uint64_t utc_us = 0;
    if (standard == kFix2TimeStandardUtc) {
        utc_us = gnss_us;
    } else if (standard == kFix2TimeStandardGps) {
        const uint64_t leap = leap_s != 0 ? leap_s : kGpsUtcLeapSecondsDefault;
        utc_us = gnss_us - leap * 1000000ULL;
    }
    return utc_us >= kUtcSaneMinUs ? utc_us : 0;
}

void Here4GnssReceiver::handleFix2(const CanardRxTransfer& transfer)
{
    if (!acceptGnssSource(transfer.source_node_id)) {
        return;
    }

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

    // Fix2 carries no course-over-ground field; derive it from NED velocity
    // (only meaningful while actually moving -- otherwise keep the last one).
    if (data_.ground_speed_mps > 0.5f) {
        float course_deg = atan2f(velocity_e, velocity_n) * (180.0f / static_cast<float>(M_PI));
        if (course_deg < 0.0f) {
            course_deg += 360.0f;
        }
        data_.course_deg = course_deg;
    }

    switch (status) {
        case kFix2Status3D:
            data_.fix_type = GnssFixType::Fix3D;
            break;
        case kFix2Status2D:
            data_.fix_type = GnssFixType::Fix2D;
            break;
        case kFix2StatusNoFix:
        default:  // includes STATUS_TIME_ONLY (1): no usable position
            data_.fix_type = GnssFixType::NoFix;
            break;
    }

    data_.timestamp_us = static_cast<uint32_t>(transfer.timestamp_usec);
    data_.utc_us = decodeUtcUs(transfer);
    data_.sequence += 1U;
    data_.valid = status >= kFix2Status2D;

    fix2_count_ += 1U;
    (void)height_ellipsoid_mm;  // Not exposed on the shared GnssFixData; MSL is.
    setError(Here4CanError::None);
}

void Here4GnssReceiver::handleAuxiliary(const CanardRxTransfer& transfer)
{
    if (!acceptGnssSource(transfer.source_node_id)) {
        return;
    }

    uint16_t hdop_raw = 0;
    if (canardDecodeScalar(&transfer, kAuxiliaryOffsetHdop, 16, false, &hdop_raw) <= 0) {
        setError(Here4CanError::DecodeFailed);
        return;
    }

    const float hdop = canardConvertFloat16ToNativeFloat(hdop_raw);
    if (!isnan(hdop)) {  // DSDL: unknown DOP values are sent as NaN
        data_.hdop = hdop;
    }
    auxiliary_count_ += 1U;
}

// --------------------------------------------------------------------------
// Dynamic node ID allocation server
// (uavcan.protocol.dynamic_node_id.Allocation, DroneCAN specification).
//
// Request payload: byte0 = node_id(7, preferred or 0) << 1 | first_part(1),
// followed by up to 6 bytes of the requester's 16-byte unique ID. The
// requester sends the ID in up to three stages (6+6+4 bytes); after each
// stage the allocator echoes the ID bytes received so far with node_id = 0,
// and only after all 16 bytes arrive replies with the allocated node ID.
// --------------------------------------------------------------------------
void Here4GnssReceiver::handleAllocationRequest(const CanardRxTransfer& transfer)
{
    if (transfer.payload_len < 1U) {
        return;
    }
    allocation_request_count_ += 1U;

    const uint8_t header = transfer.payload_head[0];
    const uint8_t preferred_node_id = header >> 1;
    const bool first_part = (header & 0x01U) != 0U;
    const size_t chunk_length = transfer.payload_len - 1U;
    const uint32_t now_ms = millis();

    if (first_part || (now_ms - partial_unique_id_ms_) > kAllocationFollowupTimeoutMs) {
        partial_unique_id_length_ = 0;
    }
    if (!first_part && partial_unique_id_length_ == 0U) {
        return;  // follow-up stage without a (still valid) first stage
    }
    if (partial_unique_id_length_ + chunk_length > kUniqueIdLength) {
        partial_unique_id_length_ = 0;
        return;
    }

    memcpy(&partial_unique_id_[partial_unique_id_length_], &transfer.payload_head[1], chunk_length);
    partial_unique_id_length_ = static_cast<uint8_t>(partial_unique_id_length_ + chunk_length);
    partial_unique_id_ms_ = now_ms;

    if (partial_unique_id_length_ < kUniqueIdLength) {
        sendAllocationResponse(0, partial_unique_id_, partial_unique_id_length_);
        return;
    }

    uint8_t unique_id[kUniqueIdLength];
    memcpy(unique_id, partial_unique_id_, kUniqueIdLength);
    partial_unique_id_length_ = 0;

    const uint8_t node_id = allocateNodeId(unique_id, preferred_node_id);
    if (node_id != 0U) {
        last_allocated_node_id_ = node_id;
        sendAllocationResponse(node_id, unique_id, kUniqueIdLength);
    }
}

uint8_t Here4GnssReceiver::allocateNodeId(const uint8_t* unique_id, uint8_t preferred_node_id)
{
    for (const AllocatedNode& entry : allocated_) {
        if (entry.node_id != 0U && memcmp(entry.unique_id, unique_id, kUniqueIdLength) == 0) {
            return entry.node_id;  // known node: always re-grant the same ID
        }
    }

    auto is_free = [this](uint8_t id) {
        if (id < CANARD_MIN_NODE_ID || id > kMaxAllocatableNodeId || id == config_.local_node_id) {
            return false;
        }
        for (const AllocatedNode& entry : allocated_) {
            if (entry.node_id == id) {
                return false;
            }
        }
        return true;
    };

    uint8_t chosen = 0;
    if (preferred_node_id != 0U) {
        // Spec: search upward from the preferred ID, then downward.
        for (uint16_t id = preferred_node_id; id <= kMaxAllocatableNodeId && chosen == 0U; ++id) {
            if (is_free(static_cast<uint8_t>(id))) {
                chosen = static_cast<uint8_t>(id);
            }
        }
        for (int16_t id = static_cast<int16_t>(preferred_node_id) - 1; id >= CANARD_MIN_NODE_ID && chosen == 0U; --id) {
            if (is_free(static_cast<uint8_t>(id))) {
                chosen = static_cast<uint8_t>(id);
            }
        }
    } else {
        // Spec: no preference -> highest unused ID, starting from 125.
        for (int16_t id = kMaxAllocatableNodeId; id >= CANARD_MIN_NODE_ID && chosen == 0U; --id) {
            if (is_free(static_cast<uint8_t>(id))) {
                chosen = static_cast<uint8_t>(id);
            }
        }
    }

    if (chosen == 0U) {
        return 0;
    }

    for (AllocatedNode& entry : allocated_) {
        if (entry.node_id == 0U) {
            memcpy(entry.unique_id, unique_id, kUniqueIdLength);
            entry.node_id = chosen;
            return chosen;
        }
    }
    return 0;  // table full
}

void Here4GnssReceiver::sendAllocationResponse(uint8_t node_id, const uint8_t* unique_id, size_t length)
{
    uint8_t payload[1 + kUniqueIdLength];
    payload[0] = static_cast<uint8_t>(node_id << 1);  // first_part_of_unique_id = 0 in responses
    memcpy(&payload[1], unique_id, length);

    canardBroadcast(&canard_, kAllocationDataTypeSignature, kAllocationDataTypeId,
                    &allocation_transfer_id_, CANARD_TRANSFER_PRIORITY_LOW, payload,
                    static_cast<uint16_t>(1U + length));
}

// uavcan.protocol.NodeStatus: uptime_sec u32 | health u2 | mode u3 |
// sub_mode u3 | vendor_specific_status_code u16  (7 bytes, single frame).
void Here4GnssReceiver::sendNodeStatus()
{
    // Don't pile up status frames when nobody is ACKing on the bus.
    if (canardPeekTxQueue(&canard_) != nullptr) {
        return;
    }

    uint8_t payload[7] = {};
    const uint32_t uptime_sec = millis() / 1000U;
    const uint8_t health = 0;     // HEALTH_OK
    const uint8_t mode = 0;       // MODE_OPERATIONAL
    const uint8_t sub_mode = 0;
    const uint16_t vendor_status = 0;
    canardEncodeScalar(payload, 0, 32, &uptime_sec);
    canardEncodeScalar(payload, 32, 2, &health);
    canardEncodeScalar(payload, 34, 3, &mode);
    canardEncodeScalar(payload, 37, 3, &sub_mode);
    canardEncodeScalar(payload, 40, 16, &vendor_status);

    canardBroadcast(&canard_, kNodeStatusDataTypeSignature, kNodeStatusDataTypeId,
                    &node_status_transfer_id_, CANARD_TRANSFER_PRIORITY_LOW, payload,
                    static_cast<uint16_t>(sizeof(payload)));
}

void Here4GnssReceiver::pumpTxQueue()
{
    for (uint8_t i = 0; i < 16; ++i) {
        const CanardCANFrame* frame = canardPeekTxQueue(&canard_);
        if (frame == nullptr) {
            break;
        }

        CAN_message_t message;
        message.id = frame->id & CANARD_CAN_EXT_ID_MASK;
        message.flags.extended = 1;
        message.len = frame->data_len;
        memcpy(message.buf, frame->data, frame->data_len);

        if (can_.write(message) == 0) {
            break;  // mailboxes + FlexCAN_T4 TX ring full; retry next poll
        }
        canardPopTxQueue(&canard_);
    }
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

uint8_t Here4GnssReceiver::lastAllocatedNodeId() const
{
    return last_allocated_node_id_;
}

uint32_t Here4GnssReceiver::rxFrameCount() const
{
    return rx_frame_count_;
}

uint32_t Here4GnssReceiver::allocationRequestCount() const
{
    return allocation_request_count_;
}

uint32_t Here4GnssReceiver::fix2Count() const
{
    return fix2_count_;
}

uint32_t Here4GnssReceiver::auxiliaryCount() const
{
    return auxiliary_count_;
}

void Here4GnssReceiver::setError(Here4CanError error)
{
    last_error_ = error;
}

}  // namespace fc
