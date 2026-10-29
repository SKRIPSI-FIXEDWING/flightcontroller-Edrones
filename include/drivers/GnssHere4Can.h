#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

extern "C" {
#include <libcanard/canard.h>
}

#include "navigation/GnssFixData.h"

namespace fc {

enum class Here4CanError : uint8_t {
    None = 0,
    NotInitialized,
    BusInitFailed,
    NoNodeSeen,
    DecodeFailed,
};

struct Here4CanConfig {
    // DroneCAN classic CAN bitrate. AP_Periph (Here4 firmware) defaults to
    // 1 Mbit/s classic CAN; the "8 Mbit/s" in CubePilot's spec table is the
    // CAN FD *capability*, not the default bus setting.
    uint32_t bitrate_hz = 1000000;

    // Node ID this flight controller takes on the bus. Also the node ID of
    // the dynamic-node-ID-allocation server below (ArduPilot FCs use 10).
    uint8_t local_node_id = 10;
};

/**
 * Here4 GNSS receiver via DroneCAN (UAVCAN v0) over CAN3
 * (Teensy 4.1: CRX3 = pin 30, CTX3 = pin 31).
 *
 * What this class does, per the DroneCAN specification and the DSDL
 * definitions (dronecan/DSDL):
 *   1. Acts as the bus's dynamic node ID *allocator*
 *      (uavcan.protocol.dynamic_node_id.Allocation, ID 1). Here4 runs
 *      AP_Periph with CAN_NODE=0 by default, i.e. it asks for a node ID and
 *      does NOT broadcast any multi-frame message (Fix2, Auxiliary) until
 *      one is granted -- anonymous nodes may only send single-frame
 *      transfers. Without this allocator the bus stays silent.
 *   2. Broadcasts uavcan.protocol.NodeStatus at 1 Hz (spec requirement for
 *      every node; also makes this FC visible in DroneCAN GUI tools).
 *   3. Receives uavcan.equipment.gnss.Fix2 (ID 1063) and
 *      uavcan.equipment.gnss.Auxiliary (ID 1061) and publishes
 *      fc::GnssFixData like the NMEA/UBX backends.
 *
 * Data type signatures below come from the output of libcanard's
 * show_data_type_info.py over the standard `uavcan` DSDL namespace (they
 * are what libcanard uses for the multi-frame transfer CRC).
 *
 * Transport notes: needs an external 3.3 V CAN transceiver between
 * CRX3/CTX3 and CAN_H/CAN_L; 120 ohm termination at both ends of the bus.
 * Only one instance may exist (the FlexCAN_T4 receive callback is a plain
 * function pointer, so it reaches the instance via a static pointer).
 *
 * Scheduling stays outside this class: call begin() once, then poll()
 * from a task (>= 100 Hz recommended; Here4 also publishes IMU/mag/baro
 * frames that are received and discarded).
 */
class Here4GnssReceiver final {
public:
    explicit Here4GnssReceiver(const Here4CanConfig& config = Here4CanConfig{});
    ~Here4GnssReceiver();

    Here4GnssReceiver(const Here4GnssReceiver&) = delete;
    Here4GnssReceiver& operator=(const Here4GnssReceiver&) = delete;

    bool begin();

    /** Drain pending CAN frames, run the allocator, send NodeStatus/TX queue. */
    void poll();

    /** Same name as GnssNmea/GnssUbx so the backends are drop-in swappable. */
    bool update()
    {
        const uint32_t before = data_.sequence;
        poll();
        return data_.sequence != before;
    }

    GnssFixData data() const;
    bool isFresh(uint32_t maximum_age_us) const;

    Here4CanError lastError() const;

    /** 0 until Here4 has been seen sending Fix2 (or Auxiliary). */
    uint8_t detectedNodeId() const;

    /** Node ID the allocator granted to the most recent allocation request. */
    uint8_t lastAllocatedNodeId() const;

    // Diagnostics (bench bring-up): where in the chain does it stall?
    uint32_t rxFrameCount() const;          // any CAN frame received
    uint32_t allocationRequestCount() const;  // anonymous Allocation requests seen
    uint32_t fix2Count() const;             // Fix2 transfers decoded
    uint32_t auxiliaryCount() const;        // Auxiliary transfers decoded

private:
    // uavcan.equipment.gnss.Fix2 / .Auxiliary / .NodeStatus / .Allocation.
    static constexpr uint16_t kFix2DataTypeId = 1063U;
    static constexpr uint16_t kAuxiliaryDataTypeId = 1061U;
    static constexpr uint16_t kNodeStatusDataTypeId = 341U;
    static constexpr uint16_t kAllocationDataTypeId = 1U;

    static constexpr uint64_t kFix2DataTypeSignature = 0xCA41E7000F37435FULL;
    static constexpr uint64_t kAuxiliaryDataTypeSignature = 0x9BE8BDC4C3DBBFD2ULL;
    static constexpr uint64_t kNodeStatusDataTypeSignature = 0x0F0868D0C1A7C6F1ULL;
    static constexpr uint64_t kAllocationDataTypeSignature = 0x0B2A812620A11D40ULL;

    static constexpr size_t kUniqueIdLength = 16;
    static constexpr size_t kMaxAllocatedNodes = 8;

    struct AllocatedNode {
        uint8_t unique_id[kUniqueIdLength];
        uint8_t node_id;  // 0 = slot unused
    };

    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);
    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
    static void onCanFrame(const CAN_message_t& message);

    void handleFrame(const CAN_message_t& message);
    /** Fix2 gnss_timestamp as UTC microseconds since 1970, 0 if the receiver has no time yet. */
    static uint64_t decodeUtcUs(const CanardRxTransfer& transfer);
    void handleFix2(const CanardRxTransfer& transfer);
    void handleAuxiliary(const CanardRxTransfer& transfer);
    void handleAllocationRequest(const CanardRxTransfer& transfer);
    uint8_t allocateNodeId(const uint8_t* unique_id, uint8_t preferred_node_id);
    void sendAllocationResponse(uint8_t node_id, const uint8_t* unique_id, size_t length);
    void sendNodeStatus();
    void pumpTxQueue();
    bool acceptGnssSource(uint8_t source_node_id);
    uint64_t nowUs64();
    void setError(Here4CanError error);

    Here4CanConfig config_{};
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_{};

    CanardInstance canard_{};
    uint8_t canard_memory_pool_[2048] = {};

    // Dynamic node ID allocation server state (spec: uavcan.protocol.
    // dynamic_node_id.Allocation, single non-redundant allocator; table is
    // RAM only, so IDs are re-granted from scratch after an FC reboot --
    // a re-requesting Here4 simply gets the same or a new ID).
    AllocatedNode allocated_[kMaxAllocatedNodes] = {};
    uint8_t partial_unique_id_[kUniqueIdLength] = {};
    uint8_t partial_unique_id_length_ = 0;
    uint32_t partial_unique_id_ms_ = 0;
    uint8_t last_allocated_node_id_ = 0;

    uint8_t node_status_transfer_id_ = 0;
    uint8_t allocation_transfer_id_ = 0;
    uint32_t last_node_status_ms_ = 0;
    uint32_t last_cleanup_ms_ = 0;

    uint32_t last_micros_ = 0;
    uint32_t micros_high_ = 0;

    GnssFixData data_{};
    uint8_t detected_node_id_ = 0;
    uint32_t last_gnss_seen_ms_ = 0;
    Here4CanError last_error_ = Here4CanError::NotInitialized;
    bool initialized_ = false;

    uint32_t rx_frame_count_ = 0;
    uint32_t allocation_request_count_ = 0;
    uint32_t fix2_count_ = 0;
    uint32_t auxiliary_count_ = 0;
};

}  // namespace fc
