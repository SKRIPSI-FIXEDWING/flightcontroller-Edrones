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
    // DroneCAN's standard bus bitrate. Verify Here4 hasn't been configured
    // otherwise before trusting this default (see docs/gnss-here4-dronecan.md).
    uint32_t bitrate_hz = 1000000;
};

/**
 * Here4 GNSS receiver via DroneCAN (UAVCAN v0) over CAN3.
 *
 * CAN3 (pins 30/31 on Teensy 4.1) is used deliberately instead of CAN2
 * (pins 0/1), because GnssNmea already claims pins 0/1 for Serial1 — wiring
 * Here4 to CAN2 would collide with the SE100 NMEA GPS backend if both are
 * ever populated on the same board. Change the FlexCAN_T4 template
 * parameter below if your wiring differs.
 *
 * IMPORTANT — read docs/gnss-here4-dronecan.md before trusting this on real
 * hardware. Two things are unverified and require a bench CAN sniffer:
 *   1. Whether Here4 free-runs broadcasting Fix2/Auxiliary immediately, or
 *      waits for a DroneCAN dynamic node-ID allocation exchange first (this
 *      driver assumes the former — the common case — and does not
 *      implement an allocation responder).
 *   2. The exact DSDL "data type signature" constants used below for
 *      multi-frame CRC verification (kFix2DataTypeSignature,
 *      kAuxiliaryDataTypeSignature) are PLACEHOLDERS. Fix2's payload spans
 *      multiple CAN frames, and libcanard rejects every multi-frame
 *      transfer whose signature doesn't match — get this wrong and no fix
 *      data will ever be delivered, with no other symptom. Compute the
 *      real values with libcanard's own `show_data_type_info.py` (or
 *      equivalent DSDL-signature tooling) against
 *      `dronecan/DSDL/uavcan/equipment/gnss/{1063.Fix2,1061.Auxiliary}.uavcan`
 *      before flight.
 *
 * Scheduling stays outside this class, same as the other GNSS backends:
 * call begin() once, then poll() from the flight scheduler.
 */
class Here4GnssReceiver final {
public:
    explicit Here4GnssReceiver(const Here4CanConfig& config = Here4CanConfig{});

    Here4GnssReceiver(const Here4GnssReceiver&) = delete;
    Here4GnssReceiver& operator=(const Here4GnssReceiver&) = delete;

    bool begin();

    /** Drain all pending CAN frames and feed them through libcanard. */
    void poll();

    GnssFixData data() const;
    bool isFresh(uint32_t maximum_age_us) const;

    Here4CanError lastError() const;

    /** 0 until the first frame from Here4 has been seen. */
    uint8_t detectedNodeId() const;

private:
    // uavcan.equipment.gnss.Fix2 / .Auxiliary, per dronecan/DSDL.
    static constexpr uint16_t kFix2DataTypeId = 1063U;
    static constexpr uint16_t kAuxiliaryDataTypeId = 1061U;

    // PLACEHOLDER — see the class-level warning above. Not yet verified
    // against the official DSDL signature tooling.
    static constexpr uint64_t kFix2DataTypeSignature = 0x0000000000000000ULL;
    static constexpr uint64_t kAuxiliaryDataTypeSignature = 0x0000000000000000ULL;

    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);
    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);

    void handleFix2(const CanardRxTransfer& transfer);
    void handleAuxiliary(const CanardRxTransfer& transfer);
    void setError(Here4CanError error);

    Here4CanConfig config_{};
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_{};

    CanardInstance canard_{};
    uint8_t canard_memory_pool_[1024] = {};

    GnssFixData data_{};
    uint8_t detected_node_id_ = 0;
    Here4CanError last_error_ = Here4CanError::NotInitialized;
    bool initialized_ = false;
};

}  // namespace fc
