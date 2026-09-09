#pragma once

#include <stdint.h>

#include "drivers/Imu.h"

namespace fc {

enum class ImuCalibrationStorageResult : uint8_t {
    Success = 0,
    NoValidData,
    ChecksumMismatch,
};

namespace ImuCalibrationEeprom {
constexpr uint16_t kMagicNumber = 0xBC55;
constexpr uint16_t kAddrMagic = 2000;
constexpr uint16_t kAddrChecksum = 2002;
constexpr uint16_t kAddrData = 2004;  // sizeof(ImuCalibrationOffsets) = 18 bytes
}  // namespace ImuCalibrationEeprom

/**
 * BNO055 accel/mag/gyro offset-register persistence (see
 * ImuCalibrationOffsets's doc comment in drivers/Imu.h for why this exists).
 * Stateless (static methods), matching storage/Waypoints.h's style.
 *
 * Address block: [2000, 2022). Waypoints (storage/Waypoints.h) occupies
 * [0, 808) and Params (storage/Params.cpp) occupies [1000, 1132) on this
 * Teensy 4.1 build -- both were re-addressed to stop colliding with each
 * other (Waypoints used to swallow Params' old 650-782 block whole; see
 * Params.cpp's kAddrMagic comment). 2000 leaves ample margin past both.
 */
class ImuCalibrationStorage final {
public:
    static ImuCalibrationStorageResult save(const ImuCalibrationOffsets& offsets);
    static ImuCalibrationStorageResult load(ImuCalibrationOffsets& offsets_out);
    static void clear();

private:
    static uint16_t computeChecksum(const ImuCalibrationOffsets& offsets);
};

}  // namespace fc
