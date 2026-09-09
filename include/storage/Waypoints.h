#pragma once

#include <Locations.h>
#include <stdint.h>

#include "navigation/MissionState.h"

namespace fc {

enum class WaypointStorageResult : uint8_t {
    Success = 0,
    TooManyWaypoints,
    NoValidData,
    InvalidCount,
    ChecksumMismatch,
};

// Occupies [0, 8 + kMaxEepromWaypoints*sizeof(Locations)) = [0, 808) on this
// Teensy 4.1 build. storage/Params.cpp's block starts at 1000 and
// storage/ImuCalibrationStorage.h's at 2000, both chosen with margin past
// this range -- if kMaxEepromWaypoints or Locations grows, re-check those
// two don't get swallowed the way Params used to be (see Params.cpp's
// kAddrMagic comment for that incident).
namespace WaypointEeprom {
constexpr uint16_t kMagicNumber = 0xABCD;
constexpr uint16_t kAddrMagic = 0;
constexpr uint16_t kAddrWpCount = 2;
constexpr uint16_t kAddrChecksum = 4;
constexpr uint16_t kAddrWpData = 8;
constexpr uint16_t kMaxEepromWaypoints = 50;
}  // namespace WaypointEeprom

/**
 * Waypoint EEPROM persistence (legacy WP_EEPROM.h), operating on a
 * MissionState instead of the global `waypoint[]`/`wp_sum` pair. Stateless
 * (static methods) — no Serial logging internally, unlike legacy (see
 * docs/waypoints.md); callers report/log the returned WaypointStorageResult.
 */
class Waypoints final {
public:
    static WaypointStorageResult save(const MissionState& mission);
    static WaypointStorageResult load(MissionState& mission);
    static void clear();
    static bool hasValidData();
    static bool getStoredCount(uint16_t& count);

private:
    static uint16_t computeChecksum(const Locations* waypoint, int count);
};

}  // namespace fc
