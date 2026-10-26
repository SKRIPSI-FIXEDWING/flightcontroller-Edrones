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

// Blok terpisah untuk jenis perintah tiap item (MAV_CMD_*) + param1/param3 +
// radius klaim per-WP, supaya item LOITER_* tidak berubah menjadi
// NAV_WAYPOINT setelah reboot. Menempati [2100, 2108 + 50*16) = [2100, 2908):
// sesudah blok IMU kalibrasi (2000..2022) dan jauh di bawah batas EEPROM
// Teensy 4.1 (4284). Magic/versi sendiri: EEPROM lama (tanpa blok ini) atau
// blok yang jumlahnya tidak cocok dengan blok waypoint dimuat sebagai
// "semua NAV_WAYPOINT, radius default".
constexpr uint16_t kItemMagicNumber = 0xC3D1;
constexpr uint16_t kItemVersion = 1;
constexpr uint16_t kAddrItemMagic = 2100;
constexpr uint16_t kAddrItemVersion = 2102;
constexpr uint16_t kAddrItemCount = 2104;
constexpr uint16_t kAddrItemChecksum = 2106;
constexpr uint16_t kAddrItemData = 2108;
}  // namespace WaypointEeprom

// Satu entri blok item di EEPROM (16 byte, tanpa padding).
struct WaypointItemRecord {
    uint16_t cmd;       // MAV_CMD_*; 0 = NAV_WAYPOINT
    uint16_t reserved;  // selalu 0 (perataan)
    float p1;
    float p3;
    float radius_m;
};
static_assert(sizeof(WaypointItemRecord) == 16, "WaypointItemRecord harus 16 byte");

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
    static uint16_t computeItemChecksum(const WaypointItemRecord* item, int count);
    static void saveItems(const MissionState& mission);
    // Isi waypoint_cmd/p1/p3/radius_m untuk `count` item; jika blok item
    // tidak valid, semua item dianggap NAV_WAYPOINT dengan radius default.
    static void loadItems(MissionState& mission, uint16_t count);
};

}  // namespace fc
