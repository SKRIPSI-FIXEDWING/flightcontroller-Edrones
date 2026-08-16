#include "storage/Waypoints.h"

#include <EEPROM.h>

namespace fc {

uint16_t Waypoints::computeChecksum(const Locations* waypoint, int count)
{
    uint16_t checksum = 0;
    for (int i = 0; i < count; ++i) {
        checksum ^= (waypoint[i].lat >> 16) & 0xFFFF;
        checksum ^= waypoint[i].lat & 0xFFFF;
        checksum ^= (waypoint[i].lng >> 16) & 0xFFFF;
        checksum ^= waypoint[i].lng & 0xFFFF;
        checksum ^= (waypoint[i].alt >> 16) & 0xFFFF;
        checksum ^= waypoint[i].alt & 0xFFFF;
    }
    return checksum;
}

WaypointStorageResult Waypoints::save(const MissionState& mission)
{
    if (mission.wp_sum > static_cast<int>(WaypointEeprom::kMaxEepromWaypoints)) {
        return WaypointStorageResult::TooManyWaypoints;
    }

    EEPROM.put(WaypointEeprom::kAddrMagic, WaypointEeprom::kMagicNumber);
    EEPROM.put(WaypointEeprom::kAddrWpCount, static_cast<uint16_t>(mission.wp_sum));

    const uint16_t checksum = computeChecksum(mission.waypoint, mission.wp_sum);
    EEPROM.put(WaypointEeprom::kAddrChecksum, checksum);

    for (int i = 0; i < mission.wp_sum; ++i) {
        const uint16_t addr = WaypointEeprom::kAddrWpData + (i * sizeof(Locations));
        EEPROM.put(addr, mission.waypoint[i]);
    }

    return WaypointStorageResult::Success;
}

WaypointStorageResult Waypoints::load(MissionState& mission)
{
    uint16_t magic = 0;
    EEPROM.get(WaypointEeprom::kAddrMagic, magic);
    if (magic != WaypointEeprom::kMagicNumber) {
        return WaypointStorageResult::NoValidData;
    }

    uint16_t saved_count = 0;
    EEPROM.get(WaypointEeprom::kAddrWpCount, saved_count);
    if (saved_count > WaypointEeprom::kMaxEepromWaypoints) {
        return WaypointStorageResult::InvalidCount;
    }

    Locations temp_waypoint[MissionState::kMaxWaypoints]{};
    for (uint16_t i = 0; i < saved_count; ++i) {
        const uint16_t addr = WaypointEeprom::kAddrWpData + (i * sizeof(Locations));
        EEPROM.get(addr, temp_waypoint[i]);
    }

    const uint16_t calculated_checksum = computeChecksum(temp_waypoint, saved_count);
    uint16_t saved_checksum = 0;
    EEPROM.get(WaypointEeprom::kAddrChecksum, saved_checksum);

    if (calculated_checksum != saved_checksum) {
        return WaypointStorageResult::ChecksumMismatch;
    }

    for (uint16_t i = 0; i < saved_count; ++i) {
        mission.waypoint[i] = temp_waypoint[i];
    }
    mission.wp_sum = saved_count;

    return WaypointStorageResult::Success;
}

void Waypoints::clear()
{
    EEPROM.put(WaypointEeprom::kAddrMagic, static_cast<uint16_t>(0x0000));
    EEPROM.put(WaypointEeprom::kAddrWpCount, static_cast<uint16_t>(0));
}

bool Waypoints::hasValidData()
{
    uint16_t magic = 0;
    EEPROM.get(WaypointEeprom::kAddrMagic, magic);
    return magic == WaypointEeprom::kMagicNumber;
}

bool Waypoints::getStoredCount(uint16_t& count)
{
    if (!hasValidData()) {
        return false;
    }
    EEPROM.get(WaypointEeprom::kAddrWpCount, count);
    return true;
}

}  // namespace fc
