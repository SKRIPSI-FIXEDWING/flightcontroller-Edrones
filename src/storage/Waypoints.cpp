#include "storage/Waypoints.h"

#include <EEPROM.h>
#include <string.h>

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

uint16_t Waypoints::computeItemChecksum(const WaypointItemRecord* item, int count)
{
    // XOR per kata 16-bit atas seluruh byte record (record 16 byte, tanpa padding).
    uint16_t checksum = 0;
    for (int i = 0; i < count; ++i) {
        uint16_t words[sizeof(WaypointItemRecord) / 2];
        memcpy(words, &item[i], sizeof(words));
        for (uint16_t w : words) {
            checksum ^= w;
        }
    }
    return checksum;
}

void Waypoints::saveItems(const MissionState& mission)
{
    // Magic dihapus dulu supaya tulis yang terputus (mati daya) tidak
    // meninggalkan blok setengah jadi yang dianggap valid.
    EEPROM.put(WaypointEeprom::kAddrItemMagic, static_cast<uint16_t>(0x0000));

    uint16_t checksum = 0;
    for (int i = 0; i < mission.wp_sum; ++i) {
        const WaypointItemRecord record{mission.waypoint_cmd[i], 0, mission.waypoint_p1[i], mission.waypoint_p3[i],
                                        mission.waypoint_radius_m[i]};
        checksum ^= computeItemChecksum(&record, 1);
        const uint16_t addr = WaypointEeprom::kAddrItemData + (i * sizeof(WaypointItemRecord));
        EEPROM.put(addr, record);
    }

    EEPROM.put(WaypointEeprom::kAddrItemVersion, WaypointEeprom::kItemVersion);
    EEPROM.put(WaypointEeprom::kAddrItemCount, static_cast<uint16_t>(mission.wp_sum));
    EEPROM.put(WaypointEeprom::kAddrItemChecksum, checksum);
    EEPROM.put(WaypointEeprom::kAddrItemMagic, WaypointEeprom::kItemMagicNumber);
}

void Waypoints::loadItems(MissionState& mission, uint16_t count)
{
    // Default: semua item NAV_WAYPOINT, radius default.
    for (int i = 0; i < MissionState::kMaxWaypoints; ++i) {
        mission.waypoint_cmd[i] = 0;
        mission.waypoint_p1[i] = 0.0f;
        mission.waypoint_p3[i] = 0.0f;
        mission.waypoint_radius_m[i] = 0.0f;
    }

    uint16_t magic = 0;
    uint16_t version = 0;
    uint16_t saved_count = 0;
    EEPROM.get(WaypointEeprom::kAddrItemMagic, magic);
    EEPROM.get(WaypointEeprom::kAddrItemVersion, version);
    EEPROM.get(WaypointEeprom::kAddrItemCount, saved_count);
    if (magic != WaypointEeprom::kItemMagicNumber || version != WaypointEeprom::kItemVersion ||
        saved_count != count || saved_count > WaypointEeprom::kMaxEepromWaypoints) {
        return;
    }

    WaypointItemRecord temp_item[WaypointEeprom::kMaxEepromWaypoints]{};
    for (uint16_t i = 0; i < saved_count; ++i) {
        const uint16_t addr = WaypointEeprom::kAddrItemData + (i * sizeof(WaypointItemRecord));
        EEPROM.get(addr, temp_item[i]);
    }

    uint16_t saved_checksum = 0;
    EEPROM.get(WaypointEeprom::kAddrItemChecksum, saved_checksum);
    if (computeItemChecksum(temp_item, saved_count) != saved_checksum) {
        return;
    }

    for (uint16_t i = 0; i < saved_count; ++i) {
        mission.waypoint_cmd[i] = temp_item[i].cmd;
        mission.waypoint_p1[i] = temp_item[i].p1;
        mission.waypoint_p3[i] = temp_item[i].p3;
        mission.waypoint_radius_m[i] = temp_item[i].radius_m;
    }
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

    saveItems(mission);

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
    loadItems(mission, saved_count);

    return WaypointStorageResult::Success;
}

void Waypoints::clear()
{
    EEPROM.put(WaypointEeprom::kAddrMagic, static_cast<uint16_t>(0x0000));
    EEPROM.put(WaypointEeprom::kAddrWpCount, static_cast<uint16_t>(0));
    EEPROM.put(WaypointEeprom::kAddrItemMagic, static_cast<uint16_t>(0x0000));
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
