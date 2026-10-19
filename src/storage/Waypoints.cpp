#include "storage/Waypoints.h"

#include <EEPROM.h>
#include <math.h>

namespace fc {

struct __attribute__((packed)) MissionMeta {
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    uint8_t loiter;
};
static_assert(sizeof(MissionMeta) == 19, "Mission metadata layout changed");

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
        const MissionMeta meta{mission.waypoint_command[i], mission.waypoint_param1[i],
                               mission.waypoint_radius_m[i], mission.waypoint_param3[i],
                               mission.waypoint_param4[i], static_cast<uint8_t>(mission.waypoint_loiter[i])};
        EEPROM.put(WaypointEeprom::kAddrMissionMeta + sizeof(uint16_t) +
                       (i * sizeof(MissionMeta)),
                   meta);
    }
    EEPROM.put(WaypointEeprom::kAddrMissionMeta, WaypointEeprom::kMissionMetaMagic);

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
    uint16_t meta_magic = 0;
    EEPROM.get(WaypointEeprom::kAddrMissionMeta, meta_magic);
    if (meta_magic == WaypointEeprom::kMissionMetaMagic) {
        for (uint16_t i = 0; i < saved_count; ++i) {
            MissionMeta meta{};
            EEPROM.get(WaypointEeprom::kAddrMissionMeta + sizeof(uint16_t) +
                           (i * sizeof(MissionMeta)),
                       meta);
            mission.waypoint_command[i] = meta.command;
            mission.waypoint_param1[i] = meta.param1;
            mission.waypoint_radius_m[i] = meta.param2 > 0.0f ? meta.param2 : 0.0f;
            mission.waypoint_param3[i] = meta.param3;
            mission.waypoint_param4[i] = meta.param4;
            mission.waypoint_loiter[i] = meta.loiter != 0;
            mission.waypoint_loiter_radius_m[i] =
                mission.waypoint_loiter[i] && fabsf(meta.param3) >= 20.0f ? fabsf(meta.param3) : 0.0f;
        }
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
