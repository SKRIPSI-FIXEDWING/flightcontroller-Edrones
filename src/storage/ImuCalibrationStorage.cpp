#include "storage/ImuCalibrationStorage.h"

#include <EEPROM.h>

namespace fc {

uint16_t ImuCalibrationStorage::computeChecksum(const ImuCalibrationOffsets& offsets)
{
    uint16_t checksum = 0;
    checksum ^= static_cast<uint16_t>(offsets.accel_x);
    checksum ^= static_cast<uint16_t>(offsets.accel_y);
    checksum ^= static_cast<uint16_t>(offsets.accel_z);
    checksum ^= static_cast<uint16_t>(offsets.mag_x);
    checksum ^= static_cast<uint16_t>(offsets.mag_y);
    checksum ^= static_cast<uint16_t>(offsets.mag_z);
    checksum ^= static_cast<uint16_t>(offsets.gyro_x);
    checksum ^= static_cast<uint16_t>(offsets.gyro_y);
    checksum ^= static_cast<uint16_t>(offsets.gyro_z);
    return checksum;
}

ImuCalibrationStorageResult ImuCalibrationStorage::save(const ImuCalibrationOffsets& offsets)
{
    EEPROM.put(ImuCalibrationEeprom::kAddrMagic, ImuCalibrationEeprom::kMagicNumber);
    EEPROM.put(ImuCalibrationEeprom::kAddrChecksum, computeChecksum(offsets));
    EEPROM.put(ImuCalibrationEeprom::kAddrData, offsets);
    return ImuCalibrationStorageResult::Success;
}

ImuCalibrationStorageResult ImuCalibrationStorage::load(ImuCalibrationOffsets& offsets_out)
{
    uint16_t magic = 0;
    EEPROM.get(ImuCalibrationEeprom::kAddrMagic, magic);
    if (magic != ImuCalibrationEeprom::kMagicNumber) {
        return ImuCalibrationStorageResult::NoValidData;
    }

    ImuCalibrationOffsets temp{};
    EEPROM.get(ImuCalibrationEeprom::kAddrData, temp);

    uint16_t saved_checksum = 0;
    EEPROM.get(ImuCalibrationEeprom::kAddrChecksum, saved_checksum);
    if (computeChecksum(temp) != saved_checksum) {
        return ImuCalibrationStorageResult::ChecksumMismatch;
    }

    offsets_out = temp;
    return ImuCalibrationStorageResult::Success;
}

void ImuCalibrationStorage::clear()
{
    EEPROM.put(ImuCalibrationEeprom::kAddrMagic, static_cast<uint16_t>(0x0000));
}

}  // namespace fc
