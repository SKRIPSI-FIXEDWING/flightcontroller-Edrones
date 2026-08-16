# Refactor Waypoint EEPROM Storage

Port langsung dari `WP_EEPROM.h` lama, beroperasi pada `MissionState`
(`include/navigation/MissionState.h`) alih-alih global `waypoint[]`/`wp_sum`.

## Struktur modul

```text
include/storage/Waypoints.h   Kontrak class Waypoints (static methods, stateless)
src/storage/Waypoints.cpp     Implementasi baca/tulis EEPROM
```

## Perubahan

- **Tanpa `Serial.print`/`Serial.printf` internal** — konsisten dengan seluruh modul
  lain di refactor ini. `save()`/`load()` mengembalikan `WaypointStorageResult`
  (enum: `Success`, `TooManyWaypoints`, `NoValidData`, `InvalidCount`,
  `ChecksumMismatch`); pemanggil (lapisan telemetry/log, Fase 7) yang memutuskan
  apa yang dicetak.
- Tidak ada perubahan pada format/alamat EEPROM — byte-compatible dengan data yang
  sudah tersimpan dari program lama (magic number, alamat, ukuran maks 50 waypoint
  semua identik).

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `saveWaypointsToEEPROM()` | `Waypoints::save(mission_state)` |
| `loadWaypointsFromEEPROM()` | `Waypoints::load(mission_state)` |
| `clearWaypointEEPROM()` | `Waypoints::clear()` |
| `hasValidWaypointsInEEPROM()` | `Waypoints::hasValidData()` |
| `printEEPROMStatus()` | `Waypoints::getStoredCount(count)` + `hasValidData()`, dicetak oleh pemanggil |

## File lama yang tidak dimigrasikan

```text
include/WP_EEPROM.h
```
