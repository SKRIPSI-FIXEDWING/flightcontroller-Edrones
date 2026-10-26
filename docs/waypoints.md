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
- Blok waypoint (0..808) tidak berubah — byte-compatible dengan data yang
  sudah tersimpan dari program lama (magic number, alamat, ukuran maks 50 waypoint
  semua identik).

## Jenis item misi disimpan di EEPROM (2026-10-23)

Sebelumnya hanya lat/lng/alt yang disimpan; `waypoint_cmd`/`waypoint_p1`/
`waypoint_p3`/`waypoint_radius_m` hanya di RAM, sehingga setelah reboot
(misalnya MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN dari Mission Planner) item
LOITER_TURNS dimuat ulang sebagai NAV_WAYPOINT biasa (tlog 2026-10-22 13-35-40:
WP4 cmd 18 dilewati sebagai "WP 4 REACHED" tanpa loiter sampai misi di-upload
ulang).

Sekarang `Waypoints::save()` juga menulis blok item terpisah, `load()` membacanya,
dan `clear()` menghapus magic-nya. Pemanggil tidak berubah.

| Alamat | Isi |
| --- | --- |
| 2100 | magic `0xC3D1` |
| 2102 | versi (1) |
| 2104 | jumlah item (harus sama dengan jumlah waypoint) |
| 2106 | checksum XOR 16-bit |
| 2108.. | 50 × `WaypointItemRecord` 16 byte (cmd, reserved, p1, p3, radius_m) → berakhir 2908 |

Tidak tumpang-tindih dengan waypoint (0..808), Params (1000..~1196) maupun
kalibrasi IMU (2000..2022); batas EEPROM Teensy 4.1 = 4284. Jika magic/versi/
jumlah/checksum tidak cocok (EEPROM lama), semua item dimuat sebagai NAV_WAYPOINT
dengan radius default — waypoint lat/lng/alt tetap dimuat seperti biasa.

## Upload misi: item duplikat tidak lagi membatalkan upload

Lewat radio telemetri yang lossy, Mission Planner kadang mengirim ulang item
seq sebelumnya. Dulu `item.seq != current_upload_seq_` langsung membalas
MAV_MISSION_INVALID_SEQUENCE dan upload batal ("Invalid mission sequence").
Sekarang (sama seperti ArduPilot) `handleMissionItemInt`/`handleMissionItem`:

- `seq < yang ditunggu` → diabaikan, kirim ulang MISSION_REQUEST(_INT) untuk seq
  yang ditunggu;
- `seq > yang ditunggu` → tetap dibatalkan dengan MAV_MISSION_INVALID_SEQUENCE;
- item terakhir datang lagi setelah upload selesai (ACK akhir hilang) →
  MISSION_ACK ACCEPTED dikirim ulang tanpa menyimpan ulang ke EEPROM. Hanya
  berlaku 8 detik setelah upload selesai (`MavlinkConfig::kMissionAckResendWindowMs`)
  dan batal lebih awal oleh MISSION_COUNT atau MISSION_CLEAR_ALL berikutnya.

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
