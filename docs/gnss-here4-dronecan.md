# GNSS DroneCAN (CubePilot Here4) via CAN3 Teensy 4.1

Backend GNSS untuk CubePilot Here4 lewat DroneCAN (UAVCAN v0) di CAN bus. Menerbitkan
`fc::GnssFixData` yang sama dengan backend NMEA dan UBX
(`include/navigation/GnssFixData.h`), sehingga `Ahrs`/`Navigation`/`Mavlink` tidak
berubah.

Dipilih di `include/FC_Config.h`:

```cpp
#define FC_GNSS_BACKEND_HERE4 1   // 1 = Here4 (CAN3), 0 = SE100 NMEA (Serial1)
```

## Wiring

| Teensy 4.1 | Fungsi | Ke |
|---|---|---|
| pin 30 (`CRX3`) | CAN3 RX | RXD transceiver CAN 3.3 V |
| pin 31 (`CTX3`) | CAN3 TX | TXD transceiver CAN 3.3 V |

- Teensy **tidak punya transceiver bawaan** — wajib transceiver eksternal 3.3 V
  (SN65HVD230 / TJA1051T/3 / MCP2562FD, dsb.) antara pin CAN Teensy dan CAN_H/CAN_L.
- Kabel Here4 (4-pin, sumber: manual CubePilot Here4): `1 = 5V`, `2 = CAN_H`,
  `3 = CAN_L`, `4 = GND`. Here4 butuh **5 V** dari BEC, bukan 3.3 V Teensy.
- Terminasi 120 Ω di kedua ujung bus (Here4 biasanya sudah terminasi di sisinya;
  tambahkan satu di sisi Teensy).
- CAN3 dipilih (bukan CAN2 pin 0/1) karena pin 0/1 dipakai `Serial1` NMEA.

## Sumber verifikasi (apa yang dibaca dan dari mana)

| Fakta | Sumber |
|---|---|
| Here4: u-blox NEO-F9P, firmware AP_Periph, DroneCAN, pin CAN 4-pin | Manual CubePilot Here4 (docs.cubepilot.org/user-guides/here-4/here-4-manual) |
| Fix2 = ID 1063, field/bit width, konstanta `STATUS_*`/`MODE_*` | Spesifikasi DroneCAN, daftar standard data types (dronecan.github.io/Specification) |
| Allocation = ID 1, NodeStatus = ID 341, Auxiliary = ID 1061 | idem |
| Signature DSDL Fix2/Auxiliary/NodeStatus/Allocation | Keluaran `show_data_type_info.py` libcanard atas namespace `uavcan` (tabel di forum Zubax) |
| Alur dynamic node ID allocation | DSDL `uavcan.protocol.dynamic_node_id.Allocation` (dronecan/DSDL) |

Catatan dari manual Here4: tabel spesifikasi menyebut "DroneCAN 8 Mbit/s" — itu
**kemampuan CAN FD**. Manual tidak menyebut bitrate default atau `CAN_NODE`; bitrate
default AP_Periph adalah **1 Mbit/s CAN klasik**, yang dipakai driver ini
(`Here4CanConfig::bitrate_hz`).

## Signature DSDL (konstanta di `GnssHere4Can.h`)

| Tipe | ID | Signature |
|---|---|---|
| `uavcan.equipment.gnss.Fix2` | 1063 | `0xCA41E7000F37435F` |
| `uavcan.equipment.gnss.Auxiliary` | 1061 | `0x9BE8BDC4C3DBBFD2` |
| `uavcan.protocol.NodeStatus` | 341 | `0x0F0868D0C1A7C6F1` |
| `uavcan.protocol.dynamic_node_id.Allocation` | 1 | `0x0B2A812620A11D40` |

Fix2 (>7 byte) dan Auxiliary (16 byte) adalah transfer multi-frame; libcanard menolak
transfer yang signature-nya tidak cocok, jadi nilai ini kritis.

## Layout bit Fix2 yang didekode

```text
  0  timestamp 56        56  gnss_timestamp 56      112  gnss_time_standard 3 + void13
128  num_leap_seconds 8  136 longitude_deg_1e8 int37 173 latitude_deg_1e8 int37
210  height_ellipsoid_mm int27   237 height_msl_mm int27
264/296/328 ned_velocity N/E/D float32     360 sats_used 6   366 status 2
```

`status`: 0 = no fix, 1 = time only, 2 = 2D, 3 = 3D. Auxiliary: `hdop` float16 di bit 32.
Field variabel (`covariance`, `pdop`, `ecef_position_velocity`) sengaja tidak didekode.

## Mengapa FC harus jadi "allocator" node ID

AP_Periph (firmware Here4) default `CAN_NODE = 0`, artinya meminta node ID lewat
**dynamic node ID allocation**. Selama belum dapat ID, node berstatus *anonymous* dan
spesifikasi DroneCAN hanya mengizinkan transfer *single-frame* — padahal Fix2 dan
Auxiliary multi-frame, jadi **tidak ada data GPS sama sekali** sampai ada allocator di
bus. Driver ini mengimplementasikan allocator minimal:

1. FC mengambil node ID `local_node_id` (default 10) dan menyiarkan `NodeStatus` 1 Hz.
2. Here4 mengirim request allocation anonim 3 tahap (unique ID 16 byte: 6+6+4).
3. FC membalas tiap tahap dengan gema ID parsial, dan di tahap terakhir dengan node ID
   (default: tertinggi yang bebas ≤ 125; mendukung *preferred ID* sesuai spesifikasi).
4. Setelah itu Here4 mem-broadcast Fix2/Auxiliary dan driver mulai men-decode.

Tabel alokasi hanya di RAM; setelah reboot FC, Here4 meminta ulang (otomatis, tiap
~1 detik) dan dapat ID lagi.

## Pemakaian

```cpp
fc::Here4GnssReceiver g_gnss;           // Here4CanConfig{} default: 1 Mbit/s, node 10

g_gnss.begin();                         // sekali, di setup()
// di task periodik (>= 100 Hz; main.cpp: taskGps 10 ms):
g_gnss.update();                        // = poll(), true bila ada Fix2 baru
const fc::GnssFixData fix = g_gnss.data();
```

## Data yang dihasilkan

- `latitude_deg` / `longitude_deg`: Fix2 `*_deg_1e8` × 1e-8.
- `altitude_m`: `height_msl_mm` / 1000 (MSL, bukan ellipsoid).
- `velocity_ned_mps[0..2]`: `ned_velocity` Fix2 — kecepatan vertikal sungguhan.
- `ground_speed_mps`: √(N²+E²). `course_deg`: `atan2(E, N)` (hanya diperbarui saat
  kecepatan > 0,5 m/s; Fix2 tidak punya field course).
- `satellites`: `sats_used`. `fix_type`: dari `status` (1/time-only → `NoFix`).
- `hdop`: dari pesan Auxiliary (bisa sedikit lebih basi daripada field Fix2).

## Diagnostik bench

`rxFrameCount()` (frame CAN apa pun), `allocationRequestCount()`, `lastAllocatedNodeId()`,
`detectedNodeId()`, `fix2Count()`, `auxiliaryCount()`, `lastError()`. Cara baca:

| Gejala | Artinya |
|---|---|
| `rxFrameCount() == 0` | Wiring / transceiver / terminasi / bitrate salah |
| frame masuk, `allocationRequestCount() == 0` | Here4 sudah punya node ID statis, atau CAN FD aktif |
| request ada, `fix2Count() == 0` | Allocation belum selesai, atau Here4 belum dapat fix (cek `status`) |
| `lastError() == DecodeFailed` | Layout/versi DSDL tidak cocok |

## Yang belum bisa diverifikasi tanpa hardware

- Perilaku sebenarnya Here4 Anda (firmware/parameter `CAN_NODE`, `CAN_FDMODE`) — uji
  dengan hardware. Jika Here4 diset CAN FD, FlexCAN_T4 klasik tidak akan melihat frame
  FD; set kembali ke CAN klasik 1 Mbit/s lewat Mission Planner (DroneCAN GUI).
- Pemakaian bersama node DroneCAN lain di bus yang sama (mis. FC ArduPilot sebagai
  allocator kedua) tidak didukung: hanya boleh satu allocator di bus.
- Kode sudah dikompilasi sukses untuk Teensy 4.1; belum diuji di bus nyata.

## File terkait

```text
include/drivers/GnssHere4Can.h, src/drivers/GnssHere4Can.cpp   Driver + allocator
lib/third_party/libcanard/                                      libcanard (MIT)
include/FC_Config.h                                             FC_GNSS_BACKEND_HERE4
```
