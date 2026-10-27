# Refactor Params (parameter EEPROM)

Rebuild FW-only dari `Params.h` lama sebagai `class Params` (bukan static
global array + kumpulan `inline` function). Skema EEPROM baru sepenuhnya
(magic number `0xFC01`, beda dari `0xCDAC` lama) -- **data EEPROM lama tidak
kompatibel dan tidak akan ter-load**, dianggap tidak valid dan diganti default,
karena layout parameternya sudah beda total.

## Yang dihapus

- **Semua param copter** (`CP_STAB_*`, `CP_RATE_*`, `CP_ALT_P`, `CP_ZVEL_P`) --
  tidak relevan, tidak ada copter lagi.
- **`MODEL_UAV`** -- tidak ada lagi switch tipe wahana; build ini FW-only
  secara compile-time.
- **`FW_ROLL/PITCH/YAW/THR_P/I/D`** (gain PID lama) -- digantikan `K` LQR yang
  **tetap, dihitung offline** (lihat `docs/attitude-lqr.md`) -- **sengaja
  tidak diekspos** sebagai param EEPROM, karena keputusan Anda adalah `K`
  tetap/tidak berubah saat runtime.
- **`FW_ATHR_MIN/MAX`** (batas PWM auto-throttle `FW_CONTROL`) -- `FW_CONTROL`
  sendiri sudah tidak di-port.
- **Pin/channel servo** (`SERVO_AIL_L` dst, `MOTOR_1-4_PIN`) -- lihat
  `docs/actuator.md`, pin jarang perlu di-tuning tanpa reflash.

## Yang ditambahkan

- **`NAVL1_PERIOD`, `NAVL1_DAMPING`, `NAVL1_XTRACK_I`** (dulu `L1_PERIOD`, `L1_DAMPING`, `L1_XTRACK_I`; lihat bagian "Nama param & deskripsi Mission Planner") -- di program lama, `_L1_period`
  adalah `float` bebas file-scope, **tidak terdaftar sebagai param sama
  sekali** (tidak ada expose EEPROM). Sekarang bisa di-tuning tanpa reflash.
- **`FUZZY_MIN_PER`, `FUZZY_MAX_PER`** -- batas skala period fuzzy tuner (inti
  skripsi).

## Catatan desain: `AIRSPEED_CRUISE`/`MIN_AIRSPEED`/`MAX_AIRSPEED`

Param ini didaftarkan menunjuk ke field `TecsConfig` (satu-satunya sumber
otoritatif), **bukan** didaftarkan dua kali ke `NavigationConfig`/
`AttitudeControllerConfig` juga (yang masing-masing punya field serupa untuk
keperluan internal mereka sendiri). Ini untuk menghindari EEPROM menulis ke
banyak salinan yang bisa desinkron. Saat inisialisasi kendaraan (Fase 8),
pemilik vehicle bertanggung jawab menyalin nilai `TecsConfig` yang sudah
di-load ke config lain yang butuh nilai sama, sebelum konstruksi objek
tersebut selesai.

## Kepemilikan

```cpp
#include "storage/Params.h"

fc::L1ControllerConfig l1_config;
fc::FuzzyL1TunerConfig fuzzy_config;
fc::TecsConfig tecs_config;

fc::Params params;
params.initFixedWing(l1_config, fuzzy_config, tecs_config);
fc::ParamLoadResult result = params.load();  // isi l1_config/fuzzy_config/tecs_config dari EEPROM (atau default)

// Baru SEKARANG konstruksi objek yang memakai config ini:
fc::L1Controller l1(l1_config);
fc::FuzzyL1Tuner fuzzy_tuner(fuzzy_config);
fc::Tecs tecs(tecs_config);
```

Tidak ada `Serial.print`/`printf` internal (beda dari program lama yang
mencetak di hampir setiap operasi) -- `load()` mengembalikan
`ParamLoadResult` (`LoadedFromEeprom`/`NoValidEepromData`/`SchemaUpgraded`),
pemanggil yang mencetak/mencatat jika perlu.

## Alamat EEPROM (diperbaiki 2026-08-20)

`Params` sekarang menempati `[1000, 1140)` (sejak 2026-10-26 maksimum `[1000, 1260)`, `kMaxParams` 64), bukan `[650, 782)` seperti
sebelumnya. Alamat lama tumpang tindih penuh dengan `storage/Waypoints.h`
(`[0, 808)` untuk `kMaxEepromWaypoints=50 * sizeof(Locations)=16` byte) —
setiap `Waypoints::save()` menimpa data `Params`, dan sebaliknya, tanpa ada
yang menyadarinya karena kedua modul independen dan tidak saling tahu
alamat masing-masing.

Peta EEPROM saat ini (Teensy 4.1, `kTeensy41EepromSize=4284`):

| Modul | Rentang alamat |
| --- | --- |
| `storage/Waypoints.h` | `[0, 808)` |
| `storage/Params.cpp` | `[1000, 1260)` |
| `storage/ImuCalibrationStorage.h` | `[2000, 2022)` |

Konsekuensi upgrade: begitu firmware ini pertama kali boot, magic number di
alamat 1000 tidak akan cocok (data lama di sana, kalaupun ada, adalah bekas
tulisan `Waypoints` di rentang lamanya yang kebetulan tumpang tindih dengan
alamat baru `Params`) — `Params::load()` sudah menangani ini secara aman
lewat jalur `NoValidEepromData` yang sudah ada (reset ke default lalu
simpan), tidak ada perubahan kode yang perlu dilakukan untuk migrasi ini,
tapi tuning param yang tersimpan sebelum perbaikan ini akan hilang sekali.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `initParams()` | `params.initFixedWing(l1_config, fuzzy_config, tecs_config)` |
| `loadParamsFromEEPROM()` | `params.load()` |
| `saveParamsToEEPROM()` | `params.save()` |
| `resetParamsToDefault()` | `params.resetToDefaults()` |
| `setParamValue(name, val)` | `params.setValue(name, val)` |
| `getParamValue(name, &val)` | `params.getValue(name, val)` |
| `printParams()` | Iterasi `params.count()`/`params.entryAt(i)`, dicetak pemanggil |

## File lama yang tidak dimigrasikan

```text
include/Params.h
```

## `WP_RADIUS` -- radius klaim waypoint dari Mission Planner (2026-10-17)

`WP_RADIUS` (default 15 m, rentang 2-100 m) mengisi
`NavigationConfig::wp_radius_default_m`. Namanya sama dengan ArduPlane, jadi
kotak **WP Radius** di tab Flight Plan Mission Planner membaca dan
menulisnya langsung (MP hanya mengaktifkan kotak itu bila param `WP_RADIUS`
ada di daftar param). Berbeda dengan param lain, nilainya berlaku **langsung**:
`Mavlink::handleParamSet()` meneruskannya ke `Navigation::setWaypointRadius()`.
Radius per-WP (MAVLink `param2 > 0`) tetap diutamakan.

WP tetap dianggap tercapai bila wahana melewati *finish-line* (garis tegak
lurus leg di titik WP), walaupun di luar radius -- radius hanya salah satu
syarat klaim.

`WP_RADIUS` diletakkan di **akhir** tabel (`kMaxParams = 33`). `Params::load()`
membaca EEPROM berdasarkan **urutan**, bukan nama: menyisipkan param di tengah
menggeser nilai semua param sesudahnya (terjadi saat `AIL_OUT_SIGN` disisipkan;
log 2026-10-15 menunjukkan `ROLL_KP`..`YAW_KP` tergeser satu slot). Setelah
flashing firmware dengan param baru di akhir, EEPROM lama naik skema dengan
aman (`SchemaUpgraded`): param lama tidak bergeser, param baru memakai default.

## Nama param & deskripsi Mission Planner (2026-10-18)

Protokol MAVLink tidak membawa teks deskripsi param (`PARAM_VALUE` hanya
berisi nama, nilai, tipe). Kolom **Description**, **Units**, dan **Range** di
*Config > Full Parameter List* diisi Mission Planner dari file definisi
ArduPlane (`C:\ProgramData\Mission Planner\ArduPlane.apm.pdef.xml`), yang
diunduh ulang otomatis oleh Mission Planner. Karena dicocokkan berdasarkan
**nama**, hanya param bernama sama dengan ArduPlane yang punya keterangan;
mengedit file itu sendiri tidak awet (tertimpa saat MP memperbarui).

Karena itu param L1 memakai nama ArduPlane:

| Param | Dulu | Default | Keterangan |
| --- | --- | --- | --- |
| `NAVL1_PERIOD` | `L1_PERIOD` | 22 s | periode loop L1 -- penentu utama agresivitas belok di AUTO |
| `NAVL1_DAMPING` | `L1_DAMPING` | 0.707 | rasio redaman L1 (nilai tetap proposal) |
| `NAVL1_XTRACK_I` | `L1_XTRACK_I` | 0.2 | gain integrator cross-track |
| `WP_RADIUS` | -- | 15 m | radius klaim WP (lihat bagian `WP_RADIUS`) |
| `ROLL_LIMIT_DEG` | -- (tetap 36) | 45 deg | batas bank perintah navigasi AUTO, berlaku langsung |

Catatan:
- Nilai yang tersimpan di EEPROM ikut terbawa (EEPROM dimuat berdasarkan
  urutan, bukan nama). Default di atas hanya berlaku pada EEPROM kosong.
- Setelah flashing, tekan **Refresh Params** di MP agar nama lama hilang.
- Range MP untuk `NAVL1_XTRACK_I` adalah 0-0.1 (ArduPlane); nilai 0.2 di sini
  akan ditandai di luar rentang oleh MP, tetapi tetap diterima firmware
  (batas firmware 0-1; integrator sendiri dijepit +-0.1 rad di L1Controller).
- ~~Param L1 berlaku setelah reboot~~ (diubah 2026-10-25): `NAVL1_PERIOD`,
  `NAVL1_DAMPING`, `NAVL1_XTRACK_I`, `FUZZY_ENABLE`, `FUZZY_MIN_PER`,
  `FUZZY_MAX_PER`, `WP_RADIUS`, `ROLL_LIMIT_DEG` dan `DIST_ENABLE` berlaku
  LANGSUNG (lihat bagian di bawah). Param lain tetap butuh reboot.

## `ROLL_LIMIT_DEG` -- batas bank AUTO (2026-10-19)

Mengisi `NavigationConfig::roll_limit_deg`: `nav_roll` dari L1 dijepit ke
+-`ROLL_LIMIT_DEG` sebelum masuk kendali attitude (Navigation.cpp,
`updateAutoAttitudeTargets()`). Default 45 deg (dulu tetap 36 = HEAD_MAX
Khageswara), rentang 20-60 deg, berlaku langsung tanpa reboot. Hanya AUTO;
FBWA tetap memakai pemetaan stick. Ditambahkan di akhir tabel
(`kMaxParams = 34`), jadi EEPROM lama naik skema dan param ini mengambil
default 45 saat boot pertama.

Bank lebih besar = radius belok lebih kecil (R = v^2 / (g tan(bank))), tetapi
faktor beban naik (1/cos 45 = 1.41, kecepatan stall naik ~19%) dan wahana lebih
mudah kehilangan ketinggian saat belok.

## `WP_LOITER_RAD` -- radius & arah mode LOITER dari Mission Planner (2026-10-19)

`WP_LOITER_RAD` (default -50 m, rentang -500..500) = kotak "Loiter Radius" di
Flight Plan Mission Planner. Tanda = arah (negatif = CCW, positif = CW, konvensi
ArduPlane); radius efektif minimal 20 m. Berlaku LIVE
(`Mavlink::handleParamSet` -> `Navigation::setLoiterRadius`) dan juga dipakai
RTL / loiter akhir misi. Diletakkan di akhir tabel; `kMaxParams` = 48.

Mode LOITER (`ModeLoiter`, custom_mode 12 = LOITER di dropdown Mission Planner):
saat dimasuki, pusat putaran = posisi GPS saat itu dan ketinggian = ketinggian
saat itu. Kendali attitude/throttle sama dengan AUTO. Belum ada slot saklar RC
untuk LOITER; pilih dari Mission Planner.

## Data galat radius loiter (tlog) (2026-10-19)

Saat wahana berputar (mode LOITER, item LOITER_* di AUTO, atau loiter akhir
misi/RTL setelah tertangkap), firmware mengirim 5 Hz `NAMED_VALUE_FLOAT`:
`LOIT_RAD` (radius perintah, m), `LOIT_DST` (jarak ke pusat, m),
`LOIT_ERR` (= DST - RAD; + di luar lingkaran, - di dalam), `LOIT_DIR`
(+1 CW / -1 CCW). Ikut terekam di tlog Mission Planner. `wp_dist` di
NAV_CONTROLLER_OUTPUT juga berisi jarak ke pusat putaran.

## Batas bank & data loiter tambahan (2026-10-19)

- `ROLL_LIMIT_DEG` (default 45, 20-60): batas |nav_roll| AUTO ke waypoint.
- `LOIT_BANK_DEG` (default 45, 10-60): batas bank khusus loiter; dipakai membatasi
  nav_roll saat berputar dan oleh L1 untuk radius minimum loiter. Keduanya LIVE.
- Tlog (5 Hz, saat berputar) kini juga memuat `LOIT_N`/`LOIT_E` (posisi wahana
  relatif pusat, m) dan `LOIT_BNK` (batas bank yang dipakai).

## Evaluasi terbang 2026-10-19 C (loiter gagal) dan perbaikannya (2026-10-20)

Gejala: mode LOITER tidak pernah muncul di tlog, lintasan berupa loop raksasa
(deviasi sampai ~325 m), tidak melingkar.

1. **LOITER dari Mission Planner diabaikan.** Dropdown mode / tombol Set Mode
   Mission Planner mengirim pesan `SET_MODE` (#11), bukan `COMMAND_LONG
   DO_SET_MODE`; firmware hanya menangani yang kedua, sehingga pilihan mode
   dibuang diam-diam (HEARTBEAT tidak pernah melaporkan custom_mode 12).
   Perbaikan: `Mavlink::handleSetMode()` + helper bersama
   `Mavlink::applyModeRequest()`.
2. **`nav_roll` terpotong tepat ±20 deg** (= batas bawah `ROLL_LIMIT_DEG`)
   selama AUTO. Radius belok minimum R = V^2 / (g tan(bank)); dengan bank 20
   deg dan groundspeed 12-31 m/s (angin ~9 m/s) R = 110-270 m, jadi lingkaran
   50 m mustahil. Simulasi algoritma L1 loiter firmware
   (`updateLoiter`, bank 20, angin 9 m/s, R 50 m): jarak ke pusat 44-380 m --
   cocok dengan log. Dengan bank 45 tanpa angin: 51 m stabil.
3. **L1 memakai radius yang tidak bisa diterbangi.** `loiterRadius()` (ikut
   ArduPlane) membiarkan radius perintah < radius minimum, sehingga nav_roll
   jenuh terus. Sekarang radius dinaikkan ke V^2/(g tan(LOIT_BANK_DEG)), dan
   `LOIT_RAD`/`LOIT_ERR` memakai radius efektif ini
   (`L1Controller::loiterRadiusUsed()`).

Setelan yang disarankan sebelum uji loiter berikutnya: `ROLL_LIMIT_DEG` 45,
`LOIT_BANK_DEG` 45, `LOIT_ROLL_SIGN` 1, `WP_LOITER_RAD` -80 s.d. -100 bila
berangin (simulasi angin 9 m/s, bank 45: R 80 -> 80-96 m, R 100 -> 100-110 m).
`analyze_tlog_l1_auto.py` kini mengenali mode LOITER/RTL, membaca
`ROLL_LIMIT_DEG` dari log, dan membuat `12_loiter.png` dari `LOIT_*`.

### Terbang 2026-10-19 16-12 (folder `flight_2026-10-19_d`, `analyze_tlog_loiter.py`)

Misi: WP1-4 biasa, WP5 item LOITER (CW, `WP_LOITER_RAD` +65, `LOIT_BANK_DEG` 40).
Item LOITER di AUTO tetap melaporkan mode AUTO (seperti ArduPlane); deteksi dari
`MISSION_CURRENT`=5 dan `LOIT_*`. Hasil: hanya ~23 s berputar, arah AKTUAL CCW
(-268 deg) padahal perintah CW, jarak ke pusat 40 -> 218 m (divergen).

- `LOIT_ROLL_SIGN = -1` membalik roll loiter: L1 meminta belok kanan (CW),
  wahana belok kiri. Koreksi radial jadi terbalik -> tidak stabil. Harus 1.
- `ROLL_LIMIT_DEG = 20`: setelah WP4 wahana butuh putar balik ke WP5 dengan
  bank 20 deg pada airspeed 27 m/s -> radius ~200 m, wp_dist sampai 532 m.
- Airspeed aktual 27-28 m/s (throttle manual) vs `AIRSPEED_CRUISE` 18: radius
  minimum pada bank 40 = 93 m > 65 m yang diminta (nav_roll jenuh 60% waktu).
  L1 kini memakai airspeed terukur (low-pass) untuk radius minimum.
- Angin ~7-8 m/s (groundspeed 15-30 m/s pada airspeed konstan 27-28).

## Default baru + migrasi `PARAM_REV`, dan LOITER di heartbeat (2026-10-20)

Default: `ROLL_LIMIT_DEG` 40, `LOIT_BANK_DEG` 35, `WP_LOITER_RAD` +100 (CW),
`LOIT_ROLL_SIGN` 1. Bank loiter dibuat lebih kecil dari bank WP karena loiter
adalah belok TERUS-MENERUS (faktor beban 1.22 g di 35 deg vs 1.31 g di 40 deg;
kecepatan stall naik 10% vs 14%) pada wahana bentang sayap besar.

Default hanya berlaku untuk param baru, jadi board yang sudah menyimpan nilai
lama perlu migrasi: param baru `PARAM_REV` (akhir tabel). EEPROM yang belum
memuatnya menjalankan `Params::applyRev1Migration()` SEKALI saat boot: keempat
param di atas di-set ke default baru lalu disimpan. Setelah itu nilai dari
Mission Planner dihormati seperti biasa.

Heartbeat: selama AUTO menjalankan item LOITER_* (sesudah tertangkap) atau
loiter akhir misi, custom_mode dilaporkan 12 (LOITER) dan STATUSTEXT
"AUTO: LOITER WP n" / "AUTO: LOITER selesai" dikirim. Mode internal tetap
AUTO (`Mavlink::autoIsCircling`), jadi misi lanjut sendiri setelah loiter
selesai dan saklar RC tidak terpengaruh. Memilih AUTO dari GCS saat itu
melanjutkan WP yang sama (hitungan putaran LOITER_TURNS mulai ulang).

## Auto throttle TECS (`FC_AUTO_THROTTLE_ENABLE`, 2026-10-20)

Saklar compile-time di `include/FC_Config.h` (default 0 = throttle manual).
Bila 1, AUTO/LOITER/RTL memakai `Tecs::throttleDemand()` (TECS_THR_MIN..MAX,
target `AIRSPEED_CRUISE`); FBWA/MANUAL/GUIDED tetap manual. Penjaga
(`autoThrottleBlockReason()` di `FixedWingModes.cpp`) mengembalikan ke
throttle manual bila sensor airspeed tidak sehat, `TECS_THR_MAX` <
`TECS_THR_MIN` + 10, atau urutan `MIN_AIRSPEED` < `AIRSPEED_CRUISE` <
`MAX_AIRSPEED` salah. GCS menerima STATUSTEXT "AUTO THR: ON (TECS)",
"AUTO THR: manual", atau "AUTO THR OFF: <alasan>".

Migrasi `PARAM_REV` rev 1 juga mengembalikan TECS_THR_MIN/MAX (30/85) dan
MIN/MAX_AIRSPEED (14/22) ke default: di tlog 2026-10-19 tersimpan 20/10 dan
5/15 (MAX < CRUISE 18).

PERINGATAN: di darat airspeed ~0 -> TECS underspeed -> throttle = TECS_THR_MAX.
Jangan masuk AUTO/LOITER di darat dengan propeler terpasang.

## `FUZZY_ENABLE`, `DIST_ENABLE`, dan param L1 live (2026-10-25)

Dua param baru di **akhir** tabel, sesudah `PARAM_REV` (urutan param loiter
tidak berubah; total 47 param, `kMaxParams` 48; EEPROM lama otomatis memberi
nilai default pada slot baru):

| Param | Default | Fungsi |
|---|---|---|
| `FUZZY_ENABLE` | `FC_FUZZY_L1_ENABLE` (0) | 1 = fuzzy self-tuning periode L1; 0 = L1 periode tetap = `NAVL1_PERIOD` |
| `DIST_ENABLE` | 0 | 1 = `DisturbanceInjector` boleh menyuntik gangguan roll di AUTO (`docs/disturbance-injection.md`) |

`Mavlink::handleParamSet()` meneruskan perubahan langsung (di bawah
`ControlLock`) ke objek yang memakainya, tanpa reboot:
`NAVL1_PERIOD` → `L1Controller::setPeriod()` + `FuzzyL1Tuner::setBasePeriod()`;
`NAVL1_DAMPING` / `NAVL1_XTRACK_I` → setter L1; `FUZZY_ENABLE` →
`FuzzyL1Tuner::applyEnabled()`; `FUZZY_MIN_PER` / `FUZZY_MAX_PER` →
`setPeriodLimits()`; `DIST_ENABLE` dibaca langsung oleh `DisturbanceInjector`.
Jika `ControlLock` tidak didapat dalam 30 ms, nilai tetap tersimpan di EEPROM
dan GCS mendapat peringatan "PARAM: busy, applies after reboot".

Hasil nyata tercatat di log SD (`# EFFECTIVE`, event `PARAM`, kolom
`l1_period_s`/`fuzzy_on`), jadi tidak perlu lagi memotret layar parameter.

### Tambahan 2026-10-26: `DIST_US`, `DIST_MS`, `kMaxParams` 64

Dua param lagi di akhir tabel, sesudah `DIST_ENABLE` (total 49 param, jadi
`kMaxParams` dinaikkan dari 48 ke **64**; tanpa itu `DIST_MS` terbuang diam-diam
oleh `Params::add()`). Rentang EEPROM `Params` maksimum sekarang `[1000, 1260)`,
masih jauh sebelum blok IMU di 2000.

| Param | Default | Fungsi |
|---|---|---|
| `DIST_US` | 250 | besar pulsa gangguan CH7, µs bertanda (+ = roll kanan), ±400 |
| `DIST_MS` | 2000 | lama pulsa gangguan CH7, ms, maks 3000 |

Keduanya dibaca langsung (live) oleh `DisturbanceInjector` saat pulsa mulai.
`DIST_ENABLE` sekarang berlaku di AUTO **dan LOITER**. Default
`FUZZY_MIN_PER`/`FUZZY_MAX_PER` diubah ke 12/16 s (hanya untuk EEPROM baru;
EEPROM lama tetap memakai nilai tersimpan, set manual dari Mission Planner).
