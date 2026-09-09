# Logging IMU/altitude lewat USB Serial (fc::DataLogger)

> **Status 2026-08-22: aktif, tapi bergantian dengan MAVLink** (bukan lagi
> port kedua yang selalu terpisah seperti versi lama dokumen ini). Teensy
> 4.1 tidak punya kombinasi USB "Dual Serial + MTP" (lihat
> `docs/sd-logger-mtp.md`), jadi `SerialUSB1` yang dulu didedikasikan untuk
> `fc::DataLogger` sudah tidak ada di build ini. `DataLogger` sekarang
> dipakai lewat port `Serial` primer, hanya saat `FC_DEBUG_SERIAL_ENABLE=1`
> (`include/FC_Config.h`) -- lihat bagian "Kenapa satu port bergantian" di
> bawah.

Modul CSV bench-logging untuk skripsi: satu baris per sampel berisi seluruh
data IMU mentah (accel/gyro/mag/linear-accel/gravity) plus roll/pitch/yaw
hasil fusion on-chip BNO055, altitude Kalman + complementary filter. Dipakai
untuk diagnosis axis-convention/interferensi magnet tanpa perlu menarik SD
card (lihat `docs/imu-bno055.md`) -- baris log CSV yang sudah dianalisis di
percakapan ini (`AKUSISI-IMU ....txt`) persis format yang dihasilkan modul
ini.

## Kenapa satu port bergantian, bukan dua port terpisah

Sebelumnya (`USB_DUAL_SERIAL`) `Serial` murni MAVLink dan `SerialUSB1`
murni CSV logger, dua COM port berbeda. Setelah MTP (SD card browsable
lewat USB, `docs/sd-logger-mtp.md`) ditambahkan, Teensy 4.1 tidak lagi
punya `SerialUSB1` tersedia (tidak ada kombinasi USB personality "Dual
Serial + MTP" pada board ini) -- hanya satu CDC port (`Serial`) plus MTP.

Solusinya: `FC_DEBUG_SERIAL_ENABLE` (compile-time, `FC_Config.h`) memilih
salah satu peran untuk `Serial`:

- **`1`**: `Serial` jadi teks CSV PuTTY-readable (`fc::DataLogger` lewat
  `g_usbLogger`). `MAVLINK_USB_ENABLE_RX/TX` otomatis dipaksa `0` supaya
  MAVLink tidak ikut menulis/membaca port yang sama dan merusak parsing di
  kedua sisi. Telemetry radio (`Serial2`) dan companion computer
  (`Serial7`) tetap jalan seperti biasa -- GCS tetap bisa connect lewat
  radio telemetry, cuma USB langsung yang berubah peran.
- **`0`** (default): `Serial` kembali jadi MAVLink seperti biasa (Mission
  Planner/QGC connect di sini). Boot/status diagnostic (`reportStatus()` di
  `main.cpp`) keluar sebagai MAVLink STATUSTEXT, bukan teks biasa.

Flip satu baris di `FC_Config.h` dan reflash untuk berpindah mode -- lihat
`docs/sd-logger-mtp.md` untuk konteks selengkapnya soal selector ini.

## Struktur modul

```text
include/communication/DataLogger.h   Kontrak class DataLogger
src/communication/DataLogger.cpp     Implementasi (format CSV, snprintf ke buffer tetap)
```

## Format CSV

```text
imu_seq,baro_seq,timestamp_us,roll_deg,pitch_deg,yaw_deg,heading_deg,altitude_m,raw_altitude_m,climb_rate_mps,pressure_pa,temperature_c,comp_altitude_m,comp_climb_rate_mps,comp_accel_up_mss,accel_x_mss,accel_y_mss,accel_z_mss,gyro_x_dps,gyro_y_dps,gyro_z_dps,mag_x_ut,mag_y_ut,mag_z_ut,linacc_x_mss,linacc_y_mss,linacc_z_mss,grav_x_mss,grav_y_mss,grav_z_mss,calib_sys,calib_gyro,calib_accel,calib_mag,imu_valid,baro_valid
```

`calib_sys/gyro/accel/mag` (2026-08-22) -- status kalibrasi on-chip BNO055,
0-3 masing-masing (lihat `docs/imu-bno055.md`'s bagian "Calibrate Level
hilang lagi setelah beberapa detik"). Diambil dari `Imu::calibration()`,
di-refresh 1 Hz oleh `updateCalibration()` di `taskMavlink` -- jadi nilai
ini berulang di beberapa baris berturut-turut antar polling, itu wajar.
**"Calibrate Level" hanya boleh dipercaya kalau keempat kolom ini sudah
3 semua** -- kalau belum, baseline roll/pitch masih bergeser sendiri di
background walau sudah di-trim.

- `roll_deg`/`pitch_deg`/`yaw_deg`/`heading_deg` -- Opsi 1, fusion on-chip
  BNO055 (register Euler, lihat `docs/imu-bno055.md`).
- `altitude_m`/`raw_altitude_m`/`climb_rate_mps` -- Opsi 1 altitude (Kalman
  onboard + nilai mentah pra-filter, lihat `docs/barometer-ms5611.md`).
- `comp_altitude_m`/`comp_climb_rate_mps`/`comp_accel_up_mss` -- Opsi 2
  altitude (`fc::AltitudeComplementaryFilter`, lihat
  `docs/altitude-complementary-filter.md`), murni perbandingan.
- `accel_x/y/z_mss`, `gyro_x/y/z_dps`, `mag_x/y/z_ut` -- pembacaan mentah
  BNO055 (frame chip, bukan hasil rotasi/kalibrasi tambahan) -- input yang
  sama yang dipakai `fc::AttitudeMahonyFilter` (`docs/attitude-mahony-filter.md`)
  dan yang dipakai untuk analisis interferensi magnet.
- `linacc_x/y/z_mss` -- percepatan linear (gravitasi sudah dikurangi
  on-chip).
- `grav_x/y/z_mss` -- vektor gravitasi hasil fusion on-chip (`acceleration_mss
  ~= linear_acceleration_mss + gravity_mss`, berguna sebagai cross-check
  konsistensi internal fusion BNO055).
- `imu_valid`/`baro_valid` -- `1`/`0`.

Baris dilewati kalau `imu.valid` dan `baro.valid` dua-duanya `false`.
`timestamp_us` mengambil dari IMU kalau valid, jatuh ke baro kalau tidak.

## Boot diagnostic: axis remap yang benar-benar terpasang

`setup()` mencetak satu baris begitu `g_imu.initialized()`:

```text
[IMU] BNO055 axis_map=0x24 sign_x=0 sign_y=0 sign_z=0
```

Nilai ini berasal dari `FC_BNO055_AXIS_MAP_CONFIG`/`FC_BNO055_AXIS_SIGN_X/Y/Z`
(`FC_Config.h`) -- yang benar-benar diterapkan ke register `AXIS_MAP_CONFIG`/
`AXIS_MAP_SIGN` chip di `Imu::configureSensor()`, bukan cuma dicatat.
Default `DEFAULT_AXIS`/`0`/`0`/`0` = orientasi P1 pabrik, tidak diremap.
Baris ini membuat setiap log CSV self-describing: kalau Anda mengubah remap
untuk eksperimen, log yang dihasilkan tercatat memakai konfigurasi yang
mana, tanpa perlu mengingat-ingat firmware mana yang di-flash saat itu.

## Rate: 10 Hz

`kUsbLogPeriodMs = 100` (`FC_Config.h`) -- cukup untuk melihat tren
attitude/altitude tanpa membanjiri terminal serial atau bersaing CPU
dengan task IMU/control 200 Hz. Task IMU sendiri (`taskImu`) tidak
terpengaruh; `taskUsbLog` hanya membaca `g_imu.data()`/`g_baro.data()`/
`g_altComplementary.data()` yang sudah ada, tidak memicu pembacaan sensor
tambahan.

## Kepemilikan dan penjadwalan

```cpp
fc::DataLogger g_usbLogger(Serial);

void setup() {
    Serial.begin(kUsbBaud);
    // ...
#if FC_DEBUG_SERIAL_ENABLE
    g_usbLogger.begin();  // menulis header CSV
#endif
}

#if FC_DEBUG_SERIAL_ENABLE
void taskUsbLog(void*) {
    for (;;) {
        g_usbLogger.logAttitudeAltitude(g_imu.data(), g_baro.data(), g_altComplementary.data());
        vTaskDelay(pdMS_TO_TICKS(kUsbLogPeriodMs));
    }
}
#endif
```

Task dibuat (`xTaskCreate`) hanya kalau `FC_DEBUG_SERIAL_ENABLE=1` --
konsisten dengan pola exclude-at-compile-time yang sama dipakai
`FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE` (`docs/attitude-mahony-filter.md`).

## Membaca log di PC

Konfigurasi PuTTY:

- Connection type: `Serial`
- Serial line: COM port Teensy yang muncul di Windows
- Speed: `115200`
- Logging: `Session logging -> All session output`

Atau redirect langsung ke file lewat PlatformIO:

```bash
pio device monitor -p COMx -b 115200 --raw > imu_log.csv
```

Header CSV ditulis sekali saat boot oleh `DataLogger::begin()`.

## Bekas SD card logging terpisah

`fc::SdLogger` (SD card, `docs/sd-logger-mtp.md`) tetap berjalan independen
dari modul ini di `FC_DEBUG_SERIAL_ENABLE` apa pun -- dua jalur logging
berbeda tujuan: `SdLogger` untuk data terbang tanpa PC terhubung terus,
`DataLogger` (dokumen ini) untuk bench test dengan PC/PuTTY langsung
terhubung.
