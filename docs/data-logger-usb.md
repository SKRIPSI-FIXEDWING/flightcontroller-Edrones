# Data logging barometer MS5611 lewat USB (fc::DataLogger)

Modul baru untuk keperluan pengambilan data skripsi: log CSV mentah dari
`fc::Barometer` (MS5611), dikirim lewat port USB CDC kedua Teensy 4.1,
dijadwalkan sebagai task FreeRTOS tersendiri.

## Kenapa bukan lewat `Serial` yang sudah ada

Port `Serial` (USB pertama) sudah dipakai penuh oleh `fc::Mavlink` untuk
protokol biner MAVLink ke GCS (`Mavlink.cpp`'s `mavWrite()`/`handlePorts()`).
Menulis teks CSV ke port yang sama akan bercampur dengan byte stream MAVLink
dan merusak parsing di kedua sisi (GCS gagal decode heartbeat/telemetry, baris
CSV juga akan terpotong oleh byte biner).

Solusinya: Teensy 4.1 mendukung lebih dari satu port USB CDC sekaligus
(`board_build.usb_type = USB_DUAL_SERIAL` di `platformio.ini`), yang
menyediakan `SerialUSB1` selain `Serial` -- dua port USB terpisah muncul di PC
sebagai dua COM port berbeda, tanpa hardware tambahan. `Serial` tetap murni
MAVLink; `SerialUSB1` didedikasikan untuk `fc::DataLogger`.

## Struktur modul

```text
include/communication/DataLogger.h   Kontrak class DataLogger
src/communication/DataLogger.cpp     Implementasi (format CSV, snprintf ke buffer tetap)
```

## Format CSV

```text
seq,timestamp_us,pressure_pa,temperature_c,altitude_m,raw_altitude_m,climb_rate_mps,comp_altitude_m,comp_climb_rate_mps
1,123456,101325.00,27.30,0.120,0.115,0.010,0.118,0.008
```

`altitude_m` sudah difilter Kalman onboard/Opsi 1 (dipakai controller);
`raw_altitude_m` adalah nilai yang sama sebelum filter -- disertakan khusus
supaya tuning `kalman_measurement_noise_r`/`kalman_process_noise_q`
(`BarometerConfig`, lihat `docs/barometer-ms5611.md`) bisa dihitung dari
noise sensor yang sesungguhnya, bukan noise yang sudah dihaluskan.
`comp_altitude_m`/`comp_climb_rate_mps` adalah Opsi 2, hasil complementary
filter baro+accel BNO055 yang berjalan paralel (lihat
`docs/altitude-complementary-filter.md`) -- juga tidak dipakai controller,
murni untuk perbandingan.

Header ditulis sekali di `DataLogger::begin()`. Baris invalid
(`BarometerData::valid == false`, misalnya sebelum `Barometer::begin()`
sukses) tidak ditulis.

## Rate: 20 Hz, terpisah dari rate sampling sensor

`taskBaro` tetap sampling MS5611 di 100 Hz nominal (`kBaroPeriodMs`, dipakai
Kalman filter/estimation) -- ini tidak berubah. `taskBaroLog` (task baru,
prioritas 2, paling rendah) hanya membaca `g_baro.data()` dan menulis CSV di
20 Hz (`kBaroLogPeriodMs = 50`), independen dari rate sampling.

Alasan 20 Hz: ArduPilot sendiri menjadwalkan `Baro::update()` di scheduler
task table Plane/Copter pada 10 Hz -- tren tekanan/altitude/climb-rate tidak
berubah cukup cepat untuk butuh resolusi lebih tinggi, dan noise-level
karakterisasi getaran (yang butuh rate tinggi) bukan tujuan log ini. 20 Hz
dipilih sebagai dua kali lipat referensi tersebut, memberi resolusi lebih
halus untuk validasi altitude/climb-rate di skripsi, sambil tetap jauh di
bawah batas bandwidth USB CDC untuk baris CSV ~40 byte.

## Kepemilikan dan penjadwalan

```cpp
#include "communication/DataLogger.h"

fc::DataLogger baro_logger(SerialUSB1);

void setup() {
    SerialUSB1.begin(115200);
    baro_logger.begin();  // menulis header CSV
}

void taskBaroLog(void*) {
    for (;;) {
        baro_logger.logBarometer(g_baro.data());
        vTaskDelay(pdMS_TO_TICKS(50));  // 20 Hz
    }
}
```

## Membaca log di PC

`SerialUSB1` muncul sebagai COM port terpisah dari `Serial` (mis. `Serial`
jadi `COM5` untuk Mission Planner/QGC, `SerialUSB1` jadi `COM6` untuk logger).
Buka `COM6` di Serial Monitor/PuTTY/`pio device monitor -p COMx`, atau redirect
langsung ke file `.csv`:

```bash
pio device monitor -p COM6 -b 115200 --raw > baro_log.csv
```

Baud rate untuk USB CDC bersifat kosmetik (koneksi sudah full-speed lewat USB,
bukan UART); nilai apa pun umumnya diterima, tapi disamakan dengan
`kUsbBaud` (115200) di `main.cpp` untuk konsistensi.

## Tidak dimigrasikan / bukan bagian modul ini

Log ke SD card (`lib/freertos-teensy-11.0.1_v1/example/sdfat` tersedia di
repo tapi tidak dipakai) sengaja tidak diimplementasikan di sini -- lingkup
permintaan ini murni logging lewat USB serial. Jika nanti dibutuhkan logging
persisten tanpa PC yang terhubung terus-menerus, itu modul terpisah
(`storage/`), bukan perluasan `DataLogger` ini.
