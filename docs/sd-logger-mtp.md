# SD card logging + akses MTP (fc::SdLogger)

Ditambahkan 2026-08-19: log CSV flight-data (roll/pitch/yaw/altitude/mode/
armed/channel 1-5) ke microSD bawaan Teensy 4.1 (`BUILTIN_SDCARD`), dan SD
card itu bisa langsung dibuka di File Explorer/file manager PC begitu kabel
USB Teensy dicolok -- tanpa cabut kartu -- lewat protokol MTP.

## Kenapa menggantikan `fc::DataLogger`/`SerialUSB1`, bukan tambahan

Teensy 4.1 tidak punya kombinasi USB "Dual Serial + MTP" (dicek langsung ke
`boards.txt` core-nya, menu `teensy41.menu.usb.*`) -- pilihannya cuma salah
satu:

- `USB_DUAL_SERIAL` (kondisi sebelumnya): `Serial` (MAVLink) + `SerialUSB1`
  (live CSV barometer, `fc::DataLogger`), TANPA MTP.
- `USB_MTPDISK_SERIAL` (kondisi sekarang): `Serial` (MAVLink) + MTP disk,
  TANPA `SerialUSB1`.

Karena permintaannya eksplisit "colok kabel, SD kebaca di file manager", yang
dipilih adalah `USB_MTPDISK_SERIAL`. Data yang dulu di-live-stream lewat
`SerialUSB1` (perbandingan altitude Kalman vs complementary filter) sekarang
sudah tidak di-log sama sekali oleh build aktif -- kalau butuh lagi, itu bisa
ditambahkan sebagai kolom baru di `SdLogger`, bukan mengaktifkan
`SerialUSB1` lagi (karena tidak mungkin coexist dengan MTP di board ini).
`fc::DataLogger` sendiri TIDAK dihapus dari source tree, lihat
`docs/data-logger-usb.md`'s status note.

## Struktur modul

```text
include/communication/SdLogger.h   Kontrak class SdLogger
src/communication/SdLogger.cpp     Implementasi (buffer RAM, mount/retry, penamaan file, CSV)
include/FC_Config.h                FC_SD_LOG_ENABLE / FC_SD_MTP_ENABLE + periode & ukuran buffer
```

## Arsitektur non-blocking (dibangun ulang 2026-10-25)

Logging SD sempat gagal di bench. Penyebab di versi lama:

1. **Kartu diakses dua task sekaligus.** `taskSdLog` (tulis) dan `taskMtp`
   (`MTP.loop()`) sama-sama memakai SdFat, yang TIDAK thread-safe (satu
   sector cache bersama). Saat PC terhubung, Windows menanyakan sisa
   kapasitas -> `SD.usedSize()` -> `freeClusterCount()` membaca SELURUH FAT
   (beberapa detik di kartu 32 GB FAT32). Kalau itu bertabrakan dengan
   write logger, state file rusak dan write gagal.
2. **Task SD berprioritas 2, sama dengan task lain, dan
   `configUSE_TIME_SLICING = 0`.** Tanpa time slicing, task yang sedang
   busy-wait (driver SDIO mode FIFO menunggu kartu, atau scan FAT di atas)
   memonopoli prioritasnya sampai selesai.
3. **SD hanya di-mount sekali di `setup()`.** Kartu yang lambat siap atau
   baru dimasukkan setelah boot tidak pernah dicoba lagi.

Desain baru, satu-satunya pemilik kartu adalah `taskSdWriter`:

```text
taskSdLog (prio 2, 20 Hz)   FlightLog::step(): 1 baris CSV (+ header/event/snapshot bila ada) -> stream buffer RAM (64 KB), timeout 0
                            buffer penuh = baris dibuang + dihitung, TIDAK pernah menunggu
taskSdWriter (prio 1, 50 Hz) satu-satunya yang menyentuh kartu:
                            mount (retry tiap 3 s kalau gagal/tidak ada kartu) -> buka LOGnnn.CSV
                            -> kuras buffer ke file -> flush tiap 1 s DAN tepat setelah DISARM
                            write error (kartu dicabut) -> tutup, remount, file baru
taskMtp (prio 1, 100 Hz)    MTP.loop() hanya sambil memegang mutex SD yang sama,
                            dan DIJEDA selama ARMED (PC tidak bisa merebut kartu saat terbang)
```

**Memori:** buffer 64 KB serta stack `taskSdLog` dan `taskSdWriter` dialokasikan STATIS di
RAM2 (`DMAMEM`). Heap `malloc` di port FreeRTOS ini (termasuk semua stack
`xTaskCreate()`) diambil dari RAM1, yang sudah hampir penuh. Versi pertama
rebuild ini (2026-10-25) mengalokasikan buffer dari heap, sehingga
`vApplicationMallocFailedHook` terpicu: LED 13 berkedip 2x berulang dan
tidak ada task yang jalan. Kode kedip LED 13 dari FreeRTOS: 1x = assert,
2x = malloc gagal (RAM habis), 3x = stack overflow, 4x = hard fault.

Prioritas 1 = di bawah semua task terbang (IMU 6, Control/Mavlink/Radio 5,
GPS 4, Baro/Airspeed 3). Kartu yang macet ratusan ms hanya menahan dirinya
sendiri, MTP, dan LED. `setup()` tidak menyentuh kartu sama sekali.

Status SD dilaporkan lewat `reportStatus()` dari `taskMavlink` (~1 s setelah
boot): Messages tab Mission Planner, atau PuTTY bila
`FC_DEBUG_SERIAL_ENABLE=1`.

| Pesan | Arti |
|---|---|
| `[SD] logging LOG012.CSV 30436MB FAT32` | OK, sedang merekam ke file itu |
| `[SD] no card/mount fail err=0x..` | kartu tidak terbaca; dicoba lagi tiap 3 s |
| `[SD] log renamed 2026-10-29_08-58-36.CSV` | waktu GPS diterima, file diberi nama tanggal |
| `[SD] rename failed, keep LOG012.CSV` | rename gagal, rekaman tetap jalan dengan nama lama |
| `[SD] cannot create LOG012.CSV` | kartu ter-mount tapi tidak bisa ditulis (lock switch, FAT rusak -> format ulang) |
| `[SD] write error LOG012.CSV, remounting` | write gagal di tengah jalan (kartu longgar/dicabut) |
| `[SD] N rows dropped (card busy)` | buffer RAM sempat penuh (biasanya karena PC sedang membaca kartu lewat MTP saat disarm) |
| `[SD] 999 logs on card, delete old ones` | hapus LOGnnn.CSV lama |

## Format CSV

Sejak 2026-10-25 (format v2) isi file log disusun oleh `fc::FlightLog`
(`include/communication/FlightLog.h`), bukan lagi oleh `SdLogger`: 57 kolom
data 20 Hz, baris `#EVENT`, header berisi parameter/versi firmware, dan
snapshot parameter + waypoint setiap ARM. Semua kolom, satuan, tanda, dan
cara memakainya ada di **`docs/flight-log-csv.md`**.

`SdLogger` sekarang hanya "pipa" teks yang aman (`writeLine()` tidak pernah
menunggu kartu); ia tidak tahu isi kolom.

## Penamaan file: satu file baru per boot

Writer mencari `LOG001.CSV`, `LOG002.CSV`, ... sampai ketemu nama yang belum
ada di kartu, supaya data penerbangan sebelumnya tidak tertimpa (juga setiap
remount setelah error). Data di-flush ke kartu tiap ~1 s dan tepat setelah
DISARM -- baterai putus mendadak saat terbang kehilangan paling banyak ~1 s
terakhir. Cabut baterai setelah DISARM, bukan saat masih ARMED.

**Nama tanggal (sejak 2026-10-29).** Saat boot tanggal belum diketahui, jadi
file tetap dibuka sebagai `LOGnnn.CSV`. Begitu Here4 mengirim waktu UTC
(Fix2 `gnss_timestamp`, biasanya bersamaan dengan fix pertama), writer
sekali saja flush + close, `SD.rename()` ke tanggal-jam lokal saat file
dibuka (`YYYY-MM-DD_HH-MM-SS.CSV`, WIB = UTC + `kSdLogUtcOffsetMinutes`), lalu
membuka lagi dengan append. Baris yang masuk selama itu menunggu di buffer
RAM, jadi tidak ada yang hilang. Nama sudah dipakai → akhiran `_2` ... `_9`.
Rename gagal → nama `LOGnnn.CSV` dipertahankan. Karena file lama sudah
berganti nama, nomor `LOGnnn` bisa terpakai ulang di boot berikutnya.

## Status output: selector Serial (PuTTY) / MAVLink, bukan buzzer (fix 2026-08-20)

`Serial` cuma satu port, dipakai bareng MAVLink (lihat bagian "Kenapa
menggantikan..." di atas) -- kalau `taskMavlink` jalan bareng, pesan teks
biasa (`Serial.println(...)`) tenggelam/rusak diselang-seling stream biner
MAVLink. Percobaan sebelumnya (2026-08-19) mengonfirmasi status SD lewat
`fc::Buzzer::playPattern()` (1 beep panjang = OK, 6 beep cepat = gagal),
tapi itu cuma bisa membawa 1 bit info (OK/gagal) untuk SATU hal (SD card) --
tidak untuk semua status boot lainnya (IMU/Baro/Airspeed/IMU-calibration),
dan tidak pernah terkonfirmasi bunyinya kedengaran di hardware ini.

Diganti dengan selector `kStatusOutput` di `main.cpp` (konstanta compile-time
-- ubah nilainya lalu re-flash untuk pindah mode, BUKAN prompt runtime):

```cpp
enum class StatusOutput : uint8_t { SerialText = 0, Mavlink = 1 };
constexpr StatusOutput kStatusOutput = StatusOutput::SerialText;
```

- **`SerialText` (0)** -- semua diagnostik boot (SD card, IMU/Baro/Airspeed
  init, IMU calibration restore/save) dicetak sebagai teks biasa ke
  `Serial`. Buka terminal serial (PuTTY, dll) di `kUsbBaud` (115200) dan
  baca langsung. `fc::Mavlink::setUsbEnabled(false)` dipanggil di awal
  `setup()` untuk mode ini, supaya MAVLink TIDAK menulis/membaca apa pun di
  port USB (Telemetry/`Serial2` dan Companion/`Serial7` tetap jalan seperti
  biasa) -- jadi teksnya tidak ketiban traffic biner.
- **`Mavlink` (1)** -- pesan yang sama dikirim sebagai MAVLink `STATUSTEXT`
  (`fc::Mavlink::sendStatusText()`, sekarang public), muncul di tab
  Messages GCS (Mission Planner/QGC). Konfigurasi normal untuk terbang.
  `setUsbEnabled(true)` (default), MAVLink jalan penuh di USB seperti
  sebelumnya.

Semua titik yang dulu manggil `Serial.println`/`Serial.printf` langsung
untuk status boot sekarang lewat satu fungsi `reportStatus(severity, text)`
di `main.cpp` -- satu jalur, bukan tersebar, dan otomatis ikut selector di
atas tanpa perlu diubah satu-satu lagi kalau mode-nya diganti nanti.

## Membaca log di PC

Colok kabel USB Teensy ke laptop seperti biasa. Selain COM port MAVLink
(`Serial`), akan muncul satu drive baru ("SD Card") di File Explorer/file
manager -- buka, cari `LOGnnn.CSV` terbaru, salin/buka langsung seperti
flashdisk biasa. Tidak perlu cabut microSD dari slot Teensy 4.1.

## Library eksternal

`MTP_Teensy` (KurtE/MTP_Teensy, MIT), bukan bagian Teensyduino core --
di-pin ke satu commit di `platformio.ini` `lib_deps`, sama seperti pola
`FlexCAN_T4` di file yang sama. `SD`/`SdFat` sudah bagian dari
`framework-arduinoteensy`, tidak perlu `lib_deps` tambahan.

## Urutan init: SD/MTP sebelum interlock radio (fix 2026-08-20)

`Radio::begin()` (dipanggil di `setup()`) punya `while(armed_)`/
`while(signal_lost_)` TANPA timeout -- sengaja blocking sebagai safety
interlock pre-flight (lihat `docs/radio.md`), tapi ini berarti kalau belum
ada transmitter yang di-bind/sinyalnya belum bagus (mis. saat bench-test
cuma nyolok USB ke laptop buat cek SD card), `setup()` nyangkut selamanya
di situ SEBELUM sempat sampai ke `MTP.begin()`/`g_sdLogger.begin()`.

Akibatnya: Teensy tetap muncul sebagai device "Teensy" di File
Explorer/file manager (itu terjadi di level deskriptor USB, dari
`USB_MTPDISK_SERIAL` di `platformio.ini`, sebelum `setup()` jalan sama
sekali) TAPI tidak pernah benar-benar menjawab request MTP (list storage,
kapasitas, file) -- makanya kelihatan seperti drive kosong tanpa info
GB kosong, dan `SdLogger` tidak pernah sempat `begin()` jadi tidak ada yang
ke-log ke SD card sama sekali.

Fix: blok SD card + MTP (`MTP.begin()`, `g_sdLogger.begin()`,
`MTP.addFilesystem()`) dipindah ke SEBELUM `g_radio.begin()` di
`main.cpp`. Sekarang SD card mount dan MTP jadi responsif duluan,
independen dari lama/tidaknya interlock radio menunggu transmitter.

## Status validasi

- **Tervalidasi di hardware (2026-10-25):** arsitektur non-blocking di atas
  (kartu 32 GB FAT32) menulis `LOG001.CSV` (29,1 KB, format v1 lama) dan
  kartu terbaca di File Explorer lewat MTP.
- **BELUM tervalidasi di hardware:** format v2 (`fc::FlightLog`, 57 kolom,
  `#EVENT`, snapshot ARM). Kode terkompilasi bersih dan jumlah kolom/format
  sudah dicek otomatis, tetapi belum ada file v2 nyata. Langkah uji bangku
  ada di `docs/flight-log-csv.md` ("Uji bangku").
- Pastikan microSD terpasang SEBELUM FC dinyalakan. Kalau tidak terbaca,
  `taskSdWriter` mencoba mount lagi tiap 3 s dan pesan `[SD] ...` muncul di
  Messages GCS (atau PuTTY bila `FC_DEBUG_SERIAL_ENABLE=1`).
- Kalau drive MTP tidak muncul/hang, cabut-colok ulang kabel USB dulu
  sebelum curiga ke firmware -- device MTP butuh enumerasi ulang oleh host.
