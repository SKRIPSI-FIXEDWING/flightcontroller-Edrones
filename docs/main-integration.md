# Integrasi main.cpp

Titik masuk yang menyatukan seluruh modul dari Fase 1-7 menjadi satu firmware
FreeRTOS. Ini bukan port 1:1 dari `main.cpp` lama (yang mem-branch runtime
antara FW/VTOL) -- ini **FW-only secara compile-time**: tidak ada cabang
`model_uav`, tidak ada task/kode copter yang ikut ter-compile sama sekali.

## Backend GNSS yang aktif: `GnssNmea` (SE100)

Dikonfirmasi bersama Anda: `Here4GnssReceiver` (DroneCAN) dan `GnssUbx` (M10)
sudah lengkap dan siap pakai, tapi **belum diverifikasi hardware** (lihat
`docs/gnss-here4-dronecan.md` -- signature DSDL placeholder, node-ID
allocation belum dites). `GnssNmea` dipilih sebagai default karena siap pakai
tanpa perangkat tambahan dan sesuai spesifikasi hardware di README asli.

**Untuk beralih ke Here4 atau M10** setelah diverifikasi: ganti deklarasi
`fc::GnssNmea g_gnss(Serial1);` di `src/main.cpp` dengan
`fc::Here4GnssReceiver g_gnss;` atau `fc::GnssUbx g_gnss(Wire);` -- seluruh
kode lain (`VehicleContext`, `Navigation`, `Mavlink`) sudah bergantung pada
`fc::GnssFixData` yang seragam, jadi tidak perlu perubahan lain di luar baris
deklarasi dan pemanggilan `g_gnss.update()`/`g_gnss.begin()` di task GPS.

## Kenapa sebagian objek adalah pointer global, bukan objek biasa

`L1Controller`, `FuzzyL1Tuner`, `Tecs`, `Navigation`, `AttitudeController`,
dan `VehicleContext` sendiri dibuat lewat `new` di dalam `setup()`, bukan
sebagai objek global biasa. Alasannya: `L1ControllerConfig`/`FuzzyL1TunerConfig`/
`TecsConfig` diisi dari EEPROM oleh `Params::load()`, yang hanya bisa
dipanggil di dalam `setup()` (butuh `Serial`/`EEPROM` yang belum siap saat
inisialisasi statis C++). Kalau objek-objek ini dibuat sebagai variabel
global biasa, konstruktornya akan berjalan **sebelum** `setup()` -- dengan
config default yang belum sempat di-load dari EEPROM sama sekali. Pola
`new` di `setup()` ini sudah dipakai di codebase lama untuk objek eFLL
(`Fuzzy* f = new Fuzzy();`), jadi bukan pola asing untuk proyek ini.

## Tata letak task FreeRTOS

| Task | Periode | Prioritas | Fungsi |
| --- | --- | --- | --- |
| IMU | 5 ms (200 Hz) | 6 (tertinggi) | `imu.update()` |
| Control | 5 ms (200 Hz) | 5 | Refresh snapshot GNSS/dt/waktu, ganti mode dari RC, `modes.update()` |
| Mavlink | 20 ms | 5 | `handlePorts()` + `update()` (masing-masing punya rate-limit internal per jenis pesan) + `battery.update()` (10 Hz) |
| Radio | 10 ms (100 Hz) | 5 | `radio.update()` |
| GPS | 10 ms (100 Hz) | 4 | `gnss.update()` |
| Barometer | 10 ms (100 Hz) | 3 | `baro.update()` |
| Airspeed | 50 ms (20 Hz) | 3 | `airspeed.update()` |
| Buzzer | 10 ms (100 Hz) | 3 | `buzzer.update(radio.armed())` |
| BaroLog | 50 ms (20 Hz) | 2 (terendah) | `DataLogger::logBarometer()` ke `SerialUSB1` (lihat `docs/data-logger-usb.md`) |

Task `Print`/`Print_To_Raspi`/`Print_telem`/`voltage` (terpisah) dari program
lama **dihapus** -- masing-masing menggantikan debug print USB murni (`Print`),
superseded oleh telemetry MAVLink terstruktur (`Print_To_Raspi`), atau
menyetel gain yang salah/rusak (`Print_telem`, lihat `docs/mavlink.md`).
`voltage` sekarang tergabung ke dalam task Mavlink (`battery.update()` di 10 Hz)
alih-alih task terpisah, karena satu-satunya konsumennya (`sendBatteryStatus`)
memang ada di situ.

## Mode RC: pemetaan `mode_now` -> `ModeId`

Persis sama seperti `Mode_Manager.h::setup_mode()`'s cabang
`MODEL_UAV_FIXEDWING` lama: `mode_now` 1=Manual, 2=Fbwa, 3=Auto, 4=Guided.
Nilai lain (termasuk 5, yang di program lama tidak pernah dipetakan untuk FW)
tidak melakukan apa pun -- tetap di mode aktif saat ini.

## Waypoint default

`seedDefaultMission()` menyediakan satu waypoint (rumah, `0,0`) hanya supaya
AUTO/GUIDED punya tujuan saat EEPROM belum berisi misi -- **bukan** pengganti
koordinat lapangan terbang milik tim lama (`wp_setup()` lama berisi koordinat
GPS spesifik lokasi latihan mereka, tidak relevan untuk lokasi Anda). Isi misi
sesungguhnya lewat upload MAVLink (Mission Planner/QGC) sebelum uji AUTO.

## Verifikasi

`pio run -e teensy41` -- build+link bersih, ~127 KB flash / 8 MB (jauh dari
batas), tidak ada simbol copter/VTOL aktif di `include/`/`src/` (hanya
disebutkan di komentar migrasi yang menjelaskan penghapusannya). **Verifikasi
di atas HANYA compile-time** -- belum ada uji hardware/terbang; lihat daftar
"perlu diverifikasi sebelum terbang" di `docs/attitude-lqr.md` dan
`docs/gnss-here4-dronecan.md` sebelum uji nyata.
