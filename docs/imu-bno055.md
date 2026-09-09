# Refactor IMU BNO055

Dokumen ini menjelaskan batas modul, konvensi data, dan migrasi dari program
KHAGESWARA lama. Implementasi baru dikhususkan untuk wahana fixed-wing.

## Struktur modul

```text
include/drivers/Imu.h       Kontrak data dan deklarasi class Imu
src/drivers/Imu.cpp         Implementasi sensor, konversi, dan transport I2C
lib/BNO055/src/BNO055.h     API register Bosch (vendor)
lib/BNO055/src/BNO055.c     Implementasi API register Bosch (vendor)
```

`BNO055_support.h/.cpp` lama tidak digunakan. Bridge Arduino/I2C sudah berada
di `Imu.cpp`, sehingga pilihan bus, pemeriksaan jumlah byte, dan status error
berada di satu tempat. `TeensyThreads`, `I2CDev`, MPU6050, dan magnetometer
HMC5883L juga bukan dependensi modul ini.

## Kepemilikan dan penjadwalan

Driver tidak membuat task dan tidak mendefinisikan global `imu`. Pemilik utama
flight controller membuat tepat satu object `fc::Imu`, memanggil `begin()` saat
inisialisasi, lalu memanggil `update()` dari scheduler 200 Hz.

```cpp
#include "drivers/Imu.h"

fc::Imu imu(Wire);

void setupImu()
{
    if (!imu.begin()) {
        // Masuk ke penanganan pre-arm/failsafe; jangan mulai penerbangan.
    }
}

void updateImu200Hz()
{
    if (imu.update()) {
        const fc::ImuData imu_data = imu.data();
        // Teruskan data ini ke AHRS/control/navigation.
    }
}
```

Apabila pembaruan dan pembacaan dilakukan dari task yang berbeda, pemilik
object wajib melindungi pasangan `update()`/`data()` dengan mutex scheduler.
Driver sengaja tidak bergantung pada FreeRTOS agar tetap dapat diuji dan tidak
mengambil alih kebijakan scheduling.

Kalibrasi cukup dibaca pada task lambat, misalnya 1 Hz:

```cpp
if (imu.updateCalibration()) {
    const fc::ImuCalibration calibration = imu.calibration();
}
```

Tidak ada lagi loop kalibrasi yang memblokir flight loop.

## Data yang dihasilkan

`ImuData` mengikuti data yang sebelumnya disediakan oleh `bno055.h`, tetapi
nama setiap field sekarang menyebutkan unitnya:

| Data | Unit |
| --- | --- |
| `roll_deg`, `pitch_deg`, `yaw_deg`, `heading_deg` | derajat |
| `roll_rad`, `pitch_rad`, `yaw_rad` | radian |
| `angular_rate_dps` | derajat/detik |
| Percepatan | m/s² |
| Waktu sampel | mikrodetik |

Fungsi `rollDegrees()`, `pitchDegrees()`, `yawDegrees()`,
`headingDegrees()`, dan `angularRateDps()` disediakan untuk mempermudah migrasi
controller lama. Kode baru dapat mengambil seluruh hasil sekaligus melalui
`imu.data()`.

Quaternion **bukan** field `ImuData` — lihat `updateQuaternion()`/`quaternion()`
di bagian berikutnya untuk alasannya (biaya I2C di task 200 Hz).

### Sumber data: Euler, quaternion dipoll terpisah

`Imu::readData()` membaca **kedua** register: Euler (`bno055_read_euler_hrp`)
dan quaternion (`bno055_read_quaternion_wxyz`).

`roll_deg`/`pitch_deg`/`yaw_deg`/`heading_deg` — yang dipakai AHRS/control —
bersumber dari register **Euler**, sama seperti implementasi awal modul ini.
Sempat dicoba menurunkan sudut ini dari quaternion (motivasinya: Bosch sendiri
menyatakan output Euler punya bug firmware dan gimbal lock di sekitar pitch
±90°), tetapi percobaan itu dibatalkan setelah dua masalah muncul berurutan
saat bench test nyata:

1. Formula konversi quaternion→Euler ArduPilot (`Quaternion::to_euler()`,
   urutan aerospace Z-Y-X) tidak cocok dengan urutan dekomposisi Euler
   internal BNO055 sendiri (Y-X-Z) — menghasilkan offset besar (~19°).
2. Setelah formula dikoreksi ke urutan Y-X-Z, masih ada **cross-axis
   coupling**: menaikkan pitch ke 28° ikut menggeser roll ke -47° (lihat
   percobaan bench 2026-08-05). Ini tanda input sumbu yang salah pasang ke
   formula atan2/asin, bukan sekadar label roll/pitch tertukar.

Temuan ini konsisten dengan proyek referensi `fc-skripsi-main` (lihat bagian
di bawah): penulisnya sendiri mencoba jalur quaternion serupa dan
mencatatnya sebagai "ancur" saat diuji di kontrol, lalu kembali memakai
register Euler sebagai sumber utama. Modul ini mengikuti keputusan yang sama:
**Euler sebagai sumber utama roll/pitch/yaw**, quaternion tetap tersedia
untuk logging atau AHRS/EKF di masa depan — tapi tidak dipakai untuk
kontrol saat ini.

Quaternion **bukan** bagian dari `ImuData` lagi, dan tidak dibaca dari
`readData()`/`update()`. `g_imu.update()` berjalan di task 200 Hz
(`taskImu`, `kImuPeriodMs = 5`, lihat `src/main.cpp`) lewat `vTaskDelay()`
(bukan `vTaskDelayUntil()`), jadi setiap transaksi I2C tambahan yang
dijalankan di `readData()` langsung memperpanjang periode task tersebut —
saat pembacaan quaternion sempat digabung ke `readData()`, itu menambah
transaksi I2C blocking ke-5 setiap 5 ms dan terasa sebagai lag/tersendat di
seluruh sistem (control task disinkronkan ke IMU). Quaternion sekarang
dibaca lewat `updateQuaternion()`, method terpisah yang meniru pola
`updateCalibration()`: dipanggil dari task rate rendah kalau/ketika
dibutuhkan, tidak pernah dari task 200 Hz. Pemilik `fc::Imu` yang mau
memakai quaternion untuk logging perlu memanggil `updateQuaternion()`
sendiri dan mengambil hasilnya lewat `quaternion()`.

`ImuConfig` menambahkan dua mekanisme dari `fc-skripsi-main` untuk membantu
level di bangku:

- `roll_trim_deg` / `pitch_trim_deg` — trim statis yang dikurangkan dari
  `roll_deg`/`pitch_deg` setelah dibaca dari register, setara
  `roll_error`/`pitch_error` di `bno_euler.h` mereka. Tentukan nilainya
  dengan meletakkan airframe rata dan mencatat sisa bacaan roll/pitch.
- `yaw_filter_alpha` — EMA (`yaw = (1-alpha)*lama + alpha*baru`) hanya untuk
  yaw, setara `yaw = 0.95*last_yaw + 0.05*yaw` di referensi (alpha = 0.05).
  Default `0` (nonaktif) di modul ini karena wahana ini fixed-wing, dan
  smoothing berat menambah lag kontrol yaw yang mungkin lebih ditoleransi
  quadcopter (platform referensi) daripada fixed-wing. Filter ini juga tidak
  menangani wrap ±180° secara khusus — sama seperti referensi, jadi ada
  potensi glitch satu sample saat yaw melewati ±180°.

Offset heading bawaan adalah -180 derajat. Ini, `roll_trim_deg`,
`pitch_trim_deg`, dan `yaw_filter_alpha` dapat diubah melalui `ImuConfig`,
tetapi harus diverifikasi dengan bench test sebelum uji terbang.

## Inversi pitch angle dan pitch rate (terverifikasi bangku 2026-08-11)

Nilai default lama (`invert_gyro_y = true`, warisan asumsi program lama,
belum pernah diverifikasi) dan `pitch_deg` yang tidak pernah dibalik sama
sekali ternyata **berlawanan dengan konvensi aerospace** pada airframe/mount
ini, ditemukan lewat dua pengecekan independen:

1. **Pitch angle**: HUD Mission Planner menampilkan pitch terbalik (badan
   pesawat diangkat, HUD menunjukkan turun) -- diverifikasi dari
   `ATTITUDE.pitch` yang bersumber dari `pitch_deg` ([Imu.cpp](../src/drivers/Imu.cpp),
   dibaca dari register Euler BNO055 `euler.p`, dikirim lewat
   `Mavlink::sendAttitude()`).
2. **Pitch rate**: dicek terpisah lewat MAVLink Inspector (gyro Y /
   `angular_rate_dps.y`) -- juga terbalik, dengan `invert_gyro_y=true`
   (default lama) hidung diangkat malah membaca rate negatif.

Kedua temuan **independen** (angle dari register Euler BNO055, rate dari
register gyro BNO055 -- dua sumber data berbeda), jadi keduanya perlu
dikoreksi terpisah, bukan diasumsikan satu penyebab yang sama:

- `invert_gyro_y` diubah default dari `true` menjadi **`false`** -- nilai
  `true` yang lama justru membalik gyro Y mentah yang sebenarnya sudah benar
  tandanya untuk hardware ini.
- `invert_pitch` (field baru) default **`true`** -- `pitch_deg` mentah dari
  register Euler perlu dibalik untuk hardware ini. Diterapkan SEBELUM
  `pitch_trim_deg` dikurangkan (lihat `Imu.cpp`'s `readData()`), supaya
  prosedur kalibrasi trim ("letakkan airframe rata, catat sisa bacaan")
  tetap mengacu ke nilai akhir yang sudah benar tandanya, bukan nilai mentah
  pra-inversi.

Ini penting untuk loop kontrol pitch ([AttitudeController.cpp](../src/control/AttitudeController.cpp)),
yang memakai `pitch_deg` (P-term) dan `angular_rate_dps.y` (D-term) sekaligus
-- kalau hanya salah satu dibalik tanpa verifikasi keduanya, angle dan rate
bisa berlawanan tanda dan loop kontrol pitch bisa menguatkan osilasi alih-alih
meredamnya. Kedua flag di atas sudah diverifikasi bangku secara terpisah
sebelum diubah, bukan tebakan.

## Kalibrasi BNO055 tersimpan lintas boot

BNO055 tidak punya penyimpanan non-volatile sendiri untuk hasil kalibrasinya
— setiap power-cycle, chip kalibrasi ulang dari nol dan konvergensinya
tergantung gerakan yang kebetulan terjadi tepat setelah nyala. Ini penyebab
laporan "offset pitch kadang 10°, kadang 3°" di lantai yang sama antar sesi
uji.

`Imu::readCalibrationOffsets()`/`writeCalibrationOffsets()` membaca/menulis
9 register offset accel+mag+gyro chip (`lib/BNO055/src/BNO055.h:1944-1996`).
`storage/ImuCalibrationStorage.h` menyimpannya ke EEPROM (alamat 2000+,
dengan magic number + checksum, meniru pola `storage/Waypoints.h`). Alur di
`src/main.cpp`:

- **Boot**: setelah `g_imu.begin()` sukses, coba muat offset tersimpan dan
  tulis balik ke chip lewat `writeCalibrationOffsets()` sebelum flight loop
  mulai bergantung pada attitude segar.
- **Runtime**: `taskMavlink` (rate 1 Hz, numpang di pengecekan waktu yang
  sama dengan `g_battery.update()`) memanggil `g_imu.updateCalibration()`.
  Begitu `isFullyCalibrated()` pertama kali `true` di sesi itu, offset dibaca
  dan disimpan ke EEPROM sekali (flag `imu_calibration_saved` mencegah tulis
  berulang).

Ini juga akhirnya men-wiring `updateCalibration()` yang sebelumnya ada di
class tapi tidak pernah dipanggil dari task manapun.

**Catatan alamat EEPROM**: saat menambahkan modul ini, ditemukan
`storage/Params.cpp` (alamat lama mulai 650) dan `storage/Waypoints.h`
(alamat mulai 8, sampai byte 808 untuk `kMaxEepromWaypoints=50 *
sizeof(Locations)=16`) **saling tumpang tindih** di peta EEPROM Teensy 4.1
ini — bug yang sudah ada sebelum perubahan IMU ini. Sudah diperbaiki:
`Params` dipindah ke `[1000, 1132)`, lihat `docs/params.md` untuk detail dan
peta alamat lengkapnya. `ImuCalibrationStorage` tetap di alamat 2000, sudah
aman dari kedua rentang tersebut sejak awal.

## Temuan bench (2026-08-20): cross-axis coupling menetap walau via Euler register, dugaan kalibrasi EEPROM buruk

Setelah "Calibrate Level" dijalankan, roll/pitch **masih** ikut berubah saat
yaw digerakkan di meja datar (roll 19°, pitch -10° saat yaw ~105°). Ini
gejala yang sama seperti bug axis-convention quaternion yang sudah
diperbaiki minggu sebelumnya — tapi sekarang terjadi lewat register
**Euler** (`euler.r`/`euler.p`), yaitu dekomposisi internal Bosch sendiri,
bukan kode konversi proyek ini. Itu berarti penyebabnya **bukan** bug di
`Imu.cpp` seperti sebelumnya.

Dugaan utama: `storage/ImuCalibrationStorage.h` (ditambahkan sesi
sebelumnya untuk mengatasi "offset pitch tidak konsisten antar boot") bisa
jadi menyimpan snapshot kalibrasi accel/mag/gyro yang **buruk** — misalnya
terekam saat ada gangguan magnet, atau sebelum benar-benar konvergen — lalu
snapshot itu **dipaksa ditulis ulang ke chip setiap boot**
(`writeCalibrationOffsets()` dipanggil dari `setup()`). Offset yang salah
bisa mengacaukan referensi internal fusion Bosch sendiri, menghasilkan
cross-axis coupling walau datanya diambil dari register Euler yang
seharusnya sudah didekomposisi dengan benar oleh chip.

**Perbaikan sementara**: tombol "Calibrate Accel" di Mission Planner
(param5=1) untuk sementara di-*repurpose* jadi "hapus kalibrasi tersimpan +
nol-kan register offset chip", bukan flow 6-posisi asli (lihat bagian di
bawah). Klik tombol itu, lalu **power-cycle** FC dan biarkan kalibrasi
ulang dari nol sebelum menguji ulang skenario yaw-mempengaruhi-roll/pitch.

**Cara membaca hasilnya** (diagnostik, bukan cuma perbaikan):
- Kalau setelah clear + power-cycle + kalibrasi ulang masalahnya **hilang**:
  benar snapshot lama yang buruk, kasus selesai.
- Kalau masalahnya **langsung muncul lagi** sebelum kalibrasi baru sempat
  auto-save (`taskMavlink`'s pengecekan 1 Hz, lihat bagian "Kalibrasi
  BNO055 tersimpan lintas boot"): bukan EEPROM, curigai interferensi
  magnet di lokasi uji (motor/ESC/kabel arus tinggi/tulangan beton lantai)
  atau pemasangan mekanis board yang miring dari rangka airframe.
- Kalau masalahnya muncul lagi **setelah** auto-save kalibrasi baru: berarti
  proses auto-save-nya sendiri yang menangkap snapshot buruk (mis. gate
  `isFullyCalibrated()` terlalu longgar, langsung percaya begitu status
  pertama kali jadi 3 tanpa menunggu stabil) -- perlu diperketat, belum
  diperbaiki di sesi ini.

## Calibrate Level hilang lagi setelah beberapa detik (2026-08-22)

Ditemukan lewat uji lanjutan (100kHz I2C tidak mengubah pola noise
magnetometer, dan meja/tangan sudah dipastikan bebas logam): setelah
"Calibrate Level" dijalankan, `roll_deg`/`pitch_deg` sempat 0, tapi
beberapa detik kemudian bergeser sendiri kembali ke residual sebelumnya
(~4°). Trim (`roll_trim_deg`/`pitch_trim_deg`) itu **konstanta statis** --
kalau baseline mentah dari register Euler terus bergeser di background
(BNO055 masih menyempurnakan kalibrasi accel/mag on-chip-nya), trim yang
dihitung di satu momen tidak akan tetap pas beberapa detik kemudian.

`Imu::updateCalibration()` sudah polling status ini di `taskMavlink` (1 Hz)
sejak beberapa sesi lalu, tapi **tidak pernah ditampilkan** ke pengguna --
gap inilah yang bikin sulit tahu apakah kalibrasi sudah benar-benar selesai
sebelum trim dijalankan. Sekarang `calib_sys/gyro/accel/mag` (0-3 tiap
kolom) ada di CSV `fc::DataLogger` (lihat `docs/data-logger-usb.md`).

**Prosedur yang benar**: tunggu keempat angka kalibrasi jadi 3 (gerakkan
sensor ke beberapa orientasi berbeda dulu -- lihat panduan kalibrasi Bosch
di bagian bawah dokumen ini) **sebelum** klik "Calibrate Level", bukan
langsung trim begitu terlihat rata secara visual. Kalau baseline masih
bergeser walau keempat kolom sudah 3, itu baru mengindikasikan masalah
lain (bukan kalibrasi belum selesai) dan perlu diselidiki terpisah.

## Temuan bench lanjutan (2026-08-22): pola noise magnetometer berubah setelah menjauh dari logam

Uji ulang jauh dari benda logam/magnet (`AKUSISI-IMU 22, 08, 2026, 140503.txt`)
menunjukkan dua hal:

1. **Bias roll/pitch ~4° saat diam** (roll=4.31°, pitch=-3.94° konsisten,
   std <0.6° sepanjang file) dibanding pengukur level HP yang menunjukkan
   0° — ini bias konstan wajar (toleransi mekanis pemasangan), bukan bug.
   Solusinya "Calibrate Level" (param5=2 MAVLink, lihat bagian di bawah),
   bukan perubahan kode.
2. **Interferensi magnetometer masih ada, malah lebih ekstrem** (sampai
   1204 µT) meski sudah menjauh dari logam -- tapi polanya **berubah**:
   `mag_x` bolak-balik tanda tiap sampel berurutan (-218 → +400 → -205 →
   +362 → +511 → -263 ...), bukan perubahan halus/gradual seperti saat
   diseret dekat logam di uji sebelumnya. Pola bolak-balik-tanda-tiap-
   sampel ini lebih mirip **bit error I2C** (wiring/pull-up/EMI di jalur
   SDA-SCL) daripada sumber magnet fisik nyata yang tersapu -- gerakan
   fisik menghasilkan perubahan gradual, bukan lompatan tanda acak per
   sampel.

**Sedang diuji**: `FC_BNO055_I2C_CLOCK_HZ` (`FC_Config.h`, default
400000/Fast Mode) diturunkan ke 100000 (Standard Mode) untuk melihat
apakah pola bolak-balik ini hilang -- kalau ya, itu konfirmasi kuat
masalah sinyal I2C, bukan interferensi magnet eksternal. Belum ada hasil
uji baru saat catatan ini ditulis.

## Axis remap dan gravity_mss (2026-08-22, disinkronkan dari build diagnostic terpisah)

`ImuData` menambahkan `gravity_mss` -- vektor gravitasi hasil fusion
on-chip BNO055 (register 0x2E-0x33, satu transaksi I2C bersama
`linear_acceleration_mss` di 0x28-0x2D karena keduanya bersebelahan di
peta register). Berguna sebagai cross-check: `acceleration_mss` idealnya
`~= linear_acceleration_mss + gravity_mss`; kalau menyimpang jauh, itu
tanda fusion internal BNO055 sendiri sedang tidak konsisten (relevan untuk
diagnosis interferensi magnet, lihat analisis log 2026-08-22).

`FC_Config.h` juga menambahkan `FC_BNO055_AXIS_MAP_CONFIG`/
`FC_BNO055_AXIS_SIGN_X/Y/Z`, benar-benar diterapkan ke register
`AXIS_MAP_CONFIG`/`AXIS_MAP_SIGN` chip di `Imu::configureSensor()` (bukan
cuma dicatat) -- default `DEFAULT_AXIS`/`0`/`0`/`0` mempertahankan
orientasi P1 pabrik, sama seperti sebelum flag ini ada. Ini beda dari
`invert_pitch`/`invert_gyro_y`/`roll_trim_deg`/`pitch_trim_deg` di
`ImuConfig`: yang terakhir menambal nilai SETELAH keluar dari chip, remap
ini mengubah bagaimana chip sendiri menghitung raw register DAN fusion
Euler/quaternion-nya. Nilai yang benar-benar terpasang dicetak saat boot
(lihat `docs/data-logger-usb.md`'s bagian "Boot diagnostic") supaya log
CSV mana pun self-describing soal konfigurasi remap yang dipakai.

## Kalibrasi Mission Planner ("Calibrate Level")

`Mavlink::handleCommandLong()` menangani `MAV_CMD_PREFLIGHT_CALIBRATION`
dengan `param5`:

| `param5` | Tombol Mission Planner | Status |
| --- | --- | --- |
| 2 | Calibrate Level | **Diimplementasikan** — satu round-trip MAVLink, set `roll_trim_deg`/`pitch_trim_deg` ke residual `roll_deg`/`pitch_deg` saat ini, simpan lewat `Params` (`IMU_ROLL_TRIM`/`IMU_PITCH_TRIM`) |
| 1 | Calibrate Accel (6 posisi) | **Di-repurpose sementara** (2026-08-20) jadi "hapus kalibrasi tersimpan" — lihat bagian "Temuan bench" di atas. Bukan flow 6-posisi asli; itu masih belum diimplementasikan, butuh handshake `MAV_CMD_ACCELCAL_VEHICLE_POS` dan strategi penghitungan offset yang belum diputuskan |
| 4 | Simple Accel Cal (1 posisi) | Belum — ditunda sampai strategi offset diputuskan |

Field `roll_trim_deg`/`pitch_trim_deg` (`ImuConfig`) diekspos ke `Params`
lewat `Imu::rollTrimDegRef()`/`pitchTrimDegRef()` — referensi langsung ke
member `config_`, dipakai persis seperti `float*` param lain di
`Params::initFixedWing()`.

### Referensi: `fc-skripsi-main`

Proyek referensi ini (skripsi lain berbasis Teensy + BNO055, arsitektur
quadcopter dengan kontrol LQR) berisi dua implementasi IMU:

| File | Dipakai di `main.cpp`? | Sumber | Filter |
| --- | --- | --- | --- |
| `include/bno_euler.h` | Ya — di-`#include` dan dipanggil dari `drone_controller()` | Register Euler (`bno055_read_euler_hrp`) | EMA pada yaw saja (α=0.05); roll/pitch tanpa filter software |
| `include/bno_quaternion.h` | Tidak — `#include`-nya dikomentari di `main.cpp` | Register quaternion, dikonversi manual ke Euler urutan Z-Y-X dengan input qx/qy ditukar | Tidak ada |

**Apa yang dievaluasi (dari komentar & struktur kode mereka):**

- Status kalibrasi gyro/accel/mag/sistem (`bno055_get_*calib_status`), lewat
  fungsi `check_imu_calibration()`/`check_imu_calibration_NDOF()` — tersedia
  tapi **tidak** dipanggil otomatis saat `bno055_init()` (baris pemanggilannya
  dikomentari), jadi tidak ada gating arming berbasis kalibrasi di
  implementasi aktif mereka.
- Kecepatan sudut mentah (`gxrs`/`gyrs`/`gzrs`) dipakai langsung sebagai
  suku derivatif (D) pada kontrol LQR (`error_roll_rate = -gyrs`, dst di
  `fc_euler.h`), bukan didiferensiasi dari sudut Euler — pola umum di
  flight controller karena gyro lebih bersih dari turunan numerik sudut.
- Delta waktu kontrol dihitung dari `micros()` per loop (`dt = t_now -
  t_last`), dipakai untuk turunan setpoint (`setpoint_roll_rate`, dst), tapi
  **tidak** dipakai untuk mem-filter roll/pitch — hanya `bno055_update()`
  yang throttle ke ~100 Hz lewat `millis()`.

**Apakah memakai filter?** Sebagian, dan tidak konsisten:

- `bno_euler.h` (yang aktif): hanya **yaw** yang difilter, EMA `yaw =
  0.95*last_yaw + 0.05*yaw`. Ada komentar "Apply simple low-pass filter to
  gyro data (α = 0.1)" di atas pembacaan gyro, tapi ini **komentar basi** —
  tidak ada filter matematis yang benar-benar diterapkan ke `gxrs`/`gyrs`/
  `gzrs`, itu murni pembagian skala mentah. Roll/pitch tidak difilter sama
  sekali di level software — mengandalkan sepenuhnya fusion internal BNO055.
- `bno_quaternion.h` (nonaktif): tidak ada filter software apa pun.

Jadi "filter" yang sebenarnya ada di kedua file hanyalah fusion NDOF on-chip
BNO055 sendiri (black box, gyro+accel+mag dengan Kalman-like fusion) —
software di atasnya nyaris tidak menambah smoothing, kecuali EMA yaw di
`bno_euler.h`.

**Alur logika IMU di `fc-skripsi-main` (jalur aktif, `bno_euler.h`):**

1. `bno055_init()` — `Wire.begin()`, `BNO_Init()`, set mode `OPERATION_MODE_NDOF`,
   `delay(50)`. Tidak menunggu kalibrasi (baris `check_imu_calibration_NDOF()`
   dikomentari).
2. `loop()` di `main.cpp` memanggil `bno055_update()` setiap iterasi
   (bukan lewat thread terpisah — `threads.addThread(imu_thread, 1)`
   juga dikomentari, jadi task IMU terpisah tidak aktif; semua jalan di
   `loop()` utama secara sekuensial).
3. Di dalam `bno055_update()`: throttle manual ke ≥10 ms (~100 Hz) via
   `millis()`, lalu baca gyro (`bno055_read_gyro_xyz`) dan Euler
   (`bno055_read_euler_hrp`) berurutan.
4. Skala mentah dikonversi: gyro dibagi 16 → °/s, Euler dibagi 16 → derajat.
   Trim statis (`roll_error`, `pitch_error`) dikurangkan dari roll/pitch.
   Yaw di-wrap ke ±180° lalu di-EMA.
5. Variabel global (`roll`, `pitch`, `yaw`, `gxrs`, `gyrs`, `gzrs`) dibaca
   langsung oleh `drone_controller()` (LQR) di `fc_euler.h` tanpa lapisan
   abstraksi tambahan — beda dengan modul `fc::Imu` di repo ini yang
   membungkus semuanya dalam `ImuData` dan mengembalikan salinan lewat
   `data()`.

Modul `fc::Imu` di repo ini sekarang meniru pilihan sumber data mereka
(Euler untuk kontrol) sambil mempertahankan arsitektur yang lebih ketat:
non-blocking, tidak ada variabel global, dan quaternion tetap tersimpan
untuk riset lanjutan.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `IMU imu;` di `AHRS.h` | Object `fc::Imu` dimiliki vehicle/core |
| `imu.init_imu()` | `imu.begin()` dan periksa nilai kembaliannya |
| `imu.update_imu()` | `imu.update()` dan periksa nilai kembaliannya |
| `imu.roll` | `imu.data().roll_deg` atau `imu.rollDegrees()` |
| `imu.pitch` | `imu.data().pitch_deg` atau `imu.pitchDegrees()` |
| `imu.yaw` | `imu.data().yaw_deg` atau `imu.yawDegrees()` |
| `imu.heading` | `imu.data().heading_deg` atau `imu.headingDegrees()` |
| `imu.rad_roll` | `imu.data().roll_rad` |
| `imu.rad_pitch` | `imu.data().pitch_rad` |
| `imu.rad_yaw` | `imu.data().yaw_rad` |
| `imu.gyro_x/y/z` | `imu.data().angular_rate_dps.x/y/z` |
| `imu._accel` | `imu.data().acceleration_mss` |
| `imu._linear_acc` | `imu.data().linear_acceleration_mss` |
| `imu.delta_roll` | `imu.data().delta_roll_deg` |
| `check_imu_calibration()` | `updateCalibration()` non-blocking |
| `pitch_mode` | Dihapus; hanya dipakai jalur copter/transisi |

Perhatian: `_accel` pada implementasi lama berisi hitungan mentah BNO055,
walaupun beberapa controller memperlakukannya sebagai m/s². Implementasi baru
membaginya dengan 100 sehingga `acceleration_mss` benar-benar memakai m/s².
Gain atau asumsi controller yang bergantung pada nilai mentah perlu diperiksa
saat file AHRS/TECS direfactor.

## Perilaku kegagalan

- `begin()` gagal jika chip ID bukan `0xA0` atau konfigurasi NDOF gagal.
- `update()` tidak menerbitkan data parsial. Sampel terakhir tetap dipertahankan
  jika salah satu pembacaan Euler, gyro, accelerometer, atau linear
  acceleration gagal.
- `lastError()` dan `consecutiveReadFailures()` dapat dipakai oleh pre-arm,
  health monitor, dan failsafe.
- `isFresh(maximum_age_us)` mencegah controller memakai data IMU yang basi.

## File lama yang tidak dimigrasikan

```text
include/bno055.h
include/imu.h
include/drivers/mpu6050.h
include/drivers/MPU6050Reg.h
include/drivers/hmc5883l.h
lib/BNO055-Library/BNO055_support.h
lib/BNO055-Library/BNO055_support.cpp
```

File-file tersebut tetap boleh berada di arsip program lama sampai seluruh
pemakai IMU selesai dimigrasikan, tetapi tidak boleh dimasukkan ke build program
baru.

Library Bosch yang disimpan di `lib/BNO055` mempertahankan pemberitahuan hak
cipta dan lisensi GPL pada source serta README vendornya.
