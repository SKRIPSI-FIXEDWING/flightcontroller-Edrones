# Opsi 2: Mahony AHRS attitude (fc::AttitudeMahonyFilter)

Dua opsi estimasi attitude tersedia berdampingan di firmware ini, untuk
perbandingan bench/skripsi -- **bukan** untuk operator memilih salah satu
saat terbang. Pola ini sama persis dengan `docs/altitude-complementary-filter.md`
untuk altitude, hanya diterapkan ke roll/pitch/yaw:

- **Opsi 1 -- fusion on-chip BNO055 (NDOF)**, dibaca dari register Euler
  (`fc::Imu`'s `roll_deg`/`pitch_deg`/`yaw_deg`, lihat `docs/imu-bno055.md`).
  Ini yang **tetap** dikonsumsi AttitudeController/AHRS untuk kontrol
  penerbangan -- tidak berubah oleh modul ini.
- **Opsi 2 -- Mahony AHRS 9-axis** (`fc::AttitudeMahonyFilter`, modul baru),
  dihitung sendiri dari gyro+accel+mag mentah. Berjalan paralel, di-log
  berdampingan dengan Opsi 1, **tidak** disambungkan ke AttitudeController.

## Kenapa tidak langsung menggantikan Opsi 1

Sama seperti alasan `AltitudeComplementaryFilter`: mengganti sumber attitude
yang dipakai kontrol penerbangan adalah perubahan safety-critical yang butuh
validasi dulu. Modul ini murni estimator paralel yang di-log, supaya Opsi 1
vs Opsi 2 bisa dibandingkan dari data bench/terbang nyata -- itu memang
tujuan utama fitur ini diminta.

## Referensi

Mahony, R.; Hamel, T.; Pflimlin, J.-M. **"Nonlinear Complementary Filters on
the Special Orthogonal Group."** *IEEE Transactions on Automatic Control*
2008, 53(5), 1203-1218. DOI: 10.1109/TAC.2008.923738.

Implementasi di `AttitudeMahonyFilter.cpp` mengikuti struktur referensi
terbuka yang paling umum dipakai untuk paper ini (mis. implementasi C milik
x-io Technologies / Sebastian Madgwick, dipakai luas di proyek open-source
AHRS) -- quaternion diperbarui dari integrasi gyro, dikoreksi tiap langkah
oleh umpan balik proporsional+integral dari cross-product antara vektor
referensi (gravitasi dari accel, medan magnet dari mag) dan estimasi
arahnya saat ini.

## Struktur matematis yang diimplementasikan

```text
// Error = cross product antara vektor referensi terukur dan estimasi arahnya
// (accel -> arah gravitasi body-frame, mag -> arah utara magnet body-frame)
error = (accel x v_estimasi) + (mag x w_estimasi)

// Feedback proporsional+integral ditambahkan ke gyro sebelum integrasi
integral_bias += Ki * error * dt
gyro_terkoreksi = gyro + Kp * error + integral_bias

// Integrasi quaternion: dq/dt = 0.5 * q (x) [0, gyro_terkoreksi]
q += dq/dt * dt
q = normalize(q)
```

`Kp`/`Ki` (`AttitudeMahonyConfig`, default 2.0 dan 0.005) belum di-tuning
empiris untuk BNO055 spesifik proyek ini -- ini nilai umum yang dipakai di
literatur/implementasi referensi sebagai titik awal, bukan hasil kalibrasi
untuk hardware ini.

## Sumber data input -- PENTING, belum diverifikasi bangku

`update()` dipanggil dari `taskImu` ([main.cpp](../src/main.cpp)) dengan:

- **Gyro**: `ImuData::angular_rate_dps` (sudah dikonversi ke rad/s), yaitu
  nilai yang **sudah** melalui `invert_gyro_y` (lihat `docs/imu-bno055.md`,
  terverifikasi bangku 2026-08-11).
- **Accel**: `ImuData::acceleration_mss` mentah, **tanpa** koreksi tambahan
  apa pun -- dipakai apa adanya karena kode lain di repo ini
  (`Ahrs.cpp`) juga sudah mengasumsikan field ini langsung dalam frame body
  aircraft tanpa transformasi, tapi asumsi itu sendiri **belum pernah
  diverifikasi bangku secara eksplisit**.
- **Magnetometer**: `ImuData::magnetic_field_ut`, field yang **baru
  ditambahkan** untuk fitur ini. Dibaca langsung dari register mentah
  BNO055 (`Imu.cpp`'s burst read, lihat `docs/imu-bno055.md`) **tanpa
  remap axis apa pun** -- register data mentah BNO055 (accel/gyro/mag)
  selalu dalam frame axis fisik chip default (P1), TIDAK dipengaruhi oleh
  remap yang berlaku untuk output fusion Euler/quaternion milik Bosch.

**Konsekuensinya**: berbeda dari `roll_deg`/`pitch_deg` (yang sudah dua kali
dikoreksi lewat bench test nyata sesi ini -- `invert_pitch` dan
`invert_gyro_y`), roll/pitch/yaw hasil `AttitudeMahonyFilter` **belum
diverifikasi** sama sekali. Ada kemungkinan nyata magnetometer (atau bahkan
accel) perlu remap/inversi sumbu serupa sebelum outputnya benar-benar
sebanding dengan Opsi 1 -- lihat riwayat bug axis-convention BNO055 di
`docs/imu-bno055.md` untuk gambaran seberapa mudah asumsi ini salah tanpa
verifikasi.

`AttitudeMahonyFilter` sendiri **axis-convention-agnostic by design**
(seperti `AltitudeComplementaryFilter` menerima `accel_up_mss` yang sudah
dikoreksi pemanggilnya) -- kelas ini tidak tahu apa-apa soal BNO055,
tanggung jawab menyediakan input yang benar sepenuhnya ada di titik
pemanggilan (`taskImu`).

## Prosedur verifikasi bangku yang disarankan (belum dilakukan)

Sama seperti prosedur yang menemukan `invert_pitch`/`invert_gyro_y`:

1. Letakkan airframe rata, bandingkan `mahony_roll_deg`/`mahony_pitch_deg`
   (lewat log SD, lihat di bawah) terhadap `roll_deg`/`pitch_deg` (Opsi 1).
   Keduanya harus dekat nol dan bertanda sama.
2. Angkat hidung (pitch naik), cek tanda `mahony_pitch_deg` searah dengan
   `pitch_deg`.
3. Miringkan sayap kanan turun (roll), cek tanda `mahony_roll_deg` searah
   `roll_deg`.
4. Putar yaw perlahan 360 derajat menjauhi gangguan magnet (motor/ESC/kabel
   arus tinggi), cek `mahony_yaw_deg` mengikuti arah dan kecepatan yang
   sama dengan `yaw_deg` -- ini sekaligus mengetes magnetometer mentahnya
   punya polaritas/skala yang wajar relatif terhadap accel/gyro.

Kalau ada sumbu yang terbalik/tertukar, jangan tebak koreksinya -- catat
gejala persisnya (mana yang salah, seberapa jauh) dan koreksi di titik
pemanggilan `g_attitudeMahony.update(...)` di `taskImu`, bukan di dalam
kelas filternya sendiri.

## Penjadwalan

Dipanggil dari `taskImu` (200 Hz, [main.cpp](../src/main.cpp)) setiap kali
`g_imu.update()` sukses, dengan `dt_s` diukur dari `micros()` (bukan
`kImuPeriodMs` tetap), mengikuti pola yang sama dengan `taskControl`'s
pengukuran `dt_s`. **Tidak** ada task FreeRTOS terpisah untuk modul ini.

## Efisiensi I2C: burst read

Menambahkan pembacaan magnetometer mentah ke siklus 200 Hz berisiko
mengulangi bug lag yang pernah terjadi saat quaternion sempat ditambahkan
ke `readData()` (lihat `docs/imu-bno055.md`). Untuk menghindarinya,
`Imu::readData()` sekarang membaca accel(0x08)+mag(0x0E)+gyro(0x14)+
euler(0x1A) — 24 byte berturutan di peta register BNO055 — dalam **satu**
transaksi I2C burst, menggantikan 3 transaksi terpisah yang sebelumnya ada
(Euler, gyro, accel). Net hasilnya: dari 4 transaksi I2C per siklus
(Euler+gyro+accel+linear-accel) menjadi 2 (burst+linear-accel), sambil
menambah data magnetometer -- lebih sedikit overhead I2C dibanding sebelum
fitur ini ada, bukan lebih banyak.

## Flag compile-time (FC_Config.h)

Dua flag terpisah, karena "dikompilasi" dan "dipakai terbang" adalah dua
tingkat komitmen yang beda:

- **`FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE`** (default `1`) — apakah
  `fc::AttitudeMahonyFilter` dikompilasi sama sekali. `0` mengeluarkannya
  total dari build: tidak ada objek, tidak ada panggilan `update()` di
  `taskImu`, tidak ada kolom SD log, tidak ada telemetry `MHN_*` — build
  paling ramping untuk kondisi terbang produksi yang tidak butuh
  perbandingan. Diverifikasi menghemat sekitar 1.8 KB flash pada firmware
  ini.
- **`FC_ATTITUDE_CONTROL_SOURCE_MAHONY`** (default `0`) — apakah
  `AttitudeController` benar-benar terbang memakai roll/pitch Mahony
  (lewat `VehicleContext::controlImu()`), bukan cuma dihitung untuk
  dibandingkan. Butuh `FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE=1` (dipaksa lewat
  `#error` kalau tidak). **Default OFF, dan tetap OFF sampai prosedur
  verifikasi bangku di atas benar-benar dilakukan** — mengaktifkan ini
  sebelum verifikasi berarti pesawat terbang dengan data attitude yang
  belum tervalidasi, kelas bug axis-convention yang sama yang sudah dua
  kali ditemukan (dan diperbaiki) di jalur BNO055 lain sesi ini.

`Navigation`/`TECS`/`Ahrs` **tidak terpengaruh** oleh
`FC_ATTITUDE_CONTROL_SOURCE_MAHONY` — mereka tetap selalu membaca BNO055
(Opsi 1) langsung dari `ctx.imu.data()`, cuma loop umpan-balik
attitude-control (`ctx_.attitude.update()` di ketiga mode fixed-wing) yang
ikut berpindah. Ini karena `AttitudeController::update()` cuma memakai
`roll_deg`/`pitch_deg` dari argumen `ImuData`-nya (gyro rate sumbernya sama
persis untuk kedua estimator, jadi tidak perlu ditukar).

Cara pakai (tanpa mengubah file, sekali build):

```bash
# Build ramping, Mahony tidak dikompilasi sama sekali:
PLATFORMIO_BUILD_FLAGS="-DFC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE=0" pio run -e teensy41

# Mahony jadi sumber kontrol -- HANYA setelah verifikasi bangku selesai:
PLATFORMIO_BUILD_FLAGS="-DFC_ATTITUDE_CONTROL_SOURCE_MAHONY=1" pio run -e teensy41
```

Atau ubah nilai default langsung di `include/FC_Config.h` untuk build
permanen, mengikuti pola `FC_DEBUG_SERIAL_ENABLE` yang sudah ada di file
yang sama.

## Telemetry live (Mission Planner)

`Mavlink::sendMahonyAttitude()` mengirim `mahony.roll_deg`/`pitch_deg`/
`yaw_deg` sebagai tiga pesan `NAMED_VALUE_FLOAT` (`MHN_ROLL`, `MHN_PITCH`,
`MHN_YAW`) di 10 Hz dari `taskMavlink` ([main.cpp](../src/main.cpp)) --
tidak perlu definisi pesan MAVLink kustom, Mission Planner otomatis
menampilkannya di tab Status. Ini murni alat bantu perbandingan, **bukan**
input kontrol -- `ATTITUDE` (dipakai Mission Planner untuk HUD attitude
indicator) tetap 100% dari `roll_deg`/`pitch_deg`/`yaw_deg` (Opsi 1, BNO055
on-chip). Kalau Anda melihat HUD attitude indicator berubah, itu tetap
Opsi 1 -- lihat nilai `MHN_ROLL`/`MHN_PITCH`/`MHN_YAW` di tab Status untuk
Opsi 2.

## Format CSV (`fc::SdLogger`)

```text
timestamp_ms,roll_deg,pitch_deg,yaw_deg,altitude_m,mode,armed,ch1_roll,ch2_pitch,ch3_throttle,ch4_yaw,ch5_arm_raw,mahony_roll_deg,mahony_pitch_deg,mahony_yaw_deg
```

`roll_deg`/`pitch_deg`/`yaw_deg` = Opsi 1 (BNO055 on-chip). `mahony_roll_deg`/
`mahony_pitch_deg`/`mahony_yaw_deg` = Opsi 2 (Mahony). Log lama (12 kolom,
tanpa tiga kolom Mahony) tidak lagi cocok dengan header baru -- setiap boot
sudah membuat file `LOGnnn.CSV` baru (lihat `docs/sd-logger-mtp.md`), jadi
tidak ada file lama yang perlu dimigrasikan, cukup jangan gabungkan file
lama dan baru dalam satu analisis tanpa memperhatikan jumlah kolomnya.

## Keterbatasan yang diketahui

- **Sumber data input belum diverifikasi bangku** (lihat bagian di atas) --
  ini keterbatasan paling penting, jangan menyimpulkan apa pun dari
  perbandingan Opsi 1 vs Opsi 2 sebelum prosedur verifikasi di atas
  dilakukan.
- Gain `Kp`/`Ki` masih nilai umum dari literatur, belum di-tuning empiris
  untuk BNO055 spesifik hardware ini.
- Kalibrasi magnetometer (hard-iron/soft-iron) tidak ditangani modul ini
  sama sekali -- filter mengasumsikan `magnetic_field_ut` sudah representasi
  medan magnet yang wajar. Interferensi magnet dari motor/ESC/kabel arus
  tinggi akan langsung mendistorsi hasil Mahony (sama seperti akan
  mendistorsi fusion BNO055 sendiri), tapi BNO055 setidaknya punya
  kalibrasi mag on-chip (lihat `docs/imu-bno055.md`'s bagian kalibrasi)
  yang tidak dimanfaatkan modul ini.
- Belum divalidasi pada gerakan/terbang nyata -- baru diverifikasi compile
  dan pemetaan register/unit, sama seperti status awal
  `AltitudeComplementaryFilter` sebelum temuan bench pertamanya.
