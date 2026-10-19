# Airspeed MS5525DSO

Sensor: MS5525DSO tipe 001DS (±1 psi, differential), I2C alamat `0x76`, bus `Wire2`
(SCL = pin 24, SDA = pin 25, 3.3 V). Sumber: `airspeed_ms5525_v2.ino` yang sudah
diuji di bangku, dipindah ke `fc::Airspeed` tanpa mengubah rumusnya.

Menggantikan driver MS4525DO (alamat 0x28) sebelumnya. Antarmuka publik
(`AirspeedData::velocity_mps`, `differential_pressure_psi`, dst.) tetap sama
sehingga TECS, AHRS, dan MAVLink tidak berubah.

## File

```text
include/drivers/Airspeed.h   Kontrak data, AirspeedConfig, class Airspeed
src/drivers/Airspeed.cpp     Implementasi (I2C Wire2)
```

## Alur

`begin()` (blocking, sekitar 2 detik, dipanggil dari `setup()` sebelum task jalan):
reset sensor → baca PROM C1..C6 (ditolak jika 0 / 0xFFFF) → baca suhu →
**kalibrasi offset nol** (100 sampel, rata-rata separuh tengah setelah diurutkan).
**Pitot harus tidak kena angin saat boot.**

`update()` (task `taskAirspeed`, 25 ms / 40 Hz) **tidak memblokir**. Tiap panggilan
membaca hasil konversi yang dimulai panggilan sebelumnya, lalu memulai konversi
berikutnya (tekanan OSR 4096; suhu OSR 2048 setiap 40 sampel tekanan).

## Pengolahan data

- ADC bernilai 0 atau 0xFFFFFF dibuang (penyebab spike di versi awal).
- dP di luar ±7500 Pa dibuang; suhu di luar -40..125 °C dibuang.
- Low-pass pada dP: `alpha = 0.2`, diinisialisasi dari sampel valid pertama.
- Deadband: dP < 2 Pa → 0 m/s.
- `v = sqrt(2 * dP / rho)`, `rho = 101325 / (287.058 * (T + 273.15))`.
- Gagal baca 2 kali berturut-turut → bus I2C di-reset (`end()` / `begin()`).
- `differential_pressure_pa` sekarang Pa yang sebenarnya (sebelumnya 2× lipat).

## Perilaku kegagalan

- `begin()` gagal jika sensor tidak merespons, PROM tidak valid, atau suhu gagal dibaca.
  main.cpp mengirim pesan `[SETUP] Airspeed init failed`.
- `lastError()`: `None`, `NotInitialized`, `ReadFailed`, `InvalidData`.
- Penghitung: `i2cErrorCount()`, `invalidDataCount()`, `consecutiveReadFailures()`.
- `zeroOffsetPa()` dapat dicek: nilai absolut > 400 Pa berarti ada tekanan di pitot saat boot.
- Jika `update()` gagal, sampel terakhir dipertahankan (`valid` tetap true); gunakan
  `isFresh()` / `healthy()` bila perlu mendeteksi sensor mati.

## Catatan

- Jika dP negatif saat ada aliran ke pitot: `AirspeedConfig::pressure_sign = -1.0f`
  (setara `PRESSURE_SIGN` di sketsa).
- Deteksi "sensor masuk akal" (valid / NaN / < 50 m/s) ada di `Ahrs::update()`.
