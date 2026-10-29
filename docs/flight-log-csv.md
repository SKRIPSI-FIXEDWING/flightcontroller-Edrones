# Log penerbangan SD card (format v2, `fc::FlightLog`)

Ditulis 2026-10-25 untuk data analisis skripsi (perbandingan L1 tetap vs
L1 + fuzzy self-tuning). Mekanisme penulisan ke kartu (non-blocking, MTP,
retry mount) ada di `docs/sd-logger-mtp.md`; dokumen ini hanya isi file.

Kode: `include/communication/FlightLog.h`, `src/communication/FlightLog.cpp`.
Daftar perubahan yang diminta: `saran-sd-logging.md` (catatan sesi).

## Cara kerja singkat

- `taskControl` memanggil `FlightLog::capture()` di akhir **setiap siklus
  kendali**: menyalin nilai yang dipakai siklus itu (cte, periode L1, nav_roll,
  PWM servo, GPS, ...) ke satu snapshot dan mendeteksi event (arm, mode, WP,
  GPS). Hanya `memcpy` + seqlock, tidak pernah menunggu.
- `taskSdLog` (20 Hz, `kSdLogPeriodMs`) memanggil `FlightLog::step()`: format
  snapshot terbaru jadi 1 baris CSV, tulis header/event/snapshot bila ada,
  masukkan ke buffer RAM. `taskSdWriter` memindahkannya ke kartu.
- Logging mulai sejak boot dan terus menerus (bukan hanya saat armed): tidak
  ada data terlewat. Potong bagian di darat dengan kolom `armed`/`mode`.
- Satu file baru per boot (dan setelah remount kartu). File dibuka sebagai
  `LOGnnn.CSV`; begitu GPS Here4 punya waktu UTC, file diganti nama menjadi
  **tanggal dan jam lokal (WIB) saat file dibuka**, mis.
  `2026-10-29_08-58-36.CSV` (sejak 2026-10-29, `kSdLogUtcOffsetMinutes`).
  Tanpa waktu GPS (di dalam ruangan, backend NMEA) nama `LOGnnn.CSV` tetap.
  Header `# file=` tetap berisi nama awal; nama akhir ada di event `LOG_NAME`.

## Struktur file

```text
# EDRONES_FLIGHT_LOG format=2 ...        <- baris komentar (diawali #)
# fw_tag=... build=...
# file=LOG007.CSV boot_ms=... log_period_ms=50
# ... penjelasan kolom ...
#SNAPSHOT,BOOT,<ms>
# EFFECTIVE l1_damping=... wp_radius_m=... roll_limit_deg=...
# EFFECTIVE fuzzy_enable=... fuzzy_base_period=... fuzzy_min_per=... fuzzy_max_per=...
# PARAM_TABLE (...)
# PARAM NAVL1_PERIOD=22 NAVL1_DAMPING=0.707 ...
# HOME,<lat_e7>,<lon_e7>,<alt_cm>
# WP_FIELDS seq,lat_e7,lon_e7,alt_cm,radius_m(0=default WP_RADIUS)
# WP,0,...
time_boot_ms,gps_seq,...                 <- SATU baris nama kolom
1234,5,1,MANU,0,...                     <- data 20 Hz
#EVENT,5678,ARM                         <- event, di sela data
#ARM_SNAPSHOT,5678
# EFFECTIVE ... / # PARAM ... / # WP,...
```

Pembaca CSV: abaikan baris yang diawali `#`; baris pertama tanpa `#` adalah
nama kolom. Baris `#EVENT`/`#ARM_SNAPSHOT`/`#SNAPSHOT` dibaca terpisah.

## Kolom data (58, urutan sama dengan baris nama kolom)

| Kelompok | Kolom | Satuan / catatan |
|---|---|---|
| Waktu | `time_boot_ms` | ms, `millis()` — jam yang sama dengan `time_boot_ms` MAVLink/tlog (kunci untuk menggabung) |
| | `gps_seq` | penghitung fix GPS dari driver (menggantikan `gps_tow_ms`, lihat "Beda dari saran") |
| | `gps_new` | 1 bila `gps_seq` berubah sejak baris sebelumnya |
| Mode | `mode`, `armed` | `MANU/FBWA/AUTO/GUID`, 0/1 |
| GPS | `lat_e7`, `lon_e7` | integer derajat × 1e7 |
| | `gps_alt_m`, `gps_fix`, `gps_sats`, `gps_hdop` | `gps_fix`: 0 = tidak ada, 1 = 2D, 2 = 3D |
| | `gps_age_ms` | umur fix terakhir; jenuh di 65535 |
| | `gs_mps`, `course_deg` | groundspeed & course dari GPS |
| Kecepatan / tinggi | `airspeed_mps` | IAS dari sensor tekanan diferensial |
| | `alt_fused_m`, `alt_baro_m` | `alt_fused_m` = nilai yang dipakai TECS/navigasi (fusi baro+accel+GPS bila `FC_ALTITUDE_FUSION_ENABLE`); `alt_baro_m` = baro saja |
| Sikap | `roll_deg`, `pitch_deg`, `yaw_deg` | sikap yang dipakai kendali (BNO055 kecuali `FC_ATTITUDE_CONTROL_SOURCE_MAHONY`) |
| | `p_dps`, `q_dps`, `r_dps` | gyro |
| | `mhn_roll`, `mhn_pitch`, `mhn_yaw` | Mahony (0 bila `FC_ATTITUDE_ESTIMATOR_MAHONY_ENABLE=0`) |
| Navigasi* | `wp_prev`, `wp_next` | indeks WP; garis acuan = `wp_prev` → `wp_next`; -1 di luar AUTO/GUID |
| | `leg_from_wp` | 1 = garis dimulai di `wp_prev`; 0 = dimulai di posisi pesawat (leg pertama) |
| | `wp_dist_m` | jarak ke `wp_next` |
| | `cte_m` | cross-track error L1. **+ = pesawat di KIRI garis, − = KANAN** (dilihat searah garis) |
| | `xtrack_vel_mps` | kecepatan tegak lurus garis dari kecepatan GPS = d(cte)/dt, tanda sama dengan `cte_m` |
| | `de_mps` | d\|cte\|/dt persis seperti masuk ke fuzzy (lihat catatan) |
| | `l1_period_s` | periode L1 yang **dipakai** update guidance siklus itu |
| | `l1_dist_m` | L1 distance |
| | `nav_roll_deg`, `nav_pitch_deg` | perintah L1 / TECS ke kendali sikap |
| Fuzzy* | `fuzzy_on` | 0/1 (param `FUZZY_ENABLE`, live) |
| | `fuzzy_out` | keluaran defuzzifikasi = ΔT periode L1 (s, −2…+2); 0 bila fuzzy mati |
| Aktuator | `ail_cmd_us` | PWM servo aileron kiri dari kendali saja, **sebelum** gangguan |
| | `ail_out_us` | PWM servo aileron kiri yang benar-benar dikirim, **sesudah** gangguan (kanan identik) |
| | `ele_out_us`, `thr_out_us`, `rud_out_us` | |
| Gangguan | `dist_on`, `dist_us` | gangguan roll yang disuntik `DisturbanceInjector`: `dist_us` = offset PWM pada domain perintah roll, **+ = roll kanan** (arah stick kanan; PWM servo justru turun karena cermin diferensial). Nol kecuali `DIST_ENABLE=1`, mode AUTO, dan stick roll digerakkan (lihat `docs/disturbance-injection.md`) |
| RC | `ch1`…`ch8` | roll, pitch, throttle, yaw, arm, mode, mode_backup, vehicle_mode; PWM-ekuivalen terkalibrasi (988–2012) |
| Kesehatan | `batt_v` | tegangan baterai |
| | `imu_cal` | status kalibrasi BNO055 terkemas `sys*1000 + gyro*100 + accel*10 + mag` (masing-masing 0–3; **3333 = penuh**) |
| | `rows_dropped` | akumulasi baris yang gagal masuk buffer RAM |
| | `ctrl_overrun` | akumulasi siklus kendali dengan selang > `kCtrlOverrunThresholdMs` (15 ms; nominal ≈ 6–8 ms) |

\* Kolom navigasi dan `fuzzy_out` bernilai 0 di luar AUTO/GUIDED/LOITER/RTL; `l1_period_s`
tetap terisi (periode L1 saat itu).

### Catatan penting untuk analisis

- **Tanda `cte_m`**: dihitung `a_air % ab` (hasil kali silang 2D, sumbu N,E),
  jadi **positif = pesawat di kiri garis**. Contoh saran memakai "+ = kanan";
  yang berlaku adalah yang ditulis di sini dan di header setiap file.
  `nav_roll_deg` positif = bank kanan, jadi cte positif → nav_roll positif
  (belok kanan menuju garis).
- **`de_mps` = d\|cte\|/dt = tanda(cte) × `xtrack_vel_mps`** (sejak 2026-10-25),
  yaitu kecepatan GPS tegak lurus garis — persis yang masuk ke fuzzy. Sebelumnya
  dihitung dari selisih |cte| tiap siklus 5 ms pada posisi GPS 5 Hz (nol hampir
  selalu, lalu paku ratusan m/s); data dari firmware lama tidak sebanding.
  Positif = menjauhi garis, negatif = mendekat. Tidak ada filter low-pass.
- **Urutan fuzzy vs L1 dalam satu siklus**: L1 dihitung dulu (memakai periode
  dari siklus sebelumnya), baru fuzzy menetapkan periode untuk siklus
  berikutnya. `l1_period_s` mencatat periode yang L1 pakai, bukan yang baru
  ditetapkan fuzzy.
- **`FUZZY_ENABLE` adalah parameter** (sejak 2026-10-25; nilai awal dari
  `FC_FUZZY_L1_ENABLE`), berlaku langsung tanpa reboot. Statusnya tercatat di
  kolom `fuzzy_on`, event `PARAM`, dan baris `# EFFECTIVE fuzzy_enable=`.
- **Parameter yang berlaku** ada di baris `# EFFECTIVE ...`. `# PARAM` memuat
  isi tabel parameter; yang berlaku LANGSUNG: `NAVL1_PERIOD`, `NAVL1_DAMPING`,
  `NAVL1_XTRACK_I`, `FUZZY_ENABLE`, `FUZZY_MIN_PER`, `FUZZY_MAX_PER`,
  `WP_RADIUS`, `ROLL_LIMIT_DEG`, `DIST_ENABLE`; semua yang lain baru berlaku setelah reboot (config disalin saat
  konstruksi). Jadi `NAVL1_PERIOD` di tabel bisa berbeda dari periode yang
  terbang bila diubah tanpa reboot — pakai `l1_period_s` per baris.
- `ail_*_us` = servo aileron KIRI. Kedua servo menerima PWM sama
  (`ActuatorConfig::invert_aileron_*`).

## Baris event (`#EVENT,<time_boot_ms>,<jenis>[,...]`)

| Event | Kapan |
|---|---|
| `ARM` / `DISARM` | saklar arm berubah |
| `MODE,<MANU\|FBWA\|AUTO\|GUID>` | mode berubah (dari saklar RC maupun MAVLink) |
| `WP_REACHED,<seq>,<alasan>` | WP diklaim; alasan = `radius`, `finish_line`, `last_wp_zone`, atau gabungan dengan `+` |
| `PARAM,<nama>,<nilai>` | nilai tabel parameter berubah (resolusi 50 ms) |
| `GPS_LOST` / `GPS_OK` | fix 3D + segar (< 2 s) hilang / pulih, ditahan 1 s (debounce) |
| `DIST_ON,<±us>` / `DIST_OFF` | gangguan roll mulai atau berganti level/sisi (`DIST_ON,+250`, `DIST_ON,-125`) / berakhir |
| `ARM_SNAPSHOT_SKIPPED,buffer_full` | snapshot ARM tidak muat di buffer (jarang) |
| `UTC,<YYYY-MM-DDThh:mm:ss.sssZ>` | sekali per file, saat waktu GPS pertama tersedia: pasangan `time_boot_ms` ↔ UTC |
| `LOG_NAME,<nama>` | sekali per file, setelah file diganti nama ke tanggal |

Setiap event meminta flush kartu, jadi sampai di kartu dalam ≈ 20–50 ms.

`#ARM_SNAPSHOT,<ms>` mengikuti event ARM dan memuat ulang blok
`# EFFECTIVE` / `# PARAM` / `# HOME` / `# WP` supaya setelan dan misi tiap
penerbangan terbukti di dalam file. Nama file aktif juga dikirim ke GCS
(`[SD] ARM log=LOG007.CSV`) setiap ARM.

## Beda dari `saran-sd-logging.md`

| Saran | Yang diimplementasikan | Alasan |
|---|---|---|
| `gps_tow_ms` | `gps_seq` + `gps_age_ms` (+ `gps_new`) | `time_boot_ms` sudah satu jam dengan tlog, jadi penggabungan tetap presisi (keputusan pengguna 2026-10-25). Sejak 2026-10-29 waktu UTC Here4 dibaca, tapi hanya untuk nama file dan event `UTC`, bukan kolom |
| `dist_on`/`dist_us`, `DIST_ON/OFF`, `ail_cmd` ≠ `ail_out` | **diimplementasikan** (`DisturbanceInjector`, stick roll level diskret, maks 3 s, default mati) | Keputusan pengguna 2026-10-25; lihat `docs/disturbance-injection.md` |
| `cte_m` "+ = kanan" | **+ = kiri** | Itu tanda yang sebenarnya dihitung `L1Controller` |
| `FUZZY_ENABLE` di `# PARAM` | **diimplementasikan** sebagai parameter live (juga di `# EFFECTIVE` & kolom `fuzzy_on`) | `saran-yang-harus-dilakukan.md` 2.1 |
| `de_mps` "setelah filter" | tanpa low-pass, tapi kini dari kecepatan GPS (bukan selisih 200 Hz) | `saran-yang-harus-dilakukan.md` 2.3 |
| status kalibrasi BNO055 | kolom `imu_cal` | saran 2.7 |
| `fw=<commit git>` | `fw_tag` (`FC_BUILD_TAG`) + tanggal/jam build | Working tree sering kotor; hash commit akan menyesatkan |
| `NAVL1_PERIOD=14` (contoh) | nilai nyata dari tabel / `l1_period_now` | Default firmware 22 s |
| Buffer 32 KB ("sekitar 4 s") | 64 KB (≈ 9 s) di RAM2 | Baris ±330 B × 20 Hz = 6,6 KB/s |
| `logRow()` banyak argumen | satu `FlightLogSnapshot` | Mudah menambah kolom |

## Tambah kolom baru

1. Tambah field di `FlightLogSnapshot` dan isi di `FlightLog::capture()`.
2. Tambah nama di `kColumns` **dan** specifier + argumen di `writeRow()`
   pada posisi yang sama. Jumlah harus sama (58 sekarang) — pembaca CSV
   menolak baris dengan jumlah kolom salah.
3. Naikkan `format=` di `writeHeader()` dan `FC_BUILD_TAG` di `FC_Config.h`.
4. Perbarui tabel di dokumen ini.

## Uji bangku (belum dilakukan di hardware)

1. Pasang microSD (FAT32), nyalakan FC dengan GPS dan USB; tunggu pesan
   `[SD] logging LOGnnn.CSV ...` di Messages Mission Planner.
2. ARM → DISARM → ganti mode (MANU/FBWA/AUTO) → ubah satu parameter
   (mis. `WP_RADIUS`) dari Mission Planner → DISARM, tunggu 3 s.
3. Buka file lewat MTP atau cabut kartu. Periksa: header lengkap; baris data
   58 kolom; event `ARM`, `MODE`, `PARAM`, `DISARM`; blok snapshot setelah ARM
   (daftar WP sama dengan misi di Mission Planner); `rows_dropped` = 0 di baris
   terakhir; selisih `time_boot_ms` antar baris ≈ 50 ms; jumlah baris ≈
   durasi × 20.
4. Terbang singkat FBWA + AUTO, bandingkan `cte_m`, `wp_dist_m`, `nav_roll_deg`
   dengan `NAV_CONTROLLER_OUTPUT`/`ATTITUDE` di tlog (gabung lewat
   `time_boot_ms`).
5. Setelah file nyata ada, baru sesuaikan skrip analisis
   (`tools/flight_analysis/`) agar membaca CSV ini.

## Prosedur lapangan

**Sebelum sesi:** format kartu FAT32 (≤ 32 GB) atau exFAT (> 32 GB), kelas
A1/A2 merek jelas; pindahkan file lama ke laptop; pasang kartu **sebelum** FC
dinyalakan.

**Setiap penerbangan:** pastikan `[SD] logging LOGnnn.CSV` muncul di
Messages. **Jika tidak muncul, jangan terbang untuk pengambilan data.** Catat:
nomor penerbangan, nama file SD (juga muncul `[SD] ARM log=...` saat ARM), nama
tlog, setelan period/fuzzy, throttle, jam, kondisi angin. Tetap rekam tlog
Mission Planner sebagai cadangan.

**Setelah mendarat:** DISARM, tunggu 2–3 s (flush) sebelum cabut baterai. Ambil
data lewat USB/MTP (hanya saat disarm) atau setelah FC mati; jangan cabut
kartu saat FC menyala. Nama file sudah tanggal-jam (bila GPS dapat waktu);
tambahkan keterangan run, mis. `2026-10-29_08-58-36_F03_P14.CSV`, simpan
bersama tlog yang sama. Bila muncul
`[SD] N rows dropped`, catat jumlahnya.

**Cek cepat setelah unduh:** jumlah baris ≈ durasi × 20; tidak ada lompatan
besar `time_boot_ms`; `armed` berubah sesuai penerbangan; event ARM, MODE,
WP_REACHED ada; snapshot sesuai setelan yang direncanakan.

## Membaca dan menganalisis file ini

```text
tools/flight_analysis/sdlog_io.py            pembaca (baris #, #EVENT, snapshot, kolom -> numpy)
tools/flight_analysis/analyze_sdlog_runs.py  metrik per run AUTO + uji statistik L1 vs fuzzy
tools/flight_analysis/test_sdlog_io.py       uji dengan file sintetis (cocok dengan kolom firmware)
```

```bash
cd tools/flight_analysis
.venv/Scripts/python.exe analyze_sdlog_runs.py LOG007.CSV --out hasil/
.venv/Scripts/python.exe analyze_sdlog_runs.py LOG*.CSV --compare            # Mann-Whitney per level gangguan
.venv/Scripts/python.exe analyze_sdlog_runs.py LOG*.CSV --compare --paired   # Wilcoxon, run berpasangan
```

Definisi metrik (bagian 7.5 saran): **run** = satu rentang berurutan `mode=AUTO`;
**segmen evaluasi** = baris run dengan `leg_from_wp=1` dan GPS 3D (leg masuk dari
posisi pesawat dikeluarkan); **waktu pemulihan** = dari `DIST_OFF` sampai
|cte| ≤ `--band` (default 5 m) bertahan ≥ `--hold` (default 3 s); **overshoot** =
simpangan terbesar di sisi seberang garis setelah gangguan. Tanda `cte_m` di file
ini positif = kiri; skrip tlog lama positif = kanan (`log.cte_right_positive()`).

Skrip ini baru diuji dengan file sintetis; jalankan pada file nyata pertama dan
bandingkan beberapa angka (|cte| rata-rata, jumlah WP) dengan tlog sebelum
dipakai untuk hasil skripsi.
