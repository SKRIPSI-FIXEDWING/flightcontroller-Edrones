# Fuzzy Sugeno untuk periode L1 (`fc::FuzzySugenoL1`)

Ditambahkan 2026-11-13. Fuzzy Sugeno orde-0 yang mengubah **periode L1** selama
mode AUTO berdasarkan jarak pesawat ke garis lintasan (\|e\|) dan laju
perubahannya (d\|e\|/dt). Desainnya (himpunan, rule base, batas normalisasi)
berasal dari data uji L1 konvensional 30/10/2026:
`tools/flight_analysis/fuzzy_sugeno_2026-10-30/DESAIN_FUZZY_SUGENO.md`.

Fuzzy ini **tidak menggantikan L1**. L1 tetap menghitung perintah belok; fuzzy
hanya memilih periode yang dipakai L1.

## 1. Cara pakai

| Param | Nilai | Arti |
|---|---|---|
| `FUZZY_ENABLE` | **0** (bawaan) | L1 konvensional: periode tetap = `NAVL1_PERIOD` |
| | **1** | L1 + fuzzy: periode 11–19 s dipilih fuzzy tiap siklus AUTO |

- Diubah dari Mission Planner (Config → Full Parameter List), **berlaku langsung**
  tanpa reboot, dan tersimpan di EEPROM.
- Saat `FUZZY_ENABLE = 1`, nilai `NAVL1_PERIOD` **tidak dipakai** di AUTO. Periode
  dasar fuzzy tetap 15 s (`kP0` di kode).
- Di luar AUTO (MANUAL, FBWA, GUIDED, RTL) dan sesudah misi selesai, periode
  selalu `NAVL1_PERIOD`.

**Sebelum setiap terbang**, cek nilai `FUZZY_ENABLE` di Mission Planner.
Setelah flash firmware, nilainya bisa terbawa dari EEPROM lama (lihat bagian 8).

**Memeriksa di log SD** (`docs/flight-log-csv.md`, format 4):

| Yang dicek | Di mana |
|---|---|
| Fuzzy aktif atau tidak | kolom `fuzzy_on` (1 = aktif) |
| Periode yang dipakai L1 | kolom `l1_period_s` |
| Keluaran ternormalisasi | kolom `fuzzy_z` (periode = 15 + 4 × `fuzzy_z`) |
| Masukan fuzzy | kolom `cte_m` (e) dan `de_mps` (d\|e\|/dt) |
| Nilai param saat log dibuka | baris `# EFFECTIVE ... fuzzy_enable=` |

## 2. Alur dalam satu siklus AUTO

```
ModeAuto::_update()
  │
  ├─ navigation.navigate()            L1 menghitung e, d(e)/dt, nav_roll
  │                                   (memakai periode yang dipasang siklus SEBELUMNYA)
  │
  ├─ fuzzy.update(boleh, e, de/dt)    fuzzy menghitung periode baru
  │     ├─ FUZZY_ENABLE = 0 atau misi selesai → kembalikan NAVL1_PERIOD
  │     └─ FUZZY_ENABLE = 1                   → kembalikan periode fuzzy (11..19 s)
  │
  ├─ l1.setPeriod(periode)            dipakai L1 pada siklus BERIKUTNYA
  │
  └─ TECS, attitude, aktuator         (tidak berubah)

ModeAuto::_exit()
  └─ l1.setPeriod(NAVL1_PERIOD)       mode lain selalu memakai periode tetap
```

Fuzzy berjalan setelah L1 karena butuh e dan laju e dari siklus ini. Akibatnya
periode baru terpakai satu siklus kendali kemudian (beberapa milidetik), jauh
lebih cepat dari perubahan GPS (5–10 Hz).

## 3. File yang terlibat

| File | Isi |
|---|---|
| `include/navigation/FuzzySugenoL1.h` | struktur `FuzzyL1Output`, fungsi `fuzzySugenoL1()`, kelas `FuzzySugenoL1` |
| `src/navigation/FuzzySugenoL1.cpp` | **seluruh perhitungan fuzzy** (≈ 100 baris) dan parameter desainnya |
| `src/modes/FixedWingModes.cpp` | dipanggil di `ModeAuto::_update()`, dikembalikan ke periode tetap di `ModeAuto::_exit()` |
| `include/storage/Params.h`, `src/storage/Params.cpp` | param `FUZZY_ENABLE` (`FlightExperimentConfig::fuzzy_enable`) |
| `include/modes/VehicleContext.h`, `src/main.cpp` | objek global `g_fuzzy` dan referensinya di `VehicleContext` |
| `src/communication/FlightLog.cpp` | kolom log `fuzzy_on`, `fuzzy_z`, dan `fuzzy_enable` di baris `# EFFECTIVE` |
| `test/host/test_fuzzy_sugeno_l1.cpp` | uji di laptop (bagian 7) |

Modul fuzzy tidak memakai library apa pun, tidak bergantung pada Arduino, dan
tidak memanggil `L1Controller` sendiri. Masukannya dua angka, keluarannya satu
angka; pemasangan ke L1 dilakukan di `ModeAuto`.

## 4. Isi kode, langkah demi langkah

Semua ada di `fuzzySugenoL1(abs_e_m, de_mps)` di `src/navigation/FuzzySugenoL1.cpp`.
Langkahnya sama dengan dokumen desain.

| Langkah | Kode | Rumus |
|---|---|---|
| 0. Normalisasi | `out.e_fn`, `out.de_fn` | e_fn = clip(\|e\|/20, 0, 1); de_fn = clip(d\|e\|/dt / 13, −1, 1) |
| 1. Fuzzifikasi | `out.mu_e[3]`, `out.mu_de[3]` | himpunan segitiga/trapesium, titik 0,25/0,5/0,75 dan −0,5/0/0,5 |
| 2. Rule base | tabel `kRuleC[3][3]` | konstanta c tiap aturan |
| 3. Inferensi | `out.alpha[i][j] = fminf(mu_e[i], mu_de[j])` | α = min(μ_e, μ_de) |
| 4. Defuzzifikasi | `out.z`, `out.period_s` | z = Σα·c / Σα; periode = 15 + 4·z |

Fungsi bantu `rise(x, a, b)` adalah sisi naik himpunan: 0 di a, 1 di b,
dipotong ke 0..1. Dengan itu setiap himpunan ditulis satu baris:

```
Kecil   = 1 − rise(u, 0.25, 0.5)
Sedang  = min( rise(u, 0.25, 0.5), 1 − rise(u, 0.5, 0.75) )
Besar   = rise(u, 0.5, 0.75)
Negatif = 1 − rise(y, −0.5, 0)
Zero    = min( rise(y, −0.5, 0), 1 − rise(y, 0, 0.5) )
Positif = rise(y, 0, 0.5)
```

Hasilnya sama persis dengan rumus bercabang di catatan dan di dokumen desain.

**Masukan d\|e\|/dt** dibuat di `FuzzySugenoL1::update()` dari dua nilai L1:

```
d|e|/dt = tanda(e) × L1Controller::crosstrackVelocityMps()
```

`crosstrackVelocityMps()` adalah kecepatan GPS tegak lurus garis (m/s), besaran
yang sama dengan kolom `de_mps` di log dan yang dipakai saat desain. Negatif =
mendekati garis di sisi mana pun.

## 5. Parameter desain dan cara mengubahnya

Semua angka desain ada di bagian atas `src/navigation/FuzzySugenoL1.cpp`:

| Konstanta | Nilai | Arti |
|---|---|---|
| `kEMax` | 20 m | E_MAX, batas normalisasi \|e\| |
| `kDeMax` | 13 m/s | DE_MAX, batas normalisasi d\|e\|/dt |
| `kP0` | 15 s | periode dasar (z = 0) |
| `kS` | 4 s | faktor skala: z = −1…+1 → 11…19 s |
| `kRuleC[3][3]` | lihat di bawah | rule base (konstanta c) |

Rule base terpasang (sama dengan DESAIN bagian 0):

| \|e\| \ d\|e\|/dt | Negatif | Zero | Positif |
|---|---|---|---|
| Kecil | −1 (11 s) | −1 (11 s) | −1 (11 s) |
| Sedang | 0 (15 s) | 0 (15 s) | 0 (15 s) |
| Besar | +0,5 (17 s) | 0 (15 s) | 0 (15 s) |

**Contoh mengubah:** keputusan terbuka di DESAIN bagian 9.1 adalah baris Kecil
11 s atau 15 s. Untuk versi 15 s, ganti baris pertama `kRuleC` menjadi
`{0.0f, 0.0f, 0.0f}`. Setelah mengubah angka apa pun:

1. Perbarui dokumen desain supaya naskah dan kode sama.
2. Sesuaikan uji di `test/host/test_fuzzy_sugeno_l1.cpp`. Uji putar ulang
   membandingkan dengan CSV desain Python, jadi jalankan ulang juga
   `design_fuzzy_sugeno.py` dengan rule base yang sama.
3. Naikkan `FC_BUILD_TAG` di `include/FC_Config.h` supaya log bisa dibedakan.
4. Build dan flash.

## 6. Kapan fuzzy aktif

| Kondisi | Periode L1 | `fuzzy_on` |
|---|---|---|
| AUTO, `FUZZY_ENABLE = 1`, misi berjalan | hasil fuzzy (11–19 s) | 1 |
| AUTO, `FUZZY_ENABLE = 0` | `NAVL1_PERIOD` | 0 |
| AUTO, misi selesai (loiter / RTL setelah WP terakhir) | `NAVL1_PERIOD` | 0 |
| MANUAL, FBWA, GUIDED, RTL | `NAVL1_PERIOD` | 0 |
| `FUZZY_ENABLE` diganti saat terbang | berlaku pada siklus berikutnya | ikut berubah |

Fuzzy juga aktif di leg pertama (dari posisi saat AUTO dinyalakan ke WP1) dan
tepat setelah ganti WP. Data desain tidak memakai leg pertama, tetapi aturannya
tetap masuk akal di sana (DESAIN bagian 9.3).

## 7. Pengujian

### Uji di laptop (sudah dijalankan, lulus)

Dari root repo:

```bash
g++ -std=c++17 -Iinclude test/host/test_fuzzy_sugeno_l1.cpp src/navigation/FuzzySugenoL1.cpp -o test_fuzzy && ./test_fuzzy
```

Yang diuji:

1. Tiga contoh hitung tangan di DESAIN bagian 6 (11,0 s; 15,49 s; 17,0 s).
2. Seluruh semesta (\|e\| 0–50 m, d\|e\|/dt −25…25 m/s): jumlah μ tiap variabel = 1,
   Σα > 0, periode selalu 11–17 s.
3. Saklar: `FUZZY_ENABLE` 0/1 berlaku langsung, `allowed = false` mengembalikan
   periode tetap, perubahan `NAVL1_PERIOD` ikut terbaca.
4. Tanda d\|e\|/dt: pesawat yang mendekat dari kiri dan dari kanan garis
   memberi periode yang sama.
5. **Putar ulang 1.894 sampel terbang 30/10/2026**: hasil kode C++ sama dengan
   desain Python (selisih maksimum 0,0005 s, efek pembulatan CSV).

### Build firmware

`pio run` lulus (Teensy 4.1).

### Belum dilakukan

- **Uji bangku di hardware**: nyalakan FC di meja dengan GPS, masuk AUTO, ubah
  `FUZZY_ENABLE`, lalu pastikan `l1_period_s` dan `fuzzy_on` di log berubah.
- **Uji terbang**: bandingkan L1 + fuzzy dengan L1 konvensional (minimal 11 s dan
  15 s) dengan misi, parameter, dan throttle yang sama. Analisis log SD:
  `tools/flight_analysis/analyze_sdlog_runs.py` membaca kolom `fuzzy_on`.

## 8. Hal yang perlu diperhatikan

1. **Baris Kecil = 11 s masih keputusan terbuka** (DESAIN bagian 9.1). Cara
   menggantinya ada di bagian 5.
2. **Tidak ada pembatas laju perubahan periode.** Periode bisa turun 17 → 11 s
   dalam ±1 s saat pesawat memotong garis (DESAIN bagian 9.2). Sengaja tidak
   ditambahkan supaya kode sama persis dengan desain; tambahkan hanya bila uji
   terbang menunjukkan nav_roll tersentak.
3. **Nilai `FUZZY_ENABLE` setelah flash.** Bila EEPROM masih menyimpan tabel
   param lama berurutan (firmware sebelum blok bernama), loader memetakan nilai
   berdasarkan nama, termasuk `FUZZY_ENABLE` dari firmware lama. Selalu cek di
   Mission Planner (lihat juga catatan 26/10 di `docs/params.md`).
4. **`NAVL1_PERIOD` diabaikan saat fuzzy aktif.** Periode dasar fuzzy ditetapkan
   15 s dari data, tidak mengikuti param.
5. **Masukan tanpa filter.** d\|e\|/dt diambil langsung dari kecepatan GPS, sama
   seperti saat desain. Derau cepat e pada data uji hanya σ 0,4 m.
