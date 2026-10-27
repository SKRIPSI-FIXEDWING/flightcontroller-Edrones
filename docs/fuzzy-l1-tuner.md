# Fuzzy L1-Period Self-Tuner — INTI SKRIPSI

**Modul baru sepenuhnya** — tidak ada padanan di program KHAGESWARA lama. Ini adalah
kontribusi utama skripsi: *"KENDALI PATH FOLLOWING PESAWAT TANPA AWAK SAYAP TETAP
BERBASIS SELF-TUNING PADA KONDISI GANGGUAN ROLL EKSTERNAL"*.

Audit kode lama mengonfirmasi **fuzzy logic sebelumnya tidak pernah menyentuh
guidance/L1 sama sekali** — `Fuzzy_FW_Roll.h`/`Fuzzy_FW_Pitch.h` lama hanya
menyetel *gain* PID rate-attitude (roll/pitch), dan bahkan itu pun dormant (task
`fwRoll` yang memanggilnya di-comment di `main.cpp`, `setup_fuzzy_pitch()` malah
tidak pernah dipanggil sama sekali). Modul ini memakai ulang **pola library eFLL
yang sama persis** dari kode dormant tersebut, tapi retargeted ke parameter L1
period, bukan gain PID.

## Struktur modul

```text
include/navigation/FuzzyL1Tuner.h   Kontrak class FuzzyL1Tuner
src/navigation/FuzzyL1Tuner.cpp     Implementasi fuzzy (eFLL) + rule base
```

## Input/Output (sejak 2026-10-26 = proposal Tabel 3.4-3.6)

- **Input 1 — `|e|`** (meter): besar crosstrack error. Di loiter = jarak ke
  pusat − radius; di AUTO = jarak ke garis leg. Himpunan **Kecil/Sedang/Besar**.
- **Input 2 — `Δe = d|e|/dt`** (m/s, **bertanda**): `tanda(e) ×
  L1Controller::crosstrackVelocityMps()`. Negatif = mendekati lintasan,
  positif = menjauh. Himpunan **Negatif/Nol/Positif**. Dicatat di log kolom
  `de_mps`. (Tidak dihitung dari selisih |e| antar siklus 200 Hz karena posisi
  GPS hanya 5 Hz.)
- **Output — `ΔT`** (detik): periode yang dipakai = `NAVL1_PERIOD + ΔT`,
  dibatasi `[FUZZY_MIN_PER, FUZZY_MAX_PER]`. Himpunan **Turun/Tetap/Naik**
  (puncak −2/0/+2 s). Turun = periode kecil = L1 lebih agresif. Dicatat di
  log kolom `fuzzy_out`.
- Inferensi Mamdani eFLL: AND = min, agregasi = max, defuzzifikasi centroid.

## Rule base (proposal Tabel 3.6)

| `Δe` \\ `|e|` | Kecil | Sedang | Besar |
| --- | --- | --- | --- |
| **Negatif** | Naik | Tetap | Turun |
| **Nol** | Tetap | Tetap | Turun |
| **Positif** | Tetap | Turun | Turun |

## Fungsi keanggotaan dari data terbang

Diturunkan oleh `tools/flight_analysis/fuzzy_mf_from_data.py` dari 5 terbang
2026-10-26 dengan `NAVL1_PERIOD` 12, 13, 14, 15, 16 s (`flight_2026-10-26_b`,
`_d`, `_e`, `flight_2026-10-26`, `_f`; 2811 sampel 5 Hz, 2201 di antaranya
loiter). Gambar (untuk Gambar 3.6-3.8 proposal) dan `breakpoint.csv` ada di
`tools/flight_analysis/fuzzy_mf_2026-10-26/`.

| Breakpoint | Nilai | Asal |
|---|---|---|
| Kecil penuh sampai | 3 m | median `|e|` loiter di dalam toleransi |
| Puncak Sedang | 10 m | batas toleransi (pita uji loiter, `--tol` analisis) |
| Besar penuh mulai | 30 m | median `|e|` semua sampel di luar toleransi |
| Nol penuh di | ±1,25 m/s | persentil 75 `|Δe|` loiter di dalam toleransi |
| Negatif/Positif penuh di luar | ±2,25 m/s | persentil 90 `|Δe|` loiter di dalam toleransi |
| Turun/Naik | ±2 s | `NAVL1_PERIOD` 14 ± 2 = 12-16 s, rentang yang sudah diterbangkan |

Bentuk: segitiga untuk himpunan tengah, trapesium bahu untuk himpunan tepi
(nilai di luar semesta dijenuhkan, bukan nol). Konstanta ada di
`FuzzyL1TunerConfig` (`include/navigation/FuzzyL1Tuner.h`).

Rekomendasi param untuk run fuzzy: `NAVL1_PERIOD = 14` (galat loiter steady
terkecil pada sweep: RMS 2,8 m), `FUZZY_MIN_PER = 12`, `FUZZY_MAX_PER = 16`.
Run pembanding L1 tetap memakai `NAVL1_PERIOD = 14` yang sama.

Catatan: data 2026-10-26 belum memuat gangguan terkendali; deviasi besar
berasal dari masuk loiter dan pergantian WP. Setelah ada run gangguan CH7
(`docs/disturbance-injection.md`), jalankan ulang script dengan folder
terbang baru untuk memeriksa breakpoint Besar dan Δe.

## A/B toggle: L1 konvensional vs L1 + fuzzy

Ini **sumbu perbandingan skripsi** — bukan dua firmware terpisah, tapi satu
saklar runtime:

```cpp
#include "navigation/FuzzyL1Tuner.h"
#include "navigation/L1Controller.h"

fc::L1Controller l1;
fc::FuzzyL1Tuner fuzzy_tuner;

void setupNavigation()
{
    fuzzy_tuner.begin();
    fuzzy_tuner.setEnabled(true);  // false = L1 konvensional (period tetap)
}

void updateNavigation(float dt_s)
{
    // ... l1.updateWaypoint(...) dipanggil dulu untuk mendapatkan crosstrack error terbaru ...
    fuzzy_tuner.update(l1, l1.crosstrackError(), dt_s);
    // l1.period() sekarang sudah disetel fuzzy_tuner (atau tetap di base_period_s
    // kalau setEnabled(false)).
}
```

Saat `setEnabled(false)`, `FuzzyL1Tuner::update()` **tidak memanggil**
`l1.setPeriod()` — period tetap pada apa pun yang sudah dikonfigurasi
(`L1ControllerConfig::period_s`, mode konvensional).

## Urutan pemanggilan penting

`fuzzy_tuner.update()` harus dipanggil **setelah** `l1.updateWaypoint()`/
`updateLoiter()` pada iterasi yang sama, supaya crosstrack error yang diumpankan
adalah nilai terbaru — dan **sebelum** `l1.navRollCd()` dibaca untuk perintah roll,
supaya period yang sudah disetel fuzzy dipakai pada iterasi kontrol yang sama
(bukan tertunda satu iterasi).

## File terkait

```text
include/Fuzzy_FW_Roll.h / Fuzzy_FW_Pitch.h   Pola eFLL yang dipakai ulang, TIDAK dimigrasikan
                                               (dormant, hanya menyetel gain PID attitude)
```

## Perubahan 2026-10-25 (eksperimen skripsi)

- **`FUZZY_ENABLE`** adalah parameter (Mission Planner), berlaku langsung tanpa
  reboot, tersimpan di EEPROM; nilai awal dari `FC_FUZZY_L1_ENABLE`. Mematikan
  fuzzy (`applyEnabled(false)`) langsung mengembalikan periode L1 ke periode
  dasar; setiap masuk AUTO/GUIDED periode juga dimulai dari periode dasar
  (`resetToBase()`), jadi run fuzzy tidak mewarisi periode run sebelumnya.
- **`base_period_s` mengikuti `NAVL1_PERIOD`** (saat boot dan saat param diubah),
  bukan lagi konstanta 22: himpunan "Normal" fuzzy berpusat di periode yang
  dipakai pembanding L1 tetap, sehingga perbandingan adil.
- **`FUZZY_MIN_PER` / `FUZZY_MAX_PER`** berlaku langsung (`setPeriodLimits()`).
- (Diganti 2026-10-26, lihat di bawah.)

## Perubahan 2026-10-26

- Rule base diganti persis Tabel 3.6 proposal; Δe sekarang **bertanda**
  (sebelumnya `|Δe|`, sehingga mendekati dan menjauhi lintasan tidak bisa
  dibedakan).
- Keluaran diganti dari pengali `T = base × skala` menjadi `T = base + ΔT`
  (tetap pemetaan langsung, bukan inkremental `T(k−1) + ΔT`, jadi tidak ada
  windup). Kolom log `fuzzy_out` sekarang berisi ΔT (s).
- Fungsi keanggotaan dari data terbang (tabel di atas); default
  `FUZZY_MIN_PER/MAX_PER` 12/16 s (hanya untuk EEPROM baru).
- Fuzzy juga di-reset ke periode dasar saat masuk LOITER, dan kolom navigasi
  log SD (`cte_m`, `de_mps`, `fuzzy_out`, ...) sekarang terisi juga di
  LOITER/RTL.
- Uji unit host: `test/host/test_fuzzy_l1_tuner.cpp` (eFLL asli + stub L1).
