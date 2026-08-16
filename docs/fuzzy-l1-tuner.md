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

## Input/Output (dikonfirmasi bersama pengguna)

- **Input 1 — crosstrack error `e`** (meter, nilai absolut): seberapa jauh
  penyimpangan posisi wahana dari jalur acuan L1.
- **Input 2 — laju perubahan crosstrack error `Δe`** (m/s, nilai absolut): dihitung
  internal dari selisih `e` antar panggilan `update()` dibagi `dt`.
- **Output — faktor skala L1 period**: dikalikan ke `base_period_s` untuk
  menghasilkan period yang benar-benar dipakai `L1Controller::setPeriod()`.
  Period lebih kecil = L1 lebih agresif (jarak L1 lebih pendek, belok lebih
  tajam mengejar jalur); period lebih besar = lebih halus.

## Rule base (starting point, perlu divalidasi simulasi/data terbang)

| `e` \\ `Δe` | kecil | sedang | besar |
| --- | --- | --- | --- |
| **kecil** | halus (gentle) | normal | normal |
| **sedang** | normal | normal | agresif |
| **besar** | agresif | agresif | agresif |

Logikanya: penyimpangan kecil + laju kecil → wahana sudah stabil di jalur, pakai
period besar (halus, hindari osilasi/chattering). Penyimpangan besar dan/atau
melebar cepat → butuh koreksi cepat, pakai period kecil (agresif).

**Breakpoint membership function** (`FuzzyL1TunerConfig`) adalah **titik awal**,
bukan hasil tuning akhir — nilainya dipilih berdasarkan estimasi kasar geometri L1
(pada period=22s, damping=0.73, cruise 18 m/s, jarak L1 ≈ 92 m), belum divalidasi
lewat simulasi atau data terbang sungguhan. Sesuaikan `e_small`/`e_medium`/
`e_large`/`de_small`/`de_medium`/`de_large` berdasarkan crosstrack error yang
benar-benar teramati saat pengujian.

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
