# Catatan perbaikan dan perubahan gangguan otomatis (DIST_ENABLE = 2)

Dicatat: 15 November 2026, diperbarui 16 November 2026. **Status 16 November 2026: sudah dikerjakan di kode** (butir 1, 2, 3, 7 daftar pekerjaan): `along_m` dihitung dari posisi, `kAmp` = 150/300 µs, belok selalu berlawanan, uji unit diperbarui dan lulus. `DIST_MS` tetap 2000 ms. Ambang batal 60 m dipertahankan. Dokumen `disturbance-injection.md` dan `flight-log-csv.md` sudah diperbarui (tabel S1 dan S2 baru). Firmware perlu di-build dan di-upload ulang.
Sumber temuan: analisis 5 terbang 15 November 2026, lihat `tools/flight_analysis/flight_2026-11-15_gangguan_5x/laporan-lengkap-5-terbang-gangguan.md`.

## 1. Kesalahan yang ditemukan

### 1.1 Bug utama: event gangguan lurus (E1) selalu di-skip

- **Gejala:** di kelima terbang (DIST_FLT 1 – 5) hanya event belok (E2) yang sampai ke servo. E1 di leg lurus WP1→WP2 tidak pernah dikirim. Telemetri servo membuktikannya: residual servo terhadap kontroler 0 µs di jendela E1, dan ±100 sampai ±270 µs di jendela E2.
- **Penyebab:** di `ModeAuto::_update` (`src/modes/FixedWingModes.cpp:107-125`), `navigate()` dipanggil dulu. Saat WP diklaim, `flag_wp` naik dan leg diganti (`setNextWaypoint`), tetapi `distance_next_wp` hanya dihitung di awal `navigate()` (`src/navigation/Navigation.cpp:472`) dan tidak dihitung ulang. Di siklus pertama setelah WP1 diklaim, `along_m = panjang leg − distance_next_wp` memakai jarak lama ke WP1 (±15 m), jadi `along_m ≈ 285 m`.
- **Akibat:** di `DisturbanceScript::update` (`src/control/DisturbanceScript.cpp:134-135`) nilai itu melewati `end_m` (150 m), sehingga E1 langsung `skip(1)` ("tidak tenang"). Indeks maju ke E2, dan E2 menembak normal. Satu siklus 50 Hz sudah cukup.
- **Cakupan:** skenario 3 WP (`wp_sum = 4`): E1 hilang. Skenario 6 WP (`wp_sum = 7`): semua event lurus (L1 dan L5) kena. Event belok tidak terpengaruh.
- **Kenapa tidak tertangkap uji:** `test/host/test_disturbance_script.cpp` memberi `along_m` yang naik mulus dari 0, tanpa sampel basi di awal leg.
- **Konfirmasi tambahan yang masih bisa diambil:** CSV SD (LOG029 – LOG034) seharusnya berisi `#EVENT,...,DIST_SKIP,E1,reason=1` tepat saat WP1 diklaim di tiap terbang dan tidak ada `DIST_ON,...,E1`.

### 1.2 Kesalahan dalam analisis (diperbaiki)

- Sempat disimpulkan telemetri servo tidak memuat offset gangguan. Salah: `SERVO_OUTPUT_RAW` membaca `aileron_left_.readMicroseconds()`, yaitu nilai yang ditulis ke servo, termasuk offset.
- Sempat disimpulkan event lurus mungkin terkirim tetapi pesawat tidak merespons. Salah: E1 tidak pernah dikirim.
- Perkiraan respons roll awal (125 µs ≈ 12°) terlalu kecil; ukuran yang benar ±0,13 sampai 0,17 °/µs (125 µs ≈ 16 – 22°, 250 µs ≈ 30 – 39°).
- Metrik `pemulihan_s` bawaan `analyze_tlog_l1_auto.py` mencatat waktu pertama masuk pita ±5 m, bisa 0 s walau CTE kemudian keluar lagi. Untuk perbandingan dipakai "keluar pita ±5 m terakhir".

## 2. Perubahan yang diminta (keputusan Anda, 16 November 2026)

| Item | Sekarang | Diminta |
|---|---|---|
| Amplitudo level rendah | 125 µs | **150 µs** |
| Amplitudo level tinggi | 250 µs | **300 µs** |
| Durasi pulsa `DIST_MS` | 2000 ms | **tetap 2000 ms** (sempat dicatat 3000 ms, dibatalkan 16 Nov 2026; tidak berubah) |
| Berlaku di | leg lurus dan belok | **leg lurus dan belok** |
| Arah gangguan di belok | searah **dan** berlawanan arah belok | **selalu berlawanan arah belok**; tidak ada lagi gangguan searah belok |

Arti "berlawanan arah belok": belok WP2 adalah belok kiri, jadi gangguan di belok selalu **roll kanan** (mengurangi bank). Tanda ditentukan otomatis dari arah belok yang dihitung `next_turn_angle_deg`, bukan angka tetap.

### 2.1 Usulan tabel 5 terbang (BELUM dikonfirmasi)

Anda belum menyebut kombinasi per nomor terbang. Karena tidak ada lagi gangguan searah belok, tabel lama (A1 … A5 dengan tanda relatif belok) perlu disusun ulang. Usulan, mengikuti pola lama untuk leg lurus:

| DIST_FLT | E1 leg lurus (tanda mutlak, + = roll kanan) | E2 belok (selalu berlawanan belok) |
|---|---|---|
| 1 | +150 µs | 150 µs |
| 2 | −150 µs | 150 µs |
| 3 | +300 µs | 300 µs |
| 4 | −300 µs | 300 µs |
| 5 | +300 µs (ulangan, untuk keterulangan) | 300 µs |

Yang perlu Anda putuskan: (a) apakah pola tanda di leg lurus seperti di atas, (b) apa isi F5 (ulangan F3 atau kombinasi lain), (c) skenario 6 WP (`wp_sum = 7`) bila nanti dipakai, tabelnya juga perlu disusun ulang dengan aturan belok selalu berlawanan.

## 3. Daftar pekerjaan (nanti, setelah disetujui)

1. **Perbaiki `along_m`** (`src/modes/FixedWingModes.cpp:119-122`): hitung dari posisi, bukan dari `distance_next_wp` yang bisa basi, misalnya `mission.prev_wp_loc.get_distance(mission.current_loc)`. Alternatif: hitung ulang `distance_next_wp` setelah `setNextWaypoint`.
2. **Ubah tabel amplitudo** `kAmp` di `src/control/DisturbanceScript.cpp:27` dan komentar di baris 25 sesuai tabel final (bagian 2.1).
3. **Belok selalu berlawanan:** di `DisturbanceScript.cpp:147-151`, tanda event belok tidak lagi mengikuti tanda `kAmp`; pakai besar amplitudo (`|A|`) dan arahkan roll **berlawanan** arah belok (belok kiri → roll kanan, +). Event dengan arah belok tidak diketahui tetap di-skip (alasan 5).
4. **Durasi tetap 2000 ms:** `DIST_MS` tidak diubah (default di `src/storage/Params.cpp:238` sudah 2000). Batas `kDistMaxDurationMs = 3000` dan `kDistMaxAmplitudeUs = 400` (`include/FC_Config.h:283-284`) sudah cukup (300 µs < 400 µs) dan tidak perlu diubah.
5. **Jendela waktu event:** karena durasi tidak berubah (2 s), jendela event lama tetap berlaku dan tidak perlu ditinjau ulang.
6. **Tinjau ambang batal event belok** `kDistAbortTurnM = 60 m` (`include/FC_Config.h:294`): lihat risiko bagian 4.
7. **Perbarui uji unit:** `test/host/test_disturbance_script.cpp` (nilai amplitudo, `dur` 2000 → 3000, dan ekspektasi tanda belok yang kini selalu berlawanan) serta `test/host/test_disturbance_injector.cpp` bila perlu. **Tambahkan kasus** satu sampel `along_m ≈ 285 m` di awal leg (sampel basi), yang harus tetap menembak E1 di sekitar 90 m.
8. **Perbarui dokumen:** `docs/disturbance-injection.md` (baris 7, 89, 97-98, 121), `docs/params.md`, `docs/flight-log-csv.md` (contoh `DIST_ON`), serta analisis di `tools/flight_analysis` bila nilainya disebut.
9. **Ulangi uji terbang** (lihat bagian 5).

## 4. Risiko dan pertanyaan terbuka

- **Overshoot belok bisa membesar.** Data lama: gangguan berlawanan belok 250 µs selama 2 s menghasilkan overshoot 25,6 m (F4), dan 125 µs hanya 17,9 m (baseline 16,2 m). Dengan 300 µs selama 2 s (durasi sama dengan data lama), roll aktual tertinggal dari perintah L1 sekitar 39 – 51° selama 2 s. **Perkiraan kasar (ekstrapolasi dari F4, bukan hasil ukur): overshoot sekitar 30 – 40 m**, mendekati batas evaluasi 45 m tetapi kemungkinan masih di bawah ambang batal `kDistAbortTurnM = 60 m`. Perlu diputuskan apakah ambang 60 m dipertahankan; karena durasi tidak lagi 3 s, risikonya lebih kecil dibanding perkiraan sebelumnya (40 – 60 m).
- **Ruang manuver:** pesawat bisa menyimpang jauh dari garis di sisi luar belok; pastikan area terbang aman dan pilot siap mengambil alih.
- **Risiko roll terlalu besar (bank berlebih) berkurang**, karena tidak ada lagi gangguan searah belok (di data lama F3 −59,8° dan F5 −49,3°). Namun roll ke kanan saat perintah L1 ke kiri selama 2 s menghasilkan kombinasi attitude yang tidak biasa; pantau kecepatan dan pitch.
- **Persetujuan pembimbing:** 125/250 µs dan 2 s berasal dari Tabel 3.7 proposal; perubahan ke 150/300 µs dan hanya berlawanan belok (durasi tetap 2 s) perlu disetujui atau dicatat sebagai revisi.
- **Efek gangguan lurus belum pernah terukur.** Sampai bug diperbaiki dan terbang diulang, tidak ada data respons jalur terhadap gangguan di leg lurus. Respons roll per µs dari data belok (±0,13 – 0,17 °/µs) bisa dipakai sebagai perkiraan.

## 5. Dampak ke rencana eksperimen

- Lima terbang 15 November 2026 (125/250 µs, 2 s, hanya E2 yang efektif, campuran searah dan berlawanan) tetap berguna sebagai data awal tetapi **tidak sebanding** dengan data baru.
- Karena perbandingan akhir adalah L1 tetap vs L1 + fuzzy pada kondisi yang sama, setelah perbaikan dan perubahan nilai ini **kedua kondisi harus diterbangkan ulang dengan nilai yang sama**:
  1. 1× tanpa gangguan, tanpa fuzzy (baseline ulang bila kondisi lapangan berbeda).
  2. 5× gangguan (DIST_FLT 1 – 5), tanpa fuzzy.
  3. 5× gangguan (DIST_FLT 1 – 5), dengan fuzzy.
- Karena belok kini selalu berlawanan, semua terbang memang diharapkan memperburuk lintasan, jadi perbandingan dengan fuzzy lebih bermakna dibanding data lama (yang tercampur gangguan yang malah memperkecil overshoot).
- Selama terbang catat IAS dan GS (kecepatan udara di leg belok pada data lama berkisar 14,3 – 21,3 m/s dan memengaruhi overshoot).
- Satu baseline saja tidak memberi gambaran variasi alami; pertimbangkan beberapa baseline.
