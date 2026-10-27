# Injeksi gangguan roll (`fc::DisturbanceInjector`)

Untuk eksperimen skripsi *kendali loitering pada kondisi gangguan eksternal*
(proposal 1.3: gangguan diberikan manual lewat remote controller).

- 2026-10-25: versi pertama, dipicu stick roll (dua level ±125/±250 µs, lama
  = selama stick ditahan, maks 3 s), hanya di AUTO.
- **2026-10-26: dipicu switch CH7, satu pulsa tetap per pemicu, di AUTO dan
  LOITER.** Alasan: gangguan dari stick bergantung tangan pilot (besar dan
  lamanya tidak pernah persis sama antar run), padahal perbandingan L1 tetap
  vs L1 + fuzzy butuh gangguan yang identik.

Kode: `include/control/DisturbanceInjector.h`, `src/control/DisturbanceInjector.cpp`,
`Actuator::writeAttitude(..., aileron_disturbance_us)`, `ModeAuto::_update()`,
`ModeLoiter::_update()`. Uji unit: `test/host/test_disturbance_injector.cpp`.

## Cara kerja

Selama `DIST_ENABLE = 1`, setiap perpindahan switch **CH7 (ch[6]) dari RENDAH
ke TINGGI** menyuntik **satu pulsa** gangguan:

| Param | Default | Arti |
|---|---|---|
| `DIST_ENABLE` | 0 | 1 = gangguan boleh dipicu |
| `DIST_US` | 250 | besar pulsa, µs, **bertanda** (+ = roll kanan, − = roll kiri), dibatasi ±400 |
| `DIST_MS` | 2000 | lama pulsa, ms, dibatasi 3000 |

- CH7 **RENDAH** = < 1300 µs, **TINGGI** = > 1700 µs (`kDistSwitchLowUs/HighUs`).
- Besar dan lama pulsa **dikunci saat pulsa mulai**, dan pulsa berjalan
  penuh selama `DIST_MS` walaupun switch dikembalikan. Jadi setiap pemicu
  menghasilkan gangguan yang sama persis.
- Offset ditambahkan ke perintah aileron **sesudah** kendali sikap dan
  **sebelum** cermin diferensial (sama seperti stick di mode manual). Di log,
  `ail_out_us − ail_cmd_us` bernilai negatif untuk `dist_us > 0`.
- Arah pada loiter: untuk loiter CW (searah jarum jam, belok kanan), `DIST_US`
  positif mendorong pesawat **ke dalam** lingkaran, negatif **ke luar**.
  Tetapkan satu arah untuk seluruh run pembanding.

## Aturan keselamatan (semuanya dipaksa di dalam kelas)

1. **Mati kecuali `DIST_ENABLE = 1`** (default 0, berlaku live). Set 0 dari
   Mission Planner menghentikan pulsa seketika.
2. **Hanya di AUTO/LOITER, ARMED, sinyal RC hidup.** Salah satu hilang →
   pulsa berhenti pada siklus itu juga.
3. **CH7 harus RENDAH ≥ 300 ms** (`kDistSwitchLowHoldMs`) setelah masuk mode
   atau setelah pulsa selesai, baru perpindahan ke TINGGI dihitung. Switch
   yang sudah TINGGI saat mode aktif tidak melakukan apa-apa; menahan TINGGI
   tidak memicu ulang.
4. Kanal yang tidak dipasang ke switch mengambang di sekitar 1500
   (`Radio::decodeModeSwitch()`), jadi tidak pernah RENDAH maupun TINGGI dan
   tidak bisa memicu.
5. Pilot selalu bisa membatalkan dengan ganti mode (pulsa langsung berhenti).

## Log

- Kolom `dist_on`, `dist_us`, `ail_cmd_us`, `ail_out_us`; kanal CH7 di `ch7`.
- Event `#EVENT,<ms>,DIST_ON,+250` dan `#EVENT,<ms>,DIST_OFF`.
- `DIST_ENABLE`, `DIST_US`, `DIST_MS` tercatat di header `# PARAM`.

## Prosedur pakai

**Persiapan remote:** pasang switch 2 posisi ke CH7 (mis. SA/SD di radio
EdgeTX), pastikan nilainya < 1300 di bawah dan > 1700 di atas (lihat tab
Radio/Mission Planner atau kolom `ch7` di log).

**Uji darat (propeller dilepas, FC di-ARM, servo terlihat):**
1. `DIST_ENABLE = 1`, `DIST_US = 250`, `DIST_MS = 2000`, masuk AUTO atau
   LOITER (butuh GPS).
2. CH7 RENDAH ≥ 1 detik, lalu TINGGI: aileron harus bergeser ±250 µs selama
   2 detik lalu kembali sendiri. **Arah gerak permukaan harus sama dengan
   stick roll kanan di mode manual.** Jika terbalik, jangan terbang.
3. Kembalikan CH7 ke RENDAH di tengah pulsa: pulsa tetap selesai 2 detik.
4. Tahan CH7 TINGGI: tidak ada pulsa kedua. Ganti mode saat pulsa: langsung
   berhenti. `DIST_ENABLE = 0`: langsung berhenti.
5. Periksa log SD: event `DIST_ON/OFF`, `dist_us`, `ail_out_us − ail_cmd_us`.

**Saat terbang:** run tanpa gangguan → `DIST_ENABLE = 0`. Run gangguan →
`DIST_ENABLE = 1`, pilot memindah CH7 ke TINGGI pada titik uji yang disepakati
(sama untuk L1 tetap dan L1 + fuzzy), lalu kembali ke RENDAH sebelum pemicu
berikutnya.

## Batasan

- Besar gangguan dinyatakan dalam µs servo; efek aerodinamikanya (derajat
  permukaan ≈ µs ÷ 11,4) bergantung kecepatan. Laporkan kecepatan tiap run.
- Gangguan hanya pada aileron (roll); tidak ada pada GUIDED/FBWA/RTL.
- Belum diuji di hardware (lihat prosedur di atas).
