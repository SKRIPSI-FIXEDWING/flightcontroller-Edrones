# Injeksi gangguan roll (`fc::DisturbanceInjector`)

Untuk eksperimen skripsi *path-following pada kondisi gangguan eksternal*
(proposal 1.3: gangguan diberikan manual lewat remote controller). Di branch
ini injektor hanya aktif di **AUTO** (branch ini belum punya mode LOITER).

- 2026-10-25: versi pertama, dipicu stick roll (dua level ±125/±250 µs, lama
  = selama stick ditahan, maks 3 s), hanya di AUTO.
- **2026-10-26: dipicu switch CH7, satu pulsa tetap per pemicu.** Dibawa ke
  branch ini 2026-11-14 (dari `coba-cek-mode-loiter`). Alasan: gangguan dari stick bergantung tangan pilot (besar dan
  lamanya tidak pernah persis sama antar run), padahal perbandingan L1 tetap
  vs L1 + fuzzy butuh gangguan yang identik.

Kode: `include/control/DisturbanceInjector.h`, `src/control/DisturbanceInjector.cpp`,
`Actuator::writeAttitude(..., aileron_disturbance_us)`, `ModeAuto::_update()`. Uji unit: `test/host/test_disturbance_injector.cpp`.

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
- Arah gangguan dinyatakan dengan tanda `DIST_US` (+ roll kanan, − roll kiri).
  Tetapkan urutan arah yang sama untuk run L1 tetap dan L1 + fuzzy.

## Aturan keselamatan (semuanya dipaksa di dalam kelas)

1. **Mati kecuali `DIST_ENABLE = 1`** (default 0, berlaku live). Set 0 dari
   Mission Planner menghentikan pulsa seketika.
2. **Hanya di AUTO, ARMED, sinyal RC hidup.** Salah satu hilang →
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

**Persiapan remote:** pasang switch 2 posisi ke CH7 (kanal ini TIDAK dipakai
untuk mode terbang; mode tetap dari CH6) (mis. SA/SD di radio
EdgeTX), pastikan nilainya < 1300 di bawah dan > 1700 di atas (lihat tab
Radio/Mission Planner atau kolom `ch7` di log).

**Uji darat (propeller dilepas, FC di-ARM, servo terlihat):**
1. `DIST_ENABLE = 1`, `DIST_US = 250`, `DIST_MS = 2000`, masuk AUTO
   (butuh misi valid dan GPS).
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

## Mode otomatis (`DIST_ENABLE = 2`, 2026-11-15)

Untuk perbandingan L1 tetap vs L1 + fuzzy: **CH7 tidak dipakai**. Program
(`fc::DisturbanceScript`) menembak sendiri satu pulsa tiap kejadian di tempat
yang tetap pada rute, sehingga kedua metode menerima gangguan identik.

Cara pakai: `DIST_ENABLE = 2`, `DIST_FLT = nomor penerbangan (1..5)`,
`DIST_MS = 2000`, lalu terbang AUTO seperti biasa. Urutan dimulai dari
kejadian 1 setiap masuk AUTO. Berhenti: ganti mode dari AUTO, DISARM, RC hilang,
atau `DIST_ENABLE` bukan 2. **Setelah terbang set `DIST_ENABLE = 0`** (kalau
tidak, setiap AUTO berikutnya akan menembak lagi).

Skenario dikenali dari jumlah item misi (`wp_sum`): 4 = Skenario 1 (3 WP),
7 = Skenario 2 (6 WP). Ukuran lain atau `DIST_FLT` di luar tabel: tidak ada gangguan.

Amplitudo urut (diubah 2026-11-16, sebelumnya 125/250 µs dengan campuran searah/berlawanan):
A1 = +150, A2 = −150, A3 = +300, A4 = −300, A5 = +300 µs.
- **Lurus:** tanda mutlak (+ roll kanan, − roll kiri), jadi ada gangguan kanan dan kiri.
- **Belok:** hanya besar |Ak| yang dipakai, arahnya **selalu berlawanan arah belok**
  (belok kiri → roll kanan, belok kanan → roll kiri); tidak ada gangguan searah belok.
  Arah belok dari `next_turn_angle_deg`.
- Durasi semua pulsa = `DIST_MS` = 2000 ms. `DIST_US` tidak dipakai di mode ini.

| Skenario | `DIST_FLT` | Urutan kejadian (belok selalu berlawanan, hanya besarnya dari Ak) |
|---|---|---|
| S1 | k = 1..5 | L1 lurus (Ak bertanda) di 90 m dari WP1; B2 belok saat WP2 diklaim (\|Ak\|) |
| S2 | 1 | L1 (A1 = +150) di 70 m; B3 (150) saat WP3 diklaim; B5 (150) saat WP5 diklaim |
| S2 | 2 | L1 (A2 = −150); B3 (300); B5 (300) |
| S2 | 3 | L1 (A3 = +300); B3 (300); L5 (A4 = −300) di 135 m dari WP5 |
| S2 | 4 | L1 (A5 = +300) |

Tabel S1 per terbang (3 WP, belok WP2 = kiri, jadi gangguan belok selalu +, roll kanan):

| `DIST_FLT` | Lurus L1 | Belok B2 |
|---|---|---|
| 1 | +150 | +150 |
| 2 | −150 | +150 |
| 3 | +300 | +300 |
| 4 | −300 | +300 |
| 5 | +300 | +300 |

Aturan tiap kejadian:

- **Lurus:** menembak bila pesawat di leg slot, sudah menempuh >= posisi slot,
  dan tenang (`|e| <= 5 m` selama >= 3 s). Bila belum tenang sampai batas akhir
  slot (150 m, atau 190 m untuk L5) atau leg sudah lewat: dilewati (SKIP), tidak dikejar.
- **Belok:** menembak saat WP diklaim (leg berganti), hanya bila pesawat tenang
  di leg sebelumnya.
- Pulsa dihentikan bila `|e|` > 45 m (lurus) atau > 60 m (belok); dicatat sebagai SKIP alasan 3.
  Angka 60 m untuk belok dipertahankan (2026-11-16). Ambang ini hanya mengatur
  gangguan (offset menjadi 0); kendali L1 dan attitude tidak terpengaruh.
- `along_m` (jarak di leg) dihitung dari posisi pesawat terhadap WP sebelumnya, bukan dari
  `distance_next_wp`. Dulu memakai `distance_next_wp` yang basi di siklus pertama setelah WP
  diklaim sehingga event lurus L1 selalu dilewati (SKIP alasan 1); lihat
  `docs/catatan-perbaikan-gangguan-otomatis.md`.
- Konstanta: `kDistCalm*`, `kDistAbort*` di `FC_Config.h`; tabel di `src/control/DisturbanceScript.cpp`.

Log: `#EVENT,<ms>,DIST_ON,+150,E<id>` (id = urutan kejadian di tabel),
`DIST_OFF`, dan `#EVENT,<ms>,DIST_SKIP,E<id>,reason=<n>` (1 tidak tenang,
2 slot lewat, 3 dibatalkan |e| besar, 4 tidak ada pergantian leg, 5 arah belok tidak diketahui).
Kolom CSV tidak berubah.

Uji unit: `test/host/test_disturbance_script.cpp`.

## Batasan

- Besar gangguan dinyatakan dalam µs servo; efek aerodinamikanya (derajat
  permukaan ≈ µs ÷ 11,4) bergantung kecepatan. Laporkan kecepatan tiap run.
- Gangguan hanya pada aileron (roll); tidak ada pada GUIDED/FBWA/RTL.
- Belum diuji di hardware (lihat prosedur di atas).
