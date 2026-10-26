# Injeksi gangguan roll (`fc::DisturbanceInjector`)

Ditambahkan 2026-10-25 untuk eksperimen skripsi *path-following pada kondisi
gangguan roll eksternal* (Tabel 3.7 proposal: ±125 µs ringan, ±250 µs sedang).
Keputusan desain dari pengguna: dipicu **stick roll, level diskret**, durasi
maksimum **3 detik**.

Kode: `include/control/DisturbanceInjector.h`, `src/control/DisturbanceInjector.cpp`,
`Actuator::writeAttitude(..., aileron_disturbance_us)`, `ModeAuto::_update()`.
Uji unit: `test/host/test_disturbance_injector.cpp` (di PC, kode aslinya).

## Cara kerja

Di AUTO stick roll tidak dipakai kendali. Selama `DIST_ENABLE = 1`, simpangan
stick roll dari tengah (`|ch_roll − 1500|`, penuh ±512) menentukan gangguan:

| Simpangan stick | Gangguan |
|---|---|
| < 100 µs (`kDistDeadbandUs`) | tidak ada |
| 100 – 399 µs | ±125 µs (`kDistLowUs`) |
| ≥ 400 µs (`kDistLevelSplitUs`) | ±250 µs (`kDistHighUs`) |

Arah = arah stick (**+ = roll kanan**). Offset ditambahkan ke perintah aileron
**sesudah** kendali sikap dan **sebelum** cermin diferensial, jadi servo kiri dan
kanan bergerak persis seperti jika stick digerakkan di mode manual. Karena
cermin itu, PWM servo justru turun saat offset positif (stick kanan di mode
manual juga menurunkan PWM) — di log, `ail_out_us − ail_cmd_us` bernilai
negatif untuk `dist_us > 0`. Offset disaturasi di 988–2012 µs.

## Aturan keselamatan (semuanya dipaksa di dalam kelas)

1. **Mati kecuali `DIST_ENABLE = 1`** (default 0, param live). Sentuhan tak
   sengaja pada stick tidak pernah menyuntik gangguan. Set 0 dari Mission
   Planner menghentikannya seketika.
2. **Hanya di AUTO, ARMED, dengan sinyal RC hidup.** Salah satu hilang →
   gangguan berhenti pada siklus itu juga dan butuh periode "stick di tengah"
   baru. Mode lain tidak pernah menerima offset: offset adalah argumen
   `writeAttitude()` (default 0), bukan state yang bisa tertinggal; `writeManual()`
   tidak punya argumen itu sama sekali.
3. **Stick harus di tengah ≥ 300 ms (`kDistCenterHoldMs`)** setelah masuk AUTO
   atau setelah satu aktivasi selesai sebelum gangguan boleh dipicu. Stick yang
   sudah miring saat AUTO aktif tidak melakukan apa-apa.
4. **Satu aktivasi maksimum 3 detik (`kDistMaxDurationMs`)**, lalu mati sendiri
   walau stick masih ditahan; baru bisa lagi setelah stick kembali ke tengah.
5. Lepas stick = gangguan berhenti seketika (langkah/impuls dikendalikan pilot).

## Log

- Kolom `dist_on` (0/1), `dist_us` (offset yang diminta, bertanda), `ail_cmd_us`
  (servo kiri tanpa gangguan), `ail_out_us` (servo kiri yang dikirim).
- Event `#EVENT,<ms>,DIST_ON,+250` saat mulai atau berganti level/sisi, dan
  `#EVENT,<ms>,DIST_OFF` saat berakhir (cap waktu dari siklus kendali).
- `DIST_ENABLE` tercatat di `# PARAM` header dan setiap snapshot ARM.

## Prosedur pakai

**Uji darat (propeller dilepas, FC di-ARM, servo terlihat):**
1. Set `DIST_ENABLE = 1`, masuk AUTO (butuh misi valid dan GPS) — atau uji
   logika murni di PC dengan `test/host/test_disturbance_injector.cpp`.
2. Biarkan stick di tengah ≥ 1 detik, lalu gerakkan stick roll ke kanan sedikit:
   aileron harus bergeser sekitar 125 µs dari posisi kendali; stick penuh ke
   kanan: sekitar 250 µs. **Pastikan arah gerak permukaan sama dengan jika
   stick digerakkan di mode manual.** Jika terbalik, jangan terbang; kabari
   agar tanda diperiksa.
3. Tahan stick > 3 detik: aileron harus kembali ke posisi kendali sendiri.
4. Keluar dari AUTO dengan stick masih miring: tidak boleh ada offset di mode
   baru. Set `DIST_ENABLE = 0`: berhenti seketika.
5. Periksa log SD: event `DIST_ON/OFF`, `dist_us` dan `ail_out_us − ail_cmd_us`.

**Saat terbang:** `DIST_ENABLE = 1` hanya untuk run gangguan; set 0 untuk run
tanpa gangguan. Pilot memicu gangguan dengan menggerakkan stick roll sesaat
pada titik uji yang disepakati (sama untuk kedua metode), lalu melepasnya.
Pilot selalu bisa mengambil alih dengan mengganti mode.

## Batasan yang perlu diketahui

- Besar gangguan dinyatakan dalam µs servo; efek aerodinamikanya (derajat
  permukaan ≈ µs ÷ 11,4) bergantung pada kecepatan. Laporkan kecepatan tiap run.
- Gangguan hanya pada aileron (roll); tidak ada pada GUIDED/FBWA/RTL.
- Belum diuji di hardware (lihat prosedur di atas).
