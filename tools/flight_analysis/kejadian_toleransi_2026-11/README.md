# Kejadian keluar toleransi ±10 m — loiter fase stabil, data November 2026

Format ini mengikuti arahan pembimbing di komentar draft 14 Nov 2026: galat loiter tidak dilaporkan sebagai rata-rata, RMS
atau % waktu, tetapi **per kejadian** keluar toleransi. Untuk tiap kejadian dicatat jenis, puncak, durasi, sisi orbit,
penyebab (pulsa, roll jenuh, searah angin), serta period/dT saat kejadian. Definisinya ada di docstring
`../analyze_kejadian_toleransi.py`. Untuk membuat ulang folder ini, jalankan `buat_ulang.sh` dari `tools/flight_analysis`.

Isi folder:
- `rekap_kejadian.md`: tabel per penerbangan dan per kejadian (bahan Bab 5).
- `kejadian_toleransi.csv`, `rekap_per_penerbangan.csv`: tabel yang sama dalam CSV.
- `<segmen>_galat_waktu.png`: galat vs waktu (setpoint merah, ±10 m putus-putus, kejadian diarsir dan dinomori,
  pulsa oranye), lalu perintah roll terhadap batas bank, lalu period L1 dan GS.
- `<segmen>_orbit.png`: jejak loiter fase stabil, dengan segmen kejadian diwarnai menurut jenisnya, lokasi pulsa, dan angin.

Nama segmen mengikuti `flight_2026-11-16/ringkasan_hari.md`:
- T2 = L1 tetap R100 (16 Nov).
- N14_* = L1 tetap R100 (14 Nov). **Semua L1 tetap R100 dipakai**, termasuk yang anginnya 6,9–7,5 m/s (12-22-15,
  14-21-31) dan 14-29-40 (arus naik). Angin tiap segmen ada di tabel rekap. N14_12-22-15 dibuat dari tlog 10 Hz.
- T3a–T4b = fuzzy R100 dengan gangguan. T4c (fuzzy tanpa gangguan, data tambahan) tidak dipakai.
- N16_14-07-25b = L1 tetap **R80**, satu-satunya segmen R80 dengan angin ≤ 4 m/s (3,2 m/s). Dibuat dari tlog yang
  dikonversi (10 Hz), karena log SD `2026-11-16_14-06-34.CSV` belum disalin. Waktu di grafik = detik sejak awal
  potongan tlog.

**R80 yang dibuang** (angin di atas batasan ≤ 4 m/s): T5 11-17-00 (6,2 m/s), 14-07-25 segmen a dan c–f (4,5–6,3 m/s),
dan 14-28-43 a–c (fuzzy, 6,7–7,2 m/s, ketinggian tidak terkendali). Data fuzzy R80 yang layak belum ada.

## Catatan untuk penulisan

- Arah gangguan −300 µs mendorong wahana ke **luar** pada loiter CW. Semua overshoot di rekap ini
  berpuncak 3,4–5,0 s setelah pulsa, jadi seluruh overshoot itu respons terhadap gangguan.
- Beberapa undershoot berpuncak < 1 s setelah pulsa (T3a#1, T3a#4, N14_14-21-31#3, N14_15-03-50#1/#3). Pulsa −300 µs
  tidak mungkin menyebabkan galat ke dalam secepat itu. Galat ke dalam ini sudah berjalan sebelum pulsa, dan pulsa malah
  menariknya kembali. Kejadian yang pulsanya baru diberikan saat galat sudah di luar toleransi ditandai otomatis
  ("pulsa diberikan saat sudah di luar toleransi") dan tidak dihitung "terkait pulsa". Kejadian yang pulsanya datang
  sedikit sebelum kejadian dimulai belum tertangkap tanda ini, jadi perlu dibaca manual.
- Ambang "roll jenuh" = |nav_roll| ≥ 44,5°. Kolom `roll_jenuh_pct` menunjukkan berapa persen waktu kejadian roll jenuh.
  Banyak puncak terjadi tepat sesudah perintah roll turun dari batas (nav_roll 40–44° saat puncak).
- Fase stabil di tiap segmen hanya 1,3–2,7 putaran (rencana: R100 4 putaran, R80 8 putaran), dan jarak antar-pulsa ±8 s. Akibatnya, satu kejadian sering masih
  bercampur dengan respons pulsa sebelumnya.
- Definisi overshoot/undershoot di sini berdasarkan **arah galat terhadap orbit** (luar/dalam), bukan terhadap arah
  gangguan. Definisi ini harus sama dengan yang dipakai di Bab 3 (komentar pembimbing no. 33).
