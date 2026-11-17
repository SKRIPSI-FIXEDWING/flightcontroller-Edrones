# Rekap terbang 17 Nov 2026 (pagi) — R80, L1 tetap vs L1 + fuzzy

Firmware `EXP_V3_20261025` (build 15 Nov 14:27), period dasar 14 s, damping 0,707, `LOIT_BANK_DEG` 45,
**`WP_LOITER_RAD` 80**, gangguan **−300 µs, 2 s**. Misi WP1 → WP2 → WP3 → WP4 loiter CW. Format sama dengan 16 Nov:
satu folder `terbang_<jam ARM>` per terbang (jam dari waktu GPS di log SD), subfolder `auto<N>/` = tiap masuk AUTO.
Semua langkah ada di `buat_ulang.sh`.

## Daftar log

| Log SD | tlog | Isi | Folder |
|---|---|---|---|
| `2026-11-17_08-43-41.CSV` | – | sesi darat, tidak ARM (`FUZZY_ENABLE` 1 → 0) | tidak dipakai |
| `2026-11-17_08-51-16.CSV` | `2026-11-17 08-48-08.tlog` | **L1 tetap R80**, ARM 09:03:03, 4× AUTO, 26 pulsa | `terbang_09-03-03` |
| `2026-11-17_09-17-42.CSV` | `2026-11-17 09-18-02.tlog` | **L1 + fuzzy R80**, ARM 09:20:26, 5× AUTO, 35 pulsa | `terbang_09-20-26` |

Kedua terbang punya ARM 3 s di darat sebelum ARM terbang (dibuang saat dipotong). Trim IMU sudah −1,5625 / −0,5 di
kedua terbang (diubah di darat sebelum terbang L1).

## Kondisi

- **Angin tenang: 0,1–2,3 m/s** (fit loiter stabil per AUTO). Semua segmen memenuhi syarat R80 (≤ 4 m/s). Kolom
  "searah/lawan angin" di rekap kejadian tidak bermakna untuk angin selemah ini.
- **Kecepatan kedua terbang tidak sama.** Throttle AUTO = stick pilot (`FC_AUTO_THROTTLE_ENABLE` = 0) dan stick hampir
  sama (1447 vs 1451 µs), tetapi baterai berbeda:

| Terbang | Baterai (loiter stabil) | Airspeed | GS | Pitch rata-rata | Ketinggian (std galat, tlog) |
|---|---|---|---|---|---|
| L1 tetap | 15,0 → 14,2 V | 15,9 (AUTO 1) → 12,7–13,4 m/s | 14,4–17,4 m/s | +5° | 1,4–1,7 m |
| L1 + fuzzy | 16,1 → 15,3 V | 18,0 (AUTO 1–2) → 14,9 m/s | 16,3–19,2 m/s | +0,4° | **5–8 m** (naik-turun ±7 m tiap putaran) |

  Terbang L1 jauh lebih lambat (airspeed rata-rata AUTO 3 12,5 m/s, di bawah `MIN_AIRSPEED` 14), terbang fuzzy lebih cepat dan
  ketinggiannya berosilasi. Dengan R = 80 m, bank tunak yang dibutuhkan ≈ atan(V²/gR): **±15° pada 14,5 m/s, ±25° pada
  19 m/s**, jadi margin ke batas bank 45° lebih kecil di terbang fuzzy. Ini harus disebut setiap kali dua terbang ini dibandingkan.

## Pembanding tanpa gangguan (satu putaran) — `pembanding_tanpa_gangguan_R80/`

Di dua segmen, pulsa pertama baru diberikan setelah ±1 putaran fase stabil penuh. Putaran itu dipakai sebagai pembanding
tanpa gangguan (fase stabil = mulai putaran ke-2 loiter, sama dengan analisis lain).

| | L1 tetap (`terbang_09-03-03` AUTO 3) | L1 + fuzzy (`terbang_09-20-26` AUTO 1) |
|---|---|---|
| t_boot putaran | 1046,2–1080,4 s (34 s) | 252,8–278,9 s (26 s) |
| Angin | 0,8 m/s | 2,0 m/s |
| GS / airspeed | 14,5 / 12,8 m/s | 18,9 / 18,0 m/s |
| **Galat radius (min … maks)** | **−1,8 … +0,7 m** | **−9,5 … +6,3 m** |
| Kejadian keluar ±10 m | tidak ada | tidak ada (paling dekat −9,5 m) |
| Bias (rata-rata ± sd) | −0,6 ± 0,8 m | −1,5 ± 5,2 m |
| Period L1 | 14 s tetap | 12,7–16,0 s (menyentuh 16 s di ±80°) |
| `nav_roll` | 9–19° | 12–30° |
| Ketinggian | 98,9–101,2 m | 95,3–109,1 m |

Grafik: `01_orbit_putaran_bebas.png` (dua orbit berdampingan, setpoint merah, ±10 m putus-putus) dan
`02_galat_period_roll_vs_sudut.png` (galat, period, `nav_roll` terhadap sudut orbit). Per kejadian: `kejadian_toleransi_R80/BEBAS_*`.

Cek konsistensi (baris "tambahan" di `banding_putaran_bebas.csv`): putaran berikutnya di L1 AUTO 3 (1080,4–1113,6 s)
sama, −2,3 … +0,5 m. Fuzzy AUTO 3 (519,5–543,4 s, hanya 0,9 putaran sebelum pulsa) −10,2 … +8,3 m, satu kejadian
singkat di −10,2 m.

- **Tanpa gangguan, keduanya tetap di dalam ±10 m.** L1 tetap jauh lebih rapat (±2 m) daripada fuzzy (±10 m).
- Ini **tidak** berarti fuzzy lebih buruk, karena kondisinya tidak setara: fuzzy terbang 30% lebih cepat, angin 2,5× lebih
  besar, dan ketinggiannya naik-turun 14 m dalam satu putaran. Pusat orbit fuzzy juga bergeser ±5–7 m ke barat daya (galat + di
  selatan–barat daya, − di timur laut), pola khas angin/kecepatan, bukan khas aturan fuzzy.
- Fuzzy aktif walau tanpa gangguan: period menyentuh 16 s (Naik) saat galat berubah cepat di awal putaran, dan turun ke
  12,7 s (Turun) saat galat mendekati −9,5 m.
- Untuk perbandingan tanpa gangguan yang adil, perlu putaran bebas di **terbang yang sama kecepatannya** (baterai
  setara, atau throttle diatur TECS). Data hari ini cukup sebagai pembanding kualitatif.

## Dengan gangguan — `kejadian_toleransi_R80/`

Bagian bergangguan = fase stabil mulai pulsa pertama (L1 AUTO 3, fuzzy AUTO 1 dan 3 dipotong mulai pulsa pertama;
segmen lain memang langsung diberi pulsa). Format per kejadian sesuai arahan pembimbing, sama dengan
`kejadian_toleransi_2026-11/`.

| Segmen | Metode | Stabil | Putaran | Angin | Airspeed | Pulsa | Kejadian | Puncak luar | Terlama | Roll jenuh saat puncak |
|---|---|---|---|---|---|---|---|---|---|---|
| N17_09-03-03a | L1 tetap | 73 s | 2,3 | 1,4 m/s | 15,9 | 7 | 7 | 16,3 m | 3,4 s | 0 |
| N17_09-03-03b | L1 tetap | 47 s | 1,3 | 0,4 m/s | 13,4 | 4 | 5 | 18,4 m | 3,4 s | 0 |
| N17_09-03-03c | L1 tetap | 47 s | 1,1 | 2,3 m/s | 12,7 | 5 | 5 | 19,4 m | 3,8 s | 0 |
| N17_09-03-03d | L1 tetap | 52 s | 1,4 | 1,7 m/s | 13,1 | 6 | 7 | 16,4 m | 3,2 s | 0 |
| N17_09-20-26a | L1 + fuzzy | 58 s | 2,0 | 1,7 m/s | 18,0 | 5 | 3 | 24,1 m | 5,8 s | 0 |
| N17_09-20-26b | L1 + fuzzy | 51 s | 1,8 | 1,3 m/s | 18,1 | 7 | 3 | 26,7 m | 6,2 s | 2 |
| N17_09-20-26c | L1 + fuzzy | 73 s | 2,5 | 1,2 m/s | 17,1 | 8 | 5 | 24,2 m | 6,0 s | 1 |
| N17_09-20-26d | L1 + fuzzy | 84 s | 2,6 | 0,9 m/s | 15,3 | 7 | 2 | 24,2 m | 5,4 s | 0 |
| N17_09-20-26e | L1 + fuzzy | 82 s | 2,5 | 0,1 m/s | 14,9 | 7 | 5 (1 undershoot −11,4 m) | 30,7 m | 7,2 s | 1 |

(Kolom "Pulsa" = pulsa di bagian stabil; pulsa P0 sebelum fase stabil bisa ikut menyebabkan kejadian.)

- **L1 tetap: hampir setiap pulsa = satu overshoot kecil.** 24 kejadian dari 22 pulsa, puncak 11–19 m, durasi 1,4–3,8 s,
  roll tidak pernah jenuh saat puncak. Tidak ada undershoot.
- **L1 + fuzzy: lebih jarang keluar, tetapi sekali keluar lebih jauh dan lebih lama.** 18 kejadian dari 34 pulsa, puncak
  13–31 m, durasi sampai 7,2 s. Puncak besar (> 20 m) hampir semua di sisi **barat daya–selatan** orbit (sektor BD/S,
  sama dengan sisi galat + pada putaran bebas), `nav_roll` 40–45° (jenuh 30–60% waktu kejadian), dan period sudah
  turun ke 12,0–12,9 s (dT −1,1 … −2,0 s). Fuzzy sudah meminta L1 lebih agresif, tetapi bank sudah di batas.
- Respons per pulsa (bertanda, `pulsa_bertanda.txt`): geser keluar median **L1 13,4 m vs fuzzy 12,0 m**, waktu puncak
  sama 3,9 s; Mann-Whitney p = 0,83 (geser keluar) dan 0,34 (|galat| maks). **Tidak berbeda nyata**, sama dengan
  kesimpulan R100 16 Nov.
- **Pembanding pada kecepatan mirip**: L1 AUTO 1 (airspeed 15,9 m/s, 15,0 V) vs fuzzy AUTO 4–5 (15,3 / 14,9 m/s,
  15,4 V). L1: 7 kejadian, puncak 12–16 m, ≤ 3,4 s. Fuzzy: 7 kejadian, puncak 13–31 m, sampai 7,2 s. Jadi puncak fuzzy
  yang lebih besar **tidak sepenuhnya** dijelaskan oleh kecepatan; ketinggian fuzzy yang naik-turun (alt std 5 m vs
  1,5 m) dan sisi orbit yang sama tiap kali tetap jadi faktor yang belum bisa dipisahkan.
- Jarak antar-pulsa median 8,8 s (L1) dan 9,6 s (fuzzy), masih < 15 s, jadi waktu pulih jarang terukur (L1 8/22, fuzzy 2/34).

Tambahan dari analisis lama (per AUTO, `terbang_*/auto<N>/gangguan/`): L1 21 pulsa stabil dinilai "buruk"
(keluar toleransi) dan 1 terpotong; fuzzy 11 baik / 2 cukup / 16 buruk / 2 tercampur / 4 terpotong.

## Perbandingan dengan R80 16 Nov

R80 di `kejadian_toleransi_2026-11/` baru punya satu segmen layak (L1 tetap 14-07-25b, angin 3,2 m/s: 4 kejadian,
puncak 12–17 m). Hari ini menambah **4 segmen L1 tetap dan 5 segmen fuzzy R80 dengan angin ≤ 2,3 m/s**, sehingga
**fuzzy R80 sekarang punya data layak** (sebelumnya belum ada). L1 hari ini (puncak 11–19 m) konsisten dengan 14-07-25b.

## Catatan

- Fase stabil per segmen 1,1–2,6 putaran (rencana R80: 8 putaran).
- Analisis tlog (`terbang_*/` folder utama, `loiter/`, `full/`) dan log SD (`fuzzy/`, `loiter/13_*_sd.png`) dibuat seperti 16 Nov.
- `analyze_kejadian_toleransi.py` diberi argumen opsional ke-7 `s0-s1` (potong fase stabil) untuk memisahkan putaran bebas
  dan bagian bergangguan dari satu segmen AUTO.
