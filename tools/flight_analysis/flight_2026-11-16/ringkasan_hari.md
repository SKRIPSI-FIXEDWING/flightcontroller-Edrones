# Rekap terbang 16 Nov 2026

Firmware `EXP_V3_20261025` (build 15 Nov 14:27), period 14 s, damping 0,707, `LOIT_BANK_DEG` 45, gangguan
**−300 µs, 2 s**. Misi WP1 → WP2 → WP3 → WP4 loiter CW. Format sama dengan 14 Nov: satu folder `terbang_<jam ARM>` per terbang
berisi analisis tlog (folder utama, `loiter/`, `full/`), log SD (`fuzzy/`, `gangguan/`, `kecepatan/`) dan `ringkasan.md`.

## Daftar terbang

| Terbang | Folder | tlog | Log SD | Metode | R | Segmen loiter | Pulsa di loiter stabil | Dipakai |
|---|---|---|---|---|---|---|---|---|
| 1 | – | 10-21-27 | 10-40-05 (ARM 64 s) | L1 tetap | 100 | 1 | – | **tidak** (salah, `DIST_US` masih +300, loiter 35 s) |
| 2 | `terbang_10-44-59` | 10-21-27 | 10-40-05 (ARM 311 s) | L1 tetap | 100 | 1 | 5 | L1 tetap + gangguan (pengganti 14 Nov 12-14-37) |
| 3 | `terbang_10-51-03` | 10-21-27 | 10-50-34 | L1 + fuzzy | 100 | 3 | 6 / 5 / 8 | fuzzy + gangguan |
| 4 | `terbang_11-03-36` | 11-02-20 | 11-02-12 | L1 + fuzzy | 100 | 3 | 9 / 6 / **0** | segmen 1–2 fuzzy + gangguan, segmen 3 fuzzy **tanpa** gangguan |
| 5 | `terbang_11-17-00` | 11-13-33 | 11-15-54 | L1 tetap | **80** | 1 | 6 | grup R80 |
| 6 | `terbang_14-07-25` | 14-04-15 | 14-06-34 (**belum disalin**) | L1 tetap | **80** | 6 | 6 / 6 / 7 / 6 / 8 / 8 | grup R80, angin kencang |
| 7 | `terbang_14-28-43` | 14-27-05 | 14-26-33 (**belum disalin**) | L1 + fuzzy | **80** | 3 (+2 AUTO tanpa loiter) | 7 / 1 / 5 | grup R80, hanya AUTO 1 layak |

Terbang 3 dan 4 berisi 3 loiter karena pilot pindah STAB 1–2 s lalu AUTO lagi (misi di-reset ke WP1). **Tiap masuk AUTO
dihitung sebagai satu penerbangan**, dan analisis gangguannya ada di subfolder `auto1/`, `auto2/`, `auto3/`. Di tabel bawah
segmennya ditulis T3a/b/c dan T4a/b/c (= `auto1/2/3` = "terbang 1/2/3" di grafik tlog folder itu).

Data per kelompok:
- **L1 + fuzzy + gangguan: 5 penerbangan** — T3a (`terbang_10-51-03/auto1`), T3b (`auto2`), T3c (`auto3`),
  T4a (`terbang_11-03-36/auto1`), T4b (`auto2`).
- **L1 + fuzzy tanpa gangguan: 1** — T4c (`terbang_11-03-36/auto3`), data tambahan.
- **L1 tetap + gangguan: 5** — 14 Nov 12-22-15, 14-21-31, 14-29-40 (arus naik, catatan), 15-03-50, dan 16 Nov T2 (`terbang_10-44-59`).
- **L1 tetap R80: 7** — T5 (`terbang_11-17-00`) dan 6 AUTO di `terbang_14-07-25` (`auto1`–`auto6`).
- **L1 + fuzzy R80: 1 layak** — `terbang_14-28-43/auto1`; `auto2`–`auto3` tercampur naik-turun ketinggian.

Terbang 6 dan 7 (sore) belum punya log SD di Mac: `fuzzy/`, `auto<N>/` dan grafik `_sd` dibuat dari CSV pengganti
`tlog_to_sdlog.py` (10 Hz). Ulangi dengan `2026-11-16_14-06-34.CSV` dan `2026-11-16_14-26-33.CSV` bila kartunya sudah dibaca.

Grafik lingkaran loiter stabil versi log SD: `terbang_*/loiter/13_lingkaran_steady_terbang<N>_sd.png`
(`plot_loiter_steady_sdlog.py`). Pakai ini untuk skripsi: versi tlog di loiter 1 terbang 10-51-03 bersegi karena telemetri putus.
Sesi darat tanpa ARM: SD 10-18-32, 10-22-57, 10-36-35.

tlog dan log SD dipotong per terbang dengan `skrip/potong_tlog.py` dan `skrip/potong_sdlog.py` (jendela waktu di
`ringkasan.md` tiap terbang). Angka gabungan di bawah dari `skrip/ringkas_loiter_stabil.py` dan `skrip/pulsa_bertanda.py`
(log SD yang dipotong per segmen loiter).

## Loiter fase stabil (putaran ke-2 dst, termasuk gangguan), galat = jarak ke pusat − R

| Segmen | Metode | Stabil | Galat rata-rata | RMS | Maks | Keluar ±10 m | Angin | Periode (min–maks) |
|---|---|---|---|---|---|---|---|---|
| T2 | L1 tetap | 55 s | +3,6 m | 8,8 m | 21,3 m | 22% | 3,3 m/s dari 176° | 14 |
| T3a | fuzzy | 77 s | +2,5 m | 8,0 m | 20,3 m | 22% | 5,1 / 200° | 12–16 |
| T3b | fuzzy | 51 s | +1,5 m | 5,2 m | 12,0 m | 5% | 2,5 / 196° | 12–16 |
| T3c | fuzzy | 63 s | +2,9 m | 5,3 m | 12,8 m | 9% | 2,4 / 223° | 12–16 |
| T4a | fuzzy | 87 s | +1,3 m | 5,6 m | 18,3 m | 6% | 4,3 / 193° | 12–16 |
| T4b | fuzzy | 56 s | +1,8 m | 6,7 m | 17,5 m | 10% | 3,7 / 177° | 12–16 |
| **T4c** | fuzzy, **tanpa gangguan** | 49 s | **−3,5 m** | **5,5 m** | **9,3 m** | **0%** | 4,3 / 174° | 13,4–15,6 |
| T5 | L1 tetap, **R80** | 62 s | +3,2 m | 6,6 m | 14,3 m | 19% | **6,2 / 200°** | 14 |

Pembanding L1 tetap 14 Nov (dihitung ulang dengan cara yang sama dari log SD): 14-21-31 +0,7 / 10,0 / 30,5 m / 18%,
15-03-50 +0,8 / 9,9 / 26,9 m / 25%; 12-22-15 (tlog) −1,1 / 5,8 m / 5%.

- Rata-rata RMS loiter stabil: **fuzzy 6,2 m (5 penerbangan)** vs **L1 tetap 8,6 m (4 terbang: 12-22-15, 14-21-31, 15-03-50, T2)**.
  Tapi angin 14 Nov lebih kencang (3,6–7,5 m/s) dan pembanding L1 di hari yang sama hanya **satu** (T2, RMS 8,8 m).
- Fase stabil di semua segmen hanya **±1,5 putaran** (49–87 s). Rencana checklist: 4 putaran.
- Fuzzy aktif di loiter stabil: periode menyentuh batas 12/16 s 12–17% waktu, aturan Turun 17–31%, Naik 18–40%.
  Tanpa gangguan (T4c) periode hanya 13,4–15,6 s (Tetap 83%).

## Respons per pulsa (loiter stabil), bertanda

`geser keluar` = galat maksimum 15 s setelah pulsa − median galat 2 s sebelumnya (−300 µs mendorong ke **luar** di loiter CW).

| Grup | Pulsa | Galat sebelum | Geser keluar | Waktu puncak | \|galat\| maks | Jarak antar-pulsa |
|---|---|---|---|---|---|---|
| L1 tetap (SD 14-19-44, 14-48-03, T2) | 16 | −4,5 m | **11,2 m** | 3,7 s | 12,0 m | 7,6 s |
| L1 + fuzzy (T3a–T4b) | 34 | −0,7 m | **12,0 m** | 3,8 s | 10,0 m | 8,2 s |
| L1 tetap R80 (T5) | 6 | +0,2 m | 13,1 m | 3,9 s | 12,8 m | 8,1 s |

- Uji Mann-Whitney L1 vs fuzzy: geser keluar p = 0,83, |galat| maks p = 0,28. **Respons terhadap pulsa tidak berbeda nyata.**
- Kolom `kenaikan_m` di `respons_gangguan.csv` (selisih |galat|) memberi kesan L1 lebih baik (median 2,7 vs 7,6 m). Ini artefak:
  L1 punya bias ke dalam −4,5 m, jadi pulsa ke luar mula-mula **mengurangi** |galat|. Untuk perbandingan pakai galat bertanda di atas.
- Waktu pulih hampir tidak terukur (fuzzy 6/34, L1 1/16): pulsa berikutnya datang ±8 s kemudian, sebelum pesawat kembali ke galat awal.
  Checklist meminta jeda ≥ 15 s.

## Bias tanpa gangguan

| Data | Durasi bebas gangguan | Bias |
|---|---|---|
| Fuzzy T4c (seluruh loiter stabil) | 49 s | −3,5 ± 4,2 m |
| Fuzzy T3a, T4a, T4b (celah antar-pulsa) | 2–12 s | −1,2 … −2,0 m |
| L1 tetap R80 T5 (celah) | 8 s | −3,6 m |
| L1 tetap 14 Nov: 12-14-37 sebelum gangguan (folder sudah dihapus, ada di commit `d750be0`) | 52 s | −6,7 ± 1,5 m |
| L1 tetap 14 Nov: 14-19-44 / 14-48-03 (celah) | 11 / 3 s | −7,2 / −4,9 m |

Fuzzy tampak punya bias ke dalam yang lebih kecil, tapi **L1 tetap tanpa gangguan belum pernah diterbangkan di hari dengan angin
yang sama**, jadi belum bisa disimpulkan.

## R80

| Data | Metode | Angin (fit) | Loiter stabil: galat rata-rata / RMS / maks / keluar ±10 m | Pulsa stabil baik/cukup/buruk/tercampur |
|---|---|---|---|---|
| T5 `terbang_11-17-00` | L1 tetap | 6,2 m/s | +3,2 / 6,6 / 14,3 m / 19% | 0 / 0 / 6 / 0 |
| `terbang_14-07-25/auto1` | L1 tetap | 5,2 m/s | +0,8 / 5,4 / 18,1 m / 3% | 4 / 1 / 1 / 0 |
| `terbang_14-07-25/auto2` | L1 tetap | 3,2 m/s | +3,3 / 7,0 / 17,0 m / 19% | 2 / 0 / 4 / 0 |
| `terbang_14-07-25/auto3` | L1 tetap | 4,5 m/s | +6,6 / 12,4 / 28,6 m / 31% | 2 / 0 / 2 / 3 |
| `terbang_14-07-25/auto4` | L1 tetap | 6,0 m/s | +6,0 / 8,5 / 20,2 m / 29% | 0 / 0 / 4 / 2 |
| `terbang_14-07-25/auto5` | L1 tetap | 6,3 m/s | +7,0 / 21,3 / 58,6 m / 48% | 1 / 0 / 2 / 5 |
| `terbang_14-07-25/auto6` | L1 tetap | 5,3 m/s | +4,2 / 12,9 / 32,6 m / 40% | 1 / 0 / 3 / 4 |
| `terbang_14-28-43/auto1` | **fuzzy** | 6,7 m/s | +2,1 / 9,6 / 24,9 m / 21% | 1 / 1 / 3 / 2 |
| `terbang_14-28-43/auto2` | fuzzy | 7,0 m/s | +8,4 / 19,4 / 39,7 m / 34% (stabil 16 s) | 0 / 0 / 0 / 1 |
| `terbang_14-28-43/auto3` | fuzzy | 7,2 m/s | +13,0 / 22,5 / 70,4 m / 72% | 0 / 0 / 0 / 5 |

Loiter dari tlog (`loiter/ringkasan_steady.csv`), pulsa dari `auto<N>/gangguan/respons_gangguan.csv`.

- **Angin sore 5–10 m/s**, jauh di atas batas checklist R80 (≤ 4 m/s). Semua data R80 diambil di luar syarat itu.
- **Ketinggian tidak terkendali** di terbang fuzzy R80 (AUTO 2–5) dan mulai di L1 R80 AUTO 5–6: pesawat naik 3–5 m/s di sisi
  melawan angin padahal `nav_pitch` sudah mentok −10°. AUTO 4–5 terbang 14-28-43 naik ke 171 m dan 193 m, lalu pilot ambil alih.
  Throttle AUTO = stick pilot (`FC_AUTO_THROTTLE_ENABLE` = 0), dan sore itu stick 1600–1740 µs (pagi ±1440 µs); batas pitch
  −10° tidak cukup membuang kelebihan tenaga, ditambah udara naik siang hari (lihat `terbang_14-28-43/ringkasan.md`).
- Perbandingan fuzzy vs L1 di R80 **belum adil**: fuzzy hanya 1 segmen layak (angin 6,7 m/s) vs L1 7 segmen.
  Pada AUTO pertama tiap terbang (kondisi paling bersih), L1 R80 RMS 5,4 m vs fuzzy R80 9,6 m, tapi angin fuzzy lebih kencang.

## Perlu ambil data lagi?

1. **R80: masih perlu fuzzy R80**, kalau grup R80 tetap masuk skripsi. L1 R80 sudah 7 segmen (walau angin kencang), fuzzy R80
   baru 1 segmen layak. Minimal fuzzy R80 ×2–3 lagi, angin ≤ 4 m/s, pagi hari (hindari thermal ±14.00), dan **throttle stick
   dijaga ±1440 µs seperti terbang pagi** saat AUTO (throttle tidak diatur TECS).
   Kalau waktu tidak cukup, R80 lebih aman dikeluarkan dari analisis utama (cukup jadi catatan uji tambahan).
2. **L1 tetap tanpa gangguan, 1–2 terbang**, di hari yang sama dengan fuzzy tanpa gangguan, supaya bias bisa dibandingkan.
3. Kalau terbang lagi: loiter **≥ 3 putaran** sebelum keluar AUTO, jeda pulsa **≥ 15 s**, supaya waktu pulih terukur.
4. Fuzzy R100 + gangguan (5 penerbangan, 34 pulsa) dan L1 tetap R100 + gangguan (4 terbang + 14-29-40 sebagai catatan) sudah cukup untuk perbandingan utama,
   dengan catatan beda hari/angin.
5. PID (P1) belum diterbangkan.
