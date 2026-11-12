# Checklist terbang: perbandingan Fuzzy vs L1 tetap vs PID (November 2026)

Tujuan: data final untuk membandingkan L1 + fuzzy, L1 tetap, dan PID, dengan
gangguan roll di setiap arah. Semua terbang memakai firmware dan setelan yang
sama, kecuali yang memang sedang dibandingkan.

---

## A. Sebelum hari terbang (di rumah / lab)

- [ ] Firmware berisi **radius loiter tetap** (commit `0f4c961`) dan **STABILIZE me-reset misi**. Build lalu flash ke Teensy.
- [ ] Firmware berisi **navigasi PID** (`NAV_PID_*`, default mati). Lihat `docs/navigation-pid.md`.
- [ ] Kartu SD kosong / cukup ruang. Jam GPS Here4 dapat fix (nama file log sesuai tanggal).
- [ ] Baterai terbang dan baterai remote penuh. Hitung: minimal 18 terbang (lihat bagian D).
- [ ] Siapkan misi di Mission Planner (bagian C) dan simpan sebagai file `.waypoints`.
- [ ] Cetak lembar catatan terbang (bagian G).

## B. Parameter (set lewat Mission Planner, lalu Write Params)

| Parameter | Nilai | Catatan |
|---|---|---|
| `LOIT_BANK_DEG` | **45** | dulu 35 (live) |
| `ROLL_LIMIT_DEG` | 40 | tetap |
| `WP_LOITER_RAD` | **100** | grup radius kecil: **80** |
| `WP_RADIUS` | 15 | tetap |
| `NAVL1_PERIOD` | **14** | dulu default 22 |
| `NAVL1_DAMPING` | 0.707 | tetap |
| `FUZZY_MIN_PER` / `FUZZY_MAX_PER` | 12 / 16 | tetap |
| `FUZZY_ENABLE` | 1 = fuzzy, 0 = L1 tetap / PID | ganti per terbang (live) |
| `NAV_PID_ENABLE` | **0**, hanya **1** di terbang P1 | live, MP menampilkan "NAV: PID ON" |
| `NAV_PID_KP` / `KD` / `KI` | 0.2014 / 0.6347 / 0.0101 | default, jangan diubah |
| `DIST_ENABLE` | **1** | live |
| `DIST_US` | **+300** | ganti ke **−300** di tengah loiter (live) |
| `DIST_MS` | **2000** | |
| `LOIT_ROLL_SIGN` | 1 | jangan diubah |
| `AIRSPEED_CRUISE` | 18 | tetap (dipakai pengaman radius minimum) |

- [ ] Semua parameter di atas sudah ditulis.
- [ ] Refresh Params, cek ulang `LOIT_BANK_DEG` = 45, `NAVL1_PERIOD` = 14, `NAV_PID_ENABLE` = 0.

## C. Misi

- WP1 sebagai titik awal (leg pesawat → WP1 tidak punya garis, tidak dinilai).
- Leg ke **4 arah: utara, timur, selatan, barat**. Buat leg **sepanjang mungkin (≥ 200 m)** sesuai lapangan. Misi 29 Okt legnya hanya 56–141 m, terlalu pendek untuk gangguan 2 s + waktu pulih.
- Item terakhir **LOITER_TURNS** (`MAV_CMD 18`), arah CW, kolom radius **0** (pakai `WP_LOITER_RAD`):
  - R100: **4 putaran**
  - R80: **8 putaran**
- Pusat loiter cukup jauh dari pohon/gedung: lingkaran 100 m butuh area ±220 m.

- [ ] Misi R100 di-upload dan di-Read ulang untuk dicek.
- [ ] Siapkan juga misi R80 (bisa misi yang sama, cukup ganti `WP_LOITER_RAD` ke 80 dan jumlah putaran 8).

## D. Daftar terbang

| Kode | Metode | `FUZZY_ENABLE` / `NAV_PID_ENABLE` | Radius | Isi | Jumlah |
|---|---|---|---|---|---|
| U1 | L1 tetap | 0 / 0 | 100 | Uji awal, tidak masuk data | 1 |
| F1–F5 | **Fuzzy** | 1 / 0 | 100 | 4 leg + loiter 4 putaran | 5 |
| L1–L5 | **L1 tetap** | 0 / 0 | 100 | 4 leg + loiter 4 putaran | 5 |
| P1 | **PID** | 0 / **1** | 100 | 4 leg + loiter 4 putaran | 1 |
| FK1–FK3 | Fuzzy | 1 / 0 | **80** | loiter saja, 8 putaran | 3 |
| LK1–LK3 | L1 tetap | 0 / 0 | **80** | loiter saja, 8 putaran | 3 |
| | | | | **Total** | **18** |

**Urutan:** selang-seling dalam satu sesi, misalnya F1 → L1 → F2 → L2 → …, supaya perbedaan angin tidak tercampur ke hasil. Jangan semua fuzzy di satu hari dan semua L1 di hari lain.

**Grup R80 (FK, LK):** hanya saat angin **≤ 4 m/s**. Kalau lebih kencang, tunda; bank akan mentok di sisi angin belakang.

**P1:** terbangkan setelah minimal F1 dan L1 berhasil di hari yang sama, supaya ada pembanding dengan angin serupa.

## E. Uji awal U1 (wajib, sekali di awal hari)

- [ ] Di darat: CH7 RENDAH ≥ 1 s lalu TINGGI → aileron bergeser selama 2 s **searah stick roll kanan**. Kalau terbalik, **jangan terbang**.
- [ ] Di darat: masuk AUTO, set WP 3 dari Mission Planner, pindah STABILIZE, masuk AUTO lagi → WP aktif kembali ke **1**.
- [ ] Di udara: loiter, cek di Mission Planner nilai `LOIT_RAD` = **100** terus (tidak naik ke 110+).
- [ ] Di udara: 2–3 gangguan +300 µs di loiter. Pesawat pulih dengan aman, roll tidak lebih dari sekitar 55°.
- [ ] Kalau terasa terlalu agresif: turunkan `DIST_US` ke 250 atau `LOIT_BANK_DEG` ke 42, dan **pakai nilai itu untuk semua terbang berikutnya**.

## F. Prosedur tiap terbang (F, L, P)

Sebelum takeoff:
- [ ] `FUZZY_ENABLE` dan `NAV_PID_ENABLE` sesuai kode terbang. P1: `FUZZY_ENABLE` 0, `NAV_PID_ENABLE` 1, cek pesan "NAV: PID ON".
- [ ] `DIST_US` = **+300**.
- [ ] CH7 di posisi **RENDAH**.
- [ ] Catat jam, baterai, perkiraan angin.

Di udara:
1. Takeoff STABILIZE, naik ke ketinggian misi.
2. Masuk **AUTO** (misi otomatis mulai dari WP1).
3. Leg pesawat → WP1: **tanpa gangguan**.
4. Leg U, T, S, B: **1 gangguan per leg**, setelah pesawat lurus di garis, jangan menjelang WP.
5. Loiter putaran 1: gangguan di sisi **utara** dan **selatan**.
6. Loiter putaran 2: gangguan di sisi **timur** dan **barat**.
7. Ubah `DIST_US` ke **−300** dari Mission Planner.
8. Loiter putaran 3: sisi **utara** dan **selatan**. Putaran 4: sisi **timur** dan **barat**.
9. Jeda antar gangguan **minimal 15 detik**.
10. Pindah ke **STABILIZE** (misi otomatis di-reset), mendarat.

Grup R80 (FK, LK):
1. Takeoff STABILIZE, masuk AUTO, langsung ke loiter (leg tidak perlu diberi gangguan).
2. Putaran 1–4: `DIST_US` +300, **1 gangguan per putaran** di sisi U, T, S, B (1 sisi per putaran).
3. Ubah `DIST_US` ke −300. Putaran 5–8: sisi U, T, S, B lagi.

Setelah mendarat:
- [ ] Kembalikan `DIST_US` ke +300 untuk terbang berikutnya.
- [ ] Ganti `FUZZY_ENABLE` sesuai terbang berikutnya.
- [ ] Setelah P1: **`NAV_PID_ENABLE` kembali 0** (cek pesan "NAV: L1 (PID OFF)").
- [ ] Isi lembar catatan.

## G. Lembar catatan per terbang

| Kode | Jam | File tlog | File log SD | Angin (m/s, dari) | Baterai awal/akhir | Gangguan + (jumlah) | Gangguan − (jumlah) | Catatan (telemetri putus, keluar loiter, dll) |
|---|---|---|---|---|---|---|---|---|
| U1 | | | | | | | | |
| F1 | | | | | | | | |
| L1 | | | | | | | | |
| F2 | | | | | | | | |
| L2 | | | | | | | | |
| F3 | | | | | | | | |
| L3 | | | | | | | | |
| F4 | | | | | | | | |
| L4 | | | | | | | | |
| F5 | | | | | | | | |
| L5 | | | | | | | | |
| P1 | | | | | | | | |
| FK1 | | | | | | | | |
| LK1 | | | | | | | | |
| FK2 | | | | | | | | |
| LK2 | | | | | | | | |
| FK3 | | | | | | | | |
| LK3 | | | | | | | | |

## H. Analisis setelah terbang

- `analyze_sdlog_gangguan_kecepatan.py LOG.CSV -o flight_... --radius 100 --bank 45` (R80: `--radius 80`).
- `analyze_fuzzy_sdlog.py` untuk terbang fuzzy.
- `analyze_tlog_loiter.py` dan `analyze_tlog_l1_auto.py` untuk tlog.
- Data lama (19–29 Okt) dipakai sebagai **uji pendahuluan**, tidak dicampur dengan data ini.
