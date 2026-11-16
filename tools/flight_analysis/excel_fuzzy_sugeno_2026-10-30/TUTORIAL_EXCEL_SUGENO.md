# Tutorial Excel: fuzzy Sugeno orde-0 untuk periode L1 dari data terbang 30/10/2026

Tutorial ini menuntunmu mengerjakan **semuanya sendiri di Excel**: dari data
mentah lima terbang (periode L1 11, 13, 15, 17, 19 s) sampai periode keluaran
Sugeno. Tidak ada yang dimasukkan ke firmware.

## 0. Persiapan

### Isi folder

| File | Gunanya |
|---|---|
| `data_fuzzy_sugeno_KOSONG.xlsx` | **Kerjakan di sini.** Data mentah sudah ada (kuning), kolom hijau masih kosong |
| `data_fuzzy_sugeno_KUNCI.xlsx` | Sama persis, tetapi semua kolom hijau sudah berisi **rumus**. Pakai untuk mencocokkan, jangan disalin mentah-mentah |
| `TUTORIAL_EXCEL_SUGENO.md` | Dokumen ini |

Kedua file dibuat oleh `tools/flight_analysis/export_excel_fuzzy.py`. Rumus di
KUNCI sudah diuji dengan mesin penghitung rumus Excel (pustaka Python
`formulas`) pada 60 sampel pertama tiap periode: hasilnya sama dengan
perhitungan Python (selisih < 10⁻⁸), termasuk fase, statistik, dan pemilihan
aturan. Angka "titik cek" di tutorial ini berasal dari data lengkap.

### Warna sel

- **Kuning** = data mentah dari tlog. Jangan diubah.
- **Hijau** = kolom yang kamu hitung dengan rumus.
- **Biru** = parameter atau keputusan desain.

### Hal teknis Excel

1. Pakai **Excel 2019 atau Microsoft 365** (ada fungsi `MAXIFS`).
2. Saat file dibuka muncul *Protected View*: klik **Enable Editing**, supaya
   rumus dihitung.
3. Excel berbahasa Indonesia biasanya memakai **titik koma** sebagai pemisah
   argumen dan **koma** sebagai desimal. Di tutorial rumus ditulis gaya
   Inggris. Contoh `=MAX(0,MIN(1,(0.5-AE2)/0.25))` diketik sebagai
   `=MAX(0;MIN(1;(0,5-AE2)/0,25))`.
4. Beberapa sel sudah diberi **nama** (Formulas → Name Manager): `lat0`, `lon0`,
   `K_M`, `BAND`, `E_MAX`, `DE_MAX`, `P0`, `S_SKALA`, `MIN_NEFF`. Semua menunjuk
   ke lembar **Parameter** kolom C. Di rumus, nama itu bisa dipakai seperti
   alamat sel.
5. Cara cepat mengisi satu kolom: ketik rumus di baris 2, lalu **klik dua kali
   kotak kecil di pojok kanan bawah sel** (fill handle). Rumus turun sampai
   baris data terakhir.

### Peta tahap

| Tahap | Isi | Lembar |
|---|---|---|
| A | Pengolahan data: lat/lon → e dan d\|e\|/dt, fase, J dan O | Parameter, Waypoint, P11–P19 |
| B | Menentukan semesta: E_MAX dan DE_MAX dari data | Gabungan, Statistik |
| C | **Fuzzifikasi** | P11–P19, TabelMF |
| D | **Rule base** | BuktiRule, RuleBase |
| E | **Inferensi** (firing strength, min) | P11–P19 |
| F | **Defuzzifikasi** (weighted average) | P11–P19 |
| G | Ringkasan dan grafik | Ringkasan |

Kerjakan dulu **seluruh tahap di lembar P11**, cocokkan dengan KUNCI, baru salin
ke P13–P19 (bagian A.12).

---

## Tahap A. Pengolahan data

### A.1 Kenali data mentah (kolom A–F di P11..P19)

| Kolom | Isi | Satuan |
|---|---|---|
| A `t_s` | waktu sampel (5 Hz, tiap 0,2 s) | s |
| B `lat`, C `lon` | posisi GPS | derajat |
| D `gs_mps` | groundspeed GPS | m/s |
| E `cog_deg` | course GPS (arah gerak, 0 = utara, searah jarum jam) | derajat |
| F `wp_aktif` | nomor WP yang sedang dituju | – |

Baris sudah dibatasi pada mode AUTO, leg WP1→WP2 sampai WP5→WP6 (wp_aktif ≥ 2).
Leg masuk→WP1 dibuang karena tidak ada garis yang dilacak. Jumlah sampel: P11
408, P13 377, P15 392, P17 358, P19 359 (total 1.894).

### A.2 Parameter dan titik nol

Di lembar **Parameter**, isi:

| Nama | Isi | Alasan |
|---|---|---|
| lat0 | `=Waypoint!B2` | HOME (seq 0) jadi titik nol koordinat |
| lon0 | `=Waypoint!C2` | |
| K_M | 111320 (sudah ada) | panjang 1 derajat lintang ≈ 111.320 m |
| BAND | 5 (sudah ada) | pita pulih \|e\| ≤ 5 m |
| P0, S_SKALA | isi nanti di Tahap D | |
| E_MAX, DE_MAX | isi nanti di Tahap B | |

### A.3 Waypoint ke meter (lembar Waypoint, kolom D dan E)

Dekat titik nol, bumi bisa dianggap datar:

```
x (timur) = (lon − lon0) · 111320 · cos(lat0)
y (utara) = (lat − lat0) · 111320
```

| Sel | Rumus |
|---|---|
| D2 | `=(C2-lon0)*K_M*COS(RADIANS(lat0))` |
| E2 | `=(B2-lat0)*K_M` |

Tarik ke bawah sampai seq 6. **Cek:** WP1 = (58,91; 193,64), WP2 = (−53,02;
−139,50), WP6 = (336,61; −246,56).

### A.4 Posisi dan kecepatan pesawat dalam meter (P11, kolom G–J)

| Kolom | Rumus baris 2 | Arti |
|---|---|---|
| G `x_m` | `=(C2-lon0)*K_M*COS(RADIANS(lat0))` | posisi timur |
| H `y_m` | `=(B2-lat0)*K_M` | posisi utara |
| I `ve_mps` | `=D2*SIN(RADIANS(E2))` | kecepatan ke timur |
| J `vn_mps` | `=D2*COS(RADIANS(E2))` | kecepatan ke utara |

Course diukur dari utara, jadi komponen timur memakai **sin** dan utara **cos**.

### A.5 Garis leg yang sedang dilacak (kolom K–Q)

Leg yang sedang diterbangkan adalah garis dari **WP sebelumnya (A)** ke **WP
tujuan (B)**. Ambil koordinatnya dari lembar Waypoint dengan INDEX/MATCH:

| Kolom | Rumus baris 2 |
|---|---|
| K `Ax` | `=INDEX(Waypoint!$D$2:$D$20,MATCH(F2-1,Waypoint!$A$2:$A$20,0))` |
| L `Ay` | `=INDEX(Waypoint!$E$2:$E$20,MATCH(F2-1,Waypoint!$A$2:$A$20,0))` |
| M `Bx` | `=INDEX(Waypoint!$D$2:$D$20,MATCH(F2,Waypoint!$A$2:$A$20,0))` |
| N `By` | `=INDEX(Waypoint!$E$2:$E$20,MATCH(F2,Waypoint!$A$2:$A$20,0))` |
| O `dx` | `=M2-K2` |
| P `dy` | `=N2-L2` |
| Q `L_leg` | `=SQRT(O2^2+P2^2)` |

Tanda `$` membuat rentang tabel Waypoint tidak ikut bergeser saat ditarik ke bawah.

### A.6 Cross-track error e (kolom R)

e = jarak tegak lurus pesawat ke garis A→B, **positif bila pesawat di kanan
garis**. Rumusnya hasil kali silang vektor garis (dx, dy) dengan vektor dari A
ke pesawat (x − Ax, y − Ay), dibagi panjang garis:

```
e = ( dy·(x − Ax) − dx·(y − Ay) ) / L
```

| Kolom | Rumus |
|---|---|
| R `e_m` | `=(P2*(G2-K2)-O2*(H2-L2))/Q2` |
| S `abs_e` | `=ABS(R2)` |

### A.7 Laju |e| (kolom T dan U)

Vektor satuan tegak lurus garis ke arah kanan adalah n = (dy, −dx) / L. Laju e
= kecepatan pesawat diproyeksikan ke n:

```
de/dt   = ( ve·dy − vn·dx ) / L                 (bertanda, ikut e)
d|e|/dt = tanda(e) · de/dt                      (− = mendekati garis, + = menjauhi)
```

| Kolom | Rumus |
|---|---|
| T `e_rate` | `=(I2*P2-J2*O2)/Q2` |
| U `de` | `=SIGN(R2)*T2` |

Kenapa tidak memakai selisih \|e\| antar sampel? Posisi GPS 5 Hz membuat turunan
numerik berderau dan bisa memberi puncak palsu melebihi groundspeed.
Proyeksi kecepatan jauh lebih halus.

**Titik cek P11 baris 2** (t = 861,407 s, wp 2):

| x | y | ve | vn | dx | dy | L | e | e_rate | d\|e\|/dt |
|---|---|---|---|---|---|---|---|---|---|
| 74,86 | 195,67 | −14,150 | −3,199 | −111,93 | −333,14 | 351,44 | −14,473 | 12,394 | −12,394 |

Artinya: pesawat 14,5 m di kiri garis WP1→WP2 dan sedang mendekati garis
dengan laju 12,4 m/s (baru selesai belok).

### A.8 Waktu sejak awal leg (kolom V)

| Kolom | Rumus baris 2 |
|---|---|
| V `t_leg` | `=IF(F2=F1,V1+(A2-A1),0)` |

Bila WP aktif sama dengan baris di atasnya, waktu ditambah; bila berbeda (leg
baru), mulai lagi dari 0. Di baris 2, F1 berisi judul kolom sehingga hasilnya 0.

### A.9 Fase tunak atau transisi (kolom W–Z)

Definisi: **tunak** dimulai saat \|e\| ≤ 5 m **dan bertahan 2 s** (11 sampel);
sebelumnya **transisi**.

| Kolom | Rumus baris 2 | Arti |
|---|---|---|
| W `dalam5` | `=IF(S2<=BAND,1,0)` | 1 bila \|e\| ≤ 5 m |
| X `tahan2s` | `=IF(COUNTIFS(F2:F12,F2,W2:W12,1)=11,1,0)` | 1 bila 11 sampel (2 s) ke depan semuanya di leg yang sama **dan** dalam 5 m |
| Y `pulih` | `=IF(F2<>F1,X2,MAX(Y1,X2))` | 1 sejak pertama kali pulih di leg ini |
| Z `fase` | `=IF(Y2=1,"tunak","transisi")` | |

### A.10 Bahan bukti rule base: J dan O (kolom AA–AD)

Kolom ini dipakai di Tahap D untuk menilai "apa yang terjadi 6 s ke depan" di
tiap sampel.

| Kolom | Rumus baris 2 | Arti |
|---|---|---|
| AA `tanda` | `=IF(R2<0,-1,1)` | sisi garis: −1 kiri, +1 kanan |
| AB `valid6s` | `=IF(COUNTIF(F3:F32,F2)=30,1,0)` | 1 bila 30 sampel (6 s) sesudahnya masih leg yang sama |
| AC `J_m` | `=IF(AB2=1,AVERAGE(S3:S32),0)` | \|e\| rata-rata 6 s ke depan |
| AD `O_m` | `=IF(AB2=1,MAXIFS(S3:S32,AA3:AA32,-AA2),0)` | overshoot: \|e\| terbesar di **sisi seberang** garis dalam 6 s ke depan (0 bila tidak menyeberang) |

Excel tanpa `MAXIFS`: ketik `=IF(AB2=1,MAX(IF(AA3:AA32=-AA2,S3:S32,0)),0)` lalu
tekan **Ctrl+Shift+Enter**.

**Titik cek P11 baris 2:** t_leg = 0; fase = transisi; valid6s = 1; J = 4,550;
O = 8,931.

### A.11 Cocokkan

Buka KUNCI lembar P11, bandingkan kolom G–AD baris 2 dan beberapa baris acak.
Kalau sudah sama, lanjut.

### A.12 Salin ke periode lain

Blok rumus G2:AY2 (setelah semua tahap selesai) bisa disalin ke lembar P13–P19
di sel G2, lalu tarik ke bawah (klik dua kali fill handle). Rumusnya tidak
menyebut nama lembar, jadi otomatis memakai data lembar tujuan.

---

## Tahap B. Menentukan semesta (E_MAX dan DE_MAX)

Sesuai catatanmu, batas atas harus ditentukan **sebelum** fuzzifikasi. Di sini
batas diambil dari data kelima terbang sekaligus.

### B.1 Satukan kelima periode (lembar Gabungan)

Kolom A sudah berisi periode per baris (baris 2–409 = P11, 410–786 = P13,
787–1178 = P15, 1179–1536 = P17, 1537–1895 = P19). Isi:

| Kolom | Rumus (contoh baris 2, untuk baris P11) | Arti |
|---|---|---|
| B `abs_e` | `=P11!S2` | |
| C `de` | `=P11!U2` | |
| D `fase` | `=P11!Z2` | |
| E `t_leg` | `=P11!V2` | |
| F `abs_e_transisi` | `=IF(D2="transisi",B2,"")` | \|e\| hanya saat transisi |
| G `abs_de_tunak` | `=IF(D2="tunak",ABS(C2),"")` | \|d\|e\|/dt\| hanya saat tunak |
| H `de_transisi_3s` | `=IF(AND(D2="transisi",E2>3),C2,"")` | d\|e\|/dt transisi, 3 s awal leg dibuang |
| I `periode_sugeno` | `=P11!AY2` | diisi setelah Tahap F |

Untuk baris P13 (mulai baris 410) rujukannya `=P13!S2` dan seterusnya. Cara
termudah: isi B2:E2, tarik sampai baris 409; lalu di baris 410 ketik ulang
dengan `P13!` dan tarik sampai 786; begitu seterusnya. Kolom F–H cukup ditarik
dari baris 2 sampai 1895.

Isian `""` (teks kosong) sengaja dipakai: MEDIAN dan PERCENTILE mengabaikan teks,
jadi kolom F–H otomatis tersaring.

### B.2 Hitung jangkar (lembar Statistik)

| Sel | Besaran | Rumus | Hasil |
|---|---|---|---|
| B2 | jumlah sampel | `=COUNT(Gabungan!B2:B1895)` | 1894 |
| B3 | median \|e\| transisi | `=MEDIAN(Gabungan!F2:F1895)` | 14,59 m |
| B4 | p95 \|d\|e\|/dt\| tunak | `=PERCENTILE.INC(Gabungan!G2:G1895,0.95)` | 3,48 m/s |
| B5 | p10 d\|e\|/dt transisi (t_leg > 3 s) | `=PERCENTILE.INC(Gabungan!H2:H1895,0.1)` | −6,46 m/s |

### B.3 Ubah jangkar menjadi E_MAX dan DE_MAX

Himpunan di catatanmu punya titik-titik tetap pada skala ternormalisasi
(0,25 / 0,5 / 0,75 dan −0,5 / 0 / 0,5). Jadi batas atas dipilih supaya titik
itu jatuh di tempat yang bermakna:

| Sel | Jangkar | Rumus | Hasil |
|---|---|---|---|
| B7 | Kecil penuh sampai 0,25·E_MAX = pita pulih 5 m | `=4*BAND` | 20,00 |
| B8 | Besar penuh mulai 0,75·E_MAX = median \|e\| transisi | `=B3/0.75` | 19,45 |
| B9 | rata-rata | `=AVERAGE(B7:B8)` | 19,73 |
| **B10** | **E_MAX** | `=ROUND(B9,0)` | **20 m** |
| B13 | Zero = 0,5 di ±0,25·DE_MAX = p95 tunak | `=B4/0.25` | 13,91 |
| B14 | Negatif penuh di −0,5·DE_MAX = p10 transisi | `=-B5/0.5` | 12,93 |
| B15 | rata-rata | `=AVERAGE(B13:B14)` | 13,42 |
| **B16** | **DE_MAX** | `=ROUND(B15,0)` | **13 m/s** |

Lalu di lembar Parameter isi `E_MAX` = `=Statistik!B10` dan `DE_MAX` =
`=Statistik!B16`.

Bacaan: setiap batas punya dua jangkar yang hampir sama (20,0 vs 19,5; 13,9 vs
12,9), jadi pilihannya tidak bergantung pada satu angka kebetulan.

---

## Tahap C. Fuzzifikasi

### C.1 Normalisasi (kolom AE, AF)

```
e_fn  = min(1, |e| / E_MAX)          → 0 … 1
de_fn = clip(d|e|/dt / DE_MAX, −1, 1) → −1 … 1
```

| Kolom | Rumus |
|---|---|
| AE `e_fn` | `=MIN(1,S2/E_MAX)` |
| AF `de_fn` | `=MAX(-1,MIN(1,U2/DE_MAX))` |

### C.2 Fungsi keanggotaan (kolom AG–AL)

Rumus bercabang di catatanmu, misalnya

```
μ_Kecil(u) = 1 bila u ≤ 0,25 ;  (0,5 − u)/0,25 bila 0,25 < u < 0,5 ;  0 bila u ≥ 0,5
```

bisa ditulis **satu baris** dengan MAX dan MIN: `(0,5 − u)/0,25` dibatasi
ke [0, 1]. Di bawah 0,25 nilainya > 1 lalu dipotong jadi 1; di atas 0,5 nilainya
negatif lalu dipotong jadi 0. Hasilnya persis sama dengan tiga cabang itu.

| Kolom | Himpunan | Rumus |
|---|---|---|
| AG | Kecil | `=MAX(0,MIN(1,(0.5-AE2)/0.25))` |
| AH | Sedang | `=MAX(0,MIN((AE2-0.25)/0.25,(0.75-AE2)/0.25))` |
| AI | Besar | `=MAX(0,MIN(1,(AE2-0.5)/0.25))` |
| AJ | Negatif | `=MAX(0,MIN(1,(0-AF2)/0.5))` |
| AK | Zero | `=MAX(0,MIN((AF2+0.5)/0.5,(0.5-AF2)/0.5))` |
| AL | Positif | `=MAX(0,MIN(1,AF2/0.5))` |

Untuk segitiga (Sedang, Zero), MIN memilih sisi naik atau sisi turun yang lebih
rendah, lalu MAX(0, …) membuang nilai negatif di luar alas segitiga.

**Titik cek P11 baris 2:** e_fn = 0,7237; de_fn = −0,9534; Kecil 0; Sedang
0,1053; Besar 0,8947; Negatif 1; Zero 0; Positif 0.

Dalam satuan fisik: Kecil penuh sampai 5 m dan nol di 10 m; Sedang puncak 10 m;
Besar penuh mulai 15 m. Negatif penuh di −6,5 m/s; Zero puncak di 0; Positif
penuh di +6,5 m/s.

### C.3 Grafik fungsi keanggotaan (lembar TabelMF)

Kolom A (\|e\| 0–45 m) dan G (d\|e\|/dt −23…23 m/s) sudah terisi. Isi B–E dan
H–K dengan rumus yang sama seperti C.1–C.2 (merujuk A dan G di baris yang sama).
Lalu blok A1:A92 dan C1:E92 → **Insert → Scatter → Scatter with Straight
Lines**. Ulangi untuk G, I:K. Grafik ini bisa langsung dipakai di naskah.

---

## Tahap D. Menentukan rule base

### D.1 Bentuk aturan dan keluaran

Dua masukan × tiga himpunan = **9 aturan**:

```
R1 Kecil & Negatif   R2 Kecil & Zero    R3 Kecil & Positif
R4 Sedang & Negatif  R5 Sedang & Zero   R6 Sedang & Positif
R7 Besar & Negatif   R8 Besar & Zero    R9 Besar & Positif

JIKA e_fn adalah A DAN de_fn adalah B MAKA z = c   (c konstanta: Sugeno orde-0)
periode = P0 + S·z
```

Isi di lembar Parameter: **P0 = 15** (periode kompromi terbaik uji
konvensional) dan **S_SKALA = 4**, sehingga keluaran berada di 11–19 s, persis
rentang yang sudah diterbangkan. Nilai c yang boleh dipakai:

| c | −1 | −0,5 | 0 | +0,5 | +1 |
|---|---|---|---|---|---|
| periode | 11 s | 13 s | 15 s | 17 s | 19 s |

Setiap konsekuen adalah **satu periode yang benar-benar pernah diterbangkan**.

### D.2 Bobot sampel per aturan

Untuk menilai aturan, setiap sampel perlu tahu seberapa ia termasuk daerah
aturan itu. Bobotnya α = min(μ masukan 1, μ masukan 2), yang juga dipakai di
inferensi. Jadi kerjakan dulu **kolom α di Tahap E.1** (AM–AU), lalu kembali ke
sini.

### D.3 Biaya per aturan per periode (lembar BuktiRule)

Untuk aturan k dan periode p:

```
n_eff = Σ α_k · valid6s                           (jumlah sampel efektif)
C     = Σ α_k · valid6s · (J + O) / n_eff         (biaya: lebih kecil = lebih baik)
```

C kecil berarti **cepat mendekati garis (J kecil) dan tidak melewati garis
(O kecil)**.

Rumus untuk R1 (baris 4) dan periode 11 s (P11 berisi data baris 2–409, α R1
di kolom AM):

| Sel | Rumus |
|---|---|
| B4 `n_eff 11 s` | `=SUMPRODUCT(P11!$AM$2:$AM$409,P11!$AB$2:$AB$409)` |
| G4 `C 11 s` | `=IF(B4>0,SUMPRODUCT(P11!$AM$2:$AM$409,P11!$AB$2:$AB$409,P11!$AC$2:$AC$409+P11!$AD$2:$AD$409)/B4,0)` |
| L4 `C' 11 s` | `=IF(B4>=MIN_NEFF,G4,1000000000)` |

- Untuk periode lain ganti nama lembar dan baris terakhir: P13 → 378, P15 → 393,
  P17 → 359, P19 → 360.
- Untuk aturan lain ganti kolom α: R1 AM, R2 AN, R3 AO, R4 AP, R5 AQ, R6 AR,
  R7 AS, R8 AT, R9 AU.
- C' membuang periode yang sampelnya terlalu sedikit (n_eff < 5) dengan memberi
  biaya sangat besar.

Pilih periode terbaik dan ubah ke c:

| Sel | Rumus |
|---|---|
| Q4 `Periode terpilih` | `=CHOOSE(MATCH(MIN(L4:P4),L4:P4,0),11,13,15,17,19)` |
| R4 `c terpilih` | `=(Q4-P0)/S_SKALA` |

**Titik cek (C dalam meter, n_eff dalam kurung):**

| Aturan | 11 s | 13 s | 15 s | 17 s | 19 s | Terpilih |
|---|---|---|---|---|---|---|
| R1 Kecil & Negatif | **12,34** (30,5) | 13,01 (27,5) | 12,84 (30,7) | 16,70 (26,9) | 22,82 (22,0) | 11 s |
| R2 Kecil & Zero | **1,56** (112,7) | 2,87 (80,6) | 1,98 (87,9) | 4,39 (47,3) | 8,26 (46,7) | 11 s |
| R3 Kecil & Positif | **7,36** (16,2) | 12,62 (15,5) | 8,03 (16,9) | 12,76 (15,7) | 11,23 (25,4) | 11 s |
| R4 Sedang & Negatif | 16,15 (15,6) | 15,84 (15,8) | **13,17** (18,5) | 14,22 (23,0) | 19,98 (12,7) | 15 s |
| R5 Sedang & Zero | 4,78 (3,9)\* | 2,13 (2,7)\* | **4,63** (15,0) | 6,65 (27,0) | 7,96 (20,5) | 15 s |
| R6 Sedang & Positif | 12,25 (8,2) | 17,91 (6,6) | **11,10** (10,0) | 14,13 (11,4) | 14,89 (9,7) | 15 s |
| R7 Besar & Negatif | 26,46 (40,9) | 26,22 (42,6) | 25,12 (35,2) | **20,24** (41,0) | 25,61 (43,5) | **17 s** |
| R8 Besar & Zero | 11,12 (10,7) | 10,01 (13,1) | **8,69** (23,8) | 10,77 (22,3) | 14,63 (17,3) | 15 s |
| R9 Besar & Positif | 14,16 (23,6) | 20,03 (27,3) | **12,21** (17,2) | 16,64 (19,1) | 21,59 (26,7) | 15 s |

\* n_eff < 5, tidak ikut dipilih.

### D.4 Isi tabel aturan (lembar RuleBase)

Isi B4:D6 dengan c. Dari data: `=BuktiRule!R4` (R1), `=BuktiRule!R5` (R2), dst.
Hasilnya:

| \|e\| \ d\|e\|/dt | Negatif | Zero | Positif |
|---|---|---|---|
| **Kecil** | −1 (11 s) | −1 (11 s) | −1 (11 s) |
| **Sedang** | 0 (15 s) | 0 (15 s) | 0 (15 s) |
| **Besar** | **+0,5 (17 s)** | 0 (15 s) | 0 (15 s) |

Baris 11 lembar RuleBase (B11:J11) sudah merujuk ke tabel ini dalam urutan
R1…R9; baris inilah yang dipakai rumus defuzzifikasi.

**Kamu boleh mengubah c secara manual** bila punya alasan. Contoh yang sudah
kita bahas: baris Kecil bisa diganti 15 s (c = 0), karena saat tunak 11 s dan
15 s sama telitinya, tetapi 15 s hanya butuh separuh gerakan nav_roll. Kalau
mengubah, ganti satu baris sekaligus (bukan satu sel saja) dan catat alasannya.

---

## Tahap E. Inferensi (firing strength)

### E.1 Sembilan kolom α (AM–AU)

Operator DAN = **min** (sesuai catatanmu):

```
α_k = min( μ_E(e_fn) , μ_D(de_fn) )
```

| Kolom | Aturan | Rumus |
|---|---|---|
| AM | R1 Kecil & Negatif | `=MIN(AG2,AJ2)` |
| AN | R2 Kecil & Zero | `=MIN(AG2,AK2)` |
| AO | R3 Kecil & Positif | `=MIN(AG2,AL2)` |
| AP | R4 Sedang & Negatif | `=MIN(AH2,AJ2)` |
| AQ | R5 Sedang & Zero | `=MIN(AH2,AK2)` |
| AR | R6 Sedang & Positif | `=MIN(AH2,AL2)` |
| AS | R7 Besar & Negatif | `=MIN(AI2,AJ2)` |
| AT | R8 Besar & Zero | `=MIN(AI2,AK2)` |
| AU | R9 Besar & Positif | `=MIN(AI2,AL2)` |

Paling banyak 4 aturan bernilai > 0 di setiap baris.

---

## Tahap F. Defuzzifikasi (Sugeno orde-0, weighted average)

```
z* = Σ α_k · c_k / Σ α_k
periode = P0 + S · z*
```

| Kolom | Rumus | Arti |
|---|---|---|
| AV `sum_alpha` | `=SUM(AM2:AU2)` | Σα |
| AW `sum_alpha_c` | `=SUMPRODUCT(AM2:AU2,RuleBase!$B$11:$J$11)` | Σα·c (sembilan α dikali sembilan c lalu dijumlah) |
| AX `z` | `=AW2/AV2` | z* ternormalisasi |
| AY `periode_s` | `=P0+S_SKALA*AX2` | periode L1 keluaran Sugeno |

### Contoh 1: P11 baris 2 (awal leg, error besar mendekat cepat)

| Langkah | Hitungan |
|---|---|
| Normalisasi | e_fn = 14,473/20 = 0,7237; de_fn = −12,394/13 = −0,9534 |
| Fuzzifikasi | Sedang = (0,75 − 0,7237)/0,25 = 0,1053; Besar = (0,7237 − 0,5)/0,25 = 0,8947; Negatif = 1 |
| Inferensi | R4 = min(0,1053; 1) = 0,1053; R7 = min(0,8947; 1) = 0,8947 (lainnya 0) |
| Defuzzifikasi | Σα = 1; Σα·c = 0,1053·0 + 0,8947·0,5 = 0,4473; z* = 0,4473 |
| Keluaran | periode = 15 + 4·0,4473 = **16,79 s** |

### Contoh 2: P15 baris 19 (error sedang hampir diam)

| Langkah | Hitungan |
|---|---|
| Data | \|e\| = 10,65 m; d\|e\|/dt = −0,163 m/s |
| Normalisasi | e_fn = 0,5325; de_fn = −0,0126 |
| Fuzzifikasi | Sedang 0,8701; Besar 0,1299; Negatif 0,0251; Zero 0,9749 |
| Inferensi | R4 = 0,0251; R5 = 0,8701; R7 = 0,0251; R8 = 0,1299 |
| Defuzzifikasi | Σα = 1,0503; Σα·c = 0,0251·0,5 = 0,0126; z* = 0,0120 |
| Keluaran | periode = 15 + 4·0,0120 = **15,05 s** |

Di contoh 2, Σα ≠ 1 karena operator min. Itulah sebabnya z* dibagi Σα.

---

## Tahap G. Ringkasan dan grafik

### G.1 Lembar Ringkasan (kolom B = 11 s; ganti nama lembar dan baris terakhir untuk kolom lain)

| Metrik | Rumus (11 s) |
|---|---|
| Jumlah sampel | `=COUNT(P11!S2:S409)` |
| \|e\| rata-rata | `=AVERAGE(P11!S2:S409)` |
| e RMS | `=SQRT(SUMSQ(P11!R2:R409)/COUNT(P11!R2:R409))` |
| \|e\| maks | `=MAX(P11!S2:S409)` |
| \|e\| rata-rata fase tunak | `=AVERAGEIF(P11!Z2:Z409,"tunak",P11!S2:S409)` |
| % waktu \|e\| ≤ 5 m | `=100*COUNTIF(P11!S2:S409,"<=5")/COUNT(P11!S2:S409)` |
| % waktu transisi | `=100*COUNTIF(P11!Z2:Z409,"transisi")/COUNTA(P11!Z2:Z409)` |
| Periode Sugeno rata-rata / min / maks | `=AVERAGE(P11!AY2:AY409)` dst. |

**Titik cek:**

| Metrik | 11 s | 13 s | 15 s | 17 s | 19 s |
|---|---|---|---|---|---|
| \|e\| rata-rata (m) | 6,67 | 8,96 | 6,78 | 9,70 | 12,02 |
| e RMS (m) | 11,02 | 14,00 | 10,23 | 12,73 | 15,41 |
| \|e\| maks (m) | 40,47 | 42,54 | 37,68 | 31,00 | 37,75 |
| \|e\| tunak (m) | 1,26 | 1,67 | 1,23 | 3,23 | 5,04 |
| % waktu ≤ 5 m | 65,0 | 58,6 | 57,9 | 39,4 | 23,7 |
| % waktu transisi | 37,7 | 45,9 | 48,5 | 55,6 | 57,7 |
| Periode Sugeno rata-rata (s) | 12,46 | 12,82 | 12,64 | 13,27 | 13,53 |

Angka metrik ini sama dengan `RINGKASAN.md` (selisih pembulatan saja).

### G.2 Grafik putar ulang satu leg

Di P15, filter kolom F = 3 (leg WP2→WP3). Blok kolom V (t_leg), R (e) dan AY
(periode) → Insert → Scatter with Straight Lines; pindahkan seri periode ke
sumbu kedua (Format Data Series → Secondary Axis). Terlihat pola: awal leg
≈ 17 s, turun saat memotong garis, naik saat kembali dari sisi seberang,
lalu 11 s saat tunak.

> Ini putar ulang **loop terbuka**: masukannya berasal dari terbang dengan
> periode tetap. Grafik ini memeriksa perilaku aturan, bukan bukti fuzzy lebih
> baik. Bukti tetap dari uji terbang dengan setting yang sama.

---

## Kesalahan yang sering terjadi

| Gejala | Penyebab | Perbaikan |
|---|---|---|
| `#NAME?` | nama sel (E_MAX dll.) belum ada, atau Excel tidak mengenal `MAXIFS` | cek Name Manager; pakai rumus alternatif A.10 |
| `#N/A` di Ax/Bx | seq WP tidak ditemukan | pastikan tabel Waypoint lengkap seq 0–6 |
| `#DIV/0!` di z | semua α = 0 | kolom μ salah; dengan himpunan yang benar, Σα selalu > 0 |
| Semua rumus menampilkan rumus, bukan angka | sel berformat Text | ubah format ke General lalu tekan F2 + Enter |
| Angka beda sedikit dari KUNCI | pembulatan atau rentang baris berbeda | cek rentang baris akhir tiap lembar (409/378/393/359/360) |
| Rumus ditolak saat diketik | pemisah argumen | pakai `;` dan koma desimal (bagian 0) |

## Hubungan dengan desain Python

`design_fuzzy_sugeno.py` (dokumen `fuzzy_sugeno_2026-10-30/DESAIN_FUZZY_SUGENO.md`)
melakukan langkah yang sama. Versi Excel menghitung d\|e\|/dt dari arah garis
antar-WP (bukan garis yang di-fit dari lintasan) dan memakai kecepatan GPS
pada jam FC, sehingga angka C berbeda sedikit. Hasil akhirnya **sama**: E_MAX
20 m, DE_MAX 13 m/s, dan rule base yang sama persis.
