#!/usr/bin/env python3
"""Workbook Excel untuk latihan fuzzy Sugeno orde-0 (periode L1) secara manual.

Mengambil data mentah per sampel dari tlog uji L1 konvensional (lat, lon,
groundspeed dan course GPS, WP aktif) lalu menulis dua workbook:

  data_fuzzy_sugeno_KOSONG.xlsx  data mentah + kolom kosong untuk dikerjakan sendiri
  data_fuzzy_sugeno_KUNCI.xlsx   sama, tetapi semua kolom sudah berisi RUMUS Excel
                                 (bukan angka), untuk dicocokkan

Sampel yang dipakai sama dengan cte_per_sampel.csv hasil analyze_tlog_l1_auto.py
(waktu, WP aktif), leg WP1->WP2 dst. (wp_aktif >= 2). Kecepatan GPS_RAW_INT
diinterpolasi ke waktu sampel posisi.

Alur rumus (lihat TUTORIAL_EXCEL_SUGENO.md):
  lat/lon -> x, y (m) -> e (cross-track, + kanan) -> |e|, d|e|/dt
  -> fase tunak/transisi -> J, O (6 s ke depan) -> e_fn, de_fn -> mu (6)
  -> alpha (9, min) -> z* = sum(alpha c)/sum(alpha) -> periode = P0 + S z*

Run:
    python export_excel_fuzzy.py -o excel_fuzzy_sugeno_2026-10-30 \
        "11|flight_2026-10-30_P11|C:/.../2026-10-30 10-01-14.tlog" ...
"""

import argparse
import csv
import os

import numpy as np
from openpyxl import Workbook
from openpyxl.styles import Alignment, Font, PatternFill
from openpyxl.workbook.defined_name import DefinedName

import analyze_tlog_l1_auto as A

YELLOW = PatternFill("solid", fgColor="FFF2CC")   # data mentah
GREEN = PatternFill("solid", fgColor="E2EFDA")    # kolom yang dikerjakan (rumus)
BLUE = PatternFill("solid", fgColor="DDEBF7")     # parameter / keputusan desain
BOLD = Font(bold=True)
HOLD_N = 11      # 2 s pada 5 Hz (sampel j sampai j+10) untuk definisi pulih
FUT_N = 30       # 6 s ke depan pada 5 Hz
RULES = [(i, j) for i in range(3) for j in range(3)]
E_N, D_N = ("Kecil", "Sedang", "Besar"), ("Negatif", "Zero", "Positif")
PERIODS = (11, 13, 15, 17, 19)


def extract(folder, tlog):
    D = A.load(tlog)
    t0 = D["ATTITUDE"][0]["_t"]
    tp, lat = A.series(D, "GLOBAL_POSITION_INT", "lat", t0, 1e-7)
    _, lon = A.series(D, "GLOBAL_POSITION_INT", "lon", t0, 1e-7)
    tg, vel = A.series(D, "GPS_RAW_INT", "vel", t0, 0.01)
    _, cog = A.series(D, "GPS_RAW_INT", "cog", t0, 0.01)
    cog_u = np.degrees(np.unwrap(np.radians(cog)))
    rows = [r for r in csv.DictReader(open(os.path.join(folder, "cte_per_sampel.csv"))) if int(r["wp_aktif"]) >= 2]
    out = []
    for r in rows:
        t = float(r["t_s"])
        k = int(np.argmin(np.abs(tp - t)))
        out.append(dict(t=round(t, 3), lat=float(lat[k]), lon=float(lon[k]), gs=round(float(np.interp(t, tg, vel)), 3),
                        cog=round(float(np.interp(t, tg, cog_u)) % 360.0, 2), wp=int(r["wp_aktif"]),
                        cte_ref=float(r["cte_m(+kanan)"])))
    mission = A.last_mission(D, t0)
    return out, [(i, m[0], m[1]) for i, m in enumerate(mission)]


# ---------------------------------------------------------------- kolom lembar periode
# (huruf, judul, rumus baris r atau None untuk data mentah, penjelasan singkat)
def period_columns():
    c = []
    c += [("A", "t_s", None, "waktu (s)"), ("B", "lat", None, "lintang (derajat)"), ("C", "lon", None, "bujur (derajat)"),
          ("D", "gs_mps", None, "groundspeed GPS (m/s)"), ("E", "cog_deg", None, "course GPS (derajat, 0 = utara)"),
          ("F", "wp_aktif", None, "WP yang sedang dituju")]
    c += [("G", "x_m", "=(C{r}-lon0)*K_M*COS(RADIANS(lat0))", "timur dari HOME"),
          ("H", "y_m", "=(B{r}-lat0)*K_M", "utara dari HOME"),
          ("I", "ve_mps", "=D{r}*SIN(RADIANS(E{r}))", "kecepatan ke timur"),
          ("J", "vn_mps", "=D{r}*COS(RADIANS(E{r}))", "kecepatan ke utara"),
          ("K", "Ax", "=INDEX(Waypoint!$D$2:$D$20,MATCH(F{r}-1,Waypoint!$A$2:$A$20,0))", "x WP awal leg"),
          ("L", "Ay", "=INDEX(Waypoint!$E$2:$E$20,MATCH(F{r}-1,Waypoint!$A$2:$A$20,0))", "y WP awal leg"),
          ("M", "Bx", "=INDEX(Waypoint!$D$2:$D$20,MATCH(F{r},Waypoint!$A$2:$A$20,0))", "x WP tujuan"),
          ("N", "By", "=INDEX(Waypoint!$E$2:$E$20,MATCH(F{r},Waypoint!$A$2:$A$20,0))", "y WP tujuan"),
          ("O", "dx", "=M{r}-K{r}", ""), ("P", "dy", "=N{r}-L{r}", ""),
          ("Q", "L_leg", "=SQRT(O{r}^2+P{r}^2)", "panjang leg"),
          ("R", "e_m", "=(P{r}*(G{r}-K{r})-O{r}*(H{r}-L{r}))/Q{r}", "cross-track error, + kanan"),
          ("S", "abs_e", "=ABS(R{r})", "|e|"),
          ("T", "e_rate", "=(I{r}*P{r}-J{r}*O{r})/Q{r}", "de/dt bertanda"),
          ("U", "de", "=SIGN(R{r})*T{r}", "d|e|/dt, - = mendekat"),
          ("V", "t_leg", "=IF(F{r}=F{p},V{p}+(A{r}-A{p}),0)", "waktu sejak awal leg"),
          ("W", "dalam5", "=IF(S{r}<=BAND,1,0)", "|e| <= 5 m?"),
          ("X", "tahan2s", "=IF(COUNTIFS(F{r}:F{h},F{r},W{r}:W{h},1)=%d,1,0)" % HOLD_N, "dalam 5 m selama 2 s?"),
          ("Y", "pulih", "=IF(F{r}<>F{p},X{r},MAX(Y{p},X{r}))", "sudah pernah pulih di leg ini?"),
          ("Z", "fase", '=IF(Y{r}=1,"tunak","transisi")', ""),
          ("AA", "tanda", "=IF(R{r}<0,-1,1)", "sisi garis"),
          ("AB", "valid6s", "=IF(COUNTIF(F{n1}:F{n2},F{r})=%d,1,0)" % FUT_N, "ada 6 s data sesudahnya?"),
          ("AC", "J_m", "=IF(AB{r}=1,AVERAGE(S{n1}:S{n2}),0)", "|e| rata-rata 6 s ke depan"),
          ("AD", "O_m", "=IF(AB{r}=1,MAXIFS(S{n1}:S{n2},AA{n1}:AA{n2},-AA{r}),0)", "overshoot sisi seberang 6 s ke depan"),
          ("AE", "e_fn", "=MIN(1,S{r}/E_MAX)", "normalisasi |e|"),
          ("AF", "de_fn", "=MAX(-1,MIN(1,U{r}/DE_MAX))", "normalisasi d|e|/dt"),
          ("AG", "mu_Kecil", "=MAX(0,MIN(1,(0.5-AE{r})/0.25))", ""),
          ("AH", "mu_Sedang", "=MAX(0,MIN((AE{r}-0.25)/0.25,(0.75-AE{r})/0.25))", ""),
          ("AI", "mu_Besar", "=MAX(0,MIN(1,(AE{r}-0.5)/0.25))", ""),
          ("AJ", "mu_Negatif", "=MAX(0,MIN(1,(0-AF{r})/0.5))", ""),
          ("AK", "mu_Zero", "=MAX(0,MIN((AF{r}+0.5)/0.5,(0.5-AF{r})/0.5))", ""),
          ("AL", "mu_Positif", "=MAX(0,MIN(1,AF{r}/0.5))", "")]
    ecol, dcol = ("AG", "AH", "AI"), ("AJ", "AK", "AL")
    acols = ["AM", "AN", "AO", "AP", "AQ", "AR", "AS", "AT", "AU"]
    for k, (i, j) in enumerate(RULES):
        c.append((acols[k], "a%d_%s_%s" % (k + 1, E_N[i], D_N[j]), "=MIN(%s{r},%s{r})" % (ecol[i], dcol[j]),
                  "alpha R%d" % (k + 1)))
    c += [("AV", "sum_alpha", "=SUM(AM{r}:AU{r})", ""),
          ("AW", "sum_alpha_c", "=SUMPRODUCT(AM{r}:AU{r},RuleBase!$B$11:$J$11)", ""),
          ("AX", "z", "=AW{r}/AV{r}", "z* ternormalisasi"),
          ("AY", "periode_s", "=P0+S_SKALA*AX{r}", "keluaran Sugeno")]
    return c, acols


def fill(ws, r, col, formula):
    p, h, n1, n2 = r - 1, r + HOLD_N - 1, r + 1, r + FUT_N
    ws["%s%d" % (col, r)] = formula.format(r=r, p=p, h=h, n1=n1, n2=n2)


def build(path, data, wps, key):
    wb = Workbook()
    cols, acols = period_columns()
    nrow = {p: len(d) for p, d in data.items()}

    # --- Petunjuk
    ws = wb.active
    ws.title = "Petunjuk"
    lines = ["Latihan fuzzy Sugeno orde-0 untuk periode L1, data uji L1 konvensional 30/10/2026",
             "Versi: " + ("KUNCI (semua kolom hijau sudah berisi rumus)" if key else "KOSONG (isi kolom hijau sendiri)"),
             "", "Warna sel:", "  kuning = data mentah dari tlog (jangan diubah)",
             "  hijau  = kolom yang dihitung dengan rumus", "  biru   = parameter atau keputusan desain",
             "", "Urutan lembar = urutan tahap pada TUTORIAL_EXCEL_SUGENO.md:",
             "  Parameter  -> konstanta dan keputusan desain (E_MAX, DE_MAX, P0, S, ...)",
             "  Waypoint   -> koordinat WP, ubah ke meter",
             "  P11..P19   -> satu lembar per periode L1: pengolahan data, fuzzifikasi, inferensi, defuzzifikasi",
             "  Gabungan   -> |e|, d|e|/dt, fase dari kelima periode dalam satu tabel",
             "  Statistik  -> jangkar data untuk E_MAX dan DE_MAX",
             "  BuktiRule  -> biaya C per aturan per periode untuk memilih konsekuen",
             "  RuleBase   -> tabel aturan 3x3 dan baris c yang dipakai rumus",
             "  Ringkasan  -> metrik per periode dan sebaran keluaran Sugeno",
             "  TabelMF    -> tabel untuk menggambar grafik fungsi keanggotaan",
             "", "Excel berbahasa Indonesia biasanya memakai ; sebagai pemisah argumen dan koma desimal.",
             "Rumus di file ini tetap berjalan; saat mengetik sendiri pakai pemisah yang dipakai Excel-mu.",
             "Fungsi MAXIFS butuh Excel 2019 / Microsoft 365."]
    for k, s in enumerate(lines, 1):
        ws["A%d" % k] = s
    ws["A1"].font = Font(bold=True, size=13)
    ws.column_dimensions["A"].width = 110

    # --- Parameter (nama sel didefinisikan)
    ws = wb.create_sheet("Parameter")
    prm = [("lat0", "Lintang HOME (derajat)", "=Waypoint!B2", "titik nol koordinat"),
           ("lon0", "Bujur HOME (derajat)", "=Waypoint!C2", ""),
           ("K_M", "Meter per derajat lintang", 111320, "konstanta pendekatan bumi bulat"),
           ("BAND", "Pita pulih |e| (m)", 5, "definisi tunak: |e| <= 5 m bertahan 2 s"),
           ("E_MAX", "E_MAX (m)", "=Statistik!B10", "batas normalisasi |e| (Tahap B)"),
           ("DE_MAX", "DE_MAX (m/s)", "=Statistik!B16", "batas normalisasi d|e|/dt (Tahap B)"),
           ("P0", "Periode dasar P0 (s)", 15, "periode kompromi terbaik uji konvensional"),
           ("S_SKALA", "Faktor skala S (s)", 4, "P0 +- S = 11..19 s = rentang yang diterbangkan"),
           ("MIN_NEFF", "n_eff minimum", 5, "sampel efektif minimum agar periode ikut dipilih")]
    ws.append(["Nama", "Keterangan", "Nilai", "Catatan"])
    for name, desc, val, note in prm:
        if not key and name in ("lat0", "lon0", "E_MAX", "DE_MAX", "P0", "S_SKALA"):
            val = None
        ws.append([name, desc, val, note])
        r = ws.max_row
        ws["C%d" % r].fill = BLUE
        wb.defined_names[name] = DefinedName(name, attr_text="Parameter!$C$%d" % r)
    for c, w in zip("ABCD", (12, 30, 14, 60)):
        ws.column_dimensions[c].width = w
    for c in ws[1]:
        c.font = BOLD

    # --- Waypoint
    ws = wb.create_sheet("Waypoint")
    ws.append(["seq", "lat", "lon", "x_m", "y_m"])
    for i, la, lo in wps:
        r = ws.max_row + 1
        ws.append([i, la, lo, "=(C%d-lon0)*K_M*COS(RADIANS(lat0))" % r if key else None,
                   "=(B%d-lat0)*K_M" % r if key else None])
        for c in "ABC":
            ws["%s%d" % (c, r)].fill = YELLOW
        for c in "DE":
            ws["%s%d" % (c, r)].fill = GREEN
    for c in ws[1]:
        c.font = BOLD
    ws["G2"] = "seq 0 = HOME. x ke timur, y ke utara (meter)."

    # --- lembar per periode
    for p in PERIODS:
        ws = wb.create_sheet("P%d" % p)
        for col, title, f, note in cols:
            ws["%s1" % col] = title
            ws["%s1" % col].font = BOLD
            ws["%s1" % col].fill = YELLOW if f is None else GREEN
            ws["%s1" % col].alignment = Alignment(wrap_text=True, vertical="top")
            ws.column_dimensions[col].width = 11
        for k, s in enumerate(data[p]):
            r = k + 2
            for col, val in zip("ABCDEF", (s["t"], s["lat"], s["lon"], s["gs"], s["cog"], s["wp"])):
                ws["%s%d" % (col, r)] = val
            if key:
                for col, title, f, note in cols:
                    if f is not None:
                        fill(ws, r, col, f)
        ws.freeze_panes = "B2"
        ws.column_dimensions["B"].width = 12
        ws.column_dimensions["C"].width = 12

    # --- Gabungan
    ws = wb.create_sheet("Gabungan")
    ws.append(["periode", "abs_e", "de", "fase", "t_leg", "abs_e_transisi", "abs_de_tunak", "de_transisi_3s", "periode_sugeno"])
    for c in ws[1]:
        c.font = BOLD
        c.fill = GREEN
    r = 2
    for p in PERIODS:
        for k in range(nrow[p]):
            s = k + 2
            ws["A%d" % r] = p
            if key:
                ws["B%d" % r] = "=P%d!S%d" % (p, s)
                ws["C%d" % r] = "=P%d!U%d" % (p, s)
                ws["D%d" % r] = "=P%d!Z%d" % (p, s)
                ws["E%d" % r] = "=P%d!V%d" % (p, s)
                ws["F%d" % r] = '=IF(D{r}="transisi",B{r},"")'.format(r=r)
                ws["G%d" % r] = '=IF(D{r}="tunak",ABS(C{r}),"")'.format(r=r)
                ws["H%d" % r] = '=IF(AND(D{r}="transisi",E{r}>3),C{r},"")'.format(r=r)
                ws["I%d" % r] = "=P%d!AY%d" % (p, s)
            r += 1
    last = r - 1

    # --- Statistik
    ws = wb.create_sheet("Statistik")
    st = [("Jumlah sampel", "=COUNT(Gabungan!B2:B%d)" % last, ""),
          ("Median |e| fase transisi (m)", "=MEDIAN(Gabungan!F2:F%d)" % last, ""),
          ("p95 |d|e|/dt| fase tunak (m/s)", "=PERCENTILE.INC(Gabungan!G2:G%d,0.95)" % last, ""),
          ("p10 d|e|/dt transisi, t_leg > 3 s (m/s)", "=PERCENTILE.INC(Gabungan!H2:H%d,0.1)" % last, ""),
          ("", None, ""),
          ("E_MAX jangkar 1: 4 x pita pulih", "=4*BAND", "Kecil penuh sampai 0,25 E_MAX = 5 m"),
          ("E_MAX jangkar 2: median transisi / 0,75", "=B3/0.75", "Besar penuh mulai 0,75 E_MAX = median"),
          ("E_MAX rata-rata jangkar", "=AVERAGE(B7:B8)", ""),
          ("E_MAX dipakai (dibulatkan)", "=ROUND(B9,0)", ""),
          ("", None, ""),
          ("", None, ""),
          ("DE_MAX jangkar 1: p95 tunak / 0,25", "=B4/0.25", "Zero bernilai 0,5 di +-0,25 DE_MAX"),
          ("DE_MAX jangkar 2: -p10 transisi / 0,5", "=-B5/0.5", "Negatif penuh di -0,5 DE_MAX"),
          ("DE_MAX rata-rata jangkar", "=AVERAGE(B13:B14)", ""),
          ("DE_MAX dipakai (dibulatkan)", "=ROUND(B15,0)", "")]
    ws.append(["Besaran", "Nilai", "Catatan"])
    for d, f, n in st:
        ws.append([d, f if key else None, n])
        if f is not None:
            ws["B%d" % ws.max_row].fill = GREEN
    for c, w in zip("ABC", (42, 14, 44)):
        ws.column_dimensions[c].width = w
    for c in ws[1]:
        c.font = BOLD

    # --- BuktiRule
    ws = wb.create_sheet("BuktiRule")
    ws["A1"] = "Biaya C = J + O (rata-rata berbobot alpha, hanya sampel valid6s). Kecil = lebih baik."
    ws["A1"].font = BOLD
    ws.append([])
    head = ["Aturan"] + ["n_eff %d s" % p for p in PERIODS] + ["C %d s" % p for p in PERIODS] + \
           ["C' %d s (n_eff<min -> 1000000000)" % p for p in PERIODS] + ["Periode terpilih (s)", "c terpilih"]
    ws.append(head)
    for c in ws[3]:
        c.font = BOLD
        c.alignment = Alignment(wrap_text=True)
    for k, (i, j) in enumerate(RULES):
        r = 4 + k
        ws["A%d" % r] = "R%d %s & %s" % (k + 1, E_N[i], D_N[j])
        for col in "QR":
            ws["%s%d" % (col, r)].fill = BLUE
        if not key:
            continue
        ac = acols[k]
        for q, p in enumerate(PERIODS):
            rng = lambda col: "P%d!$%s$2:$%s$%d" % (p, col, col, nrow[p] + 1)
            ncol, ccol, c2col = chr(ord("B") + q), chr(ord("G") + q), chr(ord("L") + q)
            ws["%s%d" % (ncol, r)] = "=SUMPRODUCT(%s,%s)" % (rng(ac), rng("AB"))
            ws["%s%d" % (ccol, r)] = "=IF(%s%d>0,SUMPRODUCT(%s,%s,%s+%s)/%s%d,0)" % (
                ncol, r, rng(ac), rng("AB"), rng("AC"), rng("AD"), ncol, r)
            ws["%s%d" % (c2col, r)] = "=IF(%s%d>=MIN_NEFF,%s%d,1000000000)" % (ncol, r, ccol, r)
        ws["Q%d" % r] = "=CHOOSE(MATCH(MIN(L{r}:P{r}),L{r}:P{r},0),11,13,15,17,19)".format(r=r)
        ws["R%d" % r] = "=(Q{r}-P0)/S_SKALA".format(r=r)
    ws.column_dimensions["A"].width = 22

    # --- RuleBase
    ws = wb.create_sheet("RuleBase")
    ws["A1"] = "Tabel aturan (isi c: -1 = 11 s, -0,5 = 13 s, 0 = 15 s, +0,5 = 17 s, +1 = 19 s)"
    ws["A1"].font = BOLD
    ws["A3"], ws["B3"], ws["C3"], ws["D3"] = "|e| \\ d|e|/dt", "Negatif", "Zero", "Positif"
    for i in range(3):
        ws["A%d" % (4 + i)] = E_N[i]
        for j in range(3):
            cell = "%s%d" % ("BCD"[j], 4 + i)
            ws[cell] = "=BuktiRule!R%d" % (4 + 3 * i + j) if key else None
            ws[cell].fill = BLUE
    ws["F3"] = "periode (s)"
    for i in range(3):
        for j in range(3):
            ws["%s%d" % ("FGH"[j], 4 + i)] = "=P0+S_SKALA*%s%d" % ("BCD"[j], 4 + i) if key else None
    ws["A10"] = "Baris c dalam urutan R1..R9 (dipakai rumus sum_alpha_c di lembar periode):"
    for k, (i, j) in enumerate(RULES):
        col = chr(ord("B") + k)
        ws["%s10" % col] = "R%d" % (k + 1)
        ws["%s11" % col] = "=%s%d" % ("BCD"[j], 4 + i)
    ws["A11"] = "c"
    for c in ("A3", "B3", "C3", "D3", "F3", "A10"):
        ws[c].font = BOLD
    ws.column_dimensions["A"].width = 16

    # --- Ringkasan
    ws = wb.create_sheet("Ringkasan")
    ws.append(["Metrik"] + ["%d s" % p for p in PERIODS])
    met = [("Jumlah sampel", "=COUNT(P{p}!S2:S{n})"),
           ("|e| rata-rata (m)", "=AVERAGE(P{p}!S2:S{n})"),
           ("e RMS (m)", "=SQRT(SUMSQ(P{p}!R2:R{n})/COUNT(P{p}!R2:R{n}))"),
           ("|e| maks (m)", "=MAX(P{p}!S2:S{n})"),
           ("|e| rata-rata fase tunak (m)", '=AVERAGEIF(P{p}!Z2:Z{n},"tunak",P{p}!S2:S{n})'),
           ("% waktu |e| <= 5 m", '=100*COUNTIF(P{p}!S2:S{n},"<=5")/COUNT(P{p}!S2:S{n})'),
           ("% waktu fase transisi", '=100*COUNTIF(P{p}!Z2:Z{n},"transisi")/COUNTA(P{p}!Z2:Z{n})'),
           ("Periode Sugeno rata-rata (s)", "=AVERAGE(P{p}!AY2:AY{n})"),
           ("Periode Sugeno min (s)", "=MIN(P{p}!AY2:AY{n})"),
           ("Periode Sugeno maks (s)", "=MAX(P{p}!AY2:AY{n})")]
    for d, f in met:
        ws.append([d] + ([f.format(p=p, n=nrow[p] + 1) for p in PERIODS] if key else [None] * 5))
    for c in ws[1]:
        c.font = BOLD
    ws.column_dimensions["A"].width = 32

    # --- TabelMF
    ws = wb.create_sheet("TabelMF")
    ws.append(["abs_e (m)", "e_fn", "Kecil", "Sedang", "Besar", "", "de (m/s)", "de_fn", "Negatif", "Zero", "Positif"])
    for k in range(91):
        r = k + 2
        ws["A%d" % r] = k * 0.5
        de = -23 + k * (46 / 90.0)
        ws["G%d" % r] = round(de, 3)
        if key:
            ws["B%d" % r] = "=MIN(1,A%d/E_MAX)" % r
            ws["C%d" % r] = "=MAX(0,MIN(1,(0.5-B{r})/0.25))".format(r=r)
            ws["D%d" % r] = "=MAX(0,MIN((B{r}-0.25)/0.25,(0.75-B{r})/0.25))".format(r=r)
            ws["E%d" % r] = "=MAX(0,MIN(1,(B{r}-0.5)/0.25))".format(r=r)
            ws["H%d" % r] = "=MAX(-1,MIN(1,G%d/DE_MAX))" % r
            ws["I%d" % r] = "=MAX(0,MIN(1,(0-H{r})/0.5))".format(r=r)
            ws["J%d" % r] = "=MAX(0,MIN((H{r}+0.5)/0.5,(0.5-H{r})/0.5))".format(r=r)
            ws["K%d" % r] = "=MAX(0,MIN(1,H{r}/0.5))".format(r=r)
    for c in ws[1]:
        c.font = BOLD
    wb.calculation.fullCalcOnLoad = True    # Excel menghitung semua rumus saat file dibuka
    wb.save(path)


def python_mirror(data, wps):
    """Perhitungan yang sama persis dengan rumus Excel di KUNCI, untuk verifikasi dan ringkasan."""
    lat0, lon0 = wps[0][1], wps[0][2]
    W = {i: ((lo - lon0) * 111320 * np.cos(np.radians(lat0)), (la - lat0) * 111320) for i, la, lo in wps}
    res = {}
    for p, d in data.items():
        t = np.array([s["t"] for s in d])
        wp = np.array([s["wp"] for s in d])
        x = np.array([(s["lon"] - lon0) * 111320 * np.cos(np.radians(lat0)) for s in d])
        y = np.array([(s["lat"] - lat0) * 111320 for s in d])
        ve = np.array([s["gs"] * np.sin(np.radians(s["cog"])) for s in d])
        vn = np.array([s["gs"] * np.cos(np.radians(s["cog"])) for s in d])
        Ax = np.array([W[w - 1][0] for w in wp])
        Ay = np.array([W[w - 1][1] for w in wp])
        dx = np.array([W[w][0] for w in wp]) - Ax
        dy = np.array([W[w][1] for w in wp]) - Ay
        L = np.hypot(dx, dy)
        e = (dy * (x - Ax) - dx * (y - Ay)) / L
        de = np.sign(e) * (ve * dy - vn * dx) / L
        n = len(t)
        inside = (np.abs(e) <= 5).astype(int)
        hold = np.array([int(sum(1 for q in range(k, min(n, k + HOLD_N)) if wp[q] == wp[k] and inside[q]) == HOLD_N)
                         for k in range(n)])
        pul = np.zeros(n, int)
        tleg = np.zeros(n)
        for k in range(n):
            new_leg = k == 0 or wp[k] != wp[k - 1]
            pul[k] = hold[k] if new_leg else max(pul[k - 1], hold[k])
            tleg[k] = 0.0 if new_leg else tleg[k - 1] + t[k] - t[k - 1]
        tanda = np.where(e < 0, -1, 1)
        valid = np.array([int(sum(1 for q in range(k + 1, min(n, k + 1 + FUT_N)) if wp[q] == wp[k]) == FUT_N)
                          for k in range(n)])
        J = np.array([np.abs(e[k + 1:k + 1 + FUT_N]).mean() if valid[k] else 0.0 for k in range(n)])
        O = np.array([max([abs(e[q]) for q in range(k + 1, k + 1 + FUT_N) if tanda[q] == -tanda[k]] or [0.0])
                      if valid[k] else 0.0 for k in range(n)])
        res[p] = dict(e=e, de=de, fase=np.where(pul == 1, "tunak", "transisi"), tleg=tleg, valid=valid, J=J, O=O)
    ae = np.concatenate([np.abs(res[p]["e"]) for p in PERIODS])
    de = np.concatenate([res[p]["de"] for p in PERIODS])
    fase = np.concatenate([res[p]["fase"] for p in PERIODS])
    tleg = np.concatenate([res[p]["tleg"] for p in PERIODS])
    st = dict(med_tr=float(np.median(ae[fase == "transisi"])),
              p95_tun=float(np.percentile(np.abs(de[fase == "tunak"]), 95)),
              p10_tr3=float(np.percentile(de[(fase == "transisi") & (tleg > 3)], 10)))
    st["E_MAX"] = float(round((4 * 5 + st["med_tr"] / 0.75) / 2))
    st["DE_MAX"] = float(round((st["p95_tun"] / 0.25 - st["p10_tr3"] / 0.5) / 2))
    clip = lambda v, lo, hi: np.minimum(hi, np.maximum(lo, v))
    for p in PERIODS:
        r = res[p]
        efn = np.minimum(1, np.abs(r["e"]) / st["E_MAX"])
        dfn = clip(r["de"] / st["DE_MAX"], -1, 1)
        me = [clip((0.5 - efn) / 0.25, 0, 1), np.maximum(0, np.minimum((efn - .25) / .25, (.75 - efn) / .25)), clip((efn - .5) / .25, 0, 1)]
        md = [clip(-dfn / 0.5, 0, 1), np.maximum(0, np.minimum((dfn + .5) / .5, (.5 - dfn) / .5)), clip(dfn / .5, 0, 1)]
        r["alpha"] = np.array([np.minimum(me[i], md[j]) for i, j in RULES])
    ev, rules = {}, []
    for k in range(9):
        best = None
        for p in PERIODS:
            r = res[p]
            n_eff = float((r["alpha"][k] * r["valid"]).sum())
            C = float((r["alpha"][k] * r["valid"] * (r["J"] + r["O"])).sum() / n_eff) if n_eff > 0 else 0.0
            ev[(k, p)] = (n_eff, C)
            Cp = C if n_eff >= 5 else 1e9
            if best is None or Cp < best[0]:
                best = (Cp, p)
        rules.append((best[1] - 15) / 4.0)
    for p in PERIODS:
        r = res[p]
        c = np.array(rules)[:, None]
        r["periode"] = 15 + 4 * (r["alpha"] * c).sum(0) / r["alpha"].sum(0)
    return res, st, ev, rules


def check(path_key, mir, data):
    """Hitung workbook dengan pustaka 'formulas' lalu bandingkan dengan python_mirror."""
    import formulas
    sol = formulas.ExcelModel().loads(path_key).finish().calculate()
    val = {}
    for k, v in sol.items():
        val[k.upper()] = v.value[0][0] if hasattr(v, "value") else v
    base = "'[%s]" % os.path.basename(path_key).upper()

    def cell(sheet, ref):
        return val.get("%s%s'!%s" % (base, sheet.upper(), ref))

    res, st, ev, rules = mir
    worst = 0.0
    for p in PERIODS:
        for k in range(len(data[p])):
            r = k + 2
            for col, key in (("R", "e"), ("U", "de"), ("AC", "J"), ("AD", "O"), ("AY", "periode")):
                x = cell("P%d" % p, "%s%d" % (col, r))
                worst = max(worst, abs(float(x) - float(res[p][key][k])))
            if cell("P%d" % p, "Z%d" % r) != res[p]["fase"][k]:
                print("fase beda", p, r)
    print("Excel vs Python: selisih maks %.2e" % worst)
    print("Excel E_MAX %s DE_MAX %s | Python %s %s" % (cell("Statistik", "B10"), cell("Statistik", "B16"), st["E_MAX"], st["DE_MAX"]))
    print("Excel c:", [cell("BuktiRule", "R%d" % (4 + k)) for k in range(9)], "| Python c:", rules)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", help='"periode|folder_hasil_analyze|path.tlog"')
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--cek", type=int, default=0,
                    help="verifikasi rumus: hitung workbook uji berisi N sampel pertama per periode dengan pustaka formulas")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    data, wps = {}, None
    for spec in args.runs:
        p, folder, tlog = spec.split("|")
        data[int(p)], w = extract(folder, tlog)
        wps = wps or w
        print("periode %s: %d sampel" % (p, len(data[int(p)])))
    build(os.path.join(args.out, "data_fuzzy_sugeno_KUNCI.xlsx"), data, wps, key=True)
    build(os.path.join(args.out, "data_fuzzy_sugeno_KOSONG.xlsx"), data, wps, key=False)

    res, st, ev, rules = python_mirror(data, wps)
    for p in PERIODS:
        ref = np.array([s["cte_ref"] for s in data[p]])
        print("periode %d: e vs cte analyze_tlog_l1_auto: selisih maks %.2f m; periode Sugeno %.2f..%.2f s, rata-rata %.2f s"
              % (p, np.max(np.abs(res[p]["e"] - ref)), res[p]["periode"].min(), res[p]["periode"].max(), res[p]["periode"].mean()))
    print("statistik:", {k: round(v, 3) for k, v in st.items()})
    print("c R1..R9:", rules)
    for k in range(9):
        print("  R%d" % (k + 1), " ".join("%g s: n=%.1f C=%.2f" % (p, *ev[(k, p)]) for p in PERIODS))

    if args.cek:
        import tempfile
        sub = {p: d[:args.cek] for p, d in data.items()}
        tmp = os.path.join(tempfile.mkdtemp(), "cek.xlsx")
        build(tmp, sub, wps, key=True)
        check(tmp, python_mirror(sub, wps), sub)
    print("selesai ->", args.out)


if __name__ == "__main__":
    main()
