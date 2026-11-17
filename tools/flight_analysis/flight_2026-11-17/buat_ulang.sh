#!/bin/sh
# Buat ulang seluruh analisis 17 Nov 2026 (R80). Jalankan dari tools/flight_analysis.
# Log asli di ~/Downloads (tidak di-commit). Venv .venv berisi pymavlink (untuk potong tlog dan analisis tlog).
set -e
D="$HOME/Downloads"
P=.venv/bin/python
K=flight_2026-11-16/skrip
O=flight_2026-11-17
TMP="${TMP:-/tmp/n17}"
mkdir -p "$TMP"
L1="$D/2026-11-17_08-51-16.CSV"   # terbang 09-03-03, L1 tetap
FZ="$D/2026-11-17_09-17-42.CSV"   # terbang 09-20-26, L1 + fuzzy
# 2026-11-17_08-43-41.CSV = sesi darat tanpa ARM (FUZZY_ENABLE 1 -> 0), tidak dipakai.

# Potong per terbang (tlog: detik sejak awal tlog; log SD: t_boot), membuang ARM 3 s di darat.
$P $K/potong_tlog.py "$D/2026-11-17 08-48-08.tlog" "$TMP/L1.tlog" 893 1512
$P $K/potong_tlog.py "$D/2026-11-17 09-18-02.tlog" "$TMP/FZ.tlog" 141 920
$P $K/potong_sdlog.py "$L1" "$TMP/L1.CSV" 718 1336
$P $K/potong_sdlog.py "$FZ" "$TMP/FZ.CSV" 174 951

for x in L1:terbang_09-03-03 FZ:terbang_09-20-26; do
  L=${x%%:*}; F=$O/${x#*:}
  $P analyze_tlog_l1_auto.py "$TMP/$L.tlog" -o "$F"
  $P analyze_tlog_loiter.py "$TMP/$L.tlog" -o "$F/loiter" --bagian loiter
  $P analyze_tlog_loiter.py "$TMP/$L.tlog" -o "$F/full" --bagian full
  $P analyze_fuzzy_sdlog.py "$TMP/$L.CSV" -o "$F/fuzzy"
  $P plot_loiter_steady_sdlog.py "$TMP/$L.CSV" -o "$F/loiter" --radius 80
done

# Gangguan + kecepatan per masuk AUTO (jendela t_boot dipisah di tengah STAB 1-2 s).
g() { $P $K/potong_sdlog.py "$TMP/$1.CSV" "$TMP/$1_a$2.CSV" $3 $4
      $P analyze_sdlog_gangguan_kecepatan.py "$TMP/$1_a$2.CSV" -o "$O/$5/auto$2" --radius 80 --bank 45; }
g L1 1 718 867.1 terbang_09-03-03;   g L1 2 867.1 978.1 terbang_09-03-03
g L1 3 978.1 1187.8 terbang_09-03-03; g L1 4 1187.8 1336 terbang_09-03-03
g FZ 1 174 354.7 terbang_09-20-26;   g FZ 2 354.7 462.9 terbang_09-20-26
g FZ 3 462.9 616.8 terbang_09-20-26; g FZ 4 616.8 758.3 terbang_09-20-26
g FZ 5 758.3 951 terbang_09-20-26

# Kejadian keluar toleransi, bagian BERGANGGUAN. Segmen yang punya putaran bebas gangguan di awal fase stabil
# (L1 c, fuzzy a dan c) dipotong mulai pulsa pertama di fase stabil. Putaran bebasnya masuk sebagai BEBAS_*.
python3 analyze_kejadian_toleransi.py \
  "N17_09-03-03|$L1|80|718|1336|a" "N17_09-03-03|$L1|80|718|1336|b" \
  "N17_09-03-03|$L1|80|718|1336|c|1140.5-" "N17_09-03-03|$L1|80|718|1336|d" \
  "N17_09-20-26|$FZ|80|174|951|a|296.4-" "N17_09-20-26|$FZ|80|174|951|b" \
  "N17_09-20-26|$FZ|80|174|951|c|543.4-" "N17_09-20-26|$FZ|80|174|951|d" "N17_09-20-26|$FZ|80|174|951|e" \
  "BEBAS_09-03-03|$L1|80|718|1336|c|1046.2-1080.4" "BEBAS_09-20-26|$FZ|80|174|951|a|252.8-278.9" \
  -o $O/kejadian_toleransi_R80

# Pembanding tanpa gangguan: satu putaran fase stabil penuh sebelum pulsa pertama.
# Batas putaran (t_boot) dari loiter_turn_index seluruh loiter segmen itu.
python3 $O/skrip/banding_putaran_bebas.py -o $O/pembanding_tanpa_gangguan_R80 --radius 80 \
  "L1-AUTO3-put1|$L1|1046.2|1080.4|utama" "FZ-AUTO1-put1|$FZ|252.8|278.9|utama" \
  "L1-AUTO3-put2|$L1|1080.4|1113.6|tambahan" "FZ-AUTO3-put1|$FZ|519.5|543.4|tambahan"

# Respons bertanda per pulsa (L1 vs fuzzy, uji Mann-Whitney).
python3 $K/pulsa_bertanda.py "L1|$L1|80" "FZ|$FZ|80" > $O/pulsa_bertanda.txt
R=80 python3 $K/ringkas_loiter_stabil.py "$L1" "$FZ" > $O/loiter_stabil_sd.csv
