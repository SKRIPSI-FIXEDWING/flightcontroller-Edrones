#!/bin/sh
# Buat ulang folder ini dari tools/flight_analysis. Log SD asli di ~/Downloads (tidak di-commit).
# 14 Nov 12-22-15 tidak punya log SD: dikonversi dulu dari tlog (10 Hz) dengan venv berisi pymavlink:
#   .venv/bin/python tlog_to_sdlog.py "$HOME/Downloads/2026-11-14 12-22-15.tlog" -o /tmp/12-22-15_dari_tlog.CSV
# Jendela t_boot sama dengan ringkasan.md tiap terbang (flight_2026-11-14, flight_2026-11-16).
# 14-29-40 (arus naik) ikut dihitung tetapi bukan data utama (lihat flight_2026-11-14/ringkasan_hari.md).
D="$HOME/Downloads"
T1222="${T1222:-/tmp/12-22-15_dari_tlog.CSV}"
# T4c (fuzzy tanpa gangguan) tidak dipakai: data tambahan, pembanding tanpa gangguan sudah ada dari uji sebelumnya.
# R80 14-07-25: log SD belum disalin, pakai tlog yang dipotong lalu dikonversi (waktu = detik sejak awal potongan):
#   .venv/bin/python flight_2026-11-16/skrip/potong_tlog.py "$D/2026-11-16 14-04-15.tlog" /tmp/F7.tlog 189 1042
#   .venv/bin/python tlog_to_sdlog.py /tmp/F7.tlog -o /tmp/F7.CSV
F7="${F7:-/tmp/F7.CSV}"
# R80 hanya dipakai bila angin <= 4 m/s (batasan). Yang memenuhi hanya 14-07-25 segmen b (3,2 m/s).
# Dibuang: T5 11-17-00 (6,2 m/s), 14-07-25 a/c-f (4,5-6,3 m/s), 14-28-43 a-c (6,7-7,2 m/s, ketinggian tidak terkendali).
python3 analyze_kejadian_toleransi.py \
  "N14_12-22-15|$T1222|100" \
  "N14_14-21-31|$D/2026-11-14_14-19-44.CSV|100" \
  "N14_14-29-40|$D/2026-11-14_14-30-17.CSV|100" \
  "N14_15-03-50|$D/2026-11-14_14-48-03.CSV|100" \
  "T2|$D/2026-11-16_10-40-05.CSV|100|311|502" \
  "T3|$D/2026-11-16_10-50-34.CSV|100|49|532.3" \
  "T4|$D/2026-11-16_11-02-12.CSV|100|101|594|ab" \
  "N16_14-07-25|$F7|80|||b" \
  -o kejadian_toleransi_2026-11
