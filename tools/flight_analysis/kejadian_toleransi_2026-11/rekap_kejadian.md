# Kejadian keluar toleransi ±10 m, loiter fase stabil

Dibuat oleh `analyze_kejadian_toleransi.py` (bank 45°, kejadian berjarak < 1.0 s digabung). overshoot = keluar orbit (radius melebar), undershoot = masuk ke dalam orbit.

## Per penerbangan

| Segmen | Metode | Stabil | Putaran | Angin | Pulsa | Kejadian | Overshoot | Undershoot | Puncak luar | Puncak dalam | Terlama | Terkait pulsa | Roll jenuh (puncak / selama) | Searah angin |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| N14_12-22-15 | L1 tetap | 45 s | 1.4 | 6.9 m/s dari 208° | 5 | 1 | 1 | 0 | 13.0 | – | 2.2 s | 1 | 0 / 1 | 1 |
| N14_14-21-31 | L1 tetap | 58 s | 1.8 | 7.5 m/s dari 191° | 5 | 4 | 2 | 2 | 30.5 | -11.7 | 6.0 s | 3 | 1 / 1 | 1 |
| N14_14-29-40 | L1 tetap | 104 s | 2.7 | 6.0 m/s dari 227° | 11 | 8 | 7 | 1 | 26.1 | -16.5 | 8.2 s | 6 | 0 / 3 | 1 |
| N14_15-03-50 | L1 tetap | 45 s | 1.4 | 3.6 m/s dari 197° | 6 | 3 | 1 | 2 | 26.9 | -14.1 | 5.8 s | 2 | 1 / 1 | 1 |
| T2 | L1 tetap | 55 s | 1.4 | 3.3 m/s dari 176° | 5 | 3 | 3 | 0 | 21.3 | – | 5.4 s | 3 | 0 / 2 | 2 |
| T3a | L1 + fuzzy | 78 s | 1.8 | 5.1 m/s dari 200° | 6 | 5 | 3 | 2 | 20.3 | -12.4 | 4.2 s | 3 | 0 / 3 | 2 |
| T3b | L1 + fuzzy | 52 s | 1.3 | 2.5 m/s dari 196° | 5 | 2 | 2 | 0 | 12.0 | – | 1.5 s | 2 | 0 / 0 | 0 |
| T3c | L1 + fuzzy | 63 s | 1.5 | 2.4 m/s dari 223° | 8 | 4 | 4 | 0 | 12.8 | – | 2.0 s | 4 | 0 / 0 | 0 |
| T4a | L1 + fuzzy | 88 s | 2.5 | 4.3 m/s dari 193° | 9 | 3 | 3 | 0 | 18.3 | – | 3.4 s | 3 | 0 / 2 | 1 |
| T4b | L1 + fuzzy | 56 s | 1.4 | 3.7 m/s dari 177° | 6 | 2 | 2 | 0 | 17.5 | – | 3.2 s | 2 | 0 / 2 | 1 |
| N16_14-07-25b | L1 tetap | 53 s | 1.6 | 3.2 m/s dari 191° | 6 | 4 | 4 | 0 | 16.9 | – | 3.4 s | 4 | 0 / 1 | 2 |

## Per kejadian

| Segmen | No | Jenis | Puncak | Durasi | Sisi orbit | Arah terbang | GS | nav_roll | Jenuh | Period (dT) | Pulsa | Catatan |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| N14_12-22-15 | 1 | overshoot | +13.0 m | 2.2 s | BL (329°) | searah angin | 27.5 m/s | +43.8° | tidak (45%) | 14.0 s (+0.0) | P3 (-300 µs, puncak +4.5 s) |  |
| N14_14-21-31 | 1 | undershoot | -10.1 m | 0.8 s | TL (53°) | lawan angin | 17.1 m/s | +6.8° | tidak (0%) | 14.0 s (+0.0) | – |  |
| N14_14-21-31 | 2 | overshoot | +30.5 m | 6.0 s | BL (333°) | searah angin | 24.6 m/s | +45.0° | ya (60%) | 14.0 s (+0.0) | P3 (-300 µs, puncak +5.0 s) |  |
| N14_14-21-31 | 3 | undershoot | -11.7 m | 1.3 s | TL (35°) | menyamping | 20.5 m/s | +13.6° | tidak (0%) | 14.0 s (+0.0) | P4 (-300 µs, puncak +0.8 s) |  |
| N14_14-21-31 | 4 | overshoot | +12.0 m | 2.2 s | BD (212°) | menyamping | 23.2 m/s | +36.8° | tidak (0%) | 14.0 s (+0.0) | P5 (-300 µs, puncak +4.1 s) |  |
| N14_14-29-40 | 1 | undershoot | -16.5 m | 6.0 s | TL (30°) | menyamping | 23.0 m/s | +17.2° | tidak (0%) | 14.0 s (+0.0) | – |  |
| N14_14-29-40 | 2 | overshoot | +20.4 m | 6.6 s | Tg (127°) | lawan angin | 12.9 m/s | +30.4° | tidak (0%) | 14.0 s (+0.0) | P1 (-300 µs, puncak +4.1 s) |  |
| N14_14-29-40 | 3 | overshoot | +26.1 m | 8.2 s | S (199°) | menyamping | 18.4 m/s | +40.5° | tidak (13%) | 14.0 s (+0.0) | P2 (-300 µs, puncak +4.1 s) | pulsa diberikan saat sudah di luar toleransi |
| N14_14-29-40 | 4 | overshoot | +17.8 m | 3.8 s | B (272°) | searah angin | 25.5 m/s | +43.6° | tidak (27%) | 14.0 s (+0.0) | P3 (-300 µs, puncak +4.9 s) |  |
| N14_14-29-40 | 5 | overshoot | +15.0 m | 3.2 s | Tg (120°) | lawan angin | 13.0 m/s | +25.0° | tidak (0%) | 14.0 s (+0.0) | P5 (-300 µs, puncak +3.8 s) |  |
| N14_14-29-40 | 6 | overshoot | +21.1 m | 5.4 s | S (196°) | lawan angin | 17.8 m/s | +35.6° | tidak (2%) | 14.0 s (+0.0) | P6 (-300 µs, puncak +4.1 s) |  |
| N14_14-29-40 | 7 | overshoot | +13.9 m | 3.2 s | T (97°) | lawan angin | 13.0 m/s | +25.6° | tidak (0%) | 14.0 s (+0.0) | P9 (-300 µs, puncak +3.8 s) |  |
| N14_14-29-40 | 8 | overshoot | +17.2 m | 5.1 s | S (185°) | lawan angin | 17.2 m/s | +32.8° | tidak (0%) | 14.0 s (+0.0) | P10 (-300 µs, puncak +3.8 s) | masih di luar saat fase stabil berakhir |
| N14_15-03-50 | 1 | undershoot | -12.0 m | 1.8 s | TL (53°) | lawan angin | 20.2 m/s | +12.6° | tidak (0%) | 14.0 s (+0.0) | P1 (-300 µs, puncak +0.9 s) |  |
| N14_15-03-50 | 2 | overshoot | +26.9 m | 5.8 s | BL (316°) | searah angin | 23.5 m/s | +45.0° | ya (56%) | 14.0 s (+0.0) | P4 (-300 µs, puncak +4.9 s) |  |
| N14_15-03-50 | 3 | undershoot | -14.1 m | 3.6 s | T (86°) | lawan angin | 17.8 m/s | +5.0° | tidak (0%) | 14.0 s (+0.0) | P6 (-300 µs, puncak +0.7 s) | pulsa diberikan saat sudah di luar toleransi |
| T2 | 1 | overshoot | +21.3 m | 5.4 s | BL (304°) | searah angin | 19.0 m/s | +37.3° | tidak (26%) | 14.0 s (+0.0) | P1 (-300 µs, puncak +4.3 s) |  |
| T2 | 2 | overshoot | +15.1 m | 2.8 s | U (1°) | menyamping | 16.5 m/s | +31.4° | tidak (0%) | 14.0 s (+0.0) | P2 (-300 µs, puncak +3.7 s) |  |
| T2 | 3 | overshoot | +17.0 m | 4.0 s | B (266°) | searah angin | 21.3 m/s | +37.9° | tidak (20%) | 14.0 s (+0.0) | P5 (-300 µs, puncak +4.0 s) |  |
| T3a | 1 | undershoot | -12.4 m | 2.0 s | B (270°) | searah angin | 21.3 m/s | +14.8° | tidak (0%) | 13.6 s (-0.4) | P1 (-300 µs, puncak +0.6 s) | pulsa diberikan saat sudah di luar toleransi |
| T3a | 2 | overshoot | +15.7 m | 3.0 s | TL (25°) | menyamping | 16.6 m/s | +31.9° | tidak (28%) | 13.3 s (-0.7) | P2 (-300 µs, puncak +3.9 s) |  |
| T3a | 3 | overshoot | +20.3 m | 4.2 s | T (108°) | lawan angin | 13.1 m/s | +33.4° | tidak (4%) | 13.0 s (-1.0) | P3 (-300 µs, puncak +3.7 s) |  |
| T3a | 4 | undershoot | -11.3 m | 4.2 s | BL (304°) | searah angin | 20.4 m/s | +12.8° | tidak (0%) | 13.7 s (-0.3) | P5 (-300 µs, puncak +0.2 s) | pulsa diberikan saat sudah di luar toleransi |
| T3a | 5 | overshoot | +19.3 m | 4.0 s | TL (48°) | menyamping | 14.4 m/s | +32.8° | tidak (35%) | 13.1 s (-0.9) | P6 (-300 µs, puncak +3.8 s) |  |
| T3b | 1 | overshoot | +12.0 m | 1.5 s | T (112°) | lawan angin | 14.7 m/s | +23.0° | tidak (0%) | 13.7 s (-0.3) | P3 (-300 µs, puncak +3.7 s) |  |
| T3b | 2 | overshoot | +11.1 m | 1.2 s | S (196°) | menyamping | 16.2 m/s | +31.3° | tidak (0%) | 12.0 s (-2.0) | P4 (-300 µs, puncak +3.7 s) |  |
| T3c | 1 | overshoot | +12.7 m | 1.6 s | B (248°) | menyamping | 17.1 m/s | +35.0° | tidak (0%) | 12.0 s (-2.0) | P1 (-300 µs, puncak +4.0 s) |  |
| T3c | 2 | overshoot | +11.5 m | 1.2 s | T (78°) | lawan angin | 14.4 m/s | +23.0° | tidak (0%) | 13.7 s (-0.3) | P4 (-300 µs, puncak +3.9 s) |  |
| T3c | 3 | overshoot | +12.8 m | 2.0 s | Tg (150°) | lawan angin | 14.1 m/s | +25.5° | tidak (0%) | 13.6 s (-0.4) | P5 (-300 µs, puncak +3.8 s) |  |
| T3c | 4 | overshoot | +10.4 m | 0.8 s | BD (234°) | menyamping | 15.7 m/s | +27.0° | tidak (0%) | 13.0 s (-1.0) | P6 (-300 µs, puncak +3.9 s) |  |
| T4a | 1 | overshoot | +18.3 m | 3.4 s | BL (318°) | searah angin | 22.3 m/s | +42.9° | tidak (40%) | 13.1 s (-0.9) | P4 (-300 µs, puncak +4.4 s) |  |
| T4a | 2 | overshoot | +10.2 m | 0.4 s | TL (29°) | menyamping | 17.3 m/s | +27.5° | tidak (0%) | 13.8 s (-0.2) | P5 (-300 µs, puncak +3.6 s) |  |
| T4a | 3 | overshoot | +13.2 m | 1.0 s | U (1°) | menyamping | 19.7 m/s | +35.6° | tidak (50%) | 13.6 s (-0.5) | P9 (-300 µs, puncak +3.4 s) | masih di luar saat fase stabil berakhir |
| T4b | 1 | overshoot | +16.8 m | 2.6 s | BL (324°) | searah angin | 19.6 m/s | +40.7° | tidak (37%) | 12.0 s (-2.0) | P1 (-300 µs, puncak +3.6 s) |  |
| T4b | 2 | overshoot | +17.5 m | 3.2 s | U (340°) | menyamping | 19.4 m/s | +35.9° | tidak (32%) | 13.2 s (-0.8) | P6 (-300 µs, puncak +3.9 s) |  |
| N16_14-07-25b | 1 | overshoot | +16.9 m | 3.2 s | BL (323°) | searah angin | 16.3 m/s | +33.8° | tidak (0%) | 14.0 s (+0.0) | P1 (-300 µs, puncak +4.3 s) |  |
| N16_14-07-25b | 2 | overshoot | +12.3 m | 1.6 s | TL (54°) | lawan angin | 13.5 m/s | +26.2° | tidak (0%) | 14.0 s (+0.0) | P2 (-300 µs, puncak +4.0 s) |  |
| N16_14-07-25b | 3 | overshoot | +15.8 m | 3.4 s | BL (330°) | searah angin | 22.4 m/s | +40.6° | tidak (26%) | 14.0 s (+0.0) | P5 (-300 µs, puncak +4.4 s) |  |
| N16_14-07-25b | 4 | overshoot | +14.7 m | 1.5 s | TL (36°) | menyamping | 17.7 m/s | +32.0° | tidak (0%) | 14.0 s (+0.0) | P6 (-300 µs, puncak +3.5 s) | masih di luar saat fase stabil berakhir |
