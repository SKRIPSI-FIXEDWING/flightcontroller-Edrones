# Kejadian keluar toleransi ±10 m, loiter fase stabil

Dibuat oleh `analyze_kejadian_toleransi.py` (bank 45°, kejadian berjarak < 1.0 s digabung). overshoot = keluar orbit (radius melebar), undershoot = masuk ke dalam orbit.

## Per penerbangan

| Segmen | Metode | Stabil | Putaran | Angin | Pulsa | Kejadian | Overshoot | Undershoot | Puncak luar | Puncak dalam | Terlama | Terkait pulsa | Roll jenuh (puncak / selama) | Searah angin |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| N17_09-03-03a | L1 tetap | 73 s | 2.3 | 1.4 m/s dari 133° | 7 | 7 | 7 | 0 | 16.3 | – | 3.4 s | 7 | 0 / 3 | 3 |
| N17_09-03-03b | L1 tetap | 47 s | 1.3 | 0.4 m/s dari 88° | 4 | 5 | 5 | 0 | 18.4 | – | 3.4 s | 5 | 0 / 1 | 2 |
| N17_09-03-03c | L1 tetap | 47 s | 1.1 | 2.3 m/s dari 61° | 5 | 5 | 5 | 0 | 19.4 | – | 3.8 s | 5 | 0 / 0 | 2 |
| N17_09-03-03d | L1 tetap | 52 s | 1.4 | 1.7 m/s dari 134° | 6 | 7 | 7 | 0 | 16.4 | – | 3.2 s | 7 | 0 / 1 | 3 |
| N17_09-20-26a | L1 + fuzzy | 58 s | 2.0 | 1.7 m/s dari 254° | 5 | 3 | 3 | 0 | 24.1 | – | 5.8 s | 3 | 0 / 3 | 0 |
| N17_09-20-26b | L1 + fuzzy | 51 s | 1.8 | 1.3 m/s dari 277° | 7 | 3 | 3 | 0 | 26.7 | – | 6.2 s | 3 | 2 / 3 | 0 |
| N17_09-20-26c | L1 + fuzzy | 73 s | 2.5 | 1.2 m/s dari 249° | 8 | 5 | 5 | 0 | 24.2 | – | 6.0 s | 5 | 1 / 5 | 2 |
| N17_09-20-26d | L1 + fuzzy | 84 s | 2.6 | 0.9 m/s dari 279° | 7 | 2 | 2 | 0 | 24.2 | – | 5.4 s | 2 | 0 / 2 | 0 |
| N17_09-20-26e | L1 + fuzzy | 82 s | 2.5 | 0.1 m/s dari 148° | 7 | 5 | 4 | 1 | 30.7 | -11.4 | 7.2 s | 4 | 1 / 4 | 2 |
| BEBAS_09-03-03c | L1 tetap | 34 s | 1.0 | 0.8 m/s dari 77° | 0 | **tidak ada** | 0 | 0 | – | – | – | 0 | 0 / 0 | 0 |
| BEBAS_09-20-26a | L1 + fuzzy | 26 s | 1.0 | 2.0 m/s dari 232° | 0 | **tidak ada** | 0 | 0 | – | – | – | 0 | 0 / 0 | 0 |

## Per kejadian

| Segmen | No | Jenis | Puncak | Durasi | Sisi orbit | Arah terbang | GS | nav_roll | Jenuh | Period (dT) | Pulsa | Catatan |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| N17_09-03-03a | 1 | overshoot | +12.4 m | 1.8 s | BL (330°) | menyamping | 18.9 m/s | +32.9° | tidak (3%) | 14.0 s (+0.0) | P1 (-300 µs, puncak +4.0 s) |  |
| N17_09-03-03a | 2 | overshoot | +14.9 m | 2.8 s | T (88°) | lawan angin | 18.3 m/s | +34.2° | tidak (0%) | 14.0 s (+0.0) | P2 (-300 µs, puncak +3.9 s) |  |
| N17_09-03-03a | 3 | overshoot | +12.9 m | 1.9 s | S (187°) | searah angin | 20.4 m/s | +36.3° | tidak (0%) | 14.0 s (+0.0) | P3 (-300 µs, puncak +3.8 s) |  |
| N17_09-03-03a | 4 | overshoot | +13.0 m | 1.8 s | B (278°) | searah angin | 19.6 m/s | +37.0° | tidak (13%) | 14.0 s (+0.0) | P4 (-300 µs, puncak +3.7 s) |  |
| N17_09-03-03a | 5 | overshoot | +12.9 m | 1.8 s | U (19°) | lawan angin | 16.1 m/s | +31.0° | tidak (0%) | 14.0 s (+0.0) | P5 (-300 µs, puncak +3.8 s) |  |
| N17_09-03-03a | 6 | overshoot | +16.3 m | 3.4 s | Tg (127°) | menyamping | 17.8 m/s | +34.8° | tidak (11%) | 14.0 s (+0.0) | P6 (-300 µs, puncak +3.9 s) |  |
| N17_09-03-03a | 7 | overshoot | +14.1 m | 2.4 s | BD (246°) | searah angin | 18.0 m/s | +35.0° | tidak (0%) | 14.0 s (+0.0) | P7 (-300 µs, puncak +3.8 s) |  |
| N17_09-03-03b | 1 | overshoot | +14.9 m | 2.8 s | BD (217°) | searah angin | 16.1 m/s | +29.8° | tidak (0%) | 14.0 s (+0.0) | P0 (-300 µs, puncak +4.0 s) |  |
| N17_09-03-03b | 2 | overshoot | +15.6 m | 2.8 s | BL (296°) | menyamping | 16.4 m/s | +31.6° | tidak (0%) | 14.0 s (+0.0) | P1 (-300 µs, puncak +4.1 s) |  |
| N17_09-03-03b | 3 | overshoot | +14.0 m | 2.0 s | U (18°) | lawan angin | 15.3 m/s | +28.7° | tidak (0%) | 14.0 s (+0.0) | P2 (-300 µs, puncak +3.8 s) |  |
| N17_09-03-03b | 4 | overshoot | +18.4 m | 3.4 s | Tg (128°) | searah angin | 17.1 m/s | +35.1° | tidak (11%) | 14.0 s (+0.0) | P3 (-300 µs, puncak +3.8 s) |  |
| N17_09-03-03b | 5 | overshoot | +14.1 m | 2.4 s | B (266°) | menyamping | 15.8 m/s | +29.9° | tidak (0%) | 14.0 s (+0.0) | P4 (-300 µs, puncak +4.0 s) |  |
| N17_09-03-03c | 1 | overshoot | +12.6 m | 1.8 s | S (165°) | searah angin | 16.4 m/s | +31.6° | tidak (0%) | 14.0 s (+0.0) | P0 (-300 µs, puncak +3.7 s) |  |
| N17_09-03-03c | 2 | overshoot | +14.6 m | 2.4 s | BD (224°) | menyamping | 15.2 m/s | +31.8° | tidak (0%) | 14.0 s (+0.0) | P1 (-300 µs, puncak +3.9 s) |  |
| N17_09-03-03c | 3 | overshoot | +19.4 m | 3.8 s | BL (299°) | lawan angin | 14.0 m/s | +31.5° | tidak (0%) | 14.0 s (+0.0) | P2 (-300 µs, puncak +4.4 s) |  |
| N17_09-03-03c | 4 | overshoot | +18.3 m | 3.6 s | U (18°) | lawan angin | 13.6 m/s | +30.8° | tidak (0%) | 14.0 s (+0.0) | P3 (-300 µs, puncak +4.3 s) |  |
| N17_09-03-03c | 5 | overshoot | +15.3 m | 2.8 s | Tg (115°) | searah angin | 16.5 m/s | +31.3° | tidak (0%) | 14.0 s (+0.0) | P4 (-300 µs, puncak +4.0 s) |  |
| N17_09-03-03d | 1 | overshoot | +15.0 m | 2.4 s | BD (242°) | searah angin | 19.4 m/s | +35.6° | tidak (25%) | 14.0 s (+0.0) | P1 (-300 µs, puncak +3.8 s) |  |
| N17_09-03-03d | 2 | overshoot | +16.4 m | 3.2 s | BL (334°) | menyamping | 16.0 m/s | +30.9° | tidak (0%) | 14.0 s (+0.0) | P2 (-300 µs, puncak +4.0 s) |  |
| N17_09-03-03d | 3 | overshoot | +13.4 m | 2.2 s | TL (62°) | lawan angin | 14.8 m/s | +29.4° | tidak (0%) | 14.0 s (+0.0) | P3 (-300 µs, puncak +3.6 s) |  |
| N17_09-03-03d | 4 | overshoot | +11.2 m | 1.4 s | Tg (125°) | menyamping | 15.0 m/s | +26.2° | tidak (0%) | 14.0 s (+0.0) | P4 (-300 µs, puncak +4.0 s) |  |
| N17_09-03-03d | 5 | overshoot | +11.1 m | 1.4 s | BD (211°) | searah angin | 16.8 m/s | +29.0° | tidak (0%) | 14.0 s (+0.0) | P4 (-300 µs, puncak +12.6 s) |  |
| N17_09-03-03d | 6 | overshoot | +13.6 m | 2.4 s | B (269°) | searah angin | 16.8 m/s | +32.1° | tidak (0%) | 14.0 s (+0.0) | P5 (-300 µs, puncak +4.0 s) |  |
| N17_09-03-03d | 7 | overshoot | +13.1 m | 2.2 s | BL (327°) | menyamping | 14.9 m/s | +26.5° | tidak (0%) | 14.0 s (+0.0) | P6 (-300 µs, puncak +4.2 s) |  |
| N17_09-20-26a | 1 | overshoot | +13.5 m | 2.2 s | Tg (126°) | lawan angin | 18.3 m/s | +34.3° | tidak (17%) | 13.5 s (-0.5) | P0 (-300 µs, puncak +4.0 s) |  |
| N17_09-20-26a | 2 | overshoot | +22.5 m | 5.2 s | BD (222°) | lawan angin | 17.9 m/s | +42.4° | tidak (38%) | 12.8 s (-1.2) | P1 (-300 µs, puncak +3.7 s) |  |
| N17_09-20-26a | 3 | overshoot | +24.1 m | 5.8 s | BD (221°) | lawan angin | 19.0 m/s | +44.1° | tidak (48%) | 12.7 s (-1.3) | P4 (-300 µs, puncak +4.3 s) |  |
| N17_09-20-26b | 1 | overshoot | +26.7 m | 6.2 s | BD (246°) | lawan angin | 19.8 m/s | +45.0° | ya (57%) | 12.4 s (-1.6) | P1 (-300 µs, puncak +4.8 s) |  |
| N17_09-20-26b | 2 | overshoot | +24.4 m | 6.2 s | S (194°) | lawan angin | 18.7 m/s | +45.0° | ya (35%) | 12.7 s (-1.4) | P4 (-300 µs, puncak +4.4 s) |  |
| N17_09-20-26b | 3 | overshoot | +17.2 m | 3.2 s | B (286°) | menyamping | 18.9 m/s | +39.2° | tidak (43%) | 13.2 s (-0.8) | P5 (-300 µs, puncak +3.8 s) |  |
| N17_09-20-26c | 1 | overshoot | +24.2 m | 6.0 s | BD (214°) | lawan angin | 19.2 m/s | +44.7° | ya (46%) | 12.7 s (-1.3) | P1 (-300 µs, puncak +4.6 s) |  |
| N17_09-20-26c | 2 | overshoot | +14.8 m | 2.4 s | BL (306°) | searah angin | 18.8 m/s | +36.7° | tidak (41%) | 13.4 s (-0.6) | P2 (-300 µs, puncak +3.5 s) |  |
| N17_09-20-26c | 3 | overshoot | +21.0 m | 5.2 s | S (191°) | lawan angin | 18.9 m/s | +42.7° | tidak (30%) | 12.9 s (-1.1) | P4 (-300 µs, puncak +4.3 s) |  |
| N17_09-20-26c | 4 | overshoot | +17.9 m | 3.8 s | S (177°) | lawan angin | 18.3 m/s | +39.9° | tidak (32%) | 13.2 s (-0.8) | P7 (-300 µs, puncak +4.1 s) |  |
| N17_09-20-26c | 5 | overshoot | +16.1 m | 2.4 s | BL (314°) | searah angin | 19.5 m/s | +38.5° | tidak (49%) | 13.3 s (-0.7) | P8 (-300 µs, puncak +3.6 s) | masih di luar saat fase stabil berakhir |
| N17_09-20-26d | 1 | overshoot | +24.2 m | 5.4 s | BD (245°) | lawan angin | 17.6 m/s | +44.0° | tidak (40%) | 12.7 s (-1.3) | P3 (-300 µs, puncak +4.1 s) |  |
| N17_09-20-26d | 2 | overshoot | +23.2 m | 5.2 s | BD (216°) | lawan angin | 16.6 m/s | +40.5° | tidak (40%) | 12.7 s (-1.3) | P6 (-300 µs, puncak +4.2 s) |  |
| N17_09-20-26e | 1 | overshoot | +23.5 m | 5.8 s | B (254°) | searah angin | 15.9 m/s | +41.3° | tidak (30%) | 12.7 s (-1.3) | P1 (-300 µs, puncak +3.9 s) |  |
| N17_09-20-26e | 2 | undershoot | -11.4 m | 2.8 s | TL (34°) | lawan angin | 16.0 m/s | +7.0° | tidak (0%) | 13.7 s (-0.3) | P2 (-300 µs, puncak +0.2 s) | pulsa diberikan saat sudah di luar toleransi |
| N17_09-20-26e | 3 | overshoot | +18.0 m | 4.6 s | S (170°) | menyamping | 18.3 m/s | +38.9° | tidak (34%) | 13.2 s (-0.8) | P3 (-300 µs, puncak +4.6 s) |  |
| N17_09-20-26e | 4 | overshoot | +30.7 m | 7.2 s | BD (222°) | searah angin | 18.7 m/s | +45.0° | ya (61%) | 12.0 s (-2.0) | P6 (-300 µs, puncak +5.1 s) |  |
| N17_09-20-26e | 5 | overshoot | +12.8 m | 1.8 s | BL (314°) | menyamping | 17.0 m/s | +33.9° | tidak (23%) | 13.6 s (-0.4) | P7 (-300 µs, puncak +3.8 s) |  |
