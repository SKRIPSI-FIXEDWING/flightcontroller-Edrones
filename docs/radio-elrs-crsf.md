# ELRS (CRSF) RC input

Dipilih lewat `FC_RC_PROTOCOL_CRSF` di `include/FC_Config.h` (1 = ELRS/CRSF di Serial3, 0 = SBUS di Serial8).

## Wiring (Teensy 4.1)

| ELRS RX | Teensy 4.1 |
|---|---|
| TX | pin 15 (RX3) |
| RX | pin 14 (TX3) |
| 5 V / GND | 5 V / GND |

CRSF: 420000 baud, 8N1, non-inverted (tidak perlu inverter seperti SBUS).

## Perilaku

- Frame `RC_CHANNELS_PACKED` (0x16) di-unpack ke skala 172..1811 (sama dengan SBUS),
  sehingga scaling, arm (CH5), mode (CH6) di `Radio` tidak berubah.
- `LINK_STATISTICS` (0x14) -> `Radio::linkQuality()`, `rssiDbm()`, `snrDb()`.
- Signal lost: tidak ada frame RC > `signal_timeout_ms` (500 ms), atau LQ = 0.

## Setting ELRS yang dipakai

Mode Normal, 50 mW, 333Hz Full 2.4G (8ch full-res), Telem 1:8, Antenna Gemini.
Rekomendasi: lihat jawaban di sesi / catatan skripsi (daya 100-250 mW untuk terbang,
telemetri ELRS tidak dipakai untuk MAVLink karena sudah ada Holybro 915 MHz di Serial2).
