# Refactor Battery (voltage sensor)

Modul baru berdasarkan implementasi ADC nyata yang sudah ada di program lama
tapi tidak pernah tersambung ke mana pun. Nama file lama **`Voltage.h`
sebenarnya hanya deklarasi kosong** -- implementasi sesungguhnya
(`init_voltage()`, `read_voltage()`, `update_voltage()`, `display_batt()`)
ada di `controlinfo.h` (nama file yang tidak sesuai isinya). `Mavlink.h` lama
**tidak memanggil salah satu dari fungsi ini** -- ia mengirim nilai baterai
sintetis (gelombang sinus berosilasi 10.5-12.6V mengikuti waktu, ditandai
jelas sebagai placeholder di komentar aslinya).

## Struktur modul

```text
include/vehicle/Battery.h   Kontrak class Battery
src/vehicle/Battery.cpp     Implementasi (moving average ADC, identik logika lama)
```

## Kepemilikan dan penjadwalan

```cpp
#include "vehicle/Battery.h"

fc::Battery battery;

void setupBattery() { battery.begin(); }

void updateBattery() {
    battery.update();  // rate-limited internal ke 50ms, aman dipanggil tiap loop
}
```

## Pemetaan API lama

| Program lama (`controlinfo.h`) | Program baru |
| --- | --- |
| `init_voltage()` | `battery.begin()` |
| `update_voltage()` / `read_voltage()` | `battery.update()` |
| `batt_v` | `battery.data().voltage_v` |
| `perc_batt` / `display_batt()` | `battery.data().percent` |

## File lama yang tidak dimigrasikan

```text
include/Voltage.h     (deklarasi kosong/marker saja)
include/controlinfo.h (implementasi sesungguhnya ada di sini, tapi sebagian besar file ini
                        adalah sketsa LQR+Fuzzy yang di-comment total -- lihat docs/attitude-lqr.md)
```

## Update: sensor 0-25V di pin 26 (A12), baterai 4S 16.8 V

Logika dari sketsa `voltage_sensor_teensy41.ino` sekarang dipakai di `Battery`:

- ADC 12-bit (`analogReadResolution(12)`, `analogReadAveraging(4)`), pin `INPUT_DISABLE`.
- 32 sampel per pembacaan, filter EMA `alpha = 0.20`, dibaca tiap 50 ms.
- `V = (adc / 4095) * 3.3 * DIVIDER_RATIO(5.0) * CAL_FACTOR(0.9956)` -- ubah di `BatteryConfig`.
- Persen dari kurva LiPo per sel (21 titik), status `Ok / Low (<=3.60 V/sel) /
  Critical (<=3.40 V/sel) / AdcSaturated`.
- Catatan: dengan rasio 5:1 ADC saturasi di 16.5 V, jadi baterai penuh 16.8 V
  terbaca terpotong ~16.4 V sampai tegangannya turun.

Ke Mission Planner (1 Hz):

- `SYS_STATUS` (`voltage_battery`, `battery_remaining`) -> teks HUD "Bat1 xx.xxv ... %".
- `BATTERY_STATUS` -> tab Status, `charge_state` LOW/CRITICAL sesuai status.
- `STATUSTEXT` "BATTERY LOW/CRITICAL (xx.xx V)" sekali saat status berubah.
