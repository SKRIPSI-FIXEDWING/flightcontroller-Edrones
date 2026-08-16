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
