# Refactor Buzzer

Port langsung dari `buzzer.h` lama, menyisakan hanya method yang benar-benar
dipanggil dari `buzzerTask`: lagu startup (Mario Bros theme) lalu pola beep
3x-jeda saat disarmed, senyap saat armed.

## Method lama yang tidak dimigrasikan (dead code)

```text
beep()              blocking delay loop, dipanggil dari loop() lama yang tidak dipakai
soundBuzzer()/stop() pakai analogWrite() bukan tone(), tidak konsisten, tidak dipanggil
playHappyBirthday()  dideklarasikan tapi TIDAK PERNAH diimplementasikan di source lama
playNote()           dideklarasikan, tidak pernah dipanggil
```

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `buzzer.initWithMusic()` + `buzzer.init()` | `buzzer.begin()` |
| `buzzer.update(armed)` | `buzzer.update(armed)` (sama) |
| `buzzer.isPlayingInitMusic` | Internal, tidak perlu dibaca dari luar |

## File lama yang tidak dimigrasikan

```text
include/buzzer.h
```
