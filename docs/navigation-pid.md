# Navigasi PID pembanding L1 (2026-11-07)

Kode: `include/navigation/NavPid.h`, `src/navigation/NavPid.cpp`, dipanggil dari
`L1Controller::updateWaypoint()` dan `updateLoiter()`. Tes host:
`test/host/test_nav_pid.cpp`.

**Default MATI** (`NAV_PID_ENABLE` = 0). Selama mati, tidak ada satu baris pun
keluaran L1 yang berubah.

## Hukum kendali

    a_lat = Kp e + Kd de/dt + Ki int(e)      (m/s^2, + = belok kanan)

- `e` = `crosstrackError()` (garis) atau galat radius (lingkaran), `de/dt` =
  `crosstrackVelocityMps()`. Tanda sama dengan kolom `cte_m` / `xtrack_vel_mps` log SD.
- Gain default **setara L1 pada periode 14 s, damping 0.707** (linearisasi
  error kecil, Park et al. 2004): Kp = w^2, Kd = 2 zeta w, Ki = Kp / 20. Sama dengan
  `tools/flight_analysis/sim_guidance_compare.py`.
- Integrator dijepit +-1.729 m/s^2 (setara bank 10 deg). Di-reset saat pindah
  garis <-> lingkaran, ganti leg / pusat loiter, PID dihidupkan/dimatikan,
  `NAV_PID_KI` diubah, atau jeda update > 1 s. Integrasi hanya saat |e| < 40 m
  (garis, lihat batas sudut potong) atau |e| < 20 m (lingkaran).
- `nav_roll` tetap dihitung dari `a_lat` lewat `navRollCd()` dan dijepit oleh
  `ROLL_LIMIT_DEG` / `LOIT_BANK_DEG` yang sama dengan L1.

## Di mana PID dipakai, di mana tetap L1

| Situasi | Hukum |
|---|---|
| Leg dengan garis, cabang ikuti-garis | **PID** |
| Leg langsung ke WP (tanpa garis, leg pertama) | L1 |
| Cabang tangkap titik A (di belakang WP awal) / titik B (lewat WP akhir) | L1 |
| Loiter, di atas lingkaran | **PID** menggantikan suku PD lingkaran L1; feedforward sentripetal tetap |
| Loiter, menangkap lingkaran dari luar | L1 (cabang capture) |

Alasannya: masuk dan keluar leg/loiter sama untuk semua metode, jadi yang
dibandingkan hanya pelacakan garis/lingkaran dan respons gangguan. PID juga
tidak menerima galat ratusan meter.

**Batas sudut potong 45 deg di garis.** Suku P dijepit di
`|e| <= Kd * GS * sin 45 / Kp` (sekitar 40 m pada GS 18 m/s), setara batas
`sin(nu1) <= 0.7071` di L1. Tanpa batas ini, galat awal 100 m membuat PID
memotong garis 82 deg (hampir tegak lurus); dengan batas 43 deg. Gangguan
biasa (e < 30 m) tetap di daerah linear.

## Catatan untuk skripsi

- **Di loiter, PD lingkaran L1 sudah berbentuk PID tanpa I**:
  `xtrack_err * w^2 + xtrack_vel * 2 zeta w`. Jadi PID di loiter hanya berbeda
  dari L1 tetap lewat suku integral, dan gainnya tidak ikut periode L1.
  Di garis lurus bedanya nyata: L1 nonlinear (asin, sin nu, L1 ~ GS), PID linear
  dengan gain tetap.
- Fuzzy mengubah periode L1, yang tidak dipakai PID kecuali di cabang capture.
  Untuk terbang PID, set `FUZZY_ENABLE` = 0.

## Cara menghidupkan

1. `FUZZY_ENABLE` = 0.
2. `NAV_PID_ENABLE` = 1 (Mission Planner menampilkan "NAV: PID ON").
3. Mematikan: `NAV_PID_ENABLE` = 0 ("NAV: L1 (PID OFF)").

Status yang dipakai tercatat di header log SD (`# EFFECTIVE nav_pid_enable=...`)
dan setiap perubahan param tercatat sebagai event.
