#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

namespace fc {

/**
 * Satu mutex yang melindungi state yang ditulis oleh task Mavlink DAN dibaca
 * task Control: daftar waypoint (waypoint[], wp_sum, flag_wp, next/prev WP),
 * dan pergantian mode (ModeManager::setMode vs ModeManager::update).
 *
 * Aturan pakai:
 *  - taskControl menahan lock selama applyRcModeSwitch() + ModeManager::update()
 *    (satu siklus kendali, ~ms).
 *  - Handler MAVLink menahan lock HANYA saat menulis state itu / mengganti
 *    mode, lalu melepasnya (jangan menahan lock saat menulis EEPROM atau
 *    mengirim paket).
 *  - Bila lock tidak didapat dalam timeout, aksi dibatalkan dan GCS diberi
 *    jawaban gagal -- bukan dibiarkan menulis tanpa lock.
 */
inline SemaphoreHandle_t& controlMutexHandle()
{
    static SemaphoreHandle_t handle = nullptr;
    return handle;
}

/** Panggil sekali di setup() SEBELUM task dibuat. */
inline void initControlMutex()
{
    if (controlMutexHandle() == nullptr) {
        controlMutexHandle() = xSemaphoreCreateMutex();
    }
}

class ControlLock final {
public:
    explicit ControlLock(TickType_t timeout_ticks = pdMS_TO_TICKS(30))
    {
        SemaphoreHandle_t handle = controlMutexHandle();
        if (handle == nullptr) {
            acquired_ = true;  // mutex belum dibuat (mis. sebelum setup selesai): tanpa proteksi
            return;
        }
        acquired_ = (xSemaphoreTake(handle, timeout_ticks) == pdTRUE);
        owns_ = acquired_;
    }

    ~ControlLock()
    {
        if (owns_) {
            xSemaphoreGive(controlMutexHandle());
        }
    }

    ControlLock(const ControlLock&) = delete;
    ControlLock& operator=(const ControlLock&) = delete;

    bool acquired() const { return acquired_; }

private:
    bool acquired_ = false;
    bool owns_ = false;
};

}  // namespace fc
