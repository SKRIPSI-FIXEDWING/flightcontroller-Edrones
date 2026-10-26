#include "communication/SdLogger.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FC_Config.h"
#include "task.h"

namespace fc {

namespace {

// Upper bound on what one service() call writes, so a large backlog after
// an MTP pause is drained over a few cycles instead of one long burst.
constexpr size_t kMaxBytesPerService = 4096;

// Statically in RAM2 (DMAMEM), NOT from the heap: this FreeRTOS port's
// malloc (and every xTaskCreate() stack) comes out of RAM1, which is nearly
// full. A 32 KB xStreamBufferCreate() there ran it out on 2026-10-25 --
// vApplicationMallocFailedHook, LED 13 blinking 2x forever, nothing started.
DMAMEM uint8_t g_stream_storage[kSdBufferBytes + 1];
StaticStreamBuffer_t g_stream_struct;

}  // namespace

bool SdLogger::init()
{
    if (stream_ == nullptr) {
        stream_ = xStreamBufferCreateStatic(kSdBufferBytes, 1, g_stream_storage, &g_stream_struct);
    }
    if (sd_mutex_ == nullptr) {
        sd_mutex_ = xSemaphoreCreateMutex();
    }
    return stream_ != nullptr && sd_mutex_ != nullptr;
}

bool SdLogger::writeLine(const char* text, size_t len)
{
    if (!open_ || stream_ == nullptr || len == 0) {
        rows_dropped_ = rows_dropped_ + 1;
        return false;
    }
    // All-or-nothing, zero timeout: a full buffer drops this line rather
    // than making the sampler wait for the card.
    if (xStreamBufferSpacesAvailable(stream_) < len) {
        rows_dropped_ = rows_dropped_ + 1;
        return false;
    }
    xStreamBufferSend(stream_, text, len, 0);
    return true;
}

size_t SdLogger::bufferFree() const
{
    return (open_ && stream_ != nullptr) ? xStreamBufferSpacesAvailable(stream_) : 0;
}

void SdLogger::requestFlush()
{
    flush_requested_ = true;
}

void SdLogger::setArmed(bool armed)
{
    armed_ = armed;
}

void SdLogger::announce(const char* message)
{
    putMailbox(message);
}

void SdLogger::service()
{
    if (stream_ == nullptr) {
        return;
    }
    const uint32_t now_ms = millis();

    // --- Bring-up: mount + open, throttled to one attempt per retry interval.
    if (!open_) {
        if (mount_attempted_ && now_ms - last_mount_attempt_ms_ < kSdRetryIntervalMs) {
            return;
        }
        mount_attempted_ = true;
        last_mount_attempt_ms_ = now_ms;
        if (!lockSd(0)) {
            return;  // MTP is using the card; next attempt after the retry interval
        }
        if (!mounted_) {
            mount();
        }
        if (mounted_) {
            openNextFile();
        }
        unlockSd();
        return;
    }

    // --- Normal operation: drain the RAM buffer into the file.
    if (!lockSd(0)) {
        return;  // MTP busy (disarmed only) -- rows wait in the buffer
    }
    size_t total = 0;
    while (total < kMaxBytesPerService) {
        const size_t n = xStreamBufferReceive(stream_, chunk_, sizeof(chunk_), 0);
        if (n == 0) {
            break;
        }
        if (file_.write(chunk_, n) != n) {
            closeAfterError("write");
            unlockSd();
            return;
        }
        total += n;
    }

    // Flush = commit data + file size to the FAT, so a power cut loses at
    // most the last ~kSdFlushIntervalMs. Also right after DISARM, so the
    // whole flight is on the card by the time the battery is unplugged.
    const bool armed = armed_;
    const bool just_disarmed = last_armed_seen_ && !armed;
    last_armed_seen_ = armed;
    const bool drained = xStreamBufferIsEmpty(stream_) == pdTRUE;
    if (just_disarmed || (flush_requested_ && drained) || now_ms - last_flush_ms_ >= kSdFlushIntervalMs) {
        file_.flush();
        last_flush_ms_ = now_ms;
        if (drained) {
            flush_requested_ = false;
        }
    }
    unlockSd();

    const uint32_t dropped = rows_dropped_;
    if (dropped != dropped_reported_ && now_ms - last_drop_report_ms_ >= 10000) {
        dropped_reported_ = dropped;
        last_drop_report_ms_ = now_ms;
        postStatus("[SD] %lu rows dropped (card busy)", static_cast<unsigned long>(dropped));
    }
}

bool SdLogger::mount()
{
    if (!SD.begin(BUILTIN_SDCARD)) {
        mounted_ = false;
        postStatus("[SD] no card/mount fail err=0x%02X", static_cast<unsigned>(SD.sdfs.sdErrorCode()));
        return false;
    }
    mounted_ = true;
    return true;
}

bool SdLogger::openNextFile()
{
    // Next unused LOGnnn.CSV, so every boot (and every remount after a card
    // error) gets its own file instead of appending to an older flight.
    uint16_t index = 1;
    for (; index <= 999; ++index) {
        snprintf(file_name_, sizeof(file_name_), "LOG%03u.CSV", index);
        if (!SD.exists(file_name_)) {
            break;
        }
    }
    if (index > 999) {
        postStatus("[SD] 999 logs on card, delete old ones");
        return false;
    }

    file_ = SD.open(file_name_, FILE_WRITE);
    if (!file_) {
        // Most often a card SD.begin() accepted but can't write (read-only
        // lock switch, corrupt FAT): force a full remount next attempt.
        mounted_ = false;
        postStatus("[SD] cannot create %s", file_name_);
        return false;
    }
    // The file header (comment lines + column names) is written by the
    // producer (fc::FlightLog) once it sees the new generation_, so it can
    // include params/mission that only it can read.
    //
    // Producer isn't sending while open_ is false, and it outranks this task
    // so it can't be mid-send either -- safe to clear any leftover bytes of a
    // file that died mid-row.
    xStreamBufferReset(stream_);
    last_flush_ms_ = millis();
    flush_requested_ = false;
    generation_ = generation_ + 1;
    open_ = true;

    const uint32_t size_mb = static_cast<uint32_t>(SD.totalSize() / (1024ULL * 1024ULL));
    postStatus("[SD] logging %s %luMB %s", file_name_, static_cast<unsigned long>(size_mb),
               SD.sdfs.fatType() == FAT_TYPE_EXFAT ? "exFAT" : "FAT32");
    return true;
}

void SdLogger::closeAfterError(const char* what)
{
    open_ = false;
    mounted_ = false;  // a pulled/failed card needs a full SD.begin() again
    file_.close();
    mount_attempted_ = true;
    last_mount_attempt_ms_ = millis();
    postStatus("[SD] %s error %s, remounting", what, file_name_);
}

bool SdLogger::lockSd(uint32_t timeout_ms)
{
    return sd_mutex_ != nullptr && xSemaphoreTake(sd_mutex_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void SdLogger::unlockSd()
{
    xSemaphoreGive(sd_mutex_);
}

bool SdLogger::isMounted() const
{
    return mounted_;
}

bool SdLogger::isOpen() const
{
    return open_;
}

uint32_t SdLogger::fileGeneration() const
{
    return generation_;
}

const char* SdLogger::fileName() const
{
    return file_name_;
}

uint32_t SdLogger::rowsDropped() const
{
    return rows_dropped_;
}

void SdLogger::postStatus(const char* fmt, ...)
{
    char msg[sizeof(status_msg_)];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    // A missing card is retried every few seconds -- only report changes.
    if (strcmp(msg, last_posted_) == 0) {
        return;
    }
    memcpy(last_posted_, msg, sizeof(last_posted_));
    putMailbox(msg);
}

void SdLogger::putMailbox(const char* msg)
{
    taskENTER_CRITICAL();
    strncpy(status_msg_, msg, sizeof(status_msg_) - 1);
    status_msg_[sizeof(status_msg_) - 1] = '\0';
    status_pending_ = true;
    taskEXIT_CRITICAL();
}

bool SdLogger::takeStatusMessage(char* out, size_t out_len)
{
    if (!status_pending_ || out_len == 0) {
        return false;
    }
    taskENTER_CRITICAL();
    strncpy(out, status_msg_, out_len - 1);
    out[out_len - 1] = '\0';
    status_pending_ = false;
    taskEXIT_CRITICAL();
    return true;
}

}  // namespace fc
