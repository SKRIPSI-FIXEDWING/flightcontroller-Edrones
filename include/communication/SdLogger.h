#pragma once

#include <Arduino.h>
#include <SD.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"

namespace fc {

/** Calendar date/time of a Unix time (seconds since 1970-01-01, proleptic Gregorian). */
struct CivilTime {
    int year;
    unsigned month, day, hour, minute, second;
};
CivilTime civilFromUnixSeconds(int64_t unix_s);

/**
 * Non-blocking SD card text-log pipe for the Teensy 4.1's built-in microSD
 * slot (BUILTIN_SDCARD), pulled after the flight over MTP (the card shows up
 * in the PC's file manager over the same USB cable, see
 * docs/sd-logger-mtp.md). WHAT is written (CSV columns, #EVENT rows, ARM
 * snapshots) is decided by fc::FlightLog; this class only gets bytes safely
 * onto the card.
 *
 * Split into a producer and a consumer so the card can never stall the rest
 * of the firmware:
 *
 * - writeLine() (producer, ONE task only -- taskSdLog): copies a text line
 *   into a RAM stream buffer with zero timeout. Never touches the card,
 *   never waits. If the buffer is full the line is dropped and counted.
 * - service() (consumer, taskSdWriter, lowest priority): the ONLY code that
 *   touches the card. Mounts it (retrying while it's missing), opens the
 *   next LOGnnn.CSV, drains the buffer into it, flushes ~1 Hz, right after
 *   DISARM and whenever requestFlush() was called (important events). SD
 *   writes can stall for tens to hundreds of ms (card wear-levelling) and
 *   Teensy's FIFO SDIO driver busy-waits during that -- harmless here, every
 *   flight-critical task has a higher priority.
 *
 * Other SD users (MTP) must hold lockSd() around their access: SdFat shares
 * one sector cache and is not thread-safe, and MTP asking for the free
 * space walks the whole FAT (seconds on a 32 GB FAT32 card) -- run
 * concurrently with a write, that corrupts the logger's file state.
 *
 * A new file is opened (fileGeneration() increments) on boot and after every
 * card error/remount; the producer must write its file header again each
 * time the generation changes.
 *
 * Files are opened as LOGnnn.CSV because the date is not known at boot.
 * Once the producer hands over a GPS UTC time (setUtcReference()), the
 * writer renames the open file once to the local date/time it was opened,
 * e.g. 2026-10-29_08-58-36.CSV (kSdLogUtcOffsetMinutes). Without GPS time
 * the file keeps its LOGnnn.CSV name.
 */
class SdLogger final {
public:
    /** Creates the RAM buffer and the SD mutex. No card access -- call once
     * from setup(), before any task that calls the other methods exists. */
    bool init();

    // ---------------- producer side (taskSdLog only) ----------------

    /**
     * Queues one text line (caller includes the "\r\n"). All-or-nothing and
     * non-blocking: false (and rowsDropped() + 1) if no file is open or the
     * buffer lacks room for the whole line.
     */
    bool writeLine(const char* text, size_t len);

    /** Free bytes in the RAM buffer, for callers that want to write a
     * multi-line block only if all of it fits. */
    size_t bufferFree() const;

    /** Ask the writer to flush the card as soon as the buffer is drained. */
    void requestFlush();

    /** Latest arm state, so the writer can flush right after DISARM. */
    void setArmed(bool armed);

    /** UTC time [us since 1970] at millis() = at_ms, from the GPS. Lets the
     * writer give the open file its date name; call whenever one is known. */
    void setUtcReference(uint64_t utc_us, uint32_t at_ms);

    /** Mailbox message for the GCS (e.g. file name on ARM), bypasses the
     * de-duplication postStatus() applies to repeated mount errors. */
    void announce(const char* message);

    // ---------------- consumer side (taskSdWriter only) ----------------

    /** Call every kSdWriterPeriodMs from exactly one low-priority task. */
    void service();

    // ---------------- shared ----------------

    /** Exclusive card access for other SD users (MTP). Returns false if the
     * writer holds it longer than timeout_ms. */
    bool lockSd(uint32_t timeout_ms);
    void unlockSd();

    /** Card mounted (SD.begin() succeeded and not failed since). */
    bool isMounted() const;
    /** Log file open and accepting lines. */
    bool isOpen() const;
    /** Increments every time a new log file is opened. */
    uint32_t fileGeneration() const;
    /** Name of the current/last log file, e.g. "LOG007.CSV" or, after the
     * date rename, "2026-10-29_08-58-36.CSV". */
    const char* fileName() const;
    /** The current file already has its date name. */
    bool isDated() const;
    /** Lines lost since boot (buffer full / no file open). */
    uint32_t rowsDropped() const;

    /**
     * Pops the latest pending status line (mount result, write error,
     * dropped rows, ARM file name) for main.cpp's reportStatus(). Kept as a
     * mailbox instead of calling reportStatus() from the writer task because
     * fc::Mavlink and the USB Serial port are only safe to use from
     * taskMavlink.
     */
    bool takeStatusMessage(char* out, size_t out_len);

private:
    bool mount();
    bool openNextFile();
    void renameToDate();
    void closeAfterError(const char* what);
    void postStatus(const char* fmt, ...);
    void putMailbox(const char* msg);

    StreamBufferHandle_t stream_ = nullptr;
    SemaphoreHandle_t sd_mutex_ = nullptr;

    File file_{};
    char file_name_[32] = {};
    volatile bool mounted_ = false;
    volatile bool dated_ = false;
    volatile bool open_ = false;
    volatile uint32_t generation_ = 0;

    // Written by the producer, read by the writer.
    volatile bool armed_ = false;
    volatile bool flush_requested_ = false;
    volatile uint32_t rows_dropped_ = 0;
    uint64_t utc_ref_us_ = 0;  // 0 = no GPS time yet; read/written in a critical section
    uint32_t utc_ref_ms_ = 0;

    // Writer-only state.
    bool last_armed_seen_ = false;
    uint32_t last_flush_ms_ = 0;
    uint32_t file_open_ms_ = 0;
    bool rename_tried_ = false;  // one date-rename attempt per file
    uint32_t last_mount_attempt_ms_ = 0;
    bool mount_attempted_ = false;
    uint32_t dropped_reported_ = 0;
    uint32_t last_drop_report_ms_ = 0;
    uint8_t chunk_[512] = {};

    // MAVLink STATUSTEXT carries 50 chars, so messages are kept under that.
    char status_msg_[64] = {};
    char last_posted_[64] = {};  // writer-only, de-duplicates retry spam
    volatile bool status_pending_ = false;
};

}  // namespace fc
