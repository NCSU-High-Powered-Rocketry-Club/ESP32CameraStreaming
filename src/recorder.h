// recorder.h - microSD MJPEG (.avi) recorder for the XIAO ESP32-S3 Sense.
//
// Merged from the standalone "camera" AVI-recorder sketch: the SD mount, the
// AVI (Motion-JPEG) container writer and the per-clip performance report now
// live here, and the ESP-NOW sender hands the SAME captured JPEG to both the SD
// card and the air link. The arbitration between those two consumers is the
// RECORD_PRIORITY_* policy implemented in stream_espnow.cpp.
//
// Used by the ESP-NOW ("dish") target ONLY - see ENABLE_SD_RECORDING in
// config.h. With that flag at 0 nothing here is ever called.
#pragma once

#include <stddef.h>
#include <stdint.h>

// --- card / clip lifecycle -------------------------------------------------

// Mount the microSD card (idempotent, safe to call from setup()). Returns true
// when a card is mounted and usable.
bool recorderBegin();

// True once recorderBegin() mounted a card.
bool recorderSdReady();

// A clip is being written right now.
bool recorderActive();

// Start a clip. Called from the capture loop, which passes the frame's REAL
// dimensions (the AVI header needs them) plus the nominal container frame rate.
// Returns false (and logs) when there is no card / a clip is already running.
bool recorderStart(uint16_t width, uint16_t height, uint16_t fps);

// Append one JPEG frame to the clip in progress. `len` may include sensor
// padding and/or concatenated JPEGs: the frame is trimmed at its EOI marker.
bool recorderWriteFrame(const uint8_t* jpeg, size_t len);

// Count a captured frame that the priority policy kept away from the card.
void recorderNoteSkippedFrame();

// Finalise the clip (patch the AVI header, append idx1) and print the report.
bool recorderStop();

// Auto-stop once the configured maximum duration has elapsed (call every pass).
void recorderTick();

// Length of one JPEG: up to and including the first EOI (FF D9). Shared with
// the streaming path so both consumers emit the same self-contained image
// (the OV3660 can hand back several concatenated JPEGs in one buffer).
size_t recorderJpegLength(const uint8_t* buf, size_t len);

// --- runtime settings (dashboard / serial) ---------------------------------

void        recorderSetMaxSeconds(uint32_t sec);   // 0 = record until stopped
uint32_t    recorderMaxSeconds();
void        recorderSetPriority(uint8_t priority); // RECORD_PRIORITY_* in config.h
uint8_t     recorderPriority();
const char* recorderPriorityName();                // "SD", "balanced", "STREAM"

// --- status ----------------------------------------------------------------

uint16_t    recorderFileIndex();     // clip number in /videoNNNNN.avi (0 = idle)
const char* recorderFileName();      // "/video00003.avi", "" when idle
uint32_t    recorderFrameCount();    // frames written into the clip in progress
uint32_t    recorderSecondsLeft();   // seconds until auto-stop (0 = unlimited/idle)

// One machine-readable line for the serial console and for the ESP-NOW status
// datagram the dashboard displays, e.g.
//   state=on idx=3 frames=142 left=17s prio=1 sd=1
void recorderStatusLine(char* out, size_t n);
